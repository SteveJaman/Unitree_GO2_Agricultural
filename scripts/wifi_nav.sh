#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# wifi_nav.sh
#
# Live waypoint navigation + autonomous frontier exploration over Wi-Fi
# using the go2-nav2-wifi stack.
#
# Architecture:
#   Robot (Jetson) : Point-LIO onboard. Raw LiDAR never leaves the robot.
#                    Bidirectional relay: sensors up, /cmd_vel down.
#   VM (this host) : Docker container runs RViz + Nav2 + explore_lite.
#
# What this wrapper does:
#   - Pre-flight cleanup of stale processes (local, container, remote)
#   - Removal of incomplete run directories on the Jetson
#   - Prerequisite checks (Docker, repo, SSH, explore_lite, panel)
#   - Optional autonomous frontier exploration (explore_lite + Go2 Explore panel)
#   - Optional floor-plane filter to prevent floor-as-obstacle failures
#   - Post-run log tail and archive summary
#
# Usage:
#   ./wifi_nav.sh                  # manual waypoint navigation
#   ./wifi_nav.sh --explore        # autonomous frontier exploration
#   ./wifi_nav.sh --explore --floor-filter  # exploration + floor filter
#   ./wifi_nav.sh --plan           # preview plans only (no motion)
#   ./wifi_nav.sh --record         # also save input data for replay
#   ./wifi_nav.sh --clean          # clean both ends and exit
#   ./wifi_nav.sh --check          # verify prerequisites only
#   ./wifi_nav.sh --setup-ssh      # one-time passwordless SSH setup
#
# In RViz with --explore:
#   1. Wait ~12 s for Nav2 lifecycle
#   2. 2D Pose Estimate — set robot's current position
#   3. Click "Auto Explore: START" in the Go2 Explore panel
#   4. Robot explores autonomously; click STOP to end
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# --- Configuration ---------------------------------------------------------
NAV2_WIFI_DIR="${NAV2_WIFI_DIR:-$HOME/files/go2-nav2-wifi}"
GO2_HOST_IP="${GO2_HOST_IP:-192.168.1.5}"
GO2_ROBOT_IP="${GO2_ROBOT_IP:-192.168.1.9}"
GO2_ROBOT_USER="${GO2_ROBOT_USER:-unitree}"
GO2_RELAY_DOMAIN_ID="${GO2_RELAY_DOMAIN_ID:-64}"
MAPS_DIR="${NAV2_WIFI_DIR}/ws/maps/lidar3d"
ROBOT_MAPS_DIR="${ROBOT_MAPS_DIR:-go2-nav2-lidar3d-onboard/ws/maps/lidar3d}"
CONTAINER_NAME="${CONTAINER_NAME:-go2-lidar3d}"
FLOOR_FILTER_SCRIPT="${SCRIPT_DIR}/go2_floor_filter.py"

# --- Parse args ------------------------------------------------------------
MODE="--nav"
ACTION="run"
EXPLORE="false"
FLOOR_FILTER="false"
export GO2_RECORD="${GO2_RECORD:-0}"

while [ $# -gt 0 ]; do
    case "$1" in
        --explore)      EXPLORE="true" ;;
        --floor-filter) FLOOR_FILTER="true" ;;
        --plan)         MODE="--plan" ;;
        --nav)          MODE="--nav" ;;
        --record)       export GO2_RECORD=1 ;;
        --clean)        ACTION="clean" ;;
        --check)        ACTION="check" ;;
        --setup-ssh)    ACTION="setup-ssh" ;;
        -h|--help)      sed -n '2,32p' "$0"; exit 0 ;;
        *)              echo "Unknown option: $1" >&2; exit 1 ;;
    esac
    shift
done

# ===========================================================================
# Helper: kill every explore_lite process inside the container by explicit PID
# ===========================================================================
kill_explore_inside_container() {
    docker exec -u root "${CONTAINER_NAME}" bash -c '
        pids=$(ps -eo pid,cmd | grep -E "explore_lite|ros2 run explore" \
               | grep -v grep | awk "{print \$1}")
        if [ -n "$pids" ]; then
            for pid in $pids; do
                kill -9 "$pid" 2>/dev/null || true
            done
        fi
        sleep 1
    ' 2>/dev/null || true
}

# ===========================================================================
# Helper: kill floor filter inside the container
# ===========================================================================
kill_floor_filter_inside_container() {
    docker exec -u root "${CONTAINER_NAME}" bash -c '
        pids=$(ps -eo pid,cmd | grep -E "go2_floor_filter" \
               | grep -v grep | awk "{print \$1}")
        if [ -n "$pids" ]; then
            for pid in $pids; do
                kill -9 "$pid" 2>/dev/null || true
            done
        fi
    ' 2>/dev/null || true
}

# ===========================================================================
# Cleanup: kills stale processes and removes incomplete runs
# ===========================================================================
do_clean() {
    # --- Local side ---
    echo "[clean] Killing local stale processes..."
    pkill -9 -f "rviz2"             2>/dev/null || true
    pkill -9 -f "mapping.sh"        2>/dev/null || true
    pkill -9 -f "pointlio_mapping"  2>/dev/null || true
    pkill -9 -f "go2_cloud_stamp"   2>/dev/null || true

    # --- Inside the container ---
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${CONTAINER_NAME}"; then
        echo "[clean] Killing stale explore_lite + floor filter inside container..."
        kill_explore_inside_container
        kill_floor_filter_inside_container
    fi

    # --- Docker compose down ---
    echo "[clean] Stopping local Docker containers..."
    if [ -f "${NAV2_WIFI_DIR}/docker/docker-compose.yml" ]; then
        (cd "${NAV2_WIFI_DIR}/docker" && docker compose down 2>/dev/null) || true
    fi

    # --- Remote side ---
    if ! ssh -o BatchMode=yes -o ConnectTimeout=5 \
            "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" "true" >/dev/null 2>&1; then
        echo "[clean] WARN: cannot reach robot — skipping remote cleanup"
        return 0
    fi

    echo "[clean] Killing remote stale relays..."
    ssh -o BatchMode=yes "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" '
        pkill -9 -f robot_relay_wifi   2>/dev/null || true
        pkill -9 -f robot-relay-wifi   2>/dev/null || true
        pkill -9 -f go2_cmd_vel_tcp    2>/dev/null || true
        pkill -9 -f robot_sport_bridge 2>/dev/null || true
        sleep 1
    ' >/dev/null 2>&1 || true

    echo "[clean] Removing incomplete runs on the robot..."
    ssh -o BatchMode=yes "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" "
        BASE=\"\$HOME/${ROBOT_MAPS_DIR}\"
        if [ -d \"\$BASE\" ]; then
            removed=0
            for d in \"\$BASE\"/run-*/; do
                [ -d \"\$d\" ] || continue
                if [ ! -f \"\$d/result.json\" ]; then
                    echo \"  removing incomplete: \$(basename \$d)\"
                    rm -rf \"\$d\"
                    removed=\$((removed + 1))
                fi
            done
            echo \"[clean] removed \$removed incomplete run(s)\"
            echo \"[clean] remaining runs on robot:\"
            ls -1 \"\$BASE\" 2>/dev/null | head -10 || true
        fi
    " 2>/dev/null || true

    echo "[clean] Done."
}

# ===========================================================================
# Dispatch: setup-ssh
# ===========================================================================
if [ "${ACTION}" = "setup-ssh" ]; then
    if [ -x "${SCRIPT_DIR}/wifi_mapping.sh" ]; then
        exec "${SCRIPT_DIR}/wifi_mapping.sh" --setup-ssh
    else
        echo "ERROR: wifi_mapping.sh not found — cannot delegate --setup-ssh." >&2
        exit 1
    fi
fi

# ===========================================================================
# Dispatch: clean-only
# ===========================================================================
if [ "${ACTION}" = "clean" ]; then
    echo "================================================================"
    echo " Cleaning stale processes and partial runs"
    echo "================================================================"
    do_clean
    exit 0
fi

# ===========================================================================
# Banner
# ===========================================================================
echo "================================================================"
echo " Go2 Nav2 Wi-Fi waypoint navigation"
echo "================================================================"
echo "  Action      : ${ACTION}"
echo "  Mode        : ${MODE}"
echo "  Explore     : ${EXPLORE}"
echo "  Floor filter: ${FLOOR_FILTER}"
echo "  GO2_RECORD  : ${GO2_RECORD}"
echo "  Host IP     : ${GO2_HOST_IP}"
echo "  Robot       : ${GO2_ROBOT_USER}@${GO2_ROBOT_IP}"
echo "  Container   : ${CONTAINER_NAME}"
echo "  Repo        : ${NAV2_WIFI_DIR}"
echo

# ===========================================================================
# Prerequisite checks
# ===========================================================================
if ! command -v docker >/dev/null 2>&1; then
    echo "ERROR: docker not found." >&2
    echo "  Install:  curl -fsSL https://get.docker.com | sudo sh" >&2
    exit 1
fi
if ! docker ps >/dev/null 2>&1; then
    echo "ERROR: cannot talk to Docker daemon." >&2
    echo "  Try:  newgrp docker" >&2
    exit 1
fi
[ -d "${NAV2_WIFI_DIR}" ] || {
    echo "ERROR: ${NAV2_WIFI_DIR} not found." >&2
    exit 1
}
[ -f "${NAV2_WIFI_DIR}/mapping.sh" ] || {
    echo "ERROR: ${NAV2_WIFI_DIR}/mapping.sh not found." >&2
    exit 1
}
echo "[checks] docker + repo present .................... OK"

SSH_KEY_READY="false"
if [ -f "${HOME}/.ssh/id_ed25519" ] && \
   ssh -o BatchMode=yes -o ConnectTimeout=3 \
       "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" "echo OK" >/dev/null 2>&1; then
    SSH_KEY_READY="true"
    echo "[checks] passwordless SSH to robot ............... OK"
else
    echo "[checks] passwordless SSH to robot ............... NOT SET UP"
    echo "         Run:  ./wifi_nav.sh --setup-ssh"
fi

# --- Explore prerequisite checks -------------------------------------------
if [ "${EXPLORE}" = "true" ]; then
    if docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}" && \
       docker exec -u root "${CONTAINER_NAME}" bash -c \
           "source /opt/ros/humble/setup.bash && \
            source /ws/install/setup.bash 2>/dev/null; \
            ros2 pkg executables explore_lite 2>/dev/null | grep -q explore" \
       2>/dev/null; then
        echo "[checks] explore_lite in container ................ OK"
    else
        echo "[checks] explore_lite in container ................ MISSING"
    fi

    if docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}" && \
       docker exec -u root "${CONTAINER_NAME}" bash -c \
           "source /opt/ros/humble/setup.bash && \
            source /ws/install/setup.bash 2>/dev/null; \
            ros2 pkg list 2>/dev/null | grep -q go2_explore_panel" \
       2>/dev/null; then
        echo "[checks] go2_explore_panel built .................. OK"
    else
        echo "[checks] go2_explore_panel built .................. MISSING"
    fi
fi

# --- Floor filter prerequisite check ---------------------------------------
if [ "${FLOOR_FILTER}" = "true" ]; then
    if [ -f "${FLOOR_FILTER_SCRIPT}" ]; then
        echo "[checks] floor filter script present .............. OK"
    else
        echo "[checks] floor filter script present .............. MISSING"
        echo "         Expected at: ${FLOOR_FILTER_SCRIPT}"
        echo "         Set FLOOR_FILTER=false or provide the script."
        FLOOR_FILTER="false"
    fi
fi

# ===========================================================================
# Pre-flight cleanup (skipped for --check)
# ===========================================================================
if [ "${ACTION}" != "check" ]; then
    echo
    echo "[pre-flight] Cleaning stale processes + incomplete runs..."
    do_clean
    echo
fi

# ===========================================================================
# Dispatch: check-only
# ===========================================================================
if [ "${ACTION}" = "check" ]; then
    echo
    echo "All checks done. Run without --check to launch."
    echo
    echo "Workflow:"
    echo "  1. Start robot relay in FULL mode (separate terminal):"
    echo "       ssh ${GO2_ROBOT_USER}@${GO2_ROBOT_IP}"
    echo "       export GO2_HOST_IP=${GO2_HOST_IP}"
    echo "       bash ~/robot-relay-wifi.sh"
    echo
    echo "  2. Then run:  ./wifi_nav.sh [--explore] [--floor-filter]"
    exit 0
fi

# ===========================================================================
# Reminder
# ===========================================================================
echo "================================================================"
echo " REMINDER — robot relay must be running in FULL mode"
echo "================================================================"
echo
echo " In a SEPARATE terminal, start the relay on the robot:"
echo
echo "     ssh ${GO2_ROBOT_USER}@${GO2_ROBOT_IP}"
echo "     export GO2_HOST_IP=${GO2_HOST_IP}"
echo "     bash ~/robot-relay-wifi.sh"
echo
echo " Watch for:  'Relay running: onboard domain 0 -> Wi-Fi domain 64'"
echo " Do NOT run with --sensors-only (blocks /cmd_vel)."
echo "================================================================"
echo

if [ "${EXPLORE}" = "true" ]; then
    echo "================================================================"
    echo " EXPLORE MODE — the robot will move autonomously"
    echo "================================================================"
    echo
    echo " In RViz:"
    echo "   1. Wait ~12 s for Nav2 lifecycle"
    echo "   2. 2D Pose Estimate — set robot's current position"
    echo "   3. Click 'Auto Explore: START' in the Go2 Explore panel"
    echo "   4. Robot explores autonomously; click STOP to end"
    echo
    echo " Emergency stop: physical remote or Ctrl+C in this terminal."
    echo "================================================================"
    echo
fi

# ===========================================================================
# Launch
# ===========================================================================
cd "${NAV2_WIFI_DIR}"
export GO2_HOST_IP GO2_ROBOT_IP GO2_RELAY_DOMAIN_ID

EXPLORE_HOST_PID=""

cleanup() {
    echo
    echo "[wifi_nav] Shutting down..."
    if [ -n "${EXPLORE_HOST_PID}" ]; then
        kill "${EXPLORE_HOST_PID}" 2>/dev/null || true
    fi
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${CONTAINER_NAME}"; then
        kill_explore_inside_container
        kill_floor_filter_inside_container
    fi
    (cd "${NAV2_WIFI_DIR}/docker" && docker compose down 2>/dev/null) || true
    echo "[wifi_nav] Done."
}
trap cleanup EXIT INT TERM

echo "Launching: ./mapping.sh --3d ${MODE}"
echo

if [ "${EXPLORE}" = "true" ] || [ "${FLOOR_FILTER}" = "true" ]; then
    # ------------------------------------------------------------------
    # Background: wait for mapping + Nav2, then start optional helpers
    # ------------------------------------------------------------------
    (
        sleep 20

        # ---------- Floor filter ----------
        if [ "${FLOOR_FILTER}" = "true" ]; then
            # Kill any stale floor filter
            docker exec -u root "${CONTAINER_NAME}" bash -c '
                pids=$(ps -eo pid,cmd | grep -E "go2_floor_filter" \
                       | grep -v grep | awk "{print \$1}")
                if [ -n "$pids" ]; then
                    for pid in $pids; do
                        kill -9 "$pid" 2>/dev/null || true
                    done
                fi
            ' 2>/dev/null || true

            # Copy script into the container (fresh each run)
            docker cp "${FLOOR_FILTER_SCRIPT}" \
                "${CONTAINER_NAME}:/tmp/go2_floor_filter.py" 2>/dev/null || true

            # Launch it
            docker exec -u root "${CONTAINER_NAME}" bash -c \
                "source /opt/ros/humble/setup.bash && \
                 source /ws/install/setup.bash 2>/dev/null; \
                 nohup python3 /tmp/go2_floor_filter.py \
                     --ros-args \
                     -p input_topic:=/lidar3d/registered \
                     -p output_topic:=/lidar3d/obstacle_points \
                     -p floor_margin:=0.20 \
                     > /tmp/floor_filter.log 2>&1 &" \
                2>/dev/null || true
        fi

        # ---------- explore_lite ----------
        if [ "${EXPLORE}" = "true" ]; then
            # Write params file with WILDCARD namespace so explore_lite picks them up
            EXPLORE_PARAMS_TMP=$(mktemp /tmp/explore_params_XXXXXX.yaml)
            cat > "${EXPLORE_PARAMS_TMP}" << 'EOF'
/**:
  ros__parameters:
    robot_base_frame: base_link
    costmap_topic: /map
    costmap_updates_topic: /map_updates
    visualize: true
    planner_frequency: 0.5
    progress_timeout: 30.0
    potential_scale: 3.0
    orientation_scale: 0.0
    gain_scale: 1.0
    transform_tolerance: 0.3
    min_frontier_size: 0.75
    return_to_init: false
EOF
            docker cp "${EXPLORE_PARAMS_TMP}" \
                "${CONTAINER_NAME}:/tmp/explore_params.yaml" 2>/dev/null || true
            rm -f "${EXPLORE_PARAMS_TMP}"

            # Kill stale instances before starting fresh
            docker exec -u root "${CONTAINER_NAME}" bash -c '
                pids=$(ps -eo pid,cmd | grep -E "explore_lite|ros2 run explore" \
                       | grep -v grep | awk "{print \$1}")
                if [ -n "$pids" ]; then
                    for pid in $pids; do
                        kill -9 "$pid" 2>/dev/null || true
                    done
                fi
                sleep 1
            ' 2>/dev/null || true

            # Start ONE fresh instance
            docker exec -u root "${CONTAINER_NAME}" bash -c \
                "source /opt/ros/humble/setup.bash && \
                 source /ws/install/setup.bash 2>/dev/null; \
                 exec ros2 run explore_lite explore \
                     --ros-args --params-file /tmp/explore_params.yaml" \
                > "${NAV2_WIFI_DIR}/ws/log/explore.log" 2>&1 &
        fi
    ) &
    EXPLORE_HOST_PID=$!
    echo "[wifi_nav] background helpers will start after ~20 s (host PID ${EXPLORE_HOST_PID})"
    echo
fi

./mapping.sh --3d ${MODE} || true

# ===========================================================================
# Post-run: slam log tail
# ===========================================================================
echo
echo "================================================================"
echo " Last 20 lines of the newest slam log:"
echo "================================================================"
LATEST_LOG=$(ls -1t "${NAV2_WIFI_DIR}/ws/log/" 2>/dev/null \
    | grep "^mapping" | head -1 || true)
if [ -n "${LATEST_LOG}" ] && \
   [ -f "${NAV2_WIFI_DIR}/ws/log/${LATEST_LOG}/slam.log" ]; then
    tail -20 "${NAV2_WIFI_DIR}/ws/log/${LATEST_LOG}/slam.log"
else
    echo "  (no slam.log found)"
fi
echo

# ===========================================================================
# Post-run: explore / floor filter logs
# ===========================================================================
if [ "${EXPLORE}" = "true" ] && \
   [ -f "${NAV2_WIFI_DIR}/ws/log/explore.log" ]; then
    echo "================================================================"
    echo " Last 20 lines of explore.log:"
    echo "================================================================"
    tail -20 "${NAV2_WIFI_DIR}/ws/log/explore.log"
    echo
fi

# ===========================================================================
# Post-run: archived runs summary
# ===========================================================================
echo "================================================================"
echo " Archived runs in ${MAPS_DIR}:"
echo "================================================================"
if [ -d "${MAPS_DIR}" ]; then
    ls -1t "${MAPS_DIR}" 2>/dev/null | head -5 | while read -r d; do
        [ -d "${MAPS_DIR}/${d}" ] || continue
        size=$(du -sh "${MAPS_DIR}/${d}" 2>/dev/null | cut -f1)
        if [ -f "${MAPS_DIR}/${d}/result.json" ]; then
            printf "  %-45s  (%s)  ✓ complete\n" "$d" "$size"
        else
            printf "  %-45s  (%s)  ✗ partial\n"  "$d" "$size"
        fi
    done
else
    echo "  (no runs yet)"
fi
echo
echo "Run './wifi_mapping.sh --list' for full details."
