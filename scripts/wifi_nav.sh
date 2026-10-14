#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# wifi_nav.sh
#
# Live waypoint navigation + autonomous frontier exploration over Wi-Fi
# using the go2-nav2-wifi stack, with a CUSTOM frontier explorer.
#
# One-command workflow:
#   1. Power on the dog and stand it
#   2. Run:  ./wifi_nav.sh
#   3. In RViz: wait ~12 s, set 2D Pose Estimate, click "Auto Explore: START"
#
# Bare invocation turns ON explore + camera. Use flags to turn them off:
#   --no-explore    manual waypoint mode (2D Goal Pose only)
#   --no-camera     skip the camera relay
#   --plan          preview only, no motion, no explore, no camera
#
# Network (Wi-Fi):
#   VM      : 192.168.1.4
#   Robot   : 192.168.1.9
#
# Usage:
#   ./wifi_nav.sh                            # explore + camera (default)
#   ./wifi_nav.sh --no-camera                # explore only
#   ./wifi_nav.sh --no-explore               # camera only, manual goals
#   ./wifi_nav.sh --explore --floor-filter   # + floor plane filter
#   ./wifi_nav.sh --nav                      # manual waypoint navigation
#   ./wifi_nav.sh --plan                     # preview only (no motion)
#   ./wifi_nav.sh --record                   # also save input data for replay
#   ./wifi_nav.sh --clean                    # clean both ends and exit
#   ./wifi_nav.sh --check                    # verify prerequisites only
#   ./wifi_nav.sh --setup-ssh                # one-time passwordless SSH setup
#   ./wifi_nav.sh --no-relay                 # skip auto relay startup
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# --- Configuration ---------------------------------------------------------
NAV2_WIFI_DIR="${NAV2_WIFI_DIR:-$HOME/files/go2-nav2-wifi}"

GO2_HOST_IP="${GO2_HOST_IP:-192.168.1.4}"
GO2_ROBOT_IP="${GO2_ROBOT_IP:-192.168.1.9}"

GO2_ROBOT_USER="${GO2_ROBOT_USER:-unitree}"
GO2_RELAY_DOMAIN_ID="${GO2_RELAY_DOMAIN_ID:-64}"
MAPS_DIR="${NAV2_WIFI_DIR}/ws/maps/lidar3d"
ROBOT_MAPS_DIR="${ROBOT_MAPS_DIR:-go2-nav2-lidar3d-onboard/ws/maps/lidar3d}"
CONTAINER_NAME="${CONTAINER_NAME:-go2-lidar3d}"
FLOOR_FILTER_SCRIPT="${SCRIPT_DIR}/go2_floor_filter.py"
EXPLORER_SCRIPT="${SCRIPT_DIR}/frontier_explorer.py"
CAMERA_RELAY_SCRIPT="${SCRIPT_DIR}/go2_camera_relay.py"

# --- SAFETY thresholds -----------------------------------------------------
SAFETY_STOP_DISTANCE="${SAFETY_STOP_DISTANCE:-0.45}"
SAFETY_WARN_DISTANCE="${SAFETY_WARN_DISTANCE:-0.70}"
SAFETY_MAX_VEL_X="${SAFETY_MAX_VEL_X:-0.20}"
SAFETY_MAX_VEL_THETA="${SAFETY_MAX_VEL_THETA:-0.50}"
SAFETY_INFLATION_RADIUS="${SAFETY_INFLATION_RADIUS:-0.70}"

# --- Explore runtime overrides --------------------------------------------
EXPLORE_MAP_FRAME="${EXPLORE_MAP_FRAME:-lidar3d_map}"
EXPLORE_MAP_TOPIC="${EXPLORE_MAP_TOPIC:-/global_costmap/costmap}"
EXPLORE_STOP_TOPIC="${EXPLORE_STOP_TOPIC:-/cmd_vel}"
EXPLORE_MIN_FRONTIER_SIZE="${EXPLORE_MIN_FRONTIER_SIZE:-4}"
EXPLORE_MIN_KNOWN_CELLS="${EXPLORE_MIN_KNOWN_CELLS:-200}"
EXPLORE_BLIND_ENABLE="${EXPLORE_BLIND_ENABLE:-false}"
EXPLORE_COVERAGE_ENABLE="${EXPLORE_COVERAGE_ENABLE:-false}"
EXPLORE_RETURN_TO_INIT="${EXPLORE_RETURN_TO_INIT:-false}"

# --- Parse args ------------------------------------------------------------
# Default: explore + camera ON.
MODE="--nav"
ACTION="run"
EXPLORE="true"
FLOOR_FILTER="false"
CAMERA="true"
START_RELAY="true"
export GO2_RECORD="${GO2_RECORD:-0}"

while [ $# -gt 0 ]; do
    case "$1" in
        --explore)      EXPLORE="true" ;;
        --no-explore)   EXPLORE="false" ;;
        --camera)       CAMERA="true" ;;
        --no-camera)    CAMERA="false" ;;
        --floor-filter) FLOOR_FILTER="true" ;;
        --plan)         MODE="--plan"; EXPLORE="false"; CAMERA="false" ;;
        --nav)          MODE="--nav";  EXPLORE="false" ;;
        --record)       export GO2_RECORD=1 ;;
        --clean)        ACTION="clean" ;;
        --check)        ACTION="check" ;;
        --setup-ssh)    ACTION="setup-ssh" ;;
        --no-relay)     START_RELAY="false" ;;
        -h|--help)      sed -n '2,42p' "$0"; exit 0 ;;
        *)              echo "Unknown option: $1" >&2; exit 1 ;;
    esac
    shift
done

# ===========================================================================
# Helpers
# ===========================================================================
kill_explore_inside_container() {
    docker exec -u root "${CONTAINER_NAME}" bash -c '
        pids=$(ps -eo pid,cmd | grep -E "frontier_explorer|explore_lite|ros2 run explore" \
               | grep -v grep | awk "{print \$1}")
        if [ -n "$pids" ]; then
            for pid in $pids; do kill -9 "$pid" 2>/dev/null || true; done
        fi
        sleep 1
    ' 2>/dev/null || true
}

kill_floor_filter_inside_container() {
    docker exec -u root "${CONTAINER_NAME}" bash -c '
        pids=$(ps -eo pid,cmd | grep -E "go2_floor_filter" \
               | grep -v grep | awk "{print \$1}")
        if [ -n "$pids" ]; then
            for pid in $pids; do kill -9 "$pid" 2>/dev/null || true; done
        fi
    ' 2>/dev/null || true
}

kill_camera_relay_inside_container() {
    docker exec -u root "${CONTAINER_NAME}" bash -c '
        pids=$(ps -eo pid,cmd | grep -E "go2_camera_relay" \
               | grep -v grep | awk "{print \$1}")
        if [ -n "$pids" ]; then
            for pid in $pids; do kill -9 "$pid" 2>/dev/null || true; done
        fi
    ' 2>/dev/null || true
}

stop_exploration() {
    docker exec -u root "${CONTAINER_NAME}" bash -c '
        source /opt/ros/humble/setup.bash
        source /ws/install/setup.bash 2>/dev/null
        timeout 3 ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}" 2>/dev/null || true
    ' 2>/dev/null || true
}

start_robot_relay() {
    if [ "${START_RELAY}" != "true" ]; then
        echo "[relay] Skipping (--no-relay)"
        return 0
    fi

    echo "[relay] Reaching robot at ${GO2_ROBOT_USER}@${GO2_ROBOT_IP}..."

    if ! ssh -o BatchMode=yes -o ConnectTimeout=5 \
            "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" "true" >/dev/null 2>&1; then
        echo "[relay] ERROR: cannot SSH to robot."
        echo "         Run:  ./wifi_nav.sh --setup-ssh"
        return 1
    fi

    echo "[relay] Killing stale relay processes and starting FULL mode..."

    ssh -o BatchMode=yes -o ConnectTimeout=10 \
        "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" bash -lc "
        set -e
        pkill -9 -f 'robot-relay-wifi|robot-motion-wifi|go2_cmd_vel_tcp|robot_sport_bridge' 2>/dev/null || true
        rm -f /tmp/go2-relay-wifi.sock*
        sleep 3

        if ss -tlnp 2>/dev/null | grep -q ':17999'; then
            echo '[relay] ERROR: port 17999 still held after pkill'
            ss -tlnp 2>/dev/null | grep ':17999'
            exit 1
        fi

        export GO2_HOST_IP=${GO2_HOST_IP}
        nohup bash ~/robot-relay-wifi.sh > /tmp/robot-relay.log 2>&1 &
        echo \$! > /tmp/robot-relay.pid

        for i in \$(seq 1 20); do
            if ss -tlnp 2>/dev/null | grep -q ':17999'; then
                echo '[relay] FULL mode confirmed on port 17999'
                exit 0
            fi
            sleep 1
        done

        echo '[relay] ERROR: port 17999 never bound. Last 30 lines:'
        tail -30 /tmp/robot-relay.log 2>/dev/null || true
        exit 1
    " || {
        echo "[relay] ERROR: relay start failed."
        echo "[relay] Debug: ssh ${GO2_ROBOT_USER}@${GO2_ROBOT_IP} 'cat /tmp/robot-relay.log'"
        return 1
    }

    return 0
}

stop_robot_relay() {
    if [ "${START_RELAY}" != "true" ]; then
        return 0
    fi
    ssh -o BatchMode=yes -o ConnectTimeout=3 \
        "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" \
        "pkill -9 -f 'robot-relay-wifi|robot-motion-wifi|go2_cmd_vel_tcp|robot_sport_bridge' 2>/dev/null || true" \
        >/dev/null 2>&1 || true
}

do_clean() {
    echo "[clean] Killing local stale processes..."
    pkill -9 -f "rviz2"             2>/dev/null || true
    pkill -9 -f "mapping.sh"        2>/dev/null || true
    pkill -9 -f "pointlio_mapping"  2>/dev/null || true
    pkill -9 -f "go2_cloud_stamp"   2>/dev/null || true
    pkill -9 -f "frontier_explorer" 2>/dev/null || true
    pkill -9 -f "go2_camera_relay"  2>/dev/null || true
    pkill -9 -f "safety_watchdog"   2>/dev/null || true

    if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${CONTAINER_NAME}"; then
        echo "[clean] Killing stale helpers inside container..."
        kill_explore_inside_container
        kill_floor_filter_inside_container
        kill_camera_relay_inside_container
    fi

    echo "[clean] Stopping local Docker containers..."
    if [ -f "${NAV2_WIFI_DIR}/docker/docker-compose.yml" ]; then
        (cd "${NAV2_WIFI_DIR}/docker" && docker compose down 2>/dev/null) || true
    fi

    if ! ssh -o BatchMode=yes -o ConnectTimeout=5 \
            "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" "true" >/dev/null 2>&1; then
        echo "[clean] WARN: cannot reach robot at ${GO2_ROBOT_IP} — skipping remote cleanup"
        return 0
    fi

    echo "[clean] Killing remote stale relays..."
    ssh -o BatchMode=yes "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" '
        pkill -9 -f robot_relay_wifi   2>/dev/null || true
        pkill -9 -f robot-relay-wifi   2>/dev/null || true
        pkill -9 -f robot-motion-wifi  2>/dev/null || true
        pkill -9 -f go2_cmd_vel_tcp    2>/dev/null || true
        pkill -9 -f robot_sport_bridge 2>/dev/null || true
        rm -f /tmp/robot-relay.pid
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
        echo "ERROR: wifi_mapping.sh not found." >&2; exit 1
    fi
fi

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
echo " Go2 Nav2 Wi-Fi navigation"
echo "================================================================"
echo "  Action      : ${ACTION}"
echo "  Mode        : ${MODE}"
echo "  Explore     : ${EXPLORE}"
echo "  Floor filter: ${FLOOR_FILTER}"
echo "  Camera      : ${CAMERA}"
echo "  Auto relay  : ${START_RELAY}"
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
    echo "ERROR: docker not found." >&2; exit 1
fi
if ! docker ps >/dev/null 2>&1; then
    echo "ERROR: cannot talk to Docker daemon. Try: newgrp docker" >&2; exit 1
fi
[ -d "${NAV2_WIFI_DIR}" ] || { echo "ERROR: ${NAV2_WIFI_DIR} not found." >&2; exit 1; }
[ -f "${NAV2_WIFI_DIR}/mapping.sh" ] || { echo "ERROR: mapping.sh not found." >&2; exit 1; }
echo "[checks] docker + repo present .................... OK"

if ssh -o BatchMode=yes -o ConnectTimeout=3 \
        "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" "echo OK" >/dev/null 2>&1; then
    echo "[checks] passwordless SSH to robot ............... OK"
else
    echo "[checks] passwordless SSH to robot ............... NOT SET UP"
    echo "         Run:  ./wifi_nav.sh --setup-ssh"
    if [ "${START_RELAY}" = "true" ] && [ "${MODE}" != "--plan" ]; then
        echo "[checks] ERROR: auto relay requires SSH. Use --no-relay to skip."
        exit 1
    fi
fi

if [ "${EXPLORE}" = "true" ]; then
    if [ -f "${EXPLORER_SCRIPT}" ]; then
        echo "[checks] frontier_explorer.py present ............ OK"
    else
        echo "[checks] frontier_explorer.py present ............ MISSING"
        EXPLORE="false"
    fi

    if docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}" && \
       docker exec -u root "${CONTAINER_NAME}" bash -c \
           "source /opt/ros/humble/setup.bash; \
            source /ws/install/setup.bash 2>/dev/null; \
            ros2 pkg list 2>/dev/null | grep -q go2_explore_panel" \
       2>/dev/null; then
        echo "[checks] go2_explore_panel built .................. OK"
    else
        echo "[checks] go2_explore_panel built .................. MISSING"
    fi
fi

if [ "${FLOOR_FILTER}" = "true" ]; then
    if [ -f "${FLOOR_FILTER_SCRIPT}" ]; then
        echo "[checks] floor filter script present .............. OK"
    else
        echo "[checks] floor filter script present .............. MISSING"
        FLOOR_FILTER="false"
    fi
fi

if [ "${CAMERA}" = "true" ]; then
    if [ -f "${CAMERA_RELAY_SCRIPT}" ]; then
        echo "[checks] camera relay script present .............. OK"
    else
        echo "[checks] camera relay script present .............. MISSING"
        CAMERA="false"
    fi
fi

# ===========================================================================
# Pre-flight cleanup
# ===========================================================================
if [ "${ACTION}" != "check" ]; then
    echo
    echo "[pre-flight] Cleaning stale processes + incomplete runs..."
    do_clean
    echo
fi

if [ "${ACTION}" = "check" ]; then
    echo "All checks done."
    exit 0
fi

# ===========================================================================
# Start robot relay (automated)
# ===========================================================================
if [ "${MODE}" != "--plan" ] && [ "${START_RELAY}" = "true" ]; then
    echo "================================================================"
    echo " Starting robot relay (FULL mode)"
    echo "================================================================"
    if ! start_robot_relay; then
        echo
        echo "ERROR: could not start robot relay."
        echo "  Check: ssh ${GO2_ROBOT_USER}@${GO2_ROBOT_IP} 'cat /tmp/robot-relay.log'"
        exit 1
    fi
    echo
fi

# ===========================================================================
# Explore-mode banner
# ===========================================================================
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
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${CONTAINER_NAME}"; then
        echo "[wifi_nav] SAFETY: publishing stop to /explore/resume"
        stop_exploration
    fi
    if [ -n "${EXPLORE_HOST_PID}" ]; then
        kill "${EXPLORE_HOST_PID}" 2>/dev/null || true
    fi
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${CONTAINER_NAME}"; then
        kill_explore_inside_container
        kill_floor_filter_inside_container
        kill_camera_relay_inside_container
    fi
    (cd "${NAV2_WIFI_DIR}/docker" && docker compose down 2>/dev/null) || true
    stop_robot_relay
    echo "[wifi_nav] Done."
}
trap cleanup EXIT INT TERM

echo "Launching: ./mapping.sh --3d ${MODE}"
echo

if [ "${EXPLORE}" = "true" ] || [ "${FLOOR_FILTER}" = "true" ] || [ "${CAMERA}" = "true" ]; then
    (
        sleep 20

        # ---------- Floor filter ----------
        if [ "${FLOOR_FILTER}" = "true" ]; then
            docker exec -u root "${CONTAINER_NAME}" bash -c '
                pids=$(ps -eo pid,cmd | grep -E "go2_floor_filter" \
                       | grep -v grep | awk "{print \$1}")
                if [ -n "$pids" ]; then
                    for pid in $pids; do kill -9 "$pid" 2>/dev/null || true; done
                fi
            ' 2>/dev/null || true

            docker cp "${FLOOR_FILTER_SCRIPT}" \
                "${CONTAINER_NAME}:/tmp/go2_floor_filter.py" 2>/dev/null || true

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

        # ---------- Camera relay ----------
        if [ "${CAMERA}" = "true" ]; then
            echo "[camera] Killing stale camera relay instances..."
            kill_camera_relay_inside_container
            echo "[camera] Copying go2_camera_relay.py into container..."
            docker cp "${CAMERA_RELAY_SCRIPT}" \
                "${CONTAINER_NAME}:/tmp/go2_camera_relay.py" 2>/dev/null || true
            echo "[camera] Starting camera relay..."
            docker exec -u root "${CONTAINER_NAME}" bash -c \
                "source /opt/ros/humble/setup.bash && \
                 source /ws/install/setup.bash 2>/dev/null; \
                 nohup python3 /tmp/go2_camera_relay.py \
                     --ros-args \
                     -p source_topic:=/frontvideostream \
                     -p output_topic:=/camera/image_raw \
                     -p frame_id:=camera_link \
                     -p resolution:=720p \
                     > /tmp/camera_relay.log 2>&1 &" \
                2>/dev/null || true
        fi

        # ---------- Custom frontier explorer ----------
        if [ "${EXPLORE}" = "true" ]; then
            echo "[explore] Killing stale explorer instances..."
            docker exec -u root "${CONTAINER_NAME}" bash -c '
                pids=$(ps -eo pid,cmd | grep -E "frontier_explorer|explore_lite|ros2 run explore" \
                       | grep -v grep | awk "{print \$1}")
                if [ -n "$pids" ]; then
                    for pid in $pids; do kill -9 "$pid" 2>/dev/null || true; done
                fi
                sleep 1
            ' 2>/dev/null || true

            echo "[explore] Copying frontier_explorer.py into container..."
            docker cp "${EXPLORER_SCRIPT}" \
                "${CONTAINER_NAME}:/tmp/frontier_explorer.py" 2>/dev/null || true

            echo "[explore] Starting custom frontier explorer..."
            docker exec -u root "${CONTAINER_NAME}" bash -c \
                "source /opt/ros/humble/setup.bash && \
                 source /ws/install/setup.bash 2>/dev/null; \
                 exec python3 /tmp/frontier_explorer.py \
                     --ros-args \
                     -p robot_base_frame:=base_link \
                     -p map_frame:=${EXPLORE_MAP_FRAME} \
                     -p map_topic:=${EXPLORE_MAP_TOPIC} \
                     -p goal_topic:=/goal_pose \
                     -p cmd_vel_topic:=/cmd_vel_nav \
                     -p stop_topic:=${EXPLORE_STOP_TOPIC} \
                     -p min_frontier_size:=${EXPLORE_MIN_FRONTIER_SIZE} \
                     -p min_known_cells:=${EXPLORE_MIN_KNOWN_CELLS} \
                     -p blind_enable:=${EXPLORE_BLIND_ENABLE} \
                     -p blind_trigger_cells:=100 \
                     -p blind_step_distance:=0.6 \
                     -p blind_step_speed:=0.12 \
                     -p coverage_enable:=${EXPLORE_COVERAGE_ENABLE} \
                     -p return_to_init:=${EXPLORE_RETURN_TO_INIT} \
                     -p tick_hz:=1.0" \
                > "${NAV2_WIFI_DIR}/ws/log/frontier.log" 2>&1 &
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
# Post-run: frontier explorer log tail
# ===========================================================================
if [ "${EXPLORE}" = "true" ] && \
   [ -f "${NAV2_WIFI_DIR}/ws/log/frontier.log" ]; then
    echo "================================================================"
    echo " Last 20 lines of frontier.log:"
    echo "================================================================"
    tail -20 "${NAV2_WIFI_DIR}/ws/log/frontier.log"
    echo
fi

# ===========================================================================
# Post-run: camera relay log tail
# ===========================================================================
if [ "${CAMERA}" = "true" ] && \
   docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${CONTAINER_NAME}"; then
    echo "================================================================"
    echo " Last 10 lines of camera_relay.log:"
    echo "================================================================"
    docker exec -u root "${CONTAINER_NAME}" tail -10 /tmp/camera_relay.log 2>/dev/null || echo "  (no log)"
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