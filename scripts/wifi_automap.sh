#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# wifi_automap.sh
#
# Roomba-style autonomous mapping.
#
# Flow:
#   1. Pre-flight cleanup (stale processes, incomplete runs)
#   2. Launch go2-nav2-wifi mapping + Nav2 (via mapping.sh --3d --nav)
#   3. Wait for Nav2 lifecycle nodes to become active
#   4. Auto-publish 'true' to /explore/resume — no button click needed
#   5. explore_lite drives the robot around; return_to_init=true
#   6. When frontiers are exhausted, robot returns to origin
#   7. save-map.sh dumps the 2D occupancy grid
#   8. Clean shutdown
#
# Prerequisites:
#   - Robot relay running in FULL mode (see Reminder section)
#   - go2-nav2-wifi stack installed at $NAV2_WIFI_DIR
#   - explore_lite in the container
#   - go2_explore_panel built (loaded in RViz, but not required for auto-start)
#
# Usage:
#   ./wifi_automap.sh                    # launch + auto-explore + save
#   ./wifi_automap.sh --map-name lab     # custom map name
#   ./wifi_automap.sh --check            # verify prerequisites only
#   ./wifi_automap.sh --clean            # kill stale processes and exit
#
# On Ctrl+C: immediately stops the robot (via explore pause) and saves a map.
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
MAP_NAME="${MAP_NAME:-automap_$(date +%Y%m%d_%H%M%S)}"
MAP_SAVE_PATH="${NAV2_WIFI_DIR}/ws/maps/${MAP_NAME}"

# Timeouts
NAV2_READY_TIMEOUT=45       # seconds to wait for Nav2 lifecycle
EXPLORE_START_DELAY=25      # seconds after mapping.sh starts
RETURN_WAIT_TIMEOUT=120     # seconds to wait for robot to return home
MAX_EXPLORE_TIME=600        # seconds before forcibly stopping exploration

# --- Parse args ------------------------------------------------------------
ACTION="run"

while [ $# -gt 0 ]; do
    case "$1" in
        --map-name) MAP_NAME="$2"; MAP_SAVE_PATH="${NAV2_WIFI_DIR}/ws/maps/${MAP_NAME}"; shift ;;
        --check)    ACTION="check" ;;
        --clean)    ACTION="clean" ;;
        -h|--help)  sed -n '2,35p' "$0"; exit 0 ;;
        *)          echo "Unknown option: $1" >&2; exit 1 ;;
    esac
    shift
done

# ===========================================================================
# Helpers
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

do_clean() {
    echo "[clean] Killing local stale processes..."
    pkill -9 -f "rviz2"             2>/dev/null || true
    pkill -9 -f "mapping.sh"        2>/dev/null || true
    pkill -9 -f "pointlio_mapping"  2>/dev/null || true
    pkill -9 -f "go2_cloud_stamp"   2>/dev/null || true

    if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${CONTAINER_NAME}"; then
        echo "[clean] Killing stale explore_lite inside container..."
        kill_explore_inside_container
    fi

    echo "[clean] Stopping local Docker containers..."
    if [ -f "${NAV2_WIFI_DIR}/docker/docker-compose.yml" ]; then
        (cd "${NAV2_WIFI_DIR}/docker" && docker compose down 2>/dev/null) || true
    fi

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
            ls -1 \"\$BASE\" 2>/dev/null | head -10 || true
        fi
    " 2>/dev/null || true

    echo "[clean] Done."
}

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
echo " Go2 Roomba-style autonomous mapping"
echo "================================================================"
echo "  Action      : ${ACTION}"
echo "  Map name    : ${MAP_NAME}"
echo "  Save to     : ${MAP_SAVE_PATH}"
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
    echo
    echo "All checks done. Run without --check to launch auto-mapping."
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
echo " Do NOT run with --sensors-only."
echo "================================================================"
echo
echo " IMPORTANT: Place the robot on flat ground and stand it with the"
echo " physical remote BEFORE launching. The robot will move autonomously"
echo " once the stack comes up. Emergency stop: physical remote or Ctrl+C."
echo
read -rp " Press Enter when the robot is standing and the relay is running..." _
echo

# ===========================================================================
# Cleanup handler
# ===========================================================================
PIDS=()
EXPLORE_HOST_PID=""
MAP_SAVED="false"

cleanup() {
    echo
    echo "[automap] Shutting down..."

    # 1. Stop the robot — publish false to /explore/resume
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${CONTAINER_NAME}"; then
        docker exec -u root "${CONTAINER_NAME}" bash -c '
            source /opt/ros/humble/setup.bash
            source /ws/install/setup.bash 2>/dev/null
            timeout 3 ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}" 2>/dev/null || true
        ' 2>/dev/null || true
    fi

    # 2. Save the map (before we kill the stack)
    if [ "${MAP_SAVED}" = "false" ] && \
       docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${CONTAINER_NAME}"; then
        echo "[automap] Saving map to ${MAP_SAVE_PATH}.yaml ..."
        docker exec -u root "${CONTAINER_NAME}" bash -c \
            "source /opt/ros/humble/setup.bash && \
             source /ws/install/setup.bash 2>/dev/null; \
             timeout 20 ros2 run nav2_map_server map_saver_cli \
                 -f /ws/maps/${MAP_NAME} 2>&1 | tail -5" \
            2>/dev/null || echo "[automap] WARN: map_saver failed"
        MAP_SAVED="true"
    fi

    # 3. Kill explore_lite + background
    if [ -n "${EXPLORE_HOST_PID}" ]; then
        kill "${EXPLORE_HOST_PID}" 2>/dev/null || true
    fi
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${CONTAINER_NAME}"; then
        kill_explore_inside_container
    fi

    # 4. Kill the mapping stack
    for pid in "${PIDS[@]}"; do
        kill -INT "${pid}" 2>/dev/null || true
    done
    sleep 2
    for pid in "${PIDS[@]}"; do
        kill -TERM "${pid}" 2>/dev/null || true
    done
    wait 2>/dev/null || true

    (cd "${NAV2_WIFI_DIR}/docker" && docker compose down 2>/dev/null) || true

    echo "[automap] Done."
    if [ -f "${MAP_SAVE_PATH}.yaml" ]; then
        echo "[automap] Map saved: ${MAP_SAVE_PATH}.yaml"
        echo "[automap]          ${MAP_SAVE_PATH}.pgm"
    else
        echo "[automap] WARN: no map file at ${MAP_SAVE_PATH}.yaml"
    fi
}
trap cleanup EXIT INT TERM

# ===========================================================================
# Launch
# ===========================================================================
cd "${NAV2_WIFI_DIR}"
export GO2_HOST_IP GO2_ROBOT_IP GO2_RELAY_DOMAIN_ID

echo "[automap] Launching mapping stack (mapping.sh --3d --nav)..."
./mapping.sh --3d --nav \
    > "${NAV2_WIFI_DIR}/ws/log/automap.log" 2>&1 &
PIDS+=("$!")

# ===========================================================================
# Wait for Nav2 lifecycle to become active
# ===========================================================================
echo "[automap] Waiting up to ${NAV2_READY_TIMEOUT}s for Nav2 lifecycle..."
NAV2_READY="false"
for i in $(seq 1 "${NAV2_READY_TIMEOUT}"); do
    ok=0
    for n in planner_server controller_server bt_navigator; do
        state=$(docker exec -u root "${CONTAINER_NAME}" bash -c \
            "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash 2>/dev/null; \
             ros2 lifecycle get /$n 2>/dev/null" 2>/dev/null || echo "unknown")
        echo "$state" | grep -q "active" && ok=$((ok + 1))
    done
    if [ "${ok}" -eq 3 ]; then
        NAV2_READY="true"
        echo "[automap] Nav2 active after ${i}s."
        break
    fi
    sleep 1
done

if [ "${NAV2_READY}" != "true" ]; then
    echo "[automap] ERROR: Nav2 did not activate within ${NAV2_READY_TIMEOUT}s."
    echo "[automap] Check ${NAV2_WIFI_DIR}/ws/log/automap.log"
    exit 1
fi

# Extra settle time for costmap to populate
sleep 3

# ===========================================================================
# Launch explore_lite (return_to_init=true → roomba behavior)
# ===========================================================================
echo "[automap] Starting explore_lite with return_to_init=true..."

EXPLORE_PARAMS_TMP=$(mktemp /tmp/explore_params_XXXXXX.yaml)
cat > "${EXPLORE_PARAMS_TMP}" << 'EOF'
/**:
  ros__parameters:
    robot_base_frame: base_link
    costmap_topic: /map
    costmap_updates_topic: /map_updates
    visualize: true
    planner_frequency: 1.0
    progress_timeout: 20.0
    potential_scale: 3.0
    orientation_scale: 0.0
    gain_scale: 1.0
    transform_tolerance: 0.3
    min_frontier_size: 0.4
    return_to_init: true
EOF
docker cp "${EXPLORE_PARAMS_TMP}" \
    "${CONTAINER_NAME}:/tmp/explore_params.yaml" 2>/dev/null || true
rm -f "${EXPLORE_PARAMS_TMP}"

docker exec -u root "${CONTAINER_NAME}" bash -c '
    pids=$(ps -eo pid,cmd | grep -E "explore_lite|ros2 run explore" \
           | grep -v grep | awk "{print \$1}")
    if [ -n "$pids" ]; then
        for pid in $pids; do kill -9 "$pid" 2>/dev/null || true; done
    fi
    sleep 1
' 2>/dev/null || true

docker exec -u root "${CONTAINER_NAME}" bash -c \
    "source /opt/ros/humble/setup.bash && \
     source /ws/install/setup.bash 2>/dev/null; \
     exec ros2 run explore_lite explore --ros-args --params-file /tmp/explore_params.yaml" \
    > "${NAV2_WIFI_DIR}/ws/log/explore.log" 2>&1 &
EXPLORE_HOST_PID=$!

# ===========================================================================
# Auto-start exploration — publish true to /explore/resume
# ===========================================================================
echo "[automap] Auto-starting exploration (no button click)..."
for i in $(seq 1 15); do
    if docker exec -u root "${CONTAINER_NAME}" bash -c \
        "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash 2>/dev/null; \
         ros2 topic list 2>/dev/null | grep -q /explore/resume"; then
        break
    fi
    sleep 1
done

docker exec -u root "${CONTAINER_NAME}" bash -c '
    source /opt/ros/humble/setup.bash
    source /ws/install/setup.bash 2>/dev/null
    timeout 5 ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"
' 2>/dev/null || echo "[automap] WARN: failed to auto-start exploration"

echo "[automap] Exploration started. Robot should begin moving."
echo

# ===========================================================================
# Monitor exploration progress
# ===========================================================================
echo "================================================================"
echo " AUTO-MAPPING IN PROGRESS"
echo "================================================================"
echo " Watch the RViz window (opened by mapping.sh) for the map building."
echo " The robot will explore until no frontiers remain, then return to"
echo " its starting position automatically."
echo
echo " Press Ctrl+C at any time to stop early and save the current map."
echo "================================================================"
echo

ELAPSED=0
FRONTIER_EMPTY_COUNT=0

while [ "${ELAPSED}" -lt "${MAX_EXPLORE_TIME}" ]; do
    sleep 5
    ELAPSED=$((ELAPSED + 5))

    # Check if explore_lite still finds frontiers
    frontiers=$(docker exec -u root "${CONTAINER_NAME}" bash -c '
        source /opt/ros/humble/setup.bash
        source /ws/install/setup.bash 2>/dev/null
        timeout 2 ros2 topic echo /explore/frontiers --once 2>/dev/null | \
            grep -c "markers:" || echo "0"
    ' 2>/dev/null || echo "0")

    # Frontiers empty for 3 consecutive checks → assume done
    if [ "${frontiers}" = "0" ]; then
        FRONTIER_EMPTY_COUNT=$((FRONTIER_EMPTY_COUNT + 1))
    else
        FRONTIER_EMPTY_COUNT=0
    fi

    echo "[automap] t=${ELAPSED}s  frontiers_present=${frontiers}  empty_checks=${FRONTIER_EMPTY_COUNT}"

    if [ "${FRONTIER_EMPTY_COUNT}" -ge 3 ]; then
        echo "[automap] No more frontiers found — exploration complete."
        break
    fi
done

if [ "${ELAPSED}" -ge "${MAX_EXPLORE_TIME}" ]; then
    echo "[automap] Max explore time (${MAX_EXPLORE_TIME}s) reached — stopping."
fi

# ===========================================================================
# Wait for robot to return to origin (if return_to_init worked)
# ===========================================================================
echo "[automap] Waiting up to ${RETURN_WAIT_TIMEOUT}s for robot to return home..."
return_start=$(date +%s)
while true; do
    now=$(date +%s)
    [ $((now - return_start)) -ge "${RETURN_WAIT_TIMEOUT}" ] && {
        echo "[automap] Return timeout reached — proceeding to save."
        break
    }

    # Check if /cmd_vel is idle (robot stopped moving)
    hz=$(docker exec -u root "${CONTAINER_NAME}" bash -c \
        "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash 2>/dev/null; \
         timeout 2 ros2 topic echo /cmd_vel --once 2>/dev/null | \
         grep -E 'linear:|angular:' | grep -vE 'x: 0.0|z: 0.0' | wc -l" \
        2>/dev/null || echo "0")

    if [ "${hz}" = "0" ]; then
        # Robot is idle — check pose is near origin
        sleep 3
        break
    fi
    sleep 2
done

# ===========================================================================
# Save the map and exit
# ===========================================================================
echo
echo "[automap] Exploration complete. Saving map..."
mkdir -p "${NAV2_WIFI_DIR}/ws/maps" 2>/dev/null || true

docker exec -u root "${CONTAINER_NAME}" bash -c \
    "source /opt/ros/humble/setup.bash && \
     source /ws/install/setup.bash 2>/dev/null; \
     ros2 run nav2_map_server map_saver_cli -f /ws/maps/${MAP_NAME} 2>&1 | tail -5" \
    2>/dev/null || echo "[automap] WARN: map_saver failed"

MAP_SAVED="true"

echo
echo "================================================================"
echo " AUTO-MAPPING COMPLETE"
echo "================================================================"
if [ -f "${MAP_SAVE_PATH}.yaml" ]; then
    echo " Map saved:"
    echo "   ${MAP_SAVE_PATH}.yaml"
    echo "   ${MAP_SAVE_PATH}.pgm"
else
    echo " Map file check failed. Look in:"
    echo "   ${NAV2_WIFI_DIR}/ws/maps/"
fi
echo
echo " Next session: load this map for autonomous navigation with"
echo "   ./wifi_nav.sh"
echo "================================================================"
echo

# Trigger cleanup (saves map again if needed, kills stack)