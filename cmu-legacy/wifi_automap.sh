#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# wifi_automap.sh
#
# Roomba-style autonomous mapping with collision safety.
#
# Safety features (marked SAFETY below):
#   1. Lower max velocity (0.20 m/s) for slower, safer motion
#   2. Larger inflation radius (0.70 m) to keep distance from walls
#   3. Local costmap: track_unknown_space=true (treat unknown as blocked)
#   4. Scan-based emergency stop watchdog: if any lidar return is closer
#      than SAFETY_STOP_DISTANCE, exploration is immediately paused
#   5. Physical remote remains the primary emergency stop
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

# --- SAFETY: proximity thresholds ------------------------------------------
SAFETY_STOP_DISTANCE="${SAFETY_STOP_DISTANCE:-0.45}"    # m — hard stop if closer
SAFETY_WARN_DISTANCE="${SAFETY_WARN_DISTANCE:-0.70}"    # m — log warning if closer
SAFETY_MAX_VEL_X="${SAFETY_MAX_VEL_X:-0.20}"            # m/s — top speed
SAFETY_MAX_VEL_THETA="${SAFETY_MAX_VEL_THETA:-0.50}"    # rad/s — top turn
SAFETY_INFLATION_RADIUS="${SAFETY_INFLATION_RADIUS:-0.70}"  # m — costmap inflation

# --- Timeouts --------------------------------------------------------------
NAV2_READY_TIMEOUT=45
RETURN_WAIT_TIMEOUT=120
MAX_EXPLORE_TIME=600

# --- Parse args ------------------------------------------------------------
ACTION="run"
while [ $# -gt 0 ]; do
    case "$1" in
        --map-name) MAP_NAME="$2"; MAP_SAVE_PATH="${NAV2_WIFI_DIR}/ws/maps/${MAP_NAME}"; shift ;;
        --check)    ACTION="check" ;;
        --clean)    ACTION="clean" ;;
        --help|-h)  sed -n '2,32p' "$0"; exit 0 ;;
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
            for pid in $pids; do kill -9 "$pid" 2>/dev/null || true; done
        fi
        sleep 1
    ' 2>/dev/null || true
}

# SAFETY: publish stop to explore_lite
stop_exploration() {
    docker exec -u root "${CONTAINER_NAME}" bash -c '
        source /opt/ros/humble/setup.bash
        source /ws/install/setup.bash 2>/dev/null
        timeout 3 ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}" 2>/dev/null || true
    ' 2>/dev/null || true
}

do_clean() {
    echo "[clean] Killing local stale processes..."
    pkill -9 -f "rviz2"             2>/dev/null || true
    pkill -9 -f "mapping.sh"        2>/dev/null || true
    pkill -9 -f "pointlio_mapping"  2>/dev/null || true
    pkill -9 -f "go2_cloud_stamp"   2>/dev/null || true
    pkill -9 -f "safety_watchdog"   2>/dev/null || true

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
# SAFETY: pre-flight patch — enforce collision detection in launch params
# ===========================================================================
patch_collision_config() {
    echo "[safety] Enforcing collision detection in the container..."

    docker exec -u root "${CONTAINER_NAME}" bash -c "
        LCTRL=/ws/src/go2_nav2/launch/lidar3d_controller.launch.py
        if [ ! -f \"\$LCTRL\" ]; then
            echo '[safety] WARN: launch file not found at \$LCTRL'
            exit 0
        fi

        # Backup once
        [ -f \"\$LCTRL.orig\" ] || cp \"\$LCTRL\" \"\$LCTRL.orig\"

        # 1. Lower max velocities for safer operation
        sed -i 's/max_vel_x=0\.30/max_vel_x=${SAFETY_MAX_VEL_X}/g'   \"\$LCTRL\" || true
        sed -i 's/max_vel_theta=0\.70/max_vel_theta=${SAFETY_MAX_VEL_THETA}/g' \"\$LCTRL\" || true
        sed -i 's/max_speed_xy=0\.30/max_speed_xy=${SAFETY_MAX_VEL_X}/g' \"\$LCTRL\" || true

        # 2. Increase inflation radius
        sed -i 's/inflation_radius.: 0\.55/inflation_radius\": ${SAFETY_INFLATION_RADIUS}/g' \"\$LCTRL\" || true
        sed -i \"s/inflation_radius'\\] = 0\\.55/inflation_radius'] = ${SAFETY_INFLATION_RADIUS}/g\" \"\$LCTRL\" || true

        # 3. Enable collision detection explicitly
        grep -q 'use_collision_detection' \"\$LCTRL\" || \\
            sed -i \"s/closed_loop=False)/closed_loop=False, use_collision_detection=True)/\" \"\$LCTRL\" || true

        echo '[safety] Launch file updated:'
        grep -E 'max_vel_x|max_vel_theta|inflation_radius|use_collision_detection' \"\$LCTRL\" | head
    " 2>/dev/null || echo "[safety] WARN: patch step had errors (continuing)"
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
echo " Go2 Roomba-style autonomous mapping (SAFETY ON)"
echo "================================================================"
echo "  Action        : ${ACTION}"
echo "  Map name      : ${MAP_NAME}"
echo "  Save to       : ${MAP_SAVE_PATH}"
echo
echo "  SAFETY settings:"
echo "    Max velocity     : ${SAFETY_MAX_VEL_X} m/s  /  ${SAFETY_MAX_VEL_THETA} rad/s"
echo "    Inflation radius : ${SAFETY_INFLATION_RADIUS} m"
echo "    Hard stop        : ${SAFETY_STOP_DISTANCE} m"
echo "    Warn distance    : ${SAFETY_WARN_DISTANCE} m"
echo
echo "  Host IP       : ${GO2_HOST_IP}"
echo "  Robot         : ${GO2_ROBOT_USER}@${GO2_ROBOT_IP}"
echo "  Container     : ${CONTAINER_NAME}"
echo "  Repo          : ${NAV2_WIFI_DIR}"
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
    echo "All checks done."
    exit 0
fi

# ===========================================================================
# Reminder
# ===========================================================================
echo "================================================================"
echo " SAFETY REMINDER"
echo "================================================================"
echo
echo " Physical remote = PRIMARY emergency stop. Keep it in your hand."
echo
echo " Robot relay must be running in FULL mode (separate terminal):"
echo "     ssh ${GO2_ROBOT_USER}@${GO2_ROBOT_IP}"
echo "     export GO2_HOST_IP=${GO2_HOST_IP}"
echo "     bash ~/robot-relay-wifi.sh"
echo
echo " Watch for:  'Relay running: onboard domain 0 -> Wi-Fi domain 64'"
echo
echo " Place the robot on open flat ground with at least 1.5 m clearance"
echo " in every direction before continuing."
echo "================================================================"
echo
read -rp " Press Enter when the robot is standing, the relay is running, and the area is clear..." _
echo

# ===========================================================================
# Cleanup handler
# ===========================================================================
PIDS=()
EXPLORE_HOST_PID=""
WATCHDOG_PID=""
MAP_SAVED="false"

cleanup() {
    echo
    echo "[automap] Shutting down..."

    # SAFETY: publish stop FIRST
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${CONTAINER_NAME}"; then
        echo "[automap] SAFETY: publishing stop to /explore/resume"
        stop_exploration
    fi

    # Kill watchdog
    if [ -n "${WATCHDOG_PID}" ]; then
        kill "${WATCHDOG_PID}" 2>/dev/null || true
    fi

    # Save map
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

    if [ -n "${EXPLORE_HOST_PID}" ]; then
        kill "${EXPLORE_HOST_PID}" 2>/dev/null || true
    fi
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${CONTAINER_NAME}"; then
        kill_explore_inside_container
    fi

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
    [ -f "${MAP_SAVE_PATH}.yaml" ] && {
        echo "[automap] Map saved: ${MAP_SAVE_PATH}.yaml"
        echo "[automap]          ${MAP_SAVE_PATH}.pgm"
    } || echo "[automap] WARN: no map file at ${MAP_SAVE_PATH}.yaml"
}
trap cleanup EXIT INT TERM

# ===========================================================================
# SAFETY: write safety watchdog script (runs inside container)
# ===========================================================================
cat > /tmp/go2_safety_watchdog.py << 'WATCHDOG_EOF'
#!/usr/bin/env python3
"""Safety watchdog: publishes /explore/resume=false if any lidar return
   is closer than SAFETY_STOP_DISTANCE. Also logs warnings below SAFETY_WARN."""
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

STOP_DISTANCE = float(os.environ.get('SAFETY_STOP_DISTANCE', '0.45'))
WARN_DISTANCE = float(os.environ.get('SAFETY_WARN_DISTANCE', '0.70'))

class Watchdog(Node):
    def __init__(self):
        super().__init__('safety_watchdog')
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(Bool, '/explore/resume', 10)
        self.triggered = False
        self.get_logger().info(
            f'safety watchdog: stop<{STOP_DISTANCE}m, warn<{WARN_DISTANCE}m')

    def on_scan(self, msg):
        finite = [r for r in msg.ranges
                  if r > msg.range_min and r < msg.range_max]
        if not finite:
            return
        closest = min(finite)

        if closest < STOP_DISTANCE:
            if not self.triggered:
                self.get_logger().error(
                    f'SAFETY: obstacle at {closest:.2f} m — stopping exploration')
                self.triggered = True
            msg_out = Bool()
            msg_out.data = False
            self.pub.publish(msg_out)
        elif closest < WARN_DISTANCE:
            self.get_logger().warn(f'close obstacle at {closest:.2f} m')
        else:
            if self.triggered:
                self.get_logger().info(
                    f'clear again (min {closest:.2f} m) — not auto-resuming')
            self.triggered = False

def main():
    rclpy.init()
    node = Watchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
WATCHDOG_EOF

# ===========================================================================
# Launch
# ===========================================================================
cd "${NAV2_WIFI_DIR}"
export GO2_HOST_IP GO2_ROBOT_IP GO2_RELAY_DOMAIN_ID

echo "[automap] Launching mapping stack (mapping.sh --3d --nav)..."
./mapping.sh --3d --nav \
    > "${NAV2_WIFI_DIR}/ws/log/automap.log" 2>&1 &
PIDS+=("$!")

# SAFETY: apply launch file patches once the container is running
sleep 5
patch_collision_config

# Wait for Nav2 lifecycle
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
    exit 1
fi
sleep 3

# ===========================================================================
# SAFETY: start the watchdog BEFORE exploration
# ===========================================================================
echo "[automap] SAFETY: starting watchdog..."
docker cp /tmp/go2_safety_watchdog.py \
    "${CONTAINER_NAME}:/tmp/go2_safety_watchdog.py" 2>/dev/null || true

docker exec -u root -e SAFETY_STOP_DISTANCE="${SAFETY_STOP_DISTANCE}" \
                     -e SAFETY_WARN_DISTANCE="${SAFETY_WARN_DISTANCE}" \
    "${CONTAINER_NAME}" bash -c \
    "source /opt/ros/humble/setup.bash && \
     source /ws/install/setup.bash 2>/dev/null; \
     nohup python3 /tmp/go2_safety_watchdog.py \
         > /tmp/safety_watchdog.log 2>&1 &" \
    2>/dev/null || true

sleep 2
if docker exec -u root "${CONTAINER_NAME}" bash -c \
    "ps aux | grep go2_safety_watchdog | grep -v grep" 2>/dev/null | grep -q watchdog; then
    echo "[automap] SAFETY: watchdog running."
else
    echo "[automap] SAFETY: WARN — watchdog not confirmed running."
fi

# ===========================================================================
# Launch explore_lite
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

# Auto-start exploration
echo "[automap] Auto-starting exploration..."
for i in $(seq 1 15); do
    docker exec -u root "${CONTAINER_NAME}" bash -c \
        "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash 2>/dev/null; \
         ros2 topic list 2>/dev/null | grep -q /explore/resume" && break
    sleep 1
done

docker exec -u root "${CONTAINER_NAME}" bash -c '
    source /opt/ros/humble/setup.bash
    source /ws/install/setup.bash 2>/dev/null
    timeout 5 ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"
' 2>/dev/null || echo "[automap] WARN: failed to auto-start"

# ===========================================================================
# Monitor
# ===========================================================================
echo
echo "================================================================"
echo " AUTO-MAPPING IN PROGRESS (SAFETY ON)"
echo "================================================================"
echo " Physical remote is your primary e-stop. Ctrl+C here also stops."
echo " Watchdog will pause exploration if obstacles get within"
echo " ${SAFETY_STOP_DISTANCE} m of the LiDAR."
echo "================================================================"
echo

ELAPSED=0
FRONTIER_EMPTY_COUNT=0
WATCHDOG_TRIGGERS=0
LAST_TRIGGER_SEEN="false"

while [ "${ELAPSED}" -lt "${MAX_EXPLORE_TIME}" ]; do
    sleep 5
    ELAPSED=$((ELAPSED + 5))

    # Check watchdog log for stops
    if docker exec -u root "${CONTAINER_NAME}" bash -c \
        "tail -1 /tmp/safety_watchdog.log 2>/dev/null" 2>/dev/null | \
        grep -q "SAFETY: obstacle"; then
        if [ "${LAST_TRIGGER_SEEN}" = "false" ]; then
            WATCHDOG_TRIGGERS=$((WATCHDOG_TRIGGERS + 1))
            echo "[automap] !!! SAFETY STOP TRIGGERED (${WATCHDOG_TRIGGERS} total) !!!"
            LAST_TRIGGER_SEEN="true"
        fi
    else
        LAST_TRIGGER_SEEN="false"
    fi

    # Count frontiers
    frontiers=$(docker exec -u root "${CONTAINER_NAME}" bash -c '
        source /opt/ros/humble/setup.bash
        source /ws/install/setup.bash 2>/dev/null
        timeout 2 ros2 topic echo /explore/frontiers --once 2>/dev/null | \
            grep -c "markers:" || echo "0"
    ' 2>/dev/null || echo "0")

    if [ "${frontiers}" = "0" ]; then
        FRONTIER_EMPTY_COUNT=$((FRONTIER_EMPTY_COUNT + 1))
    else
        FRONTIER_EMPTY_COUNT=0
    fi

    echo "[automap] t=${ELAPSED}s  frontiers=${frontiers}  safety_stops=${WATCHDOG_TRIGGERS}"

    if [ "${FRONTIER_EMPTY_COUNT}" -ge 3 ]; then
        echo "[automap] No more frontiers — exploration complete."
        break
    fi
done

[ "${ELAPSED}" -ge "${MAX_EXPLORE_TIME}" ] && \
    echo "[automap] Max explore time (${MAX_EXPLORE_TIME}s) reached — stopping."

# ===========================================================================
# Wait for return home
# ===========================================================================
echo "[automap] Waiting up to ${RETURN_WAIT_TIMEOUT}s for robot to return home..."
return_start=$(date +%s)
while true; do
    now=$(date +%s)
    [ $((now - return_start)) -ge "${RETURN_WAIT_TIMEOUT}" ] && {
        echo "[automap] Return timeout — proceeding."
        break
    }
    hz=$(docker exec -u root "${CONTAINER_NAME}" bash -c \
        "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash 2>/dev/null; \
         timeout 2 ros2 topic echo /cmd_vel --once 2>/dev/null | \
         grep -E 'linear:|angular:' | grep -vE 'x: 0.0|z: 0.0' | wc -l" \
        2>/dev/null || echo "0")
    [ "${hz}" = "0" ] && { sleep 3; break; }
    sleep 2
done

# ===========================================================================
# Save map
# ===========================================================================
echo
echo "[automap] Saving map..."
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
echo " Safety stops triggered: ${WATCHDOG_TRIGGERS}"
if [ -f "${MAP_SAVE_PATH}.yaml" ]; then
    echo " Map saved:"
    echo "   ${MAP_SAVE_PATH}.yaml"
    echo "   ${MAP_SAVE_PATH}.pgm"
else
    echo " Map not found. Check ${NAV2_WIFI_DIR}/ws/maps/"
fi
echo "================================================================"