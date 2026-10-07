#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# live_mapping.sh
#
# Minimal live mapping: CMU autonomy stack + map_node.py + RViz.
# Auto-detects cloud topic and top TF frame. Regenerates RViz config each run.
#
# What you get:
#   /map/occupancy  — 2D log-odds occupancy grid
#   /map/points     — accumulated 3D point cloud
#
# On Ctrl+C: saves map.png, map.ply, mesh.obj to ~/go2_maps/<timestamp>/
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"

NET_MODE="${NET_MODE:-ethernet}"

case "$NET_MODE" in
  ethernet)
    XML="${WS_ROOT}/config/cyclonedds_ethernet.xml"
    ;;
  wireless)
    XML="${WS_ROOT}/config/cyclonedds_wireless.xml"
    ;;
  *)
    echo "Unknown NET_MODE=$NET_MODE (use ethernet|wireless)"
    exit 1
    ;;
esac

AUTONOMY_WS="${AUTONOMY_WS:-$HOME/files/autonomy_stack_go2}"
LOG_DIR="${WS_ROOT}/log"
RVIZ_CFG="${HOME}/.rviz2/go2_mapping.rviz"
DIAG_FILE="${LOG_DIR}/diagnostics.txt"

mkdir -p "${LOG_DIR}" "$(dirname "${RVIZ_CFG}")"
: > "${DIAG_FILE}"

# --- ROS 2 environment -----------------------------------------------------
set +u
source /opt/ros/humble/setup.bash
source "${AUTONOMY_WS}/install/setup.bash"
source "${WS_ROOT}/install/setup.bash"
set -u

# --- CycloneDDS ------------------------------------------------------------
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${XML}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

log() { echo "$@" | tee -a "${DIAG_FILE}"; }

echo "================================"
echo " Go2 Live Mapping"
echo "================================"
echo "Network: ${NET_MODE}"
echo "DDS:     ${CYCLONEDDS_URI}"
echo ""

# --- Sanity check (matches move_forward.sh pattern) ------------------------
if [ "$NET_MODE" = "ethernet" ]; then
    if ! ping -c 1 -W 1 192.168.123.18 > /dev/null 2>&1; then
        echo "ERROR: Cannot reach Go2 at 192.168.123.18"
        exit 1
    fi
    echo "Go2 reachable at 192.168.123.18"
fi

echo ""

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
wait_for_topic() {
    local topic="$1"; local timeout="${2:-90}"; local i=0
    while [ "${i}" -lt "${timeout}" ]; do
        ros2 topic list 2>/dev/null | grep -qxF "${topic}" && return 0
        sleep 1; i=$((i + 1))
    done
    return 1
}

detect_cloud_topic() {
    for t in /registered_scan /cloud_registered /cloud_registered_body \
             /utlidar/cloud_deskewed /utlidar/cloud; do
        ros2 topic list 2>/dev/null | grep -qxF "$t" && { echo "$t"; return 0; }
    done
    return 1
}

detect_odom_topic() {
    for t in /state_estimation /utlidar/robot_odom /utlidar/robot_pose; do
        ros2 topic list 2>/dev/null | grep -qxF "$t" && { echo "$t"; return 0; }
    done
    return 1
}

detect_top_frame() {
    local frames
    frames=$(timeout 3 ros2 topic echo /tf_static --once 2>/dev/null \
        | grep -E "frame_id|child_frame_id" \
        | sed -E 's/.*:\s*"?([^"]*)"?/\1/' | tr -d ' ')
    if [ -z "$frames" ]; then
        frames=$(timeout 3 ros2 topic echo /tf --once 2>/dev/null \
            | grep -E "frame_id|child_frame_id" \
            | sed -E 's/.*:\s*"?([^"]*)"?/\1/' | tr -d ' ')
    fi
    [ -z "$frames" ] && return 1
    local parents children
    parents=$(echo "$frames" | awk 'NR%2==1')
    children=$(echo "$frames" | awk 'NR%2==0')
    while read -r p; do
        echo "$children" | grep -qxF "$p" || { echo "$p"; return 0; }
    done <<< "$parents"
    return 1
}

# ---------------------------------------------------------------------------
# Cleanup on Ctrl+C
# ---------------------------------------------------------------------------
PIDS=()
cleanup() {
    echo
    echo "[mapping] Shutting down..."
    for pid in "${PIDS[@]}"; do
        kill -INT "${pid}" 2>/dev/null || true
    done
    sleep 2
    for pid in "${PIDS[@]}"; do
        kill -TERM "${pid}" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    echo "[mapping] Done. Diagnostics: ${DIAG_FILE}"
}
trap cleanup EXIT INT TERM

# ---------------------------------------------------------------------------
# 1. Launch CMU autonomy stack
# ---------------------------------------------------------------------------
log "[1/4] Starting CMU autonomy stack..."
ros2 launch vehicle_simulator system_real_robot.launch \
    > "${LOG_DIR}/autonomy.log" 2>&1 &
PIDS+=("$!")

log "[1/4] Waiting for /state_estimation (up to 90s)..."
if ! wait_for_topic /state_estimation 90; then
    log "  ERROR: /state_estimation did not appear."
    log "  Check ${LOG_DIR}/autonomy.log"
    log "  If stuck at 'IMU Initializing: 100.0%', fix use_sim_time:"
    log "    grep use_sim_time ${AUTONOMY_WS}/src/slam/point_lio_unilidar/config/utlidar.yaml"
    exit 1
fi
log "  /state_estimation is up."

# ---------------------------------------------------------------------------
# 2. Detect topics
# ---------------------------------------------------------------------------
log "[2/4] Detecting cloud and odom topics..."
CLOUD_TOPIC=$(detect_cloud_topic || true)
ODOM_TOPIC=$(detect_odom_topic || true)

if [ -z "${CLOUD_TOPIC}" ]; then
    log "  ERROR: no cloud topic found. Available topics:"
    ros2 topic list 2>/dev/null | sed 's/^/    /' | tee -a "${DIAG_FILE}"
    exit 1
fi

[ -z "${ODOM_TOPIC}" ] && ODOM_TOPIC="/state_estimation"

log "  cloud topic = ${CLOUD_TOPIC}"
log "  odom topic  = ${ODOM_TOPIC}"

# ---------------------------------------------------------------------------
# 3. Detect top TF frame + write fresh RViz config
# ---------------------------------------------------------------------------
log "[3/4] Detecting top TF frame..."
TOP_FRAME=$(detect_top_frame || true)

if [ -z "${TOP_FRAME}" ]; then
    log "  WARN: could not detect TF frame. Dumping raw /tf and /tf_static:"
    timeout 3 ros2 topic echo /tf --once 2>/dev/null \
        | head -20 | sed 's/^/    /' | tee -a "${DIAG_FILE}" || true
    timeout 3 ros2 topic echo /tf_static --once 2>/dev/null \
        | head -20 | sed 's/^/    /' | tee -a "${DIAG_FILE}" || true
    log "  Falling back to 'camera_init'."
    TOP_FRAME="camera_init"
else
    log "  top frame   = ${TOP_FRAME}"
fi

log "[3/4] Writing RViz config (Fixed Frame = ${TOP_FRAME})..."
cat > "${RVIZ_CFG}" << EOF
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Cell Size: 1
      Plane Cell Count: 30
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Show Names: true
    - Class: rviz_default_plugins/PointCloud2
      Name: RegisteredScan
      Enabled: true
      Topic:
        Value: ${CLOUD_TOPIC}
        Reliability Policy: Best Effort
        Durability Policy: Volatile
        History Policy: Keep Last
        Depth: 5
      Style: Points
      Size (Pixels): 2
    - Class: rviz_default_plugins/PointCloud2
      Name: MapPoints
      Enabled: true
      Topic:
        Value: /map/points
        Reliability Policy: Best Effort
        Durability Policy: Volatile
        History Policy: Keep Last
        Depth: 5
      Style: Points
      Size (Pixels): 2
    - Class: rviz_default_plugins/Map
      Name: Occupancy
      Enabled: true
      Topic:
        Value: /map/occupancy
        Reliability Policy: Reliable
        Durability Policy: Transient Local
        History Policy: Keep Last
        Depth: 1
      Color Scheme: map
      Alpha: 0.7
    - Class: rviz_default_plugins/Odometry
      Name: StateEstimation
      Enabled: true
      Topic:
        Value: ${ODOM_TOPIC}
        Reliability Policy: Best Effort
  Global Options:
    Fixed Frame: ${TOP_FRAME}
    Background Color: 48; 48; 48
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/SetInitialPose
      Topic: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic: /goal_pose
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 20
      Focal Point: {X: 0, Y: 0, Z: 0}
      Pitch: 0.785
      Yaw: 0.785
EOF

# ---------------------------------------------------------------------------
# 4. Launch map_node.py with explicit topics
# ---------------------------------------------------------------------------
log "[4/4] Starting map_node.py..."
ros2 run go2_integration_pkg map_node.py --ros-args \
    -p cloud_topic:="${CLOUD_TOPIC}" \
    -p odom_topic:="${ODOM_TOPIC}" \
    > "${LOG_DIR}/map_node.log" 2>&1 &
PIDS+=("$!")

log "[4/4] Waiting for /map/occupancy (max 30s)..."
if ! wait_for_topic /map/occupancy 30; then
    log "  ERROR: /map/occupancy did not appear."
    log "  map_node.log tail:"
    tail -30 "${LOG_DIR}/map_node.log" | sed 's/^/    /' | tee -a "${DIAG_FILE}"
    exit 1
fi

log ""
log "================================"
log " Live mapping running"
log "================================"
log " Fixed Frame : ${TOP_FRAME}"
log " Cloud topic : ${CLOUD_TOPIC}"
log " Odom topic  : ${ODOM_TOPIC}"
log ""
log " Drive the robot with the physical remote."
log " Press Ctrl+C to save the map."
log " Save location: ~/go2_maps/<timestamp>/"
log " Diagnostics: ${DIAG_FILE}"
log "================================"
log ""

rviz2 -d "${RVIZ_CFG}"