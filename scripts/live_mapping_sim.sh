#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# live_mapping_sim.sh
#
# Test live_mapping.sh logic without a robot or Gazebo.
# Uses mock_sim.py to publish fake /registered_scan, /state_estimation, /tf.
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"

LOG_DIR="${WS_ROOT}/log"
RVIZ_CFG="${HOME}/.rviz2/go2_mapping_sim.rviz"
DIAG_FILE="${LOG_DIR}/diagnostics_sim.txt"

mkdir -p "${LOG_DIR}" "$(dirname "${RVIZ_CFG}")"
: > "${DIAG_FILE}"

set +u
source /opt/ros/humble/setup.bash
source "${WS_ROOT}/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
# No CYCLONEDDS_URI — mock runs locally

log() { echo "$@" | tee -a "${DIAG_FILE}"; }

echo "================================"
echo " Go2 Live Mapping (MOCK SIM)"
echo "================================"
echo ""

wait_for_topic() {
    local topic="$1"; local timeout="${2:-30}"; local i=0
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

PIDS=()
cleanup() {
    echo
    echo "[mapping-sim] Shutting down..."
    for pid in "${PIDS[@]}"; do kill -INT "${pid}" 2>/dev/null || true; done
    sleep 2
    for pid in "${PIDS[@]}"; do kill -TERM "${pid}" 2>/dev/null || true; done
    wait 2>/dev/null || true
    echo "[mapping-sim] Done. Diagnostics: ${DIAG_FILE}"
}
trap cleanup EXIT INT TERM

# 1. Mock publisher
log "[1/4] Starting mock_sim.py..."
python3 "${SCRIPT_DIR}/mock_sim.py" \
    > "${LOG_DIR}/mock_sim.log" 2>&1 &
PIDS+=("$!")

log "[1/4] Waiting for /registered_scan..."
if ! wait_for_topic /registered_scan 30; then
    log "  ERROR: mock didn't publish. Check ${LOG_DIR}/mock_sim.log"
    exit 1
fi
log "  /registered_scan is up."

# 2. Detect
log "[2/4] Detecting topics..."
CLOUD_TOPIC=$(detect_cloud_topic || true)
ODOM_TOPIC=$(detect_odom_topic || true)
[ -z "${ODOM_TOPIC}" ] && ODOM_TOPIC="/state_estimation"
log "  cloud topic = ${CLOUD_TOPIC}"
log "  odom topic  = ${ODOM_TOPIC}"

# 3. TF frame + RViz config
log "[3/4] Detecting top TF frame..."
TOP_FRAME=$(detect_top_frame || true)
[ -z "${TOP_FRAME}" ] && TOP_FRAME="map"
log "  top frame   = ${TOP_FRAME}"

log "[3/4] Writing RViz config..."
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

# 4. map_node.py
log "[4/4] Starting map_node.py..."
ros2 run go2_integration_pkg map_node.py --ros-args \
    -p cloud_topic:="${CLOUD_TOPIC}" \
    -p odom_topic:="${ODOM_TOPIC}" \
    > "${LOG_DIR}/map_node_sim.log" 2>&1 &
PIDS+=("$!")

log "[4/4] Waiting for /map/occupancy (max 30s)..."
if ! wait_for_topic /map/occupancy 30; then
    log "  ERROR: /map/occupancy did not appear."
    tail -30 "${LOG_DIR}/map_node_sim.log" | sed 's/^/    /' | tee -a "${DIAG_FILE}"
    exit 1
fi

log ""
log "================================"
log " SIM test running"
log "================================"
log " Fixed Frame : ${TOP_FRAME}"
log " Cloud topic : ${CLOUD_TOPIC}"
log " Odom topic  : ${ODOM_TOPIC}"
log ""
log " Watch the map build in RViz. Ctrl+C to stop."
log "================================"
log ""

rviz2 -d "${RVIZ_CFG}"
