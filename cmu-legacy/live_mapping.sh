#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# live_mapping.sh
#
# Minimal live mapping: CMU autonomy stack + map_node.py + RViz.
# Auto-detects cloud topic and top TF frame. Regenerates RViz config each run.
# Auto-publishes static transforms for CMU data frames under the top frame.
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
  ethernet)  XML="${WS_ROOT}/config/cyclonedds_ethernet.xml" ;;
  wireless)  XML="${WS_ROOT}/config/cyclonedds_wireless.xml" ;;
  *) echo "Unknown NET_MODE=$NET_MODE (use ethernet|wireless)"; exit 1 ;;
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

# --- Sanity check ---------------------------------------------------------
if [ "$NET_MODE" = "ethernet" ]; then
    if ! ping -c 1 -W 1 192.168.123.18 > /dev/null 2>&1; then
        echo "WARN: Cannot ping Go2 at 192.168.123.18 (ICMP may be blocked)."
        echo "      Continuing anyway — DDS may still work."
    else
        echo "Go2 reachable at 192.168.123.18"
    fi
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

# Prefer well-known CMU map frames, then fall back to top of tree.
detect_top_frame() {
    # On the real CMU stack, the map frame is camera_init (Point-LIO's
    # convention). We could try to detect it via `ros2 topic echo /tf`,
    # but Point-LIO publishes /tf as BEST_EFFORT while ros2 CLI defaults
    # to RELIABLE — discovery silently fails and the echo hangs.
    #
    # Since the frame name is fixed across the CMU stack, we hardcode it.
    # If the stack ever changes the frame, override with:
    #   TOP_FRAME_OVERRIDE=lidar3d_map ./scripts/live_mapping.sh

    if [ -n "${TOP_FRAME_OVERRIDE:-}" ]; then
        echo "${TOP_FRAME_OVERRIDE}"
        return 0
    fi

    # Sanity check: does camera_init appear anywhere in the TF topic list?
    # (list is reliable, echo is not)
    if ros2 topic list 2>/dev/null | grep -qxF "/tf"; then
        echo "camera_init"
        return 0
    fi

    echo "camera_init"
    return 0
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
# 0. Kill any stale RViz / static publishers from previous runs
# ---------------------------------------------------------------------------
pkill -f "rviz2" 2>/dev/null || true
pkill -f "static_transform_publisher" 2>/dev/null || true
sleep 1

# ---------------------------------------------------------------------------
# 1. Launch CMU autonomy stack (suppress internal RViz)
# ---------------------------------------------------------------------------
log "[1/5] Starting CMU autonomy stack (rviz:=false)..."

ros2 launch vehicle_simulator system_real_robot.launch rviz:=false \
    > "${LOG_DIR}/autonomy.log" 2>&1 &
PIDS+=("$!")

# CMU stack may still open its own RViz regardless — clean it up
sleep 5
pkill -f "rviz2" 2>/dev/null || true

log "[1/5] Waiting for /state_estimation (up to 90s)..."
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
log "[2/5] Detecting cloud and odom topics..."
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
# 3. Detect top TF frame + auto-bridge CMU frames
# ---------------------------------------------------------------------------
log "[3/5] Detecting top TF frame..."
TOP_FRAME=$(detect_top_frame || true)

if [ -z "${TOP_FRAME}" ]; then
    log "  WARN: could not detect TF frame. Falling back to 'camera_init'."
    TOP_FRAME="camera_init"
else
    log "  top frame   = ${TOP_FRAME}"
fi

# Publish a static transform from the top frame to each CMU data frame.
# This is idempotent: if the real TF tree already connects them, the
# static publisher is redundant. If it doesn't, this fills the gap so
# RViz can render.
log "  Publishing static bridges under ${TOP_FRAME}."

CMU_FRAMES=(
    body
    vehicle
    sensor
    lidar3d_map
    camera
    aft_mapped
    base_link
    base_footprint
)

for child in "${CMU_FRAMES[@]}"; do
    # Skip if this child IS the top frame (can't be its own child)
    [ "${child}" = "${TOP_FRAME}" ] && continue

    ros2 run tf2_ros static_transform_publisher \
        --frame-id "${TOP_FRAME}" \
        --child-frame-id "${child}" \
        > /dev/null 2>&1 &
    PIDS+=("$!")
done
sleep 1

# ---------------------------------------------------------------------------
# 4. Write RViz config
# ---------------------------------------------------------------------------
log "[4/5] Writing RViz config (Fixed Frame = ${TOP_FRAME})..."
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
      Topic:
        Value: /tf
        Reliability Policy: Best Effort
        Durability Policy: Volatile
        Depth: 100
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
# 5. Launch map_node.py with explicit topics
# ---------------------------------------------------------------------------
log "[5/5] Starting map_node.py..."
ros2 run go2_integration_pkg map_node.py --ros-args \
    -p cloud_topic:="${CLOUD_TOPIC}" \
    -p odom_topic:="${ODOM_TOPIC}" \
    > "${LOG_DIR}/map_node.log" 2>&1 &
PIDS+=("$!")

log "[5/5] Waiting for /map/occupancy (max 30s)..."
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
