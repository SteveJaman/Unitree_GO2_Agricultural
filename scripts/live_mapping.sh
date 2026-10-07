#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# live_mapping.sh
#
# Minimal live mapping: CMU autonomy stack + map_node.py + RViz.
# No camera, no SLAM, no Nav2, no scan conversion.
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
LOG_DIR="${WS_ROOT}/log"
AUTONOMY_WS="${AUTONOMY_WS:-$HOME/files/autonomy_stack_go2}"
CYCLONEDDS_XML="${WS_ROOT}/config/cyclonedds_ethernet.xml"
RVIZ_CFG="${HOME}/.rviz2/go2_mapping.rviz"

mkdir -p "${LOG_DIR}" "$(dirname "${RVIZ_CFG}")"

# --- Sources ---
set +u
source /opt/ros/humble/setup.bash
source "${AUTONOMY_WS}/install/setup.bash"
source "${WS_ROOT}/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${CYCLONEDDS_XML}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# ---------------------------------------------------------------------------
# Generate the correct RViz config on first run
# Fixed Frame = camera_init (Point-LIO's actual frame)
# QoS = Best Effort (matches Point-LIO)
# ---------------------------------------------------------------------------
if [ ! -f "${RVIZ_CFG}" ]; then
    echo "[mapping] Writing RViz config to ${RVIZ_CFG}"
    cat > "${RVIZ_CFG}" << 'EOF'
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
      Plane Cell Count: 20
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Show Names: true
    - Class: rviz_default_plugins/PointCloud2
      Name: RegisteredScan
      Enabled: true
      Topic:
        Value: /registered_scan
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
        Value: /state_estimation
        Reliability Policy: Best Effort
  Global Options:
    Fixed Frame: camera_init
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
      Distance: 15
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Pitch: 0.785
      Yaw: 0.785
EOF
fi

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
    echo "[mapping] Done."
}
trap cleanup EXIT INT TERM

# ---------------------------------------------------------------------------
# 1. Launch CMU autonomy stack (runs Point-LIO)
# ---------------------------------------------------------------------------
echo "[mapping] Starting CMU autonomy stack..."
ros2 launch vehicle_simulator system_real_robot.launch \
    > "${LOG_DIR}/autonomy.log" 2>&1 &
PIDS+=("$!")

# Wait for Point-LIO to publish
echo "[mapping] Waiting for /state_estimation..."
for i in $(seq 1 60); do
    if ros2 topic list 2>/dev/null | grep -qxF "/state_estimation"; then
        echo "[mapping] /state_estimation is up (after ${i}s)."
        break
    fi
    sleep 1
done

if ! ros2 topic list 2>/dev/null | grep -qxF "/state_estimation"; then
    echo "[mapping] ERROR: /state_estimation did not appear."
    echo "[mapping] Check ${LOG_DIR}/autonomy.log"
    exit 1
fi

# ---------------------------------------------------------------------------
# 2. Launch map_node.py
# ---------------------------------------------------------------------------
echo "[mapping] Starting map_node.py..."
ros2 run go2_integration_pkg map_node.py \
    > "${LOG_DIR}/map_node.log" 2>&1 &
PIDS+=("$!")
sleep 3

if ros2 topic list 2>/dev/null | grep -qxF "/map/occupancy"; then
    echo "[mapping] /map/occupancy is up."
else
    echo "[mapping] WARNING: /map/occupancy did not appear. Check ${LOG_DIR}/map_node.log"
fi

# ---------------------------------------------------------------------------
# 3. Summary
# ---------------------------------------------------------------------------
echo
echo "================================================================"
echo " Live mapping is running."
echo "================================================================"
echo " Drive the robot with the physical remote to build the map."
echo
echo " RViz displays:"
echo "   Occupancy    -> /map/occupancy   (2D grid)"
echo "   MapPoints    -> /map/points      (3D accumulated cloud)"
echo "   RegisteredScan -> /registered_scan"
echo
echo " Press Ctrl+C to stop. The map saves automatically."
echo " Save location: ~/go2_maps/<timestamp>/"
echo "================================================================"
echo

# ---------------------------------------------------------------------------
# 4. Launch RViz in foreground
# ---------------------------------------------------------------------------
rviz2 -d "${RVIZ_CFG}"