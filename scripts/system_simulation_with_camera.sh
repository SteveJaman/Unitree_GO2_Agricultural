#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# system_simulation_with_camera.sh
#
# Complete simulation workflow: Unity + autonomy stack + mapping + camera.
#
# Launches:
#   1. Unity environment (background)
#   2. CMU autonomy stack (background)
#   3. pointcloud_to_scan (background)
#   4. map_node (background)
#   5. slam_toolbox (background, for mapping)
#   6. RViz (foreground)
#
# On Ctrl+C, saves map.png + map.ply + mesh.obj to ~/go2_maps/<timestamp>/
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
AUTONOMY_WS="${AUTONOMY_WS:-$HOME/files/autonomy_stack_go2}"
UNITY_BIN="${AUTONOMY_WS}/src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64"
CYCLONEDDS_XML="${WS_ROOT}/config/cyclonedds_ethernet.xml"
mkdir -p "${WS_ROOT}/log"

[ -f "${CYCLONEDDS_XML}" ]  || { echo "Missing ${CYCLONEDDS_XML}"; exit 1; }
[ -f "${WS_ROOT}/install/setup.bash" ] || { echo "Run colcon build first"; exit 1; }

set +u
source /opt/ros/humble/setup.bash
source "${AUTONOMY_WS}/install/setup.bash"
source "${WS_ROOT}/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${CYCLONEDDS_XML}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "================================================================"
echo " Full simulation with camera + mapping + navigation"
echo "================================================================"

# --- 1. Unity ---
if [ -x "${UNITY_BIN}" ]; then
    echo "[1/6] Starting Unity environment..."
    "${UNITY_BIN}" > "${WS_ROOT}/log/unity.log" 2>&1 &
    UNITY_PID=$!
    for i in $(seq 1 20); do
        ros2 node list 2>/dev/null | grep -q ros_tcp_endpoint && break
        sleep 1
    done
    sleep 3
else
    echo "[1/6] Unity not found. Continuing without it."
    UNITY_PID=""
fi

cleanup() {
    echo "Shutting down..."
    [ -n "${UNITY_PID}" ] && kill -INT "${UNITY_PID}" 2>/dev/null || true
    pkill -f pointcloud_to_scan 2>/dev/null || true
    pkill -f map_node 2>/dev/null || true
    pkill -f async_slam_toolbox 2>/dev/null || true
    pkill -f camera_relay 2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# --- 2. Autonomy stack ---
echo "[2/6] Starting CMU autonomy stack..."
ros2 launch vehicle_simulator system_simulation.launch \
    > "${WS_ROOT}/log/autonomy.log" 2>&1 &

for i in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | grep -qE "/registered_scan|/cloud_registered"; then
        break
    fi
    sleep 1
done

# --- 3. Detect topics ---
CLOUD_TOPIC=""
for candidate in /registered_scan /cloud_registered /utlidar/cloud_deskewed /utlidar/cloud; do
    if ros2 topic list 2>/dev/null | grep -qx "$candidate"; then
        CLOUD_TOPIC="$candidate"
        break
    fi
done
echo "[3/6] Using cloud topic: ${CLOUD_TOPIC}"

# --- 4. PointCloud to LaserScan ---
echo "[4/6] Starting pointcloud_to_scan..."
ros2 run go2_integration_pkg pointcloud_to_scan.py \
    --ros-args -p input_topic:="${CLOUD_TOPIC}" \
    > "${WS_ROOT}/log/pc2scan.log" 2>&1 &

sleep 2

# --- 5. Map node ---
echo "[5/6] Starting map_node..."
ros2 run go2_integration_pkg map_node.py \
    > "${WS_ROOT}/log/map_node.log" 2>&1 &

# --- 6. SLAM Toolbox ---
echo "[6/6] Starting slam_toolbox (online mapping)..."
ros2 launch go2_integration_pkg slam_mapping.launch.py \
    > "${WS_ROOT}/log/slam.log" 2>&1 &

sleep 3

# --- Camera (if available) ---
if ros2 topic list 2>/dev/null | grep -qx "/camera/image_raw"; then
    echo "[+] Camera topic detected. Starting relay..."
    ros2 run go2_integration_pkg camera_relay.py \
        > "${WS_ROOT}/log/camera_relay.log" 2>&1 &
fi

echo ""
echo "================================================================"
echo " All systems running."
echo ""
echo " In RViz, add these displays:"
echo "   Map          -> /map"
echo "   LaserScan    -> /scan"
echo "   PointCloud2  -> /map/points"
echo "   Map          -> /map/occupancy"
echo "   Image        -> /camera/image_raw_relayed"
echo "   TF           -> /tf, /tf_static"
echo ""
echo " To save the map:"
echo "   ros2 run nav2_map_server map_saver_cli -f ~/go2_maps/my_map"
echo ""
echo " Press Ctrl+C to stop."
echo "================================================================"

# Open RViz in foreground
rviz2 -d "${AUTONOMY_WS}/install/far_planner/share/far_planner/rviz/default.rviz" 2>/dev/null || rviz2