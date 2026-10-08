#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# system_navigation.sh
#
# Launches the full navigation stack with localization against a saved map.
#
# Usage:
#   ./scripts/system_navigation.sh ~/go2_maps/my_map.yaml
# ---------------------------------------------------------------------------
set -eo pipefail

MAP_FILE="${1:?Usage: $0 <path_to_map.yaml>}"
[ -f "${MAP_FILE}" ] || { echo "Map not found: ${MAP_FILE}"; exit 1; }

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
AUTONOMY_WS="${AUTONOMY_WS:-$HOME/files/autonomy_stack_go2}"
CYCLONEDDS_XML="${WS_ROOT}/config/cyclonedds_ethernet.xml"
mkdir -p "${WS_ROOT}/log"

set +u
source /opt/ros/humble/setup.bash
source "${AUTONOMY_WS}/install/setup.bash"
source "${WS_ROOT}/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${CYCLONEDDS_XML}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "================================================================"
echo " Navigation with localization"
echo " Map: ${MAP_FILE}"
echo "================================================================"

# --- 1. Detect topics ---
CLOUD_TOPIC=""
for candidate in /registered_scan /cloud_registered /utlidar/cloud_deskewed /utlidar/cloud; do
    if ros2 topic list 2>/dev/null | grep -qx "$candidate"; then
        CLOUD_TOPIC="$candidate"
        break
    fi
done

if [ -z "$CLOUD_TOPIC" ]; then
    echo "No point cloud topic found. Start the robot or autonomy stack first."
    exit 1
fi

echo "Using cloud topic: ${CLOUD_TOPIC}"

cleanup() {
    echo "Shutting down..."
    pkill -f pointcloud_to_scan 2>/dev/null || true
    pkill -f camera_relay 2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# --- 2. PointCloud to LaserScan ---
echo "Starting pointcloud_to_scan..."
ros2 run go2_integration_pkg pointcloud_to_scan.py \
    --ros-args -p input_topic:="${CLOUD_TOPIC}" \
    > "${WS_ROOT}/log/pc2scan.log" 2>&1 &

sleep 2

# --- 3. Localization (map_server + AMCL) ---
echo "Starting localization (map_server + AMCL)..."
ros2 launch go2_integration_pkg localization.launch.py \
    map_file:="${MAP_FILE}" \
    > "${WS_ROOT}/log/localization.log" 2>&1 &

sleep 5

# --- 4. Nav2 navigation stack ---
echo "Starting Nav2 navigation stack..."
ros2 launch go2_integration_pkg nav2_bringup.launch.py \
    > "${WS_ROOT}/log/nav2.log" 2>&1 &

sleep 3

# --- 5. Camera ---
if ros2 topic list 2>/dev/null | grep -qx "/camera/image_raw"; then
    echo "Starting camera relay..."
    ros2 run go2_integration_pkg camera_relay.py \
        > "${WS_ROOT}/log/camera_relay.log" 2>&1 &
fi

echo ""
echo "================================================================"
echo " Navigation stack running."
echo ""
echo " In RViz:"
echo "   1. Click '2D Pose Estimate' and drag on the map"
echo "      to set the robot's initial position."
echo "   2. Click '2D Goal Pose' and drag to send the robot."
echo ""
echo " Press Ctrl+C to stop."
echo "================================================================"

rviz2