#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# system_simulation_with_mapping.sh
#
# Complete simulation workflow: Unity + autonomy stack + map node.
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
[ -x "${UNITY_BIN}" ]       || { echo "Unity not found: ${UNITY_BIN}"; exit 1; }
[ -f "${WS_ROOT}/install/setup.bash" ] || { echo "Run colcon build first"; exit 1; }

set +u
source /opt/ros/humble/setup.bash
source "${AUTONOMY_WS}/install/setup.bash"
source "${WS_ROOT}/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${CYCLONEDDS_XML}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "[sim+mapping] Starting Unity environment..."
"${UNITY_BIN}" > "${WS_ROOT}/log/unity.log" 2>&1 &
UNITY_PID=$!

cleanup() {
    echo "[sim+mapping] Shutting down..."
    kill -INT "${UNITY_PID}" 2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[sim+mapping] Waiting for Unity ROS-TCP-Endpoint..."
for i in $(seq 1 20); do
    ros2 node list 2>/dev/null | grep -q ros_tcp_endpoint && break
    sleep 1
done
sleep 3

# Detect the Point-LIO output topic name (varies by build)
if ros2 topic list 2>/dev/null | grep -qx "/registered_scan"; then
    CLOUD_TOPIC="/registered_scan"
else
    CLOUD_TOPIC="/cloud_registered"
fi
echo "[sim+mapping] Using cloud topic: ${CLOUD_TOPIC}"

echo "[sim+mapping] Starting autonomy stack..."
ros2 launch vehicle_simulator system_simulation.launch \
    > "${WS_ROOT}/log/autonomy.log" 2>&1 &

echo "[sim+mapping] Waiting for Point-LIO output..."
for i in $(seq 1 30); do
    ros2 topic list 2>/dev/null | grep -qx "${CLOUD_TOPIC}" && break
    sleep 1
done

echo ""
echo "[sim+mapping] Starting map node. Ctrl+C saves the map."
echo "[sim+mapping] In RViz, add Map -> /map/occupancy"
echo ""

exec ros2 launch go2_integration_pkg mapping.launch.py \
    cloud_topic:="${CLOUD_TOPIC}"