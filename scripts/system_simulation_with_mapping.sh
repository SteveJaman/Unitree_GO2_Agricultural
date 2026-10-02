#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# system_simulation_with_mapping.sh
#
# Complete simulation workflow: Unity + autonomy stack + map node.
# On Ctrl+C, saves map.png + map.ply + mesh.obj to ~/go2_maps/<timestamp>/
#
# Works in three environments:
#   1. CMU autonomy stack with Unity        (/registered_scan)
#   2. CMU autonomy stack without Unity     (/registered_scan)
#   3. Dog directly over DDS                (/utlidar/cloud_deskewed)
#
# The map node auto-detects which topic to use.
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

# -----------------------------------------------------------------------------
# 1. Start Unity if available
# -----------------------------------------------------------------------------
UNITY_PID=""
if [ -x "${UNITY_BIN}" ]; then
    echo "[sim+mapping] Starting Unity environment..."
    "${UNITY_BIN}" > "${WS_ROOT}/log/unity.log" 2>&1 &
    UNITY_PID=$!
    echo "[sim+mapping] Unity PID = ${UNITY_PID}"
    echo "[sim+mapping] Unity log = ${WS_ROOT}/log/unity.log"

    echo "[sim+mapping] Waiting for Unity ROS-TCP-Endpoint..."
    for i in $(seq 1 20); do
        if ros2 node list 2>/dev/null | grep -q ros_tcp_endpoint; then
            echo "[sim+mapping] Endpoint up after ${i}s."
            break
        fi
        sleep 1
    done
    sleep 3
else
    echo "[sim+mapping] Unity binary not found at:"
    echo "               ${UNITY_BIN}"
    echo "[sim+mapping] Skipping Unity launch."
    echo "[sim+mapping] Download the model from the CMU autonomy_stack_go2 README."
    echo ""
fi

cleanup() {
    echo "[sim+mapping] Shutting down..."
    if [ -n "${UNITY_PID}" ] && kill -0 "${UNITY_PID}" 2>/dev/null; then
        kill -INT "${UNITY_PID}" 2>/dev/null || true
    fi
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# -----------------------------------------------------------------------------
# 2. Start autonomy stack if available
# -----------------------------------------------------------------------------
AUTONOMY_STARTED=0
if [ -f "${AUTONOMY_WS}/install/setup.bash" ]; then
    echo "[sim+mapping] Starting CMU autonomy stack in background..."
    ros2 launch vehicle_simulator system_simulation.launch \
        > "${WS_ROOT}/log/autonomy.log" 2>&1 &
    AUTONOMY_PID=$!
    AUTONOMY_STARTED=1
    echo "[sim+mapping] Autonomy PID = ${AUTONOMY_PID}"
    echo "[sim+mapping] Autonomy log = ${WS_ROOT}/log/autonomy.log"

    echo "[sim+mapping] Waiting for Point-LIO output..."
    for i in $(seq 1 30); do
        if ros2 topic list 2>/dev/null | grep -qx "/registered_scan" \
        || ros2 topic list 2>/dev/null | grep -qx "/cloud_registered"; then
            echo "[sim+mapping] Point-LIO output up after ${i}s."
            break
        fi
        sleep 1
    done
else
    echo "[sim+mapping] autonomy_stack_go2 not found. Running without it."
fi

# -----------------------------------------------------------------------------
# 3. Report which cloud/odom topics are available
# -----------------------------------------------------------------------------
echo "[sim+mapping] Available cloud topics:"
for candidate in /registered_scan /cloud_registered /utlidar/cloud_deskewed /utlidar/cloud; do
    if ros2 topic list 2>/dev/null | grep -qx "$candidate"; then
        echo "                 ${candidate}"
    fi
done

echo "[sim+mapping] Available odom topics:"
for candidate in /state_estimation /utlidar/robot_odom /utlidar/robot_pose; do
    if ros2 topic list 2>/dev/null | grep -qx "$candidate"; then
        echo "                 ${candidate}"
    fi
done

# -----------------------------------------------------------------------------
# 4. Start the mapping node (auto-detects topics)
# -----------------------------------------------------------------------------
echo ""
echo "[sim+mapping] Starting map node (auto-detecting topics)."
echo "[sim+mapping] Ctrl+C saves the map."
echo "[sim+mapping] In RViz, add:"
echo "[sim+mapping]   Map         -> /map/occupancy"
echo "[sim+mapping]   PointCloud2 -> /map/points"
echo ""

exec ros2 launch go2_integration_pkg mapping.launch.py