#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# system_simulation.sh
#
# Launches the CMU Unity environment and the ROS 2 autonomy stack together.
#
# Mirrors the CMU pattern but adds the go2_integration_pkg integration
# layer so the same repo works for both simulation and real-robot runs.
#
# Requirements:
#   - ROS 2 Humble on Ubuntu 22.04 (KSU workstation)
#   - autonomy_stack_go2 cloned at ~/files/autonomy_stack_go2
#   - Unity environment model at:
#       ~/files/autonomy_stack_go2/src/base_autonomy/vehicle_simulator/
#           mesh/unity/environment/Model.x86_64
#   - This repo built (colcon build)
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"

AUTONOMY_WS="${AUTONOMY_WS:-$HOME/files/autonomy_stack_go2}"
UNITY_BIN="${AUTONOMY_WS}/src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64"
CYCLONEDDS_XML="${WS_ROOT}/config/cyclonedds_ethernet.xml"

# -----------------------------------------------------------------------------
# Sanity checks
# -----------------------------------------------------------------------------
if [ ! -x "${UNITY_BIN}" ]; then
    echo "ERROR: Unity binary not found or not executable:"
    echo "       ${UNITY_BIN}"
    echo ""
    echo "Download the Unity environment model from the CMU README:"
    echo "  https://github.com/jizhang-cmu/autonomy_stack_go2"
    echo ""
    echo "Unzip it into:"
    echo "  ${AUTONOMY_WS}/src/base_autonomy/vehicle_simulator/mesh/unity/"
    echo ""
    echo "Then: chmod +x ${UNITY_BIN}"
    exit 1
fi

if [ ! -f "${AUTONOMY_WS}/install/setup.bash" ]; then
    echo "ERROR: autonomy_stack_go2 not built at ${AUTONOMY_WS}"
    exit 1
fi

if [ ! -f "${WS_ROOT}/install/setup.bash" ]; then
    echo "ERROR: go2_integrated_ws not built. Run: colcon build --symlink-install"
    exit 1
fi

# -----------------------------------------------------------------------------
# Environment
# -----------------------------------------------------------------------------
set +u
source /opt/ros/humble/setup.bash
source "${AUTONOMY_WS}/install/setup.bash"
source "${WS_ROOT}/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${CYCLONEDDS_XML}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "[system_simulation] RMW            = ${RMW_IMPLEMENTATION}"
echo "[system_simulation] CYCLONEDDS_URI = ${CYCLONEDDS_URI}"
echo "[system_simulation] ROS_DOMAIN_ID  = ${ROS_DOMAIN_ID}"
echo "[system_simulation] Unity binary   = ${UNITY_BIN}"

# -----------------------------------------------------------------------------
# 1. Start Unity in the background
# -----------------------------------------------------------------------------
echo "[system_simulation] Starting Unity environment..."
"${UNITY_BIN}" > "${WS_ROOT}/log/unity.log" 2>&1 &
UNITY_PID=$!

cleanup() {
    echo "[system_simulation] Shutting down Unity (pid ${UNITY_PID})..."
    kill -INT "${UNITY_PID}" 2>/dev/null || true
    wait "${UNITY_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[system_simulation] Unity PID = ${UNITY_PID}"
echo "[system_simulation] Unity log = ${WS_ROOT}/log/unity.log"

# -----------------------------------------------------------------------------
# 2. Wait for the Unity ROS-TCP-Endpoint bridge to start
# -----------------------------------------------------------------------------
echo "[system_simulation] Waiting for ROS-TCP-Endpoint..."
TIMEOUT=20
ELAPSED=0
while [ "${ELAPSED}" -lt "${TIMEOUT}" ]; do
    if ros2 node list 2>/dev/null | grep -q "/ros_tcp_endpoint"; then
        echo "[system_simulation] ROS-TCP-Endpoint is up after ${ELAPSED}s."
        break
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
done

if [ "${ELAPSED}" -ge "${TIMEOUT}" ]; then
    echo "[system_simulation] WARN: ROS-TCP-Endpoint did not appear."
    echo "                     Check ${WS_ROOT}/log/unity.log"
fi

# Give Unity a moment to spawn the vehicle and start publishing
sleep 3

# -----------------------------------------------------------------------------
# 3. Verify the sensor topics Unity publishes
# -----------------------------------------------------------------------------
echo "[system_simulation] Checking Unity sensor topics..."
for topic in /utlidar/cloud /utlidar/imu; do
    if ros2 topic list 2>/dev/null | grep -qx "${topic}"; then
        echo "  OK: ${topic}"
    else
        echo "  MISSING: ${topic}"
    fi
done

# -----------------------------------------------------------------------------
# 4. Launch the autonomy stack (blocks until Ctrl+C)
# -----------------------------------------------------------------------------
echo ""
echo "[system_simulation] Starting CMU autonomy stack (RViz will open)..."
echo ""
echo "  Once RViz is up:"
echo "    1. Wait ~10 seconds for the map to initialize"
echo "    2. Use the 'Waypoint' button to send goals"
echo "    3. Or use the control panel on the right"
echo ""

exec ros2 launch vehicle_simulator system_simulation.launch