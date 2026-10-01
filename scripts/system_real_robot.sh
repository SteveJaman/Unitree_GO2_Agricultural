#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# system_real_robot.sh
#
# Orchestration wrapper for the WebRTC Go2 + CMU autonomy stack.
#
# It performs, in order:
#   1. Source ROS 2 Humble.
#   2. Source the integration workspace overlay.
#   3. Export RMW / CycloneDDS / WebRTC credentials.
#   4. Launch the WebRTC driver in the background.
#   5. Wait for /point_cloud2 to appear (up to 30 s).
#   6. Launch the integration relay + autonomy stack.
#   7. On exit, kill the WebRTC driver.
# ---------------------------------------------------------------------------
set -euo pipefail

# ------------------------------------------------------------------
# 0. Paths and constants
# ------------------------------------------------------------------
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"

CYCLONEDDS_XML="${WS_ROOT}/config/cyclonedds_wireless.xml"

# ------------------------------------------------------------------
# 1. ROS 2 environment
# ------------------------------------------------------------------
source /opt/ros/humble/setup.bash

if [ -f "${WS_ROOT}/install/setup.bash" ]; then
    source "${WS_ROOT}/install/setup.bash"
else
    echo "[system_real_robot] ERROR: ${WS_ROOT}/install/setup.bash not found."
    echo "                     Run:  colcon build   in ${WS_ROOT}"
    exit 1
fi

# ------------------------------------------------------------------
# 2. Middleware / network configuration
# ------------------------------------------------------------------
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${CYCLONEDDS_XML}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# ------------------------------------------------------------------
# 3. Go2 WebRTC credentials  (EDIT THESE)
# ------------------------------------------------------------------
export ROBOT_IP="10.42.0.29"
export AES_KEY="1f2da7dce4745ef65113b925e24a9167"
export ROBOT_AES_KEY="1f2da7dce4745ef65113b925e24a9167"
export CONN_TYPE="webrtc"

# Optional: map save for the SDK
export MAP_NAME="${MAP_NAME:-3d_map}"
export MAP_SAVE="${MAP_SAVE:-true}"

echo "[system_real_robot] RMW            = ${RMW_IMPLEMENTATION}"
echo "[system_real_robot] CYCLONEDDS_URI = ${CYCLONEDDS_URI}"
echo "[system_real_robot] ROS_DOMAIN_ID  = ${ROS_DOMAIN_ID}"
echo "[system_real_robot] ROBOT_IP       = ${ROBOT_IP}"

# ------------------------------------------------------------------
# 4. Sanity check: cyclonedds XML must exist
# ------------------------------------------------------------------
if [ ! -f "${CYCLONEDDS_XML}" ]; then
    echo "[system_real_robot] ERROR: CycloneDDS XML not found at ${CYCLONEDDS_XML}"
    exit 1
fi

# ------------------------------------------------------------------
# 5. Start the WebRTC driver in the background
# ------------------------------------------------------------------
echo "[system_real_robot] Starting go2_robot_sdk (WebRTC driver)..."

ros2 launch go2_robot_sdk robot.launch.py \
    rviz2:=false \
    nav2:=false \
    slam:=false \
    foxglove:=false \
    joystick:=false \
    teleop:=false \
    > "${WS_ROOT}/log/go2_driver.log" 2>&1 &

DRIVER_PID=$!
echo "[system_real_robot] go2_robot_sdk PID = ${DRIVER_PID}"
echo "[system_real_robot] Driver log         = ${WS_ROOT}/log/go2_driver.log"

# ------------------------------------------------------------------
# 6. Cleanup on exit
# ------------------------------------------------------------------
cleanup() {
    echo "[system_real_robot] Shutting down..."
    if kill -0 "${DRIVER_PID}" 2>/dev/null; then
        kill -INT "${DRIVER_PID}" 2>/dev/null || true
        wait "${DRIVER_PID}" 2>/dev/null || true
    fi
    echo "[system_real_robot] Done."
}
trap cleanup EXIT INT TERM

# ------------------------------------------------------------------
# 7. Wait for /point_cloud2 to appear (up to 30 s)
# ------------------------------------------------------------------
echo "[system_real_robot] Waiting for /point_cloud2 to appear..."
TIMEOUT=30
ELAPSED=0
while [ "${ELAPSED}" -lt "${TIMEOUT}" ]; do
    if ros2 topic list 2>/dev/null | grep -qx "/point_cloud2"; then
        echo "[system_real_robot] /point_cloud2 is up after ${ELAPSED}s."
        break
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
done

if [ "${ELAPSED}" -ge "${TIMEOUT}" ]; then
    echo "[system_real_robot] WARNING: /point_cloud2 did not appear in ${TIMEOUT}s."
    echo "                     Check ${WS_ROOT}/log/go2_driver.log"
    echo "                     Continuing anyway; relay will connect when it appears."
fi

# ------------------------------------------------------------------
# 8. Launch the integration layer + autonomy stack
# ------------------------------------------------------------------
echo "[system_real_robot] Starting integration relay + autonomy stack..."

ros2 launch go2_integration_pkg integrated_robot.launch.py &

# Give the relay a moment to attach to /point_cloud2
sleep 2

# Launch the autonomy stack (this blocks until Ctrl+C)
ros2 launch vehicle_simulator system_real_robot.launch