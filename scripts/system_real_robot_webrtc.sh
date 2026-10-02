#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# system_real_robot_webrtc.sh
#
# Launch the CMU autonomy stack over WebRTC (legacy fallback).
#
# Requires that go2_robot_sdk is already running in a separate terminal,
# publishing /point_cloud2 and /imu/data. This script starts the relay
# nodes and then the autonomy stack.
#
# Order of operations:
#   Terminal 1: ros2 launch go2_robot_sdk robot.launch.py ...
#   Terminal 2: ./scripts/system_real_robot_webrtc.sh   (this script)
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
CYCLONEDDS_XML="${WS_ROOT}/config/cyclonedds_wireless.xml"
AUTONOMY_WS="$HOME/files/autonomy_stack_go2"

[ -f "${CYCLONEDDS_XML}" ] || { echo "ERROR: ${CYCLONEDDS_XML} not found"; exit 1; }
[ -d "${WS_ROOT}/install" ] || { echo "ERROR: ${WS_ROOT} not built"; exit 1; }
[ -d "${AUTONOMY_WS}/install" ] || { echo "ERROR: ${AUTONOMY_WS} not built"; exit 1; }

set +u
source /opt/ros/humble/setup.bash
source "${AUTONOMY_WS}/install/setup.bash"
source "${WS_ROOT}/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${CYCLONEDDS_XML}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "[webrtc] RMW            = ${RMW_IMPLEMENTATION}"
echo "[webrtc] CYCLONEDDS_URI = ${CYCLONEDDS_URI}"
echo "[webrtc] ROS_DOMAIN_ID  = ${ROS_DOMAIN_ID}"
echo "[webrtc] Starting integration relays in the background..."

ros2 launch go2_integration_pkg integrated_robot.launch.py &
RELAY_PID=$!
sleep 2

cleanup() {
  echo "[webrtc] Shutting down relays..."
  kill -INT "${RELAY_PID}" 2>/dev/null || true
  wait "${RELAY_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[webrtc] Launching autonomy stack..."
echo "[webrtc] Remember to STAND THE ROBOT with the physical remote."

ros2 launch vehicle_simulator system_real_robot.launch