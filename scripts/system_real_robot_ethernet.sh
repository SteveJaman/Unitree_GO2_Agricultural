#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# system_real_robot_ethernet.sh
#
# Launch the CMU autonomy stack over direct Ethernet to the Go2 Jetson.
#
# Network:
#   VM (enp0s8) : 192.168.123.100/24
#   Go2 Jetson  : 192.168.123.18/24
#
# No WebRTC, no SDK, no relay. The Jetson publishes /utlidar/cloud and
# /utlidar/imu natively over DDS, and the autonomy stack consumes them.
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
CYCLONEDDS_XML="${WS_ROOT}/config/cyclonedds_ethernet.xml"
AUTONOMY_WS="$HOME/files/autonomy_stack_go2"

[ -f "${CYCLONEDDS_XML}" ] || { echo "ERROR: ${CYCLONEDDS_XML} not found"; exit 1; }
[ -d "${AUTONOMY_WS}/install" ] || { echo "ERROR: ${AUTONOMY_WS} not built"; exit 1; }

set +u
source /opt/ros/humble/setup.bash
source "${AUTONOMY_WS}/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${CYCLONEDDS_XML}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# --- Sanity checks ---
if ! ip addr show enp0s8 2>/dev/null | grep -q "192.168.123.100"; then
  echo "[ethernet] WARN: enp0s8 missing 192.168.123.100/24"
  echo "[ethernet]       run: sudo ip addr add 192.168.123.100/24 dev enp0s8"
fi

if ! ping -c 1 -W 1 192.168.123.18 > /dev/null 2>&1; then
  echo "[ethernet] WARN: cannot reach Jetson at 192.168.123.18"
fi

echo "[ethernet] RMW            = ${RMW_IMPLEMENTATION}"
echo "[ethernet] CYCLONEDDS_URI = ${CYCLONEDDS_URI}"
echo "[ethernet] ROS_DOMAIN_ID  = ${ROS_DOMAIN_ID}"
echo "[ethernet] Launching autonomy stack..."
echo "[ethernet] Remember to STAND THE ROBOT with the physical remote."

exec ros2 launch vehicle_simulator system_real_robot.launch