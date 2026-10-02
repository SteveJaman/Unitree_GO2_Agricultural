#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# system_real_robot_wireless.sh
#
# Launch the CMU autonomy stack over native Wi-Fi DDS.
# No Zenoh, no WebRTC. Requires a Wi-Fi interface on the Jetson
# (USB dongle) and both machines on the same Wi-Fi subnet.
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
CYCLONEDDS_XML="${WS_ROOT}/config/cyclonedds_wireless.xml"
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

echo "[wireless] RMW            = ${RMW_IMPLEMENTATION}"
echo "[wireless] CYCLONEDDS_URI = ${CYCLONEDDS_URI}"
echo "[wireless] ROS_DOMAIN_ID  = ${ROS_DOMAIN_ID}"
echo "[wireless] Launching autonomy stack over native Wi-Fi DDS..."
echo "[wireless] Remember to STAND THE ROBOT with the physical remote."

exec ros2 launch vehicle_simulator system_real_robot.launch