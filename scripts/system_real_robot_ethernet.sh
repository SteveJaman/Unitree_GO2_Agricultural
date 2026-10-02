#!/bin/bash
# =============================================================================
# system_real_robot_ethernet.sh
#
# Launch the CMU autonomy stack over direct Ethernet to the Go2 Jetson.
#
# Network:
#   VM (enp0s8)      : 192.168.123.100/24
#   Go2 Jetson       : 192.168.123.18/24
#
# No WebRTC, no SDK, no relay. The Jetson publishes /utlidar/cloud and
# /utlidar/imu natively over DDS, and the autonomy stack consumes them
# directly. Control commands go back over the same DDS link via
# /api/sport/request.
# =============================================================================

# NOTE: We do NOT use `set -u` here because ROS 2's setup.bash scripts
# reference undefined variables (AMENT_TRACE_SETUP_FILES, etc.) which
# would abort the script. We DO use -e and -o pipefail.
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"

CYCLONEDDS_XML="${WS_ROOT}/config/cyclonedds_wireless.xml"
AUTONOMY_WS="$HOME/files/autonomy_stack_go2"

# -----------------------------------------------------------------------------
# 1. ROS 2 environment
# -----------------------------------------------------------------------------
# Temporarily allow unset variables (ROS setup scripts rely on this)
set +u
source /opt/ros/humble/setup.bash

if [ -f "${AUTONOMY_WS}/install/setup.bash" ]; then
    source "${AUTONOMY_WS}/install/setup.bash"
else
    echo "[ERROR] autonomy_stack_go2 install not found at ${AUTONOMY_WS}"
    exit 1
fi
set -u

# -----------------------------------------------------------------------------
# 2. Middleware
# -----------------------------------------------------------------------------
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${CYCLONEDDS_XML}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# -----------------------------------------------------------------------------
# 3. Sanity checks
# -----------------------------------------------------------------------------
if [ ! -f "${CYCLONEDDS_XML}" ]; then
    echo "[ERROR] CycloneDDS XML not found: ${CYCLONEDDS_XML}"
    exit 1
fi

if ! ip addr show enp0s8 2>/dev/null | grep -q "192.168.123.100"; then
    echo "[WARN] enp0s8 does not have 192.168.123.100/24 assigned."
    echo "       Run: sudo ip addr add 192.168.123.100/24 dev enp0s8"
fi

if ! ping -c 1 -W 1 192.168.123.18 > /dev/null 2>&1; then
    echo "[WARN] Cannot ping Go2 Jetson at 192.168.123.18."
    echo "       Check Ethernet cable and adapter bridging."
fi

echo "[system_real_robot_ethernet] RMW            = ${RMW_IMPLEMENTATION}"
echo "[system_real_robot_ethernet] CYCLONEDDS_URI = ${CYCLONEDDS_URI}"
echo "[system_real_robot_ethernet] ROS_DOMAIN_ID  = ${ROS_DOMAIN_ID}"
echo "[system_real_robot_ethernet] Launching autonomy stack..."

# -----------------------------------------------------------------------------
# 4. Launch
# -----------------------------------------------------------------------------
ros2 launch vehicle_simulator system_real_robot.launch
