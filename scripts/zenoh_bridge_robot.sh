#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# zenoh_bridge_robot.sh
#
# Runs ON THE GO2 JETSON, via SSH.
#
# Bridges the Jetson's local CycloneDDS domain 0 to Zenoh, listens on
# TCP/7447, and exposes the robot's topics to remote Zenoh clients.
#
# Prerequisites on the Jetson:
#   - zenoh-bridge-ros2dds installed
#     sudo apt install zenoh-bridge-ros2dds
#   - A working Wi-Fi interface (USB dongle or travel router)
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
CONFIG="${WS_ROOT}/config/zenoh_robot_config.json5"

[ -f "${CONFIG}" ] || { echo "ERROR: ${CONFIG} not found"; exit 1; }

# Source ROS 2 (the Jetson runs Foxy)
set +u
source /opt/ros/foxy/setup.bash
set -u

# DDS stays local to the Jetson — domain 0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

echo "[zenoh_robot] RMW           = ${RMW_IMPLEMENTATION}"
echo "[zenoh_robot] ROS_DOMAIN_ID = ${ROS_DOMAIN_ID}"
echo "[zenoh_robot] Config        = ${CONFIG}"
echo "[zenoh_robot] Listening on TCP/7447 for Zenoh clients"

exec zenoh-bridge-ros2dds -c "${CONFIG}"