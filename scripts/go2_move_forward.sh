#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# go2_move_forward.sh
#
# Minimal test: send ONE forward velocity command to the Go2 over Ethernet
# DDS, wait 3 seconds, then send stop.
#
# No SLAM, no autonomy, no mapping, no RViz. Just the robot walking forward.
#
# Requirements:
#   - Ethernet cable from KSU desktop to Go2
#   - VM Ethernet interface (enp0s8) at 192.168.123.100/24
#   - Go2 Jetson reachable at 192.168.123.18
#   - Robot is STANDING (use the physical remote first)
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
CYCLONEDDS_XML="${WS_ROOT}/config/cyclonedds_wireless.xml"

# --- ROS 2 environment (set +u around source to allow unset ROS vars) ---
set +u
source /opt/ros/humble/setup.bash
source ~/files/autonomy_stack_go2/install/setup.bash
set -u

# --- Middleware ---
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${CYCLONEDDS_XML}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "[go2_move_forward] RMW = ${RMW_IMPLEMENTATION}"
echo "[go2_move_forward] CYCLONEDDS_URI = ${CYCLONEDDS_URI}"

# --- Sanity checks ---
if ! ip addr show enp0s8 2>/dev/null | grep -q "192.168.123.100"; then
    echo "[go2_move_forward] WARN: enp0s8 missing 192.168.123.100/24"
fi

if ! ping -c 1 -W 1 192.168.123.18 > /dev/null 2>&1; then
    echo "[go2_move_forward] ERROR: cannot reach Go2 at 192.168.123.18"
    exit 1
fi

# --- Run the mover ---
python3 "${SCRIPT_DIR}/go2_move_forward.py"
