#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# move_forward.sh
#
# Minimal test: send one forward velocity command to the Go2 over DDS.
# Waits 3 seconds, then stops. No SLAM, no mapping, no RViz.
#
# Works over Ethernet and native Wi-Fi DDS. Not for WebRTC.
#
# Requirements:
#   - Ethernet cable (or Wi-Fi) between the VM and Go2
#   - Robot is STANDING (use the physical remote first)
#   - Robot is in Sport Mode
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"

NET_MODE="${NET_MODE:-ethernet}"
case "$NET_MODE" in
  ethernet) XML="${WS_ROOT}/config/cyclonedds_ethernet.xml" ;;
  wireless) XML="${WS_ROOT}/config/cyclonedds_wireless.xml" ;;
  *) echo "Unknown NET_MODE=$NET_MODE (use ethernet|wireless)"; exit 1 ;;
esac

set +u
source /opt/ros/humble/setup.bash
source ~/files/autonomy_stack_go2/install/setup.bash
source "${WS_ROOT}/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${XML}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "[move_forward] NET_MODE       = ${NET_MODE}"
echo "[move_forward] RMW            = ${RMW_IMPLEMENTATION}"
echo "[move_forward] CYCLONEDDS_URI = ${CYCLONEDDS_URI}"
echo "[move_forward] ROS_DOMAIN_ID  = ${ROS_DOMAIN_ID}"

# --- Sanity check ---
case "$NET_MODE" in
  ethernet)
    ip addr show enp0s8 2>/dev/null | grep -q "192.168.123.100" \
      || echo "[move_forward] WARN: enp0s8 missing 192.168.123.100/24"
    if ! ping -c 1 -W 1 192.168.123.18 > /dev/null 2>&1; then
      echo "[move_forward] ERROR: cannot reach Go2 at 192.168.123.18"
      exit 1
    fi
    ;;
  wireless)
    echo "[move_forward] WARN: verify both machines are on the same Wi-Fi subnet"
    ;;
esac

python3 "${WS_ROOT}/src/go2_integration_pkg/go2_integration_pkg/move_forward.py"