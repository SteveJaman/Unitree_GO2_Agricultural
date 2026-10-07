#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"

NET_MODE="${NET_MODE:-ethernet}"

case "$NET_MODE" in
  ethernet)
    XML="${WS_ROOT}/config/cyclonedds_ethernet.xml"
    ;;
  wireless)
    XML="${WS_ROOT}/config/cyclonedds_wireless.xml"
    ;;
  *)
    echo "Unknown NET_MODE=$NET_MODE (use ethernet|wireless)"
    exit 1
    ;;
esac

# ROS 2 environment
set +u
source /opt/ros/humble/setup.bash
source ~/files/autonomy_stack_go2/install/setup.bash
source "${WS_ROOT}/install/setup.bash"
set -u

# CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${XML}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "================================"
echo " Go2 Movement"
echo "================================"
echo "Network: ${NET_MODE}"
echo "DDS:     ${CYCLONEDDS_URI}"
echo ""

# Ethernet sanity check
if [ "$NET_MODE" = "ethernet" ]; then
    if ! ping -c 1 -W 1 192.168.123.18 > /dev/null 2>&1; then
        echo "ERROR: Cannot reach Go2 at 192.168.123.18"
        exit 1
    fi

    echo "Go2 reachable at 192.168.123.18"
fi

echo ""

python3 "${WS_ROOT}/src/go2_integration_pkg/go2_integration_pkg/move.py" "$@"
