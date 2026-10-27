#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# arm_bridge_wifi.sh: ON THE ROBOT. Run the D1 arm Wi-Fi bridge
# (arm_bridge_wifi.py): the arm plugged into the Go2's Ethernet port, its
# DDS on eth0 (domain 0, multicast: the arm's verified settings), the laptop
# over TCP.
#
#   bash ~/go2-agri-arm/arm_bridge_wifi.sh --build   # once: build unitree_arm (ArmString)
#   GO2_HOST_IP=<laptop Wi-Fi IP> bash ~/go2-agri-arm/arm_bridge_wifi.sh
#
# Needs robot-source-unitree-ros.sh (go2-nav2-wifi; copied alongside by
# wifi_nav.sh) for Unitree's CycloneDDS.
#
# --build compiles the one-message ROS 2 package unitree_arm in
# ~/go2-agri-arm/ws with the robot's ROS Foxy (colcon); nothing else is
# installed. Copied to the robot and started by wifi_nav.sh --arm-wifi.
# Env: ARM_BRIDGE_IFACE=eth0  ARM_DOMAIN=0  ARM_BRIDGE_PORT=18100  ARM_BRIDGE_TIMEOUT=3
# ---------------------------------------------------------------------------
set -eo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${DIR}/ws"
IFACE="${ARM_BRIDGE_IFACE:-eth0}"
[[ "${IFACE}" =~ ^[a-zA-Z0-9_.:-]+$ ]] || { echo "invalid interface ${IFACE}" >&2; exit 2; }

set +u
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash

if [ "${1:-}" = "--build" ]; then
    mkdir -p "${WS}/src"
    rm -rf "${WS}/src/unitree_arm"
    cp -r "${DIR}/unitree_arm" "${WS}/src/"
    cd "${WS}"
    colcon build --packages-select unitree_arm
    # shellcheck disable=SC1091
    source "${WS}/install/setup.bash"
    python3 -c "from unitree_arm.msg import ArmString; print('unitree_arm OK')"
    exit 0
fi

[ -f "${WS}/install/setup.bash" ] \
    || { echo "unitree_arm not built: run once: bash ${DIR}/arm_bridge_wifi.sh --build" >&2; exit 1; }
# Unitree's CycloneDDS, like every ROS node on the Go2: rclpy with Foxy's
# stock rmw_cyclonedds_cpp segfaults creating a node on this robot.
for env in "${DIR}/robot-source-unitree-ros.sh" "${HOME}/robot-source-unitree-ros.sh"; do
    # shellcheck disable=SC1090
    [ -f "${env}" ] && { source "${env}" >/dev/null; break; }
done
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"

exec 9>/tmp/arm-bridge-wifi.lock
flock -n 9 || { echo "arm_bridge_wifi already running" >&2; exit 1; }

ip -br addr show "${IFACE}" >/dev/null 2>&1 || { echo "no interface ${IFACE}" >&2; exit 1; }
XML=/tmp/arm-bridge-cyclonedds.xml
cat > "${XML}" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces><NetworkInterface name="${IFACE}" multicast="default" /></Interfaces>
    </General>
  </Domain>
</CycloneDDS>
EOF
export ROS_DOMAIN_ID="${ARM_DOMAIN:-0}" CYCLONEDDS_URI="file://${XML}" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
exec python3 -u "${DIR}/arm_bridge_wifi.py"
