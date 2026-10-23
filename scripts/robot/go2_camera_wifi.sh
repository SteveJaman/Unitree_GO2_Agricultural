#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# go2_camera_wifi.sh: ON THE ROBOT. Publish the front camera on the Wi-Fi DDS
# domain (same settings as go2-nav2-wifi's robot-relay-wifi.sh), so the laptop
# gets colour JPEG frames on /go2/front_camera/compressed.
#
#   GO2_HOST_IP=<laptop Wi-Fi IP> bash ~/go2-agri-camera/go2_camera_wifi.sh
#
# Copied to the robot and started by wifi_nav.sh (camera on). One instance at a
# time; the factory DDS configuration is never changed.
# Env: GO2_RELAY_DOMAIN_ID=64  GO2_WIFI_IFACE=wlan0 (auto)  GO2_CAM_* (see .py)
# ---------------------------------------------------------------------------
set -eo pipefail

: "${GO2_HOST_IP:?Set GO2_HOST_IP to the laptop Wi-Fi IP}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOMAIN="${GO2_RELAY_DOMAIN_ID:-64}"
IFACE="${GO2_WIFI_IFACE:-$(ip -br link | awk '/^wl/ {print $1; exit}')}"
[ -n "${IFACE}" ] || { echo "no Wi-Fi interface: set GO2_WIFI_IFACE" >&2; exit 1; }
[[ "${IFACE}" =~ ^[a-zA-Z0-9_.:-]+$ && "${GO2_HOST_IP}" =~ ^[a-zA-Z0-9_.:-]+$ ]] \
    || { echo "invalid interface or host address" >&2; exit 2; }

exec 9>/tmp/go2-camera-wifi.lock
flock -n 9 || { echo "go2_camera_wifi already running" >&2; exit 1; }

# shellcheck disable=SC1091
source "${DIR}/robot-source-unitree-ros.sh"
XML=/tmp/go2-camera-wifi-cyclonedds.xml
cat > "${XML}" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces><NetworkInterface name="${IFACE}" /></Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
      <Peers><Peer address="${GO2_HOST_IP}"/></Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
export ROS_DOMAIN_ID="${DOMAIN}" CYCLONEDDS_URI="file://${XML}" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
exec python3 "${DIR}/go2_camera_wifi.py"
