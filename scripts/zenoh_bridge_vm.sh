#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# zenoh_bridge_vm.sh
#
# Runs ON THE VM.
#
# Connects outbound to the Jetson's Zenoh router and bridges the VM's
# local CycloneDDS domain 42 to Zenoh. The autonomy stack on the VM
# must run with ROS_DOMAIN_ID=42.
#
# Override the Jetson's Wi-Fi IP with:
#   JETSON_WIFI_IP=192.168.137.99 ./scripts/zenoh_bridge_vm.sh
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
CONFIG="${WS_ROOT}/config/zenoh_vm_config.json5"

[ -f "${CONFIG}" ] || { echo "ERROR: ${CONFIG} not found"; exit 1; }

# Default Jetson IP; override via env var
JETSON_WIFI_IP="${JETSON_WIFI_IP:-192.168.137.50}"

# Verify reachability (best-effort; will still try to start the bridge)
if ! ping -c 1 -W 1 "${JETSON_WIFI_IP}" > /dev/null 2>&1; then
  echo "[zenoh_vm] WARN: cannot ping ${JETSON_WIFI_IP}"
  echo "[zenoh_vm]       verify the Jetson is on Wi-Fi and reachable"
fi

set +u
source /opt/ros/humble/setup.bash
set -u

# DDS stays local to the VM — domain 42 (different from Jetson's 0)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42

# Substitute the Jetson IP into the config on the fly
TMP_CONFIG="$(mktemp /tmp/zenoh_vm_config.XXXXXX.json5)"
trap 'rm -f "${TMP_CONFIG}"' EXIT
sed "s|tcp/192.168.137.50:7447|tcp/${JETSON_WIFI_IP}:7447|" "${CONFIG}" > "${TMP_CONFIG}"

echo "[zenoh_vm] RMW            = ${RMW_IMPLEMENTATION}"
echo "[zenoh_vm] ROS_DOMAIN_ID  = ${ROS_DOMAIN_ID}"
echo "[zenoh_vm] Jetson Wi-Fi IP= ${JETSON_WIFI_IP}"
echo "[zenoh_vm] Connecting to tcp/${JETSON_WIFI_IP}:7447"

exec zenoh-bridge-ros2dds -c "${TMP_CONFIG}"