#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# setup_env.sh
#
# Source this to prepare a shell for the Unitree_GO2_Agricultural workspace.
#
# Usage:
#   source ./scripts/setup_env.sh                # default: ethernet
#   source ./scripts/setup_env.sh wireless       # use cyclonedds_wireless.xml
#   source ./scripts/setup_env.sh --no-dds       # skip CYCLONEDDS_URI
#
# Environment overrides (set before sourcing):
#   SETUP_NETWORK=false    # skip auto-configuring enp0s3 / enp0s8
#
# After sourcing, the current shell has:
#   - ROS 2 Humble environment
#   - CMU autonomy_stack_go2 environment (if present)
#   - This workspace's install/setup.bash
#   - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#   - CYCLONEDDS_URI pointing at the requested config
#   - ROS_DOMAIN_ID=0 (override with ROS_DOMAIN_ID=... before sourcing)
#   - enp0s3 (internet) and enp0s8 (robot link) brought up automatically
# ---------------------------------------------------------------------------

# Must be sourced, not executed
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "ERROR: this script must be sourced, not executed." >&2
    echo "       Run:  source ${BASH_SOURCE[0]}" >&2
    exit 1
fi

# ---- Locate the workspace root -------------------------------------------
_SETUP_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "${_SETUP_SCRIPT_DIR}/.." &> /dev/null && pwd )"
AUTONOMY_WS="${AUTONOMY_WS:-$HOME/files/autonomy_stack_go2}"

# ---- Parse argument: ethernet | wireless | --no-dds ----------------------
NET_MODE="ethernet"
case "${1:-}" in
    ""|ethernet)    NET_MODE="ethernet" ;;
    wireless)       NET_MODE="wireless" ;;
    --no-dds)       NET_MODE="none" ;;
    *)
        echo "setup_env.sh: unknown mode '$1' (use ethernet|wireless|--no-dds)" >&2
        return 1
        ;;
esac

# ---- Network interfaces ---------------------------------------------------
# Bring up enp0s3 (internet / DHCP) and enp0s8 (robot link / static IP).
# Uses sudo, which may prompt for a password on first invocation.
# Set SETUP_NETWORK=false before sourcing to skip this.

iface_has_ip() {
    ip -br addr show "$1" 2>/dev/null | grep -q "inet "
}

iface_has_ipcidr() {
    ip -br addr show "$1" 2>/dev/null | grep -q "$2"
}

ensure_iface_static() {
    local iface="$1"
    local cidr="$2"
    if iface_has_ipcidr "${iface}" "${cidr}"; then
        return 0
    fi
    echo "setup_env.sh: configuring ${iface} with ${cidr}..."
    sudo ip link set "${iface}" up 2>/dev/null || true
    sudo ip addr add "${cidr}" dev "${iface}" 2>/dev/null || true
}

ensure_iface_dhcp() {
    local iface="$1"
    if iface_has_ip "${iface}"; then
        return 0
    fi
    echo "setup_env.sh: ${iface} has no IP — requesting DHCP..."
    sudo ip link set "${iface}" up 2>/dev/null || true
    if command -v nmcli >/dev/null 2>&1; then
        sudo nmcli device connect "${iface}" >/dev/null 2>&1 || true
    fi
    if ! iface_has_ip "${iface}"; then
        sudo dhclient "${iface}" >/dev/null 2>&1 || true
    fi
}

if [ "${SETUP_NETWORK:-true}" = "true" ]; then
    # enp0s3 — internet via DHCP
    ensure_iface_dhcp enp0s3

    # enp0s8 — static IP on the robot's subnet
    ensure_iface_static enp0s8 "192.168.123.100/24"
fi

# ---- Source ROS 2 (set -u safe) ------------------------------------------
set +u
source /opt/ros/humble/setup.bash

# ---- Source the CMU autonomy stack, if installed -------------------------
if [ -f "${AUTONOMY_WS}/install/setup.bash" ]; then
    source "${AUTONOMY_WS}/install/setup.bash"
else
    echo "setup_env.sh: note — ${AUTONOMY_WS}/install/setup.bash not found"
    echo "              CMU autonomy stack will not be available."
fi

# ---- Source this workspace ----------------------------------------------
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
    source "${WS_ROOT}/install/setup.bash"
else
    echo "setup_env.sh: WARN — ${WS_ROOT}/install/setup.bash not found."
    echo "              Run: cd ${WS_ROOT} && colcon build --symlink-install"
fi

# ---- DDS configuration ---------------------------------------------------
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

case "${NET_MODE}" in
    ethernet)   export CYCLONEDDS_URI="file://${WS_ROOT}/config/cyclonedds_ethernet.xml" ;;
    wireless)   export CYCLONEDDS_URI="file://${WS_ROOT}/config/cyclonedds_wireless.xml" ;;
    none)       unset CYCLONEDDS_URI ;;
esac
set -u

# ---- Report --------------------------------------------------------------
_ip_of() {
    ip -br addr show "$1" 2>/dev/null | awk '{print $3}' | head -1
}

echo "setup_env.sh: workspace ready"
echo "  WS_ROOT        = ${WS_ROOT}"
echo "  RMW            = ${RMW_IMPLEMENTATION}"
echo "  ROS_DOMAIN_ID  = ${ROS_DOMAIN_ID}"
echo "  NET_MODE       = ${NET_MODE}"
[ -n "${CYCLONEDDS_URI:-}" ] && echo "  CYCLONEDDS_URI = ${CYCLONEDDS_URI}"
echo "  enp0s3         = $(_ip_of enp0s3)     (internet)"
echo "  enp0s8         = $(_ip_of enp0s8)     (robot link)"

unset _SETUP_SCRIPT_DIR
