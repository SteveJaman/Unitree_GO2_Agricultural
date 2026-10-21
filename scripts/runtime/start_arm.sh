#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# start_arm.sh — launch the D1 arm over the bridged Ethernet.
#
# Uses the verified config: iface=enp0s8, domain=0, MULTICAST discovery.
# Do NOT pass --peer to the underlying scripts — unicast breaks on this arm.
#
# Usage:
#   ./start_arm.sh --check                    # preflight only
#   ./start_arm.sh state                      # read state
#   ./start_arm.sh home                       # home (starts OFF, ends OFF)
#   ./start_arm.sh enable sleep 2 joint 0 20 sleep 3 release
#   ./start_arm.sh lift --deg 45 --yes
#   ./start_arm.sh sweep --pct 90 --yes
#   ./start_arm.sh --calibrate --test-deg 10 --yes
#
# Env overrides:
#   ARM_IFACE  (default: enp0s8)
#   ARM_IP     (default: 192.168.123.100)
#   ARM_DOMAIN (default: 0)
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$( cd "${SCRIPT_DIR}/../.." &> /dev/null && pwd )"

export ARM_IFACE="${ARM_IFACE:-enp0s8}"
export ARM_IP="${ARM_IP:-192.168.123.100}"
export ARM_DOMAIN="${ARM_DOMAIN:-0}"

CONTROL="${REPO_ROOT}/scripts/nodes/d1_arm_control.py"
CALIBRATE="${REPO_ROOT}/scripts/tools/calibrate_arm.py"

preflight() {
    local fail=0
    echo "=== ARM preflight ==="
    echo "  iface  : $ARM_IFACE"
    echo "  arm ip : $ARM_IP"
    echo "  domain : $ARM_DOMAIN"
    echo
    if ! ip -br addr show "$ARM_IFACE" 2>/dev/null | grep -q "192.168.123.99"; then
        echo "  [!] $ARM_IFACE is missing 192.168.123.99"
        echo "      Fix: sudo ip addr add 192.168.123.99/24 dev $ARM_IFACE"
        fail=1
    else
        echo "  [ok] $ARM_IFACE has 192.168.123.99"
    fi
    if ! ping -c 2 -W 2 "$ARM_IP" >/dev/null 2>&1; then
        echo "  [!] $ARM_IP does not reply to ping"
        echo "      Fix: cold-boot the VM (VirtualBox Adapter 2 -> Intel PRO/1000)"
        fail=1
    else
        echo "  [ok] $ARM_IP replies to ping"
    fi
    echo
    return $fail
}

cmd="${1:-}"
case "$cmd" in
    ""|"-h"|"--help")
        sed -n '2,30p' "$0"
        exit 0
        ;;
    --check|check)
        preflight
        exit $?
        ;;
    --calibrate|calibrate)
        shift
        preflight || { echo "[!] preflight failed"; exit 1; }
        exec python3 "$CALIBRATE" "$@"
        ;;
    *)
        preflight || { echo "[!] preflight failed"; exit 1; }
        exec python3 "$CONTROL" "$@"
        ;;
esac
