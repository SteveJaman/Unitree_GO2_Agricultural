#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# start_arm.sh — launch the D1 arm over the bridged Ethernet, or over Wi-Fi
# when the arm is plugged into the Go2 (--robot <Go2 IP> / ARM_ROBOT: the
# robot's arm_bridge_wifi.py, see docs/hardware/d1arm.md).
#
# Uses the verified config: iface=enp0s8, domain=0, MULTICAST discovery.
# Do NOT pass --peer to the underlying scripts — unicast breaks on this arm.
#
# Usage:
#   ./start_arm.sh --check                    # preflight only
#   ./start_arm.sh fold                       # glide to FOLDED_ANGLES, then release
#   ./start_arm.sh read                       # print FOLDED_ANGLES (calibration)
#   ./start_arm.sh state                      # read state
#   ./start_arm.sh home                       # home (starts OFF, ends OFF)
#   ./start_arm.sh enable sleep 2 joint 0 20 sleep 3 release
#   ./start_arm.sh lift --deg 45 --yes
#   ./start_arm.sh sweep --pct 90 --yes
#   ./start_arm.sh --calibrate --test-deg 10 --yes
#   ./start_arm.sh --robot 192.168.1.9 fold   # arm on the Go2, over Wi-Fi
#
# Env overrides:
#   ARM_IFACE  (default: enp0s8)
#   ARM_IP     (default: 192.168.123.100)
#   ARM_DOMAIN (default: 0)
#   ARM_HOST_IP (default: 192.168.123.99, this VM's address on ARM_IFACE)
#   ARM_ROBOT  (unset = wired; Go2 Wi-Fi IP = through its bridge)
#   ARM_BRIDGE_PORT (default: 18100)
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$( cd "${SCRIPT_DIR}/../.." &> /dev/null && pwd )"

export ARM_IFACE="${ARM_IFACE:-enp0s8}"
export ARM_IP="${ARM_IP:-192.168.123.100}"
export ARM_DOMAIN="${ARM_DOMAIN:-0}"
ARM_HOST_IP="${ARM_HOST_IP:-192.168.123.99}"
if [ "${1:-}" = "--robot" ]; then
    [ -n "${2:-}" ] || { echo "--robot needs the Go2 Wi-Fi IP" >&2; exit 2; }
    export ARM_ROBOT="$2"
    shift 2
fi
export ARM_BRIDGE_PORT="${ARM_BRIDGE_PORT:-18100}"

CONTROL="${REPO_ROOT}/scripts/nodes/d1_arm_control.py"
CALIBRATE="${REPO_ROOT}/scripts/tools/calibrate_arm.py"
FOLD="${REPO_ROOT}/scripts/nodes/d1_arm_calibration.py"

preflight() {
    local fail=0
    if [ -n "${ARM_ROBOT:-}" ]; then
        echo "=== ARM preflight (over Wi-Fi, Go2 bridge) ==="
        echo "  bridge : ${ARM_ROBOT}:${ARM_BRIDGE_PORT}"
        if (exec 3<>"/dev/tcp/${ARM_ROBOT}/${ARM_BRIDGE_PORT}") 2>/dev/null; then
            echo "  [ok] bridge reachable"
            return 0
        fi
        echo "  [!] bridge not reachable"
        echo "      Start it: wifi_nav.sh --arm-wifi, or on the robot:"
        echo "      GO2_HOST_IP=<laptop IP> bash ~/go2-agri-arm/arm_bridge_wifi.sh"
        return 1
    fi
    echo "=== ARM preflight ==="
    echo "  iface  : $ARM_IFACE"
    echo "  arm ip : $ARM_IP"
    echo "  domain : $ARM_DOMAIN"
    echo
    if ! ip -br addr show "$ARM_IFACE" 2>/dev/null | grep -qF "$ARM_HOST_IP/"; then
        echo "  [!] $ARM_IFACE is missing $ARM_HOST_IP"
        echo "      Fix: sudo ip addr add $ARM_HOST_IP/24 dev $ARM_IFACE"
        fail=1
    else
        echo "  [ok] $ARM_IFACE has $ARM_HOST_IP"
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
        awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"
        exit 0
        ;;
    --check|check)
        preflight
        exit $?
        ;;
    --fold|fold)
        preflight || { echo "[!] preflight failed"; exit 1; }
        exec python3 "$FOLD" --iface "$ARM_IFACE" --move
        ;;
    --read|read)
        preflight || { echo "[!] preflight failed"; exit 1; }
        exec python3 "$FOLD" --iface "$ARM_IFACE" --read
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
