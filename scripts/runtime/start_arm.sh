#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# start_arm.sh — launch the D1 arm over the bridged Ethernet, or over Wi-Fi
# when the arm is plugged into the Go2 (--robot <Go2 IP> / ARM_ROBOT: the
# robot's arm_bridge_wifi.py, see docs/hardware/d1arm.md).
#
# Uses the verified config: wired iface, domain=0, MULTICAST discovery.
# Do NOT pass --peer to the underlying scripts — unicast breaks on this arm.
#
# Usage:
#   ./start_arm.sh --check                    # preflight only
#   ./start_arm.sh fold                       # glide to FOLDED_ANGLES, then release
#   ./start_arm.sh read                       # print FOLDED_ANGLES (calibration)
#   ./start_arm.sh state                      # read state
#   ./start_arm.sh home                       # home (starts OFF, ends OFF); --home works too
#   ./start_arm.sh enable sleep 2 joint 0 20 sleep 3 release
#   ./start_arm.sh lift --deg 45 --yes
#   ./start_arm.sh sweep --pct 90 --yes
#   ./start_arm.sh orbit                      # orbit scan around a table-top object
#   ./start_arm.sh --calibrate --test-deg 10 --yes
#   ./start_arm.sh --robot 192.168.1.9 fold   # arm on the Go2, over Wi-Fi
#
# Env overrides:
#   ARM_IFACE  (default: auto-detected by go2_net.sh, e.g. enp0s31f6)
#   ARM_IP     (default: 192.168.123.100)
#   ARM_DOMAIN (default: 0)
#   ARM_HOST_IP (default: 192.168.123.99, this computer's address on ARM_IFACE)
#   ARM_ROBOT  (unset = wired; Go2 Wi-Fi IP = through its bridge. Unset and
#              no cable link on ARM_IFACE: the Go2 found by go2_net.sh is used)
#   ARM_BRIDGE_PORT (default: 18100)
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$( cd "${SCRIPT_DIR}/../.." &> /dev/null && pwd )"

# ARM_IFACE defaults to the detected wired interface (see go2_net.sh).
# shellcheck source=go2_net.sh
source "${SCRIPT_DIR}/go2_net.sh"
export ARM_IFACE
export ARM_IP="${ARM_IP:-192.168.123.100}"
export ARM_DOMAIN="${ARM_DOMAIN:-0}"
ARM_HOST_IP="${ARM_HOST_IP:-192.168.123.99}"
if [ "${1:-}" = "--robot" ]; then
    [ -n "${2:-}" ] || { echo "--robot needs the Go2 Wi-Fi IP" >&2; exit 2; }
    export ARM_ROBOT="$2"
    shift 2
fi
export ARM_BRIDGE_PORT="${ARM_BRIDGE_PORT:-18100}"

# No cable link on the wired arm port (carrier 0) but the arm answers behind
# the Go2: use it over Wi-Fi through the robot's bridge instead of failing.
if [ -z "${ARM_ROBOT:-}" ] && [ "$(cat "/sys/class/net/${ARM_IFACE}/carrier" 2>/dev/null)" != 1 ]; then
    robot_ip="${GO2_ROBOT_IP:-$(head -n1 "${GO2_IP_CACHE}" 2>/dev/null || true)}"
    # Several pings (any reply counts) or the bridge port: one 1 s ping can
    # land in a Wi-Fi power-save stall and silently fall back to the cable.
    if go2_valid_ip "${robot_ip}" && { ping -c3 -i0.3 -W1 "${robot_ip}" >/dev/null 2>&1 \
            || (exec 3<>"/dev/tcp/${robot_ip}/${ARM_BRIDGE_PORT}") 2>/dev/null; }; then
        echo "[arm] ${ARM_IFACE} has no cable link; using the arm through the Go2 at ${robot_ip} (--robot)"
        export ARM_ROBOT="${robot_ip}"
    elif go2_valid_ip "${robot_ip}"; then
        echo "[arm] ${ARM_IFACE} has no cable link and the Go2 at ${robot_ip} does not answer over Wi-Fi."
        echo "      Check the dog is on and on the same Wi-Fi, or pass --robot <Go2 Wi-Fi IP>."
    fi
fi

bridge_open() { (exec 3<>"/dev/tcp/${ARM_ROBOT}/${ARM_BRIDGE_PORT}") 2>/dev/null; }

# Start the arm bridge already deployed on the robot (wifi_nav.sh --arm-wifi
# copies and builds it once). Detached with setsid, so it outlives this run.
start_bridge() {
    local robot="${GO2_ROBOT_USER}@${ARM_ROBOT}"
    if ! ssh "${GO2_SSH_OPTS[@]}" -T -n -o ConnectTimeout=5 "${robot}" \
            "test -f ~/go2-agri-arm/ws/install/setup.bash" 2>/dev/null; then
        echo "  [!] the arm bridge is not deployed/built on the robot (or no SSH)"
        echo "      Once: ./scripts/runtime/wifi_nav.sh --arm-wifi, then on the robot:"
        echo "      bash ~/go2-agri-arm/arm_bridge_wifi.sh --build"
        return 1
    fi
    go2_detect_host_ip 2>/dev/null || { echo "  [!] no Wi-Fi address on this computer"; return 1; }
    echo "  starting the arm bridge on ${ARM_ROBOT} (robot log /tmp/arm_bridge_wifi.log)..."
    # Two calls: the start line names arm_bridge_wifi.sh, so a pkill in the
    # same remote shell would match and kill that shell.
    ssh "${GO2_SSH_OPTS[@]}" -T -n -o ConnectTimeout=5 "${robot}" \
        "pkill -TERM -f 'arm_bridge_[w]ifi'" >/dev/null 2>&1 || true
    ssh "${GO2_SSH_OPTS[@]}" -T -n -o ConnectTimeout=5 "${robot}" \
        "GO2_HOST_IP=${GO2_HOST_IP} ARM_BRIDGE_PORT=${ARM_BRIDGE_PORT} setsid nohup bash ~/go2-agri-arm/arm_bridge_wifi.sh > /tmp/arm_bridge_wifi.log 2>&1 < /dev/null &" \
        2>/dev/null || { echo "  [!] could not start the arm bridge over SSH"; return 1; }
    for _ in $(seq 1 20); do
        bridge_open && return 0
        sleep 1
    done
    echo "  [!] the bridge did not open port ${ARM_BRIDGE_PORT}; on the robot: tail /tmp/arm_bridge_wifi.log"
    return 1
}

CONTROL="${REPO_ROOT}/scripts/nodes/d1_arm_control.py"
CALIBRATE="${REPO_ROOT}/scripts/tools/calibrate_arm.py"
FOLD="${REPO_ROOT}/scripts/nodes/d1_arm_calibration.py"
ORBIT="${REPO_ROOT}/scripts/nodes/orbit_scan.py"

preflight() {
    local fail=0
    if [ -n "${ARM_ROBOT:-}" ]; then
        echo "=== ARM preflight (over Wi-Fi, Go2 bridge) ==="
        echo "  bridge : ${ARM_ROBOT}:${ARM_BRIDGE_PORT}"
        if bridge_open || start_bridge; then
            echo "  [ok] bridge reachable"
            return 0
        fi
        echo "  [!] bridge not reachable"
        return 1
    fi
    echo "=== ARM preflight ==="
    echo "  iface  : $ARM_IFACE"
    echo "  arm ip : $ARM_IP"
    echo "  domain : $ARM_DOMAIN"
    echo
    if [ "$(cat "/sys/class/net/${ARM_IFACE}/carrier" 2>/dev/null)" != 1 ]; then
        echo "  [!] $ARM_IFACE has no cable link"
        echo "      Plug the arm into this port, or if it is plugged into the Go2:"
        echo "      ./scripts/runtime/start_arm.sh --robot <Go2 Wi-Fi IP> ..."
        fail=1
    fi
    if ! ip -br addr show "$ARM_IFACE" 2>/dev/null | grep -qF "$ARM_HOST_IP/"; then
        echo "  [!] $ARM_IFACE is missing $ARM_HOST_IP"
        echo "      Fix: sudo ip addr add $ARM_HOST_IP/24 dev $ARM_IFACE   (or: sudo nmcli connection up go2-arm)"
        fail=1
    else
        echo "  [ok] $ARM_IFACE has $ARM_HOST_IP"
    fi
    if ! ping -c 2 -W 2 "$ARM_IP" >/dev/null 2>&1; then
        echo "  [!] $ARM_IP does not reply to ping"
        echo "      Fix: check the arm power (24V) and cable, then power-cycle the arm"
        fail=1
    else
        echo "  [ok] $ARM_IP replies to ping"
    fi
    echo
    return $fail
}

# Accept --home, --state, ... like --fold / --check: d1_arm_control.py takes
# its commands without dashes.
case "${1:-}" in
    --state|--enable|--release|--home|--power|--joint|--joints|--feedback|--status|--motors|--lift|--sweep)
        set -- "${1#--}" "${@:2}" ;;
esac

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
    --orbit|orbit)
        shift
        preflight || { echo "[!] preflight failed"; exit 1; }
        exec python3 "$ORBIT" --iface "$ARM_IFACE" --skip-preflight "$@"
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
