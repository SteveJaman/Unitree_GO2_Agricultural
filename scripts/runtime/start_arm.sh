#!/usr/bin/env bash
# start_arm.sh: fold the D1 arm into its calibrated resting pose from the go2 container.
#
# Usage:
#   ./start_arm.sh                 # enable -> glide to FOLDED_ANGLES -> release
#   ./start_arm.sh --read          # print current angles (for calibration)
#   ./start_arm.sh --no-home       # enable only (no motion)
#   ./start_arm.sh --setup-net     # give the arm interface a 192.168.123.x address (sudo)
#
# Calibration (once, or whenever the mount changes):
#   1. Fold the arm by hand onto the dog's back (motors released)
#   2. ./start_arm.sh --read
#   3. Paste the printed FOLDED_ANGLES line into scripts/nodes/d1_arm_calibration.py
#
# Environment overrides:
#   ARM_IFACE=enp0s8     wired interface the arm is on (auto-detected if unset)
#   ARM_SUBNET=192.168.123   subnet the arm lives on (arm default IP: .100)
#   ARM_IP=192.168.123.100
#   ARM_PEER=192.168.123.100   use unicast discovery instead of multicast (--no-home only)
#   ARM_ALLOW_DEFAULT_FOLD=1   fold even though FOLDED_ANGLES is still the placeholder
#   ARM_HOST_IP=192.168.123.51 address --setup-net assigns to the VM side
#   CONTAINER_NAME=go2-lidar3d
#   NODES_DIR=/path/to/folder/with/d1_arm_calibration.py
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NODES_DIR="${NODES_DIR:-$(dirname "$SCRIPT_DIR")/nodes}"
[ -f "$NODES_DIR/d1_arm_calibration.py" ] || NODES_DIR="$SCRIPT_DIR"

CONTAINER_NAME="${CONTAINER_NAME:-go2-lidar3d}"
SUBNET="${ARM_SUBNET:-192.168.123}"
ARM_IP="${ARM_IP:-${SUBNET}.100}"
ARM_IFACE="${ARM_IFACE:-}"
ARM_PEER="${ARM_PEER:-}"

ACTION="fold"
case "${1:-}" in
    "")          ;;
    --read)      ACTION="read" ;;
    --no-home)   ACTION="enable" ;;
    --setup-net) ACTION="setup-net" ;;
    -h|--help)   awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; exit 0 ;;
    *)           echo "Unknown option: $1" >&2; exit 1 ;;
esac

# Placeholder shipped in d1_arm_calibration.py: joints 1/2 at their +/-90 limits.
DEFAULT_FOLD_LINE='FOLDED_ANGLES = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0, 0.0]'

die() { echo "ERROR: $*" >&2; exit 1; }

# --- network setup (no container needed) ----------------------------------
# Non-destructive: only adds an address when the interface has none on the
# arm subnet. Never flushes, so other addresses on the interface survive.
if [ "$ACTION" = "setup-net" ]; then
    IFACE="${ARM_IFACE:-enp0s8}"
    HOST_IP="${ARM_HOST_IP:-${SUBNET}.51}"
    ip link show dev "$IFACE" >/dev/null 2>&1 \
        || die "interface $IFACE not found. Attach the arm's adapter to the VM, or set ARM_IFACE."
    SUDO=()
    [ "$(id -u)" -eq 0 ] || SUDO=(sudo)
    "${SUDO[@]}" ip link set "$IFACE" up
    if ip -4 -o addr show dev "$IFACE" | awk '{print $4}' | grep -q "^${SUBNET//./\\.}\\."; then
        echo "[arm] $IFACE already has an address on ${SUBNET}.x:"
    else
        echo "[arm] adding ${HOST_IP}/24 to $IFACE"
        "${SUDO[@]}" ip addr add "${HOST_IP}/24" dev "$IFACE"
    fi
    ip -4 -o addr show dev "$IFACE" | awk '{print "      " $4}'
    if ping -c3 -W2 -I "$IFACE" "$ARM_IP" >/dev/null 2>&1; then
        echo "[arm] ping $ARM_IP: OK"
    else
        echo "[arm] WARNING: $ARM_IP does not answer yet (powered? cable? ~1 min boot)"
    fi
    echo "[arm] Not persistent across reboots; re-run after a VM restart."
    exit 0
fi

# --- container -------------------------------------------------------------
docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME" \
    || die "container $CONTAINER_NAME not running (docker start $CONTAINER_NAME)"

NETMODE="$(docker inspect -f '{{.HostConfig.NetworkMode}}' "$CONTAINER_NAME")"
[ "$NETMODE" = "host" ] \
    || die "container network mode is '$NETMODE', but DDS needs --network host. Recreate the container with --network host."

# --- interface -------------------------------------------------------------
if [ -z "$ARM_IFACE" ]; then
    ARM_IFACE="$(ip -4 -o addr show | awk -v p="^${SUBNET}\\." '$4 ~ p {print $2; exit}')"
fi
if [ -z "$ARM_IFACE" ]; then
    echo "ERROR: no interface has an address in ${SUBNET}.x" >&2
    echo "Interfaces:" >&2
    ip -br addr | sed 's/^/  /' >&2
    echo >&2
    echo "Plug the arm into the VM (USB Ethernet / bridged adapter), then e.g.:" >&2
    echo "  sudo ip addr add ${SUBNET}.51/24 dev <iface> && sudo ip link set <iface> up" >&2
    echo "and re-run with ARM_IFACE=<iface> ./start_arm.sh" >&2
    exit 1
fi
echo "[arm] interface: $ARM_IFACE   arm ip: $ARM_IP   container: $CONTAINER_NAME"

docker exec -u root "$CONTAINER_NAME" test -e "/sys/class/net/${ARM_IFACE}" \
    || die "interface ${ARM_IFACE} not visible inside the container"

if ping -c2 -W1 -I "$ARM_IFACE" "$ARM_IP" >/dev/null 2>&1; then
    echo "[arm] ping $ARM_IP: OK"
else
    echo "[arm] WARNING: $ARM_IP does not answer ping on $ARM_IFACE."
    echo "      Check cable, host IP on ${SUBNET}.x, and that the arm has finished booting (~1 min)."
fi

# --- python side -----------------------------------------------------------
[ -f "$NODES_DIR/d1_arm_calibration.py" ] || die "d1_arm_calibration.py not found in $NODES_DIR"
echo "[arm] copying d1_arm_calibration.py into container..."
docker cp "$NODES_DIR/d1_arm_calibration.py" "${CONTAINER_NAME}:/tmp/d1_arm_calibration.py"
if [ -f "$NODES_DIR/d1_arm_control.py" ]; then
    docker cp "$NODES_DIR/d1_arm_control.py" "${CONTAINER_NAME}:/tmp/d1_arm_control.py"
fi

if ! docker exec -u root "$CONTAINER_NAME" python3 -c "import cyclonedds" >/dev/null 2>&1; then
    echo "[arm] cyclonedds missing, installing..."
    # 0.10.2 is the version Unitree's own SDK is built and tested against.
    docker exec -u root "$CONTAINER_NAME" pip3 install cyclonedds==0.10.2 \
        || die "pip install cyclonedds==0.10.2 failed (no wheel? build CycloneDDS from source and set CYCLONEDDS_HOME)"
fi
CDDS_VER="$(docker exec -u root "$CONTAINER_NAME" pip3 show cyclonedds 2>/dev/null | awk '/^Version:/{print $2}')"
[ "$CDDS_VER" = "0.10.2" ] || echo "[arm] NOTE: cyclonedds is ${CDDS_VER:-unknown}; Unitree's SDK uses 0.10.2. If discovery fails: pip3 install cyclonedds==0.10.2"

# NOTE: no ROS sourcing on purpose; ROS's libddsc can clash with the pip cyclonedds.
# CYCLONEDDS_URI is unset so the arm gets its own config bound to ARM_IFACE only,
# never the Go2's Wi-Fi DDS config.
run_py() {
    docker exec -u root "$CONTAINER_NAME" \
        env -u CYCLONEDDS_URI -u ROS_DOMAIN_ID python3 "$@"
}

case "$ACTION" in
    read)
        echo "[arm] reading current joint angles..."
        run_py /tmp/d1_arm_calibration.py --iface "$ARM_IFACE" --read \
            || die "read failed (run ./scripts/tools/diag_arm.sh)"
        echo
        echo "[arm] Paste the FOLDED_ANGLES line above into $NODES_DIR/d1_arm_calibration.py"
        ;;
    fold)
        FOLD_LINE="$(grep -m1 '^FOLDED_ANGLES *=' "$NODES_DIR/d1_arm_calibration.py" | tr -d '\r' || true)"
        if [ "$FOLD_LINE" = "$DEFAULT_FOLD_LINE" ] && [ "${ARM_ALLOW_DEFAULT_FOLD:-0}" != "1" ]; then
            echo "ERROR: FOLDED_ANGLES is still the uncalibrated placeholder:" >&2
            echo "         $FOLD_LINE" >&2
            echo "       Calibrate first:  ./start_arm.sh --read  (then paste the line)" >&2
            echo "       Or force it:      ARM_ALLOW_DEFAULT_FOLD=1 ./start_arm.sh" >&2
            exit 1
        fi
        echo "[arm] target: ${FOLD_LINE}"
        echo "[arm] enabling, gliding to folded pose, releasing (keep clear of the arm)..."
        run_py /tmp/d1_arm_calibration.py --iface "$ARM_IFACE" --move \
            || die "fold failed (run ./scripts/tools/diag_arm.sh)"
        echo "[arm] Folded and released."
        ;;
    enable)
        [ -f "$NODES_DIR/d1_arm_control.py" ] || die "d1_arm_control.py not found in $NODES_DIR"
        PY_ARGS=(--iface "$ARM_IFACE")
        [ -n "$ARM_PEER" ] && PY_ARGS+=(--peer "$ARM_PEER")
        echo "[arm] enabling motors..."
        run_py /tmp/d1_arm_control.py "${PY_ARGS[@]}" enable || die "enable failed (run ./scripts/tools/diag_arm.sh)"
        ;;
esac

echo
echo "[arm] Manual control (angles in DEGREES):"
echo "  docker exec -u root ${CONTAINER_NAME} env -u CYCLONEDDS_URI python3 /tmp/d1_arm_control.py --iface ${ARM_IFACE} joint 0 20"
echo "  docker exec -u root ${CONTAINER_NAME} env -u CYCLONEDDS_URI python3 /tmp/d1_arm_control.py --iface ${ARM_IFACE} feedback"
