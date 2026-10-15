#!/usr/bin/env bash
# start_arm.sh: enable the D1 arm (and optionally home it) from the go2 container.
#
# Usage:
#   ./start_arm.sh                 # enable + home
#   ./start_arm.sh --no-home       # enable only
#
# Environment overrides:
#   ARM_IFACE=enp0s8     wired interface the arm is on (auto-detected if unset)
#   ARM_SUBNET=192.168.123   subnet the arm lives on (arm default IP: .100)
#   ARM_IP=192.168.123.100
#   ARM_PEER=192.168.123.100   use unicast discovery instead of multicast
#   CONTAINER_NAME=go2-lidar3d
#   NODES_DIR=/path/to/folder/with/d1_arm_control.py
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NODES_DIR="${NODES_DIR:-$(dirname "$SCRIPT_DIR")/nodes}"
[ -f "$NODES_DIR/d1_arm_control.py" ] || NODES_DIR="$SCRIPT_DIR"

CONTAINER_NAME="${CONTAINER_NAME:-go2-lidar3d}"
SUBNET="${ARM_SUBNET:-192.168.123}"
ARM_IP="${ARM_IP:-${SUBNET}.100}"
ARM_IFACE="${ARM_IFACE:-}"
ARM_PEER="${ARM_PEER:-}"

HOME_ARM="true"
[ "${1:-}" = "--no-home" ] && HOME_ARM="false"

die() { echo "ERROR: $*" >&2; exit 1; }

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
[ -f "$NODES_DIR/d1_arm_control.py" ] || die "d1_arm_control.py not found in $NODES_DIR"
echo "[arm] copying d1_arm_control.py into container..."
docker cp "$NODES_DIR/d1_arm_control.py" "${CONTAINER_NAME}:/tmp/d1_arm_control.py"

if ! docker exec -u root "$CONTAINER_NAME" python3 -c "import cyclonedds" >/dev/null 2>&1; then
    echo "[arm] cyclonedds missing, installing..."
    # 0.10.2 is the version Unitree's own SDK is built and tested against.
    docker exec -u root "$CONTAINER_NAME" pip3 install cyclonedds==0.10.2 \
        || die "pip install cyclonedds==0.10.2 failed (no wheel? build CycloneDDS from source and set CYCLONEDDS_HOME)"
fi
CDDS_VER="$(docker exec -u root "$CONTAINER_NAME" pip3 show cyclonedds 2>/dev/null | awk '/^Version:/{print $2}')"
[ "$CDDS_VER" = "0.10.2" ] || echo "[arm] NOTE: cyclonedds is ${CDDS_VER:-unknown}; Unitree's SDK uses 0.10.2. If discovery fails: pip3 install cyclonedds==0.10.2"

PY_ARGS=(--iface "$ARM_IFACE")
[ -n "$ARM_PEER" ] && PY_ARGS+=(--peer "$ARM_PEER")

# NOTE: no ROS sourcing on purpose; ROS's libddsc can clash with the pip cyclonedds.
# CYCLONEDDS_URI is unset so the script's own per-interface config is used.
run_arm() {
    docker exec -u root "$CONTAINER_NAME" \
        env -u CYCLONEDDS_URI python3 /tmp/d1_arm_control.py "${PY_ARGS[@]}" "$@"
}

# Everything in ONE process: each new process must rediscover the arm over DDS.
if [ "$HOME_ARM" = "true" ]; then
    echo "[arm] enabling motors and homing (keep clear of the arm)..."
    run_arm enable sleep 2 home sleep 6 || die "enable/home failed (run ./diag_arm.sh)"
else
    echo "[arm] enabling motors..."
    run_arm enable || die "enable failed (run ./diag_arm.sh)"
fi

echo
echo "[arm] Ready. Angles are in DEGREES. Examples:"
echo "  docker exec -u root ${CONTAINER_NAME} env -u CYCLONEDDS_URI python3 /tmp/d1_arm_control.py --iface ${ARM_IFACE} joint 0 20"
echo "  docker exec -u root ${CONTAINER_NAME} env -u CYCLONEDDS_URI python3 /tmp/d1_arm_control.py --iface ${ARM_IFACE} feedback"