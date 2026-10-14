#!/usr/bin/env bash
# start_arm.sh — initialize the D1-T arm and optionally home it.
#
# Usage:
#   ./scripts/runtime/start_arm.sh              # enable + home
#   ./scripts/runtime/start_arm.sh --no-home    # enable only
#   ARM_IFACE=enp0s8 ./scripts/runtime/start_arm.sh    # VM Ethernet (default)
#   ARM_IFACE=enp0s3 ./scripts/runtime/start_arm.sh    # VM Wi-Fi
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NODES_DIR="$(dirname "$SCRIPT_DIR")/nodes"
CONTAINER_NAME="${CONTAINER_NAME:-go2-lidar3d}"
ARM_IFACE="${ARM_IFACE:-enp0s8}"

HOME_ARM="true"
[ "${1:-}" = "--no-home" ] && HOME_ARM="false"

# ---------------------------------------------------------------------------
# Preflight
# ---------------------------------------------------------------------------
if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    echo "ERROR: container $CONTAINER_NAME not running." >&2
    echo "  Start it with: docker start $CONTAINER_NAME" >&2
    echo "  Or create it:  cd ~/files/go2-nav2-wifi && ./setup-lidar3d.sh --laptop" >&2
    exit 1
fi

echo "[arm] Copying d1_arm_control.py into container..."
docker cp "${NODES_DIR}/d1_arm_control.py" \
    "${CONTAINER_NAME}:/tmp/d1_arm_control.py"

echo "[arm] Checking interface ${ARM_IFACE}..."
docker exec -u root "$CONTAINER_NAME" bash -c "
    ip link show ${ARM_IFACE} >/dev/null 2>&1 || {
        echo 'ERROR: interface ${ARM_IFACE} not present in container';
        echo 'Interfaces available:';
        ip -brief link | awk '{print \"  \" \$1}';
        exit 1
    }
" || exit 1

echo "[arm] Checking cyclonedds..."
if ! docker exec -u root "$CONTAINER_NAME" \
        python3 -c "import cyclonedds" >/dev/null 2>&1; then
    echo "[arm] cyclonedds missing — installing..."
    docker exec -u root "$CONTAINER_NAME" pip3 install cyclonedds
fi

# ---------------------------------------------------------------------------
# Enable motors
# ---------------------------------------------------------------------------
echo "[arm] Enabling motors..."
docker exec -u root "$CONTAINER_NAME" bash -c "
    source /opt/ros/humble/setup.bash 2>/dev/null
    source /ws/install/setup.bash 2>/dev/null
    python3 /tmp/d1_arm_control.py --iface ${ARM_IFACE} enable
" || {
    echo "ERROR: enable failed" >&2
    exit 1
}

# ---------------------------------------------------------------------------
# Home (optional)
# ---------------------------------------------------------------------------
if [ "$HOME_ARM" = "true" ]; then
    echo "[arm] Homing..."
    docker exec -u root "$CONTAINER_NAME" bash -c "
        source /opt/ros/humble/setup.bash 2>/dev/null
        source /ws/install/setup.bash 2>/dev/null
        python3 /tmp/d1_arm_control.py --iface ${ARM_IFACE} home
    " || {
        echo "ERROR: home failed" >&2
        exit 1
    }
fi

# ---------------------------------------------------------------------------
# Ready
# ---------------------------------------------------------------------------
echo
echo "[arm] Ready."
echo
echo "Try:"
echo "  docker exec -it -u root ${CONTAINER_NAME} bash -c \\"
echo "    'python3 /tmp/d1_arm_control.py --iface ${ARM_IFACE} joint 0 0.5'"
echo
echo "Read joint feedback:"
echo "  docker exec -u root ${CONTAINER_NAME} bash -c \\"
echo "    'python3 /tmp/d1_arm_control.py --iface ${ARM_IFACE} feedback'"
echo
echo "Release motors (drag-teach mode):"
echo "  docker exec -u root ${CONTAINER_NAME} bash -c \\"
echo "    'python3 /tmp/d1_arm_control.py --iface ${ARM_IFACE} release'"