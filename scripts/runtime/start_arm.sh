#!/usr/bin/env bash
# start_arm.sh — initialize the D1-T arm and optionally home it.
#
# Usage:
#   ./scripts/runtime/start_arm.sh              # enable + home
#   ./scripts/runtime/start_arm.sh --no-home    # enable only
#   ARM_IFACE=eth0 ./scripts/runtime/start_arm.sh
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NODES_DIR="$(dirname "$SCRIPT_DIR")/nodes"
CONTAINER_NAME="${CONTAINER_NAME:-go2-lidar3d}"
ARM_IFACE="${ARM_IFACE:-eth0}"

HOME_ARM="true"
[ "${1:-}" = "--no-home" ] && HOME_ARM="false"

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    echo "ERROR: container $CONTAINER_NAME not running." >&2
    exit 1
fi

echo "[arm] Copying d1_arm_control.py into container..."
docker cp "${NODES_DIR}/d1_arm_control.py" \
    "${CONTAINER_NAME}:/tmp/d1_arm_control.py"

echo "[arm] Testing reachability on interface ${ARM_IFACE}..."
docker exec -u root "$CONTAINER_NAME" bash -lc "
    ip link show ${ARM_IFACE} >/dev/null 2>&1 || {
        echo 'ERROR: interface ${ARM_IFACE} not present in container';
        ip -brief link | awk '{print \$1}';
        exit 1
    }
" || exit 1

echo "[arm] Enabling motors..."
docker exec -u root "$CONTAINER_NAME" bash -lc "
    source /opt/ros/humble/setup.bash 2>/dev/null
    source /ws/install/setup.bash 2>/dev/null
    python3 /tmp/d1_arm_control.py --iface ${ARM_IFACE} enable
"

if [ "$HOME_ARM" = "true" ]; then
    echo "[arm] Homing..."
    docker exec -u root "$CONTAINER_NAME" bash -lc "
        source /opt/ros/humble/setup.bash 2>/dev/null
        source /ws/install/setup.bash 2>/dev/null
        python3 /tmp/d1_arm_control.py --iface ${ARM_IFACE} home
    "
fi

echo "[arm] Ready."
echo
echo "Try:"
echo "  docker exec -it -u root ${CONTAINER_NAME} bash -lc \\"
echo "    'python3 /tmp/d1_arm_control.py --iface ${ARM_IFACE} joint 0 0.5'"