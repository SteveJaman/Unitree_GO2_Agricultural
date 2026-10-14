#!/usr/bin/env bash
# diag_arm.sh — D1 arm network + DDS diagnostic.
# Prints a checklist and a verdict. No side effects.
set -uo pipefail

CONTAINER="${CONTAINER_NAME:-go2-lidar3d}"
IFACE="${ARM_IFACE:-enp0s8}"
SUBNET="${ARM_SUBNET:-192.168.123}"
D1_DEFAULT="${D1_DEFAULT_IP:-192.168.123.100}"

echo "=== D1 arm diagnostic ==="
echo "container: $CONTAINER"
echo "interface: $IFACE"
echo "subnet:    $SUBNET"
echo

# 1. Container
echo "--- 1. container ---"
if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "  PASS: $CONTAINER running"
else
    echo "  FAIL: $CONTAINER not running"
    echo "  fix:  docker start $CONTAINER"
    exit 1
fi

if docker exec -u root "$CONTAINER" ip link show "$IFACE" >/dev/null 2>&1; then
    echo "  PASS: container sees $IFACE"
else
    echo "  FAIL: container does not see $IFACE"
    docker exec -u root "$CONTAINER" ip -brief link | sed 's/^/        /'
    exit 1
fi

# 2. Interface IP
echo
echo "--- 2. interface IP ---"
IFACE_LINE=$(ip -brief addr show "$IFACE" 2>/dev/null)
echo "  $IFACE_LINE"
IFACE_IP=$(echo "$IFACE_LINE" | grep -oP 'inet \K[0-9.]+' | head -1)
if [ -n "$IFACE_IP" ]; then
    echo "  PASS: $IFACE has IP $IFACE_IP"
else
    echo "  FAIL: $IFACE has no IPv4"
    echo "  fix:  sudo ip addr add ${SUBNET}.51/24 dev $IFACE"
    echo "        sudo ip link set $IFACE up"
fi

# 3. IP conflict
echo
echo "--- 3. IP conflict with D1 factory default ---"
if [ "$IFACE_IP" = "$D1_DEFAULT" ]; then
    echo "  FAIL: VM $IFACE is squatting on $D1_DEFAULT"
    echo "        the arm cannot come online at its default address"
    echo "  fix:  sudo ip addr del $D1_DEFAULT/24 dev $IFACE"
    echo "        sudo ip addr add ${SUBNET}.51/24 dev $IFACE"
else
    echo "  PASS: no conflict with $D1_DEFAULT"
fi

# 4. Neighbors
echo
echo "--- 4. neighbor table ---"
NEIGH=$(ip neigh show dev "$IFACE" 2>/dev/null)
if [ -z "$NEIGH" ]; then
    echo "  (empty)"
else
    echo "$NEIGH" | sed 's/^/  /'
fi

# 5. Sweep
echo
echo "--- 5. sweep $SUBNET.0/24 ---"
if [ -z "$IFACE_IP" ]; then
    echo "  skipped (no IP on $IFACE)"
else
    FOUND=0
    for i in $(seq 1 254); do
        if ping -c1 -W1 "$SUBNET.$i" >/dev/null 2>&1; then
            MAC=$(ip neigh show "$SUBNET.$i" 2>/dev/null | awk '{print $5}')
            echo "  UP: $SUBNET.$i  $MAC"
            FOUND=$((FOUND + 1))
        fi
    done
    if [ "$FOUND" -eq 0 ]; then
        echo "  FAIL: no hosts responded"
        echo "        arm may be unpowered, wrong subnet, or cable issue"
    else
        echo "  PASS: $FOUND host(s) up"
    fi
fi

# 6. cyclonedds
echo
echo "--- 6. cyclonedds package ---"
if docker exec -u root "$CONTAINER" \
        python3 -c "import cyclonedds" >/dev/null 2>&1; then
    VER=$(docker exec -u root "$CONTAINER" \
        python3 -c "import cyclonedds; print(cyclonedds.__version__)" 2>/dev/null)
    echo "  PASS: installed (version $VER)"
else
    echo "  FAIL: not importable"
    echo "  fix:  docker exec -u root $CONTAINER pip3 install cyclonedds"
fi

# 7. Arm script
echo
echo "--- 7. arm control script ---"
if [ -f /tmp/d1_arm_control.py ]; then
    echo "  PASS: /tmp/d1_arm_control.py present"
else
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    NODES="$(dirname "$SCRIPT_DIR")/nodes"
    if [ -f "$NODES/d1_arm_control.py" ]; then
        docker cp "$NODES/d1_arm_control.py" "$CONTAINER:/tmp/d1_arm_control.py"
        echo "  PASS: copied from $NODES"
    else
        echo "  FAIL: d1_arm_control.py not found"
    fi
fi

# 8. DDS test
echo
echo "--- 8. DDS feedback test ---"
DDS_OUT=$(docker exec -u root "$CONTAINER" bash -c \
    "python3 /tmp/d1_arm_control.py --iface $IFACE --verbose feedback 2>&1" \
    2>&1 || true)
echo "$DDS_OUT" | sed 's/^/  /'
if echo "$DDS_OUT" | grep -q "joint feedback: None"; then
    echo "  WARN: DDS ran but arm did not reply"
elif echo "$DDS_OUT" | grep -q "joint feedback: \["; then
    echo "  PASS: arm replied"
else
    echo "  FAIL: DDS test crashed — see output above"
fi

echo
echo "=== done ==="