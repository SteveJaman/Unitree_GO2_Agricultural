#!/usr/bin/env bash
# diag_arm.sh — D1 arm network + DDS diagnostic.
set -uo pipefail

CONTAINER="${CONTAINER_NAME:-go2-lidar3d}"
IFACE="${ARM_IFACE:-enp0s3}"
SUBNET="${ARM_SUBNET:-192.168.1}"
D1_DEFAULT="${D1_DEFAULT_IP:-192.168.1.100}"

echo "=== D1 arm diagnostic ==="
echo "container: $CONTAINER"
echo "interface: $IFACE"
echo "subnet:    $SUBNET"
echo

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

echo
echo "--- 2. interface IP ---"
IFACE_LINE=$(ip -brief addr show "$IFACE" 2>/dev/null)
echo "  Host: $IFACE_LINE"
# FIXED: ip -brief does not output 'inet', so we use awk to grab the 3rd column
IFACE_IP=$(echo "$IFACE_LINE" | awk '{print $3}' | cut -d/ -f1)

# Check if container IP matches (ensures --network host is working)
CONT_IP=$(docker exec -u root "$CONTAINER" ip -brief addr show "$IFACE" 2>/dev/null | awk '{print $3}' | cut -d/ -f1)

if [ -n "$IFACE_IP" ]; then
    echo "  PASS: $IFACE has IP $IFACE_IP on host"
    if [ "$IFACE_IP" = "$CONT_IP" ]; then
        echo "  PASS: container shares host IP ($CONT_IP)"
    else
        echo "  FAIL: container IP ($CONT_IP) differs from host. Is --network=host missing?"
    fi
else
    echo "  FAIL: $IFACE has no IPv4"
    echo "  fix:  sudo ip addr add ${SUBNET}.51/24 dev $IFACE"
    echo "        sudo ip link set $IFACE up"
fi

echo
echo "--- 3. IP conflict with D1 factory default ---"
if [ "$IFACE_IP" = "$D1_DEFAULT" ]; then
    echo "  FAIL: VM $IFACE is squatting on $D1_DEFAULT"
    echo "  fix:  sudo ip addr del $D1_DEFAULT/24 dev $IFACE"
    echo "        sudo ip addr add ${SUBNET}.51/24 dev $IFACE"
else
    echo "  PASS: no conflict with $D1_DEFAULT"
fi

echo
echo "--- 4. neighbor table ---"
NEIGH=$(ip neigh show dev "$IFACE" 2>/dev/null)
if [ -z "$NEIGH" ]; then
    echo "  (empty)"
else
    echo "$NEIGH" | sed 's/^/  /'
fi

echo
echo "--- 5. sweep $SUBNET.0/24 (fast multi-threaded) ---"
if [ -z "$IFACE_IP" ]; then
    echo "  skipped (no IP on $IFACE)"
else
    SWEEP_DIR=$(mktemp -d)
    for i in {1..254}; do
        # Background the pings to run them all at once
        (ping -c1 -W1 "$SUBNET.$i" >/dev/null 2>&1 && echo "$SUBNET.$i" > "$SWEEP_DIR/$i") &
    done
    wait # Wait for all pings to finish (takes 1 second total)

    FOUND=$(ls -1 "$SWEEP_DIR" | wc -l)
    if [ "$FOUND" -eq 0 ]; then
        echo "  FAIL: no hosts responded"
    else
        echo "  PASS: $FOUND host(s) up:"
        for f in "$SWEEP_DIR"/*; do
            if [ -f "$f" ]; then
                IP_UP=$(cat "$f")
                MAC=$(ip neigh show "$IP_UP" 2>/dev/null | awk '{print $5}')
                echo "    UP: $IP_UP  $MAC"
            fi
        done
    fi
    rm -rf "$SWEEP_DIR"
fi

echo
echo "--- 6. cyclonedds package ---"
if docker exec -u root "$CONTAINER" python3 -c "import cyclonedds" >/dev/null 2>&1; then
    VER=$(docker exec -u root "$CONTAINER" python3 -c "import cyclonedds; print(cyclonedds.__version__)" 2>/dev/null)
    echo "  PASS: installed (version $VER)"
else
    echo "  FAIL: not importable"
    echo "  fix:  docker exec -u root $CONTAINER pip3 install cyclonedds"
fi

echo
echo "--- 7. arm control script ---"
if docker exec -u root "$CONTAINER" test -f /tmp/d1_arm_control.py; then
    echo "  PASS: /tmp/d1_arm_control.py present in container"
else
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    NODES="$(dirname "$SCRIPT_DIR")/nodes"
    if [ -f "$NODES/d1_arm_control.py" ]; then
        docker cp "$NODES/d1_arm_control.py" "$CONTAINER:/tmp/d1_arm_control.py"
        echo "  PASS: copied from $NODES to container"
    else
        echo "  FAIL: d1_arm_control.py not found on host in $NODES"
    fi
fi

echo
echo "--- 8. DDS feedback test ---"
DDS_OUT=$(docker exec -u root "$CONTAINER" bash -c \
    "source /opt/ros/humble/setup.bash 2>/dev/null || true; \
     source /ws/install/setup.bash 2>/dev/null || true; \
     python3 /tmp/d1_arm_control.py --iface $IFACE --verbose feedback 2>&1" \
    2>&1 || true)

echo "$DDS_OUT" | sed 's/^/  /'

# FIXED: Now looks for the JSON dictionary bracket '{' from the updated Python script
if echo "$DDS_OUT" | grep -q "joint feedback: None"; then
    echo "  WARN: DDS ran but arm did not reply. Check power and DDS XML mapping."
elif echo "$DDS_OUT" | grep -q "joint feedback: {"; then
    echo "  PASS: arm replied!"
else
    echo "  FAIL: DDS test crashed — see output above"
fi

echo
echo "=== done ==="