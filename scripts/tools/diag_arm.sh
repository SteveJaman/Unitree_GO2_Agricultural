#!/usr/bin/env bash
# diag_arm.sh: D1 arm network + DDS diagnostic.
#
# Environment overrides:
#   ARM_IFACE=enp0s8        interface the arm is plugged into (auto-detected if unset)
#   ARM_SUBNET=192.168.123  arm subnet (D1 factory default IP is .100)
#   ARM_IP=192.168.123.100
#   ARM_PEER=...            unicast discovery peer for the DDS test
#   CONTAINER_NAME=go2-lidar3d
set -uo pipefail

CONTAINER="${CONTAINER_NAME:-go2-lidar3d}"
SUBNET="${ARM_SUBNET:-192.168.123}"
ARM_IP="${ARM_IP:-${SUBNET}.99}"
IFACE="${ARM_IFACE:-}"
ARM_PEER="${ARM_PEER:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NODES="${NODES_DIR:-$(dirname "$SCRIPT_DIR")/nodes}"
[ -f "$NODES/d1_arm_control.py" ] || NODES="$SCRIPT_DIR"

pass() { echo "  PASS: $*"; }
fail() { echo "  FAIL: $*"; FAILS=$((FAILS + 1)); }
warn() { echo "  WARN: $*"; }
FAILS=0

echo "=== D1 arm diagnostic ==="
echo "container: $CONTAINER"
echo "subnet:    $SUBNET.x   (arm ip: $ARM_IP)"
echo

# --- 0. what interfaces exist ----------------------------------------------
echo "--- 0. host interfaces ---"
ip -br addr | sed 's/^/  /'
if [ -z "$IFACE" ]; then
    IFACE="$(ip -4 -o addr show | awk -v p="^${SUBNET}\\." '$4 ~ p {print $2; exit}')"
    if [ -n "$IFACE" ]; then
        echo "  auto-detected arm interface: $IFACE"
    else
        fail "no interface has an address in ${SUBNET}.x"
        echo "  The arm must be on its own wired interface (not your home-LAN one)."
        echo "  In VirtualBox: attach the USB Ethernet adapter to the VM, or add a Bridged"
        echo "  adapter bound to that NIC. Then:"
        echo "    sudo ip addr add ${SUBNET}.51/24 dev <iface>"
        echo "    sudo ip link set <iface> up"
        echo "  and re-run:  ARM_IFACE=<iface> $0"
        exit 1
    fi
fi
echo "interface: $IFACE"
echo

# --- 1. container -----------------------------------------------------------
echo "--- 1. container ---"
if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    pass "$CONTAINER running"
else
    fail "$CONTAINER not running (fix: docker start $CONTAINER)"
    exit 1
fi

NETMODE="$(docker inspect -f '{{.HostConfig.NetworkMode}}' "$CONTAINER" 2>/dev/null)"
if [ "$NETMODE" = "host" ]; then
    pass "network mode is host"
else
    fail "network mode is '$NETMODE'; DDS needs --network host (recreate the container)"
fi

if docker exec -u root "$CONTAINER" test -e "/sys/class/net/$IFACE"; then
    pass "container sees $IFACE"
else
    fail "container does not see $IFACE"
fi

# --- 2. interface config ----------------------------------------------------
echo
echo "--- 2. interface config ---"
CARRIER="$(cat "/sys/class/net/$IFACE/carrier" 2>/dev/null || echo 0)"
if [ "$CARRIER" = "1" ]; then
    pass "link is up (cable detected)"
else
    fail "no link/carrier on $IFACE (cable, adapter, or VM attachment problem)"
fi

mapfile -t IPS < <(ip -4 -o addr show dev "$IFACE" | awk '{print $4}' | cut -d/ -f1)
echo "  IPv4 on $IFACE: ${IPS[*]:-none}"
if [ "${#IPS[@]}" -eq 0 ]; then
    fail "$IFACE has no IPv4 (fix: sudo ip addr add ${SUBNET}.51/24 dev $IFACE; sudo ip link set $IFACE up)"
else
    IN_SUBNET=0
    for ip in "${IPS[@]}"; do
        [[ "$ip" == "$SUBNET."* ]] && IN_SUBNET=$((IN_SUBNET + 1))
    done
    if [ "$IN_SUBNET" -eq 0 ]; then
        fail "no address in ${SUBNET}.x on $IFACE (fix: sudo ip addr add ${SUBNET}.51/24 dev $IFACE)"
    else
        pass "has an address in ${SUBNET}.x"
    fi
    [ "${#IPS[@]}" -gt 1 ] && warn "multiple IPv4 addresses on $IFACE; DDS may pick the wrong one"
    for ip in "${IPS[@]}"; do
        [ "$ip" = "$ARM_IP" ] && fail "host is squatting on the arm's IP $ARM_IP"
    done
fi
if ip route show default dev "$IFACE" 2>/dev/null | grep -q default; then
    warn "$IFACE carries your default route, so it is probably your home LAN, not the arm link"
fi

# --- 3. reachability --------------------------------------------------------
echo
echo "--- 3. reach the arm ---"
if ping -c2 -W1 -I "$IFACE" "$ARM_IP" >/dev/null 2>&1; then
    pass "ping $ARM_IP via $IFACE"
    ip neigh show "$ARM_IP" dev "$IFACE" 2>/dev/null | sed 's/^/        /'
else
    fail "no ping reply from $ARM_IP via $IFACE"
    echo "  sweeping ${SUBNET}.0/24 for any live host..."
    SWEEP_DIR="$(mktemp -d)"
    for i in $(seq 1 254); do
        ( ping -c1 -W1 -I "$IFACE" "$SUBNET.$i" >/dev/null 2>&1 && : > "$SWEEP_DIR/$i" ) &
    done
    wait
    if [ -z "$(ls -A "$SWEEP_DIR")" ]; then
        echo "  no hosts responded on $IFACE"
        echo "  Next: sudo tcpdump -i $IFACE -n -c 20   (arm powered on, cable in)."
        echo "        Packets from the arm => link is fine, IP/subnet is the issue."
        echo "        Silence => cable / adapter / VirtualBox attachment."
        echo "        The arm also needs ~1 min after power-on."
    else
        for f in "$SWEEP_DIR"/*; do
            n="$(basename "$f")"
            echo "    UP: $SUBNET.$n  $(ip neigh show "$SUBNET.$n" dev "$IFACE" 2>/dev/null | awk '{print $5}')"
        done
        echo "  If the arm shows up on a different IP, re-run with ARM_IP=<that ip>."
    fi
    rm -rf "$SWEEP_DIR"
fi

# --- 4. cyclonedds ----------------------------------------------------------
echo
echo "--- 4. cyclonedds in container ---"
if docker exec -u root "$CONTAINER" python3 -c "import cyclonedds" >/dev/null 2>&1; then
    VER="$(docker exec -u root "$CONTAINER" pip3 show cyclonedds 2>/dev/null | awk '/^Version:/{print $2}')"
    pass "importable (pip version: ${VER:-unknown})"
    [ "$VER" = "0.10.2" ] || warn "Unitree's SDK uses cyclonedds 0.10.2; you have ${VER:-unknown} (try: pip3 install cyclonedds==0.10.2)"
else
    fail "not importable (fix: docker exec -u root $CONTAINER pip3 install cyclonedds==0.10.2)"
fi

# --- 5. control script ------------------------------------------------------
echo
echo "--- 5. control script ---"
if [ -f "$NODES/d1_arm_control.py" ]; then
    docker cp "$NODES/d1_arm_control.py" "$CONTAINER:/tmp/d1_arm_control.py" \
        && pass "copied latest d1_arm_control.py from $NODES" \
        || fail "docker cp failed"
else
    fail "d1_arm_control.py not found in $NODES"
fi

# --- 6. DDS test ------------------------------------------------------------
echo
echo "--- 6. DDS discovery + feedback ---"
PEER_ARGS=()
[ -n "$ARM_PEER" ] && PEER_ARGS=(--peer "$ARM_PEER")

# No ROS sourcing: the script is pure CycloneDDS and ROS's libddsc can clash.
DDS_OUT="$(docker exec -u root "$CONTAINER" \
    env -u CYCLONEDDS_URI python3 /tmp/d1_arm_control.py \
    --iface "$IFACE" "${PEER_ARGS[@]}" --verbose feedback status 2>&1 || true)"
echo "$DDS_OUT" | sed 's/^/  /'

if echo "$DDS_OUT" | grep -q "DDS match with arm: NO"; then
    fail "arm not discovered over DDS (try ARM_PEER=$ARM_IP $0)"
elif echo "$DDS_OUT" | grep -q "joint feedback: {"; then
    pass "arm replied with joint feedback"
elif echo "$DDS_OUT" | grep -q "DDS match with arm: YES"; then
    warn "matched, but no joint feedback in 5s. Is the arm powered and fully booted?"
else
    fail "DDS test crashed; see output above"
fi

echo
if [ "$FAILS" -eq 0 ]; then
    echo "=== done: all checks passed ==="
else
    echo "=== done: $FAILS check(s) failed ==="
fi
exit "$FAILS"
