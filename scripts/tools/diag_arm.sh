#!/usr/bin/env bash
# diag_arm.sh — full network + DDS diagnostic for the D1-T arm.
#
# Run on the VM. Prints a checklist, ends with a verdict.
#
# Usage:
#   ./scripts/tools/diag_arm.sh                    # default iface enp0s8
#   ARM_IFACE=enp0s3 ./scripts/tools/diag_arm.sh
#   CONTAINER_NAME=go2-lidar3d ./scripts/tools/diag_arm.sh
set -uo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-go2-lidar3d}"
ARM_IFACE="${ARM_IFACE:-enp0s8}"
ARM_SUBNET="${ARM_SUBNET:-192.168.123}"
# D1 arm's factory default. If the VM squats on it, the arm can't come online.
D1_DEFAULT_IP="${D1_DEFAULT_IP:-192.168.123.100}"

# Known Go2 internal MACs (from earlier neighbor table)
KNOWN_GO2_MACS=("48:b0:2d" "7e:1d:75")

# ---------------------------------------------------------------------------
# Output helpers
# ---------------------------------------------------------------------------
PASS=0
FAIL=0
WARN=0
VERDICT_LINES=()

ok()    { printf "  \033[32m[OK]\033[0m    %s\n" "$1"; PASS=$((PASS+1)); }
bad()   { printf "  \033[31m[FAIL]\033[0m  %s\n" "$1"; FAIL=$((FAIL+1)); VERDICT_LINES+=("$1"); }
warn()  { printf "  \033[33m[WARN]\033[0m  %s\n" "$1"; WARN=$((WARN+1)); }
info()  { printf "  \033[36m[..]\033[0m    %s\n" "$1"; }
hdr()   { printf "\n\033[1m=== %s ===\033[0m\n" "$1"; }

# ---------------------------------------------------------------------------
# 1. Container
# ---------------------------------------------------------------------------
hdr "1. Container"

if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    ok "container $CONTAINER_NAME running"
else
    bad "container $CONTAINER_NAME NOT running"
    echo
    echo "Fix:"
    echo "  docker start $CONTAINER_NAME"
    echo "  (or: cd ~/files/go2-nav2-wifi && ./setup-lidar3d.sh --laptop)"
    exit 1
fi

# Check container sees ARM_IFACE (host networking)
if docker exec -u root "$CONTAINER_NAME" ip link show "$ARM_IFACE" >/dev/null 2>&1; then
    ok "container sees interface $ARM_IFACE"
else
    bad "container cannot see interface $ARM_IFACE"
    echo "  Interfaces inside container:"
    docker exec -u root "$CONTAINER_NAME" ip -brief link | sed 's/^/    /'
    exit 1
fi

# ---------------------------------------------------------------------------
# 2. Interface state
# ---------------------------------------------------------------------------
hdr "2. Interface $ARM_IFACE"

IFACE_LINE=$(ip -brief addr show "$ARM_IFACE" 2>/dev/null)
echo "  $IFACE_LINE"

IFACE_IP=$(echo "$IFACE_LINE" | grep -oP 'inet \K[0-9.]+' | head -1)

if [ -z "$IFACE_IP" ]; then
    bad "$ARM_IFACE has no IPv4 address"
    echo "  Fix:"
    echo "    sudo ip addr add ${ARM_SUBNET}.51/24 dev $ARM_IFACE"
    echo "    sudo ip link set $ARM_IFACE up"
else
    ok "$ARM_IFACE has IP $IFACE_IP"
fi

# ---------------------------------------------------------------------------
# 3. IP conflict with the arm's factory default
# ---------------------------------------------------------------------------
hdr "3. IP conflict check"

if [ "$IFACE_IP" = "$D1_DEFAULT_IP" ]; then
    bad "VM $ARM_IFACE is squatting on $D1_DEFAULT_IP (D1's factory default)"
    echo "  The arm cannot come online while the VM holds its address."
    echo "  Fix:"
    echo "    sudo ip addr del $D1_DEFAULT_IP/24 dev $ARM_IFACE"
    echo "    sudo ip addr add ${ARM_SUBNET}.51/24 dev $ARM_IFACE"
else
    ok "no conflict with $D1_DEFAULT_IP"
fi

# ---------------------------------------------------------------------------
# 4. Neighbor table
# ---------------------------------------------------------------------------
hdr "4. Neighbor table on $ARM_IFACE"

NEIGH=$(ip neigh show dev "$ARM_IFACE" 2>/dev/null)
if [ -z "$NEIGH" ]; then
    warn "empty neighbor table"
    info "run a ping sweep (next step) to populate it"
else
    echo "$NEIGH" | while read -r line; do
        nip=$(echo "$line" | awk '{print $1}')
        nmac=$(echo "$line" | awk '{print $3}')
        vendor="unknown"
        for prefix in "${KNOWN_GO2_MACS[@]}"; do
            if [[ "$nmac" == "$prefix"* ]]; then
                vendor="Go2 internal"
            fi
        done
        echo "    $nip  $nmac  [$vendor]"
    done
fi

# ---------------------------------------------------------------------------
# 5. Subnet sweep
# ---------------------------------------------------------------------------
hdr "5. Sweeping ${ARM_SUBNET}.0/24 for live hosts"

if [ -z "$IFACE_IP" ]; then
    warn "skipping sweep — $ARM_IFACE has no IP"
else
    info "this takes ~30 seconds..."
    FOUND=()
    for i in $(seq 1 254); do
        ip="${ARM_SUBNET}.${i}"
        if ping -c1 -W1 "$ip" >/dev/null 2>&1; then
            FOUND+=("$ip")
        fi
    done

    if [ ${#FOUND[@]} -eq 0 ]; then
        bad "no hosts on $ARM_SUBNET.0/24 responded"
        info "arm may be unpowered, wrong subnet, or cable issue"
    else
        ok "${#FOUND[@]} host(s) up on $ARM_SUBNET.0/24"
        for ip in "${FOUND[@]}"; do
            mac=$(ip neigh show "$ip" 2>/dev/null | awk '{print $5}')
            echo "    $ip  ${mac:-no-mac}"
        done
    fi
fi

# ---------------------------------------------------------------------------
# 6. Cyclonedds in container
# ---------------------------------------------------------------------------
hdr "6. Cyclonedds package"

if docker exec -u root "$CONTAINER_NAME" \
        python3 -c "import cyclonedds" >/dev/null 2>&1; then
    VER=$(docker exec -u root "$CONTAINER_NAME" \
        python3 -c "import cyclonedds; print(cyclonedds.__version__)" 2>/dev/null)
    ok "cyclonedds installed (version $VER)"
else
    bad "cyclonedds not importable"
    echo "  Fix:"
    echo "    docker exec -u root $CONTAINER_NAME pip3 install cyclonedds"
fi

# ---------------------------------------------------------------------------
# 7. Arm control script
# ---------------------------------------------------------------------------
hdr "7. Arm control script"

if [ -f /tmp/d1_arm_control.py ]; then
    ok "/tmp/d1_arm_control.py present in container"
else
    NODES_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../nodes" && pwd)"
    if [ -f "${NODES_DIR}/d1_arm_control.py" ]; then
        info "copying d1_arm_control.py into container..."
        docker cp "${NODES_DIR}/d1_arm_control.py" \
            "$CONTAINER_NAME:/tmp/d1_arm_control.py"
        ok "copied"
    else
        bad "d1_arm_control.py not found"
        exit 1
    fi
fi

# ---------------------------------------------------------------------------
# 8. DDS URI test
# ---------------------------------------------------------------------------
hdr "8. DDS config test (--verbose)"

info "running feedback with --verbose..."

DDS_OUT=$(docker exec -u root "$CONTAINER_NAME" bash -c "
    python3 /tmp/d1_arm_control.py --iface $ARM_IFACE --verbose feedback 2>&1
" 2>&1 || true)

echo "$DDS_OUT" | sed 's/^/    /'

if echo "$DDS_OUT" | grep -q "joint feedback: None"; then
    warn "DDS initialized but arm did NOT reply"
    info "likely cause: CYCLONEDDS_URI missing XML namespace, or arm unpowered"
elif echo "$DDS_OUT" | grep -q "joint feedback: \["; then
    ok "arm replied with joint angles — DDS is working"
else
    bad "DDS test failed"
fi

# ---------------------------------------------------------------------------
# 9. Verdict
# ---------------------------------------------------------------------------
hdr "Verdict"

echo "  Checks passed: $PASS"
echo "  Warnings:      $WARN"
echo "  Failures:      $FAIL"

if [ "$FAIL" -eq 0 ] && [ "$WARN" -eq 0 ]; then
    echo
    echo "  \033[32mAll checks passed.\033[0m"
    echo "  If feedback still shows None, the arm's DDS is not replying — "
    echo "  check that the arm is powered (24V DC) and that the correct"
    echo "  CYCLONEDDS_URI namespace is in use (see d1_arm_control.py)."
elif [ "$FAIL" -gt 0 ]; then
    echo
    echo "  \033[31mFailures to fix:\033[0m"
    for line in "${VERDICT_LINES[@]}"; do
        echo "    - $line"
    done
fi

echo
echo "  Paste this entire output for review."