#!/usr/bin/env bash
# preflight.sh — verify the Go2 nav stack before a session.
set -eo pipefail

CONTAINER="${CONTAINER_NAME:-go2-lidar3d}"
PASS=0; FAIL=0

# shellcheck source=../runtime/go2_net.sh
source "$(dirname "${BASH_SOURCE[0]}")/../runtime/go2_net.sh"
EXPECTED_IP="${GO2_HOST_IP:-}"
go2_detect_host_ip 2>/dev/null && EXPECTED_IP="${GO2_HOST_IP}"

c_ok()   { printf "  \033[32mPASS\033[0m  %s\n" "$1"; PASS=$((PASS+1)); }
c_fail() { printf "  \033[31mFAIL\033[0m  %s\n" "$1"; FAIL=$((FAIL+1)); }

ros2sh() {
    docker exec -u root "$CONTAINER" bash -lc "
        source /opt/ros/humble/setup.bash 2>/dev/null
        source /ws/install/setup.bash 2>/dev/null
        $1
    "
}

echo "================================================================"
echo " Go2 Nav2 Preflight"
echo "================================================================"
echo

echo "--- Container ---"
docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" \
    && c_ok "container $CONTAINER" \
    || { c_fail "container not running"; echo; echo "PASS=$PASS FAIL=$FAIL"; exit 1; }

HOST_IP=$(docker exec "$CONTAINER" printenv GO2_HOST_IP 2>/dev/null || echo "?")
[ -n "$EXPECTED_IP" ] && [ "$HOST_IP" = "$EXPECTED_IP" ] \
    && c_ok "GO2_HOST_IP = $HOST_IP" \
    || c_fail "GO2_HOST_IP = $HOST_IP (${GO2_IFACE} is ${EXPECTED_IP:-unknown}; rerun wifi_nav.sh to recreate)"

BIND=$(docker exec "$CONTAINER" grep -oP 'NetworkInterface address="\K[^"]+' \
    /tmp/go2-wifi-cyclonedds.xml 2>/dev/null || echo "?")
[ -n "$EXPECTED_IP" ] && [ "$BIND" = "$EXPECTED_IP" ] \
    && c_ok "DDS bind = $BIND" \
    || c_fail "DDS bind = $BIND"

echo
echo "--- Packages ---"
for pkg in nav2_velocity_smoother nav2_regulated_pure_pursuit_controller; do
    ros2sh "ros2 pkg list" 2>/dev/null | grep -qx "$pkg" \
        && c_ok "$pkg" \
        || c_fail "$pkg missing"
done

echo
echo "--- Nodes ---"
NODES=$(ros2sh "timeout 5 ros2 node list" 2>/dev/null || echo "")
for n in planner_server controller_server velocity_smoother; do
    echo "$NODES" | grep -q "/$n" \
        && c_ok "node /$n" \
        || c_fail "node /$n missing"
done

echo
echo "--- Topics ---"
for t in /lidar3d/registered /cmd_vel_nav /cmd_vel; do
    N=$(ros2sh "timeout 3 ros2 topic info $t" 2>/dev/null | \
        grep -oP 'Publisher count: \K[0-9]+' || echo 0)
    [ "${N:-0}" -gt 0 ] \
        && c_ok "$t pub=$N" \
        || c_fail "$t has no publishers"
done

COSTMAP=""
for t in /global_costmap/costmap /global_costmap/costmap_raw /local_costmap/costmap; do
    if ros2sh "ros2 topic info $t" 2>/dev/null | grep -q "Publisher count: 1"; then
        COSTMAP="$t"; break
    fi
done
[ -n "$COSTMAP" ] && c_ok "costmap: $COSTMAP" || c_fail "no costmap topic"

echo
echo "--- TF ---"
ros2sh "timeout 3 ros2 run tf2_ros tf2_echo lidar3d_map base_link 2>&1 | head -3" \
    2>/dev/null | grep -q "Translation" \
    && c_ok "lidar3d_map -> base_link" \
    || c_fail "TF lookup failed"

echo
echo "================================================================"
echo " PASS=$PASS  FAIL=$FAIL"
echo "================================================================"
[ "$FAIL" -eq 0 ] || exit 1