#!/usr/bin/env bash
# campus_session.sh — verify, tune, patch, launch, monitor the explorer.
#
# Run this ON THE VM after standing the dog and launching
# mapping.sh --3d --nav in another terminal.
set -eo pipefail

CONTAINER="${CONTAINER_NAME:-go2-lidar3d}"
EXPLORER_SRC="${EXPLORER_SRC:-$HOME/files/Unitree_GO2_Agricultural/scripts/nodes/frontier_explorer.py}"
CONTAINER_EXPLORER="/tmp/frontier_explorer.py"
CONTAINER_LOG="/tmp/frontier.log"
MONITOR_SECONDS=30

# shellcheck source=../runtime/go2_net.sh
source "$(dirname "${BASH_SOURCE[0]}")/../runtime/go2_net.sh"
EXPECTED_IP="${GO2_HOST_IP:-}"
go2_detect_host_ip 2>/dev/null && EXPECTED_IP="${GO2_HOST_IP}"

c_ok()   { printf "  \033[32mPASS\033[0m  %s\n" "$1"; }
c_fail() { printf "  \033[31mFAIL\033[0m  %s\n" "$1"; }
c_warn() { printf "  \033[33mWARN\033[0m  %s\n" "$1"; }

ros2sh() {
    docker exec -u root "$CONTAINER" bash -lc "
        source /opt/ros/humble/setup.bash 2>/dev/null
        source /ws/install/setup.bash 2>/dev/null
        $1
    "
}

if [ "${1:-}" = "--stop" ]; then
    ros2sh "ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: false}'" 2>/dev/null || true
    docker exec -u root "$CONTAINER" pkill -9 -f frontier_explorer.py 2>/dev/null || true
    echo "[stop] Done."
    exit 0
fi

echo
echo "================================================================"
echo " Campus Session"
echo "================================================================"
echo

echo "--- 1. Container ---"
docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" \
    && c_ok "container $CONTAINER running" \
    || { c_fail "container not running"; exit 1; }

HOST_IP=$(docker exec "$CONTAINER" printenv GO2_HOST_IP 2>/dev/null || echo "?")
[ -n "$EXPECTED_IP" ] && [ "$HOST_IP" = "$EXPECTED_IP" ] \
    && c_ok "GO2_HOST_IP = $HOST_IP" \
    || c_warn "GO2_HOST_IP = $HOST_IP (${GO2_IFACE} is ${EXPECTED_IP:-unknown})"

echo
echo "--- 2. Costmap discovery ---"
COSTMAP_TOPIC=""
for t in /global_costmap/costmap /global_costmap/costmap_raw /local_costmap/costmap; do
    if ros2sh "ros2 topic info $t" 2>/dev/null | grep -q "Publisher count: 1"; then
        COSTMAP_TOPIC="$t"; break
    fi
done

if [ -z "$COSTMAP_TOPIC" ]; then
    c_fail "no costmap topic found"
    ros2sh "ros2 topic list" 2>/dev/null | grep -i costmap | sed 's/^/         /' || true
    exit 1
fi
c_ok "costmap topic: $COSTMAP_TOPIC"

HZ=$(ros2sh "timeout 5 ros2 topic hz $COSTMAP_TOPIC" 2>/dev/null | \
     grep -oP 'average rate: \K[0-9.]+' | head -1 || echo "")
[ -n "$HZ" ] && c_ok "publishing at ${HZ} Hz" || c_warn "no messages in 5s"

echo
echo "--- 3. Costmap tuning ---"
for node in /global_costmap/global_costmap /local_costmap/local_costmap; do
    ros2sh "ros2 param set $node inflation_radius 0.35" >/dev/null 2>&1 \
        && c_ok "$node inflation_radius=0.35" \
        || c_warn "$node inflation_radius not set"
    ros2sh "ros2 param set $node footprint_padding 0.02" >/dev/null 2>&1 \
        && c_ok "$node footprint_padding=0.02" \
        || c_warn "$node footprint_padding not set"
done

[ "${1:-}" = "--check" ] && { echo; echo "[check] Done."; exit 0; }

echo
echo "--- 4. Explorer source ---"
[ -f "$EXPLORER_SRC" ] || { c_fail "not found: $EXPLORER_SRC"; exit 1; }
c_ok "$EXPLORER_SRC"

echo
echo "--- 5. Launch explorer ---"
docker exec -u root "$CONTAINER" pkill -9 -f frontier_explorer.py 2>/dev/null || true
sleep 1
docker cp "$EXPLORER_SRC" "$CONTAINER:$CONTAINER_EXPLORER" >/dev/null
c_ok "copied to container"

docker exec -u root -d "$CONTAINER" bash -lc "
    source /opt/ros/humble/setup.bash
    source /ws/install/setup.bash 2>/dev/null
    exec python3 $CONTAINER_EXPLORER --ros-args \
        -p map_frame:=lidar3d_map \
        -p map_topic:=$COSTMAP_TOPIC \
        -p goal_topic:=/goal_pose \
        -p cmd_vel_topic:=/cmd_vel_nav \
        -p stop_topic:=/cmd_vel \
        -p blind_enable:=false \
        -p coverage_enable:=false \
        -p return_to_init:=false \
        -p min_frontier_size:=4
" > /tmp/explorer_stdout.log 2>&1
sleep 2

docker exec "$CONTAINER" pgrep -f frontier_explorer.py >/dev/null \
    && c_ok "explorer process alive" \
    || { c_fail "explorer died on start"; exit 1; }

echo
echo "--- 6. Enable ---"
ros2sh "ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: true}'" >/dev/null 2>&1
c_ok "published Bool(true)"

echo
echo "--- 7. Monitoring ${MONITOR_SECONDS}s ---"
DEADLINE=$(( $(date +%s) + MONITOR_SECONDS ))
BEST=0
while [ "$(date +%s)" -lt "$DEADLINE" ]; do
    STATUS=$(ros2sh "timeout 2 ros2 topic echo /explore/status --once" 2>/dev/null \
             | grep -oP "state=\S+.*" | head -1 || echo "")
    [ -n "$STATUS" ] && echo "  $STATUS"
    SENT=$(echo "$STATUS" | grep -oP 'goals_sent=\K[0-9]+' || echo 0)
    [ "$SENT" -gt "$BEST" ] && BEST=$SENT
    sleep 2
done

echo
echo "--- 8. Explorer log (last 15) ---"
docker exec "$CONTAINER" tail -15 /tmp/explorer_stdout.log 2>/dev/null | sed 's/^/  /' || true

echo
echo "================================================================"
if [ "$BEST" -gt 0 ]; then
    echo " \033[32mEXPLORER WORKING\033[0m — $BEST goal(s) sent"
else
    echo " \033[33mNO GOALS in ${MONITOR_SECONDS}s\033[0m"
    echo "   Check the log above for detect_frontiers output."
fi
echo " Stop with: ./campus_session.sh --stop"
echo "================================================================"