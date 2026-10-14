#!/usr/bin/env bash
# test_explore.sh — diagnose the explorer without the RViz button.
set -eo pipefail

CONTAINER="${CONTAINER_NAME:-go2-lidar3d}"

ros2sh() {
    docker exec -u root "$CONTAINER" bash -lc "
        source /opt/ros/humble/setup.bash 2>/dev/null
        source /ws/install/setup.bash 2>/dev/null
        $1
    "
}

if [ "${1:-}" = "--stop" ]; then
    ros2sh "ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: false}'"
    echo "[explore] stop sent"
    exit 0
fi

echo "=== Explorer diagnostic ==="
echo

echo "[1] Node present?"
ros2sh "timeout 5 ros2 node list" 2>/dev/null | grep -q frontier_explorer \
    && echo "    OK — frontier_explorer running" \
    || { echo "    MISSING — launch with ./campus_session.sh"; exit 1; }

echo
echo "[2] Map publisher?"
ros2sh "timeout 3 ros2 topic info /global_costmap/costmap" 2>/dev/null | \
    grep -q "Publisher count: 1" \
    && echo "    OK" \
    || echo "    FAIL — /global_costmap/costmap not publishing"

echo
echo "[3] Before toggle:"
ros2sh "timeout 3 ros2 topic echo /explore/status --once" 2>/dev/null || true

echo
echo "[4] Publishing Bool(true)..."
ros2sh "ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: true}'"
sleep 1

echo
echo "[5] Watching 15s..."
STATUS=$(ros2sh "timeout 15 ros2 topic echo /explore/status" 2>/dev/null || true)
echo "$STATUS" | head -20

echo
echo "[6] Verdict"
if echo "$STATUS" | grep -q "state=EXPLORING" && echo "$STATUS" | grep -qP "goals_sent=[1-9]"; then
    echo "    ✅ WORKING — explorer produced goals"
elif echo "$STATUS" | grep -q "state=EXPLORING"; then
    echo "    ⚠  EXPLORING but no goals — map may be too sparse"
else
    echo "    ❌ Did not reach EXPLORING"
fi

echo
echo "To stop: ./test_explore.sh --stop"