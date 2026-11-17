#!/bin/bash
# Sends each goal, then waits until the dog is within 0.3 m of it (max 5 min).
pos() {
  docker exec go2-lidar3d bash -lc 'source /opt/ros/humble/setup.bash; ros2 topic echo /lidar3d/odom --once --field pose.pose.position' 2>/dev/null \
    | awk '/^x:/{x=$2} /^y:/{y=$2} END{print x, y}'
}
for g in "${@:-"-1.6 -0.7" "-0.9 -0.85" "-2.8 0.0"}"; do :; done
GOALS=("-1.6 -0.7" "-0.9 -0.85" "-2.8 0.0")
[ $# -gt 0 ] && GOALS=("$@")
for g in "${GOALS[@]}"; do
  set -- $g
  echo "goal $1 $2"
  docker exec go2-lidar3d bash -lc "source /opt/ros/humble/setup.bash; ros2 topic pub -t 3 -r 1 /goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: lidar3d_map}, pose: {position: {x: $1, y: $2, z: 0.0}, orientation: {w: 1.0}}}'" > /dev/null
  for i in $(seq 1 60); do
    sleep 5
    read px py <<< "$(pos)"
    d=$(awk -v a="$px" -v b="$py" -v gx="$1" -v gy="$2" 'BEGIN{print sqrt((a-gx)^2+(b-gy)^2)}')
    echo "  at ($px, $py), ${d} m from goal"
    awk -v d="$d" 'BEGIN{exit !(d<0.3)}' && break
  done
  sleep 10
done
echo "all goals done"
