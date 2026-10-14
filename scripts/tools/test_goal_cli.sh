#!/usr/bin/env bash
# Publish a single PoseStamped to /goal_pose from the host.
# usage: ./test_goal_cli.sh X Y
set -eo pipefail
X="${1:?usage: $0 X Y}"
Y="${2:?usage: $0 X Y}"
CONTAINER_NAME="${CONTAINER_NAME:-go2-lidar3d}"

docker exec -u root "$CONTAINER_NAME" bash -lc "
    source /opt/ros/humble/setup.bash
    source /ws/install/setup.bash 2>/dev/null
    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
      \"{header: {frame_id: 'map'}, \
        pose: {position: {x: $X, y: $Y, z: 0.0}, \
               orientation: {w: 1.0}}}\"
    echo 'goal published'
"
