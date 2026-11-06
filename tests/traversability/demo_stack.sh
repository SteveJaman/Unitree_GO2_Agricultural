#!/bin/bash
# The perception stack for run_demo.sh, inside go2-lidar3d: no Nav2.
MODE="${1:-baseline}"
OUT="${2:-/tmp/trav_out/demo}"
T=/tmp/trav
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
export AMENT_PREFIX_PATH="/ws/install/go2_nav2:$AMENT_PREFIX_PATH"
export PATH="/ws/install/go2_nav2/lib/go2_nav2:$PATH"
export PYTHONPATH="/ws/install/go2_nav2/lib/python3.10/site-packages:$PYTHONPATH"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///ws/log/3d/cyclonedds.xml ROS_DOMAIN_ID=64
[ "$MODE" = traversability ] || exit 0
ros2 launch go2_nav2 lidar3d_robot_viz.launch.py > "$OUT/viz.log" 2>&1 &
ros2 run go2_nav2 go2_odom_tf --ros-args -r __node:=lidar3d_body_odom \
    -p odom_topic:=/lidar3d/odom -p odom_frame:=lidar3d_map -p base_frame:=base_link \
    -p best_effort:=true -p smooth_alpha:=1.0 -p sensor_frame:=lidar3d_sensor \
    -p "sensor_from_base:=[0.267276, 0.0, -0.120571, 0.0, -0.991341, 0.0, 0.131316]" \
    -p publish_odom:=true -p derive_planar_twist:=true -p publish_tf:=false \
    -p odom_out_topic:=/lidar3d/body_odom > "$OUT/body_odom.log" 2>&1 &
source "$T/start_extra_traversability.sh"
wait
