#!/bin/bash
# One navigation trial inside the go2-lidar3d container.
#   run_trial.sh baseline|traversability OUT_DIR
# Files are expected in /tmp/trav (see run_all.sh). Runs on wall-clock time:
# the sim has no /clock publisher.
MODE="${1:-baseline}"
OUT="${2:-/tmp/trav_out}"
T=/tmp/trav
mkdir -p "$OUT"

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
export AMENT_PREFIX_PATH="/ws/install/go2_nav2:$AMENT_PREFIX_PATH"
export PATH="/ws/install/go2_nav2/lib/go2_nav2:$PATH"
export PYTHONPATH="/ws/install/go2_nav2/lib/python3.10/site-packages:$PYTHONPATH"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///ws/log/3d/cyclonedds.xml
export ROS_DOMAIN_ID=64

STACK='lidar3d_robot_viz|robot_description|robot_state_publisher|go2_joint_state_bridge|go2_urdf_self_filter|go2_odom_tf|planner_server|controller_server|velocity_smoother|lifecycle_manager|go2_goal_pose_nav|go2_cmd_vel_tcp|object_box_fusion|gait_supervisor|sim_box_detector|save_frames'
stop_stack() {
    pkill -TERM -f "$STACK" 2>/dev/null
    sleep 2
    pkill -KILL -f "$STACK" 2>/dev/null
    sleep 1
}
trap stop_stack EXIT
stop_stack

ros2 launch go2_nav2 lidar3d_robot_viz.launch.py > "$OUT/viz.log" 2>&1 &
sleep 4

ros2 run go2_nav2 go2_odom_tf --ros-args -r __node:=lidar3d_body_odom \
    -p odom_topic:=/lidar3d/odom -p odom_frame:=lidar3d_map -p base_frame:=base_link \
    -p best_effort:=true -p smooth_alpha:=1.0 -p sensor_frame:=lidar3d_sensor \
    -p "sensor_from_base:=[0.267276, 0.0, -0.120571, 0.0, -0.991341, 0.0, 0.131316]" \
    -p publish_odom:=true -p derive_planar_twist:=true -p publish_tf:=false \
    -p odom_out_topic:=/lidar3d/body_odom > "$OUT/body_odom.log" 2>&1 &

ros2 run nav2_planner planner_server --ros-args --params-file "$T/planner.yaml" \
    > "$OUT/planner.log" 2>&1 &
ros2 run nav2_controller controller_server --ros-args \
    --params-file "$T/controller_${MODE}.yaml" -r cmd_vel:=cmd_vel_nav \
    > "$OUT/controller.log" 2>&1 &
ros2 run nav2_velocity_smoother velocity_smoother --ros-args \
    -r cmd_vel:=cmd_vel_nav -r cmd_vel_smoothed:=cmd_vel \
    -p smoothing_frequency:=20.0 -p scale_velocities:=true -p feedback:=OPEN_LOOP \
    -p "max_velocity:=[0.30, 0.0, 0.70]" -p "min_velocity:=[-0.30, 0.0, -0.70]" \
    -p "max_accel:=[0.5, 0.0, 0.8]" -p "max_decel:=[-0.5, 0.0, -0.8]" \
    -p "deadband_velocity:=[0.03, 0.0, 0.08]" -p velocity_timeout:=1.0 \
    > "$OUT/smoother.log" 2>&1 &
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_planning \
    -p autostart:=true -p "node_names:=[planner_server]" -p bond_timeout:=10.0 \
    > "$OUT/lm_plan.log" 2>&1 &
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_controller \
    -p autostart:=true -p "node_names:=[controller_server, velocity_smoother]" -p bond_timeout:=4.0 \
    > "$OUT/lm_ctrl.log" 2>&1 &
ros2 run go2_nav2 go2_goal_pose_nav --ros-args \
    -p map_frame:=lidar3d_map -p pre_rotate_enable:=false > "$OUT/goal_nav.log" 2>&1 &
python3 /ws/scripts/go2_cmd_vel_tcp.py --role client --host 127.0.0.1 --port 17999 \
    > "$OUT/cmd_tcp.log" 2>&1 &

if [ -f "$T/start_extra_${MODE}.sh" ]; then
    source "$T/start_extra_${MODE}.sh"
fi

sleep 20
python3 "$T/nav_trial_runner.py" --mode "$MODE" --out "$OUT" "${@:3}" 2>&1 | tail -12
