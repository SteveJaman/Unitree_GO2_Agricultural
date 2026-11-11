#!/bin/bash
# One navigation trial inside the go2-lidar3d container.
#   run_trial.sh baseline|traversability OUT_DIR [--randomize --seed N] [--timeout S]
# Extra arguments go to nav_trial_runner.py; with --randomize, run_all.sh has
# already placed the robot at the seed's spawn.
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

# Every node starts in its own process group (setsid), and its group id goes to
# PIDFILE; stopping kills exactly those groups, never by name.
PIDFILE=/tmp/trav/stack.pids
stop_stack() {
    [ -f "$PIDFILE" ] || return 0
    while read -r pid; do kill -TERM -- "-$pid" 2>/dev/null; done < "$PIDFILE"
    sleep 2
    while read -r pid; do kill -KILL -- "-$pid" 2>/dev/null; done < "$PIDFILE"
    rm -f "$PIDFILE"
    sleep 1
}
bg() { setsid "$@" & echo $! >> "$PIDFILE"; }
trap stop_stack EXIT
stop_stack

bg ros2 launch go2_nav2 lidar3d_robot_viz.launch.py > "$OUT/viz.log" 2>&1
sleep 4

bg ros2 run go2_nav2 go2_odom_tf --ros-args -r __node:=lidar3d_body_odom \
    -p odom_topic:=/lidar3d/odom -p odom_frame:=lidar3d_map -p base_frame:=base_link \
    -p best_effort:=true -p smooth_alpha:=1.0 -p sensor_frame:=lidar3d_sensor \
    -p "sensor_from_base:=[0.267276, 0.0, -0.120571, 0.0, -0.991341, 0.0, 0.131316]" \
    -p publish_odom:=true -p derive_planar_twist:=true -p publish_tf:=false \
    -p odom_out_topic:=/lidar3d/body_odom > "$OUT/body_odom.log" 2>&1

# A mode may bring its own global costmap (traversability_semantic: semantic layer in both).
PLANNER="$T/planner.yaml"
[ -f "$T/planner_${MODE}.yaml" ] && PLANNER="$T/planner_${MODE}.yaml"
bg ros2 run nav2_planner planner_server --ros-args --params-file "$PLANNER" \
    > "$OUT/planner.log" 2>&1
# CONTROLLER_EXTERNAL=1: controller_server runs in another container (MPPI modes,
# started by run_all.sh); the lifecycle manager below still manages it over DDS.
if [ "${CONTROLLER_EXTERNAL:-0}" != 1 ]; then
    bg ros2 run nav2_controller controller_server --ros-args \
        --params-file "$T/controller_${MODE}.yaml" -r cmd_vel:=cmd_vel_nav \
        > "$OUT/controller.log" 2>&1
fi
bg ros2 run nav2_velocity_smoother velocity_smoother --ros-args \
    -r cmd_vel:=cmd_vel_nav -r cmd_vel_smoothed:=cmd_vel \
    -p smoothing_frequency:=20.0 -p scale_velocities:=true -p feedback:=OPEN_LOOP \
    -p "max_velocity:=[0.30, 0.0, 0.70]" -p "min_velocity:=[-0.30, 0.0, -0.70]" \
    -p "max_accel:=[0.5, 0.0, 0.8]" -p "max_decel:=[-0.5, 0.0, -0.8]" \
    -p "deadband_velocity:=[0.03, 0.0, 0.08]" -p velocity_timeout:=1.0 \
    > "$OUT/smoother.log" 2>&1
bg ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_planning \
    -p autostart:=true -p "node_names:=[planner_server]" -p bond_timeout:=10.0 \
    > "$OUT/lm_plan.log" 2>&1
bg ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_controller \
    -p autostart:=true -p "node_names:=[controller_server, velocity_smoother]" -p bond_timeout:=4.0 \
    > "$OUT/lm_ctrl.log" 2>&1
bg ros2 run go2_nav2 go2_goal_pose_nav --ros-args \
    -p map_frame:=lidar3d_map -p pre_rotate_enable:=false > "$OUT/goal_nav.log" 2>&1
bg python3 /ws/scripts/go2_cmd_vel_tcp.py --role client --host 127.0.0.1 --port 17999 \
    > "$OUT/cmd_tcp.log" 2>&1

if [ -f "$T/start_extra_${MODE}.sh" ]; then
    source "$T/start_extra_${MODE}.sh"
fi

sleep 20
# INTERACTIVE=1 (lab_semantic.sh): no scripted goals; the stack runs until stopped,
# goals come from RViz's 2D Goal Pose on /goal_pose.
if [ "${INTERACTIVE:-0}" = 1 ]; then
    echo "STACK READY" > "$OUT/ready"
    wait
    exit 0
fi
python3 "$T/nav_trial_runner.py" --mode "$MODE" --out "$OUT" "${@:3}" 2>&1 | tail -12
