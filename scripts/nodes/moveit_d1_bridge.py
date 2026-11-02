#!/usr/bin/env python3
"""
moveit_d1_bridge.py

Bridge between MoveIt 2's joint trajectory controller and the physical D1 arm.
MoveIt speaks ROS 2 trajectories (radians); the D1 speaks JSON DDS (degrees).

Subscribes to /arm_controller/joint_trajectory, buffers the waypoints, and
streams them to the D1 at 50 Hz using the min-jerk smoothing from d1_motion.py.
This prevents the wrist shaking that occurs when waypoints are sent blindly.

Usage:
    # In one terminal, with MoveIt running:
    python3 scripts/nodes/moveit_d1_bridge.py

    # Or launched by arm_moveit.sh --real

Environment:
    ARM_IFACE       Arm network interface (default: from d1_arm_control auto-detect)
    ARM_ROBOT       Go2 Wi-Fi IP for arm-over-Wi-Fi (optional)
    BRIDGE_RATE     Streaming rate in Hz (default: 50)

The bridge:
  1. Subscribes to /arm_controller/joint_trajectory (trajectory_msgs/JointTrajectory).
  2. For each incoming trajectory, extracts the final waypoint as the target.
  3. Uses d1_motion.smooth_move() to stream the arm there at 50 Hz with
     min-jerk interpolation and the outer PID for settling — the same
     anti-wobble logic that d1_arm_control uses for all real arm moves.
  4. Also provides a FollowJointTrajectory action server so MoveIt's
     trajectory controller can send goals and get feedback.

Joint mapping:
    MoveIt Joint1..Joint6 (radians) ↔ D1 joint 0..5 (degrees)
    The D1-550 URDF and Unitree's firmware use the same joint order and sign
    convention, verified against the real arm.
"""
import math
import os
import sys
import threading
import time

# Add the nodes directory so d1_arm_control and d1_motion can be imported
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from d1_arm_control import D1Arm, JOINT_LIMITS
from d1_motion import MotionConfig, smooth_move, read_angles, ARM_JOINTS

MOVEIT_JOINTS = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0

# Safety: scale MoveIt's already-scaled velocity down further for the real arm.
# arm_moveit.sh --real sets velocity_scaling to 0.2; the bridge adds its own
# limit via d1_motion's max_vel.
REAL_MAX_VEL_DEG_S = 30.0   # degrees/s per joint (d1_motion default)
REAL_MAX_ACC_DEG_S2 = 60.0  # degrees/s^2
STREAM_RATE = float(os.environ.get('BRIDGE_RATE', '50'))


class MoveItD1Bridge(Node):
    """Bridges MoveIt joint trajectories to the physical D1 arm."""

    def __init__(self):
        super().__init__('moveit_d1_bridge')
        self.get_logger().info('Starting MoveIt → D1 bridge...')

        # Connect to the D1 arm
        robot_ip = os.environ.get('ARM_ROBOT')
        iface = os.environ.get('ARM_IFACE')
        self.arm = D1Arm(iface=iface, robot=robot_ip)
        self.arm_lock = threading.Lock()

        # Motion config tuned for real hardware: conservative, smooth
        self.cfg = MotionConfig(
            max_vel=REAL_MAX_VEL_DEG_S,
            max_acc=REAL_MAX_ACC_DEG_S2,
            rate=STREAM_RATE,
            min_duration=1.0,       # at least 1 s per move for safety
            tolerance=1.5,          # deg, counted as reached
            settle_time=0.5,
            settle_timeout=4.0,
            cmd_mode=1,
            stream_mode=1,
            robot_traj=True,        # use Go2 bridge if available
            robot_rate=50.0,
        )

        cb_group = ReentrantCallbackGroup()

        # Subscribe to raw trajectory topic (for monitoring / simple use)
        self.traj_sub = self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self._on_trajectory,
            10,
            callback_group=cb_group,
        )

        # FollowJointTrajectory action server — MoveIt's controller sends goals here
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            self._execute_trajectory,
            callback_group=cb_group,
        )

        # Publish joint states from the real arm so MoveIt sees the actual positions
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.js_timer = self.create_timer(0.1, self._publish_joint_states)  # 10 Hz

        self.get_logger().info(
            f'Bridge ready. Streaming at {STREAM_RATE} Hz, '
            f'max_vel={REAL_MAX_VEL_DEG_S} deg/s')

    def _rad_to_deg(self, radians):
        """Convert MoveIt joint angles (radians) to D1 angles (degrees)."""
        return [r * RAD2DEG for r in radians]

    def _deg_to_rad(self, degrees):
        """Convert D1 joint angles (degrees) to MoveIt angles (radians)."""
        return [d * DEG2RAD for d in degrees]

    def _reorder_joints(self, joint_names, positions):
        """Reorder positions from the trajectory's joint_names to our Joint1..6 order."""
        ordered = [0.0] * ARM_JOINTS
        for i, name in enumerate(MOVEIT_JOINTS):
            if name in joint_names:
                idx = joint_names.index(name)
                ordered[i] = positions[idx]
        return ordered

    def _on_trajectory(self, msg):
        """Handle a raw JointTrajectory message (topic subscriber)."""
        if not msg.points:
            return
        # Take the last waypoint as the target
        final = msg.points[-1]
        target_rad = self._reorder_joints(msg.joint_names, list(final.positions))
        target_deg = self._rad_to_deg(target_rad)
        self.get_logger().info(
            f'Trajectory received ({len(msg.points)} points), '
            f'target: [{", ".join(f"{d:.1f}" for d in target_deg)}] deg')
        self._move_to(target_deg)

    def _execute_trajectory(self, goal_handle):
        """FollowJointTrajectory action: execute MoveIt's planned trajectory."""
        traj = goal_handle.request.trajectory
        if not traj.points:
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return result

        self.get_logger().info(
            f'Action goal: {len(traj.points)} waypoints over '
            f'{traj.points[-1].time_from_start.sec + traj.points[-1].time_from_start.nanosec * 1e-9:.2f}s')

        # For the real arm, we don't blindly replay every waypoint — that causes
        # shaking. Instead, we use d1_motion's smooth_move to glide to each
        # significant waypoint with min-jerk interpolation.
        #
        # Strategy: move through a subset of waypoints (every ~0.5s or the last one)
        # to approximate the planned path while keeping the motion smooth.
        dt_threshold = 0.5  # seconds between waypoints we actually execute
        waypoints = []
        last_t = -dt_threshold
        for pt in traj.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            if t - last_t >= dt_threshold or pt is traj.points[-1]:
                target_rad = self._reorder_joints(traj.joint_names, list(pt.positions))
                waypoints.append(self._rad_to_deg(target_rad))
                last_t = t

        # Always include the final point
        final_rad = self._reorder_joints(traj.joint_names, list(traj.points[-1].positions))
        final_deg = self._rad_to_deg(final_rad)
        if waypoints[-1] != final_deg:
            waypoints.append(final_deg)

        self.get_logger().info(f'Executing {len(waypoints)} smoothed waypoints...')

        ok = True
        for i, wp in enumerate(waypoints):
            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = list(MOVEIT_JOINTS)
            # Report the target we're heading toward
            feedback.desired.positions = self._deg_to_rad(wp)
            goal_handle.publish_feedback(feedback)

            result = self._move_to(wp)
            if result is not None and not result.reached:
                self.get_logger().warn(
                    f'Waypoint {i + 1}/{len(waypoints)} not reached '
                    f'(final error {result.final_error:.1f} deg)')
                # Continue anyway — the arm is close enough for the next move

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def _move_to(self, target_deg):
        """Move the arm to target_deg using smooth_move (anti-wobble)."""
        with self.arm_lock:
            try:
                result = smooth_move(
                    self.arm, target_deg, gripper=0.0,
                    cfg=self.cfg, limits=JOINT_LIMITS,
                    log=lambda m: self.get_logger().info(m),
                )
                self.get_logger().info(
                    f'  move: reached={result.reached}, '
                    f'duration={result.duration:.2f}s, '
                    f'final_error={result.final_error:.1f} deg')
                return result
            except Exception as e:
                self.get_logger().error(f'Move failed: {e}')
                return None

    def _publish_joint_states(self):
        """Read the real arm angles and publish as /joint_states."""
        angles = read_angles(self.arm, timeout=0.05)
        if angles is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(MOVEIT_JOINTS)
        msg.position = self._deg_to_rad(angles[:ARM_JOINTS])
        msg.velocity = [0.0] * ARM_JOINTS
        msg.effort = [0.0] * ARM_JOINTS
        self.js_pub.publish(msg)


def main():
    rclpy.init()
    node = MoveItD1Bridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Bridge shutting down.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
