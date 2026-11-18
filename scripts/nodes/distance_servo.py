#!/usr/bin/env python3
"""Standalone test: hold the D1 arm's tool at a set distance from a tracked object.

Subscribes to /object_distance (std_msgs/Float32, metres, from yolo_depth_tracker.py) and
moves the tool straight along the arm base's X axis (base_link: +x ahead of the arm,
d1_go2.urdf.xacro) in small Cartesian steps:

    error = distance - target_distance          (target 0.50 m)
    |error| <= deadband (0.02 m)  -> no move
    error > 0 (object too far)    -> +X, toward it
    error < 0 (object too close)  -> -X, away from it
    step = kp * error, its size clamped to [min_step, max_step] (1-2 cm)

Each step: the tool pose now (TF base_link -> tool_link), the same pose shifted in x,
/compute_cartesian_path (MoveIt, collision-checked), then /execute_trajectory. One
step at a time; the next starts once the last has finished.

Safety:
  - No command unless the latest distance is fresh (< stale_s old) and within
    [min_distance, max_distance] (0.20-0.80 m). This is checked again just before a
    planned step is executed, and a step still moving is cancelled when the distance
    goes stale or out of range.
  - The tool never goes more than max_back / max_forward (0.10 / 0.15 m) along X from
    where it was when the node started: the stop if the arm's motion does not change the
    measured distance (camera fixed on the dog, not on the arm).
  - The path must be planned in full (fraction >= 0.99), or the step is dropped.
  - dry_run:=true plans the steps but executes nothing.

Start MoveIt first (arm_moveit.sh, domain 78) and put the arm in the start pose
yourself; this node does not move it there.

Usage (in MoveIt's environment):
    ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1 python3 scripts/nodes/distance_servo.py \
        --ros-args -p dry_run:=true
"""

import copy
import math
import time

import rclpy
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformException, TransformListener


def step_for(distance, target, deadband, kp, min_step, max_step):
    """Signed X step (m) for a measured distance: 0 inside the deadband."""
    error = distance - target
    if abs(error) <= deadband:
        return 0.0
    size = min(max(abs(kp * error), min_step), max_step)
    return math.copysign(size, error)


class DistanceServo(Node):
    def __init__(self):
        super().__init__("distance_servo")
        d = self.declare_parameter
        d("target_distance", 0.50)
        d("deadband", 0.02)
        d("kp", 0.5)  # m of step per m of error, before clamping
        d("min_step", 0.01)
        d("max_step", 0.02)
        d("min_distance", 0.20)  # hard limits on the measured distance
        d("max_distance", 0.80)
        d("stale_s", 0.5)  # distance older than this = no object
        d("max_forward", 0.15)  # tool travel limits along X from the start pose
        d("max_back", 0.10)
        d("control_hz", 5.0)
        d("velocity_scaling", 0.2)
        d("group", "arm")
        d("frame", "base_link")
        d("tool_link", "tool0")
        d("dry_run", False)
        self.p = lambda n: self.get_parameter(n).value

        self.distance = None
        self.distance_time = 0.0
        self.home_x = None
        self.busy = False
        self.goal_handle = None

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.execute = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.create_subscription(Float32, "/object_distance", self.on_distance, 10)
        self.create_timer(1.0 / float(self.p("control_hz")), self.tick)
        self.get_logger().info(
            f'target {self.p("target_distance"):.2f} m +/- {self.p("deadband"):.2f}, '
            f'limits {self.p("min_distance"):.2f}-{self.p("max_distance"):.2f} m, '
            f'{"DRY RUN" if self.p("dry_run") else "EXECUTING"}'
        )

    # ------------------------------------------------------------------ inputs
    def on_distance(self, msg):
        self.distance = float(msg.data)
        self.distance_time = time.monotonic()

    def safe_distance(self):
        """The latest distance if fresh, finite and inside the hard limits, else None."""
        if self.distance is None or time.monotonic() - self.distance_time > float(
            self.p("stale_s")
        ):
            return None
        dist = self.distance
        if not math.isfinite(dist) or not float(
            self.p("min_distance")
        ) <= dist <= float(self.p("max_distance")):
            self.get_logger().warn(
                f"distance {dist:.3f} m outside the limits: no motion",
                throttle_duration_sec=2.0,
            )
            return None
        return dist

    def tool_pose(self):
        try:
            t = self.tf.lookup_transform(self.p("frame"), self.p("tool_link"), Time())
        except TransformException as e:
            self.get_logger().warn(f"no tool pose yet: {e}", throttle_duration_sec=5.0)
            return None
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        )
        pose.orientation = t.transform.rotation
        return pose

    # ------------------------------------------------------------------ control
    def tick(self):
        dist = self.safe_distance()
        if self.busy:
            if dist is None and self.goal_handle is not None:
                self.get_logger().warn(
                    "distance lost or out of range: cancelling the step"
                )
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
            return
        if dist is None:
            return
        pose = self.tool_pose()
        if pose is None:
            return
        if self.home_x is None:
            self.home_x = pose.position.x
            self.get_logger().info(
                f'start pose: tool x = {self.home_x:.3f} m in {self.p("frame")}'
            )

        step = step_for(
            dist,
            float(self.p("target_distance")),
            float(self.p("deadband")),
            float(self.p("kp")),
            float(self.p("min_step")),
            float(self.p("max_step")),
        )
        if step == 0.0:
            return
        lo = self.home_x - float(self.p("max_back"))
        hi = self.home_x + float(self.p("max_forward"))
        new_x = min(max(pose.position.x + step, lo), hi)
        if abs(new_x - pose.position.x) < 0.002:
            self.get_logger().warn(
                f"tool at its X travel limit ({lo:.3f}-{hi:.3f} m): no motion",
                throttle_duration_sec=2.0,
            )
            return
        if not self.cartesian.service_is_ready() or not self.execute.server_is_ready():
            self.get_logger().warn(
                "MoveIt not ready (/compute_cartesian_path, /execute_trajectory)",
                throttle_duration_sec=5.0,
            )
            return

        target = copy.deepcopy(pose)
        target.position.x = new_x
        req = GetCartesianPath.Request()
        req.header.frame_id = self.p("frame")
        req.header.stamp = self.get_clock().now().to_msg()
        req.start_state = RobotState(is_diff=True)
        req.group_name = self.p("group")
        req.link_name = self.p("tool_link")
        req.waypoints = [target]
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = float(self.p("velocity_scaling"))
        req.max_acceleration_scaling_factor = float(self.p("velocity_scaling"))
        self.busy = True
        self.get_logger().info(
            f"distance {dist:.3f} m: tool x {pose.position.x:.3f} -> {new_x:.3f}"
        )
        self.cartesian.call_async(req).add_done_callback(self.on_path)

    def on_path(self, future):
        res = future.result()
        if (
            res is None
            or res.error_code.val != MoveItErrorCodes.SUCCESS
            or res.fraction < 0.99
        ):
            frac = res.fraction if res is not None else 0.0
            self.get_logger().warn(
                f"Cartesian step not planned in full ({frac:.0%}): dropped"
            )
            self.busy = False
            return
        if self.p("dry_run"):
            n = len(res.solution.joint_trajectory.points)
            self.get_logger().info(f"dry run: step planned ({n} points), not executed")
            self.busy = False
            return
        if self.safe_distance() is None:  # changed while planning
            self.busy = False
            return
        goal = ExecuteTrajectory.Goal(trajectory=res.solution)
        self.execute.send_goal_async(goal).add_done_callback(self.on_goal)

    def on_goal(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn("trajectory rejected")
            self.busy = False
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.on_done)

    def on_done(self, future):
        result = future.result()
        code = result.result.error_code.val if result is not None else None
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(f"step ended with MoveIt error code {code}")
        self.goal_handle = None
        self.busy = False


def main():
    rclpy.init()
    node = DistanceServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.goal_handle is not None:
            node.goal_handle.cancel_goal_async()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
