#!/usr/bin/env python3
"""orbit_scanner.py: 180 degree active perception sweep of the D1 arm through MoveIt 2.

Thirteen camera poses on a horizontal arc around the plant, -90 to +90 degrees
from the side facing the arm, each aimed at the plant centre. The arm moves to
each pose in turn, settles, and prints a placeholder where the capture will go.
A waypoint that cannot be planned is logged and skipped.

  ros2 launch d1_moveit_config demo.launch.py end_effector:=camera      # terminal 1
  ros2 run d1_active_perception orbit_scanner                           # terminal 2
  ros2 run d1_active_perception orbit_scanner --ros-args -p plan_only:=true

Every number below is a ROS parameter. ee_link must have z forward, x right,
y down: in d1_moveit_config that is camera_optical_frame, not camera_link.
"""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive


def look_at(eye, target):
    """Quaternion (x, y, z, w) of an optical frame at eye looking at target:
    z forward, x right, y down, with the image upright (world up is +z)."""
    forward = np.asarray(target, float) - np.asarray(eye, float)
    forward /= np.linalg.norm(forward)
    up = np.array([0.0, 0.0, 1.0])
    if abs(forward @ up) > 0.99:  # looking straight up or down
        up = np.array([1.0, 0.0, 0.0])
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    return Rotation.from_matrix(np.column_stack([right, down, forward])).as_quat()


def orbit_waypoints(target, radius, start_deg, end_deg, count):
    """Camera positions on a horizontal arc around target, angle 0 facing the origin."""
    target = np.asarray(target, float)
    facing_arm = np.arctan2(-target[1], -target[0])
    points = []
    for deg in np.linspace(start_deg, end_deg, count):
        angle = facing_arm + np.radians(deg)
        points.append(target + radius * np.array([np.cos(angle), np.sin(angle), 0.0]))
    return points


class OrbitScanner(Node):
    def __init__(self):
        super().__init__("orbit_scanner")
        d = self.declare_parameter
        d("planning_group", "arm")
        d("ee_link", "camera_optical_frame")
        d("planning_frame", "base_link")
        d("target", [-0.45, 0.0, 0.85])
        d("radius", 0.35)
        d("start_deg", -90.0)
        d("end_deg", 90.0)
        d("waypoints", 13)
        d("velocity_scaling", 0.15)
        d("acceleration_scaling", 0.15)
        d("planning_time", 2.0)
        d("position_tolerance", 0.01)
        d("orientation_tolerance", 0.1)
        d("settle_s", 1.0)
        d("plan_only", False)
        self.p = lambda name: self.get_parameter(name).value
        self.client = ActionClient(self, MoveGroup, "move_action")

    def wait(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def constraints(self, position, quaternion):
        frame, link = self.p("planning_frame"), self.p("ee_link")
        radius = float(self.p("position_tolerance"))
        tolerance = float(self.p("orientation_tolerance"))
        x, y, z = (float(v) for v in position)
        qx, qy, qz, qw = (float(v) for v in quaternion)

        pc = PositionConstraint()
        pc.header.frame_id = frame
        pc.link_name = link
        pc.constraint_region = BoundingVolume(
            primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[radius])],
            primitive_poses=[
                Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion(w=1.0))
            ],
        )
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = frame
        oc.link_name = link
        oc.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        oc.absolute_x_axis_tolerance = tolerance
        oc.absolute_y_axis_tolerance = tolerance
        oc.absolute_z_axis_tolerance = tolerance
        oc.weight = 1.0
        return Constraints(position_constraints=[pc], orientation_constraints=[oc])

    def move_to(self, position, quaternion):
        """Plan and execute one pose. Returns the MoveIt error code."""
        request = MotionPlanRequest()
        request.group_name = self.p("planning_group")
        request.num_planning_attempts = 5
        request.allowed_planning_time = float(self.p("planning_time"))
        request.max_velocity_scaling_factor = float(self.p("velocity_scaling"))
        request.max_acceleration_scaling_factor = float(self.p("acceleration_scaling"))
        request.start_state.is_diff = True
        request.goal_constraints = [self.constraints(position, quaternion)]

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions(plan_only=bool(self.p("plan_only")))
        handle = self.wait(self.client.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            return None
        result = self.wait(handle.get_result_async(), 120.0)
        return None if result is None else result.result.error_code.val

    def run(self):
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("MoveIt not found: is demo.launch.py running?")
            return 1
        target = [float(v) for v in self.p("target")]
        points = orbit_waypoints(
            target,
            float(self.p("radius")),
            float(self.p("start_deg")),
            float(self.p("end_deg")),
            int(self.p("waypoints")),
        )
        reached = 0
        for i, position in enumerate(points):
            code = self.move_to(position, look_at(position, target))
            if code != MoveItErrorCodes.SUCCESS:
                self.get_logger().warn(
                    f"Waypoint {i + 1}/{len(points)} failed (MoveIt code {code}): skipping"
                )
                continue
            reached += 1
            time.sleep(float(self.p("settle_s")))
            print(f">>> [PLACEHOLDER] CAMERA CAPTURE TRIGGERED AT WAYPOINT {i + 1} <<<", flush=True)
        self.get_logger().info(f"Sweep done: {reached}/{len(points)} waypoints reached")
        return 0 if reached else 2


def main():
    rclpy.init()
    node = OrbitScanner()
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 130
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
