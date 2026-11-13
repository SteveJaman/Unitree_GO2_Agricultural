#!/usr/bin/env python3
"""orbit_scanner.py: 180 degree active perception sweep of the D1 arm through MoveIt 2.

Thirteen camera poses on an arc around the plant, -90 to +90 degrees from the
side facing the arm, raised elevation_deg above it and aimed at its centre. The
arm moves to each pose in turn, settles, and prints a placeholder where the
capture will go. A waypoint with no solution is logged and skipped.

All waypoints are solved before anything moves, as one chain of IK solutions,
each seeded from the last: separate pose goals land on unrelated arm
configurations, and the wrist flips between neighbouring views. The chain is
then executed as joint goals.

The defaults are a low plant beside the dog, the geometry where the whole 180
degrees is reachable. A flat circle level with the plant tops out near 120
degrees, wherever the plant is (docs/hardware/d1_arm.md).

  ros2 launch d1_moveit_config demo.launch.py end_effector:=camera      # terminal 1
  ros2 run d1_active_perception orbit_scanner                           # terminal 2
  ros2 run d1_active_perception orbit_scanner --ros-args -p plan_only:=true

Every number below is a ROS parameter. ik_link is the group's tip; in
d1_moveit_config camera_link has x along the view, so the optical pose (z
forward, x right, y down) from look_at() is converted for it.
"""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    PlanningOptions,
    PlanningScene,
    PositionIKRequest,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

ARM_JOINTS = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
# camera_link -> camera_optical_frame in d1_go2.urdf.xacro
LINK_TO_OPTICAL = Rotation.from_euler("xyz", [-np.pi / 2, 0.0, -np.pi / 2])
ROLL_STEP_DEG = 30
STOW = [0.0, -1.396, 0.785, 0.0, 1.396, 0.0]  # "stow" in d1_go2.srdf: folded for walking


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


def orbit_waypoints(target, radius, start_deg, end_deg, count, elevation_deg=0.0):
    """Camera positions on an arc around target, angle 0 facing the origin, raised
    elevation_deg above the plant (0 = level, a horizontal circle)."""
    target = np.asarray(target, float)
    facing_arm = np.arctan2(-target[1], -target[0])
    el = np.radians(elevation_deg)
    points = []
    for deg in np.linspace(start_deg, end_deg, count):
        angle = facing_arm + np.radians(deg)
        direction = np.array(
            [np.cos(el) * np.cos(angle), np.cos(el) * np.sin(angle), np.sin(el)]
        )
        points.append(target + radius * direction)
    return points


class OrbitScanner(Node):
    def __init__(self):
        super().__init__("orbit_scanner")
        d = self.declare_parameter
        d("planning_group", "arm")
        d("ik_link", "camera_link")
        d("planning_frame", "world")
        d("target", [0.0, 0.40, 0.15])
        d("radius", 0.25)
        d("plant_radius", 0.08)  # canopy sphere around target, on a stem from the ground; 0 = none
        d("elevation_deg", 60.0)
        d("free_roll", True)  # any rotation about the view axis: reaches far more poses
        d("start_deg", -90.0)
        d("end_deg", 90.0)
        d("waypoints", 13)
        d("velocity_scaling", 0.15)
        d("acceleration_scaling", 0.15)
        d("planning_time", 2.0)
        d("joint_tolerance", 0.01)
        d("settle_s", 1.0)
        d("dwell_s", 0.0)  # hold after the capture message
        d("plan_only", False)
        d("return_stow", True)  # fold back after the sweep
        self.p = lambda name: self.get_parameter(name).value
        self.client = ActionClient(self, MoveGroup, "move_action")
        self.ik = self.create_client(GetPositionIK, "compute_ik")
        self.scene = self.create_client(ApplyPlanningScene, "apply_planning_scene")

    def wait(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def add_plant(self, target):
        """The plant in the planning scene, so IK and the planner keep the arm off it."""
        radius = float(self.p("plant_radius"))
        if radius <= 0.0:
            return True
        x, y, z = target

        def pose(height):
            return Pose(position=Point(x=x, y=y, z=height), orientation=Quaternion(w=1.0))

        plant = CollisionObject(id="plant", operation=CollisionObject.ADD)
        plant.header.frame_id = self.p("planning_frame")
        plant.primitives = [
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[radius]),
            SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[z, 0.02]),
        ]
        plant.primitive_poses = [pose(z), pose(z / 2)]
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [plant]
        res = self.wait(self.scene.call_async(ApplyPlanningScene.Request(scene=scene)), 10.0)
        return res is not None and res.success

    def solve_ik(self, position, optical_quat, roll_deg, seed):
        """Joint angles (radians) for this view, rolled about the optical axis, or None."""
        view = Rotation.from_quat(optical_quat) * Rotation.from_euler("z", roll_deg, degrees=True)
        link = (view * LINK_TO_OPTICAL.inv()).as_quat()
        req = PositionIKRequest()
        req.group_name = self.p("planning_group")
        req.ik_link_name = self.p("ik_link")
        req.avoid_collisions = True
        req.timeout.nanosec = 50_000_000
        req.pose_stamped.header.frame_id = self.p("planning_frame")
        pose = req.pose_stamped.pose
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position)
        q = pose.orientation
        q.x, q.y, q.z, q.w = (float(v) for v in link)
        if seed is not None:
            req.robot_state.joint_state = JointState(name=ARM_JOINTS, position=list(seed))
        res = self.wait(self.ik.call_async(GetPositionIK.Request(ik_request=req)), 2.0)
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        js = res.solution.joint_state
        return np.array([js.position[js.name.index(j)] for j in ARM_JOINTS])

    def chain_from(self, views, first_roll):
        """Greedy chain of IK solutions, each close to the last. None where unsolved."""
        rolls = range(0, 360, ROLL_STEP_DEG) if self.p("free_roll") else [0]
        chain, seed, roll, travel = [], None, first_roll, 0.0
        for position, quat in views:
            if seed is None:
                tries = [roll]
            elif self.p("free_roll"):
                tries = [roll + d for d in (0, 30, -30, 60, -60)]
            else:
                tries = list(rolls)
            best, best_cost, best_roll = None, np.inf, roll
            for r in tries:
                q = self.solve_ik(position, quat, r, seed)
                if q is None:
                    continue
                cost = 0.0 if seed is None else np.abs(q - seed).max()
                if cost < best_cost:
                    best, best_cost, best_roll = q, cost, r
            chain.append(best)
            if best is not None:
                travel += 0.0 if seed is None else best_cost
                seed, roll = best, best_roll
        return chain, travel

    def plan_sweep(self, views):
        """The chain with the most waypoints, then the least joint travel."""
        best, best_score = None, None
        rolls = range(0, 360, ROLL_STEP_DEG) if self.p("free_roll") else [0]
        for roll in rolls:
            chain, travel = self.chain_from(views, roll)
            score = (sum(q is not None for q in chain), -travel)
            if best is None or score > best_score:
                best, best_score = chain, score
        return best

    def move_to(self, joints):
        """Plan and execute one joint goal. Returns the MoveIt error code."""
        tolerance = float(self.p("joint_tolerance"))
        request = MotionPlanRequest()
        request.group_name = self.p("planning_group")
        request.num_planning_attempts = 5
        request.allowed_planning_time = float(self.p("planning_time"))
        request.max_velocity_scaling_factor = float(self.p("velocity_scaling"))
        request.max_acceleration_scaling_factor = float(self.p("acceleration_scaling"))
        request.start_state.is_diff = True
        request.goal_constraints = [
            Constraints(
                joint_constraints=[
                    JointConstraint(
                        joint_name=j,
                        position=float(a),
                        tolerance_above=tolerance,
                        tolerance_below=tolerance,
                        weight=1.0,
                    )
                    for j, a in zip(ARM_JOINTS, joints)
                ]
            )
        ]

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions(plan_only=bool(self.p("plan_only")))
        handle = self.wait(self.client.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            return None
        result = self.wait(handle.get_result_async(), 120.0)
        return None if result is None else result.result.error_code.val

    def run(self):
        ready = self.client.wait_for_server(timeout_sec=30.0)
        ready = ready and self.ik.wait_for_service(timeout_sec=30.0)
        if not ready or not self.scene.wait_for_service(timeout_sec=30.0):
            self.get_logger().error("MoveIt not found: is demo.launch.py running?")
            return 1
        target = [float(v) for v in self.p("target")]
        if not self.add_plant(target):
            self.get_logger().error("Could not add the plant to the planning scene")
            return 1
        points = orbit_waypoints(
            target,
            float(self.p("radius")),
            float(self.p("start_deg")),
            float(self.p("end_deg")),
            int(self.p("waypoints")),
            float(self.p("elevation_deg")),
        )
        chain = self.plan_sweep([(p, look_at(p, target)) for p in points])
        solved = sum(q is not None for q in chain)
        self.get_logger().info(f"Solved {solved}/{len(points)} waypoints before moving")

        reached = 0
        for i, joints in enumerate(chain):
            if joints is None:
                self.get_logger().warn(f"Waypoint {i + 1}/{len(points)}: no solution, skipping")
                continue
            code = self.move_to(joints)
            if code == MoveItErrorCodes.CONTROL_FAILED:
                # The plan was fine and the arm did not follow it: blocked, or the
                # link dropped. Do not push on: the arm holds where it stopped.
                self.get_logger().error(
                    f"Waypoint {i + 1}/{len(points)}: the arm did not follow the plan, "
                    "stopping the sweep. It holds where it is"
                )
                return 3
            if code != MoveItErrorCodes.SUCCESS:
                self.get_logger().warn(
                    f"Waypoint {i + 1}/{len(points)} failed (MoveIt code {code}): skipping"
                )
                continue
            reached += 1
            time.sleep(float(self.p("settle_s")))
            print(f">>> [PLACEHOLDER] CAMERA CAPTURE TRIGGERED AT WAYPOINT {i + 1} <<<", flush=True)
            time.sleep(float(self.p("dwell_s")))
        self.get_logger().info(f"Sweep done: {reached}/{len(points)} waypoints reached")
        if self.p("return_stow") and not self.p("plan_only"):
            code = self.move_to(STOW)
            if code != MoveItErrorCodes.SUCCESS:
                self.get_logger().error(f"Could not return to stow (MoveIt code {code})")
                return 3
            self.get_logger().info("Back at stow")
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
