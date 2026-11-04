#!/usr/bin/env python3
"""3-point plant inspection with the D1 arm through MoveIt 2.

Discrete Front / Left / Right poses instead of a continuous Cartesian arc:
the D1-550 moves to each viewpoint, pauses for capture, and returns home.
Works on mock hardware (arm_moveit.sh), Gazebo (arm_moveit.sh --gazebo) and
the real arm (arm_moveit.sh --real).

  ros2 launch d1_moveit_config demo.launch.py          # terminal 1
  ros2 run d1_moveit_config inspect_plant.py           # terminal 2
  ros2 run d1_moveit_config inspect_plant.py --ros-args -p plant_x:=0.2 -p plan_only:=true

The 3-point scan:
  1. Move to pre-grasp (Front pose).
  2. Front: hold 3 s (camera capture).
  3. Left:  hold 3 s.
  4. Right: hold 3 s.
  5. Return to Front.
  6. Retract to pre-grasp.
  7. Return Home (stow).

Parameters (world frame = ground under the Go2, x forward, y left). The
defaults are the hydroponic trial's rack and lettuce (docs/simulation/
hydroponic_trial.md) behind the dog, where the arm faces (mount yaw 180 deg):
  plant_x, plant_y      plant position (-0.465, 0: the trial's 0.333 m standoff)
  bed_height            rack height (0.864, the NFT stand)
  bed_depth, bed_width  rack size along x and y (0.254, 1.02), centred on the plant
  canopy_above_bed      canopy centre above the rack (0.09)
  canopy_radius         canopy sphere radius (0.055)
  standoff              camera distance from the canopy surface (0.095: 0.15 m
                        from the plant centre, the trial's orbit radius)
  look_up_deg           camera look-up angle toward the canopy (15 deg)
  azimuth_deg           Left / Right viewpoints, either side of Front (30 deg).
                        Planned against the solid rack box, 30 / 15 and 35 / 15
                        reach 3/3; 45 / 25 only 1/3 (the camera ends up in the rack)
  dwell_s               pause at each viewpoint for capture (3.0 s)
  plan_only             plan each move but do not execute it
  image_topic           optional sensor_msgs/Image to save at each viewpoint
  tool_link             link aimed at the plant: tool0 (gripper), or camera_link
                        with end_effector:=camera, as the trial
  velocity_scaling      MoveIt velocity/acceleration scaling (0.3; 0.2 with --real)

IMPORTANT: the dog MUST park with its tail 0.333 m from the rack's front edge
(the arm is mounted facing backward) for the arm to reach the Left and Right
angles (the D1-550's max reach is 0.605 m).
"""
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, CollisionObject, Constraints, JointConstraint,
                             MotionPlanRequest, OrientationConstraint, PlanningOptions,
                             PlanningScene, PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

ARM_JOINTS = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
STOW_ANGLES = [0.0, -1.396, 0.785, 0.0, 1.396, 0.0]  # safe walking pose
SUCCESS = 1  # moveit_msgs/MoveItErrorCodes.SUCCESS


def look_at(eye, target):
    """Quaternion (x, y, z, w) whose x axis points from eye to target, with
    z as close to world up as possible (an upright camera image)."""
    x = np.asarray(target, float) - np.asarray(eye, float)
    x /= np.linalg.norm(x)
    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(x, up)) > 0.99:
        up = np.array([1.0, 0.0, 0.0])
    z = up - np.dot(up, x) * x
    z /= np.linalg.norm(z)
    y = np.cross(z, x)
    m = np.column_stack([x, y, z])
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2.0
    if w > 1e-6:
        return ((m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w),
                (m[1, 0] - m[0, 1]) / (4 * w), w)
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(0.0, 1.0 + m[i, i] - m[j, j] - m[k, k])) * 2.0
    q = [0.0, 0.0, 0.0]
    q[i] = s / 4.0
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    return (q[0], q[1], q[2], (m[k, j] - m[j, k]) / s)


def compute_3_point_poses(canopy, toward_dog_yaw, radius, look_up_deg, azimuth_deg=30.0):
    """Compute Front, Left and Right camera positions around the canopy.

    Returns [(name, eye_xyz, target_xyz), ...] for the three discrete poses.
    Left and Right are at ±azimuth_deg from the front (toward the dog).
    """
    el = math.radians(look_up_deg)
    poses = []
    for name, az_deg in [('Front', 0.0), ('Left', -azimuth_deg), ('Right', azimuth_deg)]:
        az = toward_dog_yaw + math.radians(az_deg)
        eye = (canopy[0] + radius * math.cos(el) * math.cos(az),
               canopy[1] + radius * math.cos(el) * math.sin(az),
               canopy[2] - radius * math.sin(el))
        poses.append((name, eye, canopy))
    return poses


class InspectPlant(Node):

    def __init__(self):
        super().__init__('inspect_plant')
        d = self.declare_parameter
        d('plant_x', -0.465)
        d('plant_y', 0.0)
        d('bed_height', 0.864)
        d('bed_depth', 0.254)
        d('bed_width', 1.02)
        d('canopy_above_bed', 0.09)
        d('canopy_radius', 0.055)
        d('standoff', 0.095)
        d('look_up_deg', 15.0)
        d('azimuth_deg', 30.0)
        d('dwell_s', 3.0)
        d('plan_only', False)
        d('planning_time', 5.0)
        d('velocity_scaling', 0.3)
        d('image_topic', '')
        d('tool_link', 'tool0')
        d('image_dir', os.path.expanduser('~/plant_inspection'))
        self.p = lambda n: self.get_parameter(n).value

        self.move = ActionClient(self, MoveGroup, 'move_action')
        self.scene = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        self.markers = self.create_publisher(MarkerArray, '/inspect_plant/markers', 1)
        self.latest_image = None
        if self.p('image_topic'):
            self.create_subscription(Image, self.p('image_topic'),
                                     lambda m: setattr(self, 'latest_image', m), 1)

    # ------------------------------------------------------------------
    def wait_for_moveit(self, timeout=30.0):
        ok = (self.move.wait_for_server(timeout_sec=timeout)
              and self.scene.wait_for_service(timeout_sec=timeout))
        if not ok:
            self.get_logger().error(
                'MoveIt not found (move_action / apply_planning_scene). '
                'Is "ros2 launch d1_moveit_config demo.launch.py" running?')
        return ok

    def add_plant(self, bed_h, canopy_r):
        x, y = self.p('plant_x'), self.p('plant_y')
        above = float(self.p('canopy_above_bed'))

        def obj(name, kind, dims, pos):
            o = CollisionObject()
            o.header.frame_id = 'world'
            o.id = name
            o.operation = CollisionObject.ADD
            prim = SolidPrimitive(type=kind, dimensions=list(dims))
            pose = Pose(position=Point(x=pos[0], y=pos[1], z=pos[2]),
                        orientation=Quaternion(w=1.0))
            o.primitives.append(prim)
            o.primitive_poses.append(pose)
            return o

        objects = [
            obj('bed', SolidPrimitive.BOX, (self.p('bed_depth'), self.p('bed_width'), bed_h),
                (x, y, bed_h / 2)),
            obj('plant_stem', SolidPrimitive.CYLINDER, (above, 0.03), (x, y, bed_h + above / 2)),
            obj('plant_canopy', SolidPrimitive.SPHERE, (canopy_r,), (x, y, bed_h + above)),
        ]
        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene(is_diff=True)
        req.scene.world.collision_objects = objects
        res = self.call(self.scene.call_async(req))
        if res is None or not res.success:
            self.get_logger().error('Could not add the plant to the planning scene')
            return False
        return True

    # ------------------------------------------------------------------
    def call(self, future, timeout=60.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def request(self, constraints):
        r = MotionPlanRequest()
        r.group_name = 'arm'
        r.num_planning_attempts = 5
        r.allowed_planning_time = float(self.p('planning_time'))
        r.max_velocity_scaling_factor = float(self.p('velocity_scaling'))
        r.max_acceleration_scaling_factor = float(self.p('velocity_scaling'))
        r.start_state.is_diff = True
        r.goal_constraints = [constraints]
        return r

    def execute(self, constraints):
        goal = MoveGroup.Goal()
        goal.request = self.request(constraints)
        goal.planning_options = PlanningOptions(plan_only=bool(self.p('plan_only')))
        handle = self.call(self.move.send_goal_async(goal))
        if handle is None or not handle.accepted:
            return False, 'goal rejected'
        result = self.call(handle.get_result_async(), timeout=120.0)
        if result is None:
            return False, 'no result'
        code = result.result.error_code.val
        return code == SUCCESS, f'error code {code}'

    def pose_goal(self, pos, quat):
        c = Constraints(name='view')
        pc = PositionConstraint()
        pc.header.frame_id = 'world'
        pc.link_name = self.p('tool_link')
        pc.constraint_region = BoundingVolume(
            primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])],
            primitive_poses=[Pose(position=Point(x=pos[0], y=pos[1], z=pos[2]),
                                  orientation=Quaternion(w=1.0))])
        pc.weight = 1.0
        oc = OrientationConstraint()
        oc.header.frame_id = 'world'
        oc.link_name = self.p('tool_link')
        oc.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        oc.absolute_x_axis_tolerance = math.pi
        oc.absolute_y_axis_tolerance = 0.15
        oc.absolute_z_axis_tolerance = 0.15
        oc.weight = 1.0
        c.position_constraints = [pc]
        c.orientation_constraints = [oc]
        return c

    def joint_goal(self, angles):
        c = Constraints(name='joints')
        c.joint_constraints = [JointConstraint(joint_name=j, position=a,
                                               tolerance_above=0.01,
                                               tolerance_below=0.01, weight=1.0)
                               for j, a in zip(ARM_JOINTS, angles)]
        return c

    def home_goal(self):
        return self.joint_goal([0.0] * 6)

    def stow_goal(self):
        return self.joint_goal(STOW_ANGLES)

    # ------------------------------------------------------------------
    def publish_markers(self, canopy, poses, status):
        arr = MarkerArray()
        label = Marker(ns='plant', id=0, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
        label.header.frame_id = 'world'
        label.pose.position = Point(x=canopy[0], y=canopy[1], z=canopy[2] + 0.28)
        label.pose.orientation.w = 1.0
        label.scale.z = 0.05
        label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        label.text = 'plant'
        arr.markers.append(label)
        colors = {None: (0.6, 0.6, 0.6), True: (0.1, 0.9, 0.2), False: (0.9, 0.2, 0.1)}
        for i, (name, eye, _target) in enumerate(poses):
            m = Marker(ns='views', id=i, type=Marker.ARROW, action=Marker.ADD)
            m.header.frame_id = 'world'
            m.points = [Point(x=eye[0], y=eye[1], z=eye[2]),
                        Point(x=eye[0] + 0.6 * (canopy[0] - eye[0]),
                              y=eye[1] + 0.6 * (canopy[1] - eye[1]),
                              z=eye[2] + 0.6 * (canopy[2] - eye[2]))]
            m.scale.x, m.scale.y, m.scale.z = 0.008, 0.016, 0.02
            r, g, b = colors[status.get(i)]
            m.color = ColorRGBA(r=r, g=g, b=b, a=0.9)
            lbl = Marker(ns='labels', id=i, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            lbl.header.frame_id = 'world'
            lbl.pose.position = Point(x=eye[0], y=eye[1], z=eye[2] + 0.06)
            lbl.pose.orientation.w = 1.0
            lbl.scale.z = 0.04
            lbl.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
            lbl.text = name
            arr.markers.extend([m, lbl])
        self.markers.publish(arr)

    def save_image(self, name):
        if self.latest_image is None:
            return None
        msg = self.latest_image
        os.makedirs(self.p('image_dir'), exist_ok=True)
        path = os.path.join(self.p('image_dir'), f'view_{name.lower()}.ppm')
        if msg.encoding not in ('rgb8', 'bgr8'):
            return None
        a = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
        a = a[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == 'bgr8':
            a = a[..., ::-1]
        with open(path, 'wb') as f:
            f.write(f'P6 {msg.width} {msg.height} 255\n'.encode())
            f.write(np.ascontiguousarray(a).tobytes())
        return path

    # ------------------------------------------------------------------
    def run(self):
        if not self.wait_for_moveit():
            return 1
        bed_h = float(self.p('bed_height'))
        canopy_r = float(self.p('canopy_radius'))
        if not self.add_plant(bed_h, canopy_r):
            return 1
        x, y = self.p('plant_x'), self.p('plant_y')
        canopy = (x, y, bed_h + float(self.p('canopy_above_bed')))
        toward_dog = math.atan2(0.0 - y, 0.0 - x)
        radius = canopy_r + float(self.p('standoff'))
        look_up = float(self.p('look_up_deg'))
        dwell = float(self.p('dwell_s'))

        poses = compute_3_point_poses(canopy, toward_dog, radius, look_up, float(self.p('azimuth_deg')))
        status = {}
        self.publish_markers(canopy, poses, status)
        self.get_logger().info(
            f'3-point inspection at ({x:.2f}, {y:.2f}), canopy z={canopy[2]:.2f}: '
            f'Front / Left / Right, dwell {dwell:.1f}s, plan_only={self.p("plan_only")}')

        # Step 1: Move to pre-grasp (Front pose)
        front_name, front_eye, front_target = poses[0]
        self.get_logger().info('[1/7] Moving to pre-grasp (Front)...')
        ok, why = self.execute(self.pose_goal(front_eye, look_at(front_eye, front_target)))
        if not ok:
            self.get_logger().error(f'Pre-grasp (Front) failed: {why}')
            self.execute(self.stow_goal())
            return 1

        captured = 0
        # Steps 2-4: Front (capture), Left (capture), Right (capture)
        for i, (name, eye, target) in enumerate(poses):
            step = i + 2
            self.get_logger().info(f'[{step}/7] Moving to {name}...')
            t0 = time.time()
            ok, why = self.execute(self.pose_goal(eye, look_at(eye, target)))
            status[i] = ok
            self.publish_markers(canopy, poses, status)
            if not ok:
                self.get_logger().warn(f'{name}: unreachable ({why})')
                continue
            captured += 1
            self.get_logger().info(
                f'{name}: reached in {time.time() - t0:.1f}s, capturing for {dwell:.1f}s...')
            time.sleep(dwell)
            saved = self.save_image(name)
            if saved:
                self.get_logger().info(f'{name}: saved {saved}')

        # Step 5: Return to Front
        self.get_logger().info('[5/7] Returning to Front...')
        self.execute(self.pose_goal(front_eye, look_at(front_eye, front_target)))

        # Step 6: Retract to pre-grasp (already at Front = pre-grasp)
        self.get_logger().info('[6/7] Retract (at pre-grasp).')

        # Step 7: Home (stow)
        self.get_logger().info('[7/7] Returning home (stow)...')
        ok, why = self.execute(self.stow_goal())

        self.get_logger().info(
            f'Inspection done: {captured}/3 viewpoints reached; '
            f'back home: {"yes" if ok else "no (" + why + ")"}')
        return 0 if captured else 2


def main():
    rclpy.init()
    node = InspectPlant()
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 130
    node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
