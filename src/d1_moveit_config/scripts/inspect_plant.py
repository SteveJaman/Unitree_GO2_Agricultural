#!/usr/bin/env python3
"""Plant inspection with the D1 arm through MoveIt 2.

Puts a raised bed and a plant (stem + canopy, same sizes as
gazebo/worlds/agri_field.sdf) next to the Go2 in MoveIt's planning scene,
works out camera viewpoints on an arc around the canopy, and has MoveIt plan
and execute a collision-free move to each one. At every viewpoint the tool
frame (between the fingertips, x out of the gripper) points at the canopy
centre and holds still for dwell_s, the moment a wrist camera would capture.
Then the arm returns to "home".

  ros2 launch d1_moveit_config demo.launch.py          # terminal 1
  ros2 run d1_moveit_config inspect_plant.py           # terminal 2
  ros2 run d1_moveit_config inspect_plant.py --ros-args -p plant_x:=0.2 -p plan_only:=true

Parameters (world frame = ground under the Go2, x forward, y left):
  plant_x, plant_y      plant position (default 0.10, -0.50: right of the dog)
  bed_height            raised-bed height (0.30)
  canopy_radius         canopy sphere radius (0.17); centre 0.38 m above the bed
  standoff              camera distance from the canopy surface (0.10)
  azimuths_deg          viewpoint directions around the plant, 0 = facing the dog
  elevations_deg        viewpoint heights, 0 = level with the canopy centre
  dwell_s               pause at each viewpoint
  plan_only             plan each move but do not execute it
  image_topic           optional sensor_msgs/Image to save at each viewpoint
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
SUCCESS = 1  # moveit_msgs/MoveItErrorCodes.SUCCESS
STEM_HEIGHT = 0.30
CANOPY_ABOVE_BED = 0.38


def look_at(eye, target):
    """Quaternion (x, y, z, w) whose x axis points from eye to target, with
    z as close to world up as possible (an upright camera image)."""
    x = np.asarray(target, float) - np.asarray(eye, float)
    x /= np.linalg.norm(x)
    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(x, up)) > 0.99:          # looking straight up/down
        up = np.array([1.0, 0.0, 0.0])
    z = up - np.dot(up, x) * x
    z /= np.linalg.norm(z)
    y = np.cross(z, x)
    m = np.column_stack([x, y, z])
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2.0
    if w > 1e-6:
        return ((m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w),
                (m[1, 0] - m[0, 1]) / (4 * w), w)
    # 180-degree rotation: pick the largest diagonal term
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(0.0, 1.0 + m[i, i] - m[j, j] - m[k, k])) * 2.0
    q = [0.0, 0.0, 0.0]
    q[i] = s / 4.0
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    return (q[0], q[1], q[2], (m[k, j] - m[j, k]) / s)


def viewpoints(canopy, toward_dog_yaw, radius, azimuths_deg, elevations_deg):
    """Camera positions on a sphere around the canopy, on the dog's side.
    Ordered as a sweep: left to right at each height, alternating direction."""
    out = []
    for n, el in enumerate(elevations_deg):
        azs = azimuths_deg if n % 2 == 0 else list(reversed(azimuths_deg))
        for az in azs:
            yaw = toward_dog_yaw + math.radians(az)
            e = math.radians(el)
            out.append((canopy[0] + radius * math.cos(e) * math.cos(yaw),
                        canopy[1] + radius * math.cos(e) * math.sin(yaw),
                        canopy[2] + radius * math.sin(e), az, el))
    return out


class InspectPlant(Node):

    def __init__(self):
        super().__init__('inspect_plant')
        d = self.declare_parameter
        d('plant_x', 0.10)
        d('plant_y', -0.50)
        d('bed_height', 0.30)
        d('canopy_radius', 0.17)
        d('standoff', 0.10)
        d('azimuths_deg', [-50.0, -25.0, 0.0, 25.0, 50.0])
        d('elevations_deg', [0.0, 30.0])
        d('dwell_s', 1.5)
        d('plan_only', False)
        d('planning_time', 5.0)
        d('velocity_scaling', 0.3)
        d('image_topic', '')
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
            # 1.2 m of bed along x, 0.6 m wide, centred on the plant row
            obj('bed', SolidPrimitive.BOX, (1.2, 0.6, bed_h), (x, y, bed_h / 2)),
            obj('plant_stem', SolidPrimitive.CYLINDER, (STEM_HEIGHT, 0.03),
                (x, y, bed_h + STEM_HEIGHT / 2)),
            obj('plant_canopy', SolidPrimitive.SPHERE, (canopy_r,),
                (x, y, bed_h + CANOPY_ABOVE_BED)),
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
        pc.link_name = 'tool0'
        pc.constraint_region = BoundingVolume(
            primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])],
            primitive_poses=[Pose(position=Point(x=pos[0], y=pos[1], z=pos[2]),
                                  orientation=Quaternion(w=1.0))])
        pc.weight = 1.0
        oc = OrientationConstraint()
        oc.header.frame_id = 'world'
        oc.link_name = 'tool0'
        oc.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        # The camera may roll about its viewing axis (x); the viewing
        # direction itself must stay within ~9 degrees of the plant.
        oc.absolute_x_axis_tolerance = math.pi
        oc.absolute_y_axis_tolerance = 0.15
        oc.absolute_z_axis_tolerance = 0.15
        oc.weight = 1.0
        c.position_constraints = [pc]
        c.orientation_constraints = [oc]
        return c

    def home_goal(self):
        c = Constraints(name='home')
        c.joint_constraints = [JointConstraint(joint_name=j, position=0.0, tolerance_above=0.01,
                                               tolerance_below=0.01, weight=1.0)
                               for j in ARM_JOINTS]
        return c

    # ------------------------------------------------------------------
    def publish_markers(self, canopy, views, status):
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
        for i, v in enumerate(views):
            m = Marker(ns='views', id=i, type=Marker.ARROW, action=Marker.ADD)
            m.header.frame_id = 'world'
            m.points = [Point(x=v[0], y=v[1], z=v[2]),
                        Point(x=v[0] + 0.6 * (canopy[0] - v[0]),
                              y=v[1] + 0.6 * (canopy[1] - v[1]),
                              z=v[2] + 0.6 * (canopy[2] - v[2]))]
            m.scale.x, m.scale.y, m.scale.z = 0.008, 0.016, 0.02
            r, g, b = colors[status.get(i)]
            m.color = ColorRGBA(r=r, g=g, b=b, a=0.9)
            arr.markers.append(m)
        self.markers.publish(arr)

    def save_image(self, index):
        if self.latest_image is None:
            return None
        msg = self.latest_image
        os.makedirs(self.p('image_dir'), exist_ok=True)
        path = os.path.join(self.p('image_dir'), f'view_{index:02d}.ppm')
        if msg.encoding not in ('rgb8', 'bgr8'):
            return None
        a = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
        a = a[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == 'bgr8':
            a = a[..., ::-1]
        with open(path, 'wb') as f:   # PPM: no extra dependency
            f.write(f'P6 {msg.width} {msg.height} 255\n'.encode())
            f.write(np.ascontiguousarray(a).tobytes())
        return path

    # ------------------------------------------------------------------
    def run(self):
        if not self.wait_for_moveit():
            return 1
        bed_h, canopy_r = float(self.p('bed_height')), float(self.p('canopy_radius'))
        if not self.add_plant(bed_h, canopy_r):
            return 1
        x, y = self.p('plant_x'), self.p('plant_y')
        canopy = (x, y, bed_h + CANOPY_ABOVE_BED)
        toward_dog = math.atan2(0.0 - y, 0.0 - x)
        views = viewpoints(canopy, toward_dog, canopy_r + float(self.p('standoff')),
                           list(self.p('azimuths_deg')), list(self.p('elevations_deg')))
        status = {}
        self.publish_markers(canopy, views, status)
        self.get_logger().info(
            f'Inspecting plant at ({x:.2f}, {y:.2f}), canopy centre z={canopy[2]:.2f}: '
            f'{len(views)} viewpoints, plan_only={self.p("plan_only")}')

        captured = 0
        for i, (vx, vy, vz, az, el) in enumerate(views):
            t0 = time.time()
            ok, why = self.execute(self.pose_goal((vx, vy, vz), look_at((vx, vy, vz), canopy)))
            status[i] = ok
            self.publish_markers(canopy, views, status)
            if not ok:
                self.get_logger().warn(
                    f'view {i + 1}/{len(views)} (az {az:+.0f}, el {el:+.0f}): '
                    f'unreachable ({why})')
                continue
            captured += 1
            time.sleep(float(self.p('dwell_s')))
            saved = self.save_image(i)
            self.get_logger().info(
                f'view {i + 1}/{len(views)} (az {az:+.0f}, el {el:+.0f}): reached in '
                f'{time.time() - t0:.1f}s' + (f', saved {saved}' if saved else ''))

        ok, why = self.execute(self.home_goal())
        self.get_logger().info(
            f'Inspection done: {captured}/{len(views)} viewpoints reached; '
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
