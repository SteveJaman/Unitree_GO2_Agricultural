#!/usr/bin/env python3
"""Plant inspection over a 180 degree arc (Far left, Left, Front, Right, Far right) with the
D1 arm through MoveIt 2.

Everything is in the arm's own base frame (FRAME, base_link: +x straight ahead of the
arm, +y its left, +z up). The arm is modelled facing forward (d1_go2.urdf.xacro, yaw 0);
on the dog it faces the tail, so the operator parks the dog tail-first to the plant and
for the arm the plant is simply ahead. No mount compensation, no roll parameter.

The five viewpoints sit on a circle around the canopy: Front between the plant and the
arm, Left and Right azimuth_deg to the arm's left and right of it, Far left and Far right
side_azimuth_deg (90: directly beside the plant), each a little below the canopy and looking
up at it (look_up_deg). The arm sweeps them in order from far left to far right, so each
move is to the neighbouring view.

Directly beside the lab rack's plant is out of the D1's reach (the canopy is already
~0.68 m from the shoulder, and the camera would have to look back past the arm), so Far
left / Far right are searched: the widest azimuth from side_azimuth_deg down, in 15 deg
steps and also 15 deg more look-up, that has an IK solution. The log gives the angle used. The gripper's X axis points at the canopy and
its Z axis is kept up in the arm frame.

The arm can make the same gripper pose with different forearm / wrist twists (Joint4 and
Joint6 both roll), and MoveIt picked a different one from run to run (Front once came out
Joint4 +91, Joint6 -88; the gripper looked flipped). So each pose is solved by IK from
several seeds, the solution with the least twist (|Joint4| + |Joint6|) is kept, and the
arm is sent there as a joint goal, planned and collision-checked by MoveIt.

Parameters (metres / degrees, arm frame): plant_forward, plant_left, canopy_height,
canopy_radius, standoff, look_up_deg, azimuth_deg, side_azimuth_deg, load_rack (bed_top_height, bed_depth,
bed_width, floor_height), dwell_s, plan_only, planning_time, velocity_scaling, tool_link.
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (CollisionObject, Constraints, JointConstraint, MotionPlanRequest,
                             PlanningOptions, PlanningScene, PositionIKRequest, RobotState)
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

FRAME = 'base_link'  # the D1's base: +x ahead of the arm
ARM_JOINTS = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
STOW_ANGLES = [0.0, -1.396, 0.785, 0.0, 1.396, 0.0]
SUCCESS = 1
# IK seeds: the current state and stow, plus forearm rolls the wrist may need for Left/Right.
SEED_J4_DEG = [0.0, 90.0, -90.0]


def look_at(eye, target):
    """Quaternion (x, y, z, w): X from eye to target, Z as close to +z (arm frame up) as possible."""
    x = np.asarray(target, float) - np.asarray(eye, float)
    x /= np.linalg.norm(x)
    up = np.array([0.0, 0.0, 1.0]) if abs(x[2]) < 0.99 else np.array([1.0, 0.0, 0.0])
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


def viewpoints(canopy, radius, look_up_deg, azimuth_deg, side_azimuth_deg=90.0):
    """[(name, eye, canopy)] from far left to far right, in the arm frame. Front lies on the
    line from the canopy back toward the arm (the origin); Left / Right are turned azimuth_deg
    and FarLeft / FarRight side_azimuth_deg about the canopy toward the arm's +y / -y."""
    el = math.radians(look_up_deg)
    toward_arm = math.atan2(-canopy[1], -canopy[0])
    out = []
    a, b = math.radians(azimuth_deg), math.radians(side_azimuth_deg)
    for name, turn in (('FarLeft', -b), ('Left', -a), ('Front', 0.0), ('Right', a), ('FarRight', b)):
        # Looking back at the arm from the plant, +y (the arm's left) is to the right,
        # i.e. a clockwise (negative) turn about the canopy.
        az = toward_arm + turn
        eye = (canopy[0] + radius * math.cos(el) * math.cos(az),
               canopy[1] + radius * math.cos(el) * math.sin(az),
               canopy[2] - radius * math.sin(el))
        out.append((name, eye, canopy))
    return out


def twist(q):
    """How much the forearm and wrist are rolled (rad): the quantity kept small."""
    return abs(q[3]) + abs(q[5])


class InspectPlant(Node):
    def __init__(self):
        super().__init__('inspect_plant')
        d = self.declare_parameter
        # The plant, ahead of the arm (defaults: the lab rack, dog parked tail-first).
        d('plant_forward', 0.465)
        d('plant_left', 0.0)
        d('canopy_height', 0.607)    # canopy centre above the arm base
        d('canopy_radius', 0.055)
        d('load_rack', False)
        d('bed_top_height', 0.517)   # rack top above the arm base
        d('bed_depth', 0.254)
        d('bed_width', 1.02)
        d('floor_height', -0.347)    # the floor, relative to the arm base
        # The views.
        d('standoff', 0.150)
        d('look_up_deg', 15.0)
        d('azimuth_deg', 30.0)
        d('side_azimuth_deg', 90.0)  # Far left / Far right: directly beside the plant
        d('dwell_s', 3.0)
        d('plan_only', False)
        d('planning_time', 5.0)
        d('velocity_scaling', 0.3)
        d('tool_link', 'tool0')
        self.p = lambda n: self.get_parameter(n).value

        self.move = ActionClient(self, MoveGroup, 'move_action')
        self.scene = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        self.ik = self.create_client(GetPositionIK, 'compute_ik')
        self.markers = self.create_publisher(MarkerArray, '/inspect_plant/markers', 1)
        self.joints = None
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)

    def on_joints(self, msg):
        if all(j in msg.name for j in ARM_JOINTS):
            self.joints = [msg.position[msg.name.index(j)] for j in ARM_JOINTS]

    def call(self, future, timeout=60.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def wait_for_moveit(self, timeout=30.0):
        ok = (self.move.wait_for_server(timeout_sec=timeout)
              and self.scene.wait_for_service(timeout_sec=timeout)
              and self.ik.wait_for_service(timeout_sec=timeout))
        if not ok:
            self.get_logger().error('MoveIt not found.')
        return ok

    # ------------------------------------------------------------------ scene
    def add_plant(self, canopy):
        def obj(name, kind, dims, pos):
            o = CollisionObject(id=name, operation=CollisionObject.ADD)
            o.header.frame_id = FRAME
            o.primitives.append(SolidPrimitive(type=kind, dimensions=list(dims)))
            o.primitive_poses.append(Pose(position=Point(x=pos[0], y=pos[1], z=pos[2]),
                                          orientation=Quaternion(w=1.0)))
            return o

        x, y, z = canopy
        r = float(self.p('canopy_radius'))
        objects = [obj('plant_canopy', SolidPrimitive.SPHERE, (r,), canopy)]
        if self.p('load_rack'):
            top, floor = float(self.p('bed_top_height')), float(self.p('floor_height'))
            objects.append(obj('bed', SolidPrimitive.BOX,
                               (self.p('bed_depth'), self.p('bed_width'), top - floor),
                               (x, y, (top + floor) / 2)))
            stem = max(0.01, z - r - top)
            objects.append(obj('plant_stem', SolidPrimitive.CYLINDER, (stem, 0.015), (x, y, top + stem / 2)))
        req = ApplyPlanningScene.Request(scene=PlanningScene(is_diff=True))
        req.scene.world.collision_objects = objects
        res = self.call(self.scene.call_async(req))
        return res is not None and res.success

    # -------------------------------------------------------------------- IK
    def solve(self, eye, quat):
        """The least-twisted IK solution for tool_link at eye/quat, or None."""
        pose = PoseStamped()
        pose.header.frame_id = FRAME
        pose.pose.position = Point(x=eye[0], y=eye[1], z=eye[2])
        pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        seeds = [list(STOW_ANGLES)] + ([list(self.joints)] if self.joints else [])
        for j4 in SEED_J4_DEG:
            seeds.append([0.0, -0.3, -0.1, math.radians(j4), 0.2, 0.0])
        best = None
        for seed in seeds:
            req = GetPositionIK.Request()
            r = PositionIKRequest(group_name='arm', ik_link_name=self.p('tool_link'),
                                  avoid_collisions=True, pose_stamped=pose)
            r.robot_state = RobotState(joint_state=JointState(name=ARM_JOINTS, position=seed))
            r.timeout.sec = 1
            req.ik_request = r
            res = self.call(self.ik.call_async(req), timeout=5.0)
            if res is None or res.error_code.val != SUCCESS:
                continue
            js = res.solution.joint_state
            q = [js.position[js.name.index(j)] for j in ARM_JOINTS]
            if best is None or twist(q) < twist(best):
                best = q
        return best

    # ------------------------------------------------------------------ moves
    def execute(self, angles):
        c = Constraints(name='joints')
        c.joint_constraints = [JointConstraint(joint_name=j, position=a, tolerance_above=0.01,
                                               tolerance_below=0.01, weight=1.0)
                               for j, a in zip(ARM_JOINTS, angles)]
        req = MotionPlanRequest(group_name='arm', num_planning_attempts=5,
                                allowed_planning_time=float(self.p('planning_time')),
                                max_velocity_scaling_factor=float(self.p('velocity_scaling')),
                                max_acceleration_scaling_factor=float(self.p('velocity_scaling')),
                                goal_constraints=[c])
        req.start_state.is_diff = True
        goal = MoveGroup.Goal(request=req, planning_options=PlanningOptions(plan_only=bool(self.p('plan_only'))))
        handle = self.call(self.move.send_goal_async(goal))
        if handle is None or not handle.accepted:
            return False, 'goal rejected'
        result = self.call(handle.get_result_async(), timeout=120.0)
        if result is None:
            return False, 'no result'
        code = result.result.error_code.val
        return code == SUCCESS, f'error code {code}'

    def widest(self, canopy, radius, name, sign):
        """(eye, joints, azimuth, look_up) of the widest reachable far view, or None."""
        a, b, up = float(self.p('azimuth_deg')), float(self.p('side_azimuth_deg')), float(self.p('look_up_deg'))
        az = b
        while az > a:
            for el in (up, up + 15.0):
                _, eye, target = viewpoints(canopy, radius, el, 0.0, az)[0 if sign < 0 else 4]
                q = self.solve(eye, look_at(eye, target))
                if q is not None:
                    return eye, q, az, el
            az -= 15.0
        return None

    def publish_markers(self, canopy, views, status):
        arr = MarkerArray()
        colors = {None: (0.6, 0.6, 0.6), True: (0.1, 0.9, 0.2), False: (0.9, 0.2, 0.1)}
        for i, (name, eye, _target) in enumerate(views):
            m = Marker(ns='views', id=i, type=Marker.ARROW, action=Marker.ADD)
            m.header.frame_id = FRAME
            m.points = [Point(x=eye[0], y=eye[1], z=eye[2]),
                        Point(x=eye[0] + 0.6 * (canopy[0] - eye[0]), y=eye[1] + 0.6 * (canopy[1] - eye[1]),
                              z=eye[2] + 0.6 * (canopy[2] - eye[2]))]
            m.scale.x, m.scale.y, m.scale.z = 0.008, 0.016, 0.02
            r, g, b = colors[status.get(i)]
            m.color = ColorRGBA(r=r, g=g, b=b, a=0.9)
            arr.markers.append(m)
        self.markers.publish(arr)

    def run(self):
        if not self.wait_for_moveit():
            return 1
        canopy = (float(self.p('plant_forward')), float(self.p('plant_left')), float(self.p('canopy_height')))
        if not self.add_plant(canopy):
            self.get_logger().error('Could not add the plant to the planning scene.')
            return 1
        radius = float(self.p('canopy_radius')) + float(self.p('standoff'))
        views = viewpoints(canopy, radius, float(self.p('look_up_deg')), float(self.p('azimuth_deg')),
                           float(self.p('side_azimuth_deg')))

        solutions = {}
        for i, (name, eye, target) in enumerate(views):
            if name.startswith('Far'):
                found = self.widest(canopy, radius, name, -1 if name == 'FarLeft' else 1)
                q = None
                if found:
                    eye, q, az, el = found
                    views[i] = (name, eye, target)
                    self.get_logger().info(f'{name}: widest reachable view {az:.0f} deg around, '
                                           f'looking up {el:.0f} deg')
            else:
                q = self.solve(eye, look_at(eye, target))
            solutions[name] = q
            if q is None:
                self.get_logger().warn(f'{name}: no IK solution at ({eye[0]:.3f}, {eye[1]:.3f}, {eye[2]:.3f})')
            else:
                self.get_logger().info(f'{name}: eye ({eye[0]:.3f}, {eye[1]:.3f}, {eye[2]:.3f}) m, joints '
                                       + ' '.join(f'{math.degrees(a):.1f}' for a in q) + ' deg')
        if solutions['Front'] is None:
            self.get_logger().error('Front is unreachable: move the dog or the plant parameters.')
            return 1

        status = {}
        for i, (name, eye, _target) in enumerate(views):
            if solutions[name] is None:
                status[i] = False
                continue
            self.get_logger().info(f'Moving to {name}...')
            ok, why = self.execute(solutions[name])
            status[i] = ok
            self.publish_markers(canopy, views, status)
            if not ok:
                self.get_logger().warn(f'{name}: {why}')
            else:
                time.sleep(float(self.p('dwell_s')))

        self.get_logger().info('Returning to Front...')
        self.execute(solutions['Front'])
        self.get_logger().info('Returning home (stow)...')
        self.execute(STOW_ANGLES)
        return 0


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
