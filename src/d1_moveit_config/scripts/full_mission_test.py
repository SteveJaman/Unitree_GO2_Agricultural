#!/usr/bin/env python3
"""
full_mission_test.py: end-to-end mobile-manipulation mission in the Gazebo
hydroponic station (gazebo/worlds/hydroponic_station.sdf, a CropKing NFT 8-4).

  Navigation  Nav2 goal to the waypoint 0.6 m in front of the NFT stand
              (/goal_pose), then a short precision approach on ground truth
              (/cmd_vel): Nav2 stops within 0.25 m, the arm needs ~2 cm.
  Setup       arm folded into the start pose "stow" (SRDF) for walking.
  Approach    MoveIt: arm from the start pose to the pre-grasp pose, camera 0.30 m from
              the target lettuce, aimed at its centre.
  Extend      Cartesian straight line to 0.15 m from the plant.
  Orbit       Cartesian arc around the plant at 0.15 m, camera aimed at the
              centre: 0 -> -45 deg -> +45 deg -> 0 (90 deg of coverage; the
              D1-550 cannot reach a 180 deg orbit, see the trial report).
              Wrist RGB-D frames are saved at -45, -22.5, 0, 22.5, 45 deg.
  Retract     Cartesian straight line back to the pre-grasp pose.
  Home        back to the start pose (stow).

Runs inside the go2-sim container of a SIM_MOVEIT=1 simulation (tests/
hydroponic_trial.py starts everything): the Go2 / Nav2 topics are on ROS
domain --nav-domain (64), the arm stack on --arm-domain (66); the dog's
ground-truth pose and the screenshots come from Gazebo (gz-transport).

Writes to --out: hydroponic_trial.csv (one row per phase: timestamp, phase,
execution time, end-effector distance, success, deviation from the planned
path, joint-velocity spikes ...), samples.npz (every joint state and pose),
summary.json (Mission Score), screens/*.png (observer cameras after each
phase, wrist camera at the orbit viewpoints) and the graphs
(trajectory_3d.png, phase_times.png, joint_velocities.png, deviation.png).
"""
import argparse
import csv
import json
import math
import os
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (BoundingVolume, CollisionObject, Constraints, JointConstraint,
                             MotionPlanRequest, OrientationConstraint, PlanningOptions,
                             PlanningScene, PositionConstraint, RobotState)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

ARM_JOINTS = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
# Start / end pose: "stow" in config/d1_go2.srdf, arm folded back over the dog for walking
STOW = {'Joint1': 0.0, 'Joint2': -1.396, 'Joint3': 0.785, 'Joint4': 0.0, 'Joint5': 1.396, 'Joint6': 0.0}
EE_LINK = 'camera_link'
VMAX = {'Joint1': 1.05, 'Joint2': 1.05, 'Joint3': 1.05, 'Joint4': 1.73, 'Joint5': 1.73, 'Joint6': 1.73}
AMAX = {'Joint1': 2.0, 'Joint2': 2.0, 'Joint3': 2.0, 'Joint4': 3.0, 'Joint5': 3.0, 'Joint6': 3.0}
SUCCESS = 1


# ------------------------------------------------------------------ geometry
def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_matrix(axis, q):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * k + (1 - math.cos(q)) * k @ k


class Kinematics:
    """Forward kinematics world -> EE_LINK from the robot description (the same
    URDF Gazebo and MoveIt use), for the executed and planned camera paths."""

    def __init__(self, urdf_path, tip=EE_LINK):
        root = ET.parse(urdf_path).getroot()
        by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        chain, link = [], tip
        while link in by_child:
            j = by_child[link]
            chain.append(j)
            link = j.find('parent').get('link')
        self.chain = []
        for j in reversed(chain):
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            ax = j.find('axis')
            self.chain.append((j.get('name'), j.get('type'), np.array(xyz), rpy_matrix(*rpy),
                               [float(v) for v in ax.get('xyz').split()] if ax is not None else [1, 0, 0]))

    def pose(self, q):
        """(position, rotation) of the tip for joint values {name: rad}."""
        T = np.eye(4)
        for name, kind, xyz, R0, ax in self.chain:
            A = np.eye(4)
            A[:3, :3] = R0 @ (axis_matrix(ax, q.get(name, 0.0)) if kind in ('revolute', 'continuous') else np.eye(3))
            A[:3, 3] = xyz
            T = T @ A
        return T[:3, 3].copy(), T[:3, :3].copy()

    def limits(self, urdf_path):
        root = ET.parse(urdf_path).getroot()
        out = {}
        for j in root.findall('joint'):
            lim = j.find('limit')
            if j.get('name') in ARM_JOINTS and lim is not None:
                out[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')))
        return out

    def ik(self, pos, R, seeds, lim, iters=200):
        """Damped least-squares IK for a full pose, from several seeds; returns the
        solution with the smallest joint angles (the branch the Cartesian orbit
        can follow, see the trial report), or None."""
        lo = np.array([lim[j][0] for j in ARM_JOINTS]) * 0.98
        hi = np.array([lim[j][1] for j in ARM_JOINTS]) * 0.98

        def err(q):
            p, Rq = self.pose(dict(zip(ARM_JOINTS, q)))
            dr = 0.5 * (np.cross(Rq[:, 0], R[:, 0]) + np.cross(Rq[:, 1], R[:, 1]) + np.cross(Rq[:, 2], R[:, 2]))
            return np.concatenate([pos - p, 0.3 * dr])
        best = None
        for seed in seeds:
            q = np.clip(np.array(seed, float), lo, hi)
            for _ in range(iters):
                e = err(q)
                if np.linalg.norm(e[:3]) < 5e-4 and np.linalg.norm(e[3:]) < 3e-3:
                    break
                Jm = np.empty((6, 6))
                for k in range(6):
                    dq = np.zeros(6)
                    dq[k] = 1e-5
                    Jm[:, k] = (err(q + dq) - e) / -1e-5
                q = np.clip(q + Jm.T @ np.linalg.solve(Jm @ Jm.T + 1e-4 * np.eye(6), e), lo, hi)
            e = err(q)
            if np.linalg.norm(e[:3]) < 1e-3 and np.linalg.norm(e[3:]) < 6e-3:
                if best is None or np.abs(q).sum() < np.abs(best).sum():
                    best = q
        return None if best is None else dict(zip(ARM_JOINTS, best.tolist()))


def upright(x):
    """Rotation whose x axis is x (unit) and z axis as close to world up as possible."""
    x = np.asarray(x, float) / np.linalg.norm(x)
    z = np.array([0.0, 0.0, 1.0]) - x[2] * x
    z /= np.linalg.norm(z)
    return np.column_stack([x, np.cross(z, x), z])


def look_at(eye, target):
    """Quaternion (x, y, z, w) whose x axis points from eye to target, z as close to up as possible."""
    x = np.asarray(target, float) - np.asarray(eye, float)
    x /= np.linalg.norm(x)
    up = np.array([0.0, 0.0, 1.0])
    z = up - np.dot(up, x) * x
    z /= np.linalg.norm(z)
    y = np.cross(z, x)
    m = np.column_stack([x, y, z])
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2.0
    return ((m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w), w)


def quat_from_matrix(m):
    """(x, y, z, w) of a rotation matrix."""
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(0.0, 1.0 + m[i, i] - m[j, j] - m[k, k])) * 2
    q = [0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    return (q[0], q[1], q[2], (m[k, j] - m[j, k]) / s)


def view_dir(az, el):
    """Unit vector from the camera to the plant: az around the plant (0 = from the dog), el > 0 looking up."""
    return np.array([math.cos(az) * math.cos(el), math.sin(az) * math.cos(el), math.sin(el)])


def cross_track(points, path):
    """Distance of each point to a polyline."""
    if len(points) == 0 or len(path) < 2:
        return np.zeros(len(points))
    a, b = path[:-1], path[1:]
    ab = b - a
    L2 = np.maximum((ab ** 2).sum(1), 1e-12)
    out = np.empty(len(points))
    for i, p in enumerate(points):
        t = np.clip(((p - a) * ab).sum(1) / L2, 0, 1)
        out[i] = np.min(np.linalg.norm(a + ab * t[:, None] - p, axis=1))
    return out


def path_length(p):
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()) if len(p) > 1 else 0.0


# ------------------------------------------------------------------ Gazebo side
class GzSide:
    """Ground-truth dog pose and camera frames from Gazebo transport."""

    def __init__(self, cameras):
        from gz.msgs10.image_pb2 import Image
        from gz.msgs10.odometry_pb2 import Odometry
        from gz.transport13 import Node
        self.node, self.Image = Node(), Image
        self.dog = None
        self.cameras = cameras
        self.node.subscribe(Odometry, '/model/go2/odometry', self._odom)

    def _odom(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.dog = (p.x, p.y, p.z, yaw, time.monotonic())

    def grab(self, topic, path, timeout=20.0):
        """Save the next frame of a camera topic (subscribing only for that frame: rendering costs sim speed)."""
        got = {}

        def cb(msg):
            if 'img' not in got:
                got['img'] = msg
        self.node.subscribe(self.Image, topic, cb)
        t0 = time.monotonic()
        while 'img' not in got and time.monotonic() - t0 < timeout:
            time.sleep(0.05)
        self.node.unsubscribe(topic)
        if 'img' not in got:
            return None
        m = got['img']
        a = np.frombuffer(m.data, np.uint8)
        if a.size == m.width * m.height * 3:
            img = a.reshape(m.height, m.width, 3)
        elif a.size == m.width * m.height * 4:          # 32-bit float depth
            d = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)
            d = np.nan_to_num(d, nan=0.0, posinf=0.0)
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            plt.imsave(path, d, cmap='viridis', vmin=0.0, vmax=max(0.5, float(np.percentile(d[d > 0], 95)) if (d > 0).any() else 1.0))
            return path
        else:
            return None
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        plt.imsave(path, img)
        return path


# ------------------------------------------------------------------ the mission
class Mission:
    def __init__(self, a):
        self.a = a
        self.world = json.load(open(a.world_json))
        self.kin = Kinematics(a.urdf)
        os.makedirs(os.path.join(a.out, 'screens'), exist_ok=True)
        self.gz = GzSide(self.world.get('observer_cameras', {}))

        self.nav_ctx, self.arm_ctx = rclpy.Context(), rclpy.Context()
        rclpy.init(context=self.nav_ctx, domain_id=a.nav_domain)
        rclpy.init(context=self.arm_ctx, domain_id=a.arm_domain)
        self.nav = rclpy.create_node('full_mission_nav', context=self.nav_ctx)
        self.arm = rclpy.create_node('full_mission_arm', context=self.arm_ctx)
        self.goal_pub = self.nav.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_pub = self.nav.create_publisher(Twist, '/cmd_vel', 10)
        self.move = ActionClient(self.arm, MoveGroup, 'move_action')
        self.exec = ActionClient(self.arm, ExecuteTrajectory, 'execute_trajectory')
        self.cart = self.arm.create_client(GetCartesianPath, 'compute_cartesian_path')
        self.scene = self.arm.create_client(ApplyPlanningScene, 'apply_planning_scene')
        self.js, self.samples, self.lock = None, [], threading.Lock()
        self.arm.create_subscription(JointState, '/joint_states', self._joints, 50)
        self.executors = []
        for node in (self.nav, self.arm):
            ex = SingleThreadedExecutor(context=node.context)
            ex.add_node(node)
            threading.Thread(target=ex.spin, daemon=True).start()
            self.executors.append(ex)
        self.rows, self.phase_log, self.data_frames = [], [], []
        self.orbit_fail = None
        self.phase = 'idle'

    # -------------------------------------------------------------- recording
    def _joints(self, msg):
        q = dict(zip(msg.name, msg.position))
        v = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        if not all(j in q for j in ARM_JOINTS):
            return
        t = time.monotonic()
        p, _ = self.kin.pose(q)
        dog = self.gz.dog
        with self.lock:
            self.js = q
            self.samples.append((t, self.phase, [q[j] for j in ARM_JOINTS], [v.get(j, 0.0) for j in ARM_JOINTS],
                                 p, dog[:4] if dog else (np.nan,) * 4))

    def wait(self, future, timeout):
        t0 = time.monotonic()
        while not future.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def current_q(self):
        with self.lock:
            return dict(self.js) if self.js else None

    # -------------------------------------------------------------- frames
    def dog_pose(self, timeout=10.0):
        t0 = time.monotonic()
        while self.gz.dog is None and time.monotonic() - t0 < timeout:
            time.sleep(0.1)
        return self.gz.dog

    def to_dog(self, p, dog):
        """World point -> MoveIt's 'world' frame (ground under the dog, x forward)."""
        c, s = math.cos(dog[3]), math.sin(dog[3])
        dx, dy = p[0] - dog[0], p[1] - dog[1]
        return np.array([c * dx + s * dy, -s * dx + c * dy, p[2]])

    def to_world(self, p, dog):
        c, s = math.cos(dog[3]), math.sin(dog[3])
        return np.array([dog[0] + c * p[0] - s * p[1], dog[1] + s * p[0] + c * p[1], p[2]])

    def snapshot(self, label):
        for name, topic in self.gz.cameras.items():
            if name in self.a.screen_cameras:
                self.gz.grab(topic, os.path.join(self.a.out, 'screens', f'{label}_{name}.png'))

    # -------------------------------------------------------------- phases
    def navigate(self):
        wx, wy, wyaw = self.world['waypoint']
        if self.a.standoff is not None:
            wx = self.world['rack']['front_edge_x'] - self.a.standoff
        g = PoseStamped()
        g.header.frame_id = 'lidar3d_map'
        g.pose.position.x, g.pose.position.y = wx, wy
        g.pose.orientation.z, g.pose.orientation.w = math.sin(wyaw / 2), math.cos(wyaw / 2)
        t0 = time.monotonic()
        while self.goal_pub.get_subscription_count() == 0 and time.monotonic() - t0 < 30:
            time.sleep(0.2)
        g.header.stamp = self.nav.get_clock().now().to_msg()
        self.goal_pub.publish(g)
        print(f'[mission] Nav2 goal ({wx:.3f}, {wy:.3f}), {self.goal_pub.get_subscription_count()} subscriber(s)', flush=True)
        arrived, still_since, last = False, None, None
        while time.monotonic() - t0 < self.a.nav_timeout:
            time.sleep(0.2)
            d = self.gz.dog
            if d is None:
                continue
            err = math.hypot(d[0] - wx, d[1] - wy)
            moving = last is not None and math.hypot(d[0] - last[0], d[1] - last[1]) > 0.004
            last = d
            # Nav2 stops within its 0.25 m goal tolerance (or a little outside it):
            # standing still near the waypoint is "arrived"; the precision approach does the rest
            if err < 0.45 and not moving:
                still_since = still_since or time.monotonic()
                if time.monotonic() - still_since > 3.0:
                    arrived = True
                    break
            else:
                still_since = None
        nav_err = math.hypot(self.gz.dog[0] - wx, self.gz.dog[1] - wy) if self.gz.dog else float('nan')
        print(f'[mission] Nav2 {"arrived" if arrived else "did not arrive"}, {nav_err:.3f} m from the waypoint '
              f'after {time.monotonic() - t0:.0f} s', flush=True)
        # precision approach on ground truth. Commands below ~0.05 m/s do not move
        # the dog (the cmd_vel path has a deadband), so each axis gets at least that
        # while its error is outside the tolerance.
        def axis(err, tol, gain, lo, hi):
            if abs(err) < tol:
                return 0.0
            return math.copysign(min(hi, max(lo, gain * abs(err))), err)
        t1 = time.monotonic()
        while time.monotonic() - t1 < 60:
            d = self.gz.dog
            c, s = math.cos(d[3]), math.sin(d[3])
            ex, ey = c * (wx - d[0]) + s * (wy - d[1]), -s * (wx - d[0]) + c * (wy - d[1])
            eyaw = math.atan2(math.sin(wyaw - d[3]), math.cos(wyaw - d[3]))
            if abs(ex) < 0.008 and abs(ey) < 0.008 and abs(eyaw) < 0.015:
                break
            cmd = Twist()
            cmd.linear.x = axis(ex, 0.008, 0.8, 0.05, 0.12)
            cmd.linear.y = axis(ey, 0.008, 0.8, 0.05, 0.10)
            cmd.angular.z = axis(eyaw, 0.015, 1.5, 0.08, 0.3)
            self.cmd_pub.publish(cmd)
            time.sleep(0.05)
        for _ in range(10):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)
        time.sleep(1.0)
        d = self.gz.dog
        final = math.hypot(d[0] - wx, d[1] - wy)
        ok = final < 0.03
        return ok, dict(exec_time_s=time.monotonic() - t0, notes=f'Nav2 {"arrived" if arrived else "timed out"} '
                        f'{nav_err:.3f} m off; precision approach to {final * 1000:.0f} mm, '
                        f'yaw {math.degrees(d[3] - wyaw):+.1f} deg'), (wx, wy)

    def add_scene(self, dog):
        objs = []
        for o in self.world['planning_scene']:
            co = CollisionObject()
            co.header.frame_id = 'world'
            co.id = o['name']
            co.operation = CollisionObject.ADD
            x, y, z, roll, _, _ = o['pose']
            p = self.to_dog((x, y, z), dog)
            yaw = -dog[3]
            q = (math.sin(roll / 2) * math.cos(yaw / 2), math.sin(roll / 2) * math.sin(yaw / 2),
                 math.cos(roll / 2) * math.sin(yaw / 2), math.cos(roll / 2) * math.cos(yaw / 2))
            if o['type'] == 'box':
                prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(o['size']))
            else:
                prim = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=list(o['size']))
            co.primitives.append(prim)
            co.primitive_poses.append(Pose(position=Point(x=float(p[0]), y=float(p[1]), z=float(p[2])),
                                           orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])))
            objs.append(co)
        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene(is_diff=True)
        req.scene.world.collision_objects = objs
        res = self.wait(self.scene.call_async(req), 30)
        return bool(res and res.success)

    def pose_constraints(self, pos, quat, tol_pos=0.005, tol_rot=0.03):
        c = Constraints(name='camera')
        pc = PositionConstraint()
        pc.header.frame_id = 'world'
        pc.link_name = EE_LINK
        pc.constraint_region = BoundingVolume(
            primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[tol_pos])],
            primitive_poses=[Pose(position=Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2])), orientation=Quaternion(w=1.0))])
        pc.weight = 1.0
        oc = OrientationConstraint()
        oc.header.frame_id = 'world'
        oc.link_name = EE_LINK
        oc.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        oc.absolute_x_axis_tolerance = math.pi          # free roll about the viewing axis
        oc.absolute_y_axis_tolerance = tol_rot
        oc.absolute_z_axis_tolerance = tol_rot
        oc.weight = 1.0
        c.position_constraints, c.orientation_constraints = [pc], [oc]
        return c

    def move_to(self, constraints):
        r = MotionPlanRequest()
        r.group_name = 'arm'
        r.num_planning_attempts = 10
        r.allowed_planning_time = 15.0
        r.max_velocity_scaling_factor = self.a.vel_scale
        r.max_acceleration_scaling_factor = self.a.vel_scale
        r.start_state.is_diff = True
        r.goal_constraints = [constraints]
        goal = MoveGroup.Goal(request=r, planning_options=PlanningOptions(plan_only=False))
        h = self.wait(self.move.send_goal_async(goal), 30)
        if h is None or not h.accepted:
            return False, 'goal rejected', None
        res = self.wait(h.get_result_async(), 180)
        if res is None:
            return False, 'no result', None
        code = res.result.error_code.val
        return code == SUCCESS, f'MoveIt error code {code}', res.result.planned_trajectory

    def joint_goal(self, q):
        return Constraints(name='joints', joint_constraints=[
            JointConstraint(joint_name=j, position=float(q[j]), tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            for j in ARM_JOINTS])

    def cartesian(self, waypoints, quats, avoid_collisions=True):
        req = GetCartesianPath.Request()
        req.header.frame_id = 'world'
        req.start_state = RobotState(is_diff=True)
        req.group_name = 'arm'
        req.link_name = EE_LINK
        req.waypoints = [Pose(position=Point(x=float(p[0]), y=float(p[1]), z=float(p[2])),
                              orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])) for p, q in zip(waypoints, quats)]
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = avoid_collisions
        res = self.wait(self.cart.call_async(req), 60)
        if res is None:
            return 0.0, None, 'no answer'
        traj = res.solution
        # slow the Cartesian trajectory down to vel_scale of the joint limits
        k = 1.0 / max(self.a.vel_scale, 1e-3)
        for pt in traj.joint_trajectory.points:
            t = (pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9) * k
            pt.time_from_start.sec, pt.time_from_start.nanosec = int(t), int((t - int(t)) * 1e9)
            pt.velocities = [v / k for v in pt.velocities]
            pt.accelerations = [x / (k * k) for x in pt.accelerations]
        return res.fraction, traj, f'MoveIt error code {res.error_code.val}'

    def execute(self, traj):
        h = self.wait(self.exec.send_goal_async(ExecuteTrajectory.Goal(trajectory=traj)), 30)
        if h is None or not h.accepted:
            return False, 'execution rejected'
        res = self.wait(h.get_result_async(), 180)
        if res is None:
            return False, 'no execution result'
        return res.result.error_code.val == SUCCESS, f'execution error code {res.result.error_code.val}'

    def planned_ee(self, traj):
        if traj is None:
            return np.zeros((0, 3))
        jt = traj.joint_trajectory
        return np.array([self.kin.pose(dict(zip(jt.joint_names, pt.positions)))[0] for pt in jt.points])

    # -------------------------------------------------------------- bookkeeping
    def record(self, name, t0, ok, planned, ideal=None, fraction=None, target=None, notes=''):
        t1 = time.monotonic()
        time.sleep(0.3)
        with self.lock:
            s = [x for x in self.samples if t0 <= x[0] <= t1 + 0.3]
        ee = np.array([x[4] for x in s]) if s else np.zeros((0, 3))
        vel = np.array([x[3] for x in s]) if s else np.zeros((0, 6))
        ref = ideal if ideal is not None and len(ideal) > 1 else planned
        # deviation only for a path that was executed (not for failed / skipped phases)
        dev = cross_track(ee, ref) if ok and len(ee) and ref is not None and len(ref) > 1 else np.zeros(0)
        plen = path_length(ref) if ref is not None else 0.0
        spikes = 0
        if len(vel) > 1:
            ts = np.array([x[0] for x in s])
            dv = np.abs(np.diff(vel, axis=0))
            dt = np.maximum(np.diff(ts), 1e-3)[:, None]
            vlim = np.array([VMAX[j] for j in ARM_JOINTS]) * 1.1
            alim = np.array([AMAX[j] for j in ARM_JOINTS]) * 3.0
            spikes = int((np.abs(vel) > vlim).any(axis=1).sum() + ((dv / dt) > alim).any(axis=1).sum())
        end_err = float(np.linalg.norm(ee[-1] - target)) if target is not None and len(ee) else float('nan')
        rms = float(np.sqrt(np.mean(dev ** 2))) if len(dev) else float('nan')
        row = dict(timestamp=time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(time.time() - (t1 - t0))), phase=name,
                   exec_time_s=round(t1 - t0, 2), ee_distance_m=round(path_length(ee), 4),
                   planned_length_m=round(plen, 4), success=bool(ok),
                   cartesian_fraction='' if fraction is None else round(fraction, 3),
                   mean_dev_mm=round(float(dev.mean()) * 1000, 2) if len(dev) else '',
                   max_dev_mm=round(float(dev.max()) * 1000, 2) if len(dev) else '',
                   dev_pct=round(100 * rms / plen, 2) if len(dev) and plen > 1e-3 else '',
                   end_error_mm=round(end_err * 1000, 1) if end_err == end_err else '',
                   joint_velocity_spikes=spikes, notes=notes)
        self.rows.append(row)
        self.phase_log.append(dict(name=name, t0=t0, t1=t1, planned=None if planned is None else np.asarray(planned).tolist(),
                                   ideal=None if ideal is None else np.asarray(ideal).tolist()))
        print(f"[mission] {name}: {'OK' if ok else 'FAILED'} in {row['exec_time_s']} s, EE {row['ee_distance_m']} m, "
              f"dev {row['mean_dev_mm']} mm mean / {row['max_dev_mm']} mm max ({row['dev_pct']} %), "
              f"spikes {spikes}. {notes}", flush=True)
        return row

    # -------------------------------------------------------------- run
    def run(self):
        a = self.a
        print('[mission] waiting for MoveIt and the arm joint states...', flush=True)
        for c in (self.move, self.exec):
            c.wait_for_server(timeout_sec=120)
        self.cart.wait_for_service(timeout_sec=60)
        self.scene.wait_for_service(timeout_sec=60)
        t0 = time.monotonic()
        while self.current_q() is None and time.monotonic() - t0 < 60:
            time.sleep(0.2)
        if self.dog_pose() is None:
            raise SystemExit('no ground-truth pose from Gazebo (/model/go2/odometry)')

        # 0. Setup: fold the arm into the start pose (stow) for walking. Not a scored phase.
        self.phase = 'Setup'
        ok_stow, why, _ = self.move_to(self.joint_goal(STOW))
        print(f"[mission] setup: arm {'stowed' if ok_stow else 'NOT stowed (' + why + ')'}", flush=True)

        # 1. Navigation
        self.phase = 'Navigation'
        t0 = time.monotonic()
        if a.skip_nav:
            ok, info, wp = True, dict(notes='skipped (--skip-nav)'), self.world['waypoint'][:2]
        else:
            ok, info, wp = self.navigate()
        self.record('Navigation', t0, ok, None, notes=info['notes'])
        self.rows[-1]['exec_time_s'] = round(info.get('exec_time_s', self.rows[-1]['exec_time_s']), 2)
        self.snapshot('1_navigation')

        dog = self.dog_pose()
        plant = self.to_dog(self.world['target_plant']['center'], dog)
        self.add_scene(dog)
        az0 = math.atan2(plant[1], plant[0])
        el = math.radians(a.look_up_deg)
        print(f"[mission] dog at ({dog[0]:.3f}, {dog[1]:.3f}, yaw {math.degrees(dog[3]):+.1f}); plant in the dog frame "
              f"({plant[0]:.3f}, {plant[1]:.3f}, {plant[2]:.3f}), {math.hypot(plant[0], plant[1]):.3f} m ahead", flush=True)

        def cam(d, az_off):
            v = view_dir(az0 + az_off, el)
            p = plant - d * v
            return p, look_at(p, plant)

        # 2. Approach (planned joint-space move to the pre-grasp pose)
        # A Cartesian path never changes IK branch, so the branch is chosen here:
        # the pre-grasp IK solution with the smallest joint angles, found from the
        # extend pose's solution (the branch that can follow the whole orbit, in the
        # reach study). MoveIt plans the collision-free joint-space move to it.
        self.phase = 'Approach'
        pre_p, pre_q = cam(a.pregrasp, 0.0)
        R_view = upright(view_dir(az0, el))
        lim = self.kin.limits(a.urdf)
        seeds = [np.zeros(6)] + [np.array(s) for s in
                                 ((0, 0.5, 0.8, 0, 0.3, 0), (0, -0.3, 1.2, 0, 0.6, 0), (0, 0.8, 0.4, 0, -0.4, 0),
                                  (0, 0.3, 1.0, 1.5, 0.5, -1.5), (0, 0.3, 1.0, -1.5, 0.5, 1.5))]
        q_ext = self.kin.ik(plant - a.radius * view_dir(az0, el), R_view, seeds, lim)
        q_pre = self.kin.ik(pre_p, R_view, [np.array([q_ext[j] for j in ARM_JOINTS])] if q_ext else seeds, lim)
        t0 = time.monotonic()
        if q_pre is not None:
            ok_app, why, traj = self.move_to(self.joint_goal(q_pre))
            how = 'IK branch chosen for the orbit'
        else:   # no IK solution: let MoveIt try the pose itself
            ok_app, why, traj = self.move_to(self.pose_constraints(pre_p, pre_q))
            how = 'no IK solution found here; MoveIt pose goal'
        self.record('Approach', t0, ok_app, self.planned_ee(traj), target=pre_p, notes=(why + '; ' + how) if not ok_app else
                    f'pre-grasp {a.pregrasp:.2f} m from the plant, camera {a.look_up_deg:.0f} deg up at its centre ({how})')
        self.snapshot('2_approach')

        # From here on the camera keeps the orientation it reached at the
        # pre-grasp: Cartesian moves only translate it (extend, retract) or turn it
        # about the vertical axis through the plant (orbit).
        def ee_now():
            q = self.current_q()
            return self.kin.pose(q) if q else (None, None)

        # 3. Extend (Cartesian line to radius)
        self.phase = 'Extend'
        ext_p, _ = cam(a.radius, 0.0)
        t0 = time.monotonic()
        if ok_app:
            p_now, R_now = ee_now()
            pre_p, pre_q = p_now, quat_from_matrix(R_now)        # where the approach really ended
            ext_q = pre_q
            line = [pre_p + (ext_p - pre_p) * s for s in np.linspace(0, 1, 11)]
            frac, traj, why = self.cartesian([ext_p], [ext_q])
            ok_ext = frac > 0.999 and self.execute(traj)[0] if traj is not None else False
            self.record('Extend', t0, ok_ext, self.planned_ee(traj), ideal=np.array(line), fraction=frac, target=ext_p,
                        notes=f'Cartesian fraction {frac:.3f}' + ('' if ok_ext else f' ({why})'))
        else:
            ok_ext = False
            self.record('Extend', t0, False, None, fraction=0.0, notes='skipped: Approach failed')
        self.snapshot('3_extend')

        # 4. Orbit (Cartesian arc, camera aimed at the plant centre), in segments:
        #    lead-in 0 -> -half, then -half -> +half in four steps, then lead-out +half -> 0.
        #    The wrist camera's RGB and depth are saved at the end of the lead-in and of each step.
        self.phase = 'Orbit'
        half = math.radians(a.orbit_deg / 2)
        legs = [(0.0, -half, None)] + [(-half + k * half / 2, -half + (k + 1) * half / 2, -half + (k + 1) * half / 2)
                                       for k in range(4)] + [(half, 0.0, None)]
        legs[0] = (0.0, -half, -half)
        # the orbit turns the camera's actual extend pose about the vertical axis through the plant:
        # a circle around the plant, the camera still aimed at its centre, roll unchanged
        orb_p, orb_R = (ee_now() if ok_ext else (ext_p, np.eye(3)))

        def orbit_pose(o):
            Rz = axis_matrix([0, 0, 1], o)
            return plant + Rz @ (orb_p - plant), quat_from_matrix(Rz @ orb_R)
        ideal = np.array([orbit_pose(o)[0] for f0, f1, _ in legs
                          for o in np.linspace(f0, f1, max(2, int(abs(f1 - f0) / math.radians(3)) + 1))])
        t0 = time.monotonic()
        ok_orb, fracs, planned = ok_ext, [], []
        if ok_ext:
            for f0, f1, capture in legs:
                offs = np.linspace(f0, f1, max(2, int(abs(f1 - f0) / math.radians(3)) + 1))[1:]
                pts = [orbit_pose(o) for o in offs]
                frac, traj, why = self.cartesian([p for p, _ in pts], [q for _, q in pts])
                fracs.append(frac)
                planned.append(self.planned_ee(traj))
                if traj is None or frac < 0.999 or not self.execute(traj)[0]:
                    ok_orb = False
                    # diagnosis: the same segment without collision checking tells reach from collision
                    frac_free = self.cartesian([p for p, _ in pts], [q for _, q in pts], avoid_collisions=False)[0]
                    cause = 'a collision with the rack / plants' if frac_free > 0.999 else 'the arm\'s reach (IK)'
                    self.orbit_fail = (math.degrees(f0), math.degrees(f1), frac, frac_free, cause)
                    print(f'[mission] orbit segment {math.degrees(f0):+.1f} -> {math.degrees(f1):+.1f} deg failed: '
                          f'fraction {frac:.3f} ({frac_free:.3f} without collision checking: limited by {cause})', flush=True)
                    break
                if capture is not None:
                    time.sleep(0.6)
                    angle = math.degrees(capture)
                    for kind in ('image', 'depth_image'):
                        path = os.path.join(a.out, 'screens', f'wrist_{kind}_{angle:+05.1f}.png')
                        if self.gz.grab(f'wrist_camera/{kind}', path):
                            self.data_frames.append(dict(angle_deg=round(angle, 1), kind=kind, file=os.path.basename(path)))
                    if abs(capture) < 1e-6:
                        self.snapshot('4_orbit')
            frac = min(fracs) if fracs else 0.0
            self.record('Orbit', t0, ok_orb, np.vstack([p for p in planned if len(p)]) if any(len(p) for p in planned) else None,
                        ideal=ideal, fraction=frac, target=ext_p,
                        notes=f'{a.orbit_deg:.0f} deg at {a.radius:.2f} m in {len(fracs)}/{len(legs)} segments, '
                        f'{len(self.data_frames)} wrist frames' + (
                            '' if ok_orb or not self.orbit_fail else
                            f'; stopped in {self.orbit_fail[0]:+.0f} -> {self.orbit_fail[1]:+.0f} deg at fraction '
                            f'{self.orbit_fail[2]:.2f} ({self.orbit_fail[3]:.2f} without collision checking: {self.orbit_fail[4]})'))
        else:
            ok_orb = False
            self.record('Orbit', t0, False, None, ideal=ideal, fraction=0.0, notes='skipped: Extend failed')
            self.snapshot('4_orbit')

        # 5. Retract (Cartesian line back to the pre-grasp pose)
        self.phase = 'Retract'
        t0 = time.monotonic()
        if ok_ext:
            start, R_now = ee_now()
            frac, traj, why = self.cartesian([pre_p], [quat_from_matrix(R_now)])
            ok_ret = frac > 0.999 and self.execute(traj)[0] if traj is not None else False
            self.record('Retract', t0, ok_ret, self.planned_ee(traj),
                        ideal=np.array([start + (pre_p - start) * s for s in np.linspace(0, 1, 11)]),
                        fraction=frac, target=pre_p, notes=f'Cartesian fraction {frac:.3f}' + ('' if ok_ret else f' ({why})'))
        else:
            ok_ret = False
            self.record('Retract', t0, False, None, fraction=0.0, notes='skipped: Extend failed')
        self.snapshot('5_retract')

        # 6. Home: back to the start pose (stow)
        self.phase = 'Home'
        t0 = time.monotonic()
        ok_home, why, traj = self.move_to(self.joint_goal(STOW))
        self.record('Home', t0, ok_home, self.planned_ee(traj), target=self.kin.pose(STOW)[0],
                    notes='back to the start pose (stow)' if ok_home else why)
        self.snapshot('6_home')
        self.phase = 'done'
        return self.finish(plant)

    # -------------------------------------------------------------- results
    def finish(self, plant):
        a = self.a
        with open(os.path.join(a.out, 'hydroponic_trial.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            w.writeheader()
            w.writerows(self.rows)
        with self.lock:
            s = list(self.samples)
        np.savez_compressed(os.path.join(a.out, 'samples.npz'), t=np.array([x[0] for x in s]),
                            phase=np.array([x[1] for x in s]), q=np.array([x[2] for x in s]),
                            v=np.array([x[3] for x in s]), ee=np.array([x[4] for x in s]),
                            dog=np.array([x[5] for x in s]))
        # wall_minus_mono: phase times are monotonic, Gazebo contact logs are wall clock
        json.dump(dict(phases=self.phase_log, plant_dog_frame=plant.tolist(), data_frames=self.data_frames,
                       wall_minus_mono=time.time() - time.monotonic(), rows=self.rows,
                       args=vars(a)), open(os.path.join(a.out, 'phases.json'), 'w'), indent=1)
        print(f'[mission] wrote {a.out}', flush=True)
        return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world-json', default='/sim/worlds/hydroponic_station.json')
    ap.add_argument('--urdf', default='/tmp/sim-gen/d1_moveit.urdf')
    ap.add_argument('--out', default='/tmp/hydroponic_trial')
    ap.add_argument('--nav-domain', type=int, default=64)
    ap.add_argument('--arm-domain', type=int, default=66)
    ap.add_argument('--skip-nav', action='store_true', help='the dog is already at the waypoint')
    ap.add_argument('--standoff', type=float, default=None,
                    help='dog base distance from the stand front edge (default: the world\'s waypoint, 0.6 m)')
    ap.add_argument('--nav-timeout', type=float, default=240.0)
    ap.add_argument('--pregrasp', type=float, default=0.30)
    ap.add_argument('--radius', type=float, default=0.15)
    ap.add_argument('--orbit-deg', type=float, default=90.0)
    ap.add_argument('--look-up-deg', type=float, default=30.0,
                    help='camera below the plant centre, looking up at it (reach; see the report)')
    ap.add_argument('--vel-scale', type=float, default=0.2)
    ap.add_argument('--screen-cameras', nargs='*', default=['side', 'front'])
    a, _ = ap.parse_known_args()
    m = Mission(a)
    code = 1
    try:
        code = m.run()
    finally:
        for ex in m.executors:
            ex.shutdown()
        for node in (m.nav, m.arm):
            node.destroy_node()
        for ctx in (m.nav_ctx, m.arm_ctx):
            rclpy.try_shutdown(context=ctx)
    return code


if __name__ == '__main__':
    # os._exit: the gz-transport node's threads abort the interpreter's normal teardown
    import sys
    sys.stdout.flush()
    os._exit(main())
