#!/usr/bin/env python3
"""
nav_tour.py: drive a fixed list of goals (one per room) after exploration and
log how each leg went, for scripts/tools/nav_compare (analyze.py).

Runs inside go2-lidar3d next to the session, like pose_logger.py. Pauses the
explorer, then for each goal publishes a PoseStamped on /goal_pose (facing
the way from the robot to the goal), the same path the explorer and the
dashboard use, and waits until:
  reached     the robot (TF lidar3d_map -> base_link) is within --tol of it
  timeout     --base-s + --per-m-s per metre of straight-line distance passed
  stuck       it moved less than 0.15 m in --stuck-s seconds
Nav2 aborts on the planner / controller actions are counted per leg (the
stuck recovery may re-send the goal after one, so an abort alone does not
end the leg). A failed leg moves on to the next goal from wherever the
robot stopped.

  python3 nav_tour.py --goals '[{"name": "lab", "x": 1.7, "y": 3.6}, ...]' --out /tmp/nav_trial

Writes OUT/tour.csv:
  idx,name,gx,gy,t_start,t_end,result,straight_m,path_m,final_err_m,planner_aborts,controller_aborts
"""

import argparse
import json
import math
import os
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


class Tour(Node):
    def __init__(self, a):
        super().__init__('nav_compare_tour')
        self.a = a
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.resume_pub = self.create_publisher(Bool, '/explore/resume', 10)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.seen = set()          # goal ids already terminal
        self.aborts = {'planner': 0, 'controller': 0}
        for kind, action in (('planner', '/compute_path_to_pose'), ('controller', '/follow_path')):
            self.create_subscription(GoalStatusArray, f'{action}/_action/status',
                                     lambda m, k=kind: self.on_status(k, m), qos)

    def on_status(self, kind, msg):
        for st in msg.status_list:
            gid = bytes(st.goal_info.goal_id.uuid)
            if gid in self.seen or st.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                                                     GoalStatus.STATUS_CANCELED):
                continue
            self.seen.add(gid)
            if st.status == GoalStatus.STATUS_ABORTED:
                self.aborts[kind] += 1

    def pose(self):
        try:
            t = self.tf.lookup_transform(self.a.map_frame, self.a.base_frame, Time())
        except Exception:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def spin_for(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def send(self, x, y, yaw):
        g = PoseStamped()
        g.header.frame_id = self.a.map_frame
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = x, y
        g.pose.orientation.z, g.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.goal_pub.publish(g)

    def leg(self, goal):
        p = None
        while p is None:
            self.spin_for(0.2)
            p = self.pose()
        gx, gy = goal['x'], goal['y']
        straight = math.hypot(gx - p[0], gy - p[1])
        limit = self.a.base_s + self.a.per_m_s * straight
        before = dict(self.aborts)
        t0 = time.time()
        self.send(gx, gy, math.atan2(gy - p[1], gx - p[0]))
        path, last, prog_xy, prog_t, result = 0.0, p, p, t0, 'timeout'
        while time.time() - t0 < limit:
            self.spin_for(0.2)
            q = self.pose()
            if q is None:
                continue
            step = math.hypot(q[0] - last[0], q[1] - last[1])
            if step < 1.0:            # skip TF jumps
                path += step
            last = q
            if math.hypot(gx - q[0], gy - q[1]) < self.a.tol:
                result = 'reached'
                break
            if math.hypot(q[0] - prog_xy[0], q[1] - prog_xy[1]) > 0.15:
                prog_xy, prog_t = q, time.time()
            elif time.time() - prog_t > self.a.stuck_s:
                result = 'stuck'
                break
        return dict(t_start=t0, t_end=time.time(), result=result, straight_m=straight, path_m=path,
                    final_err_m=math.hypot(gx - last[0], gy - last[1]),
                    planner_aborts=self.aborts['planner'] - before['planner'],
                    controller_aborts=self.aborts['controller'] - before['controller'])

    def run(self, goals):
        os.makedirs(self.a.out, exist_ok=True)
        path = os.path.join(self.a.out, 'tour.csv')
        cols = ['idx', 'name', 'gx', 'gy', 't_start', 't_end', 'result', 'straight_m', 'path_m',
                'final_err_m', 'planner_aborts', 'controller_aborts']
        with open(path, 'w', buffering=1) as f:
            f.write(','.join(cols) + '\n')
            for _ in range(5):               # the explorer must not send goals meanwhile
                self.resume_pub.publish(Bool(data=False))
                self.spin_for(0.4)
            self.spin_for(3.0)
            for i, g in enumerate(goals):
                r = self.leg(g)
                row = dict(idx=i, name=g['name'], gx=g['x'], gy=g['y'], **r)
                f.write(','.join(f'{row[c]:.3f}' if isinstance(row[c], float) else str(row[c]) for c in cols) + '\n')
                self.get_logger().info(f"tour {i + 1}/{len(goals)} {g['name']}: {r['result']} "
                                       f"in {r['t_end'] - r['t_start']:.0f} s, path {r['path_m']:.1f} m")
                self.spin_for(2.0)
        open(os.path.join(self.a.out, 'tour.done'), 'w').close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--goals', required=True, help='JSON list of {"name", "x", "y"}, or a file with it')
    ap.add_argument('--out', default='/tmp/nav_trial')
    ap.add_argument('--tol', type=float, default=0.4, help='reached within this distance (m)')
    ap.add_argument('--base-s', type=float, default=60.0)
    ap.add_argument('--per-m-s', type=float, default=6.0)
    ap.add_argument('--stuck-s', type=float, default=45.0)
    ap.add_argument('--map-frame', default='lidar3d_map')
    ap.add_argument('--base-frame', default='base_link')
    a, ros_args = ap.parse_known_args()
    goals = json.load(open(a.goals)) if os.path.exists(a.goals) else json.loads(a.goals)
    rclpy.init(args=ros_args)
    node = Tour(a)
    try:
        node.run(goals)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
