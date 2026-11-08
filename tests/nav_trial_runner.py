#!/usr/bin/env python3
"""One navigation trial: sends a fixed goal list, records what happened.

Goals go to /goal_pose once, the path the dashboard and RViz use. A leg ends:
  reached  body within GOAL_TOLERANCE of the goal
  timeout  after GOAL_TIMEOUT seconds
  stuck    less than STUCK_PROGRESS m of progress in STUCK_SECONDS

The active follow_path is cancelled when a leg ends, so a late leg cannot run
into the next goal's time. A leg that stops early (Nav2 abort) is re-sent after
RESEND_STILL seconds without motion; the leg's clock keeps running.

Writes to --out:
  trajectory.csv  t, x, y, yaw of base_link at 20 Hz
  legs.json       one entry per leg
  events.jsonl    Nav2 errors and warnings from /rosout, gait commands, 3D boxes

Scoring is done afterwards by tests/traversability/analyze.py against the
world's true geometry.
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import rclpy
from action_msgs.msg import GoalInfo
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import Log
from std_msgs.msg import String

GOALS = [
    {"name": "goal_1_mid", "x": 5.0, "y": 0.0, "yaw": 0.0},
    {"name": "goal_2_end", "x": 10.0, "y": 0.0, "yaw": 0.0},
    {"name": "goal_3_return", "x": 0.0, "y": 0.0, "yaw": 3.14},
]

GOAL_TOLERANCE = 0.5
GOAL_TIMEOUT = 120.0
STUCK_SECONDS = 45.0
STUCK_PROGRESS = 0.15
RESEND_STILL = 8.0
STILL_SPEED = 0.02
LOG_NODES = {"controller_server", "planner_server", "go2_goal_pose_nav"}


def odom_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class TrialRunner(Node):
    def __init__(self, out_dir):
        super().__init__("nav_trial_runner")
        self.out_dir = out_dir
        self.pose = None
        self.last_write = 0.0
        self.traj = open(os.path.join(out_dir, "trajectory.csv"), "w", newline="")
        self.traj_csv = csv.writer(self.traj)
        self.traj_csv.writerow(["t", "x", "y", "yaw"])
        self.events = open(os.path.join(out_dir, "events.jsonl"), "w", buffering=1)
        self.t0 = time.time()

        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.cancel = self.create_client(CancelGoal, "/follow_path/_action/cancel_goal")
        sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, "/lidar3d/body_odom", self.on_odom, sensor)
        self.create_subscription(
            Log, "/rosout", self.on_log,
            QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))
        self.create_subscription(String, "/gait/status", self.on_string("gait"), 10)
        self.create_subscription(String, "/objects/boxes_json", self.on_string("boxes"), 10)
        self.feet = open(os.path.join(out_dir, "feet.jsonl"), "w", buffering=1)
        self.create_subscription(String, "/sim/feet", self.on_feet, 50)

    def now(self):
        return time.time() - self.t0

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, odom_yaw(msg.pose.pose.orientation))
        t = self.now()
        if t - self.last_write >= 0.05:
            self.last_write = t
            self.traj_csv.writerow([round(t, 3), round(p.x, 4), round(p.y, 4), round(self.pose[2], 4)])

    def on_log(self, msg):
        if msg.name in LOG_NODES and msg.level >= 30:
            self.events.write(json.dumps(
                {"t": round(self.now(), 2), "kind": "nav2", "node": msg.name,
                 "level": msg.level, "msg": msg.msg}) + "\n")

    def on_feet(self, msg):
        self.feet.write(msg.data + "\n")

    def on_string(self, kind):
        def cb(msg):
            self.events.write(json.dumps(
                {"t": round(self.now(), 2), "kind": kind, "data": msg.data}) + "\n")
        return cb

    def send_goal(self, g):
        msg = PoseStamped()
        msg.header.frame_id = "lidar3d_map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = g["x"]
        msg.pose.position.y = g["y"]
        msg.pose.orientation.z = math.sin(g["yaw"] / 2.0)
        msg.pose.orientation.w = math.cos(g["yaw"] / 2.0)
        self.goal_pub.publish(msg)

    def cancel_all(self):
        if not self.cancel.service_is_ready():
            return
        req = CancelGoal.Request()
        req.goal_info = GoalInfo()
        self.cancel.call_async(req)
        self.spin_for(1.5)

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def run_leg(self, g):
        start = self.pose
        straight = math.hypot(g["x"] - start[0], g["y"] - start[1])
        t0 = self.now()
        self.send_goal(g)
        best = straight
        best_t = t0
        last_xy, last_moved = start[:2], t0
        resends = 0
        result = "timeout"
        while self.now() - t0 < GOAL_TIMEOUT:
            rclpy.spin_once(self, timeout_sec=0.02)
            x, y, _ = self.pose
            d = math.hypot(g["x"] - x, g["y"] - y)
            if d < GOAL_TOLERANCE:
                result = "reached"
                break
            if d < best - STUCK_PROGRESS:
                best, best_t = d, self.now()
            if self.now() - best_t > STUCK_SECONDS:
                result = "stuck"
                break
            if math.hypot(x - last_xy[0], y - last_xy[1]) > STILL_SPEED:
                last_xy, last_moved = (x, y), self.now()
            elif self.now() - last_moved > RESEND_STILL:
                self.cancel_all()
                self.send_goal(g)
                resends += 1
                last_moved = self.now()
        t1 = self.now()
        self.cancel_all()
        end = self.pose
        leg = {"goal": g["name"], "goal_x": g["x"], "goal_y": g["y"], "result": result,
               "t_start": round(t0, 2), "t_end": round(t1, 2), "time_s": round(t1 - t0, 2),
               "start_x": round(start[0], 3), "start_y": round(start[1], 3),
               "end_x": round(end[0], 3), "end_y": round(end[1], 3),
               "resends": resends}
        self.get_logger().info(json.dumps(leg))
        return leg

    def run(self, goals):
        end = time.time() + 30
        while self.pose is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.pose is None:
            raise SystemExit("no /lidar3d/body_odom")
        legs = [self.run_leg(g) for g in goals]
        with open(os.path.join(self.out_dir, "legs.json"), "w") as f:
            json.dump(legs, f, indent=1)
        self.traj.close()
        self.events.close()
        self.feet.close()
        return legs


def main():
    global GOAL_TIMEOUT
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--randomize", action="store_true", help="seeded spawn and goals from --world (route.py)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--world", default="/tmp/trav/cluttered_items.json")
    ap.add_argument("--timeout", type=float, default=GOAL_TIMEOUT)
    a = ap.parse_args()
    GOAL_TIMEOUT = a.timeout
    if a.randomize:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from route import sample
        route = sample(json.load(open(a.world)), a.seed)
    else:
        route = {"seed": None, "spawn": [0.0, 0.0, 0.0], "goals": GOALS}
    with open(os.path.join(a.out, "trial.json"), "w") as f:
        json.dump({"mode": a.mode, "randomize": a.randomize, **route, "goal_timeout_s": GOAL_TIMEOUT}, f, indent=1)
    rclpy.init()
    node = TrialRunner(a.out)
    legs = node.run(route["goals"])
    print(f"{a.mode}: " + ", ".join(f"{l['goal']}={l['result']} {l['time_s']}s" for l in legs))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
