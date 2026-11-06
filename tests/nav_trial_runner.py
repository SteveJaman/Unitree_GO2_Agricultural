#!/usr/bin/env python3
"""Navigation trial runner: sends goals, records metrics for comparison.

Sends a sequence of goal poses and records:
  - Time to reach each goal
  - Total path length (from odom)
  - Minimum clearance to obstacles (from costmap)
  - Number of high-cost cell crossings (step_over, caution, lethal)
  - Whether the goal was reached or timed out

Usage (inside the go2-lidar3d container with Nav2 running):
    python3 nav_trial_runner.py --mode traversability --output /tmp/trial_results.json
    python3 nav_trial_runner.py --mode baseline --output /tmp/baseline_results.json
"""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String

COSTMAP_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

SENSOR_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
)

GOALS = [
    {"name": "goal_1_near", "x": -2.0, "y": 0.0, "yaw": 0.0},
    {"name": "goal_2_mid", "x": 1.0, "y": 0.0, "yaw": 0.0},
    {"name": "goal_3_return", "x": -4.5, "y": 0.0, "yaw": 3.14},
]

GOAL_TOLERANCE = 0.75
GOAL_TIMEOUT = 90.0

STEP_OVER_MIN = 10
CAUTION_MIN = 60
LETHAL_MIN = 100


class TrialRunner(Node):
    def __init__(self, mode):
        super().__init__("nav_trial_runner")
        self.mode = mode
        self.odom = None
        self.costmap = None
        self.path_length = 0.0
        self.prev_x = None
        self.prev_y = None
        self.step_over_crossings = 0
        self.caution_crossings = 0
        self.lethal_crossings = 0
        self.min_clearance = float("inf")

        self.goal_pub = self.create_publisher(
            PoseStamped, "/goal_pose", 10)
        self.create_subscription(
            Odometry, "/lidar3d/odom", self._odom_cb, SENSOR_QOS)
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self._cm_cb, COSTMAP_QOS)

    def _odom_cb(self, msg):
        self.odom = msg
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.prev_x is not None:
            dx = x - self.prev_x
            dy = y - self.prev_y
            self.path_length += math.sqrt(dx * dx + dy * dy)
        self.prev_x = x
        self.prev_y = y

        if self.costmap is not None:
            self._record_terrain(x, y)

    def _cm_cb(self, msg):
        self.costmap = msg

    def _record_terrain(self, rx, ry):
        if self.costmap is None:
            return
        info = self.costmap.info
        mx = int((rx - info.origin.position.x) / info.resolution)
        my = int((ry - info.origin.position.y) / info.resolution)
        if 0 <= mx < info.width and 0 <= my < info.height:
            c = self.costmap.data[my * info.width + mx]
            if c >= LETHAL_MIN:
                self.lethal_crossings += 1
            elif c >= CAUTION_MIN:
                self.caution_crossings += 1
            elif c >= STEP_OVER_MIN:
                self.step_over_crossings += 1

        min_d = float("inf")
        search_r = 20
        for dx in range(-search_r, search_r + 1):
            for dy in range(-search_r, search_r + 1):
                ix, iy = mx + dx, my + dy
                if 0 <= ix < info.width and 0 <= iy < info.height:
                    c = self.costmap.data[iy * info.width + ix]
                    if c >= LETHAL_MIN:
                        d = math.sqrt(dx * dx + dy * dy) * info.resolution
                        if d < min_d:
                            min_d = d
        if min_d < self.min_clearance:
            self.min_clearance = min_d

    def send_goal(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = "lidar3d_map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(msg)

    def wait_for_goal(self, gx, gy, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.odom is None:
                continue
            rx = self.odom.pose.pose.position.x
            ry = self.odom.pose.pose.position.y
            dist = math.sqrt((rx - gx) ** 2 + (ry - gy) ** 2)
            if dist < GOAL_TOLERANCE:
                return True, time.time() - t0
        return False, timeout

    def run_trials(self):
        self.get_logger().info("Waiting for odom...")
        t0 = time.time()
        while self.odom is None and time.time() - t0 < 30:
            rclpy.spin_once(self, timeout_sec=1.0)
        if self.odom is None:
            self.get_logger().error("No odom!")
            return []

        results = []
        for goal in GOALS:
            self.get_logger().info(
                "Sending goal %s: (%.1f, %.1f)" % (goal["name"], goal["x"], goal["y"]))

            start_length = self.path_length
            start_step = self.step_over_crossings
            start_caution = self.caution_crossings
            start_lethal = self.lethal_crossings
            self.min_clearance = float("inf")

            self.send_goal(goal["x"], goal["y"], goal["yaw"])
            time.sleep(1.0)
            self.send_goal(goal["x"], goal["y"], goal["yaw"])

            reached, elapsed = self.wait_for_goal(goal["x"], goal["y"], GOAL_TIMEOUT)
            segment_length = self.path_length - start_length

            result = {
                "goal": goal["name"],
                "reached": reached,
                "time_s": round(elapsed, 2),
                "path_length_m": round(segment_length, 3),
                "step_over_crossings": self.step_over_crossings - start_step,
                "caution_crossings": self.caution_crossings - start_caution,
                "lethal_crossings": self.lethal_crossings - start_lethal,
                "min_clearance_m": round(self.min_clearance, 3) if self.min_clearance < 100 else -1,
            }
            results.append(result)
            status = "REACHED" if reached else "TIMEOUT"
            self.get_logger().info(
                "  %s: %s in %.1fs, path=%.2fm, clearance=%.2fm"
                % (goal["name"], status, elapsed, segment_length,
                   result["min_clearance_m"]))

        return results


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="traversability",
                        choices=["traversability", "baseline"])
    parser.add_argument("--output", default="/tmp/trial_results.json")
    args = parser.parse_args()

    rclpy.init()
    node = TrialRunner(args.mode)

    results = node.run_trials()

    summary = {
        "mode": args.mode,
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "goals": results,
        "totals": {
            "goals_reached": sum(1 for r in results if r["reached"]),
            "goals_total": len(results),
            "total_time_s": round(sum(r["time_s"] for r in results), 2),
            "total_path_length_m": round(sum(r["path_length_m"] for r in results), 3),
            "total_step_over": sum(r["step_over_crossings"] for r in results),
            "total_caution": sum(r["caution_crossings"] for r in results),
            "total_lethal": sum(r["lethal_crossings"] for r in results),
        },
    }

    with open(args.output, "w") as f:
        json.dump(summary, f, indent=2)

    print("\n" + "=" * 60)
    print("Trial Summary: %s mode" % args.mode)
    print("=" * 60)
    print("Goals reached: %d / %d" % (
        summary["totals"]["goals_reached"], summary["totals"]["goals_total"]))
    print("Total time: %.1fs" % summary["totals"]["total_time_s"])
    print("Total path length: %.2fm" % summary["totals"]["total_path_length_m"])
    print("Terrain crossings: step_over=%d caution=%d lethal=%d" % (
        summary["totals"]["total_step_over"],
        summary["totals"]["total_caution"],
        summary["totals"]["total_lethal"]))
    print("Output: %s" % args.output)
    print("=" * 60)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
