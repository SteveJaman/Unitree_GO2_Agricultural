#!/usr/bin/env python3
"""Autonomous inspection: walk to a clicked 3D point, then aim the arm at it.

The dashboard's Inspect mode publishes the exact 3D point under the cursor
(on the map surface, not the floor) on /inspect_point. This node:

  1. Stores the target (x, y, z) in the map frame and pauses the explorer.
  2. Sends a Nav2 goal STANDOFF m short of the target, on the robot's side of
     it, snapped to safe floor exactly like go2_click_to_goal.py (the target
     itself is usually on a table or plant, not floor), with the yaw that
     faces the target.
  3. Watches map -> base_link. Once the dog is within TRIGGER_DISTANCE of the
     target, or has stopped at the goal with the target still within
     MAX_SCAN_DISTANCE (a table can keep it further away), it turns the
     target into base_link (dx, dy, dz) and sends {"op": "scan_target",
     "target": [dx, dy, dz]} on /d1_arm/command. arm_state_bridge.py runs
     start_arm.sh scan (arm_ik_scanner.py: IK aim at the point, sweep left
     to right across it, home, release).
  4. Follows the scan on /d1_arm/status until it is done.

It listens on its own /inspect_point, not /clicked_point: go2_click_to_goal
already turns /clicked_point into a goal, and one click must not send two.

Subscribes: /inspect_point (PointStamped, any frame), /inspection/command
            (String JSON: {"op": "cancel"}), /global_costmap/costmap,
            /d1_arm/status (String JSON, arm_state_bridge.py)
Publishes:  /goal_pose, /explore/resume (false), /d1_arm/command,
            /inspection/target (PointStamped, latched: the dashboard marker),
            /inspection/status (String JSON, 2 Hz:
              {"state": "idle|navigating|scanning|done|failed|unreachable|timeout|cancelled",
               "target": [x, y, z], "goal": [x, y], "distance": m, "note": "..."})
"""

import json
import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped with tf2)
from tf2_ros import Buffer, TransformException, TransformListener

from go2_click_to_goal import ClickToGoal

ACTIVE = ("navigating", "scanning")


class InspectionMission(Node):
    # Safe-floor snapping, shared with the click-to-goal node.
    cell_free = ClickToGoal.cell_free
    cell_safe = ClickToGoal.cell_safe
    snap_to_floor = ClickToGoal.snap_to_floor

    def __init__(self):
        super().__init__("inspection_mission")
        p = self.declare_parameter
        p("input_topic", "/inspect_point")
        p("costmap_topic", "/global_costmap/costmap")
        p("map_frame", "lidar3d_map")
        p("base_frame", "base_link")
        p("standoff", 0.45)  # m: goal this far short of the target
        p("trigger_distance", 0.5)  # m from the target (horizontal): scan
        p(
            "max_scan_distance", 1.0
        )  # m: stopped at the goal and this close: scan anyway
        p("arrive_tolerance", 0.35)  # m from the goal counted as there
        p("nav_timeout", 180.0)  # s
        p("scan_timeout", 180.0)  # s
        p("free_threshold", 50)
        p("clearance_cells", 1)
        p("max_snap_distance", 1.0)
        v = lambda n: self.get_parameter(n).value  # noqa: E731
        self.map_frame, self.base_frame = v("map_frame"), v("base_frame")
        self.standoff, self.trigger = float(v("standoff")), float(v("trigger_distance"))
        self.max_scan, self.arrive_tol = (
            float(v("max_scan_distance")),
            float(v("arrive_tolerance")),
        )
        self.nav_timeout, self.scan_timeout = (
            float(v("nav_timeout")),
            float(v("scan_timeout")),
        )
        self.free_threshold = int(v("free_threshold"))
        self.clearance = max(0, int(v("clearance_cells")))
        self.max_snap = float(v("max_snap_distance"))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.explore_pub = self.create_publisher(Bool, "/explore/resume", 10)
        self.arm_pub = self.create_publisher(String, "/d1_arm/command", 10)
        self.status_pub = self.create_publisher(String, "/inspection/status", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.target_pub = self.create_publisher(
            PointStamped, "/inspection/target", latched
        )
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.costmap = None
        self.create_subscription(
            OccupancyGrid, v("costmap_topic"), self.on_costmap, map_qos
        )
        self.create_subscription(PointStamped, v("input_topic"), self.on_target, 10)
        self.create_subscription(String, "/inspection/command", self.on_command, 10)
        self.create_subscription(String, "/d1_arm/status", self.on_arm_status, 10)

        self.state, self.note = "idle", ""
        self.target = None  # (x, y, z) in the map frame
        self.goal = None  # (x, y)
        self.distance = None
        self.since = 0.0  # when the current state began
        self.track = []  # [(t, x, y)] last robot positions
        self.arm, self.arm_seen = {}, 0.0
        self.scan_started = False
        self.create_timer(0.2, self.tick)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(
            f'inspection: {v("input_topic")} -> goal {self.standoff} m short, scan within '
            f'{self.trigger} m (or stopped within {self.max_scan} m)'
        )

    # ------------------------------------------------------------ inputs
    def on_costmap(self, msg):
        self.costmap = msg

    def on_arm_status(self, msg):
        try:
            self.arm, self.arm_seen = json.loads(msg.data), time.monotonic()
        except ValueError:
            pass

    def on_command(self, msg):
        try:
            op = json.loads(msg.data).get("op")
        except (ValueError, AttributeError):
            return
        if op == "cancel" and self.state in ACTIVE:
            if self.state == "scanning":
                self.arm_pub.publish(String(data=json.dumps({"op": "stop"})))
            self.cancel_nav()
            self.set_state("cancelled", "stopped from the dashboard")

    def robot_xy(self):
        t = self.tf_buffer.lookup_transform(
            self.map_frame, self.base_frame, Time()
        ).transform.translation
        return t.x, t.y

    def on_target(self, msg):
        if self.state == "scanning":
            self.get_logger().warn(
                "arm scan running: new target ignored (cancel it first)"
            )
            return
        msg.header.stamp = Time().to_msg()  # UI event: newest TF
        try:
            p = self.tf_buffer.transform(
                msg, self.map_frame, timeout=Duration(seconds=0.5)
            ).point
            rx, ry = self.robot_xy()
        except TransformException as e:
            self.get_logger().warn(f"cannot place the target in {self.map_frame}: {e}")
            return
        self.target = (p.x, p.y, p.z)
        latched = PointStamped()
        latched.header.frame_id = self.map_frame
        latched.header.stamp = self.get_clock().now().to_msg()
        latched.point = p
        self.target_pub.publish(latched)
        self.explore_pub.publish(
            Bool(data=False)
        )  # the explorer would override our goal

        # Stand-off on the robot's side of the target, then the nearest safe floor.
        dx, dy = p.x - rx, p.y - ry
        d = math.hypot(dx, dy)
        if d <= self.trigger:
            self.goal = (rx, ry)
            self.set_state("navigating", "already close: scanning from here")
            return
        sx, sy = p.x - dx / d * self.standoff, p.y - dy / d * self.standoff
        snapped = (
            self.snap_to_floor(self.costmap, sx, sy)
            if self.costmap is not None
            else (sx, sy)
        )
        if snapped is None:
            self.set_state(
                "unreachable",
                f"no safe floor within {self.max_snap} m of the stand-off point",
            )
            return
        gx, gy = snapped
        yaw = math.atan2(p.y - gy, p.x - gx)  # face the target from the goal
        goal = PoseStamped()
        goal.header.frame_id = (
            self.costmap.header.frame_id if self.costmap is not None else self.map_frame
        )
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = gx, gy
        goal.pose.orientation.z, goal.pose.orientation.w = (
            math.sin(yaw / 2),
            math.cos(yaw / 2),
        )
        self.goal_pub.publish(goal)
        self.goal = (gx, gy)
        self.track = []
        self.set_state("navigating", f"goal ({gx:.2f}, {gy:.2f}), facing the target")
        self.get_logger().info(
            f"target ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}): goal ({gx:.2f}, {gy:.2f}) "
            f"yaw {math.degrees(yaw):.0f} deg"
            + ("" if self.costmap is not None else " (no costmap: not snapped)")
        )

    # ------------------------------------------------------------ loop
    def set_state(self, state, note=""):
        self.state, self.note, self.since = state, note, time.monotonic()
        self.get_logger().info(f"{state}: {note}" if note else state)
        self.publish_status()

    def cancel_nav(self):
        # Hold position: a goal at the robot's own pose ends the current one.
        try:
            rx, ry = self.robot_xy()
        except TransformException:
            return
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = rx, ry
        if self.target:
            yaw = math.atan2(self.target[1] - ry, self.target[0] - rx)
            goal.pose.orientation.z, goal.pose.orientation.w = (
                math.sin(yaw / 2),
                math.cos(yaw / 2),
            )
        else:
            goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

    def stopped(self, now):
        """True if the robot moved < 3 cm over the last second."""
        recent = [(x, y) for t, x, y in self.track if now - t <= 1.0]
        if len(recent) < 4 or now - self.track[0][0] < 1.0:
            return False
        return (
            max(math.hypot(x - recent[-1][0], y - recent[-1][1]) for x, y in recent)
            < 0.03
        )

    def tick(self):
        now = time.monotonic()
        if self.state == "navigating":
            try:
                rx, ry = self.robot_xy()
            except TransformException:
                return
            self.track = [s for s in self.track if now - s[0] <= 2.0] + [(now, rx, ry)]
            tx, ty, _ = self.target
            self.distance = math.hypot(tx - rx, ty - ry)
            at_goal = (
                self.goal
                and math.hypot(self.goal[0] - rx, self.goal[1] - ry) <= self.arrive_tol
            )
            if self.distance <= self.trigger and self.stopped(now):
                self.start_scan()
            elif at_goal and self.stopped(now):
                if self.distance <= self.max_scan:
                    self.start_scan()
                else:
                    self.set_state(
                        "unreachable",
                        f"stopped {self.distance:.2f} m from the target, "
                        f"beyond the arm ({self.max_scan} m)",
                    )
            elif now - self.since > self.nav_timeout:
                self.cancel_nav()
                self.set_state("timeout", f"not there after {self.nav_timeout:.0f} s")
        elif self.state == "scanning":
            job, st = self.arm.get("job"), self.arm.get("orbit")
            if job == "scan" and st == "running":
                self.scan_started = True
            elif (
                self.scan_started
                and job == "scan"
                and st in ("done", "failed", "stopped")
            ):
                self.set_state("done" if st == "done" else "failed", f"arm scan {st}")
            elif now - self.since > (
                10.0 if not self.scan_started else self.scan_timeout
            ):
                self.set_state(
                    "failed",
                    "arm did not start the scan (arm_state_bridge running, arm online?)"
                    if not self.scan_started
                    else f"arm scan not finished after {self.scan_timeout:.0f} s",
                )

    def start_scan(self):
        p = PointStamped()
        p.header.frame_id = self.map_frame
        p.header.stamp = Time().to_msg()
        p.point.x, p.point.y, p.point.z = self.target
        try:
            b = self.tf_buffer.transform(
                p, self.base_frame, timeout=Duration(seconds=0.5)
            ).point
        except TransformException as e:
            self.set_state(
                "failed", f"cannot place the target in {self.base_frame}: {e}"
            )
            return
        if not self.arm or time.monotonic() - self.arm_seen > 5.0:
            self.set_state(
                "failed", "arrived, but the arm is not connected (arm_state_bridge.py)"
            )
            return
        if not self.arm.get("arm"):
            self.set_state("failed", "arrived, but the arm is offline")
            return
        self.scan_started = False
        self.arm_pub.publish(
            String(
                data=json.dumps(
                    {
                        "op": "scan_target",
                        "target": [round(b.x, 3), round(b.y, 3), round(b.z, 3)],
                    }
                )
            )
        )
        self.set_state(
            "scanning",
            f"arm aiming at ({b.x:+.2f}, {b.y:+.2f}, {b.z:+.2f}) m in {self.base_frame}",
        )

    def publish_status(self):
        self.status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "state": self.state,
                        "note": self.note,
                        "target": [round(v, 3) for v in self.target]
                        if self.target
                        else None,
                        "goal": [round(v, 3) for v in self.goal] if self.goal else None,
                        "distance": round(self.distance, 2)
                        if self.distance is not None
                        else None,
                    }
                )
            )
        )


def main():
    rclpy.init()
    node = InspectionMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
