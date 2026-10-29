#!/usr/bin/env python3
"""Periscope search: look over a table edge with the D1 arm's wrist camera.

A table taller than the Go2 hides its top from the base LiDAR and the front
camera. The dashboard's Periscope Scan sends a click on the table edge to
/periscope_point; this node then:

  1. Finds the edge: the nearest costmap obstacle within EDGE_RADIUS of the
     click (a click that is not near an obstacle is refused).
  2. Sends a Nav2 goal STANDOFF (0.6 m) from the edge, back towards the dog,
     snapped to safe floor like go2_click_to_goal.py, facing the edge.
  3. On arrival asks the arm for the periscope pose ({"op": "periscope"} on
     /d1_arm/command: arm_state_bridge.py -> start_arm.sh periscope ->
     arm_periscope.py), and waits until /d1_arm/joint_states shows the arm in
     it.
  4. Waits MAP_SECONDS (3 s) for the wrist camera to map the table top, then
     reports "Table mapped. Please select the target object." The arm keeps
     holding: an Inspect click (inspection_mission.py) takes over from the hold.

{"op": "here"} on /table_search/command skips the walk (periscope from where
the dog stands); {"op": "cancel"} stops the walk and lowers the arm.

The wrist camera is not mounted yet: everything up to the hold works without
it, and the 3 s "mapping" wait is then only a wait.

It listens on its own /periscope_point, not /clicked_point: go2_click_to_goal
already turns /clicked_point into a goal, and one click must not send two.
Its message goes to /table_search/status, not /vlm_decision: a VLM decision
of BLOCKED makes the dog dock.

Subscribes: /periscope_point (PointStamped), /table_search/command (String JSON),
            /global_costmap/costmap, /d1_arm/status, /d1_arm/joint_states
Publishes:  /goal_pose, /explore/resume (false), /d1_arm/command,
            /table_search/status (String JSON, 2 Hz:
              {"state": "idle|navigating|raising|mapping|mapped|failed|cancelled",
               "message": "...", "edge": [x, y], "goal": [x, y]})
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
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped with tf2)
from tf2_ros import Buffer, TransformException, TransformListener

from arm_periscope import PERISCOPE_POSE
from go2_click_to_goal import ClickToGoal

MAPPED = "Table mapped. Please select the target object."


class TableSearch(Node):
    # Safe-floor snapping, shared with the click-to-goal node.
    cell_free = ClickToGoal.cell_free
    cell_safe = ClickToGoal.cell_safe
    snap_to_floor = ClickToGoal.snap_to_floor

    def __init__(self):
        super().__init__("table_search")
        p = self.declare_parameter
        p("input_topic", "/periscope_point")
        p("costmap_topic", "/global_costmap/costmap")
        p("map_frame", "lidar3d_map")
        p("base_frame", "base_link")
        p("standoff", 0.6)  # m from the table edge
        p("edge_radius", 0.3)  # m: an obstacle this close to the click is the edge
        p("obstacle_cost", 90)  # costmap value counted as an obstacle
        p("arrive_tolerance", 0.3)  # m from the goal
        p("map_seconds", 3.0)  # s in the periscope pose before "mapped"
        p("pose_tolerance_deg", 6.0)  # arm counted as in the periscope pose
        p("nav_timeout", 180.0)
        p("raise_timeout", 40.0)
        p("free_threshold", 50)
        p("clearance_cells", 1)
        p("max_snap_distance", 1.0)
        v = lambda n: self.get_parameter(n).value  # noqa: E731
        self.map_frame, self.base_frame = v("map_frame"), v("base_frame")
        self.standoff, self.edge_radius = float(v("standoff")), float(v("edge_radius"))
        self.obstacle_cost = int(v("obstacle_cost"))
        self.arrive_tol, self.map_seconds = (
            float(v("arrive_tolerance")),
            float(v("map_seconds")),
        )
        self.pose_tol = math.radians(float(v("pose_tolerance_deg")))
        self.nav_timeout, self.raise_timeout = (
            float(v("nav_timeout")),
            float(v("raise_timeout")),
        )
        self.free_threshold = int(v("free_threshold"))
        self.clearance = max(0, int(v("clearance_cells")))
        self.max_snap = float(v("max_snap_distance"))
        self.pose_rad = [math.radians(a) for a in PERISCOPE_POSE]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.explore_pub = self.create_publisher(Bool, "/explore/resume", 10)
        self.arm_pub = self.create_publisher(String, "/d1_arm/command", 10)
        self.status_pub = self.create_publisher(String, "/table_search/status", 10)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.costmap = None
        self.create_subscription(
            OccupancyGrid,
            v("costmap_topic"),
            lambda m: setattr(self, "costmap", m),
            map_qos,
        )
        self.create_subscription(PointStamped, v("input_topic"), self.on_point, 10)
        self.create_subscription(String, "/table_search/command", self.on_command, 10)
        self.create_subscription(String, "/d1_arm/status", self.on_arm_status, 10)
        self.create_subscription(JointState, "/d1_arm/joint_states", self.on_joints, 10)

        self.state, self.message = "idle", ""
        self.edge = self.goal = None
        self.since = 0.0
        self.track = []
        self.arm, self.arm_seen = {}, 0.0
        self.joints = {}
        self.create_timer(0.2, self.tick)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(
            f'table search: {v("input_topic")} -> {self.standoff} m from the edge -> '
            f'periscope pose {PERISCOPE_POSE} -> {self.map_seconds:.0f} s -> mapped'
        )

    # ------------------------------------------------------------ inputs
    def on_arm_status(self, msg):
        try:
            self.arm, self.arm_seen = json.loads(msg.data), time.monotonic()
        except ValueError:
            pass

    def on_joints(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    def on_command(self, msg):
        try:
            op = json.loads(msg.data).get("op")
        except (ValueError, AttributeError):
            return
        if op == "cancel" and self.state not in ("idle", "cancelled"):
            if self.state == "navigating":
                self.hold_here()
            if (
                self.arm.get("job") == "periscope"
                and self.arm.get("orbit") == "running"
            ):
                self.arm_pub.publish(String(data=json.dumps({"op": "stop"})))
            self.set_state("cancelled", "Periscope search cancelled")
        elif op == "here" and self.state not in ("navigating", "raising", "mapping"):
            self.explore_pub.publish(Bool(data=False))
            self.edge = self.goal = None
            self.raise_arm()

    def robot_xy(self):
        t = self.tf_buffer.lookup_transform(
            self.map_frame, self.base_frame, Time()
        ).transform.translation
        return t.x, t.y

    def find_edge(self, x, y):
        """Nearest obstacle cell centre within edge_radius of (x, y), or None."""
        grid = self.costmap
        info = grid.info
        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        r = int(math.ceil(self.edge_radius / res))
        cx, cy = int(math.floor((x - ox) / res)), int(math.floor((y - oy) / res))
        best, best_d = None, float("inf")
        for gy in range(max(0, cy - r), min(h, cy + r + 1)):
            for gx in range(max(0, cx - r), min(w, cx + r + 1)):
                if grid.data[gy * w + gx] >= self.obstacle_cost:
                    ex, ey = ox + (gx + 0.5) * res, oy + (gy + 0.5) * res
                    d = math.hypot(ex - x, ey - y)
                    if d <= self.edge_radius and d < best_d:
                        best, best_d = (ex, ey), d
        return best

    def on_point(self, msg):
        if self.state in ("navigating", "raising", "mapping"):
            self.get_logger().warn(
                "periscope search running: new point ignored (cancel it first)"
            )
            return
        msg.header.stamp = Time().to_msg()
        try:
            p = self.tf_buffer.transform(
                msg, self.map_frame, timeout=Duration(seconds=0.5)
            ).point
            rx, ry = self.robot_xy()
        except TransformException as e:
            self.get_logger().warn(f"cannot place the point in {self.map_frame}: {e}")
            return
        if self.costmap is not None:
            edge = self.find_edge(p.x, p.y)
            if edge is None:
                self.set_state(
                    "failed",
                    f"No table edge there: no obstacle within {self.edge_radius} m of the "
                    "click. Click right on the edge of the table.",
                )
                return
        else:
            edge = (p.x, p.y)  # no costmap (mock): take the click as the edge
        self.edge = edge
        self.explore_pub.publish(Bool(data=False))
        dx, dy = rx - edge[0], ry - edge[1]
        d = math.hypot(dx, dy) or 1.0
        sx, sy = edge[0] + dx / d * self.standoff, edge[1] + dy / d * self.standoff
        snapped = (
            self.snap_to_floor(self.costmap, sx, sy)
            if self.costmap is not None
            else (sx, sy)
        )
        if snapped is None:
            self.set_state(
                "failed", f"No safe floor near {self.standoff} m from the edge"
            )
            return
        gx, gy = snapped
        yaw = math.atan2(edge[1] - gy, edge[0] - gx)  # face the table
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
        self.goal, self.track = (gx, gy), []
        self.set_state(
            "navigating", f"Walking to {self.standoff} m from the table edge"
        )

    # ------------------------------------------------------------ loop
    def set_state(self, state, message=""):
        self.state, self.message, self.since = state, message, time.monotonic()
        self.get_logger().info(f"{state}: {message}" if message else state)
        self.publish_status()

    def hold_here(self):
        try:
            rx, ry = self.robot_xy()
        except TransformException:
            return
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = rx, ry
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

    def raise_arm(self):
        if (
            not self.arm
            or time.monotonic() - self.arm_seen > 5.0
            or not self.arm.get("arm")
        ):
            self.set_state(
                "failed",
                "At the table, but the arm is not connected (arm_state_bridge.py)",
            )
            return
        self.arm_pub.publish(String(data=json.dumps({"op": "periscope", "hold": 0})))
        self.set_state("raising", "Raising the arm into the periscope pose")

    def in_pose(self):
        names = [f"Joint{i}" for i in range(1, 7)]
        if not all(n in self.joints for n in names):
            return False
        return all(
            abs(self.joints[n] - t) <= self.pose_tol
            for n, t in zip(names, self.pose_rad)
        )

    def tick(self):
        now = time.monotonic()
        if self.state == "navigating":
            try:
                rx, ry = self.robot_xy()
            except TransformException:
                return
            self.track = [s for s in self.track if now - s[0] <= 2.0] + [(now, rx, ry)]
            recent = [(x, y) for t, x, y in self.track if now - t <= 1.0]
            stopped = (
                now - self.track[0][0] >= 1.0
                and len(recent) >= 4
                and max(math.hypot(x - rx, y - ry) for x, y in recent) < 0.03
            )
            if (
                math.hypot(self.goal[0] - rx, self.goal[1] - ry) <= self.arrive_tol
                and stopped
            ):
                self.raise_arm()
            elif now - self.since > self.nav_timeout:
                self.hold_here()
                self.set_state(
                    "failed", f"Did not reach the table after {self.nav_timeout:.0f} s"
                )
        elif self.state == "raising":
            if self.in_pose():
                self.set_state(
                    "mapping",
                    f"Wrist camera over the table: mapping for {self.map_seconds:.0f} s",
                )
            elif self.arm.get("job") == "periscope" and self.arm.get("orbit") in (
                "failed",
                "stopped",
            ):
                self.set_state("failed", f'Arm periscope {self.arm.get("orbit")}')
            elif now - self.since > self.raise_timeout:
                self.set_state(
                    "failed",
                    f"Arm not in the periscope pose after {self.raise_timeout:.0f} s",
                )
        elif self.state == "mapping" and now - self.since >= self.map_seconds:
            self.set_state("mapped", MAPPED)

    def publish_status(self):
        self.status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "state": self.state,
                        "message": self.message,
                        "edge": [round(c, 3) for c in self.edge] if self.edge else None,
                        "goal": [round(c, 3) for c in self.goal] if self.goal else None,
                    }
                )
            )
        )


def main():
    rclpy.init()
    node = TableSearch()
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
