#!/usr/bin/env python3
"""Gait supervisor: adapts quadruped gait based on upcoming terrain.

Monitors the local costmap ahead of the robot and commands gait changes
through the Go2's sport client interface (Phase 6) or logs intended
commands in dry_run mode.

Gait modes:
  NORMAL:    default trot gait for free terrain
  HIGH_STEP: raised foot clearance for step_over obstacles (h < 0.15m)
  CAREFUL:   slow deliberate gait for caution terrain
  STOP:      halt for lethal obstacles (planner should reroute)

When 3D object boxes are available (object_box_fusion.py) the gait comes from
the tallest object in the corridor ahead, and HIGH_STEP is chosen only if the
raised foot clears it. The costmap decision stays in /gait/status for comparison.

Subscribes:
  /local_costmap/costmap   nav_msgs/OccupancyGrid
  /lidar3d/body_odom       nav_msgs/Odometry
  /objects/boxes_json      std_msgs/String

Publishes:
  /gait/command            std_msgs/String (gait mode name)
  /gait/status             std_msgs/String (detailed status, JSON)

Parameters:
  scan_distance:    how far ahead to scan (default 1.5m)
  scan_width:       lateral scan half-width (default 0.3m)
  check_rate_hz:    evaluation frequency (default 5.0)
  foot_margin:      clearance the raised foot must keep above an object (default 0.02m)
  lethal_height:    object height that means STOP (default 0.25m)
  dry_run:          if true, log commands but don't send to sport_client
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
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

STEP_OVER_MIN = 10
CAUTION_MIN = 60
LETHAL_MIN = 100

GAITS = {
    "NORMAL":    {"foot_height": 0.04, "speed_scale": 1.0},
    "HIGH_STEP": {"foot_height": 0.12, "speed_scale": 0.7},
    "CAREFUL":   {"foot_height": 0.12, "speed_scale": 0.4},
    "STOP":      {"foot_height": 0.04, "speed_scale": 0.0},
}


def gait_for_height(height, foot_height, margin, lethal_height):
    if height is None:      # seen, but not measured: never treat it as low
        return "CAREFUL"
    if height + margin <= foot_height:
        return "HIGH_STEP"
    return "CAREFUL" if height < lethal_height else "STOP"


def objects_ahead(boxes, x, y, yaw, scan_distance, scan_width):
    """Boxes whose footprint overlaps the corridor ahead of the body."""
    c, s = math.cos(yaw), math.sin(yaw)
    hits = []
    for b in boxes:
        dx, dy = b["x"] - x, b["y"] - y
        forward = c * dx + s * dy
        lateral = -s * dx + c * dy
        reach = max(b["sx"], b["sy"]) / 2
        if -0.45 - reach < forward < scan_distance + reach and abs(lateral) < scan_width + reach:
            hits.append(b)
    return hits


class GaitSupervisor(Node):
    def __init__(self):
        super().__init__("gait_supervisor")

        self.declare_parameter("scan_distance", 1.5)
        self.declare_parameter("scan_width", 0.3)
        self.declare_parameter("check_rate_hz", 5.0)
        self.declare_parameter("foot_margin", 0.02)
        self.declare_parameter("lethal_height", 0.25)
        self.declare_parameter("dry_run", True)

        self.foot_margin = self.get_parameter("foot_margin").value
        self.lethal_height = self.get_parameter("lethal_height").value
        self.boxes = []
        self.scan_dist = self.get_parameter("scan_distance").value
        self.scan_width = self.get_parameter("scan_width").value
        self.dry_run = self.get_parameter("dry_run").value
        rate = self.get_parameter("check_rate_hz").value

        self.costmap = None
        self.odom = None
        self.current_gait = "NORMAL"

        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self._cm_cb, COSTMAP_QOS)
        self.create_subscription(
            Odometry, "/lidar3d/body_odom", self._odom_cb, SENSOR_QOS)
        self.create_subscription(String, "/objects/boxes_json", self._boxes_cb, 10)

        self.cmd_pub = self.create_publisher(String, "/gait/command", 10)
        self.status_pub = self.create_publisher(String, "/gait/status", 10)
        self.create_timer(1.0 / rate, self._evaluate)

        mode = "DRY_RUN" if self.dry_run else "ACTIVE"
        self.get_logger().info(
            "gait_supervisor [%s]: scan=%.1fm width=%.1fm"
            % (mode, self.scan_dist, self.scan_width))

    def _cm_cb(self, msg):
        self.costmap = msg

    def _odom_cb(self, msg):
        self.odom = msg

    def _boxes_cb(self, msg):
        self.boxes = json.loads(msg.data)["boxes"]

    def _get_yaw(self):
        if self.odom is None:
            return 0.0
        q = self.odom.pose.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _scan_ahead(self):
        if self.costmap is None or self.odom is None:
            return 0

        info = self.costmap.info
        rx = self.odom.pose.pose.position.x
        ry = self.odom.pose.pose.position.y
        yaw = self._get_yaw()

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        max_cost = 0
        res = info.resolution
        n_forward = int(self.scan_dist / res)
        n_lateral = int(self.scan_width / res)

        for fd in range(1, n_forward + 1):
            for ld in range(-n_lateral, n_lateral + 1):
                wx = rx + fd * res * cos_y - ld * res * sin_y
                wy = ry + fd * res * sin_y + ld * res * cos_y

                mx = int((wx - info.origin.position.x) / res)
                my = int((wy - info.origin.position.y) / res)

                if 0 <= mx < info.width and 0 <= my < info.height:
                    c = self.costmap.data[my * info.width + mx]
                    if c > max_cost:
                        max_cost = c

        return max_cost

    def _evaluate(self):
        max_cost = self._scan_ahead()

        if max_cost >= LETHAL_MIN:
            costmap_gait = "STOP"
        elif max_cost >= CAUTION_MIN:
            costmap_gait = "CAREFUL"
        elif max_cost >= STEP_OVER_MIN:
            costmap_gait = "HIGH_STEP"
        else:
            costmap_gait = "NORMAL"

        desired = costmap_gait
        hits = []
        if self.boxes and self.odom is not None:
            p = self.odom.pose.pose.position
            hits = objects_ahead(self.boxes, p.x, p.y, self._get_yaw(),
                                 self.scan_dist, self.scan_width)
            desired = "NORMAL"
            if hits:
                gaits = [gait_for_height(b["height"] if b["traverse"] != "unknown" else None,
                                         GAITS["HIGH_STEP"]["foot_height"],
                                         self.foot_margin, self.lethal_height) for b in hits]
                desired = max(gaits, key=("NORMAL", "HIGH_STEP", "CAREFUL", "STOP").index)

        if desired != self.current_gait:
            self.current_gait = desired
            self.get_logger().info(
                "gait -> %s (max_cost=%d dry_run=%s)"
                % (desired, max_cost, self.dry_run))

            if not self.dry_run:
                self._send_gait_command(desired)

        self.cmd_pub.publish(String(data=desired))
        self.status_pub.publish(String(data=json.dumps({
            "gait": desired, "costmap_gait": costmap_gait, "cost": max_cost,
            "objects": [b["class"] for b in hits],
            "foot": GAITS[desired]["foot_height"],
            "speed_pct": GAITS[desired]["speed_scale"] * 100})))

    def _send_gait_command(self, gait_name):
        # Phase 6: sport_client integration
        # For now, log the intended command
        params = GAITS[gait_name]
        self.get_logger().info(
            "SPORT_CLIENT (not implemented): foot_height=%.2f speed_scale=%.1f"
            % (params["foot_height"], params["speed_scale"]))


def main(args=None):
    rclpy.init(args=args)
    node = GaitSupervisor()
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
