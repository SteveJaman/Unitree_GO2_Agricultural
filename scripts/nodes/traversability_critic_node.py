#!/usr/bin/env python3
"""Traversability critic: penalizes navigation through high-cost terrain.

Monitors the robot's planned path against the local costmap and publishes
speed limits when the path crosses step-over or caution zones. Works with
any Nav2 controller (RPP, DWB, MPPI).

This replaces a dedicated MPPI critic plugin, providing controller-agnostic
traversability-aware speed modulation.

Subscribes:
  /plan                  nav_msgs/Path (global plan)
  /local_costmap/costmap nav_msgs/OccupancyGrid

Publishes:
  /speed_limit           nav2_msgs/SpeedLimit
  /traversability/status std_msgs/String

Parameters:
  step_over_speed:  speed limit (%) when path crosses step_over cells (default 70)
  caution_speed:    speed limit (%) when path crosses caution cells (default 40)
  lethal_speed:     speed limit (%) when path crosses lethal cells (default 0 = stop)
  lookahead_m:      how far ahead on the path to check (default 2.0)
  check_rate_hz:    how often to evaluate (default 4.0)
  dry_run:          if true, compute but don't publish speed limits
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from nav_msgs.msg import OccupancyGrid, Path
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

STEP_OVER_RANGE = (10, 60)
CAUTION_RANGE = (60, 99)
LETHAL_VALUE = 100


class TraversabilityCritic(Node):
    def __init__(self):
        super().__init__("traversability_critic")

        self.declare_parameter("step_over_speed", 70.0)
        self.declare_parameter("caution_speed", 40.0)
        self.declare_parameter("lethal_speed", 0.0)
        self.declare_parameter("lookahead_m", 2.0)
        self.declare_parameter("check_rate_hz", 4.0)
        self.declare_parameter("dry_run", False)

        self.step_over_speed = self.get_parameter("step_over_speed").value
        self.caution_speed = self.get_parameter("caution_speed").value
        self.lethal_speed = self.get_parameter("lethal_speed").value
        self.lookahead = self.get_parameter("lookahead_m").value
        self.dry_run = self.get_parameter("dry_run").value
        rate = self.get_parameter("check_rate_hz").value

        self.costmap = None
        self.plan = None
        self.current_limit = 0.0

        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self._cm_cb, COSTMAP_QOS)
        self.create_subscription(
            Path, "/plan", self._plan_cb, 10)

        try:
            from nav2_msgs.msg import SpeedLimit
            self.SpeedLimit = SpeedLimit
            self.speed_pub = self.create_publisher(SpeedLimit, "/speed_limit", 10)
            self.has_speed_msg = True
        except ImportError:
            self.get_logger().warn("nav2_msgs not available; speed limits disabled")
            self.has_speed_msg = False
            self.speed_pub = None

        self.status_pub = self.create_publisher(
            String, "/traversability/status", 10)
        self.create_timer(1.0 / rate, self._evaluate)

        mode = "DRY_RUN" if self.dry_run else "ACTIVE"
        self.get_logger().info(
            "traversability_critic [%s]: lookahead=%.1fm "
            "speeds: step_over=%.0f%% caution=%.0f%% lethal=%.0f%%"
            % (mode, self.lookahead, self.step_over_speed,
               self.caution_speed, self.lethal_speed))

    def _cm_cb(self, msg):
        self.costmap = msg

    def _plan_cb(self, msg):
        self.plan = msg

    def _world_to_grid(self, cm, wx, wy):
        info = cm.info
        mx = int((wx - info.origin.position.x) / info.resolution)
        my = int((wy - info.origin.position.y) / info.resolution)
        if 0 <= mx < info.width and 0 <= my < info.height:
            return mx, my
        return None, None

    def _evaluate(self):
        if self.costmap is None or self.plan is None:
            return
        if len(self.plan.poses) < 2:
            return

        max_cost = 0
        dist = 0.0
        poses = self.plan.poses

        for i in range(1, len(poses)):
            if dist > self.lookahead:
                break
            p0 = poses[i - 1].pose.position
            p1 = poses[i].pose.position
            dx = p1.x - p0.x
            dy = p1.y - p0.y
            dist += math.sqrt(dx * dx + dy * dy)

            gx, gy = self._world_to_grid(self.costmap, p1.x, p1.y)
            if gx is None:
                continue
            c = self.costmap.data[gy * self.costmap.info.width + gx]
            if c > max_cost:
                max_cost = c

        if max_cost >= LETHAL_VALUE:
            speed = self.lethal_speed
            terrain = "LETHAL"
        elif max_cost >= CAUTION_RANGE[0]:
            speed = self.caution_speed
            terrain = "CAUTION"
        elif max_cost >= STEP_OVER_RANGE[0]:
            speed = self.step_over_speed
            terrain = "STEP_OVER"
        else:
            speed = 0.0
            terrain = "FREE"

        if speed != self.current_limit:
            self.current_limit = speed
            self.get_logger().info(
                "terrain=%s max_cost=%d -> speed_limit=%.0f%%"
                % (terrain, max_cost, speed))

        if not self.dry_run and self.has_speed_msg and self.speed_pub:
            msg = self.SpeedLimit()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.percentage = True
            msg.speed_limit = speed
            self.speed_pub.publish(msg)

        self.status_pub.publish(String(
            data="terrain=%s cost=%d speed=%.0f%%" % (terrain, max_cost, speed)))


def main(args=None):
    rclpy.init(args=args)
    node = TraversabilityCritic()
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
