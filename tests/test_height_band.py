#!/usr/bin/env python3
"""Phase 1 acceptance test for HeightBandLayer.

Publishes a synthetic PointCloud2 with known heights on the topic the
HeightBandLayer is subscribed to, then checks the global_costmap for
the expected costs.

The costmap is published as OccupancyGrid (0-100 scale).
Nav2 OccupancyGrid encoding:
  raw  0  (FREE)       -> occ 0
  raw  1..252          -> occ 1..98 (linear)
  raw 253 (INSCRIBED)  -> occ 99
  raw 254 (LETHAL)     -> occ 100
  raw 255 (UNKNOWN)    -> occ -1

Usage (in a shell with the sim + Nav2 running):
    python3 test_height_band.py [--enabled|--disabled]
"""
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2, PointField

# OccupancyGrid range for each raw cost
# raw 40 -> occ ~16, raw 200 -> occ ~78, raw 254 -> occ 100
BANDS = {
    "STEP_OVER": (10, 60),     # raw ~40
    "CAUTION":   (60, 100),    # raw ~200, may be inflated to 99-100
    "LETHAL":    (99, 100),    # raw 254, inflation may spread inscribed(99)
}

# Points to inject (relative to robot in lidar3d_map)
# Spaced 2m apart so inflation from higher bands does not contaminate lower ones
INJECT_POINTS = [
    {"name": "step_over_obj", "dx": 1.5, "dy":  0.0, "height": 0.10, "band": "STEP_OVER"},
    {"name": "caution_obj",   "dx": 1.5, "dy":  2.0, "height": 0.20, "band": "CAUTION"},
    {"name": "lethal_obj",    "dx": 1.5, "dy": -2.0, "height": 0.30, "band": "LETHAL"},
]

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

# Use the existing obstacle_points topic so DDS discovery is already established
CLOUD_TOPIC = "/lidar3d/obstacle_points"


def make_cloud(frame_id, points):
    """Build a PointCloud2 from a list of (x, y, z) tuples."""
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = 0
    msg.header.stamp.nanosec = 0
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    buf = bytearray()
    for x, y, z in points:
        buf.extend(struct.pack("<fff", x, y, z))
    msg.data = bytes(buf)
    return msg


class HeightBandTester(Node):
    def __init__(self, cloud_topic):
        super().__init__("height_band_tester")
        self.costmap = None
        self.odom = None
        self.cloud_pub = self.create_publisher(PointCloud2, cloud_topic, SENSOR_QOS)
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.on_costmap, COSTMAP_QOS)
        self.create_subscription(
            Odometry, "/lidar3d/odom", self.on_odom, SENSOR_QOS)
        self.get_logger().info("Publishing synthetic cloud on %s" % cloud_topic)

    def on_costmap(self, msg):
        self.costmap = msg

    def on_odom(self, msg):
        self.odom = msg

    def world_to_grid(self, wx, wy):
        if self.costmap is None:
            return None, None
        info = self.costmap.info
        mx = int((wx - info.origin.position.x) / info.resolution)
        my = int((wy - info.origin.position.y) / info.resolution)
        if 0 <= mx < info.width and 0 <= my < info.height:
            return mx, my
        return None, None

    def get_cost_at(self, wx, wy, radius=5):
        if self.costmap is None:
            return -1
        info = self.costmap.info
        cx, cy = self.world_to_grid(wx, wy)
        if cx is None:
            return -1
        max_cost = 0
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                ix, iy = cx + dx, cy + dy
                if 0 <= ix < info.width and 0 <= iy < info.height:
                    idx = iy * info.width + ix
                    c = self.costmap.data[idx]
                    if c > max_cost:
                        max_cost = c
        return max_cost


def main():
    enabled = "--disabled" not in sys.argv
    mode = "ENABLED" if enabled else "DISABLED"

    rclpy.init()
    node = HeightBandTester(CLOUD_TOPIC)

    # Wait for odom so we know the robot position in lidar3d_map
    node.get_logger().info("Waiting for odom...")
    deadline = time.time() + 30.0
    while node.odom is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=1.0)

    if node.odom is None:
        node.get_logger().error("No odom received!")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    rx = node.odom.pose.pose.position.x
    ry = node.odom.pose.pose.position.y
    node.get_logger().info("Robot at lidar3d_map (%.2f, %.2f)" % (rx, ry))

    # Build world-frame points at known positions relative to robot
    world_points = []
    inject_world = []
    for obj in INJECT_POINTS:
        wx = rx + obj["dx"]
        wy = ry + obj["dy"]
        wz = obj["height"]  # z in lidar3d_map (ground_z=0)
        world_points.append((wx, wy, wz))
        inject_world.append({"name": obj["name"], "wx": wx, "wy": wy,
                             "height": obj["height"], "band": obj["band"]})

    cloud = make_cloud("lidar3d_map", world_points)

    # Publish the synthetic cloud repeatedly
    node.get_logger().info("Publishing synthetic cloud (%d pts)..." % len(world_points))
    t0 = time.time()
    while time.time() - t0 < 15.0:
        cloud.header.stamp = node.get_clock().now().to_msg()
        node.cloud_pub.publish(cloud)
        rclpy.spin_once(node, timeout_sec=0.5)

    if node.costmap is None:
        node.get_logger().error("No costmap received!")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    info = node.costmap.info
    print("\n" + "=" * 70)
    print("HeightBandLayer acceptance test -- mode: %s" % mode)
    print("=" * 70)
    print("Costmap: %dx%d @ %.3fm, origin=(%.1f, %.1f)" % (
        info.width, info.height, info.resolution,
        info.origin.position.x, info.origin.position.y))
    print("Robot: (%.2f, %.2f) in lidar3d_map" % (rx, ry))
    print("Cloud topic: %s  frame: lidar3d_map" % CLOUD_TOPIC)
    print()

    all_pass = True
    for obj in inject_world:
        cost = node.get_cost_at(obj["wx"], obj["wy"])
        gx, gy = node.world_to_grid(obj["wx"], obj["wy"])

        if enabled:
            omin, omax = BANDS[obj["band"]]
            ok = omin <= cost <= omax
            status = "PASS" if ok else "FAIL"
        else:
            status = "INFO"
            ok = True

        if not ok:
            all_pass = False

        omin, omax = BANDS[obj["band"]]
        print("  [%s] %-18s  world=(%.2f, %.2f)  grid=(%s, %s)  "
              "h=%.2f  band=%-10s  occ_range=[%d,%d]  actual=%d" % (
                  status, obj["name"], obj["wx"], obj["wy"],
                  gx, gy, obj["height"], obj["band"],
                  omin, omax, cost))

    print()
    if enabled:
        if all_pass:
            print("RESULT: ALL CHECKS PASSED")
        else:
            print("RESULT: SOME CHECKS FAILED")
    else:
        print("RESULT: BASELINE RECORDED (no assertions in disabled mode)")
    print("=" * 70 + "\n")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
