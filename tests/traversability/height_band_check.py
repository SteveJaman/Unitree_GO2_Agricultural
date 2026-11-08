#!/usr/bin/env python3
"""HeightBandLayer acceptance: synthetic objects of known height -> raw costmap cost.

  height_band_check.py CONTROLLER_YAML OUT_JSON [layers]

Builds a standalone costmap (nav2_costmap_2d) from the local_costmap section of
CONTROLLER_YAML, every LiDAR source pointed at /hbtest/points, and publishes a
static cloud of four 0.10 x 0.10 m columns, 1.5 m ahead of base_link, with
heights 0.10, 0.14, 0.20 and 0.30 m. Reads /costmap/costmap_raw (node /costmap/costmap) and
reports the highest raw cost inside each column. `layers` (comma separated)
keeps only those plugins, e.g. height_band_layer for the layer on its own.

Run on its own ROS_DOMAIN_ID; it publishes a static lidar3d_map -> base_link TF.
"""
import json
import os
import signal
import struct
import subprocess
import sys
import tempfile
import time

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from nav2_msgs.msg import Costmap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

HEIGHTS = [0.10, 0.14, 0.20, 0.30]
X, YS, SIDE = 1.5, [-0.9, -0.3, 0.3, 0.9], 0.10
TOPIC = "/hbtest/points"


def params_file(controller_yaml, layers):
    cfg = yaml.safe_load(open(controller_yaml))["local_costmap"]["local_costmap"]["ros__parameters"]
    if layers:
        cfg["plugins"] = [p for p in cfg["plugins"] if p in layers]
    for name in cfg["plugins"]:
        layer = cfg[name]
        if "topic" in layer:
            layer["topic"] = TOPIC
        for src in str(layer.get("observation_sources", "")).split():
            layer[src]["topic"] = TOPIC
            layer[src]["sensor_frame"] = "base_link"
            layer[src]["expected_update_rate"] = 0.0
    cfg["update_frequency"] = 5.0
    f = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
    yaml.safe_dump({"costmap": {"costmap": {"ros__parameters": cfg}}}, f)
    f.close()
    return f.name, cfg["plugins"]


def cloud(stamp):
    pts = []
    for y0, h in zip(YS, HEIGHTS):
        n = 5
        for i in range(n):
            for j in range(n):
                x = X - SIDE / 2 + SIDE * i / (n - 1)
                y = y0 - SIDE / 2 + SIDE * j / (n - 1)
                z = 0.03
                while z < h:
                    pts.append((x, y, z))
                    z += 0.02
                pts.append((x, y, h))
    msg = PointCloud2()
    msg.header.frame_id = "lidar3d_map"
    msg.header.stamp = stamp
    msg.height, msg.width = 1, len(pts)
    msg.fields = [PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32, count=1)
                  for k, n in enumerate("xyz")]
    msg.point_step, msg.row_step = 12, 12 * len(pts)
    msg.is_dense = True
    msg.data = b"".join(struct.pack("<fff", *p) for p in pts)
    return msg


def main():
    controller_yaml, out = sys.argv[1:3]
    layers = sys.argv[3].split(",") if len(sys.argv) > 3 else None
    pf, plugins = params_file(controller_yaml, layers)
    proc = subprocess.Popen(["ros2", "run", "nav2_costmap_2d", "nav2_costmap_2d", "--ros-args",
                             "--params-file", pf],
                            stdout=open(out + ".log", "w"), stderr=subprocess.STDOUT,
                            start_new_session=True)   # own process group: ros2 run forks the node
    rclpy.init()
    node = Node("hbtest_pub")
    tfb = StaticTransformBroadcaster(node)
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = "lidar3d_map", "base_link"
    t.header.stamp = node.get_clock().now().to_msg()
    t.transform.rotation.w = 1.0
    tfb.sendTransform(t)
    pub = node.create_publisher(PointCloud2, TOPIC, qos_profile_sensor_data)
    got = {}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Costmap, "/costmap/costmap_raw", lambda m: got.update(m=m), qos)
    try:
        time.sleep(3)
        for state in ("configure", "activate"):
            subprocess.run(["ros2", "lifecycle", "set", "/costmap/costmap", state], check=False,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        end = time.time() + 20
        while time.time() < end:
            pub.publish(cloud(node.get_clock().now().to_msg()))
            rclpy.spin_once(node, timeout_sec=0.2)
        m = got.get("m")
        if m is None:
            raise SystemExit("no /costmap/costmap_raw received; see " + out + ".log")
        res, w = m.metadata.resolution, m.metadata.size_x
        ox, oy = m.metadata.origin.position.x, m.metadata.origin.position.y
        rows = []
        for y0, h in zip(YS, HEIGHTS):
            best = 0
            for i in range(int((X - SIDE / 2 - ox) / res), int((X + SIDE / 2 - ox) / res) + 1):
                for j in range(int((y0 - SIDE / 2 - oy) / res), int((y0 + SIDE / 2 - oy) / res) + 1):
                    best = max(best, m.data[j * w + i])
            rows.append({"height_m": h, "max_raw_cost": best})
        result = {"controller_yaml": os.path.basename(controller_yaml), "plugins": plugins, "objects": rows}
        json.dump(result, open(out, "w"), indent=1)
        print(json.dumps(result))
    finally:
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        time.sleep(1)
        os.unlink(pf)


if __name__ == "__main__":
    main()
