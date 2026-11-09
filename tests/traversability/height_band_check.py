#!/usr/bin/env python3
"""HeightBandLayer acceptance: synthetic objects of known height -> raw costmap cost.

  height_band_check.py CONTROLLER_YAML OUT_JSON [--layers L1,L2] [--floor Z]
                       [--floor-estimation fixed|histogram]

Builds a standalone costmap (nav2_costmap_2d) from the local_costmap section of
CONTROLLER_YAML, every LiDAR source pointed at /hbtest/points, and publishes a
static cloud of four 0.10 x 0.10 m columns, 1.5 m ahead of base_link, with
heights 0.10, 0.14, 0.20 and 0.30 m. Reads /costmap/costmap_raw (node /costmap/costmap) and
reports the highest raw cost inside each column. `layers` (comma separated)
keeps only those plugins, e.g. height_band_layer for the layer on its own.
--floor Z adds a floor of points at height Z (4 x 4 m around the robot) and
stands the columns on it, so heights are measured from Z; --floor-estimation
overrides the height band layer's floor_estimation. With a floor at Z != 0,
fixed (ground_z 0) misclassifies and histogram should not.

Run on its own ROS_DOMAIN_ID; it publishes a static lidar3d_map -> base_link TF.
"""
import argparse
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


def params_file(controller_yaml, layers, floor_estimation=None):
    cfg = yaml.safe_load(open(controller_yaml))["local_costmap"]["local_costmap"]["ros__parameters"]
    if floor_estimation and "height_band_layer" in cfg:
        cfg["height_band_layer"]["floor_estimation"] = floor_estimation
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


def cloud(stamp, floor=None):
    pts = []
    base = floor or 0.0
    if floor is not None:
        for i in range(81):
            for j in range(81):
                pts.append((-2.0 + 0.05 * i + 1.5, -2.0 + 0.05 * j, base))
    for y0, h in zip(YS, HEIGHTS):
        n = 5
        for i in range(n):
            for j in range(n):
                x = X - SIDE / 2 + SIDE * i / (n - 1)
                y = y0 - SIDE / 2 + SIDE * j / (n - 1)
                z = 0.03
                while z < h:
                    pts.append((x, y, base + z))
                    z += 0.02
                pts.append((x, y, base + h))
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
    ap = argparse.ArgumentParser()
    ap.add_argument("controller_yaml")
    ap.add_argument("out")
    ap.add_argument("layers_pos", nargs="?", help="same as --layers (older form)")
    ap.add_argument("--layers")
    ap.add_argument("--floor", type=float, default=None)
    ap.add_argument("--floor-estimation", choices=("fixed", "histogram"))
    a = ap.parse_args()
    controller_yaml, out = a.controller_yaml, a.out
    lay = a.layers or a.layers_pos
    layers = lay.split(",") if lay else None
    pf, plugins = params_file(controller_yaml, layers, a.floor_estimation)
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
            pub.publish(cloud(node.get_clock().now().to_msg(), a.floor))
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
        result = {"controller_yaml": os.path.basename(controller_yaml), "plugins": plugins,
                  "floor": a.floor, "floor_estimation": a.floor_estimation or "config", "objects": rows}
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
