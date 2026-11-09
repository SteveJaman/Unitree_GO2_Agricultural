#!/usr/bin/env python3
"""Clusterer acceptance: a synthetic floor with low objects -> /objects/low_clusters.

  clusterer_check.py CLUSTERER_PY [HEIGHTS]     e.g. 0.06,0.024

Publishes a static lidar3d_map -> base_link TF (base 0.30 m above the floor) and
a floor cloud with one 0.30 x 0.20 m box per height, 1.2 m ahead and spaced in y,
starts the clusterer (dry_run false) in its own process group, and prints what it
publishes. Run on its own ROS_DOMAIN_ID.
"""
import json
import os
import signal
import struct
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

TOPIC = "/clustertest/points"


def scene(heights, rng):
    xy = rng.uniform([-1.0, -2.0], [4.0, 2.0], (6000, 2))
    pts = [np.column_stack([xy, rng.normal(0.0, 0.003, len(xy))])]
    for k, h in enumerate(heights):
        cx, cy = 1.2, -0.6 + 0.6 * k
        xs, ys = np.arange(cx - 0.15, cx + 0.15, 0.02), np.arange(cy - 0.1, cy + 0.1, 0.02)
        pts.append(np.array([[x, y, h] for x in xs for y in ys]))
    return np.vstack(pts)


def cloud(pts, stamp):
    m = PointCloud2()
    m.header.frame_id, m.header.stamp = "lidar3d_map", stamp
    m.height, m.width = 1, len(pts)
    m.fields = [PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32, count=1) for k, n in enumerate("xyz")]
    m.point_step, m.row_step, m.is_dense = 12, 12 * len(pts), True
    m.data = b"".join(struct.pack("<fff", *p) for p in pts)
    return m


def main():
    script = sys.argv[1]
    heights = [float(h) for h in (sys.argv[2] if len(sys.argv) > 2 else "0.06").split(",")]
    proc = subprocess.Popen(["python3", script, "--ros-args", "-p", f"cloud_topic:={TOPIC}",
                             "-p", "dry_run:=false"], start_new_session=True,
                            stdout=open("/tmp/clusterer_check.log", "w"), stderr=subprocess.STDOUT)
    rclpy.init()
    node = Node("clusterer_check")
    tfb = StaticTransformBroadcaster(node)
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = "lidar3d_map", "base_link"
    t.header.stamp = node.get_clock().now().to_msg()
    t.transform.translation.z, t.transform.rotation.w = 0.30, 1.0
    tfb.sendTransform(t)
    pub = node.create_publisher(PointCloud2, TOPIC, qos_profile_sensor_data)
    got = {}
    node.create_subscription(String, "/objects/low_clusters_json", lambda m: got.update(j=m.data), 10)
    node.create_subscription(PointCloud2, "/objects/low_clusters", lambda m: got.update(n=m.width), 10)
    rng = np.random.default_rng(1)
    try:
        end = time.time() + 12
        while time.time() < end:
            pub.publish(cloud(scene(heights, rng), node.get_clock().now().to_msg()))
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.02)
        if "j" not in got:
            raise SystemExit("no /objects/low_clusters_json; see /tmp/clusterer_check.log")
        d = json.loads(got["j"])
        print(json.dumps({"input_heights": heights, "cloud_points_published": got.get("n"),
                          "floor_z": d["floor_z"],
                          "clusters": [{k: c[k] for k in ("x", "y", "height", "n_pts", "above_step_limit")}
                                       for c in d["clusters"]]}))
    finally:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=10)


if __name__ == "__main__":
    main()
