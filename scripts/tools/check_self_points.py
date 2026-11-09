#!/usr/bin/env python3
"""Pre-session check: does a LiDAR cloud contain the robot's own body?

  check_self_points.py [SECONDS] [TOPIC ...]
      default: 10 s of /lidar3d/registered and /lidar3d/obstacle_points

For each topic, transforms the points into base_link (TF from the cloud's frame)
and counts those inside the Go2's body box: x -0.45..0.36 m, y -0.20..0.20 m
(the Nav2 footprint) and z between 0.05 m above the floor and 0.10 m above
base_link (legs and trunk; floor_z = -base_height). A cloud the height band layer
or obstacle layer reads should have (almost) none: otherwise the costmap sees the
robot's own legs. Read-only: subscribes and prints, publishes nothing.
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

BOX_X, BOX_Y = (-0.45, 0.36), (-0.20, 0.20)
BASE_HEIGHT = 0.30   # base_link above the floor


def xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                   "offsets": [off["x"], off["y"], off["z"]], "itemsize": msg.point_step})
    a = np.frombuffer(bytes(msg.data), dtype=dt, count=msg.width * msg.height)
    p = np.stack([a["x"], a["y"], a["z"]], 1).astype(float)
    return p[np.isfinite(p).all(1)]


def matrix(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R, np.array([t.x, t.y, t.z])


def main():
    args = sys.argv[1:]
    seconds = float(args.pop(0)) if args and args[0].replace(".", "", 1).isdigit() else 10.0
    topics = args or ["/lidar3d/registered", "/lidar3d/obstacle_points"]
    rclpy.init()
    node = Node("check_self_points")
    tf = Buffer()
    TransformListener(tf, node)
    stats = {t: {"msgs": 0, "points": 0, "inside": 0, "no_tf": 0} for t in topics}

    def on_cloud(msg, topic):
        s = stats[topic]
        s["msgs"] += 1
        try:
            R, t = matrix(tf.lookup_transform("base_link", msg.header.frame_id, Time()))
        except TransformException:
            s["no_tf"] += 1
            return
        p = xyz(msg) @ R.T + t
        inside = ((p[:, 0] > BOX_X[0]) & (p[:, 0] < BOX_X[1]) & (p[:, 1] > BOX_Y[0]) & (p[:, 1] < BOX_Y[1])
                  & (p[:, 2] > -BASE_HEIGHT + 0.05) & (p[:, 2] < 0.10))
        s["points"] += len(p)
        s["inside"] += int(inside.sum())

    for topic in topics:
        node.create_subscription(PointCloud2, topic, lambda m, t=topic: on_cloud(m, t), qos_profile_sensor_data)
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    ok = True
    for topic, s in stats.items():
        if s["msgs"] == 0:
            print(f"{topic}: NO MESSAGES in {seconds:.0f} s")
            ok = False
            continue
        per = s["inside"] / max(1, s["msgs"] - s["no_tf"])
        verdict = "OK (no self-body points)" if per < 5 else "SELF-BODY POINTS PRESENT"
        print(f"{topic}: {s['msgs']} clouds, {s['inside']} points inside the body box "
              f"({per:.0f} per cloud), {s['no_tf']} without TF -> {verdict}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
