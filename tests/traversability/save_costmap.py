#!/usr/bin/env python3
"""Save local costmap snapshots with the semantic obstacles, for figures.

  save_costmap.py OUT_DIR [PERIOD_S]

Every PERIOD_S (default 5) writes OUT_DIR/costmap/NNNN.npz: the costmap grid,
its origin and resolution, and the latest /semantic/obstacles as JSON.
"""
import json
import os
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def main():
    from go2_semantic_msgs.msg import SemanticObstacleArray
    out = os.path.join(sys.argv[1], "costmap")
    os.makedirs(out, exist_ok=True)
    period = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
    rclpy.init()
    node = Node("save_costmap")
    state = {"grid": None, "sem": [], "n": 0}

    def on_grid(msg):
        state["grid"] = msg

    def on_sem(msg):
        state["sem"] = [{"label": o.label, "class": o.class_name, "x": o.position.x, "y": o.position.y,
                         "radius": o.radius, "size_x": o.size_x, "size_y": o.size_y, "yaw": o.yaw,
                         "confidence": o.confidence} for o in msg.obstacles]

    def save():
        g = state["grid"]
        if g is None:
            return
        data = np.array(g.data, np.int8).reshape(g.info.height, g.info.width)
        np.savez_compressed(os.path.join(out, f"{state['n']:04d}.npz"), grid=data,
                            origin=[g.info.origin.position.x, g.info.origin.position.y],
                            resolution=g.info.resolution, semantic=json.dumps(state["sem"]))
        state["n"] += 1

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, "/local_costmap/costmap", on_grid, qos)
    node.create_subscription(SemanticObstacleArray, "/semantic/obstacles", on_sem, 10)
    node.create_timer(period, save)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
