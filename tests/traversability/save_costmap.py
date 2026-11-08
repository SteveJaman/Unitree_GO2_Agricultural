#!/usr/bin/env python3
"""Save local costmap snapshots (raw costs 0-254) with the semantic obstacles.

  save_costmap.py OUT_DIR [PERIOD_S]

Every PERIOD_S (default 5) writes OUT_DIR/costmap/NNNN.npz: the raw local costmap
(/local_costmap/costmap_raw, nav2_msgs/Costmap: 40 step over, 200 caution,
253/254 lethal, not the 0-100 OccupancyGrid scale), its origin, resolution and
wall time, the latest /semantic/obstacles as JSON (empty without the
semantic layer), and the global costmap (/global_costmap/costmap_raw) as
global_grid, global_origin, global_resolution.
"""
import json
import os
import sys
import time

import numpy as np
import rclpy
from nav2_msgs.msg import Costmap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def main():
    out = os.path.join(sys.argv[1], "costmap")
    os.makedirs(out, exist_ok=True)
    period = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
    rclpy.init()
    node = Node("save_costmap")
    state = {"grid": None, "global": None, "sem": [], "n": 0}

    def on_grid(msg):
        state["grid"] = msg

    def on_sem(msg):
        state["sem"] = [{"label": o.label, "class": o.class_name, "x": o.position.x, "y": o.position.y,
                         "radius": o.radius, "size_x": o.size_x, "size_y": o.size_y, "yaw": o.yaw,
                         "confidence": o.confidence} for o in msg.obstacles]

    def raw(g):
        m = g.metadata
        return (np.frombuffer(bytes(g.data), np.uint8).reshape(m.size_y, m.size_x),
                [m.origin.position.x, m.origin.position.y], m.resolution)

    def save():
        if state["grid"] is None:
            return
        grid, origin, res = raw(state["grid"])
        extra = {}
        if state["global"] is not None:
            gg, go, gr = raw(state["global"])
            extra = {"global_grid": gg, "global_origin": go, "global_resolution": gr}
        np.savez_compressed(os.path.join(out, f"{state['n']:04d}.npz"), grid=grid, origin=origin,
                            resolution=res, t=time.time(), semantic=json.dumps(state["sem"]), **extra)
        state["n"] += 1

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Costmap, "/local_costmap/costmap_raw", on_grid, qos)
    node.create_subscription(Costmap, "/global_costmap/costmap_raw", lambda m: state.update({"global": m}), qos)
    try:
        from go2_semantic_msgs.msg import SemanticObstacleArray
        node.create_subscription(SemanticObstacleArray, "/semantic/obstacles", on_sem, 10)
    except ImportError:
        pass
    node.create_timer(period, save)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
