#!/usr/bin/env python3
"""Log /semantic/obstacles to OUT/semantic.jsonl for analyze_semantic.py.

  record_semantic.py OUT_DIR
"""
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def main():
    from go2_semantic_msgs.msg import SemanticObstacleArray
    out = open(os.path.join(sys.argv[1], "semantic.jsonl"), "w", buffering=1)
    rclpy.init()
    node = Node("record_semantic")

    def on_msg(msg):
        obs = []
        for o in msg.obstacles:
            d = {"class": o.class_name, "confidence": round(float(o.confidence), 3),
                 "x": round(o.position.x, 3), "y": round(o.position.y, 3), "cost": int(o.cost),
                 "radius": round(float(o.radius), 3)}
            for f in ("label", "track_id"):
                if hasattr(o, f):
                    d[f] = getattr(o, f)
            obs.append(d)
        out.write(json.dumps({"t": time.time(), "obstacles": obs}) + "\n")

    node.create_subscription(SemanticObstacleArray, "/semantic/obstacles", on_msg, qos_profile_sensor_data)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
