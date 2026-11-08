#!/usr/bin/env python3
"""Log the semantic labeller's output to OUT/semantic.jsonl for analyze_semantic.py.

  record_semantic.py OUT_DIR [TOPIC]

Records /semantic/labels_json (vlm_heuristic_node.py), which is published in
dry_run too and carries every track: final label, source, the raw VLM answer,
its confidence and latency. Each line: {"t", "obstacles": [...]}, the shape
analyze_semantic.py reads.
"""
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    topic = sys.argv[2] if len(sys.argv) > 2 else "/semantic/labels_json"
    out = open(os.path.join(sys.argv[1], "semantic.jsonl"), "w", buffering=1)
    rclpy.init()
    node = Node("record_semantic")

    def on_msg(msg):
        d = json.loads(msg.data)
        out.write(json.dumps({"t": time.time(), "asked": d.get("asked"), "failed": d.get("failed"),
                              "obstacles": d.get("objects", [])}) + "\n")

    node.create_subscription(String, topic, on_msg, 10)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
