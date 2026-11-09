#!/usr/bin/env python3
"""Log a std_msgs/String JSON topic to a .jsonl file, one {"t", "data"} per message.

  record_json_topic.py TOPIC OUT_FILE
"""
import json
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def main():
    topic, path = sys.argv[1:3]
    out = open(path, "w", buffering=1)
    rclpy.init()
    node = Node("record_json_topic")
    node.create_subscription(String, topic,
                             lambda m: out.write(json.dumps({"t": time.time(), "data": json.loads(m.data)}) + "\n"), 10)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
