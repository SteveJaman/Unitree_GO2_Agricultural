#!/usr/bin/env python3
"""Map YOLO object classes to traversability costs.

Subscribes to /yolo/objects (JSON string with class, score, x, y, depth)
and publishes SemanticObstacleArray with per-class cost assignments.

Does NOT modify the YOLO node — reads its output only.

Parameters:
  class_costs_file: YAML mapping class names to {cost, radius}
  min_confidence:   ignore detections below this score (default 0.3)
  dry_run:          if true, publish but set all costs to 0
"""
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Point
import yaml

SENSOR_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
)

DEFAULT_CLASS_COSTS = {
    "person":       {"cost": 254, "radius": 0.5},
    "bicycle":      {"cost": 254, "radius": 0.4},
    "chair":        {"cost": 200, "radius": 0.3},
    "couch":        {"cost": 200, "radius": 0.5},
    "potted plant": {"cost": 40,  "radius": 0.2},
    "backpack":     {"cost": 40,  "radius": 0.2},
    "suitcase":     {"cost": 200, "radius": 0.3},
    "bench":        {"cost": 200, "radius": 0.5},
    "dog":          {"cost": 254, "radius": 0.4},
    "cat":          {"cost": 200, "radius": 0.3},
    "bed":          {"cost": 254, "radius": 0.6},
    "dining table": {"cost": 254, "radius": 0.5},
    "toilet":       {"cost": 254, "radius": 0.3},
    "tv":           {"cost": 200, "radius": 0.3},
    "laptop":       {"cost": 40,  "radius": 0.2},
    "refrigerator": {"cost": 254, "radius": 0.4},
    "power_cable":  {"cost": 200, "radius": 0.3},
    "debris":       {"cost": 40,  "radius": 0.3},
    "pipe":         {"cost": 200, "radius": 0.2},
    "hose":         {"cost": 40,  "radius": 0.2},
}


class VlmHeuristicNode(Node):
    def __init__(self):
        super().__init__("vlm_heuristic_node")

        self.declare_parameter("class_costs_file", "")
        self.declare_parameter("min_confidence", 0.3)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("map_frame", "lidar3d_map")

        costs_file = self.get_parameter("class_costs_file").value
        self.min_conf = self.get_parameter("min_confidence").value
        self.dry_run = self.get_parameter("dry_run").value
        self.map_frame = self.get_parameter("map_frame").value

        if costs_file:
            with open(costs_file) as f:
                self.class_costs = yaml.safe_load(f)
        else:
            self.class_costs = DEFAULT_CLASS_COSTS

        try:
            from go2_semantic_msgs.msg import SemanticObstacle, SemanticObstacleArray
            self.SemanticObstacle = SemanticObstacle
            self.SemanticObstacleArray = SemanticObstacleArray
            self.pub = self.create_publisher(
                SemanticObstacleArray, "/semantic/obstacles", 10)
            self.has_msgs = True
        except ImportError:
            self.get_logger().warn(
                "go2_semantic_msgs not built; publishing JSON on /semantic/obstacles_json")
            self.pub = self.create_publisher(String, "/semantic/obstacles_json", 10)
            self.has_msgs = False

        self.create_subscription(String, "/yolo/objects", self.on_yolo, SENSOR_QOS)

        mode = "DRY_RUN" if self.dry_run else "ACTIVE"
        self.get_logger().info(
            "vlm_heuristic_node [%s]: %d class mappings, min_conf=%.2f"
            % (mode, len(self.class_costs), self.min_conf))

    def on_yolo(self, msg):
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if not isinstance(detections, list):
            detections = [detections]

        obstacles = []
        for det in detections:
            cls = det.get("class", "")
            score = float(det.get("score", 0))
            if score < self.min_conf:
                continue
            if cls not in self.class_costs:
                continue

            entry = self.class_costs[cls]
            cost = 0 if self.dry_run else int(entry.get("cost", 0))
            radius = float(entry.get("radius", 0.3))

            x = float(det.get("x", 0))
            y = float(det.get("y", 0))
            z = float(det.get("depth", 0))

            obstacles.append({
                "class_name": cls,
                "confidence": score,
                "x": x, "y": y, "z": z,
                "cost": cost,
                "radius": radius,
            })

        if self.has_msgs:
            arr = self.SemanticObstacleArray()
            arr.header.frame_id = self.map_frame
            arr.header.stamp = self.get_clock().now().to_msg()
            for o in obstacles:
                sob = self.SemanticObstacle()
                sob.header = arr.header
                sob.class_name = o["class_name"]
                sob.confidence = o["confidence"]
                sob.position = Point(x=o["x"], y=o["y"], z=o["z"])
                sob.cost = o["cost"]
                sob.radius = o["radius"]
                arr.obstacles.append(sob)
            self.pub.publish(arr)
        else:
            self.pub.publish(String(data=json.dumps(obstacles)))


def main(args=None):
    rclpy.init(args=args)
    node = VlmHeuristicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
