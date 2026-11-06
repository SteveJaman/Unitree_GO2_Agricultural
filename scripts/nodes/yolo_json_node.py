#!/usr/bin/env python3
"""Real YOLO on the camera, published in the JSON form object_box_fusion.py reads.

A thin wrapper for the sim's go2-3d container: go2-lidar3d has no vision_msgs,
so twin/yolo_node.py's /yolo/detections cannot reach the fusion there. It does
not change the YOLO node.

Publishes /objects/detections_json: {"detections": [{class, score, bbox}]}.
With dry_run, detections are logged and nothing is published.
"""
import json

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


class YoloJson(Node):
    def __init__(self):
        super().__init__("yolo_json_node")
        p = self.declare_parameter
        p("model", "/opt/yolo/yolo11s.pt")
        p("rate_hz", 2.0)
        p("confidence", 0.4)
        p("threads", 2)
        p("dry_run", False)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        import torch
        from ultralytics import YOLO
        torch.set_num_threads(int(gp("threads")))
        self.model = YOLO(gp("model"))
        self.conf, self.dry_run = float(gp("confidence")), bool(gp("dry_run"))
        self.image = None
        self.pub = self.create_publisher(String, "/objects/detections_json", 10)
        self.create_subscription(Image, "/camera/image_raw", self.on_image, QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1))
        self.create_timer(1.0 / float(gp("rate_hz")), self.tick)

    def on_image(self, msg):
        self.image = msg

    def tick(self):
        m, self.image = self.image, None
        if m is None:
            return
        img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)
        if m.encoding == "rgb8":
            img = img[:, :, ::-1]
        r = self.model.predict(img, conf=self.conf, imgsz=640, verbose=False)[0]
        dets = [{"class": self.model.names[int(c)], "score": round(float(s), 3),
                 "bbox": [round(float(v), 1) for v in b], "source": "yolo"}
                for c, s, b in zip(r.boxes.cls, r.boxes.conf, r.boxes.xyxy)]
        if self.dry_run:
            self.get_logger().info(f"dry_run detections: {len(dets)}", throttle_duration_sec=5.0)
        else:
            self.pub.publish(String(data=json.dumps({"detections": dets})))


def main():
    rclpy.init()
    rclpy.spin(YoloJson())


if __name__ == "__main__":
    main()
