#!/usr/bin/env python3
"""Save camera frames with the camera pose and current boxes, for the report's figures.

Runs in go2-lidar3d, which has no OpenCV: raw arrays go to OUT/NNNN.npz and
make_figures.py draws them.
"""
import json
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class Saver(Node):
    def __init__(self, out):
        super().__init__("frame_saver")
        self.out, self.n, self.image, self.boxes, self.dets = out, 0, None, "[]", "[]"
        os.makedirs(out, exist_ok=True)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(Image, "/camera/image_raw", self.on_image, 2)
        self.create_subscription(String, "/objects/boxes_json", self.on_boxes, 5)
        self.create_subscription(String, "/objects/detections_json", self.on_dets, 5)
        self.create_timer(1.0, self.save)

    def on_image(self, msg):
        self.image = msg

    def on_boxes(self, msg):
        self.boxes = json.dumps(json.loads(msg.data)["boxes"])

    def on_dets(self, msg):
        self.dets = json.dumps(json.loads(msg.data)["detections"])

    def save(self):
        if self.image is None:
            return
        try:
            tf = self.tf.lookup_transform("front_camera", "lidar3d_map", Time())
        except TransformException:
            return
        m = self.image
        img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)
        t, q = tf.transform.translation, tf.transform.rotation
        np.savez_compressed(os.path.join(self.out, f"{self.n:04d}.npz"), image=img, encoding=m.encoding,
                            trans=[t.x, t.y, t.z], quat=[q.x, q.y, q.z, q.w],
                            boxes=self.boxes, dets=self.dets)
        self.n += 1


def main():
    rclpy.init()
    rclpy.spin(Saver(sys.argv[1]))


if __name__ == "__main__":
    main()
