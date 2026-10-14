#!/usr/bin/env python3
"""
camera_relay.py

Relays the camera image from the WebRTC SDK's topic to the standard
/camera/image_raw topic, ensuring QoS compatibility.

Subscribes:
  /camera/image_raw (from WebRTC SDK, may have different QoS)

Publishes:
  /camera/image_raw_relayed (sensor_msgs/Image, BEST_EFFORT)
  /camera/camera_info_relayed (sensor_msgs/CameraInfo, BEST_EFFORT)
"""

import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
    QoSDurabilityPolicy)

from sensor_msgs.msg import Image, CameraInfo


def sensor_qos(depth: int = 2) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


class CameraRelay(Node):
    def __init__(self):
        super().__init__('camera_relay')

        self.declare_parameter('input_image_topic', '/camera/image_raw')
        self.declare_parameter('output_image_topic', '/camera/image_raw_relayed')
        self.declare_parameter('input_info_topic', '/camera/camera_info')
        self.declare_parameter('output_info_topic', '/camera/camera_info_relayed')

        in_img = self.get_parameter('input_image_topic').value
        out_img = self.get_parameter('output_image_topic').value
        in_info = self.get_parameter('input_info_topic').value
        out_info = self.get_parameter('output_info_topic').value

        qos = sensor_qos(2)

        self.img_pub = self.create_publisher(Image, out_img, qos)
        self.info_pub = self.create_publisher(CameraInfo, out_info, qos)

        self.img_sub = self.create_subscription(
            Image, in_img, self.on_image, qos)
        self.info_sub = self.create_subscription(
            CameraInfo, in_info, self.on_info, qos)

        self.get_logger().info(
            f'camera_relay: {in_img} -> {out_img}, {in_info} -> {out_info}')

    def on_image(self, msg: Image) -> None:
        self.img_pub.publish(msg)

    def on_info(self, msg: CameraInfo) -> None:
        self.info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()