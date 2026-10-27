#!/usr/bin/env python3
"""Publish a CameraInfo for every frame of a camera that has none.

The Go2 front camera (via go2_camera_relay.py) and the Gazebo camera publish
only sensor_msgs/Image. RTAB-Map and the LiDAR -> depth projection need
intrinsics, so this node republishes a pinhole CameraInfo with the same
header (stamp + frame) as each image.

Intrinsics come from fx/fy/cx/cy if set (> 0), else from hfov_deg and the
image size (square pixels, centred principal point, no distortion). The
defaults are an approximation, not a calibration: run camera_calibration and
pass the real values for accurate map colours and obstacle positions.

Subscribes: /camera/image_raw    (sensor_msgs/Image)
Publishes:  /camera/camera_info  (sensor_msgs/CameraInfo)
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('go2_camera_info')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('hfov_deg', 120.0)
        self.declare_parameter('fx', 0.0)
        self.declare_parameter('fy', 0.0)
        self.declare_parameter('cx', 0.0)
        self.declare_parameter('cy', 0.0)

        self.hfov = math.radians(float(self.get_parameter('hfov_deg').value))
        self.fixed = [float(self.get_parameter(k).value) for k in ('fx', 'fy', 'cx', 'cy')]
        image_topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        self.size = None
        self.template = None

        self.pub = self.create_publisher(CameraInfo, info_topic, qos_profile_sensor_data)
        self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        src = 'fx/fy/cx/cy' if self.fixed[0] > 0 else f'hfov {math.degrees(self.hfov):.1f} deg'
        self.get_logger().info(f'camera info: {image_topic} -> {info_topic} ({src})')

    def build(self, w, h):
        fx, fy, cx, cy = self.fixed
        if fx <= 0:
            fx = (w / 2.0) / math.tan(self.hfov / 2.0)
        fy = fy if fy > 0 else fx
        cx = cx if cx > 0 else (w - 1) / 2.0
        cy = cy if cy > 0 else (h - 1) / 2.0
        info = CameraInfo()
        info.width, info.height = w, h
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.get_logger().info(f'{w}x{h}: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}')
        return info

    def on_image(self, msg):
        if self.size != (msg.width, msg.height):
            self.size = (msg.width, msg.height)
            self.template = self.build(msg.width, msg.height)
        self.template.header = msg.header
        self.pub.publish(self.template)


def main():
    rclpy.init()
    node = CameraInfoPublisher()
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
