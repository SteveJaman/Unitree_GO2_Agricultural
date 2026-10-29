#!/usr/bin/env python3
"""
D435i receiver: runs on the ROS2 Humble host (native Ubuntu 22.04).

Subscribes to the ZeroMQ stream from d435i_sender.py and republishes:
    /camera/color/image_raw                     (sensor_msgs/Image, bgr8)
    /camera/color/camera_info                   (sensor_msgs/CameraInfo)
    /camera/aligned_depth_to_color/image_raw    (sensor_msgs/Image, 16UC1, millimetres)
    /camera/aligned_depth_to_color/camera_info  (sensor_msgs/CameraInfo)

Install (on the host):
    pip install pyzmq opencv-python numpy

Run:
    source /opt/ros/humble/setup.bash
    python3 d435i_receiver.py --ros-args -p endpoint:=tcp://<sender-ip>:5555
"""

import json

import cv2
import numpy as np
import rclpy
import zmq
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class D435iReceiver(Node):
    def __init__(self):
        super().__init__("d435i_receiver")
        self.declare_parameter("endpoint", "tcp://127.0.0.1:5555")
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        # True: use the sender's capture time (needs clocks synced between machines).
        # False: stamp with this node's clock on arrival (simple, but adds network latency).
        self.declare_parameter("use_sender_stamp", True)

        endpoint = self.get_parameter("endpoint").value
        self.frame_id = self.get_parameter("frame_id").value
        self.use_sender_stamp = self.get_parameter("use_sender_stamp").value

        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.SUB)
        self.sock.setsockopt(zmq.RCVHWM, 2)
        self.sock.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sock.connect(endpoint)
        self.get_logger().info(f"Subscribed to {endpoint}")

        qos = qos_profile_sensor_data
        self.pub_color = self.create_publisher(Image, "camera/color/image_raw", qos)
        self.pub_color_info = self.create_publisher(
            CameraInfo, "camera/color/camera_info", qos
        )
        self.pub_depth = self.create_publisher(
            Image, "camera/aligned_depth_to_color/image_raw", qos
        )
        self.pub_depth_info = self.create_publisher(
            CameraInfo, "camera/aligned_depth_to_color/camera_info", qos
        )

        self.create_timer(0.005, self.poll)

    # -- helpers -------------------------------------------------------------
    def _stamp(self, t):
        if not self.use_sender_stamp:
            return self.get_clock().now().to_msg()
        msg = self.get_clock().now().to_msg()
        msg.sec = int(t)
        msg.nanosec = int((t - int(t)) * 1e9)
        return msg

    def _camera_info(self, hdr, stamp):
        i = hdr["intrinsics"]
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width, info.height = i["width"], i["height"]
        info.distortion_model = "plumb_bob"
        info.d = [float(c) for c in i["coeffs"]]
        info.k = [i["fx"], 0.0, i["cx"], 0.0, i["fy"], i["cy"], 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            i["fx"],
            0.0,
            i["cx"],
            0.0,
            0.0,
            i["fy"],
            i["cy"],
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        return info

    def _image(self, arr, encoding, bytes_per_px, stamp):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = arr.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = msg.width * bytes_per_px
        msg.data = arr.tobytes()
        return msg

    # -- main loop -----------------------------------------------------------
    def poll(self):
        latest = None
        while True:  # drain the socket, keep only the newest frame
            try:
                latest = self.sock.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
        if latest is None or len(latest) != 3:
            return

        hdr = json.loads(latest[0])
        color = cv2.imdecode(np.frombuffer(latest[1], np.uint8), cv2.IMREAD_COLOR)
        depth = cv2.imdecode(np.frombuffer(latest[2], np.uint8), cv2.IMREAD_UNCHANGED)
        if color is None or depth is None:
            self.get_logger().warn("Failed to decode a frame")
            return

        # ROS convention for 16UC1 depth is millimetres.
        scale_to_mm = hdr["depth_scale"] / 0.001
        if abs(scale_to_mm - 1.0) > 1e-6:
            depth = (
                (depth.astype(np.float32) * scale_to_mm)
                .clip(0, 65535)
                .astype(np.uint16)
            )

        stamp = self._stamp(hdr["stamp"])
        info = self._camera_info(hdr, stamp)

        self.pub_color.publish(self._image(color, "bgr8", 3, stamp))
        self.pub_color_info.publish(info)
        self.pub_depth.publish(self._image(depth, "16UC1", 2, stamp))
        self.pub_depth_info.publish(info)


def main():
    rclpy.init()
    node = D435iReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close(0)
        node.ctx.term()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
