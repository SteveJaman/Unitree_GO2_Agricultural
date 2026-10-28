#!/usr/bin/env python3
"""
go2_camera_relay.py

Decode the Go2 front camera to sensor_msgs/Image (bgr8) on /camera/image_raw
so RViz, rqt_image_view, YOLO, etc. can consume it. Two sources:
  source_type compressed (default): sensor_msgs/CompressedImage JPEG on
      /go2/front_camera/compressed, published over Wi-Fi by
      scripts/robot/go2_camera_wifi.sh on the robot; the robot's stamp is kept.
  source_type unitree: unitree_go/Go2FrontVideoData on /frontvideostream.

With camera_info_file set (a calibration from scripts/tools/calibrate_camera.py),
it also publishes sensor_msgs/CameraInfo on /camera/camera_info with the same
header as each image, which the LiDAR overlay and RTAB-Map need.

Requires: python3-opencv (no cv_bridge; the Image is filled directly).

Run inside the go2-lidar3d container:
    python3 /tmp/go2_camera_relay.py --ros-args -p camera_info_file:=/tmp/front_camera.yaml
"""

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np

from sensor_msgs.msg import CameraInfo, CompressedImage, Image


def load_camera_info(path):
    """CameraInfo from a ROS camera calibration YAML (camera_calibration format)."""
    import yaml

    with open(path) as f:
        cal = yaml.safe_load(f)
    info = CameraInfo()
    info.width = int(cal["image_width"])
    info.height = int(cal["image_height"])
    info.distortion_model = cal.get("distortion_model", "plumb_bob")
    info.k = [float(v) for v in cal["camera_matrix"]["data"]]
    info.d = [float(v) for v in cal["distortion_coefficients"]["data"]]
    info.r = [
        float(v)
        for v in cal.get("rectification_matrix", {}).get(
            "data", [1, 0, 0, 0, 1, 0, 0, 0, 1]
        )
    ]
    p = cal.get("projection_matrix", {}).get("data")
    k = info.k
    info.p = (
        [float(v) for v in p]
        if p
        else [k[0], 0.0, k[2], 0.0, 0.0, k[4], k[5], 0.0, 0.0, 0.0, 1.0, 0.0]
    )
    return info


class Go2CameraRelay(Node):
    def __init__(self):
        super().__init__("go2_camera_relay")

        self.declare_parameter("source_type", "compressed")  # compressed | unitree
        self.declare_parameter("source_topic", "/go2/front_camera/compressed")
        self.declare_parameter("output_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("camera_info_file", "")
        # front_camera is the camera link in the Go2 URDF (robot_state_publisher).
        self.declare_parameter("frame_id", "front_camera")
        self.declare_parameter("resolution", "720p")  # 720p | 360p | 180p

        source = self.get_parameter("source_topic").value
        output = self.get_parameter("output_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        info_file = self.get_parameter("camera_info_file").value
        self.frame_id = self.get_parameter("frame_id").value
        self.resolution = self.get_parameter("resolution").value

        # Input over Wi-Fi: best effort, latest frame only. Output on this
        # machine: reliable, because a best-effort reader drops a whole ~1.5 MB
        # raw frame whenever one of its fragments is lost.
        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        out_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub = self.create_publisher(Image, output, out_qos)
        self.info = None
        self.info_pub = None
        if info_file:
            if os.path.isfile(info_file):
                self.info = load_camera_info(info_file)
                self.info_pub = self.create_publisher(CameraInfo, info_topic, out_qos)
            else:
                self.get_logger().warn(
                    f"camera_info_file {info_file} not found; no {info_topic}"
                )
        if self.get_parameter("source_type").value == "unitree":
            from unitree_go.msg import Go2FrontVideoData

            self.sub = self.create_subscription(
                Go2FrontVideoData, source, self.on_video, 10
            )
        else:
            self.sub = self.create_subscription(
                CompressedImage, source, self.on_compressed, img_qos
            )

        self.count = 0
        self.get_logger().info(
            f'go2_camera_relay: {source} -> {output} '
            f'frame_id={self.frame_id} res={self.resolution} '
            f'camera_info={info_topic if self.info else "off"}'
        )

    def on_compressed(self, msg):
        frame = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn(
                "compressed frame not decodable", throttle_duration_sec=5.0
            )
            return
        self.publish_frame(frame, msg.header.stamp)

    def on_video(self, msg):
        # Pick the requested resolution buffer.
        if self.resolution == "720p":
            buf = bytes(msg.video720p)
        elif self.resolution == "360p":
            buf = bytes(msg.video360p)
        elif self.resolution == "180p":
            buf = bytes(msg.video180p)
        else:
            buf = bytes(msg.video720p)

        if not buf:
            return

        # The payload is raw Annex-B H.264 or a JPEG depending on firmware.
        # Try JPEG first (cheap), fall back to OpenCV's H.264 decoder.
        arr = np.frombuffer(buf, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

        if frame is None:
            # Firmware that ships H.264: skip if we have no decoder.
            if self.count % 30 == 0:
                self.get_logger().warn(
                    "frame not JPEG-decodable — check firmware / codec",
                    throttle_duration_sec=5.0,
                )
            return

        self.publish_frame(frame, self.get_clock().now().to_msg())

    def publish_frame(self, frame, stamp):
        out = Image()
        out.header.stamp = stamp
        out.header.frame_id = self.frame_id
        out.height, out.width = frame.shape[:2]
        out.encoding = "bgr8"
        out.step = out.width * 3
        out.data = np.ascontiguousarray(frame).tobytes()
        self.pub.publish(out)

        if self.info is not None:
            if (self.info.width, self.info.height) != (out.width, out.height):
                self.get_logger().warn(
                    f"calibration is {self.info.width}x{self.info.height} but frames are "
                    f"{out.width}x{out.height}: recalibrate at this resolution",
                    throttle_duration_sec=10.0,
                )
            self.info.header = out.header
            self.info_pub.publish(self.info)

        self.count += 1
        if self.count % 100 == 0:
            self.get_logger().info(f"published {self.count} frames")


def main(args=None):
    rclpy.init(args=args)
    node = Go2CameraRelay()
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
