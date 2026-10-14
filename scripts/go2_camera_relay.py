#!/usr/bin/env python3
"""
go2_camera_relay.py

Subscribe to /frontvideostream (unitree_go/Go2FrontVideoData) and
republish the decoded frames as sensor_msgs/Image on /camera/image_raw
so RViz, rqt_image_view, cv_bridge, YOLO, etc. can consume them.

Requires: cv_bridge, python3-opencv (install with apt inside container).

Run inside the go2-lidar3d container:
    python3 /tmp/go2_camera_relay.py --ros-args -p source_topic:=/frontvideostream
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from unitree_go.msg import Go2FrontVideoData


class Go2CameraRelay(Node):
    def __init__(self):
        super().__init__('go2_camera_relay')

        self.declare_parameter('source_topic', '/frontvideostream')
        self.declare_parameter('output_topic', '/camera/image_raw')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('resolution', '720p')  # 720p | 360p | 180p

        source = self.get_parameter('source_topic').value
        output = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.resolution = self.get_parameter('resolution').value

        # Low-latency QoS — drop stale frames rather than queue them.
        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, output, img_qos)
        self.sub = self.create_subscription(
            Go2FrontVideoData, source, self.on_video, 10)

        self.count = 0
        self.get_logger().info(
            f'go2_camera_relay: {source} -> {output} '
            f'frame_id={self.frame_id} res={self.resolution}')

    def on_video(self, msg: Go2FrontVideoData):
        # Pick the requested resolution buffer.
        if self.resolution == '720p':
            buf = bytes(msg.video720p)
        elif self.resolution == '360p':
            buf = bytes(msg.video360p)
        elif self.resolution == '180p':
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
                    'frame not JPEG-decodable — check firmware / codec',
                    throttle_duration_sec=5.0)
            return

        out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        self.pub.publish(out)

        self.count += 1
        if self.count % 100 == 0:
            self.get_logger().info(f'published {self.count} frames')


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


if __name__ == '__main__':
    main()