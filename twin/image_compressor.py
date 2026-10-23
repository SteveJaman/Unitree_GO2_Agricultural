#!/usr/bin/env python3
"""JPEG copies of camera images for the web dashboard (twin/webapp).

A raw 720p bgr8 frame is ~2.7 MB, far too much to push through rosbridge as
JSON. For each "input:output" pair this node republishes the newest frame as
sensor_msgs/CompressedImage (JPEG, downscaled to max_width) at most rate_hz
times a second, and only while something subscribes to the output.

Inputs are subscribed BEST_EFFORT (the camera relay and YOLO publish that
way); outputs are RELIABLE so rosbridge always matches them.

Default pairs:
  /camera/image_raw -> /twin/camera/compressed
  /yolo/image       -> /twin/yolo/compressed
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


def image_bgr(msg):
    """sensor_msgs/Image (bgr8 / rgb8 / mono8) -> HxWx3 uint8 BGR, or None."""
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if msg.encoding in ('bgr8', 'rgb8'):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1].copy() if msg.encoding == 'rgb8' else img
    if msg.encoding == 'mono8':
        img = buf.reshape(msg.height, msg.step)[:, :msg.width]
        return np.repeat(img[:, :, None], 3, axis=2)
    return None


class ImageCompressor(Node):
    def __init__(self):
        super().__init__('twin_image_compressor')
        self.declare_parameter('pairs', ['/camera/image_raw:/twin/camera/compressed',
                                         '/yolo/image:/twin/yolo/compressed'])
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('max_width', 640)
        self.declare_parameter('quality', 70)
        self.max_width = int(self.get_parameter('max_width').value)
        self.quality = int(self.get_parameter('quality').value)
        self.streams = []
        for pair in self.get_parameter('pairs').value:
            src, _, dst = pair.partition(':')
            s = {'src': src, 'pub': self.create_publisher(CompressedImage, dst, 2), 'msg': None}
            self.create_subscription(Image, src, lambda m, s=s: s.__setitem__('msg', m),
                                     qos_profile_sensor_data)
            self.streams.append(s)
            self.get_logger().info(f'{src} -> {dst} (JPEG)')
        self.create_timer(1.0 / max(0.1, float(self.get_parameter('rate_hz').value)), self.tick)

    def tick(self):
        for s in self.streams:
            msg, s['msg'] = s['msg'], None
            if msg is None or s['pub'].get_subscription_count() == 0:
                continue
            img = image_bgr(msg)
            if img is None:
                self.get_logger().warn(f'{s["src"]}: unsupported encoding {msg.encoding!r}',
                                       throttle_duration_sec=60.0)
                continue
            if img.shape[1] > self.max_width:
                h = int(img.shape[0] * self.max_width / img.shape[1])
                img = cv2.resize(img, (self.max_width, h), interpolation=cv2.INTER_AREA)
            ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if not ok:
                continue
            out = CompressedImage()
            out.header = msg.header
            out.format = 'jpeg'
            out.data = jpg.tobytes()
            s['pub'].publish(out)


def main():
    rclpy.init()
    node = ImageCompressor()
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
