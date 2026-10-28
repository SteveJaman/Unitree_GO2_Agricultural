#!/usr/bin/env python3
"""
go2_camera_wifi.py: ON THE ROBOT. Go2 front camera -> CompressedImage over Wi-Fi.

Reads the camera's own JPEG stream from go2_camera_jpeg_cli (4-byte big-endian
length + JPEG, 1920x1080 at ~16 fps) and publishes sensor_msgs/CompressedImage
on the Wi-Fi DDS domain, so colour frames reach the laptop at a small fraction
of raw bandwidth. Frames are stamped with the Jetson clock, the same clock
Point-LIO stamps the LiDAR with, so camera and LiDAR line up for fusion.

Started by go2_camera_wifi.sh (DDS setup); the laptop's go2_camera_relay.py
decodes it to /camera/image_raw.

Env: GO2_CAM_TOPIC=/go2/front_camera/compressed  GO2_CAM_FPS=5
     GO2_CAM_WIDTH=960 (0 = native, sent without re-encoding)  GO2_CAM_QUALITY=85
     GO2_CAMERA_JPEG_CLI=~/bin/go2_camera_jpeg_cli  UNITREE_SDK2=~/unitree_sdk2
"""

import os
import struct
import subprocess
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

TOPIC = os.environ.get("GO2_CAM_TOPIC", "/go2/front_camera/compressed")
FPS = float(os.environ.get("GO2_CAM_FPS", "5"))
WIDTH = int(os.environ.get("GO2_CAM_WIDTH", "960"))
QUALITY = int(os.environ.get("GO2_CAM_QUALITY", "85"))
FRAME_ID = os.environ.get("GO2_CAM_FRAME_ID", "front_camera")
CLI_CANDIDATES = [
    os.environ.get("GO2_CAMERA_JPEG_CLI", ""),
    "~/bin/go2_camera_jpeg_cli",
    "~/.go2_camera_build/out/go2_camera_jpeg_cli",
    "~/go2_camera_jpeg_cli/go2_camera_jpeg_cli",
]


def find_cli():
    for c in CLI_CANDIDATES:
        p = os.path.expanduser(c) if c else ""
        if p and os.path.isfile(p) and os.access(p, os.X_OK):
            return p
    raise SystemExit("go2_camera_jpeg_cli not found (build it with go2-nav2-wifi robot-build-camera-cli.sh)")


def cli_env():
    env = os.environ.copy()
    # The CLI asks the robot's video service over the factory (internal) DDS:
    # it must not inherit this process's Wi-Fi DDS config or domain.
    env.pop("CYCLONEDDS_URI", None)
    env.pop("ROS_DOMAIN_ID", None)
    sdk = os.path.expanduser(os.environ.get("UNITREE_SDK2", "~/unitree_sdk2"))
    arch = os.uname().machine
    libs = [os.path.join(sdk, d) for d in ("lib", "build/lib", f"lib/{arch}", f"thirdparty/lib/{arch}")]
    extra = ":".join(p for p in libs if os.path.isdir(p))
    if extra:
        env["LD_LIBRARY_PATH"] = extra + ":" + env.get("LD_LIBRARY_PATH", "")
    return env


class CameraWifi(Node):
    def __init__(self):
        super().__init__("go2_camera_wifi")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(CompressedImage, TOPIC, qos)
        self.latest = None  # (receive time, jpeg bytes)
        self.lock = threading.Lock()
        self.cli = find_cli()
        self.proc = subprocess.Popen(
            [self.cli], stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0, env=cli_env()
        )
        threading.Thread(target=self.reader, daemon=True).start()
        self.sent = 0
        self.bytes = 0
        self.create_timer(1.0 / FPS, self.publish)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f"{self.cli} -> {TOPIC} at {FPS} Hz, width {WIDTH or 'native'}")

    def read_exact(self, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = self.proc.stdout.read(n - len(buf))
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    def reader(self):
        while True:
            hdr = self.read_exact(4)
            if hdr is None:
                err = self.proc.stderr.read(2000).decode("utf-8", "replace") if self.proc.stderr else ""
                self.get_logger().error(f"camera cli exited ({self.proc.poll()}): {err}")
                os._exit(1)
            (size,) = struct.unpack("!I", hdr)
            if not 0 < size < 5_000_000:
                continue
            data = self.read_exact(size)
            if data is None:
                continue
            with self.lock:
                self.latest = (self.get_clock().now(), data)

    def publish(self):
        with self.lock:
            item, self.latest = self.latest, None
        if item is None:
            return
        stamp, jpeg = item
        if WIDTH:
            img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                return
            if img.shape[1] > WIDTH:
                img = cv2.resize(img, (WIDTH, round(img.shape[0] * WIDTH / img.shape[1])), interpolation=cv2.INTER_AREA)
                ok, enc = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, QUALITY])
                if not ok:
                    return
                jpeg = enc.tobytes()
        msg = CompressedImage()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = FRAME_ID
        msg.format = "jpeg"
        msg.data = jpeg
        self.pub.publish(msg)
        self.sent += 1
        self.bytes += len(jpeg)

    def report(self):
        kb = self.bytes / max(1, self.sent) / 1024
        self.get_logger().info(f"{self.sent} frames in 10 s, {kb:.0f} KB/frame")
        self.sent = self.bytes = 0

    def close(self):
        self.proc.terminate()
        try:
            self.proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.proc.kill()


def main():
    rclpy.init()
    node = CameraWifi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    time.sleep(0.1)


if __name__ == "__main__":
    main()
