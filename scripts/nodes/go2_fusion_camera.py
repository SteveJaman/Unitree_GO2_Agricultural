#!/usr/bin/env python3
"""
go2_fusion_camera.py: the front camera as RTAB-Map's RGB-D fusion expects it.

For each camera frame (at most max_rate), publishes with one identical header:
  /twin/rgb/image          the image, downscaled and undistorted (RTAB-Map
                           assumes rectified images), frame front_camera_optical
  /twin/rgb/camera_info    the matching distortion-free intrinsics
  /twin/depth/image        16UC1 depth in mm from the LiDAR: the last
                           cloud_history registered scans projected into this
                           camera (z-buffer), small holes filled; 0 = no data
plus the static TF front_camera -> front_camera_optical (URDF x-forward ->
optical z-forward). RTAB-Map takes the three as RGB-D and colours its 3D map.

One node does both so image, depth and camera_info always share a stamp and a
reliable QoS. An image whose stamp is far from the ROS clock (Gazebo stamps
sim time, the LiDAR wall time) is restamped with the receive time.
Run by twin/start_inside.sh in fusion mode.
"""

import time

import numpy as np
import cv2
import yaml

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener


def quat_to_mat(x, y, z, w):
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def cloud_xyz(msg):
    pts = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    return np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float64)


def load_info(path):
    with open(path) as f:
        cal = yaml.safe_load(f)
    k = np.array(cal["camera_matrix"]["data"], float).reshape(3, 3)
    d = [float(v) for v in cal["distortion_coefficients"]["data"]]
    return k, d, (int(cal["image_width"]), int(cal["image_height"]))


class FusionCamera(Node):
    def __init__(self):
        super().__init__("go2_fusion_camera")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("output_image", "/twin/rgb/image")
        self.declare_parameter("output_info", "/twin/rgb/camera_info")
        self.declare_parameter("camera_info_file", "")
        self.declare_parameter("camera_frame", "front_camera")
        self.declare_parameter("optical_frame", "front_camera_optical")
        self.declare_parameter("max_rate", 5.0)  # Hz; RTAB-Map runs at 1 Hz
        self.declare_parameter("width", 640)  # output width (0 = as received)
        self.declare_parameter("output_depth", "/twin/depth/image")
        self.declare_parameter("cloud_topic", "/lidar3d/registered")
        self.declare_parameter("cloud_history", 3)  # registered scans per depth image
        self.declare_parameter("min_depth", 0.3)
        self.declare_parameter("max_depth", 15.0)
        self.declare_parameter(
            "fill_px", 3
        )  # fill holes up to this many px from a point

        self.k, self.d, self.cal_size = load_info(
            self.get_parameter("camera_info_file").value
        )
        self.optical = self.get_parameter("optical_frame").value

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.get_parameter("camera_frame").value
        tf.child_frame_id = self.optical
        # rpy (-pi/2, 0, -pi/2): z forward, x right, y down
        tf.transform.rotation.x, tf.transform.rotation.y = -0.5, 0.5
        tf.transform.rotation.z, tf.transform.rotation.w = -0.5, 0.5
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(tf)

        out_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self.img_pub = self.create_publisher(
            Image, self.get_parameter("output_image").value, out_qos
        )
        self.info_pub = self.create_publisher(
            CameraInfo, self.get_parameter("output_info").value, out_qos
        )
        self.depth_pub = self.create_publisher(
            Image, self.get_parameter("output_depth").value, out_qos
        )
        self.tf = Buffer(cache_time=Duration(seconds=10.0))
        TransformListener(self.tf, self)
        self.clouds = []  # (frame_id, Nx3) of the latest registered scans
        self.create_subscription(
            PointCloud2,
            self.get_parameter("cloud_topic").value,
            self.on_cloud,
            qos_profile_sensor_data,
        )
        self.depth_px = 0
        self.restamp_warned = False
        self.last = 0.0
        self.count = 0
        self.warned = False
        self.rectify = None  # (size, map1, map2, K) for the current output size
        # Match the publisher's reliability (relay: best effort, Gazebo: reliable).
        self.sub = None
        self.qos_timer = self.create_timer(1.0, self.subscribe)
        self.create_timer(10.0, self.report)

    def subscribe(self):
        topic = self.get_parameter("image_topic").value
        pubs = self.get_publishers_info_by_topic(topic)
        if not pubs:
            return
        reliable = all(
            p.qos_profile.reliability == ReliabilityPolicy.RELIABLE for p in pubs
        )
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE
            if reliable
            else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(Image, topic, self.on_image, qos)
        self.qos_timer.cancel()
        self.get_logger().info(
            f"{topic} -> {self.get_parameter('output_image').value} ({self.optical})"
        )

    def on_cloud(self, msg):
        pts = cloud_xyz(msg)
        if len(pts):
            self.clouds.append((msg.header.frame_id, pts))
            del self.clouds[: -max(1, self.get_parameter("cloud_history").value)]

    def depth_image(self, stamp, k, w, h):
        """LiDAR depth (32FC1, m) in the rectified camera, or None without TF."""
        depth = np.zeros((h, w), np.float32)
        zmin, zmax = (
            self.get_parameter("min_depth").value,
            self.get_parameter("max_depth").value,
        )
        for frame, pts in self.clouds:
            try:
                t = self.tf.lookup_transform(
                    self.optical,
                    frame,
                    Time.from_msg(stamp),
                    timeout=Duration(seconds=0.05),
                )
            except Exception:
                try:
                    t = self.tf.lookup_transform(self.optical, frame, Time())
                except Exception:
                    return None
            q, tr = t.transform.rotation, t.transform.translation
            c = pts @ quat_to_mat(q.x, q.y, q.z, q.w).T + np.array([tr.x, tr.y, tr.z])
            c = c[(c[:, 2] > zmin) & (c[:, 2] < zmax)]
            u = np.round(k[0, 0] * c[:, 0] / c[:, 2] + k[0, 2]).astype(int)
            v = np.round(k[1, 1] * c[:, 1] / c[:, 2] + k[1, 2]).astype(int)
            ok = (u >= 0) & (u < w) & (v >= 0) & (v < h)
            u, v, z = u[ok], v[ok], c[ok, 2].astype(np.float32)
            cur = depth[v, u]
            # z-buffer: keep the nearest point per pixel
            order = np.argsort(-z)
            depth[v[order], u[order]] = np.where(
                (cur[order] == 0) | (z[order] < cur[order]), z[order], cur[order]
            )
        self.depth_px += int((depth > 0).sum())
        n = self.get_parameter("fill_px").value
        if n > 0:
            near = cv2.erode(
                np.where(depth > 0, depth, np.inf).astype(np.float32),
                np.ones((2 * n + 1,) * 2, np.uint8),
            )
            depth = np.where((depth == 0) & np.isfinite(near), near, depth).astype(
                np.float32
            )
        return depth

    def on_image(self, msg):
        now = time.monotonic()
        if now - self.last < 1.0 / self.get_parameter("max_rate").value:
            return
        self.last = now
        if msg.encoding not in ("bgr8", "rgb8"):
            self.get_logger().warn(
                f"unsupported encoding {msg.encoding}", throttle_duration_sec=10.0
            )
            return
        if (msg.width, msg.height) != self.cal_size and not self.warned:
            self.get_logger().warn(
                f"image {msg.width}x{msg.height}, calibration {self.cal_size[0]}x{self.cal_size[1]}: "
                "camera matrix scaled; recalibrate at this resolution for best results"
            )
            self.warned = True
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // 3, 3)[
            :, : msg.width
        ]
        out_w = self.get_parameter("width").value or msg.width
        scale = out_w / msg.width
        if scale != 1.0:
            img = cv2.resize(
                img, (out_w, round(msg.height * scale)), interpolation=cv2.INTER_AREA
            )
        h, w = img.shape[:2]

        k = self.k.copy()
        k[0] *= w / self.cal_size[0]
        k[1] *= h / self.cal_size[1]
        if any(self.d):
            if self.rectify is None or self.rectify[0] != (w, h):
                d = np.array(self.d)
                new_k, _ = cv2.getOptimalNewCameraMatrix(k, d, (w, h), 0)
                m1, m2 = cv2.initUndistortRectifyMap(
                    k, d, None, new_k, (w, h), cv2.CV_16SC2
                )
                self.rectify = ((w, h), m1, m2, new_k)
            _, m1, m2, k = self.rectify
            img = cv2.remap(img, m1, m2, cv2.INTER_LINEAR)

        stamp = msg.header.stamp
        lag = self.get_clock().now().nanoseconds * 1e-9 - (
            stamp.sec + stamp.nanosec * 1e-9
        )
        if abs(lag) > 5.0:
            if not self.restamp_warned:
                self.get_logger().warn(
                    f"image clock is {lag:.0f} s off the ROS clock (simulation?): restamping"
                )
                self.restamp_warned = True
            stamp = self.get_clock().now().to_msg()
        depth = self.depth_image(stamp, k, w, h) if self.clouds else None
        if depth is None:
            return  # no LiDAR or TF yet: RTAB-Map needs image and depth together

        out = Image()
        out.header.stamp = stamp
        out.header.frame_id = self.optical
        out.height, out.width = h, w
        out.encoding = msg.encoding
        out.step = w * 3
        out.data = np.ascontiguousarray(img).tobytes()

        info = CameraInfo()
        info.header = out.header
        info.width, info.height = w, h
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5  # rectified above
        info.k = k.reshape(-1).tolist()
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            k[0, 0],
            0.0,
            k[0, 2],
            0.0,
            0.0,
            k[1, 1],
            k[1, 2],
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        dmsg = Image()
        dmsg.header = out.header
        dmsg.height, dmsg.width = h, w
        dmsg.encoding = "16UC1"  # mm, RTAB-Map's native depth format
        dmsg.step = w * 2
        dmsg.data = np.clip(depth * 1000.0, 0, 65535).astype(np.uint16).tobytes()
        self.img_pub.publish(out)
        self.info_pub.publish(info)
        self.depth_pub.publish(dmsg)
        self.count += 1

    def report(self):
        if self.sub is None:
            self.get_logger().info(
                f"waiting for {self.get_parameter('image_topic').value}"
            )
        else:
            px = self.depth_px // max(1, self.count)
            self.get_logger().info(
                f"{self.count} RGB-D frames in 10 s, {px} LiDAR depth px/frame (before fill)"
            )
            if not self.clouds:
                self.get_logger().warn(
                    f"no cloud on {self.get_parameter('cloud_topic').value}"
                )
        self.count = 0
        self.depth_px = 0


def main():
    rclpy.init()
    node = FusionCamera()
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
