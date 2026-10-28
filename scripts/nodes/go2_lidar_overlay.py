#!/usr/bin/env python3
"""
go2_lidar_overlay.py: project LiDAR points onto the front camera image.

The check that camera and LiDAR are fused correctly: with a good lens
calibration (camera_info) and a good camera pose in TF, the coloured LiDAR
points land on the objects they hit, and depth jumps line up with object
edges. Publishes /camera/lidar_overlay (bgr8), viewable in RViz's Image
display or rqt_image_view, with no WebGL needed.

Inputs
  /camera/image_raw        sensor_msgs/Image (bgr8 / rgb8 / mono8)
  /camera/camera_info      sensor_msgs/CameraInfo, or camera_info_file (YAML)
  /lidar3d/registered      sensor_msgs/PointCloud2 (any TF frame; lidar3d_map)
  TF  <cloud frame> -> camera_frame (front_camera in the Go2 URDF)

camera_frame follows the URDF convention (x forward, y left, z up) unless
frame_is_optical is true (z forward, x right, y down). offset_xyz/offset_rpy
(m, rad) shift the camera relative to camera_frame, so the mount can be tuned
live without editing the URDF:
    ros2 param set /go2_lidar_overlay offset_rpy "[0.0, 0.05, 0.0]"
"""

import math
import os

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener

# URDF camera link (x fwd, y left, z up) -> optical (x right, y down, z fwd).
BODY_TO_OPTICAL = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])


def quat_to_mat(x, y, z, w):
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(y),
        math.sin(y),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def load_camera_info_yaml(path):
    import yaml

    with open(path) as f:
        cal = yaml.safe_load(f)
    k = np.array(cal["camera_matrix"]["data"], float).reshape(3, 3)
    d = np.array(cal["distortion_coefficients"]["data"], float)
    return k, d, (int(cal["image_width"]), int(cal["image_height"]))


def image_to_bgr(msg):
    ch = 1 if msg.encoding == "mono8" else 3
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // ch, ch)[
        :, : msg.width
    ]
    if msg.encoding == "rgb8":
        img = img[:, :, ::-1]
    if ch == 1:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return np.ascontiguousarray(img)


def cloud_xyz(msg):
    if hasattr(point_cloud2, "read_points_numpy"):
        try:
            return point_cloud2.read_points_numpy(
                msg, field_names=("x", "y", "z"), skip_nans=True
            ).astype(np.float64)
        except (AssertionError, TypeError):
            pass  # mixed field types: fall back to the structured read
    pts = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    return np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float64)


class Overlay(Node):
    def __init__(self):
        super().__init__("go2_lidar_overlay")
        p = self.declare_parameter
        p("image_topic", "/camera/image_raw")
        p("camera_info_topic", "/camera/camera_info")
        p("camera_info_file", "")
        p("cloud_topic", "/lidar3d/registered")
        p("output_topic", "/camera/lidar_overlay")
        p("camera_frame", "front_camera")
        p("frame_is_optical", False)
        p("offset_xyz", [0.0, 0.0, 0.0])
        p("offset_rpy", [0.0, 0.0, 0.0])
        p("cloud_history", 3)  # registered scans kept (Point-LIO scans are sparse)
        p("min_depth", 0.3)
        p("max_depth", 15.0)
        p("point_size", 2)
        p("use_latest_tf", False)  # true: ignore stamps (clock skew between machines)

        self.k = self.d = self.info_size = None
        info_file = self.get_parameter("camera_info_file").value
        if info_file:
            if not os.path.isfile(info_file):
                raise SystemExit(f"camera_info_file {info_file} not found")
            self.k, self.d, self.info_size = load_camera_info_yaml(info_file)
        else:
            self.create_subscription(
                CameraInfo,
                self.get_parameter("camera_info_topic").value,
                self.on_info,
                qos_profile_sensor_data,
            )

        self.tf = Buffer(cache_time=Duration(seconds=10.0))
        TransformListener(self.tf, self)
        self.clouds = []  # (frame_id, Nx3)
        self.create_subscription(
            PointCloud2,
            self.get_parameter("cloud_topic").value,
            self.on_cloud,
            qos_profile_sensor_data,
        )
        # Subscribe with the publisher's reliability: the relay is best effort, Gazebo
        # reliable, and a best-effort reader of a reliable writer drops most large frames.
        self.image_sub = None
        self.qos_timer = self.create_timer(1.0, self.subscribe_image)
        out_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(
            Image, self.get_parameter("output_topic").value, out_qos
        )

        self.stats = dict(
            images=0, published=0, tf_fail=0, no_info=0, no_cloud=0, points=0
        )
        self.busy = False
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f"overlay: {self.get_parameter('cloud_topic').value} -> "
            f"{self.get_parameter('camera_frame').value} on {self.get_parameter('image_topic').value}, "
            f"camera_info from {info_file or self.get_parameter('camera_info_topic').value}"
        )

    def subscribe_image(self):
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
        self.image_sub = self.create_subscription(Image, topic, self.on_image, qos)
        self.qos_timer.cancel()
        self.get_logger().info(
            f"{topic}: {'reliable' if reliable else 'best effort'} subscription"
        )

    def on_info(self, msg):
        self.k = np.array(msg.k, float).reshape(3, 3)
        self.d = np.array(msg.d, float)
        self.info_size = (msg.width, msg.height)

    def on_cloud(self, msg):
        pts = cloud_xyz(msg)
        if len(pts):
            self.clouds.append((msg.header.frame_id, pts))
            del self.clouds[: -max(1, self.get_parameter("cloud_history").value)]

    def camera_from_frame(self, frame, stamp):
        cam = self.get_parameter("camera_frame").value
        when = (
            Time()
            if self.get_parameter("use_latest_tf").value
            else Time.from_msg(stamp)
        )
        try:
            t = self.tf.lookup_transform(
                cam, frame, when, timeout=Duration(seconds=0.05)
            )
        except Exception:
            try:  # image newer than the last TF: take the latest one
                t = self.tf.lookup_transform(cam, frame, Time())
            except Exception:
                return None
        q, tr = t.transform.rotation, t.transform.translation
        r = quat_to_mat(q.x, q.y, q.z, q.w)
        tvec = np.array([tr.x, tr.y, tr.z])
        # Mount correction: the true camera is offset by (R_o, t_o) in camera_frame.
        r_o = rpy_to_mat(*self.get_parameter("offset_rpy").value)
        t_o = np.array(self.get_parameter("offset_xyz").value, float)
        r, tvec = r_o.T @ r, r_o.T @ (tvec - t_o)
        if not self.get_parameter("frame_is_optical").value:
            r, tvec = BODY_TO_OPTICAL @ r, BODY_TO_OPTICAL @ tvec
        return r, tvec

    def on_image(self, msg):
        self.stats["images"] += 1
        if self.busy:
            return
        if self.k is None:
            self.stats["no_info"] += 1
            return
        if not self.clouds:
            self.stats["no_cloud"] += 1
            return
        if msg.encoding not in ("bgr8", "rgb8", "mono8"):
            self.get_logger().warn(
                f"unsupported encoding {msg.encoding}", throttle_duration_sec=10.0
            )
            return
        self.busy = True
        try:
            self.render(msg)
        finally:
            self.busy = False

    def render(self, msg):
        img = image_to_bgr(msg)
        h, w = img.shape[:2]
        k = self.k.copy()
        if self.info_size and self.info_size != (w, h):
            # Calibration at another resolution: scale the camera matrix.
            sx, sy = w / self.info_size[0], h / self.info_size[1]
            k[0] *= sx
            k[1] *= sy
        cams = []
        for frame, pts in self.clouds:
            tf = self.camera_from_frame(frame, msg.header.stamp)
            if tf is None:
                self.stats["tf_fail"] += 1
                return
            r, t = tf
            cams.append(pts @ r.T + t)
        pc = np.vstack(cams)
        zmin, zmax = (
            self.get_parameter("min_depth").value,
            self.get_parameter("max_depth").value,
        )
        pc = pc[(pc[:, 2] > zmin) & (pc[:, 2] < zmax)]
        # Keep points inside a slightly widened field of view before distorting.
        half_w = (w / 2) / k[0, 0] * 1.3
        half_h = (h / 2) / k[1, 1] * 1.3
        xn, yn = pc[:, 0] / pc[:, 2], pc[:, 1] / pc[:, 2]
        pc = pc[
            (np.abs(xn - (k[0, 2] - w / 2) / k[0, 0]) < half_w)
            & (np.abs(yn - (k[1, 2] - h / 2) / k[1, 1]) < half_h)
        ]
        if len(pc):
            uv, _ = cv2.projectPoints(pc, np.zeros(3), np.zeros(3), k, self.d)
            uv = uv.reshape(-1, 2)
            inside = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
            uv, depth = uv[inside], pc[inside, 2]
        else:
            uv, depth = np.zeros((0, 2)), np.zeros(0)
        self.stats["points"] += len(uv)

        out = img.copy()
        if len(uv):
            order = np.argsort(-depth)  # near points drawn last
            uv, depth = uv[order], depth[order]
            norm = np.clip(
                (np.log(depth) - math.log(zmin)) / (math.log(zmax) - math.log(zmin)),
                0,
                1,
            )
            colors = cv2.applyColorMap(
                (255 * (1 - norm)).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_JET
            ).reshape(-1, 3)
            u, v = uv[:, 0].astype(int), uv[:, 1].astype(int)
            r = max(0, int(self.get_parameter("point_size").value) // 2)
            for du in range(-r, r + 1):
                for dv in range(-r, r + 1):
                    uu, vv = np.clip(u + du, 0, w - 1), np.clip(v + dv, 0, h - 1)
                    out[vv, uu] = colors
        label = f"{len(uv)} pts  red=near blue=far"
        cv2.putText(out, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
        cv2.putText(
            out, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1
        )

        o = Image()
        o.header = msg.header
        o.height, o.width = h, w
        o.encoding = "bgr8"
        o.step = w * 3
        o.data = out.tobytes()
        self.pub.publish(o)
        self.stats["published"] += 1

    def report(self):
        s = self.stats
        n = max(1, s["published"])
        why = []
        if s["images"] == 0:
            why.append(f"no images on {self.get_parameter('image_topic').value}")
        if s["no_info"]:
            why.append("no camera_info (calibrate, or set camera_info_file)")
        if s["no_cloud"]:
            why.append(f"no cloud on {self.get_parameter('cloud_topic').value}")
        if s["tf_fail"]:
            why.append(f"no TF to {self.get_parameter('camera_frame').value}")
        self.get_logger().info(
            f"images {s['images']}  overlays {s['published']}  points/overlay {s['points'] // n}"
            + (f"  [{'; '.join(why)}]" if why else "")
        )
        self.stats = dict.fromkeys(s, 0)


def main():
    rclpy.init()
    node = Overlay()
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
