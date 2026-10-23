#!/usr/bin/env python3
"""
go2_camera_obstacles.py: camera obstacles for the Nav2 costmap, ranged by the LiDAR.

The LiDAR misses glass, thin chair/table legs, dark absorbing surfaces and the
area just in front of the sensor. The front camera sees them but has no depth.
For each camera frame (at most `rate` Hz) this node:

  1. undistorts the image (calibration YAML) and runs Depth Anything V2-Small
     (ONNX, CPU), which gives relative inverse depth for every pixel;
  2. projects the latest registered LiDAR scans into the same image and fits
     the network output to them (disparity = a / z + b, robust least squares),
     which turns relative depth into metres;
  3. back-projects nearby pixels (<= max_range) to 3D, moves them to
     base_footprint, keeps those between the floor (estimated per frame) and
     max_height, and publishes them in the fixed map frame as
     /camera/obstacle_points.

Inference takes 0.5-2.5 s on a laptop CPU, so the last cloud is republished
(fresh stamp, world-fixed, so still correct while the dog moves) at
republish_rate for up to republish_hold seconds. If inference stalls longer,
publishing stops and Nav2's expected_update_rate stops the controller.

Nav2 adds that cloud as a marking-only observation source ("camera", see
scripts/tools/nav2_camera_source.py), so LiDAR rays through glass cannot clear
what the camera sees. /camera/obstacle_depth shows the metric depth with the
fit quality, for checking.

A frame whose fit is poor (too few LiDAR points in view, or median error above
max_fit_error) reuses the last good fit for up to fit_hold seconds, then
publishes an empty cloud.

Run by twin/start_inside.sh in go2-3d (wifi_3d_nav.sh --camera-obstacles).
Needs onnxruntime and the model (go2-3d image, wifi_3d_nav.sh --build).
"""

import math
import time

import numpy as np
import cv2
import yaml

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener

# URDF camera link (x fwd, y left, z up) -> optical (x right, y down, z fwd).
BODY_TO_OPTICAL = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)


def quat_to_mat(x, y, z, w):
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def tf_to_rt(t):
    q, p = t.transform.rotation, t.transform.translation
    return quat_to_mat(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z])


def cloud_xyz(msg):
    pts = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    return np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float64)


def fit_scale(disp, inv_z):
    """Robust a, b with disp ~ a * inv_z + b; returns (a, b, inlier mask) or None."""
    keep = np.ones(len(disp), bool)
    a = b = None
    for _ in range(4):
        if keep.sum() < 10:
            return None
        A = np.stack([inv_z[keep], np.ones(keep.sum())], axis=1)
        a, b = np.linalg.lstsq(A, disp[keep], rcond=None)[0]
        res = disp - (a * inv_z + b)
        mad = np.median(np.abs(res[keep] - np.median(res[keep]))) + 1e-6
        keep = np.abs(res) < 3.0 * 1.4826 * mad
    if a is None or a <= 0:
        return None
    return a, b, keep


class CameraObstacles(Node):
    def __init__(self):
        super().__init__("go2_camera_obstacles")
        p = self.declare_parameter
        p("image_topic", "/camera/image_raw")
        p("cloud_topic", "/lidar3d/registered")
        p("camera_info_file", "")
        p("model_path", "/opt/depth/depth_anything_v2_small.onnx")
        p("threads", 4)
        p("input_height", 266)  # model input, multiple of 14; width follows the aspect
        p("rate", 2.0)  # Hz
        p("camera_frame", "front_camera")  # URDF convention (x forward)
        p("base_frame", "base_footprint")
        p(
            "map_frame", "lidar3d_map"
        )  # output frame: obstacles stay put while the dog moves
        p("republish_rate", 2.0)  # Hz
        p("republish_hold", 3.0)  # s the last cloud may be republished
        p("output_topic", "/camera/obstacle_points")
        p("debug_topic", "/camera/obstacle_depth")
        p("min_range", 0.2)
        p("max_range", 3.0)  # m from the camera; monocular depth degrades with distance
        p("min_height", 0.08)  # m above the floor estimated per frame
        p("max_height", 1.0)  # m above the floor (the Go2 with the arm folded)
        p("stride", 3)  # px step at model resolution
        p("border", 0.04)  # fraction of the image edge ignored
        p("cloud_history", 3)
        p("min_fit_points", 30)
        p("max_fit_error", 0.25)  # median relative error of the fit on LiDAR points
        p("fit_hold", 2.0)  # s a good fit may be reused

        import onnxruntime as ort

        so = ort.SessionOptions()
        so.intra_op_num_threads = int(self.get_parameter("threads").value)
        self.session = ort.InferenceSession(
            self.get_parameter("model_path").value,
            so,
            providers=["CPUExecutionProvider"],
        )
        self.input_name = self.session.get_inputs()[0].name

        with open(self.get_parameter("camera_info_file").value) as f:
            cal = yaml.safe_load(f)
        self.k_cal = np.array(cal["camera_matrix"]["data"], float).reshape(3, 3)
        self.d_cal = np.array(cal["distortion_coefficients"]["data"], float)
        self.cal_size = (int(cal["image_width"]), int(cal["image_height"]))
        self.rect = None  # (input size, map1, map2, K at model resolution)

        self.tf = Buffer(cache_time=Duration(seconds=10.0))
        TransformListener(self.tf, self)
        self.clouds = []
        self.create_subscription(
            PointCloud2,
            self.get_parameter("cloud_topic").value,
            self.on_cloud,
            qos_profile_sensor_data,
        )
        out_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(
            PointCloud2, self.get_parameter("output_topic").value, out_qos
        )
        self.dbg = self.create_publisher(
            Image, self.get_parameter("debug_topic").value, out_qos
        )
        self.restamp_warned = False
        self.last_cloud = None  # (Nx3 in map_frame, monotonic time)
        self.create_timer(
            1.0 / self.get_parameter("republish_rate").value, self.republish
        )
        self.last_fit = None  # (a, b, monotonic time)
        self.last_run = 0.0
        self.image_sub = None
        self.qos_timer = self.create_timer(1.0, self.subscribe_image)
        self.stats = dict(
            frames=0, published=0, points=0, fit_fail=0, held=0, infer_ms=0.0, err=[]
        )
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f"camera obstacles: {self.get_parameter('model_path').value} at "
            f"{self.get_parameter('input_height').value} px, -> {self.get_parameter('output_topic').value}"
        )

    # ------------------------------------------------------------ inputs
    def subscribe_image(self):
        topic = self.get_parameter("image_topic").value
        pubs = self.get_publishers_info_by_topic(topic)
        if not pubs:
            return
        reliable = all(
            x.qos_profile.reliability == ReliabilityPolicy.RELIABLE for x in pubs
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

    def on_cloud(self, msg):
        pts = cloud_xyz(msg)
        if len(pts):
            self.clouds.append((msg.header.frame_id, pts))
            del self.clouds[: -max(1, self.get_parameter("cloud_history").value)]

    def lookup(self, target, source, stamp):
        try:
            return tf_to_rt(
                self.tf.lookup_transform(
                    target, source, Time.from_msg(stamp), timeout=Duration(seconds=0.05)
                )
            )
        except Exception:
            try:
                return tf_to_rt(self.tf.lookup_transform(target, source, Time()))
            except Exception:
                return None

    # ------------------------------------------------------------ pipeline
    def on_image(self, msg):
        now = time.monotonic()
        if now - self.last_run < 1.0 / self.get_parameter("rate").value:
            return
        self.last_run = now
        self.stats["frames"] += 1
        if msg.encoding not in ("bgr8", "rgb8"):
            self.get_logger().warn(
                f"unsupported encoding {msg.encoding}", throttle_duration_sec=10.0
            )
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // 3, 3)[
            :, : msg.width
        ]
        if msg.encoding == "bgr8":
            img = img[:, :, ::-1]  # model wants RGB

        # Undistort straight to the model resolution.
        h = int(self.get_parameter("input_height").value) // 14 * 14
        w = int(round(h * msg.width / msg.height / 14)) * 14
        if self.rect is None or self.rect[0] != (w, h, msg.width, msg.height):
            k = self.k_cal.copy()
            k[0] *= msg.width / self.cal_size[0]
            k[1] *= msg.height / self.cal_size[1]
            new_k, _ = cv2.getOptimalNewCameraMatrix(
                k, self.d_cal, (msg.width, msg.height), 0
            )
            k_model = new_k.copy()
            k_model[0] *= w / msg.width
            k_model[1] *= h / msg.height
            m1, m2 = cv2.initUndistortRectifyMap(
                k, self.d_cal, None, k_model, (w, h), cv2.CV_16SC2
            )
            self.rect = ((w, h, msg.width, msg.height), m1, m2, k_model)
        _, m1, m2, k = self.rect
        rgb = cv2.remap(np.ascontiguousarray(img), m1, m2, cv2.INTER_LINEAR)

        x = ((rgb.astype(np.float32) / 255.0 - MEAN) / STD).transpose(2, 0, 1)[None]
        t0 = time.monotonic()
        disp = self.session.run(None, {self.input_name: x})[0][0]
        self.stats["infer_ms"] += (time.monotonic() - t0) * 1000.0
        if disp.shape != (h, w):
            disp = cv2.resize(disp, (w, h), interpolation=cv2.INTER_LINEAR)

        stamp = msg.header.stamp
        lag = self.get_clock().now().nanoseconds * 1e-9 - (
            stamp.sec + stamp.nanosec * 1e-9
        )
        if abs(lag) > 5.0:
            # Gazebo stamps sim time, the LiDAR and TF wall time: Nav2 would drop
            # a cloud it cannot transform at its stamp.
            if not self.restamp_warned:
                self.get_logger().warn(
                    f"image clock is {lag:.0f} s off the ROS clock (simulation?): restamping"
                )
                self.restamp_warned = True
            stamp = self.get_clock().now().to_msg()
        cam = self.get_parameter("camera_frame").value
        fit = self.fit_to_lidar(disp, k, cam, stamp, w, h)
        if fit is None:
            if (
                self.last_fit
                and now - self.last_fit[2] < self.get_parameter("fit_hold").value
            ):
                fit = self.last_fit[:2]
                self.stats["held"] += 1
            else:
                self.stats["fit_fail"] += 1
                self.last_cloud = (np.zeros((0, 3)), now)
                self.publish_cloud(self.last_cloud[0], stamp)
                return
        else:
            self.last_fit = (fit[0], fit[1], now)
        a, b = fit

        # Metric depth, then 3D points in the optical frame.
        zmin, zmax = (
            self.get_parameter("min_range").value,
            self.get_parameter("max_range").value,
        )
        with np.errstate(divide="ignore", invalid="ignore"):
            depth = np.where(disp - b > 1e-6, a / (disp - b), np.inf)
        s = max(1, int(self.get_parameter("stride").value))
        bw = int(self.get_parameter("border").value * w)
        bh = int(self.get_parameter("border").value * h)
        vv, uu = np.mgrid[bh : h - bh : s, bw : w - bw : s]
        zz = depth[vv, uu]
        ok = np.isfinite(zz) & (zz > zmin) & (zz < zmax)
        uu, vv, zz = uu[ok], vv[ok], zz[ok]
        pts_opt = np.stack(
            [(uu - k[0, 2]) / k[0, 0] * zz, (vv - k[1, 2]) / k[1, 1] * zz, zz], axis=1
        )

        base = self.get_parameter("base_frame").value
        rt = self.lookup(base, cam, stamp)
        if rt is None:
            self.get_logger().warn(f"no TF {cam} -> {base}", throttle_duration_sec=10.0)
            return
        r, t = rt
        pts = (pts_opt @ BODY_TO_OPTICAL) @ r.T + t  # optical -> camera link -> base
        floor = self.estimate_floor(pts)
        lo = floor + self.get_parameter("min_height").value
        hi = floor + self.get_parameter("max_height").value
        pts = pts[(pts[:, 2] > lo) & (pts[:, 2] < hi)]
        mrt = self.lookup(self.get_parameter("map_frame").value, base, stamp)
        if mrt is None:
            self.get_logger().warn(
                f"no TF {base} -> {self.get_parameter('map_frame').value}",
                throttle_duration_sec=10.0,
            )
            return
        pts = pts @ mrt[0].T + mrt[1]
        self.last_cloud = (pts, now)
        self.publish_cloud(pts, stamp)
        self.publish_debug(depth, stamp, len(pts), floor, zmax)

    def fit_to_lidar(self, disp, k, cam, stamp, w, h):
        lidar = []
        for frame, pts in self.clouds:
            rt = self.lookup(cam, frame, stamp)
            if rt is None:
                continue
            r, t = rt
            lidar.append((pts @ r.T + t) @ BODY_TO_OPTICAL.T)
        if not lidar:
            return None
        c = np.vstack(lidar)
        c = c[(c[:, 2] > 0.3) & (c[:, 2] < 10.0)]
        u = np.round(k[0, 0] * c[:, 0] / c[:, 2] + k[0, 2]).astype(int)
        v = np.round(k[1, 1] * c[:, 1] / c[:, 2] + k[1, 2]).astype(int)
        ok = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        if ok.sum() < self.get_parameter("min_fit_points").value:
            return None
        z = c[ok, 2]
        d = disp[v[ok], u[ok]].astype(np.float64)
        res = fit_scale(d, 1.0 / z)
        if res is None:
            return None
        a, b, keep = res
        with np.errstate(divide="ignore", invalid="ignore"):
            z_fit = a / (d[keep] - b)
        err = float(np.median(np.abs(z_fit - z[keep]) / z[keep]))
        self.stats["err"].append(err)
        if not math.isfinite(err) or err > self.get_parameter("max_fit_error").value:
            return None
        return a, b

    @staticmethod
    def estimate_floor(pts):
        """Floor height in base_footprint: the densest low band of points."""
        if len(pts) < 50:
            return 0.0
        z = pts[:, 2]
        lo = np.percentile(z, 2)
        hist, edges = np.histogram(z[z < lo + 0.4], bins=20)
        return (
            float(edges[np.argmax(hist)] + (edges[1] - edges[0]) / 2)
            if hist.sum()
            else float(lo)
        )

    # ------------------------------------------------------------ outputs
    def republish(self):
        if self.last_cloud is None:
            return
        pts, t = self.last_cloud
        if time.monotonic() - t < self.get_parameter("republish_hold").value:
            self.publish_cloud(pts, self.get_clock().now().to_msg(), count=False)

    def publish_cloud(self, pts, stamp, count=True):
        out = PointCloud2()
        out.header.stamp = stamp
        out.header.frame_id = self.get_parameter("map_frame").value
        pts = np.asarray(pts, np.float32).reshape(-1, 3)
        out.height, out.width = 1, len(pts)
        out.fields = [
            PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, n in enumerate("xyz")
        ]
        out.is_bigendian, out.point_step = False, 12
        out.row_step, out.is_dense = 12 * len(pts), True
        out.data = pts.tobytes()
        self.pub.publish(out)
        if count:
            self.stats["published"] += 1
            self.stats["points"] += len(pts)

    def publish_debug(self, depth, stamp, n, floor, zmax):
        d = np.clip(np.nan_to_num(depth, posinf=zmax), 0, zmax) / zmax
        vis = cv2.applyColorMap((255 * (1 - d)).astype(np.uint8), cv2.COLORMAP_TURBO)
        err = self.stats["err"][-1] if self.stats["err"] else float("nan")
        label = f"{n} obstacle pts  fit err {err * 100:.0f}%  floor {floor:+.2f} m  red=near"
        cv2.putText(vis, label, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
        cv2.putText(
            vis, label, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1
        )
        o = Image()
        o.header.stamp = stamp
        o.header.frame_id = self.get_parameter("camera_frame").value
        o.height, o.width = vis.shape[:2]
        o.encoding = "bgr8"
        o.step = o.width * 3
        o.data = vis.tobytes()
        self.dbg.publish(o)

    def report(self):
        s = self.stats
        err = f"{np.median(s['err']) * 100:.0f}%" if s["err"] else "n/a"
        self.get_logger().info(
            f"frames {s['frames']}  published {s['published']}  obstacle pts/frame {s['points'] // max(1, s['published'])}  "
            f"inference {s['infer_ms'] / max(1, s['frames']):.0f} ms  fit err {err}  "
            f"held {s['held']}  no fit {s['fit_fail']}"
            + ("" if self.clouds else "  [no LiDAR cloud]")
            + (
                ""
                if self.image_sub
                else f"  [waiting for {self.get_parameter('image_topic').value}]"
            )
        )
        self.stats = dict(
            frames=0, published=0, points=0, fit_fail=0, held=0, infer_ms=0.0, err=[]
        )


def main():
    rclpy.init()
    node = CameraObstacles()
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
