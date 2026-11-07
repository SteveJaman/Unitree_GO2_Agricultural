#!/usr/bin/env python3
"""Traversability layer: which low objects may the Go2 step over?

Geometry first, semantics second:
  1. Height source (param height_source):
       lidar - /lidar3d/registered, last accum_s seconds stacked (map frame)
       depth - depth image back-projected with its camera_info
       auto  - depth while it publishes, else LiDAR
  2. Floor height from the cloud (histogram mode near the TF estimate), then
     free-standing low objects are clustered on a grid and tracked.
  3. A VLM (Ollama) is asked once per object, on a camera crop, for
     {"name", "fragile", "label"}. It runs in a background thread and its answer
     is cached on the track. While it is pending the object counts as "caution".
  4. decide(): an object is traversable only if it is lower than step_limit
     AND the VLM did not say avoid/fragile. The VLM can only restrict.

Publishes
  /traversability/obstacles  PointCloud2 (map frame): objects that must be avoided.
                             Add as a Nav2 observation source with
                             scripts/tools/add-traversability-costmap-source.sh.
                             Traversable objects are simply left out.
  /traversability/markers    MarkerArray (green traversable / orange caution / red avoid)
  /traversability/status     String JSON (tracks, source, floor) for logging and benchmarks
  /gait_mode                 String JSON {"mode": "normal"|"step_over", "clearance": m, ...}
                             The sim has no gait; on the real Go2 this is what a gait
                             supervisor would consume.
Subscribes: /lidar3d/registered, /camera/image_raw, /camera/camera_info,
            optional depth image + camera_info, TF.
"""
import base64
import json
import math
import os
import sys
import threading
import time
import urllib.error
import urllib.request
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import traversability_core as tc  # noqa: E402

IMAGE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
COLORS = {'traversable': (0.1, 0.8, 0.2), 'caution': (1.0, 0.6, 0.0), 'avoid': (0.9, 0.1, 0.1)}


# ----------------------------------------------------------------- helpers (as in yolo_node.py)
def transform_matrix(tf):
    q, t = tf.rotation, tf.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R, np.array([t.x, t.y, t.z])


def cloud_xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    if not {'x', 'y', 'z'} <= off.keys() or msg.point_step == 0:
        return np.empty((0, 3))
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                   'offsets': [off['x'], off['y'], off['z']], 'itemsize': msg.point_step})
    raw = np.frombuffer(bytes(msg.data), dtype=dt, count=msg.width * msg.height)
    pts = np.stack([raw['x'], raw['y'], raw['z']], axis=1).astype(np.float64)
    return pts[np.isfinite(pts).all(axis=1)]


def image_bgr(msg):
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if msg.encoding in ('bgr8', 'rgb8'):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1].copy() if msg.encoding == 'rgb8' else img
    if msg.encoding == 'mono8':
        img = buf.reshape(msg.height, msg.step)[:, :msg.width]
        return np.repeat(img[:, :, None], 3, axis=2)
    return None


def depth_meters(msg):
    """32FC1 (m) or 16UC1 (mm) depth image -> HxW float32 metres, or None."""
    raw = bytes(msg.data)
    if msg.encoding == '32FC1':
        return np.frombuffer(raw, np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
    if msg.encoding in ('16UC1', 'mono16'):
        d = np.frombuffer(raw, np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
        return d.astype(np.float32) / 1000.0
    return None


class Traversability(Node):
    def __init__(self):
        super().__init__('traversability')
        p = self.declare_parameter
        p('cloud_topic', '/lidar3d/registered')
        p('image_topic', '/camera/image_raw')
        p('camera_info_topic', '/camera/camera_info')
        p('depth_topic', '/camera/depth/image_raw')
        p('depth_info_topic', '/camera/depth/camera_info')
        p('height_source', 'auto')          # lidar | depth | auto
        p('depth_timeout', 3.0)             # s without depth before auto falls back to LiDAR
        p('depth_stride', 4)                # use every Nth depth pixel
        p('map_frame', 'lidar3d_map')
        p('base_frame', 'base_link')
        p('base_height', 0.30)              # m, base_link above the floor (TF floor guess)
        p('max_range', 5.0)                 # m around the robot that is analysed
        p('accum_s', 1.5)                   # s of LiDAR scans stacked together
        p('rate_hz', 2.0)
        p('step_limit', 0.08)               # m: at or above this the object is always avoided
        p('cell', 0.08)                     # m, clustering grid
        p('h_min', 0.012)                   # m above the floor before a point counts
        p('h_tall', 0.5)                    # m: taller cells are walls/furniture, not candidates
        p('min_cell_pts', 2)
        p('min_seen', 2)                    # ticks before a track is trusted
        p('track_ttl', 30.0)
        p('use_vlm', True)                  # False = height-only ablation
        p('vlm_url', os.environ.get('TWIN_VLM_URL', 'http://localhost:11434/api/generate'))
        p('vlm_model', os.environ.get('TWIN_VLM_MODEL', 'llava:7b'))
        p('vlm_timeout', 120.0)
        p('vlm_retry_s', 30.0)
        p('crop_size', 224)
        p('gait_trigger_dist', 0.7)         # m ahead: announce step_over mode
        p('gait_lateral', 0.35)             # m either side of the heading line
        p('clearance_margin', 0.04)         # m added to the object height
        p('max_clearance', 0.15)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731

        self.src = str(gp('height_source'))
        self.depth_timeout, self.stride = float(gp('depth_timeout')), max(1, int(gp('depth_stride')))
        self.map_frame, self.base_frame = gp('map_frame'), gp('base_frame')
        self.base_height, self.max_range = float(gp('base_height')), float(gp('max_range'))
        self.accum_s = float(gp('accum_s'))
        self.step_limit = float(gp('step_limit'))
        self.cell, self.h_min, self.h_tall = float(gp('cell')), float(gp('h_min')), float(gp('h_tall'))
        self.min_cell_pts, self.min_seen = int(gp('min_cell_pts')), int(gp('min_seen'))
        self.use_vlm = bool(gp('use_vlm'))
        self.vlm_url, self.vlm_model = gp('vlm_url'), gp('vlm_model')
        self.vlm_timeout, self.vlm_retry = float(gp('vlm_timeout')), float(gp('vlm_retry_s'))
        self.crop_size = int(gp('crop_size'))
        self.gait_dist, self.gait_lat = float(gp('gait_trigger_dist')), float(gp('gait_lateral'))
        self.margin, self.max_clear = float(gp('clearance_margin')), float(gp('max_clearance'))

        self.tracker = tc.Tracker(ttl=float(gp('track_ttl')))
        self.clouds = deque()               # (monotonic time, Nx3 map-frame points)
        self.image = self.info = None
        self.depth_msg = self.depth_info = None
        self.depth_seen = -1e9
        self.last_floor = None
        self.active_source = 'none'
        self.vlm_busy = False
        self.lock = threading.Lock()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.obs_pub = self.create_publisher(PointCloud2, '/traversability/obstacles', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/traversability/markers', 10)
        self.status_pub = self.create_publisher(String, '/traversability/status', 10)
        self.gait_pub = self.create_publisher(String, '/gait_mode', 10)
        self.create_subscription(PointCloud2, gp('cloud_topic'), self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Image, gp('image_topic'), self.on_image, IMAGE_QOS)
        self.create_subscription(CameraInfo, gp('camera_info_topic'), self.on_info, qos_profile_sensor_data)
        if self.src in ('depth', 'auto'):
            self.create_subscription(Image, gp('depth_topic'), self.on_depth, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, gp('depth_info_topic'), self.on_depth_info,
                                     qos_profile_sensor_data)
        self.create_timer(1.0 / max(0.2, float(gp('rate_hz'))), self.tick)
        self.get_logger().info(
            f'traversability: source={self.src} step_limit={self.step_limit} m '
            f'vlm={"on " + self.vlm_model if self.use_vlm else "off (height only)"}')

    # ------------------------------------------------------------ inputs
    def on_cloud(self, msg):
        pts = cloud_xyz(msg)
        frame = msg.header.frame_id
        if len(pts) and frame and frame != self.map_frame:
            try:
                R, t = transform_matrix(self.tf_buffer.lookup_transform(
                    self.map_frame, frame, Time()).transform)
                pts = pts @ R.T + t
            except TransformException:
                return
        now = time.monotonic()
        self.clouds.append((now, pts))
        while self.clouds and now - self.clouds[0][0] > self.accum_s:
            self.clouds.popleft()

    def on_image(self, msg):
        self.image = msg

    def on_info(self, msg):
        self.info = msg

    def on_depth(self, msg):
        self.depth_msg, self.depth_seen = msg, time.monotonic()

    def on_depth_info(self, msg):
        self.depth_info = msg

    # ------------------------------------------------------------ height source
    def depth_points(self):
        msg, info = self.depth_msg, self.depth_info or self.info
        if msg is None or info is None:
            return None
        d = depth_meters(msg)
        if d is None:
            self.get_logger().warn(f'unsupported depth encoding {msg.encoding!r}', once=True)
            return None
        try:
            R, t = transform_matrix(self.tf_buffer.lookup_transform(
                self.map_frame, msg.header.frame_id, Time()).transform)
        except TransformException:
            return None
        s = self.stride
        d = d[::s, ::s]
        v, u = np.meshgrid(np.arange(0, msg.height, s), np.arange(0, msg.width, s), indexing='ij')
        ok = np.isfinite(d) & (d > 0.2) & (d < self.max_range)
        sx = msg.width / info.width if info.width else 1.0
        sy = msg.height / info.height if info.height else 1.0
        fx, fy, cx, cy = info.k[0] * sx, info.k[4] * sy, info.k[2] * sx, info.k[5] * sy
        z = d[ok].astype(np.float64)
        cam = np.stack([(u[ok] - cx) / fx * z, (v[ok] - cy) / fy * z, z], axis=1)
        return cam @ R.T + t

    def get_points(self):
        now = time.monotonic()
        depth_ok = self.depth_msg is not None and now - self.depth_seen < self.depth_timeout
        if self.src == 'depth' or (self.src == 'auto' and depth_ok):
            pts = self.depth_points()
            if pts is not None:
                self.active_source = 'depth'
                return pts
            if self.src == 'depth':
                return None
        self.active_source = 'lidar'
        fresh = [p for _, p in self.clouds if len(p)]
        return np.vstack(fresh) if fresh else None

    # ------------------------------------------------------------ main loop
    def tick(self):
        try:
            R_b, t_b = transform_matrix(self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time()).transform)
        except TransformException as e:
            self.get_logger().warn(f'no TF {self.map_frame} -> {self.base_frame}: {e}',
                                   throttle_duration_sec=10.0)
            return
        bx, by, bz = t_b
        yaw = math.atan2(R_b[1, 0], R_b[0, 0])
        floor_z = self.last_floor if self.last_floor is not None else bz - self.base_height

        pts = self.get_points()
        if pts is not None and len(pts):
            near = np.hypot(pts[:, 0] - bx, pts[:, 1] - by) < self.max_range
            pts = pts[near]
            floor_z = tc.estimate_floor(pts, bz - self.base_height)
            self.last_floor = floor_z
            cands = tc.find_candidates(pts, floor_z, cell=self.cell, h_min=self.h_min,
                                       h_tall=self.h_tall, min_cell_pts=self.min_cell_pts)
        else:
            cands = []
        tracks = self.tracker.update(cands, time.monotonic())

        for t in tracks:
            t.decision = tc.decide(t.height, t.verdict, self.step_limit, self.use_vlm)
        if self.use_vlm:
            self.maybe_ask_vlm(tracks, floor_z, bx, by)
        self.publish_all(tracks, floor_z, bx, by, yaw)

    # ------------------------------------------------------------ VLM
    def maybe_ask_vlm(self, tracks, floor_z, bx, by):
        if self.vlm_busy:
            return
        now = time.monotonic()
        todo = [t for t in tracks if t.confirmed(self.min_seen) and t.verdict is None
                and t.height < self.step_limit and now >= t.retry_at]
        todo.sort(key=lambda t: math.hypot(t.x - bx, t.y - by))
        for t in todo:
            crop = self.make_crop(t, floor_z)
            if crop is None:
                continue
            with self.lock:
                self.vlm_busy = True
            threading.Thread(target=self.ask, args=(t, crop), daemon=True).start()
            return

    def make_crop(self, t, floor_z):
        msg, info = self.image, self.info
        if msg is None or info is None:
            return None
        try:
            R, tr = transform_matrix(self.tf_buffer.lookup_transform(
                self.map_frame, msg.header.frame_id, Time()).transform)
        except TransformException:
            return None
        (x0, y0), (x1, y1) = t.bbox
        corners = np.array([(x, y, z) for x in (x0, x1) for y in (y0, y1)
                            for z in (floor_z, floor_z + t.height)])
        cam = (corners - tr) @ R
        if np.any(cam[:, 2] < 0.2):
            return None
        sx = msg.width / info.width if info.width else 1.0
        sy = msg.height / info.height if info.height else 1.0
        fx, fy, cx, cy = info.k[0] * sx, info.k[4] * sy, info.k[2] * sx, info.k[5] * sy
        u = fx * cam[:, 0] / cam[:, 2] + cx
        v = fy * cam[:, 1] / cam[:, 2] + cy
        cu, cv = float(u.mean()), float(v.mean())
        if not (0 <= cu < msg.width and 0 <= cv < msg.height):
            return None
        half = max(48.0, 1.5 * max(u.max() - u.min(), v.max() - v.min()))
        xa, xb = int(max(0, cu - half)), int(min(msg.width, cu + half))
        ya, yb = int(max(0, cv - half)), int(min(msg.height, cv + half))
        img = image_bgr(msg)
        if img is None or xb - xa < 8 or yb - ya < 8:
            return None
        crop = img[ya:yb, xa:xb]
        scale = self.crop_size / max(crop.shape[:2])
        return cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    def ask(self, t, crop):
        try:
            ok, jpg = cv2.imencode('.jpg', crop, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                return
            body = json.dumps({
                'model': self.vlm_model, 'prompt': tc.build_prompt(t.height), 'stream': False,
                'format': 'json', 'images': [base64.b64encode(jpg.tobytes()).decode('ascii')],
            }).encode()
            req = urllib.request.Request(self.vlm_url, data=body,
                                         headers={'Content-Type': 'application/json'})
            t0 = time.monotonic()
            with urllib.request.urlopen(req, timeout=self.vlm_timeout) as r:
                text = json.loads(r.read().decode()).get('response', '')
            verdict = tc.parse_verdict(text)
            if verdict is None:
                t.retry_at = time.monotonic() + self.vlm_retry
                self.get_logger().warn(f'track {t.id}: unusable VLM reply {text[:100]!r}')
            else:
                t.verdict = verdict
                self.get_logger().info(
                    f'track {t.id} ({t.height * 100:.0f} cm at {t.x:.1f},{t.y:.1f}): '
                    f'{verdict["label"]} fragile={verdict["fragile"]} name={verdict["name"]!r} '
                    f'({time.monotonic() - t0:.0f} s)')
        except (urllib.error.URLError, OSError, TimeoutError, ValueError) as e:
            t.retry_at = time.monotonic() + self.vlm_retry
            self.get_logger().warn(f'Ollama call failed at {self.vlm_url}: {e}',
                                   throttle_duration_sec=30.0)
        finally:
            with self.lock:
                self.vlm_busy = False

    # ------------------------------------------------------------ outputs
    def publish_all(self, tracks, floor_z, bx, by, yaw):
        stamp = self.get_clock().now().to_msg()
        hdr = Header()
        hdr.stamp, hdr.frame_id = stamp, self.map_frame

        trusted = [t for t in tracks if t.confirmed(self.min_seen)]
        points = []
        for t in trusted:
            if t.decision != 'traversable':
                z = floor_z + max(0.05, min(t.height, 0.4))
                points.extend((cx, cy, z) for cx, cy in t.cells)
        self.obs_pub.publish(create_cloud_xyz32(hdr, points))

        markers = [Marker(action=Marker.DELETEALL)]
        markers[0].header = hdr
        for t in trusted:
            (x0, y0), (x1, y1) = t.bbox
            r, g, b = COLORS[t.decision]
            cube = Marker(ns='trav', id=2 * t.id, type=Marker.CUBE, action=Marker.ADD)
            cube.header = hdr
            cube.pose.position.x, cube.pose.position.y = (x0 + x1) / 2, (y0 + y1) / 2
            cube.pose.position.z = floor_z + t.height / 2
            cube.pose.orientation.w = 1.0
            cube.scale.x, cube.scale.y = max(x1 - x0, 0.05), max(y1 - y0, 0.05)
            cube.scale.z = max(t.height, 0.01)
            cube.color.r, cube.color.g, cube.color.b, cube.color.a = r, g, b, 0.6
            text = Marker(ns='trav', id=2 * t.id + 1, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            text.header = hdr
            text.pose.position.x, text.pose.position.y = cube.pose.position.x, cube.pose.position.y
            text.pose.position.z = floor_z + t.height + 0.2
            text.pose.orientation.w = 1.0
            text.scale.z = 0.15
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            name = t.verdict['name'] if t.verdict else '?'
            text.text = f'#{t.id} {name} {t.height * 100:.0f}cm {t.decision}'
            markers += [cube, text]
        self.marker_pub.publish(MarkerArray(markers=markers))

        self.gait_pub.publish(String(data=json.dumps(self.gait_mode(trusted, bx, by, yaw))))
        status = {'source': self.active_source, 'floor_z': round(floor_z, 3), 'tracks': [
            {'id': t.id, 'x': round(t.x, 2), 'y': round(t.y, 2), 'height': round(t.height, 3),
             'n_pts': t.n_pts, 'decision': t.decision,
             'vlm': t.verdict} for t in trusted]}
        self.status_pub.publish(String(data=json.dumps(status)))
        self.get_logger().info(
            f'[{self.active_source}] floor {floor_z:+.3f} m, ' + (', '.join(
                f'#{t.id} {t.height * 100:.0f}cm {t.decision}' for t in trusted) or 'no objects'),
            throttle_duration_sec=5.0)

    def gait_mode(self, trusted, bx, by, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        best = None
        for t in trusted:
            if t.decision != 'traversable':
                continue
            dx, dy = t.x - bx, t.y - by
            fwd, lat = c * dx + s * dy, -s * dx + c * dy
            if 0.0 < fwd < self.gait_dist and abs(lat) < self.gait_lat and (best is None or fwd < best[0]):
                best = (fwd, t)
        if best is None:
            return {'mode': 'normal', 'clearance': 0.0}
        t = best[1]
        return {'mode': 'step_over', 'object_id': t.id, 'distance': round(best[0], 2),
                'clearance': round(min(self.max_clear, t.height + self.margin), 3)}


def main():
    rclpy.init()
    node = Traversability()
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
