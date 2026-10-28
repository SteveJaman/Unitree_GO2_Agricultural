#!/usr/bin/env python3
"""VLM semantic costmap: camera + vision-language model -> Nav2 keepout mask + speed limit.

Each cycle the node takes the latest camera frame and the robot pose, splits
the lower (ground) part of the image into vertical strips, and asks the VLM
the configured yes/no questions about each strip. A "yes" marks that strip's
ground footprint in a mask; Nav2's KeepoutFilter (configured in go2-nav2-wifi's
lidar3d_planner.yaml) turns the mask into costmap cost, so the planner routes
around it and frontier_explorer.py stops seeing frontiers there.
"Slow" questions (e.g. a person in view) publish nav2_msgs/SpeedLimit instead.

Safety: the mask only ADDS cost. Unmarked cells are -1 (no information), so
the filter never lowers a lidar obstacle or turns unknown space into free.

Topics:
  in : image_topic (sensor_msgs/Image or CompressedImage), TF map_frame->robot_frame
  out: /vlm/keepout_mask       nav_msgs/OccupancyGrid (transient local)
       /costmap_filter_info    nav2_msgs/CostmapFilterInfo (transient local, once)
       /speed_limit            nav2_msgs/SpeedLimit (percentage; 0 = no limit)
       /vlm/status             std_msgs/String
       /vlm/debug_image        sensor_msgs/Image (strips coloured by answer)
"""
import math
import queue
import signal
import threading
import time
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
import yaml
from nav2_msgs.msg import CostmapFilterInfo, SpeedLimit
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from go2_vlm.backends import make_backend, ollama_ready
from go2_vlm.geometry import CameraModel, RobotPose, region_polygon, strip_bounds

KEEPOUT_FILTER = 0  # CostmapFilterInfo.type for KeepoutFilter


def image_to_bgr(msg) -> Optional[np.ndarray]:
    if isinstance(msg, CompressedImage):
        return cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
    channels = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4, 'mono8': 1}.get(msg.encoding)
    if channels is None:
        return None
    a = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
    a = a[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    if msg.encoding == 'rgb8':
        return a[..., ::-1].copy()
    if msg.encoding in ('bgra8', 'rgba8'):
        a = a[..., :3]
        return a[..., ::-1].copy() if msg.encoding == 'rgba8' else a.copy()
    if msg.encoding == 'mono8':
        return cv2.cvtColor(a, cv2.COLOR_GRAY2BGR)
    return a.copy()


def load_queries(path: str) -> List[Dict]:
    with open(path) as f:
        qs = yaml.safe_load(f).get('queries', [])
    for q in qs:
        q.setdefault('behavior', 'keepout')
        q.setdefault('per_strip', q['behavior'] == 'keepout')
        q.setdefault('cost', 100)
        q.setdefault('speed_percent', 50.0)
        if q['behavior'] not in ('keepout', 'slow'):
            raise ValueError(f"query {q['name']}: behavior must be keepout or slow")
        if not 1 <= int(q['cost']) <= 100:
            raise ValueError(f"query {q['name']}: cost must be 1..100")
    return qs


class SemanticCostmap(Node):

    def __init__(self):
        super().__init__('vlm_semantic_costmap')
        d = self.declare_parameter
        d('image_topic', '/camera/image_raw')
        d('compressed', False)
        d('map_frame', 'lidar3d_map')
        d('robot_frame', 'base_link')
        d('backend', 'mock')
        d('ollama_url', 'http://127.0.0.1:11434')
        d('model', 'moondream')
        d('request_timeout_s', 180.0)
        d('max_image_side', 448)
        d('queries_file', '')
        d('period_s', 5.0)
        d('n_strips', 3)
        d('ground_top_frac', 0.55)
        d('cam_hfov_deg', 90.0)
        d('cam_x', 0.0)
        d('cam_y', 0.0)
        d('cam_z', 0.0)
        d('cam_pitch_down_deg', 0.0)
        d('base_height', 0.30)
        d('max_range', 4.0)
        d('mask_topic', '/vlm/keepout_mask')
        d('filter_info_topic', '/costmap_filter_info')
        d('mask_size_m', 60.0)
        d('mask_resolution', 0.1)
        d('mark_ttl_s', 180.0)
        d('speed_limit_topic', '/speed_limit')
        d('speed_clear_cycles', 2)
        g = lambda n: self.get_parameter(n).value

        self.map_frame, self.robot_frame = g('map_frame'), g('robot_frame')
        self.period_s = float(g('period_s'))
        self.n_strips = int(g('n_strips'))
        self.top_frac = float(g('ground_top_frac'))
        self.cam_params = dict(hfov_rad=math.radians(float(g('cam_hfov_deg'))),
                               x=float(g('cam_x')), y=float(g('cam_y')), z=float(g('cam_z')),
                               pitch_down=math.radians(float(g('cam_pitch_down_deg'))))
        self.base_height = float(g('base_height'))
        self.max_range = float(g('max_range'))
        self.mask_size = float(g('mask_size_m'))
        self.res = float(g('mask_resolution'))
        self.ttl = float(g('mark_ttl_s'))
        self.speed_clear_cycles = int(g('speed_clear_cycles'))
        self.backend_kind = g('backend')

        if not g('queries_file'):
            raise RuntimeError('queries_file parameter is required')
        self.queries = load_queries(g('queries_file'))
        self.backend = make_backend(self.backend_kind, g('ollama_url'), g('model'),
                                    float(g('request_timeout_s')), int(g('max_image_side')))
        if self.backend_kind == 'ollama':
            ok, why = ollama_ready(g('ollama_url'), g('model'))
            (self.get_logger().info if ok else self.get_logger().error)(
                f'Ollama check: {why}')

        # Mask state, one layer per keepout query: value 0..100 and expiry time.
        self.origin = None          # (x, y) of cell (0, 0), set from the first pose
        n = int(round(self.mask_size / self.res))
        self.n = n
        self.layers = {q['name']: (np.zeros((n, n), np.int8), np.zeros((n, n)))
                       for q in self.queries if q['behavior'] == 'keepout'}
        self.slow_active: Dict[str, int] = {}   # name -> consecutive "no" count
        self.speed_limit = 0.0

        self.latest_image = None
        self.jobs: 'queue.Queue' = queue.Queue(maxsize=1)
        self.results: 'queue.Queue' = queue.Queue()
        self.busy = False
        self.cycles = 0
        self.last_cycle_start = 0.0

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mask_pub = self.create_publisher(OccupancyGrid, g('mask_topic'), latched)
        self.info_pub = self.create_publisher(CostmapFilterInfo, g('filter_info_topic'), latched)
        self.speed_pub = self.create_publisher(SpeedLimit, g('speed_limit_topic'), 10)
        self.status_pub = self.create_publisher(String, '/vlm/status', 10)
        self.debug_pub = self.create_publisher(Image, '/vlm/debug_image', 1)

        info = CostmapFilterInfo()
        info.header.frame_id = self.map_frame
        info.type = KEEPOUT_FILTER
        info.filter_mask_topic = g('mask_topic')
        info.base, info.multiplier = 0.0, 1.0
        self.info_pub.publish(info)

        msg_type = CompressedImage if g('compressed') else Image
        self.create_subscription(msg_type, g('image_topic'), self.on_image, 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        threading.Thread(target=self.worker, daemon=True).start()
        self.create_timer(0.5, self.tick)
        self.create_timer(2.0, self.publish_mask)

        self.get_logger().info(
            f'VLM semantic costmap: backend={self.backend_kind} image={g("image_topic")} '
            f'queries={[q["name"] for q in self.queries]} strips={self.n_strips} '
            f'mask={g("mask_topic")} {self.n}x{self.n}@{self.res}m')

    # ------------------------------------------------------------------
    def on_image(self, msg):
        self.latest_image = msg

    def robot_pose(self) -> Optional[RobotPose]:
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame,
                                                rclpy.time.Time()).transform
        except Exception:
            return None
        q = t.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return RobotPose(t.translation.x, t.translation.y, t.translation.z, yaw)

    def tick(self):
        while not self.results.empty():
            self.apply(self.results.get())
            self.busy = False
        if self.busy or self.latest_image is None:
            return
        if time.time() - self.last_cycle_start < self.period_s:
            return
        pose = self.robot_pose()
        if pose is None:
            self.get_logger().warn('no robot pose yet', throttle_duration_sec=10.0)
            return
        if self.origin is None:
            self.origin = (pose.x - self.mask_size / 2.0, pose.y - self.mask_size / 2.0)
        self.busy = True
        self.last_cycle_start = time.time()
        self.jobs.put((self.latest_image, pose))

    # ------------------------------------------------------------------
    def worker(self):
        """Runs VLM calls off the ROS thread; they take seconds to minutes."""
        while True:
            msg, pose = self.jobs.get()
            t0 = time.time()
            answers = []   # (query, strip_index or None, yes, text)
            img = None
            try:
                img = image_to_bgr(msg)
                if img is None:
                    raise RuntimeError(f'unsupported image encoding {getattr(msg, "encoding", "?")}')
                h, w = img.shape[:2]
                strips = strip_bounds(w, h, self.n_strips, self.top_frac)
                for q in self.queries:
                    if q['per_strip']:
                        for i, (u0, u1, v0, v1) in enumerate(strips):
                            yes, text = self.backend.ask(img[v0:v1 + 1, u0:u1 + 1],
                                                         q['question'], q.get('mock'))
                            answers.append((q, i, yes, text))
                    else:
                        yes, text = self.backend.ask(img, q['question'], q.get('mock'))
                        answers.append((q, None, yes, text))
                err = None
            except Exception as exc:  # keep the node alive on backend errors
                err = str(exc)
            self.results.put(dict(img=img, pose=pose, answers=answers, err=err,
                                  dt=time.time() - t0))

    # ------------------------------------------------------------------
    def apply(self, r):
        self.cycles += 1
        if r['err']:
            self.get_logger().error(f'VLM cycle failed: {r["err"]}')
            self.publish_status(f'error: {r["err"]}')
            return
        img, pose = r['img'], r['pose']
        h, w = img.shape[:2]
        cam = CameraModel(width=w, height=h, **self.cam_params)
        strips = strip_bounds(w, h, self.n_strips, self.top_frac)
        now = time.time()
        summary = []
        hits = set()
        for q, i, yes, text in r['answers']:
            summary.append(f"{q['name']}{'' if i is None else f'[{i}]'}={'Y' if yes else 'n'}")
            if q['behavior'] == 'keepout':
                u0, u1, v0, v1 = strips[i]
                cells = self.raster(region_polygon(cam, pose, u0, u1, v0, v1,
                                                   self.base_height, self.max_range))
                val, exp = self.layers[q['name']]
                if yes:
                    hits.add(i)
                    val[cells] = np.maximum(val[cells], int(q['cost']))
                    exp[cells] = now + self.ttl
                else:
                    # The VLM may withdraw only its own marks, never lidar cost.
                    val[cells] = 0
            else:
                self.update_slow(q, yes)
        self.publish_speed()
        self.publish_mask()
        self.publish_debug(img, strips, hits)
        line = (f'cycle {self.cycles} ({r["dt"]:.1f}s) at ({pose.x:.1f}, {pose.y:.1f}): '
                + ' '.join(summary) + f' speed_limit={self.speed_limit:.0f}%')
        self.get_logger().info(line)
        self.publish_status(line)

    def raster(self, poly):
        """Boolean cell mask for a map-frame polygon."""
        cells = np.zeros((self.n, self.n), np.uint8)
        if self.origin is None or len(poly) < 3:
            return cells.astype(bool)
        pts = np.array([[(x - self.origin[0]) / self.res, (y - self.origin[1]) / self.res]
                        for x, y in poly], np.float32)
        cv2.fillPoly(cells, [np.round(pts).astype(np.int32)], 1)
        return cells.astype(bool)

    def update_slow(self, q, yes):
        name = q['name']
        if yes:
            self.slow_active[name] = 0
        elif name in self.slow_active:
            self.slow_active[name] += 1
            if self.slow_active[name] >= self.speed_clear_cycles:
                del self.slow_active[name]

    def publish_speed(self):
        limits = [float(q['speed_percent']) for q in self.queries
                  if q['name'] in self.slow_active]
        new = min(limits) if limits else 0.0
        if new != self.speed_limit:
            self.get_logger().info(
                f'speed limit -> {"none" if new == 0 else f"{new:.0f}%"}')
        self.speed_limit = new
        msg = SpeedLimit()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = True
        msg.speed_limit = new  # 0.0 = no limit
        self.speed_pub.publish(msg)

    def combined_mask(self) -> np.ndarray:
        now = time.time()
        out = np.zeros((self.n, self.n), np.int8)
        for val, exp in self.layers.values():
            live = np.where(exp > now, val, 0).astype(np.int8)
            np.maximum(out, live, out=out)
        out[out == 0] = -1   # no information: the filter leaves the costmap alone
        return out

    def publish_mask(self):
        if self.origin is None:
            return
        m = OccupancyGrid()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = self.res
        m.info.width = m.info.height = self.n
        m.info.origin.position.x, m.info.origin.position.y = self.origin
        m.info.origin.orientation.w = 1.0
        m.data = self.combined_mask().ravel().tolist()
        self.mask_pub.publish(m)

    def publish_debug(self, img, strips, hits):
        out = img.copy()
        for i, (u0, u1, v0, v1) in enumerate(strips):
            color = (0, 0, 255) if i in hits else (0, 200, 0)
            cv2.rectangle(out, (u0 + 2, v0 + 2), (u1 - 2, v1 - 2), color, 3)
        msg = Image()
        msg.header.frame_id = 'camera'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width = out.shape[:2]
        msg.encoding, msg.step = 'bgr8', out.shape[1] * 3
        msg.data = out.tobytes()
        self.debug_pub.publish(msg)

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # Own signal handling (as in frontier_explorer.py), so the context is
    # still valid when the speed limit is lifted on shutdown.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_interrupt)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = SemanticCostmap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        # Never leave Nav2 throttled by a node that is gone.
        node.slow_active.clear()
        node.publish_speed()
        time.sleep(0.2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
