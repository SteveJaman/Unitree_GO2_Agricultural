#!/usr/bin/env python3
"""
map_node.py

ROS 2 mapping node with optional RGB fusion and automatic topic detection.

Subscribes:
  cloud_topic (auto)   (sensor_msgs/PointCloud2)  -- aligned point cloud
  odom_topic  (auto)   (nav_msgs/Odometry)         -- 6-DOF pose
  /camera/image/raw    (sensor_msgs/Image)         -- optional RGB
  /camera/camera_info  (sensor_msgs/CameraInfo)    -- camera intrinsics

Publishes:
  /map/occupancy       (nav_msgs/OccupancyGrid)    -- RELIABLE + TRANSIENT_LOCAL
  /map/points          (sensor_msgs/PointCloud2)   -- accumulated cloud

Auto-detection of cloud and odom topics:
  The node searches the graph in priority order and uses the first topic
  that exists. This makes the node work in three environments:

    CMU autonomy stack : /registered_scan, /state_estimation
    Dog direct over DDS: /utlidar/cloud_deskewed, /utlidar/robot_odom
    Legacy CMU naming  : /cloud_registered

  Override any default by passing the parameter explicitly.

Saves on Ctrl+C:
  ~/go2_maps/<timestamp>/
    map.png           -- top-down grayscale occupancy
    map.ply           -- binary PLY (XYZ + optional RGB)
    mesh.obj          -- Poisson-reconstructed mesh
    points.npy        -- raw numpy array
    colors.npy        -- RGB array (if camera fusion was active)
"""

import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

try:
    from cv_bridge import CvBridge
    _HAS_CV_BRIDGE = True
except ImportError:
    _HAS_CV_BRIDGE = False

try:
    import tf2_ros
    _HAS_TF2 = True
except ImportError:
    _HAS_TF2 = False

try:
    from go2_integration_pkg.core.mapping import (
        MapConfig, OccupancyGrid, CloudAccumulator,
        reconstruct_mesh, write_pointcloud_ply)
except ImportError:
    here = Path(__file__).resolve().parent
    sys.path.insert(0, str(here))
    from core.mapping import (
        MapConfig, OccupancyGrid, CloudAccumulator,
        reconstruct_mesh, write_pointcloud_ply)


# ---------------------------------------------------------------------------
# Topic auto-detection candidates (priority order)
# ---------------------------------------------------------------------------

CLOUD_TOPIC_CANDIDATES = [
    '/registered_scan',        # CMU autonomy stack (post-remap)
    '/cloud_registered',       # CMU autonomy stack (pre-remap)
    '/utlidar/cloud_deskewed', # direct from dog, motion-compensated
    '/utlidar/cloud',          # direct from dog, raw
]

ODOM_TOPIC_CANDIDATES = [
    '/state_estimation',       # CMU autonomy stack
    '/utlidar/robot_odom',     # direct from dog
    '/utlidar/robot_pose',     # alternative dog pose topic
]

# Go2 front camera intrinsics (Go2W published config).
# These are overridden by whatever /camera/camera_info publishes.
DEFAULT_FX = 864.0
DEFAULT_FY = 864.0
DEFAULT_CX = 639.2
DEFAULT_CY = 373.3


# ---------------------------------------------------------------------------
# QoS profiles
# ---------------------------------------------------------------------------

def sensor_qos(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def map_qos() -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


# ---------------------------------------------------------------------------
# MapNode
# ---------------------------------------------------------------------------

class MapNode(Node):
    def __init__(self):
        super().__init__('map_node')

        # ---- Parameters ----
        self.declare_parameter('cloud_topic', '')
        self.declare_parameter('odom_topic', '')
        self.declare_parameter('image_topic', '/camera/image/raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('use_camera', False)
        self.declare_parameter('output_dir', str(Path.home() / 'go2_maps'))
        self.declare_parameter('save_interval_s', 0.0)
        self.declare_parameter('save_on_shutdown', True)
        self.declare_parameter('map_size_m', 50.0)
        self.declare_parameter('resolution_m', 0.05)
        self.declare_parameter('publish_map_rate_hz', 2.0)

        cloud_param = self.get_parameter('cloud_topic').value
        odom_param = self.get_parameter('odom_topic').value
        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.use_camera = self.get_parameter('use_camera').value
        self.output_dir = Path(self.get_parameter('output_dir').value)
        self.save_interval = self.get_parameter('save_interval_s').value
        self.save_on_shutdown = self.get_parameter('save_on_shutdown').value
        pub_rate = self.get_parameter('publish_map_rate_hz').value

        # ---- Resolve topics (auto-detect if empty) ----
        if cloud_param:
            cloud_topic = cloud_param
            self.get_logger().info(
                f'cloud_topic set explicitly to {cloud_topic}')
        else:
            cloud_topic = self._pick_first_available(
                CLOUD_TOPIC_CANDIDATES)
            if cloud_topic is None:
                self.get_logger().fatal(
                    'No cloud topic found. Tried: '
                    + ', '.join(CLOUD_TOPIC_CANDIDATES))
                raise RuntimeError('No cloud topic available')
            self.get_logger().info(
                f'cloud_topic auto-detected: {cloud_topic}')

        if odom_param:
            odom_topic = odom_param
            self.get_logger().info(
                f'odom_topic set explicitly to {odom_topic}')
        else:
            odom_topic = self._pick_first_available(
                ODOM_TOPIC_CANDIDATES)
            if odom_topic is None:
                self.get_logger().warn(
                    'No odom topic found. Pose will stay at origin. '
                    'Tried: ' + ', '.join(ODOM_TOPIC_CANDIDATES))
                odom_topic = '/state_estimation'
            else:
                self.get_logger().info(
                    f'odom_topic auto-detected: {odom_topic}')

        # ---- Validate optional dependencies ----
        if self.use_camera:
            if not _HAS_CV_BRIDGE:
                self.get_logger().error(
                    'use_camera=true but cv_bridge missing. '
                    'Disabling camera.')
                self.use_camera = False
            elif not _HAS_TF2:
                self.get_logger().error(
                    'use_camera=true but tf2_ros missing. '
                    'Disabling camera.')
                self.use_camera = False

        # ---- Core ----
        cfg = MapConfig(
            size_m=self.get_parameter('map_size_m').value,
            resolution_m=self.get_parameter('resolution_m').value,
        )
        self.grid = OccupancyGrid(cfg)
        self.cloud = CloudAccumulator(cfg)
        self.bridge = CvBridge() if self.use_camera else None

        # ---- State ----
        self.pose_xy = (0.0, 0.0)
        self.pose_yaw = 0.0
        self.last_cloud_time = None
        self.cloud_count = 0

        self.camera_info = None
        self.last_image = None

        # ---- Publishers ----
        self.map_pub = self.create_publisher(
            OccupancyGrid, '/map/occupancy', map_qos())
        self.points_pub = self.create_publisher(
            PointCloud2, '/map/points', sensor_qos(1))

        # ---- Subscribers ----
        self.cloud_sub = self.create_subscription(
            PointCloud2, cloud_topic, self.on_cloud, sensor_qos(5))
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.on_odom, 10)

        if self.use_camera:
            self.image_sub = self.create_subscription(
                Image, image_topic, self.on_image, sensor_qos(2))
            self.caminfo_sub = self.create_subscription(
                CameraInfo, camera_info_topic, self.on_camera_info, 10)

        # ---- TF ----
        if self.use_camera:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(
                self.tf_buffer, self)

        # ---- Timers ----
        self.create_timer(1.0 / pub_rate, self.publish_map)

        # ---- Session directory ----
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.session_dir = self.output_dir / stamp
        self.session_dir.mkdir(parents=True, exist_ok=True)

        if self.save_interval > 0:
            self.create_timer(self.save_interval, self.save_map)

        # ---- Startup banner ----
        self.get_logger().info('=' * 64)
        self.get_logger().info(' map_node started')
        self.get_logger().info(f'   cloud in : {cloud_topic}')
        self.get_logger().info(f'   odom in  : {odom_topic}')
        if self.use_camera:
            self.get_logger().info(f'   image in : {image_topic}')
            self.get_logger().info(f'   caminfo  : {camera_info_topic}')
        else:
            self.get_logger().info('   RGB fusion : disabled')
        self.get_logger().info('   map out  : /map/occupancy (TRANSIENT_LOCAL)')
        self.get_logger().info('   pts out  : /map/points (BEST_EFFORT)')
        self.get_logger().info(f'   save dir : {self.session_dir}')
        self.get_logger().info('=' * 64)

    # ------------------------------------------------------------------
    def _pick_first_available(self, candidates):
        """Return the first candidate topic that exists in the graph."""
        topics = self.get_topic_names_and_types()
        names = {name for name, _ in topics}
        for c in candidates:
            if c in names:
                return c
        return None

    # ------------------------------------------------------------------
    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.pose_yaw = float(np.arctan2(siny, cosy))
        self.pose_xy = (float(p.x), float(p.y))

    # ------------------------------------------------------------------
    def on_camera_info(self, msg: CameraInfo) -> None:
        if self.camera_info is None:
            self.get_logger().info(
                f'Camera info: {msg.width}x{msg.height}, '
                f'fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}')
        self.camera_info = msg

    # ------------------------------------------------------------------
    def on_image(self, msg: Image) -> None:
        if not self.use_camera:
            return
        try:
            self.last_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image decode failed: {e}')

    # ------------------------------------------------------------------
    def on_cloud(self, msg: PointCloud2) -> None:
        pts = self._cloud_to_array(msg)
        if pts is None or len(pts) == 0:
            return

        self.grid.add_scan(self.pose_xy, pts)

        colors = None
        if self.use_camera:
            colors = self._colorize_points(msg.header, pts)

        self.cloud.add(pts, colors)

        self.cloud_count += 1
        now = self.get_clock().now()
        if self.last_cloud_time is None or \
                (now - self.last_cloud_time).nanoseconds > 5_000_000_000:
            self.last_cloud_time = now
            self.get_logger().info(
                f'cloud #{self.cloud_count}: '
                f'{len(pts)} pts, '
                f'{self.cloud.count} total, '
                f'{"colored" if colors is not None else "grey"}')

    # ------------------------------------------------------------------
    def _colorize_points(self, header: Header, pts: np.ndarray):
        if self.last_image is None or self.camera_info is None:
            return None

        target_frame = self.camera_info.header.frame_id or \
            'camera_color_optical_frame'

        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                target_frame,
                header.frame_id,
                rclpy.time.Time.from_msg(header.stamp),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except Exception:
            return None

        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        T = self._quat_trans_to_matrix(
            q.x, q.y, q.z, q.w, t.x, t.y, t.z)

        pts_h = np.hstack([pts, np.ones((len(pts), 1))])
        pts_cam = (T @ pts_h.T).T[:, :3]

        valid_z = pts_cam[:, 2] > 0.1
        if not np.any(valid_z):
            return None

        idx = np.where(valid_z)[0]
        pts_cam = pts_cam[idx]

        K = self.camera_info.k
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]

        u = (fx * pts_cam[:, 0] / pts_cam[:, 2]) + cx
        v = (fy * pts_cam[:, 1] / pts_cam[:, 2]) + cy

        h, w = self.last_image.shape[:2]
        in_bounds = (u >= 0) & (u < w) & (v >= 0) & (v < h)

        rgb = np.full((len(pts), 3), 0.5, dtype=np.float32)

        ui = u[in_bounds].astype(int)
        vi = v[in_bounds].astype(int)

        if len(ui) > 0:
            bgr = self.last_image[vi, ui]
            rgb[idx[in_bounds]] = bgr[:, ::-1] / 255.0

        return rgb

    # ------------------------------------------------------------------
    @staticmethod
    def _quat_trans_to_matrix(qx, qy, qz, qw, tx, ty, tz) -> np.ndarray:
        n = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
        ])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [tx, ty, tz]
        return T

    # ------------------------------------------------------------------
    def _cloud_to_array(self, msg: PointCloud2):
        fields = {f.name: f.offset for f in msg.fields}
        if not all(k in fields for k in ('x', 'y', 'z')):
            return None
        point_step = msg.point_step
        n = msg.width * msg.height
        if n == 0:
            return None
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        raw = raw[:n * point_step].reshape(n, point_step)

        def extract(offset):
            return raw[:, offset:offset + 4].copy().view(
                np.float32).reshape(n)

        return np.column_stack([
            extract(fields['x']),
            extract(fields['y']),
            extract(fields['z']),
        ]).astype(np.float32)

    # ------------------------------------------------------------------
    def publish_map(self) -> None:
        # --- OccupancyGrid ---
        data = self.grid.to_occupancy_grid_data()
        n = data.shape[0]

        og = OccupancyGrid()
        og.header.stamp = self.get_clock().now().to_msg()
        og.header.frame_id = 'map'
        og.info.resolution = self.grid.config.resolution_m
        og.info.width = n
        og.info.height = n
        og.info.origin = Pose()
        og.info.origin.position.x = -self.grid.config.size_m / 2.0
        og.info.origin.position.y = -self.grid.config.size_m / 2.0
        og.info.origin.position.z = 0.0
        og.info.origin.orientation.w = 1.0
        og.data = np.flipud(data).flatten().tolist()
        self.map_pub.publish(og)

        # --- Accumulated cloud ---
        pts = self.cloud.points()
        if len(pts) == 0:
            return

        cols = self.cloud.colors() if self.cloud.has_color() else None

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.height = 1
        msg.width = len(pts)

        fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]

        if cols is not None and len(cols) == len(pts):
            fields.append(PointField(name='rgb', offset=12,
                                     datatype=PointField.FLOAT32, count=1))
            msg.point_step = 16
            r = (np.clip(cols[:, 0], 0, 1) * 255).astype(np.uint32)
            g = (np.clip(cols[:, 1], 0, 1) * 255).astype(np.uint32)
            b = (np.clip(cols[:, 2], 0, 1) * 255).astype(np.uint32)
            packed = ((r << 16) | (g << 8) | b).astype(np.uint32)
            packed_f = packed.view(np.float32)
            buf = np.hstack([
                pts.astype(np.float32),
                packed_f.reshape(-1, 1),
            ]).astype(np.float32)
            msg.data = buf.tobytes()
        else:
            msg.point_step = 12
            msg.data = pts.astype(np.float32).tobytes()

        msg.row_step = msg.point_step * len(pts)
        msg.fields = fields
        msg.is_dense = True
        self.points_pub.publish(msg)

    # ------------------------------------------------------------------
    def save_map(self) -> None:
        pts = self.cloud.points()
        if len(pts) == 0:
            self.get_logger().warn('No points accumulated; skipping save')
            return

        cols = self.cloud.colors() if self.cloud.has_color() else None

        self.grid.save_png(self.session_dir / 'map.png')
        write_pointcloud_ply(self.session_dir / 'map.ply', pts, cols)
        np.save(self.session_dir / 'points.npy', pts)
        if cols is not None:
            np.save(self.session_dir / 'colors.npy', cols)

        self.get_logger().info(
            f'Reconstructing mesh from {len(pts)} points...')
        t0 = time.time()
        mesh = reconstruct_mesh(
            pts, cols, self.grid.config,
            output_path=self.session_dir / 'mesh.obj')
        dt = time.time() - t0

        stats = self.grid.stats
        self.get_logger().info('=' * 64)
        self.get_logger().info(f' Map saved: {self.session_dir}')
        self.get_logger().info(f'   points          : {len(pts)}')
        self.get_logger().info(
            f'   coloured        : {"yes" if cols is not None else "no"}')
        self.get_logger().info(
            f'   occupied cells  : {stats["occupied_cells"]}')
        self.get_logger().info(
            f'   free cells      : {stats["free_cells"]}')
        if mesh is not None:
            self.get_logger().info(
                f'   mesh vertices   : {len(mesh.vertices)}')
            self.get_logger().info(
                f'   mesh triangles  : {len(mesh.triangles)}')
        self.get_logger().info(f'   reconstruction  : {dt:.1f} s')
        self.get_logger().info('=' * 64)

    # ------------------------------------------------------------------
    def destroy_node(self):
        if self.save_on_shutdown:
            try:
                self.save_map()
            except Exception as e:
                self.get_logger().error(f'Failed to save map: {e}')
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapNode()
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