#!/usr/bin/env python3
"""
map_node.py

ROS 2 mapping node with optional RGB fusion.

Subscribes:
  /registered_scan     (sensor_msgs/PointCloud2)  -- aligned cloud from Point-LIO
  /state_estimation    (nav_msgs/Odometry)         -- 6-DOF pose
  /camera/image/raw    (sensor_msgs/Image)         -- optional RGB
  /camera/camera_info  (sensor_msgs/CameraInfo)    -- camera intrinsics

Publishes:
  /map/occupancy       (nav_msgs/OccupancyGrid)    -- RELIABLE + TRANSIENT_LOCAL
  /map/points          (sensor_msgs/PointCloud2)   -- accumulated cloud

RGB fusion pipeline:
  1. Receive a cloud and an image, both with timestamps.
  2. Look up TF transform: map -> camera_color_optical_frame.
  3. Transform the cloud points into the camera frame.
  4. Project each point: u = fx*X/Z + cx, v = fy*Y/Z + cy.
  5. Discard points behind the camera (Z <= 0) or outside image bounds.
  6. Sample the image at (u, v) to get RGB.
  7. Attach RGB to the accumulated cloud.

Camera intrinsics and extrinsics are hard-coded for the Go2 front camera
based on the published Unitree config. Override via parameters if needed.
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

# Optional: cv_bridge for image decoding
try:
    from cv_bridge import CvBridge
    _HAS_CV_BRIDGE = True
except ImportError:
    _HAS_CV_BRIDGE = False

# Optional: tf2 for coordinate transforms
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
# Go2 front camera defaults (from Unitree published config)
# ---------------------------------------------------------------------------

# Intrinsics for the Go2W front camera (default resolution)
DEFAULT_FX = 864.0
DEFAULT_FY = 864.0
DEFAULT_CX = 639.2
DEFAULT_CY = 373.3
DEFAULT_DIST = [-0.354630, 0.102054, -0.001614, -0.001249, 0.0]

# Extrinsic: LiDAR -> camera optical frame (4x4 homogeneous)
# Row-major. From the published Go2 config.
DEFAULT_LIDAR_TO_CAMERA = np.array([
    [0.0, -1.0,  0.0,  0.00],
    [0.0,  0.0, -1.0, -0.05],
    [1.0,  0.0,  0.0, -0.32],
    [0.0,  0.0,  0.0,  1.00],
], dtype=np.float64)


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
        self.declare_parameter('cloud_topic', '/registered_scan')
        self.declare_parameter('odom_topic', '/state_estimation')
        self.declare_parameter('image_topic', '/camera/image/raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('use_camera', False)
        self.declare_parameter('output_dir', str(Path.home() / 'go2_maps'))
        self.declare_parameter('save_interval_s', 0.0)
        self.declare_parameter('save_on_shutdown', True)
        self.declare_parameter('map_size_m', 50.0)
        self.declare_parameter('resolution_m', 0.05)
        self.declare_parameter('publish_map_rate_hz', 2.0)
        self.declare_parameter('camera_sync_tolerance_s', 0.05)

        cloud_topic = self.get_parameter('cloud_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.use_camera = self.get_parameter('use_camera').value
        self.output_dir = Path(self.get_parameter('output_dir').value)
        self.save_interval = self.get_parameter('save_interval_s').value
        self.save_on_shutdown = self.get_parameter('save_on_shutdown').value
        pub_rate = self.get_parameter('publish_map_rate_hz').value
        self.sync_tol = self.get_parameter('camera_sync_tolerance_s').value

        # ---- Validate dependencies ----
        if self.use_camera:
            if not _HAS_CV_BRIDGE:
                self.get_logger().error(
                    'use_camera=true but cv_bridge is not installed. '
                    'Install: sudo apt install ros-humble-cv-bridge')
                self.use_camera = False
            elif not _HAS_TF2:
                self.get_logger().error(
                    'use_camera=true but tf2_ros is not available. '
                    'Install: sudo apt install ros-humble-tf2-ros')
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

        # Camera state
        self.camera_info = None       # sensor_msgs/CameraInfo
        self.last_image = None        # decoded numpy array (HxWx3, BGR)
        self.last_image_stamp = None  # rclpy Time

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

        # ---- Session ----
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
            self.get_logger().info(
                f'   image in : {image_topic}')
            self.get_logger().info(
                f'   caminfo  : {camera_info_topic}')
        else:
            self.get_logger().info('   RGB fusion : disabled')
        self.get_logger().info('   map out  : /map/occupancy (TRANSIENT_LOCAL)')
        self.get_logger().info('   pts out  : /map/points (BEST_EFFORT)')
        self.get_logger().info(f'   save dir : {self.session_dir}')
        self.get_logger().info('=' * 64)

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
        """Store camera intrinsics for projection."""
        if self.camera_info is None:
            self.get_logger().info(
                f'Camera info received: {msg.width}x{msg.height}, '
                f'fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}, '
                f'cx={msg.k[2]:.1f}, cy={msg.k[5]:.1f}')
        self.camera_info = msg

    # ------------------------------------------------------------------
    def on_image(self, msg: Image) -> None:
        """Decode and cache the latest image."""
        if not self.use_camera:
            return
        try:
            # Decode to BGR uint8 numpy array
            self.last_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
            self.last_image_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().warn(f'Failed to decode image: {e}')

    # ------------------------------------------------------------------
    def on_cloud(self, msg: PointCloud2) -> None:
        pts = self._cloud_to_array(msg)
        if pts is None or len(pts) == 0:
            return

        # Update occupancy grid
        self.grid.add_scan(self.pose_xy, pts)

        # Try to colour the points
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
                f'{len(pts)} pts this scan, '
                f'{self.cloud.count} total, '
                f'{"colored" if colors is not None else "greyscale"}')

    # ------------------------------------------------------------------
    def _colorize_points(self, header: Header, pts: np.ndarray) -> np.ndarray:
        """
        Project points into the camera image and return Nx3 RGB in [0, 1].

        Returns None if anything is missing (no image, no camera info,
        no TF). In that case the points stay greyscale.
        """
        # --- Prerequisites ---
        if self.last_image is None:
            return None
        if self.camera_info is None:
            return None

        # --- Time sync check ---
        if header.stamp.sec == 0 and header.stamp.nanosec == 0:
            return None

        # --- Look up transform map -> camera ---
        target_frame = self.camera_info.header.frame_id
        if not target_frame:
            target_frame = 'camera_color_optical_frame'

        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                target_frame,
                header.frame_id,
                rclpy.time.Time.from_msg(header.stamp),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except Exception as e:
            self.get_logger().debug(f'TF lookup failed: {e}')
            return None

        # --- Build 4x4 transform matrix from TF ---
        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        T = self._quat_trans_to_matrix(
            q.x, q.y, q.z, q.w, t.x, t.y, t.z)

        # --- Transform points into camera frame ---
        pts_h = np.hstack([pts, np.ones((len(pts), 1))])
        pts_cam = (T @ pts_h.T).T[:, :3]

        # Filter points in front of camera
        valid_z = pts_cam[:, 2] > 0.1
        if not np.any(valid_z):
            return None

        idx = np.where(valid_z)[0]
        pts_cam = pts_cam[idx]

        # --- Intrinsics ---
        K = self.camera_info.k
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]

        # --- Project ---
        u = (fx * pts_cam[:, 0] / pts_cam[:, 2]) + cx
        v = (fy * pts_cam[:, 1] / pts_cam[:, 2]) + cy

        # --- Bounds check ---
        h, w = self.last_image.shape[:2]
        in_bounds = (u >= 0) & (u < w) & (v >= 0) & (v < h)

        # --- Sample colours ---
        rgb = np.zeros((len(pts), 3), dtype=np.float32)
        rgb[:, :] = 0.5   # default grey

        ui = u[in_bounds].astype(int)
        vi = v[in_bounds].astype(int)

        if len(ui) > 0:
            # cv_bridge returns BGR; convert to RGB and normalise
            bgr = self.last_image[vi, ui]
            rgb[idx[in_bounds]] = bgr[:, ::-1] / 255.0

        return rgb

    # ------------------------------------------------------------------
    @staticmethod
    def _quat_trans_to_matrix(qx, qy, qz, qw, tx, ty, tz) -> np.ndarray:
        """Build a 4x4 homogeneous transform from a quaternion + translation."""
        # Normalise
        n = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n

        # Rotation matrix from quaternion
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
            return raw[:, offset:offset + 4].copy().view(np.float32).reshape(n)

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

        # --- Accumulated cloud (with RGB if available) ---
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
            # Pack RGB as a single float (PCL convention) for RViz
            fields.append(
                PointField(name='rgb', offset=12,
                           datatype=PointField.FLOAT32, count=1))
            msg.point_step = 16
            packed = np.zeros(len(pts), dtype=np.float32)
            r = (np.clip(cols[:, 0], 0, 1) * 255).astype(np.uint32)
            g = (np.clip(cols[:, 1], 0, 1) * 255).astype(np.uint32)
            b = (np.clip(cols[:, 2], 0, 1) * 255).astype(np.uint32)
            packed = (r << 16) | (g << 8) | b
            packed = packed.view(np.float32)
            buf = np.hstack([
                pts.astype(np.float32),
                packed.reshape(-1, 1),
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

        # Top-down grayscale
        self.grid.save_png(self.session_dir / 'map.png')

        # Binary PLY with RGB if available
        write_pointcloud_ply(self.session_dir / 'map.ply', pts, cols)
        np.save(self.session_dir / 'points.npy', pts)
        if cols is not None:
            np.save(self.session_dir / 'colors.npy', cols)

        # Poisson mesh
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