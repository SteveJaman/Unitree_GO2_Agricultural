#!/usr/bin/env python3
"""
map_node.py

ROS 2 mapping node. Consumes Point-LIO output for aligned SLAM quality.

Subscribes:
  /cloud_registered   (sensor_msgs/PointCloud2)  -- aligned, deskewed cloud
  /state_estimation   (nav_msgs/Odometry)         -- 6-DOF pose from Point-LIO
  /camera/image/raw   (sensor_msgs/Image)         -- optional, for RGB fusion

Publishes:
  /map/occupancy      (nav_msgs/OccupancyGrid)    -- for RViz Map display
  /map/points         (sensor_msgs/PointCloud2)   -- accumulated cloud

Saves on Ctrl+C:
  ~/go2_maps/<timestamp>/
    map.png           -- top-down grayscale occupancy
    map.ply           -- binary PLY (XYZ + optional RGB)
    mesh.obj          -- Poisson-reconstructed mesh
    points.npy        -- raw numpy array

Topic QoS contract:
  /map/occupancy  -> RELIABLE + TRANSIENT_LOCAL  (RViz requires this to
                     render late-joining subscribers)
  /map/points     -> BEST_EFFORT + VOLATILE      (large cloud, no queue)
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

from sensor_msgs.msg import PointCloud2, PointField, Image
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose

try:
    from cv_bridge import CvBridge
    _HAS_CV_BRIDGE = True
except ImportError:
    _HAS_CV_BRIDGE = False

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
# QoS profiles
# ---------------------------------------------------------------------------

def sensor_qos(depth: int = 5) -> QoSProfile:
    """BEST_EFFORT, VOLATILE. Use for sensor streams and large clouds."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def map_qos() -> QoSProfile:
    """
    RELIABLE, TRANSIENT_LOCAL. Use for the occupancy grid so RViz can
    render it even when it subscribes AFTER the publisher started.

    Without TRANSIENT_LOCAL, RViz shows 'No map received' even though
    the topic exists and is publishing.
    """
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
        self.declare_parameter('cloud_topic', '/cloud_registered')
        self.declare_parameter('odom_topic', '/state_estimation')
        self.declare_parameter('image_topic', '/camera/image/raw')
        self.declare_parameter('use_camera', False)
        self.declare_parameter('output_dir', str(Path.home() / 'go2_maps'))
        self.declare_parameter('save_interval_s', 0.0)
        self.declare_parameter('save_on_shutdown', True)
        self.declare_parameter('map_size_m', 50.0)
        self.declare_parameter('resolution_m', 0.05)
        self.declare_parameter('publish_map_rate_hz', 2.0)

        cloud_topic = self.get_parameter('cloud_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        image_topic = self.get_parameter('image_topic').value
        self.use_camera = self.get_parameter('use_camera').value
        self.output_dir = Path(self.get_parameter('output_dir').value)
        self.save_interval = self.get_parameter('save_interval_s').value
        self.save_on_shutdown = self.get_parameter('save_on_shutdown').value
        pub_rate = self.get_parameter('publish_map_rate_hz').value

        if self.use_camera and not _HAS_CV_BRIDGE:
            self.get_logger().warn(
                'use_camera=true but cv_bridge is not installed. '
                'Disabling camera fusion.')
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
        self.get_logger().info('   map out  : /map/occupancy (TRANSIENT_LOCAL)')
        self.get_logger().info('   pts out  : /map/points (BEST_EFFORT)')
        self.get_logger().info(f'   save dir : {self.session_dir}')
        self.get_logger().info('=' * 64)

    # ------------------------------------------------------------------
    def on_odom(self, msg: Odometry) -> None:
        """Track robot pose from Point-LIO."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # Yaw from quaternion (rotation about Z)
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.pose_yaw = float(np.arctan2(siny, cosy))
        self.pose_xy = (float(p.x), float(p.y))

    # ------------------------------------------------------------------
    def on_cloud(self, msg: PointCloud2) -> None:
        pts = self._cloud_to_array(msg)
        if pts is None or len(pts) == 0:
            return

        # Points are already in map frame (Point-LIO output).
        # Update occupancy grid with the sensor at the current robot pose.
        self.grid.add_scan(self.pose_xy, pts)
        self.cloud.add(pts)

        self.cloud_count += 1
        now = self.get_clock().now()
        if self.last_cloud_time is None or \
                (now - self.last_cloud_time).nanoseconds > 5_000_000_000:
            self.last_cloud_time = now
            self.get_logger().info(
                f'cloud #{self.cloud_count}: '
                f'{len(pts)} pts this scan, '
                f'{self.cloud.count} total')

    # ------------------------------------------------------------------
    def on_image(self, msg: Image) -> None:
        """
        RGB fusion placeholder.

        Wired in a later revision: project LiDAR points into the camera
        frame, look up pixel color, attach to the cloud. Requires
        camera intrinsics and TF between camera and LiDAR.
        """
        pass

    # ------------------------------------------------------------------
    def _cloud_to_array(self, msg: PointCloud2):
        """Convert a PointCloud2 into an Nx3 float32 numpy array."""
        fields = {f.name: f.offset for f in msg.fields}
        if not all(k in fields for k in ('x', 'y', 'z')):
            return None

        point_step = msg.point_step
        n = msg.width * msg.height
        if n == 0:
            return None

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        raw = raw[:n * point_step].reshape(n, point_step)

        def extract(offset: int) -> np.ndarray:
            return raw[:, offset:offset + 4].copy().view(np.float32).reshape(n)

        return np.column_stack([
            extract(fields['x']),
            extract(fields['y']),
            extract(fields['z']),
        ]).astype(np.float32)

    # ------------------------------------------------------------------
    def publish_map(self) -> None:
        """Publish /map/occupancy and /map/points for RViz."""
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
        # RViz expects rows bottom-up; our array is top-down
        og.data = np.flipud(data).flatten().tolist()
        self.map_pub.publish(og)

        # --- Accumulated cloud ---
        pts = self.cloud.points()
        if len(pts) == 0:
            return

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.height = 1
        msg.width = len(pts)
        msg.fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = 12 * len(pts)
        msg.is_dense = True
        msg.data = pts.astype(np.float32).tobytes()
        self.points_pub.publish(msg)

    # ------------------------------------------------------------------
    def save_map(self) -> None:
        """Export map, cloud, mesh, and raw arrays."""
        pts = self.cloud.points()
        if len(pts) == 0:
            self.get_logger().warn('No points accumulated; skipping save')
            return

        cols = self.cloud.colors() if self.cloud.has_color() else None

        # Top-down grayscale
        self.grid.save_png(self.session_dir / 'map.png')

        # Binary PLY
        write_pointcloud_ply(self.session_dir / 'map.ply', pts, cols)

        # Raw numpy arrays (fast reload for later analysis)
        np.save(self.session_dir / 'points.npy', pts)
        if cols is not None:
            np.save(self.session_dir / 'colors.npy', cols)

        # Poisson mesh reconstruction (slow — runs only on save)
        self.get_logger().info(
            f'Reconstructing mesh from {len(pts)} points '
            f'(may take a minute)...')
        t0 = time.time()
        mesh = reconstruct_mesh(
            pts, cols, self.grid.config,
            output_path=self.session_dir / 'mesh.obj')
        dt = time.time() - t0

        # Summary
        stats = self.grid.stats
        self.get_logger().info('=' * 64)
        self.get_logger().info(f' Map saved: {self.session_dir}')
        self.get_logger().info(
            f'   points          : {len(pts)}')
        self.get_logger().info(
            f'   occupied cells  : {stats["occupied_cells"]}')
        self.get_logger().info(
            f'   free cells      : {stats["free_cells"]}')
        self.get_logger().info(
            f'   unknown cells   : {stats["unknown_cells"]}')
        if mesh is not None:
            self.get_logger().info(
                f'   mesh vertices   : {len(mesh.vertices)}')
            self.get_logger().info(
                f'   mesh triangles  : {len(mesh.triangles)}')
        self.get_logger().info(
            f'   reconstruction  : {dt:.1f} s')
        self.get_logger().info('=' * 64)

    # ------------------------------------------------------------------
    def destroy_node(self):
        if self.save_on_shutdown:
            try:
                self.save_map()
            except Exception as e:
                self.get_logger().error(f'Failed to save map: {e}')
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

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