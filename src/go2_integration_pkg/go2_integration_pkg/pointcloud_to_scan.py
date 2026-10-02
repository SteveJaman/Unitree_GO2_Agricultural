#!/usr/bin/env python3
"""
pointcloud_to_scan.py

Converts a 3D PointCloud2 into a 2D LaserScan for SLAM and localisation.

This is the standard bridge used by every Go2 SLAM setup. It takes the
LiDAR point cloud, filters by height, and projects onto the XY plane
to produce a horizontal scan.

Subscribes:
  /registered_scan   (sensor_msgs/PointCloud2)

Publishes:
  /scan              (sensor_msgs/LaserScan)
"""

import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import PointCloud2, LaserScan


def sensor_qos(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class PointCloudToScan(Node):
    def __init__(self):
        super().__init__('pointcloud_to_scan')

        self.declare_parameter('input_topic', '/registered_scan')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('min_height', -0.3)
        self.declare_parameter('max_height', 1.5)
        self.declare_parameter('angle_min', -3.14159)
        self.declare_parameter('angle_max', 3.14159)
        self.declare_parameter('angle_increment', 0.0087)   # ~0.5 deg
        self.declare_parameter('range_min', 0.2)
        self.declare_parameter('range_max', 30.0)
        self.declare_parameter('target_frame', 'body')

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.min_h = self.get_parameter('min_height').value
        self.max_h = self.get_parameter('max_height').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_inc = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value

        self.n_bins = int((self.angle_max - self.angle_min) / self.angle_inc)

        self.scan_pub = self.create_publisher(LaserScan, out_topic, sensor_qos())
        self.cloud_sub = self.create_subscription(
            PointCloud2, in_topic, self.on_cloud, sensor_qos(5))

        self.get_logger().info(
            f'pointcloud_to_scan: {in_topic} -> {out_topic} '
            f'({self.n_bins} bins, z=[{self.min_h}, {self.max_h}])')

    def on_cloud(self, msg: PointCloud2) -> None:
        pts = self._cloud_to_array(msg)
        if pts is None or len(pts) == 0:
            return

        # Filter by height
        z = pts[:, 2]
        mask = (z >= self.min_h) & (z <= self.max_h)
        pts = pts[mask]
        if len(pts) == 0:
            return

        # Polar coordinates
        x = pts[:, 0]
        y = pts[:, 1]
        r = np.hypot(x, y)
        theta = np.arctan2(y, x)

        # Filter by range
        valid = (r >= self.range_min) & (r <= self.range_max)
        r = r[valid]
        theta = theta[valid]
        if len(r) == 0:
            return

        # Bin into LaserScan
        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = self.get_parameter('target_frame').value
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = [float('inf')] * self.n_bins

        # Fill bins with nearest point
        indices = ((theta - self.angle_min) / self.angle_inc).astype(int)
        valid_idx = (indices >= 0) & (indices < self.n_bins)
        indices = indices[valid_idx]
        r = r[valid_idx]

        # Vectorized minimum per bin
        for i, dist in zip(indices, r):
            if dist < scan.ranges[i]:
                scan.ranges[i] = float(dist)

        self.scan_pub.publish(scan)

    def _cloud_to_array(self, msg: PointCloud2):
        fields = {f.name: f.offset for f in msg.fields}
        if not all(k in fields for k in ('x', 'y', 'z')):
            return None
        step = msg.point_step
        n = msg.width * msg.height
        if n == 0:
            return None
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        raw = raw[:n * step].reshape(n, step)

        def extract(offset):
            return raw[:, offset:offset+4].copy().view(np.float32).reshape(n)

        return np.column_stack([
            extract(fields['x']),
            extract(fields['y']),
            extract(fields['z']),
        ]).astype(np.float32)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToScan()
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