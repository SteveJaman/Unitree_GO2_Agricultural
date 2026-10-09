#!/usr/bin/env python3
"""Reject floor points before they reach the costmap.

Fits a ground plane per frame and removes points within `floor_margin`
of that plane. Prevents tilted-map "floor-as-obstacle" failures.

Subscribes: /lidar3d/registered  (aligned cloud in map frame)
Publishes:  /lidar3d/obstacle_points  (filtered cloud, feeds Nav2)
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points, create_cloud_xyz32


class FloorFilter(Node):
    def __init__(self):
        super().__init__('go2_floor_filter')
        self.declare_parameter('input_topic', '/lidar3d/registered')
        self.declare_parameter('output_topic', '/lidar3d/obstacle_points')
        self.declare_parameter('floor_margin', 0.20)
        self.declare_parameter('plane_search_max_z', 0.5)
        self.declare_parameter('min_plane_inliers', 200)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.margin = float(self.get_parameter('floor_margin').value)
        self.search_max_z = float(self.get_parameter('plane_search_max_z').value)
        self.min_inliers = int(self.get_parameter('min_plane_inliers').value)

        self.pub = self.create_publisher(PointCloud2, out_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            PointCloud2, in_topic, self.on_cloud, qos_profile_sensor_data)
        self.get_logger().info(
            f'floor filter: {in_topic} -> {out_topic}, margin={self.margin} m')

    def on_cloud(self, msg):
        pts = np.asarray(
            [[float(v) for v in p] for p in read_points(
                msg, field_names=['x', 'y', 'z'], skip_nans=True)],
            dtype=np.float64).reshape(-1, 3)
        if len(pts) < 100:
            self.pub.publish(create_cloud_xyz32(msg.header, pts.tolist()))
            return

        z = pts[:, 2]
        near_floor = (z > -self.search_max_z) & (z < self.search_max_z)
        if np.count_nonzero(near_floor) < self.min_inliers:
            self.pub.publish(create_cloud_xyz32(msg.header, pts.tolist()))
            return

        P = pts[near_floor]
        A = np.column_stack([P[:, 0], P[:, 1], np.ones(len(P))])
        coef, *_ = np.linalg.lstsq(A, P[:, 2], rcond=None)
        a, b, c = coef

        plane_z = a * pts[:, 0] + b * pts[:, 1] + c
        floor_dist = np.abs(pts[:, 2] - plane_z)
        keep = floor_dist > self.margin

        self.pub.publish(create_cloud_xyz32(msg.header, pts[keep].tolist()))


def main():
    rclpy.init()
    node = FloorFilter()
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
