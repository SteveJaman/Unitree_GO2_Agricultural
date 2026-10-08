#!/usr/bin/env python3
"""Publish fake PointCloud2, Odometry, and TF for testing live_mapping.sh."""

import math
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class MockSim(Node):
    def __init__(self):
        super().__init__('mock_sim')
        self.cloud_pub = self.create_publisher(PointCloud2, '/registered_scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/state_estimation', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.t = 0.0
        self.create_timer(0.1, self.tick)  # 10 Hz
        self.get_logger().info('mock_sim: publishing /registered_scan, /state_estimation, /tf')

    def tick(self):
        self.t += 0.1
        # Move in a circle, radius 3m
        x = 3.0 * math.cos(0.2 * self.t)
        y = 3.0 * math.sin(0.2 * self.t)
        yaw = 0.2 * self.t + math.pi / 2

        # Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'body'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(yaw / 2)
        odom.pose.pose.orientation.w = math.cos(yaw / 2)
        self.odom_pub.publish(odom)

        # Point cloud: ring + 4 walls
        pts = []
        for i in range(180):
            a = math.radians(i * 2)
            pts.append((x + 4.0 * math.cos(a), y + 4.0 * math.sin(a), 0.5))
        for i in range(0, 100):
            t = i / 100.0 * 4
            if t < 1:   px, py = -10 + 20 * t, -10
            elif t < 2: px, py = 10, -10 + 20 * (t - 1)
            elif t < 3: px, py = 10 - 20 * (t - 2), 10
            else:       px, py = -10, 10 - 20 * (t - 3)
            pts.append((px, py, 0.5))

        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = 'map'
        cloud.height = 1
        cloud.width = len(pts)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = 12 * len(pts)
        cloud.is_dense = True
        cloud.data = b''.join(struct.pack('fff', *p) for p in pts)
        self.cloud_pub.publish(cloud)

        # TF: map -> body -> base_link
        tf1 = TransformStamped()
        tf1.header.stamp = self.get_clock().now().to_msg()
        tf1.header.frame_id = 'map'
        tf1.child_frame_id = 'body'
        tf1.transform.translation.x = x
        tf1.transform.translation.y = y
        tf1.transform.rotation.z = math.sin(yaw / 2)
        tf1.transform.rotation.w = math.cos(yaw / 2)
        self.tf_broadcaster.sendTransform(tf1)

        tf2 = TransformStamped()
        tf2.header.stamp = self.get_clock().now().to_msg()
        tf2.header.frame_id = 'body'
        tf2.child_frame_id = 'base_link'
        tf2.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf2)


def main():
    rclpy.init()
    node = MockSim()
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
