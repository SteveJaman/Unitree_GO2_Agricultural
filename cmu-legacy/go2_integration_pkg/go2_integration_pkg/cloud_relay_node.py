#!/usr/bin/env python3
"""
cloud_relay_node.py

WebRTC fallback only. Republishes PointCloud2 from the WebRTC SDK's
topic name to the name expected by Point-LIO.

    go2_robot_sdk  ->  /point_cloud2  ->  /utlidar/cloud  ->  Point-LIO

QoS matches the WebRTC SDK: BEST_EFFORT, VOLATILE, shallow history.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import PointCloud2


def sensor_qos(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


class CloudRelayNode(Node):
    def __init__(self) -> None:
        super().__init__('cloud_relay_node')

        self.declare_parameter('input_topic', '/point_cloud2')
        self.declare_parameter('output_topic', '/utlidar/cloud')
        self.declare_parameter('queue_depth', 5)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        depth = self.get_parameter('queue_depth').value

        qos = sensor_qos(depth)

        self._pub = self.create_publisher(PointCloud2, out_topic, qos)
        self._sub = self.create_subscription(
            PointCloud2, in_topic, self._on_cloud, qos)

        self._rx = 0
        self._tx = 0

        self.get_logger().info(
            f'cloud_relay_node up: {in_topic} -> {out_topic} '
            f'(BEST_EFFORT, depth={depth})')

        self.create_timer(10.0, self._report)

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._rx += 1
        self._pub.publish(msg)
        self._tx += 1

    def _report(self) -> None:
        self.get_logger().info(
            f'relayed {self._tx}/{self._rx} clouds')


def main() -> None:
    rclpy.init()
    node = CloudRelayNode()
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