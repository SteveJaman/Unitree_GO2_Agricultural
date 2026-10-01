#!/usr/bin/env python3
"""
cloud_relay_node.py

Bridges the Go2 WebRTC driver's point cloud topic to the topic name expected
by the CMU autonomy stack's Point-LIO module.

    go2_robot_sdk  ──►  /point_cloud2  ──►  [this node]  ──►  /utlidar/cloud
                                                                     │
                                                                     ▼
                                                              point_lio_unilidar

QoS is the critical detail here:
    - The WebRTC SDK publishes PointCloud2 with a BEST_EFFORT / VOLATILE
      profile (SensorDataQoS-like).
    - Point-LIO's subscription also uses SensorDataQoS.
    - A RELIABLE subscriber cannot read from a BEST_EFFORT publisher, so we
      must match the publisher side (BEST_EFFORT) on our input.
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


# ---------------------------------------------------------------------------
# QoS matching the WebRTC driver's publisher (best-effort, volatile, shallow)
# ---------------------------------------------------------------------------
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class CloudRelayNode(Node):
    """Relays PointCloud2 from /point_cloud2 to /utlidar/cloud."""

    def __init__(self) -> None:
        super().__init__('cloud_relay_node')

        # Parameters let you override topic names without editing code.
        self.declare_parameter('input_topic',  '/point_cloud2')
        self.declare_parameter('output_topic', '/utlidar/cloud')
        self.declare_parameter('queue_depth',  5)

        in_topic  = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        depth     = self.get_parameter('queue_depth').value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=depth,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Publisher on the autonomy-stack side. We publish BEST_EFFORT too,
        # because Point-LIO subscribes BEST_EFFORT and will not accept a
        # RELIABLE publisher's stream cleanly.
        self._pub = self.create_publisher(PointCloud2, out_topic, qos)

        self._sub = self.create_subscription(
            PointCloud2,
            in_topic,
            self._on_cloud,
            qos,
        )

        # Lightweight counters for health monitoring.
        self._rx_count = 0
        self._tx_count = 0

        self.get_logger().info(
            f'cloud_relay_node up: {in_topic} -> {out_topic} '
            f'(BEST_EFFORT, depth={depth})'
        )

        # Report stats every 10 s so you can spot a silent stream.
        self.create_timer(10.0, self._report_stats)

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._rx_count += 1
        # PointCloud2 is already a complete, serialized message; we can
        # republish it directly. Zero copy on the DDS side is handled by
        # CycloneDDS for large payloads.
        self._pub.publish(msg)
        self._tx_count += 1

    def _report_stats(self) -> None:
        self.get_logger().info(
            f'relayed {self._tx_count}/{self._rx_count} clouds '
            f'(rx_total={self._rx_count})'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
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