#!/usr/bin/env python3
"""Zero-touch localization at the dock (wifi_3d_nav.sh --ai / --resume).

Publishes the start pose (default x=0, y=0, yaw=0 in lidar3d_map, i.e. the
dock when the dog starts there) on /initialpose once, so nobody has to click
"2D Pose Estimate" after a reboot, then exits.

  * Waits up to wait_for_subscriber s for a subscriber, so the message is not
    lost to DDS discovery, then publishes it `repeat` times, 0.5 s apart.
  * Point-LIO already starts every session at the lidar3d_map origin; with no
    /initialpose subscriber (none in this repo) the message is informational.
  * Safety: skipped if TF shows the robot already more than max_offset m from
    the origin, i.e. the session has been driven; re-anchoring then would be
    wrong.

Publishes: /initialpose (geometry_msgs/PoseWithCovarianceStamped)
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformException, TransformListener


class AutoLocalize(Node):
    def __init__(self):
        super().__init__('auto_localize')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('map_frame', 'lidar3d_map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('wait_for_subscriber', 20.0)   # s
        self.declare_parameter('repeat', 3)
        self.declare_parameter('max_offset', 0.5)             # m from the origin
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        self.x, self.y, self.yaw = float(gp('x')), float(gp('y')), float(gp('yaw'))
        self.map_frame, self.base_frame = gp('map_frame'), gp('robot_base_frame')
        self.wait = float(gp('wait_for_subscriber'))
        self.repeat = max(1, int(gp('repeat')))
        self.max_offset = float(gp('max_offset'))
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def robot_offset(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time()).transform.translation
            return math.hypot(t.x - self.x, t.y - self.y)
        except TransformException:
            return None

    def run(self):
        log = self.get_logger()
        end = time.monotonic() + self.wait
        while self.pub.get_subscription_count() == 0 and time.monotonic() < end:
            self.spin_for(0.5)
        subs = self.pub.get_subscription_count()

        offset = self.robot_offset()
        if offset is not None and offset > self.max_offset:
            log.warn(f'robot is {offset:.2f} m from ({self.x}, {self.y}) in {self.map_frame}: '
                     'not at the dock, initial pose NOT published')
            return False

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.x, self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05 ** 2        # x, y: 5 cm
        cov[35] = math.radians(3) ** 2     # yaw: 3 deg
        msg.pose.covariance = cov
        for _ in range(self.repeat):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            self.spin_for(0.5)
        where = 'unknown (no TF yet)' if offset is None else f'{offset:.2f} m from it'
        if subs:
            log.info(f'initial pose ({self.x}, {self.y}, yaw {self.yaw}) in {self.map_frame} '
                     f'published to {subs} subscriber(s); robot {where}')
        else:
            log.info(f'initial pose published, but nothing subscribes to /initialpose: '
                     f'Point-LIO already starts at the {self.map_frame} origin (robot {where})')
        return True


def main():
    rclpy.init()
    node = AutoLocalize()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
