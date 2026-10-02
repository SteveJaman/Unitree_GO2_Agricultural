#!/usr/bin/env python3
"""
Simple test node to command the Unitree Go2 to walk forward.
Publishes a safe, slow linear velocity to /robot0/cmd_vel.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MoveForwardTest(Node):
    def __init__(self):
        super().__init__('move_forward_test')

        # Publisher to the robot's velocity command topic
        self.publisher_ = self.create_publisher(Twist, '/robot0/cmd_vel', 10)

        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('🚀 MoveForwardTest initialized: Sending forward command (0.2 m/s)...')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.2  # 0.2 m/s forward (safe, slow testing speed)
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MoveForwardTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping motion test...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
