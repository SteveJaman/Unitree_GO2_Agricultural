#!/usr/bin/env python3
"""
Simple test node to command the Unitree Go2 to walk forward.
Publishes TwistStamped messages to /cmd_vel_stamped for the bridge.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class MoveForwardTest(Node):
    def __init__(self):
        super().__init__('move_forward_test')
        
        # Publisher to the stamped velocity topic expected by cmd_vel_bridge.py
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel_stamped', 10)
        
        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('🚀 MoveForwardTest initialized: Sending TwistStamped to /cmd_vel_stamped (0.2 m/s)...')

    def timer_callback(self):
        msg = TwistStamped()
        # Fill in the header timestamp/frame so it's a valid stamped message
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Set safe forward speed
        msg.twist.linear.x = 0.2  # 0.2 m/s forward
        msg.twist.linear.y = 0.0
        msg.twist.angular.z = 0.0
        
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
