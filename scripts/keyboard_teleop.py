#!/usr/bin/env python3

import sys
import select
import termios
import tty
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class Go2Teleop(Node):

    def __init__(self):
        super().__init__('go2_keyboard_teleop')

        self.pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        # Start conservatively
        self.linear_speed = 0.15
        self.angular_speed = 0.30

        self.cmd_x = 0.0
        self.cmd_y = 0.0
        self.cmd_z = 0.0

        # Publish continuously at 20 Hz
        self.timer = self.create_timer(
            0.05,
            self.publish_cmd
        )

    def publish_cmd(self):

        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.twist.linear.x = self.cmd_x
        msg.twist.linear.y = self.cmd_y
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = self.cmd_z

        self.pub.publish(msg)

    def set_cmd(self, x, y, z):

        self.cmd_x = x
        self.cmd_y = y
        self.cmd_z = z

        self.get_logger().info(
            f'cmd_vel: x={x:.2f}, y={y:.2f}, z={z:.2f}'
        )

    def stop(self):
        self.set_cmd(0.0, 0.0, 0.0)


def get_key():

    settings = termios.tcgetattr(sys.stdin)

    tty.setraw(sys.stdin.fileno())

    key = sys.stdin.read(1)

    termios.tcsetattr(
        sys.stdin,
        termios.TCSADRAIN,
        settings
    )

    return key


def main():

    rclpy.init()

    node = Go2Teleop()

    print()
    print('========================================')
    print('          UNITREE GO2 TELEOP')
    print('========================================')
    print()
    print('W       Forward')
    print('S       Backward')
    print('A       Rotate left')
    print('D       Rotate right')
    print('Q       Strafe left')
    print('E       Strafe right')
    print('SPACE   Stop')
    print('+       Faster')
    print('-       Slower')
    print('X       Exit')
    print()
    print(f'Linear speed : {node.linear_speed:.2f} m/s')
    print(f'Angular speed: {node.angular_speed:.2f} rad/s')
    print()
    print('Press a key...')
    print()

    try:

        while rclpy.ok():

            # Process ROS communications
            rclpy.spin_once(
                node,
                timeout_sec=0.01
            )

            # Check keyboard without blocking
            if select.select(
                [sys.stdin],
                [],
                [],
                0.01
            )[0]:

                key = get_key().lower()

                if key == 'w':
                    node.set_cmd(
                        node.linear_speed,
                        0.0,
                        0.0
                    )

                elif key == 's':
                    node.set_cmd(
                        -node.linear_speed,
                        0.0,
                        0.0
                    )

                elif key == 'a':
                    node.set_cmd(
                        0.0,
                        0.0,
                        node.angular_speed
                    )

                elif key == 'd':
                    node.set_cmd(
                        0.0,
                        0.0,
                        -node.angular_speed
                    )

                elif key == 'q':
                    node.set_cmd(
                        0.0,
                        node.linear_speed,
                        0.0
                    )

                elif key == 'e':
                    node.set_cmd(
                        0.0,
                        -node.linear_speed,
                        0.0
                    )

                elif key == ' ':
                    node.stop()

                elif key == '+':
                    node.linear_speed += 0.05
                    node.angular_speed += 0.10

                    print(
                        f'\nLinear: {node.linear_speed:.2f} m/s'
                    )
                    print(
                        f'Angular: {node.angular_speed:.2f} rad/s'
                    )

                elif key == '-':
                    node.linear_speed = max(
                        0.05,
                        node.linear_speed - 0.05
                    )

                    node.angular_speed = max(
                        0.10,
                        node.angular_speed - 0.10
                    )

                    print(
                        f'\nLinear: {node.linear_speed:.2f} m/s'
                    )
                    print(
                        f'Angular: {node.angular_speed:.2f} rad/s'
                    )

                elif key == 'x':
                    break

    except KeyboardInterrupt:
        pass

    finally:

        # Stop robot before exiting
        node.stop()

        # Keep publishing zero for a short period
        end_time = time.time() + 0.5

        while time.time() < end_time:
            rclpy.spin_once(
                node,
                timeout_sec=0.05
            )

        node.destroy_node()
        rclpy.shutdown()

        print('\nStopped.')


if __name__ == '__main__':
    main()
