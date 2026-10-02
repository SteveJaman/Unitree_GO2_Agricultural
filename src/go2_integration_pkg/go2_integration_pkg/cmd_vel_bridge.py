#!/usr/bin/env python3
"""
cmd_vel_bridge.py

Converts geometry_msgs/TwistStamped (published by the autonomy stack's
pathFollower) into geometry_msgs/Twist (expected by go2_robot_sdk).

Without this, the WebRTC SDK receives nothing on /cmd_vel and the robot
never moves — even though the planner is computing commands correctly.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelBridge(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_bridge')

        self.declare_parameter('input_topic',  '/cmd_vel_stamped')
        self.declare_parameter('output_topic', '/cmd_vel')

        in_topic  = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        self._pub = self.create_publisher(Twist, out_topic, 10)
        self._sub = self.create_subscription(
            TwistStamped, in_topic, self._on_cmd, 10)

        self.get_logger().info(
            f'cmd_vel_bridge: {in_topic} (TwistStamped) -> '
            f'{out_topic} (Twist)')

    def _on_cmd(self, msg: TwistStamped) -> None:
        out = Twist()
        out.linear  = msg.twist.linear
        out.angular = msg.twist.angular
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelBridge()
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
