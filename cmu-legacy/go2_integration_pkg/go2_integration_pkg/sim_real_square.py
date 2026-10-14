#!/usr/bin/env python3

import json
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

from unitree_api.msg import Request, RequestHeader, RequestIdentity

# Unitree Go2 Sport API 
MOVE_API_ID = 1008

# Movement parameters 
SPEED = 0.3          # m/s
SIDE_DURATION = 3.0  # seconds
PUBLISH_HZ = 50

class SquareMover(Node):
    def __init__(self, mode="sim"):
        super().__init__('go2_square')

        self.pub = self.create_publisher(
        	TwistStamped, #Request,
            '/cmd_vel', #'/api/sport/request',
            10
        )

    def send(self, vx, vy, vyaw=0.0):
        req = TwistStamped()
        req.header.stamp = self.get_clock().now().to_msg()
        req.header.frame_id = 'base_link'

        req.twist.linear.x = float(vx)
        req.twist.linear.y = float(vy)
        req.twist.angular.z = float(vyaw)

        self.pub.publish(req)

    def move(self, vx, vy):
        period = 1.0 / PUBLISH_HZ
        end_time = time.time() + SIDE_DURATION

        while rclpy.ok() and time.time() < end_time:
            self.send(vx, vy)
            time.sleep(period)

    def stop(self):
        period = 1.0 / PUBLISH_HZ
        end_time = time.monotonic() + 1.0 

        while rclpy.ok() and time.monotonic() < end_time:
        	self.send(0.0, 0.0, 0.0)
        	time.sleep(period)

def main():
    rclpy.init()

    node = SquareMover()

    # Forward
    node.move(SPEED, 0.0)

    # Left
    node.move(0.0, SPEED)

    # Backward
    node.move(-SPEED, 0.0)

    # Right
    node.move(0.0, -SPEED)

    # Stop after completing the square
    node.stop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
