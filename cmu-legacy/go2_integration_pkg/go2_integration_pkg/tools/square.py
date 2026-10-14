#!/usr/bin/env python3

import json
import time

import rclpy
from rclpy.node import Node

from unitree_api.msg import Request, RequestHeader, RequestIdentity


MOVE_API_ID = 1008

SPEED = 0.3          # m/s
SIDE_DURATION = 3.0  # seconds
PUBLISH_HZ = 50


class SquareMover(Node):
    def __init__(self):
        super().__init__('go2_square')

        self.pub = self.create_publisher(
            Request,
            '/api/sport/request',
            10
        )

    def send(self, vx, vy, vyaw=0.0):
        req = Request()

        req.header = RequestHeader()
        req.header.identity = RequestIdentity()
        req.header.identity.id = 0
        req.header.identity.api_id = MOVE_API_ID

        req.parameter = json.dumps({
            "x": vx,
            "y": vy,
            "z": vyaw
        })

        self.pub.publish(req)

    def move(self, vx, vy):
        period = 1.0 / PUBLISH_HZ
        end_time = time.time() + SIDE_DURATION

        while time.time() < end_time:
            self.send(vx, vy)
            time.sleep(period)

    def stop(self):
        period = 1.0 / PUBLISH_HZ

        for _ in range(PUBLISH_HZ):
            self.send(0.0, 0.0)
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
