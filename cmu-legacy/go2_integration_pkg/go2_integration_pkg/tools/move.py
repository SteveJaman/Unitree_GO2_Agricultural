#!/usr/bin/env python3

import json
import time
import sys

import rclpy
from rclpy.node import Node

from unitree_api.msg import Request, RequestHeader, RequestIdentity


MOVE_API_ID = 1008

SPEED = 0.3          # m/s
DURATION_S = 3.0     # seconds
PUBLISH_HZ = 50      # Sport API watchdog requires frequent commands


class Mover(Node):
    def __init__(self):
        super().__init__('go2_move')

        self.pub = self.create_publisher(
            Request,
            '/api/sport/request',
            10
        )

    def send(self, vx, vy, vyaw):
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


def get_direction():
    if len(sys.argv) > 1:
        direction = sys.argv[1].lower()
    else:
        direction = input(
            "Direction (forward/backward/left/right): "
        ).strip().lower()

    return direction


def main():
    rclpy.init()

    node = Mover()

    direction = get_direction()

    # Determine velocity based on direction
    if direction == "forward":
        vx = SPEED
        vy = 0.0

    elif direction == "backward":
        vx = -SPEED
        vy = 0.0

    elif direction == "left":
        vx = 0.0
        vy = SPEED

    elif direction == "right":
        vx = 0.0
        vy = -SPEED

    else:
        node.get_logger().error(
            "Invalid direction. Use: forward, backward, left, or right."
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    vyaw = 0.0

    node.get_logger().info(
        f"Moving {direction}: "
        f"vx={vx}, vy={vy}, duration={DURATION_S}s"
    )

    period = 1.0 / PUBLISH_HZ

    # Movement
    end_time = time.time() + DURATION_S

    while time.time() < end_time:
        node.send(vx, vy, vyaw)
        time.sleep(period)

    # Stop
    node.get_logger().info("Stopping...")

    for _ in range(PUBLISH_HZ):
        node.send(0.0, 0.0, 0.0)
        time.sleep(period)

    node.get_logger().info("Done.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
