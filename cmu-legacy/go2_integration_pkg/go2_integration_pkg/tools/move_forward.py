#!/usr/bin/env python3
"""
move_forward.py

Sends a Unitree sport API "Move" command (api_id 1008) to the Go2 at
50 Hz for 3 seconds, then sends zero velocity for 1 second.

The Move command has JSON parameter {"x": vx, "y": vy, "z": vyaw}:
  x    = forward velocity (m/s, positive = forward)
  y    = lateral velocity (m/s, positive = left)
  z    = yaw rate (rad/s, positive = counterclockwise)

Sport Mode has a watchdog that stops the robot if no Move command is
received within ~500 ms, so we publish at 50 Hz to stay well inside it.
"""

import json
import time

import rclpy
from rclpy.node import Node

from unitree_api.msg import Request, RequestHeader, RequestIdentity


MOVE_API_ID = 1008      # Unitree SPORT_API_ID_MOVE
VX = 0.3                # m/s
VY = 0.0                # m/s
VYAW = 0.0              # rad/s
DURATION_S = 3.0
PUBLISH_HZ = 50


class Mover(Node):
    def __init__(self) -> None:
        super().__init__('go2_move_forward')
        self.pub = self.create_publisher(Request, '/api/sport/request', 10)

    def send(self, vx: float, vy: float, vyaw: float) -> None:
        req = Request()
        req.header = RequestHeader()
        req.header.identity = RequestIdentity()
        req.header.identity.id = 0
        req.header.identity.api_id = MOVE_API_ID
        req.parameter = json.dumps({"x": vx, "y": vy, "z": vyaw})
        self.pub.publish(req)


def main() -> None:
    rclpy.init()
    node = Mover()
    node.get_logger().info(
        f'Moving forward: vx={VX} m/s for {DURATION_S}s at {PUBLISH_HZ} Hz')

    period = 1.0 / PUBLISH_HZ

    # Move phase
    end = time.time() + DURATION_S
    while time.time() < end:
        node.send(VX, VY, VYAW)
        time.sleep(period)

    # Stop phase
    node.get_logger().info('Stopping (sending zero velocity)')
    for _ in range(PUBLISH_HZ):
        node.send(0.0, 0.0, 0.0)
        time.sleep(period)

    node.get_logger().info('Done.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()