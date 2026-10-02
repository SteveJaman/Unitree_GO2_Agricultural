#!/usr/bin/env python3
"""
Minimal Go2 forward motion test over Ethernet DDS.

Publishes to /api/sport/request with the Unitree "Move" API (id 1008).
The Jetson receives this natively over DDS and commands the robot.

Robot must be:
  - Powered on and standing (use the physical remote to stand it up)
  - In Sport Mode (default after standing)
"""

import json
import time
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, RequestHeader, RequestIdentity


MOVE_API_ID = 1008          # Unitree SPORT_API_ID_MOVE
VX          = 0.3           # forward speed (m/s)
VY          = 0.0           # lateral speed
VYAW        = 0.0           # yaw rate (rad/s)
DURATION_S  = 3.0           # how long to move
PUBLISH_HZ  = 50            # command rate


class Mover(Node):
    def __init__(self):
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


def main():
    rclpy.init()
    node = Mover()

    node.get_logger().info(
        f'Moving forward: vx={VX} m/s for {DURATION_S}s')

    # --- move ---
    period = 1.0 / PUBLISH_HZ
    end = time.time() + DURATION_S
    while time.time() < end:
        node.send(VX, VY, VYAW)
        time.sleep(period)

    # --- stop ---
    node.get_logger().info('Stopping (sending zero velocity)')
    for _ in range(PUBLISH_HZ):
        node.send(0.0, 0.0, 0.0)
        time.sleep(period)

    node.get_logger().info('Done.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
