#!/usr/bin/env python3
"""
pose_logger.py: record one navigation trial for scripts/tools/nav_compare.

Runs inside go2-lidar3d next to the session (sim or real dog) and writes to
--out (default /tmp/nav_trial):
  pose.csv     t, x, y, yaw      robot pose in the map frame, 5 Hz (TF)
  cmd.csv      t, vx, vy, wz     every velocity command sent to the dog
                                 (/cmd_vel, after the velocity smoother)
  costmap.npz  the final global costmap (data, resolution, origin): the
               latest full /global_costmap/costmap with the
               /global_costmap/costmap_updates patches applied

Stop it with SIGINT/SIGTERM; files are flushed as they are written and the
costmap is saved on exit.

  python3 pose_logger.py --out /tmp/nav_trial [--map-frame lidar3d_map]
"""

import argparse
import math
import os
import signal
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class PoseLogger(Node):
    def __init__(self, out, map_frame, base_frame):
        super().__init__('nav_compare_pose_logger')
        os.makedirs(out, exist_ok=True)
        self.out = out
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.pose_f = open(os.path.join(out, 'pose.csv'), 'w', buffering=1)
        self.pose_f.write('t,x,y,yaw\n')
        self.cmd_f = open(os.path.join(out, 'cmd.csv'), 'w', buffering=1)
        self.cmd_f.write('t,vx,vy,wz\n')
        self.costmap = None    # (info, grid) of the latest full costmap + patches

        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 20)
        # The full costmap is latched and sent only now and then (e.g. when the
        # rolling window moves); patches come on costmap_updates in between.
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self.on_map, map_qos)
        self.create_subscription(OccupancyGridUpdate, '/global_costmap/costmap_updates',
                                 self.on_update, 10)
        self.create_timer(0.2, self.on_timer)
        self.get_logger().info(f'logging to {out}')

    def on_cmd(self, msg):
        self.cmd_f.write(f'{time.time():.3f},{msg.linear.x:.4f},'
                         f'{msg.linear.y:.4f},{msg.angular.z:.4f}\n')

    def on_map(self, msg):
        grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.costmap = (msg.info, grid)

    def on_update(self, u):
        if self.costmap is None:
            return
        grid = self.costmap[1]
        if u.y + u.height > grid.shape[0] or u.x + u.width > grid.shape[1]:
            return
        grid[u.y:u.y + u.height, u.x:u.x + u.width] = \
            np.array(u.data, dtype=np.int8).reshape(u.height, u.width)

    def on_timer(self):
        try:
            t = self.tf.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:
            return
        q, p = t.transform.rotation, t.transform.translation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose_f.write(f'{time.time():.3f},{p.x:.3f},{p.y:.3f},{yaw:.4f}\n')

    def save(self):
        self.pose_f.close()
        self.cmd_f.close()
        if self.costmap is None:
            return
        info, grid = self.costmap
        np.savez_compressed(
            os.path.join(self.out, 'costmap.npz'), data=grid,
            resolution=info.resolution,
            origin=[info.origin.position.x, info.origin.position.y])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/tmp/nav_trial')
    ap.add_argument('--map-frame', default='lidar3d_map')
    ap.add_argument('--base-frame', default='base_link')
    a, ros_args = ap.parse_known_args()
    # Own SIGINT/SIGTERM handling: rclpy's handler shuts the context down
    # mid-spin (ExternalShutdownException), which would skip save().
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = PoseLogger(a.out, a.map_frame, a.base_frame)
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    while rclpy.ok() and not stop:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.save()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
