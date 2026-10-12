#!/usr/bin/env python3
"""
frontier_explorer.py

Custom frontier-based exploration node. Replaces explore_lite.

Flow:
  1. Subscribe to /map (OccupancyGrid, TRANSIENT_LOCAL QoS to match slam_toolbox).
  2. Subscribe to /explore/resume (Bool) — the Go2 Explore panel toggles this.
  3. When enabled and the map has frontiers:
       - Detect frontier cells (free adjacent to unknown).
       - Cluster into frontier groups.
       - Score by size / distance (gain-based selection).
       - Send highest-scoring frontier to /navigate_to_pose action.
       - Wait for the goal to complete, then repeat.
  4. When no frontiers remain for N consecutive checks:
       - Send one goal back to the origin.
       - Wait for arrival.
       - Publish DONE status.

Works with partial maps — starts exploring as soon as any frontier exists.
"""
import math
import time
from dataclasses import dataclass
from typing import Optional, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy)
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener


# ---------------------------------------------------------------------------
# Tunables (override with --ros-args -p name:=value)
# ---------------------------------------------------------------------------
DEFAULTS = {
    'robot_base_frame': 'base_link',
    'map_frame':        'map',
    'map_topic':        '/map',
    'resume_topic':     '/explore/resume',
    'status_topic':     '/explore/status',

    # Frontier detection
    'min_frontier_size':     8,      # grid cells
    'min_known_cells':       200,    # don't start until map has this many known cells
    'frontier_blacklist_s':  30.0,   # don't retry a failed frontier for N sec
    'goal_timeout_s':        120.0,  # cancel goal if not reached in N sec
    'progress_timeout_s':    45.0,   # cancel if no meaningful progress in N sec

    # Loop timing
    'tick_hz':               1.0,    # exploration loop rate

    # Return-to-init
    'return_to_init':        True,
    'return_xy_tolerance':   0.3,    # consider "home" within N meters
    'return_timeout_s':      120.0,
}


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------
@dataclass
class Frontier:
    cx: float          # world x
    cy: float          # world y
    size: int          # cells in cluster
    distance: float    # straight-line from robot


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------
class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        # Parameters
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)

        p = lambda name: self.get_parameter(name).value
        self.robot_frame = p('robot_base_frame')
        self.map_frame = p('map_frame')
        self.min_frontier_size = int(p('min_frontier_size'))
        self.min_known_cells = int(p('min_known_cells'))
        self.blacklist_s = float(p('frontier_blacklist_s'))
        self.goal_timeout = float(p('goal_timeout_s'))
        self.progress_timeout = float(p('progress_timeout_s'))
        self.tick_hz = float(p('tick_hz'))
        self.return_to_init = bool(p('return_to_init'))
        self.return_tol = float(p('return_xy_tolerance'))
        self.return_timeout = float(p('return_timeout_s'))

        # State
        self.map: Optional[OccupancyGrid] = None
        self.enabled = False
        self.state = 'IDLE'         # IDLE | EXPLORING | RETURNING | DONE
        self.origin_xy: Optional[Tuple[float, float]] = None
        self.current_goal_handle = None
        self.current_goal_xy: Optional[Tuple[float, float]] = None
        self.goal_start_time = 0.0
        self.last_progress_time = 0.0
        self.last_progress_xy: Optional[Tuple[float, float]] = None
        self.blacklist = {}          # (grid_x, grid_y) -> expiry_time
        self.empty_frontier_ticks = 0
        self.goals_sent = 0
        self.goals_succeeded = 0
        self.goals_failed = 0

        # QoS: match slam_toolbox's /map publisher (TRANSIENT_LOCAL + RELIABLE)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, p('map_topic'), self.on_map, map_qos)
        self.resume_sub = self.create_subscription(
            Bool, p('resume_topic'), self.on_resume, 10)

        # Publishers
        self.status_pub = self.create_publisher(
            String, p('status_topic'), 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action client
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # Main loop
        self.create_timer(1.0 / self.tick_hz, self.tick)

        self.get_logger().info(
            f'frontier_explorer ready (min_frontier={self.min_frontier_size}, '
            f'return_to_init={self.return_to_init}, tick={self.tick_hz} Hz)')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def on_map(self, msg: OccupancyGrid):
        self.map = msg

    def on_resume(self, msg: Bool):
        new = bool(msg.data)
        if new == self.enabled:
            return
        self.enabled = new
        if new:
            self.get_logger().info('Exploration ENABLED')
            # Capture origin on first enable
            if self.origin_xy is None:
                pose = self.get_robot_pose()
                if pose is not None:
                    self.origin_xy = pose
                    self.get_logger().info(
                        f'Origin captured at ({pose[0]:.2f}, {pose[1]:.2f})')
            if self.state in ('IDLE', 'DONE'):
                self.state = 'EXPLORING'
                self.empty_frontier_ticks = 0
        else:
            self.get_logger().info('Exploration DISABLED — cancelling current goal')
            self.cancel_current_goal()
            self.state = 'IDLE'
        self.publish_status()

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def tick(self):
        if not self.enabled:
            return
        if self.map is None:
            return
        if self.state == 'IDLE' or self.state == 'DONE':
            return

        # If a goal is already running, only monitor it
        if self.current_goal_handle is not None:
            self.monitor_goal()
            return

        # Otherwise: pick a frontier
        if self.state == 'EXPLORING':
            self.step_explore()
        elif self.state == 'RETURNING':
            self.step_return()

    # ------------------------------------------------------------------
    # Exploration step
    # ------------------------------------------------------------------
    def step_explore(self):
        grid = self.to_numpy(self.map)
        if grid is None:
            return

        known_cells = int(np.count_nonzero(grid != -1))
        if known_cells < self.min_known_cells:
            self.get_logger().info(
                f'Map warming up: {known_cells}/{self.min_known_cells} known cells',
                throttle_duration_sec=5.0)
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.get_logger().warn(
                'Robot pose unavailable (TF map->base_link missing)',
                throttle_duration_sec=5.0)
            return

        frontiers = self.detect_frontiers(grid, robot_pose)
        if not frontiers:
            self.empty_frontier_ticks += 1
            if self.empty_frontier_ticks >= 5:
                self.get_logger().info('No frontiers remain — exploration complete')
                self.state = 'RETURNING' if self.return_to_init else 'DONE'
                self.empty_frontier_ticks = 0
                self.publish_status()
            return
        self.empty_frontier_ticks = 0

        # Score frontiers by gain/cost
        frontiers.sort(key=lambda f: f.size / max(f.distance, 0.5), reverse=True)
        best = frontiers[0]

        self.get_logger().info(
            f'Frontier selected: ({best.cx:.2f}, {best.cy:.2f}) '
            f'size={best.size} dist={best.distance:.2f}m '
            f'(out of {len(frontiers)} candidates)')

        self.send_goal(best.cx, best.cy, tag=(int(best.cx / self.map.info.resolution),
                                              int(best.cy / self.map.info.resolution)))

    def step_return(self):
        if self.origin_xy is None:
            self.state = 'DONE'
            self.publish_status()
            return

        pose = self.get_robot_pose()
        if pose is None:
            return
        dx = pose[0] - self.origin_xy[0]
        dy = pose[1] - self.origin_xy[1]
        dist = math.hypot(dx, dy)
        if dist < self.return_tol:
            self.get_logger().info(
                f'Returned to origin (dist={dist:.2f}m) — DONE')
            self.state = 'DONE'
            self.publish_status()
            return

        self.get_logger().info(
            f'Returning to origin ({self.origin_xy[0]:.2f}, '
            f'{self.origin_xy[1]:.2f}), currently {dist:.2f}m away')
        self.send_goal(self.origin_xy[0], self.origin_xy[1], tag=None)

    # ------------------------------------------------------------------
    # Frontier detection
    # ------------------------------------------------------------------
    def to_numpy(self, occ: OccupancyGrid) -> Optional[np.ndarray]:
        if not occ.data:
            return None
        w, h = occ.info.width, occ.info.height
        grid = np.array(occ.data, dtype=np.int8).reshape(h, w)
        return grid

    def detect_frontiers(
            self, grid: np.ndarray, robot_xy: Tuple[float, float]
    ) -> List[Frontier]:
        free = (grid == 0)
        unknown = (grid == -1)

        # Frontier cells: free cells with at least one unknown 4-neighbor
        unknown_d = (
            np.roll(unknown, 1, 0) | np.roll(unknown, -1, 0) |
            np.roll(unknown, 1, 1) | np.roll(unknown, -1, 1))
        frontier_mask = free & unknown_d

        # Remove frontier cells that are already blacklisted
        now = time.time()
        self.blacklist = {k: v for k, v in self.blacklist.items() if v > now}

        ys, xs = np.where(frontier_mask)
        if len(xs) == 0:
            return []

        # Cluster by simple BFS on a coarser grid (5-cell blocks)
        res = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y

        used = np.zeros_like(frontier_mask, dtype=bool)
        frontiers: List[Frontier] = []
        for x0, y0 in zip(xs.tolist(), ys.tolist()):
            if used[y0, x0]:
                continue
            # BFS cluster
            stack = [(x0, y0)]
            cluster_pts = []
            used[y0, x0] = True
            while stack:
                cx, cy = stack.pop()
                cluster_pts.append((cx, cy))
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nx, ny = cx + dx, cy + dy
                    if (0 <= nx < grid.shape[1] and 0 <= ny < grid.shape[0]
                            and frontier_mask[ny, nx] and not used[ny, nx]):
                        used[ny, nx] = True
                        stack.append((nx, ny))

            if len(cluster_pts) < self.min_frontier_size:
                continue

            # Cluster centroid in world coords
            px = np.array([p[0] for p in cluster_pts], dtype=float)
            py = np.array([p[1] for p in cluster_pts], dtype=float)
            cxw = px.mean() * res + ox + res / 2
            cyw = py.mean() * res + oy + res / 2

            # Skip if any cell in the cluster is blacklisted
            blk = False
            for (bx, by) in cluster_pts[:min(len(cluster_pts), 10)]:
                if (bx, by) in self.blacklist:
                    blk = True
                    break
            if blk:
                continue

            dist = math.hypot(cxw - robot_xy[0], cyw - robot_xy[1])
            frontiers.append(Frontier(cxw, cyw, len(cluster_pts), dist))

        return frontiers

    # ------------------------------------------------------------------
    # Goal management
    # ------------------------------------------------------------------
    def send_goal(self, x: float, y: float, tag):
        if self.current_goal_handle is not None:
            return
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 action server unavailable',
                                   throttle_duration_sec=10.0)
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Sending goal: ({x:.2f}, {y:.2f})')
        self.goals_sent += 1
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(lambda f: self.on_goal_accepted(f, x, y, tag))

    def on_goal_accepted(self, future, x, y, tag):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Goal rejected: ({x:.2f}, {y:.2f})')
            self.goals_failed += 1
            if tag is not None:
                self.blacklist[tag] = time.time() + self.blacklist_s
            return

        self.current_goal_handle = goal_handle
        self.current_goal_xy = (x, y)
        self.goal_start_time = time.time()
        self.last_progress_time = time.time()
        self.last_progress_xy = None

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.on_goal_done(f, tag))

    def on_goal_done(self, future, tag):
        status = future.result().status
        # 4 = SUCCEEDED in action_msgs/GoalStatus
        if status == 4:
            self.goals_succeeded += 1
            self.get_logger().info(
                f'Goal reached: ({self.current_goal_xy[0]:.2f}, '
                f'{self.current_goal_xy[1]:.2f})')
        else:
            self.goals_failed += 1
            self.get_logger().warn(f'Goal failed (status={status})')
            if tag is not None:
                self.blacklist[tag] = time.time() + self.blacklist_s

        self.current_goal_handle = None
        self.current_goal_xy = None
        self.publish_status()

    def cancel_current_goal(self):
        if self.current_goal_handle is not None:
            self.get_logger().info('Cancelling current goal')
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
            self.current_goal_xy = None

    def monitor_goal(self):
        # Progress check
        pose = self.get_robot_pose()
        if pose is None:
            return
        now = time.time()

        if self.last_progress_xy is None:
            self.last_progress_xy = pose
            self.last_progress_time = now
            return

        moved = math.hypot(pose[0] - self.last_progress_xy[0],
                           pose[1] - self.last_progress_xy[1])
        if moved > 0.2:
            self.last_progress_xy = pose
            self.last_progress_time = now

        if now - self.last_progress_time > self.progress_timeout:
            self.get_logger().warn(
                f'No progress for {self.progress_timeout:.0f}s — cancelling goal')
            self.cancel_current_goal()

        if now - self.goal_start_time > self.goal_timeout:
            self.get_logger().warn(
                f'Goal exceeded {self.goal_timeout:.0f}s — cancelling goal')
            self.cancel_current_goal()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def get_robot_pose(self) -> Optional[Tuple[float, float]]:
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def publish_status(self):
        msg = String()
        msg.data = (
            f'state={self.state} enabled={self.enabled} '
            f'goals_sent={self.goals_sent} '
            f'succeeded={self.goals_succeeded} '
            f'failed={self.goals_failed}')
        self.status_pub.publish(msg)


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_current_goal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()