#!/usr/bin/env python3
"""
frontier_explorer.py

Custom three-phase exploration node. Replaces explore_lite.

Phases:
  1. BLIND LEAD-IN (Roomba-style)
  2. FRONTIER EXPLORATION (Yamauchi 1997)
  3. VISITED-CELL COVERAGE
  4. RETURN TO ORIGIN (optional)

Communication:
  Publishes PoseStamped goals to /goal_pose.
  Emergency stop publishes Twist on /cmd_vel (bypasses the smoother so
  it can override Nav2's controller, which publishes on /cmd_vel_nav).
"""
import math
import time
from dataclasses import dataclass
from typing import Optional, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy)
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


DEFAULTS = {
    'robot_base_frame': 'base_link',
    'map_frame':        'lidar3d_map',
    'map_topic':        '/global_costmap/costmap',
    'resume_topic':     '/explore/resume',
    'status_topic':     '/explore/status',
    'goal_topic':       '/goal_pose',
    'cmd_vel_topic':    '/cmd_vel_nav',
    'stop_topic':       '/cmd_vel',           # bypasses the smoother

    # Frontier detection
    'min_frontier_size':     4,               # was 8 — frontiers are thin
    'min_known_cells':       200,
    'frontier_blacklist_s':  30.0,

    # Goal tracking
    'goal_xy_tolerance':     0.35,
    'goal_timeout_s':        60.0,
    'progress_timeout_s':    20.0,
    'progress_min_m':        0.15,

    # Blind lead-in
    'blind_enable':          True,
    'blind_trigger_cells':   100,
    'blind_step_distance':   0.6,
    'blind_step_speed':      0.12,
    'blind_step_timeout':    20.0,
    'blind_max_attempts':    4,

    # Coverage
    'coverage_enable':       True,
    'coverage_min_dist':     0.6,
    'visited_radius_m':      0.4,

    # Loop timing
    'tick_hz':               1.0,

    # Return-to-init
    'return_to_init':        True,
    'return_xy_tolerance':   0.3,
    'return_timeout_s':      120.0,
}


@dataclass
class Frontier:
    cx: float
    cy: float
    size: int
    distance: float


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)

        p = lambda name: self.get_parameter(name).value
        self.robot_frame = p('robot_base_frame')
        self.map_frame = p('map_frame')
        self.goal_topic = p('goal_topic')
        self.cmd_vel_topic = p('cmd_vel_topic')
        self.stop_topic = p('stop_topic')

        self.min_frontier_size = int(p('min_frontier_size'))
        self.min_known_cells = int(p('min_known_cells'))
        self.blacklist_s = float(p('frontier_blacklist_s'))

        self.goal_xy_tol = float(p('goal_xy_tolerance'))
        self.goal_timeout = float(p('goal_timeout_s'))
        self.progress_timeout = float(p('progress_timeout_s'))
        self.progress_min_m = float(p('progress_min_m'))

        self.blind_enable = bool(p('blind_enable'))
        self.blind_trigger_cells = int(p('blind_trigger_cells'))
        self.blind_step_distance = float(p('blind_step_distance'))
        self.blind_step_speed = float(p('blind_step_speed'))
        self.blind_step_timeout = float(p('blind_step_timeout'))
        self.blind_max_attempts = int(p('blind_max_attempts'))

        self.coverage_enable = bool(p('coverage_enable'))
        self.coverage_min_dist = float(p('coverage_min_dist'))
        self.visited_radius = float(p('visited_radius_m'))

        self.tick_hz = float(p('tick_hz'))
        self.return_to_init = bool(p('return_to_init'))
        self.return_tol = float(p('return_xy_tolerance'))
        self.return_timeout = float(p('return_timeout_s'))

        # State
        self.map: Optional[OccupancyGrid] = None
        self.enabled = False
        self.state = 'IDLE'
        self.origin_xy: Optional[Tuple[float, float]] = None
        self.blacklist = {}
        self.empty_frontier_ticks = 0
        self.goals_sent = 0
        self.goals_succeeded = 0
        self.goals_failed = 0
        self.blind_attempts = 0

        self.active_goal_xy: Optional[Tuple[float, float]] = None
        self.active_goal_tag = None
        self.active_goal_start = 0.0
        self.active_goal_last_progress_xy: Optional[Tuple[float, float]] = None
        self.active_goal_last_progress_time = 0.0

        self.blind_active = False
        self.blind_start_xy: Optional[Tuple[float, float]] = None
        self.blind_start_time = 0.0

        self.visited: Optional[np.ndarray] = None

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.map_sub = self.create_subscription(
            OccupancyGrid, p('map_topic'), self.on_map, map_qos)
        self.resume_sub = self.create_subscription(
            Bool, p('resume_topic'), self.on_resume, 10)

        self.status_pub = self.create_publisher(String, p('status_topic'), 10)
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        # Separate publisher for emergency stop, on /cmd_vel, so it
        # overrides Nav2's controller output on /cmd_vel_nav.
        self.stop_pub = self.create_publisher(Twist, self.stop_topic, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(1.0 / self.tick_hz, self.tick)

        self.get_logger().info(
            f'frontier_explorer ready: goal={self.goal_topic} '
            f'cmd_vel={self.cmd_vel_topic} stop={self.stop_topic} '
            f'map_topic={p("map_topic")} map_frame={self.map_frame} '
            f'min_frontier_size={self.min_frontier_size}')

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
            if self.origin_xy is None:
                pose = self.get_robot_pose()
                if pose is not None:
                    self.origin_xy = pose
                    self.get_logger().info(
                        f'Origin captured at ({pose[0]:.2f}, {pose[1]:.2f})')
            if self.state in ('IDLE', 'DONE'):
                self.state = 'EXPLORING'
                self.empty_frontier_ticks = 0
                self.blind_attempts = 0
        else:
            self.get_logger().info('Exploration DISABLED — stopping robot')
            self.emergency_stop()
            self.cancel_active_goal()
            self.stop_blind()
            self.state = 'IDLE'
        self.publish_status()

    # ------------------------------------------------------------------
    def tick(self):
        if not self.enabled or self.state in ('IDLE', 'DONE'):
            return
        if self.map is None:
            return

        if self.blind_active:
            self.tick_blind()
            return

        if self.active_goal_xy is not None:
            self.tick_active_goal()
            return

        if self.state == 'EXPLORING':
            self.step_explore()
        elif self.state == 'RETURNING':
            self.step_return()

    # ------------------------------------------------------------------
    def emergency_stop(self):
        """Publish zero velocity on /cmd_vel (bypasses the smoother).

        Nav2's controller publishes on /cmd_vel_nav, which the smoother
        relays to /cmd_vel. Publishing here at a high rate briefly
        dominates the wire and brings the robot to a stop.
        """
        stop = Twist()
        for _ in range(10):
            self.stop_pub.publish(stop)
            time.sleep(0.05)

    # ------------------------------------------------------------------
    def start_blind_step(self, robot_pose):
        self.blind_active = True
        self.blind_start_xy = robot_pose
        self.blind_start_time = time.time()
        self.get_logger().info(
            f'BLIND step {self.blind_attempts + 1}/{self.blind_max_attempts}: '
            f'driving {self.blind_step_distance} m at {self.blind_step_speed} m/s')

    def tick_blind(self):
        if self.blind_start_xy is None:
            self.stop_blind()
            return

        pose = self.get_robot_pose()
        if pose is None:
            self.stop_blind()
            return

        elapsed = time.time() - self.blind_start_time
        travelled = math.hypot(pose[0] - self.blind_start_xy[0],
                               pose[1] - self.blind_start_xy[1])

        if (travelled >= self.blind_step_distance
                or elapsed >= self.blind_step_timeout):
            self.get_logger().info(
                f'BLIND step done: {travelled:.2f} m in {elapsed:.1f}s')
            self.blind_attempts += 1
            self.stop_blind()
            return

        twist = Twist()
        twist.linear.x = self.blind_step_speed
        self.cmd_vel_pub.publish(twist)

    def stop_blind(self):
        if not self.blind_active:
            return
        self.cmd_vel_pub.publish(Twist())
        self.blind_active = False
        self.blind_start_xy = None
        self.blind_start_time = 0.0

    # ------------------------------------------------------------------
    def step_explore(self):
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.get_logger().warn(
                'Robot pose unavailable', throttle_duration_sec=5.0)
            return

        self.mark_visited(robot_pose)

        grid = self.to_numpy(self.map)
        if grid is None:
            return

        known_cells = int(np.count_nonzero(grid != -1))

        if (self.blind_enable
                and known_cells < self.blind_trigger_cells
                and self.blind_attempts < self.blind_max_attempts):
            self.start_blind_step(robot_pose)
            return

        if known_cells < self.min_known_cells:
            self.get_logger().info(
                f'Waiting for map to grow: {known_cells}/{self.min_known_cells}',
                throttle_duration_sec=5.0)
            return

        frontiers = self.detect_frontiers(grid, robot_pose)
        self.get_logger().info(
            f'detect_frontiers: known={known_cells} frontiers={len(frontiers)}',
            throttle_duration_sec=5.0)

        if frontiers:
            self.empty_frontier_ticks = 0
            frontiers.sort(
                key=lambda f: f.size / max(f.distance, 0.5), reverse=True)
            best = frontiers[0]
            self.get_logger().info(
                f'FRONTIER: ({best.cx:.2f}, {best.cy:.2f}) '
                f'size={best.size} dist={best.distance:.2f}m '
                f'({len(frontiers)} candidates)')
            tag = (int((best.cx - self.map.info.origin.position.x)
                       / self.map.info.resolution),
                   int((best.cy - self.map.info.origin.position.y)
                       / self.map.info.resolution))
            self.publish_goal(best.cx, best.cy, tag=tag)
            return

        self.empty_frontier_ticks += 1

        if self.coverage_enable and self.empty_frontier_ticks >= 2:
            target = self.nearest_unvisited(robot_pose)
            if target is not None:
                dist = math.hypot(target[0] - robot_pose[0],
                                  target[1] - robot_pose[1])
                if dist >= self.coverage_min_dist:
                    self.get_logger().info(
                        f'COVERAGE: ({target[0]:.2f}, {target[1]:.2f}) '
                        f'dist={dist:.2f}m')
                    self.publish_goal(target[0], target[1], tag=None)
                    return

        if self.empty_frontier_ticks >= 5:
            self.get_logger().info('Exploration complete')
            self.state = 'RETURNING' if self.return_to_init else 'DONE'
            self.empty_frontier_ticks = 0
            self.publish_status()

    # ------------------------------------------------------------------
    def step_return(self):
        if self.origin_xy is None:
            self.state = 'DONE'
            self.publish_status()
            return

        pose = self.get_robot_pose()
        if pose is None:
            return

        dist = math.hypot(pose[0] - self.origin_xy[0],
                          pose[1] - self.origin_xy[1])
        if dist < self.return_tol:
            self.get_logger().info(f'Returned to origin — DONE')
            self.state = 'DONE'
            self.publish_status()
            return

        self.publish_goal(self.origin_xy[0], self.origin_xy[1], tag=None)

    # ------------------------------------------------------------------
    def publish_goal(self, x, y, tag=None):
        if self.active_goal_xy is not None:
            return

        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)
        self.goals_sent += 1
        self.active_goal_xy = (x, y)
        self.active_goal_tag = tag
        self.active_goal_start = time.time()
        self.active_goal_last_progress_xy = None
        self.active_goal_last_progress_time = time.time()

        self.get_logger().info(f'Goal sent: ({x:.2f}, {y:.2f})')

    def tick_active_goal(self):
        pose = self.get_robot_pose()
        if pose is None:
            return
        now = time.time()

        dist = math.hypot(pose[0] - self.active_goal_xy[0],
                          pose[1] - self.active_goal_xy[1])
        if dist < self.goal_xy_tol:
            self.goals_succeeded += 1
            self.get_logger().info(
                f'Goal reached: ({self.active_goal_xy[0]:.2f}, '
                f'{self.active_goal_xy[1]:.2f})')
            self.clear_active_goal()
            return

        if self.active_goal_last_progress_xy is None:
            self.active_goal_last_progress_xy = pose
            self.active_goal_last_progress_time = now
        else:
            moved = math.hypot(
                pose[0] - self.active_goal_last_progress_xy[0],
                pose[1] - self.active_goal_last_progress_xy[1])
            if moved >= self.progress_min_m:
                self.active_goal_last_progress_xy = pose
                self.active_goal_last_progress_time = now

        if now - self.active_goal_last_progress_time > self.progress_timeout:
            self.get_logger().warn('No progress — abandoning goal')
            self.blacklist_tag()
            self.goals_failed += 1
            self.clear_active_goal()
            return

        if now - self.active_goal_start > self.goal_timeout:
            self.get_logger().warn('Goal timeout — abandoning')
            self.blacklist_tag()
            self.goals_failed += 1
            self.clear_active_goal()
            return

    def blacklist_tag(self):
        if self.active_goal_tag is not None:
            self.blacklist[self.active_goal_tag] = time.time() + self.blacklist_s

    def clear_active_goal(self):
        self.active_goal_xy = None
        self.active_goal_tag = None
        self.active_goal_last_progress_xy = None

    def cancel_active_goal(self):
        if self.active_goal_xy is not None:
            self.get_logger().info('Cancelling active goal')
            self.emergency_stop()
            self.clear_active_goal()

    # ------------------------------------------------------------------
    def mark_visited(self, world_xy):
        if self.map is None:
            return
        grid = self.to_numpy(self.map)
        if grid is None:
            return

        if self.visited is None or self.visited.shape != grid.shape:
            new_visited = np.zeros(grid.shape, dtype=bool)
            if self.visited is not None:
                h = min(self.visited.shape[0], grid.shape[0])
                w = min(self.visited.shape[1], grid.shape[1])
                new_visited[:h, :w] = self.visited[:h, :w]
            self.visited = new_visited

        res = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y
        cx = int((world_xy[0] - ox) / res)
        cy = int((world_xy[1] - oy) / res)
        r = max(1, int(self.visited_radius / res))
        x0 = max(0, cx - r); x1 = min(grid.shape[1], cx + r + 1)
        y0 = max(0, cy - r); y1 = min(grid.shape[0], cy + r + 1)
        if x0 < x1 and y0 < y1:
            self.visited[y0:y1, x0:x1] = True

    def nearest_unvisited(self, robot_xy):
        if self.visited is None or self.map is None:
            return None
        grid = self.to_numpy(self.map)
        if grid is None or grid.shape != self.visited.shape:
            return None

        free = (grid == 0)
        unvisited = free & ~self.visited
        ys, xs = np.where(unvisited)
        if len(xs) == 0:
            return None

        res = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y
        wx = xs * res + ox + res / 2
        wy = ys * res + oy + res / 2
        dists = (wx - robot_xy[0]) ** 2 + (wy - robot_xy[1]) ** 2
        i = int(np.argmin(dists))
        return (float(wx[i]), float(wy[i]))

    # ------------------------------------------------------------------
    def to_numpy(self, occ):
        if not occ.data:
            return None
        w, h = occ.info.width, occ.info.height
        return np.array(occ.data, dtype=np.int8).reshape(h, w)

    def detect_frontiers(self, grid, robot_xy):
        free = (grid == 0)
        unknown = (grid == -1)

        padded = np.pad(unknown, 1, mode='constant', constant_values=False)
        unknown_d = (padded[:-2, 1:-1] | padded[2:, 1:-1] |
                     padded[1:-1, :-2] | padded[1:-1, 2:])
        frontier_mask = free & unknown_d

        now = time.time()
        self.blacklist = {k: v for k, v in self.blacklist.items() if v > now}

        ys, xs = np.where(frontier_mask)
        if len(xs) == 0:
            return []

        res = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y

        used = np.zeros_like(frontier_mask, dtype=bool)
        frontiers = []
        for x0, y0 in zip(xs.tolist(), ys.tolist()):
            if used[y0, x0]:
                continue
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

            px = np.array([p[0] for p in cluster_pts], dtype=float)
            py = np.array([p[1] for p in cluster_pts], dtype=float)
            cxw = px.mean() * res + ox + res / 2
            cyw = py.mean() * res + oy + res / 2

            if any((bx, by) in self.blacklist
                   for bx, by in cluster_pts[:min(len(cluster_pts), 10)]):
                continue

            dist = math.hypot(cxw - robot_xy[0], cyw - robot_xy[1])
            frontiers.append(Frontier(cxw, cyw, len(cluster_pts), dist))

        return frontiers

    # ------------------------------------------------------------------
    def get_robot_pose(self):
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
            f'failed={self.goals_failed} '
            f'blind_attempts={self.blind_attempts}')
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_blind()
        node.emergency_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()