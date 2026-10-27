#!/usr/bin/env python3
"""
frontier_explorer.py

Custom three-phase exploration node. Replaces explore_lite.

Phases:
  1. BLIND LEAD-IN (Roomba-style)
       When the map has < min_known_cells, drive forward directly
       via cmd_vel for a short distance. Forces new sensor data
       so the map can grow.

  2. FRONTIER EXPLORATION (Yamauchi 1997)
       Detect frontier cells (free adjacent to unknown), cluster,
       score by gain/distance, publish goal to /goal_pose.

  3. VISITED-CELL COVERAGE
       When no frontiers remain, drive to the nearest unvisited
       free cell. Fills gaps and completes coverage.

  4. RETURN TO ORIGIN (optional)
       When coverage complete, drive back to captured origin.

Communication:
  Publishes PoseStamped goals to /goal_pose, which goal_pose_nav.py
  picks up and forwards to the Nav2 planner and controller.

  Blind lead-in publishes Twist on /cmd_vel_nav (through the smoother).
  Emergency stop publishes Twist on /cmd_vel (bypasses the smoother so
  it overrides Nav2's controller).

  Optional spin-on-goal rotates the robot 360 degrees after each
  successful goal, for tour capture.

Frontier choice (phase 2):
  * Distance is the path distance: one BFS over traversable cells
    (cost < traversable_max_cost) from the robot per selection, so a
    frontier behind a wall scores by the way round, and unreachable
    frontiers are dropped.
  * The goal is not the cluster centroid (often in unknown space on curved
    frontiers) but the reachable cell near it with the lowest cost, i.e.
    the most clearance, kept 2 cells back from unknown space.
  * The goal faces the frontier, so the robot arrives looking at the
    unknown area instead of turning to +x.
  * While driving, the goal is dropped as soon as its frontier has been
    seen (no frontier cells left near it) and the next one is chosen:
    the LiDAR usually clears a frontier long before the robot gets there.
  * Abandoned goals are blacklisted by position (frontier_blacklist_radius)
    for frontier_blacklist_s, so an unreachable frontier is not retried
    at once even if its cluster shape changes.

Nothing blocks the executor: the stop burst and the spin-on-goal run from
a 20 Hz timer, so TF, map updates and STOP are handled during them.
"""
import math
import time
from collections import deque
from dataclasses import dataclass, field
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


# ---------------------------------------------------------------------------
# Tunables
# ---------------------------------------------------------------------------
DEFAULTS = {
    'robot_base_frame': 'base_link',
    'map_frame':        'lidar3d_map',
    'map_topic':        '/global_costmap/costmap',
    'resume_topic':     '/explore/resume',
    'status_topic':     '/explore/status',
    'goal_topic':       '/goal_pose',
    'cmd_vel_topic':    '/cmd_vel_nav',
    'stop_topic':       '/cmd_vel',

    # Frontier detection
    'min_frontier_size':     4,
    'min_known_cells':       200,
    'frontier_blacklist_s':  30.0,
    'frontier_blacklist_radius': 0.6,   # m around an abandoned goal
    'traversable_max_cost':  90,        # costmap value; 99+ inscribed / lethal
    'goal_snap_radius':      1.0,       # m around the frontier centroid
    'frontier_gone_radius':  0.6,       # m: no frontier cells left -> re-plan
    'replan_on_explored':    True,

    # Goal tracking
    'goal_xy_tolerance':     0.35,
    'goal_timeout_s':        60.0,
    'progress_timeout_s':    20.0,
    'progress_min_m':        0.15,

    # Blind lead-in
    'blind_enable':          False,
    'blind_trigger_cells':   100,
    'blind_step_distance':   0.6,
    'blind_step_speed':      0.12,
    'blind_step_timeout':    20.0,
    'blind_max_attempts':    4,

    # Coverage
    'coverage_enable':       False,
    'coverage_min_dist':     0.6,
    'visited_radius_m':      0.4,

    # Return-to-init
    'return_to_init':        False,
    'return_xy_tolerance':   0.3,
    'return_timeout_s':      120.0,

    # Spin-on-goal capture (Phase 1 tour)
    'spin_on_goal':          False,
    'spin_rate_rad_s':       0.4,

    # Loop timing
    'tick_hz':               1.0,
}


@dataclass
class Frontier:
    cx: float            # cluster centroid (world)
    cy: float
    size: int            # cells
    distance: float      # path distance from the robot (m)
    gx: float = 0.0      # goal (world): reachable, clear, near the centroid
    gy: float = 0.0
    yaw: float = 0.0     # goal heading: toward the frontier
    cells: list = field(default_factory=list)


# ---------------------------------------------------------------------------
# Grid helpers (pure: no ROS, tested off-robot)
# ---------------------------------------------------------------------------
def neighbours4(mask):
    """True where any 4-neighbour of a cell is True (no wrap-around)."""
    p = np.pad(mask, 1, mode='constant', constant_values=False)
    return p[:-2, 1:-1] | p[2:, 1:-1] | p[1:-1, :-2] | p[1:-1, 2:]


def frontier_mask(grid):
    """Free cells (cost 0) next to unknown (-1)."""
    return (grid == 0) & neighbours4(grid == -1)


def clusters(mask, min_size):
    """4-connected clusters of a mask: lists of (row, col), >= min_size."""
    ys, xs = np.where(mask)
    used = np.zeros_like(mask, dtype=bool)
    out = []
    h, w = mask.shape
    for y0, x0 in zip(ys.tolist(), xs.tolist()):
        if used[y0, x0]:
            continue
        stack, pts = [(y0, x0)], []
        used[y0, x0] = True
        while stack:
            y, x = stack.pop()
            pts.append((y, x))
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not used[ny, nx]:
                    used[ny, nx] = True
                    stack.append((ny, nx))
        if len(pts) >= min_size:
            out.append(pts)
    return out


def bfs_distances(trav, start):
    """Cells-to-start path length over traversable cells (4-connected), -1 if unreachable."""
    h, w = trav.shape
    dist = np.full(h * w, -1, dtype=np.int32)
    flat = trav.ravel()
    s = start[0] * w + start[1]
    if not flat[s]:
        return dist.reshape(h, w)
    dist[s] = 0
    q = deque([s])
    while q:
        i = q.popleft()
        d = dist[i] + 1
        c = i % w
        for j in ((i - 1) if c > 0 else -1, (i + 1) if c < w - 1 else -1, i - w, i + w):
            if 0 <= j < h * w and flat[j] and dist[j] < 0:
                dist[j] = d
                q.append(j)
    return dist.reshape(h, w)


def nearest_true(mask, cell, radius):
    """Closest True cell to `cell` within a square radius, or None."""
    r0, c0 = cell
    h, w = mask.shape
    y0, y1 = max(0, r0 - radius), min(h, r0 + radius + 1)
    x0, x1 = max(0, c0 - radius), min(w, c0 + radius + 1)
    ys, xs = np.nonzero(mask[y0:y1, x0:x1])
    if len(ys) == 0:
        return None
    k = int(np.argmin((ys + y0 - r0) ** 2 + (xs + x0 - c0) ** 2))
    return (int(ys[k] + y0), int(xs[k] + x0))


def choose_goal(grid, reach, near_unknown, cells, centroid, radius_cells):
    """Goal cell for a frontier: reachable, away from unknown, lowest cost
    (= most clearance in a Nav2 costmap), close to the centroid. Falls back
    to the nearest reachable frontier cell. None if unreachable."""
    cr, cc = centroid
    h, w = grid.shape
    r0, r1 = max(0, int(cr) - radius_cells), min(h, int(cr) + radius_cells + 1)
    c0, c1 = max(0, int(cc) - radius_cells), min(w, int(cc) + radius_cells + 1)
    win = (reach[r0:r1, c0:c1] >= 0) & ~near_unknown[r0:r1, c0:c1]
    ys, xs = np.nonzero(win)
    if len(ys):
        cost = grid[r0:r1, c0:c1][ys, xs].astype(float)
        d = np.hypot(ys + r0 - cr, xs + c0 - cc)
        ok = d <= radius_cells
        if ok.any():
            score = np.where(ok, cost / 100.0 * radius_cells * 2 + d, np.inf)
            k = int(np.argmin(score))
            return (int(ys[k] + r0), int(xs[k] + c0))
    reachable = [(r, c) for r, c in cells if reach[r, c] >= 0]
    if not reachable:
        return None
    return min(reachable, key=lambda rc: reach[rc[0], rc[1]])


def find_frontiers(grid, res, origin, robot_xy, min_size=4, trav_max_cost=90,
                   snap_radius=1.0, skip=None):
    """Frontiers the robot can reach: path distance from one BFS, a clear
    goal near each centroid, heading toward the frontier. skip(x, y) -> True
    drops a frontier (blacklist)."""
    ox, oy = origin
    fmask = frontier_mask(grid)
    if not fmask.any():
        return []
    trav = (grid >= 0) & (grid < trav_max_cost)
    h, w = grid.shape
    rc = (int((robot_xy[1] - oy) / res), int((robot_xy[0] - ox) / res))
    rc = (min(max(rc[0], 0), h - 1), min(max(rc[1], 0), w - 1))
    if not trav[rc]:                      # robot cell inflated: nearest free cell
        rc = nearest_true(trav, rc, max(2, int(0.5 / res)))
        if rc is None:
            return []
    reach = bfs_distances(trav, rc)
    near_unknown = neighbours4(neighbours4(grid == -1))   # 2 cells from unknown
    radius_cells = max(1, int(snap_radius / res))

    out = []
    for cells in clusters(fmask, min_size):
        d = [reach[r, c] for r, c in cells if reach[r, c] >= 0]
        if not d:
            continue                      # unreachable
        cr = float(np.mean([p[0] for p in cells]))
        cc = float(np.mean([p[1] for p in cells]))
        cx, cy = ox + (cc + 0.5) * res, oy + (cr + 0.5) * res
        if skip and skip(cx, cy):
            continue
        g = choose_goal(grid, reach, near_unknown, cells, (cr, cc), radius_cells)
        if g is None:
            continue
        gx, gy = ox + (g[1] + 0.5) * res, oy + (g[0] + 0.5) * res
        if skip and skip(gx, gy):
            continue
        yaw = math.atan2(cy - gy, cx - gx) if math.hypot(cx - gx, cy - gy) > 0.1 \
            else math.atan2(gy - robot_xy[1], gx - robot_xy[0])
        out.append(Frontier(cx, cy, len(cells), min(d) * res, gx, gy, yaw, cells))
    return out


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)

        p = lambda name: self.get_parameter(name).value
        self.robot_frame = p('robot_base_frame')
        self.map_frame = p('map_frame')
        self.map_topic = p('map_topic')
        self.goal_topic = p('goal_topic')
        self.cmd_vel_topic = p('cmd_vel_topic')
        self.stop_topic = p('stop_topic')

        self.min_frontier_size = int(p('min_frontier_size'))
        self.min_known_cells = int(p('min_known_cells'))
        self.blacklist_s = float(p('frontier_blacklist_s'))
        self.blacklist_radius = float(p('frontier_blacklist_radius'))
        self.trav_max_cost = int(p('traversable_max_cost'))
        self.goal_snap_radius = float(p('goal_snap_radius'))
        self.frontier_gone_radius = float(p('frontier_gone_radius'))
        self.replan_on_explored = bool(p('replan_on_explored'))

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

        self.return_to_init = bool(p('return_to_init'))
        self.return_tol = float(p('return_xy_tolerance'))
        self.return_timeout = float(p('return_timeout_s'))

        self.spin_on_goal = bool(p('spin_on_goal'))
        self.spin_rate_rad_s = float(p('spin_rate_rad_s'))

        self.tick_hz = float(p('tick_hz'))

        # State
        self.map: Optional[OccupancyGrid] = None
        self.enabled = False
        self.state = 'IDLE'
        self.origin_xy: Optional[Tuple[float, float]] = None
        self.blacklist = {}      # (x, y) world of an abandoned frontier -> expiry
        self.empty_frontier_ticks = 0
        self.goals_sent = 0
        self.goals_succeeded = 0
        self.goals_failed = 0
        self.replans = 0
        self.blind_attempts = 0

        # Non-blocking stop burst and spin (driven by the 20 Hz timer)
        self.stop_burst = 0
        self.spin_until = 0.0
        self.spin_rate = 0.0

        # Active goal tracking
        self.active_goal_xy: Optional[Tuple[float, float]] = None
        self.active_goal_tag = None      # frontier centroid (world) of a frontier goal
        self.active_goal_start = 0.0
        self.active_goal_last_progress_xy: Optional[Tuple[float, float]] = None
        self.active_goal_last_progress_time = 0.0

        # Blind lead-in state
        self.blind_active = False
        self.blind_start_xy: Optional[Tuple[float, float]] = None
        self.blind_start_time = 0.0

        # Visited grid
        self.visited: Optional[np.ndarray] = None

        # QoS for costmap: match Nav2's costmap publisher
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.on_map, map_qos)
        self.resume_sub = self.create_subscription(
            Bool, p('resume_topic'), self.on_resume, 10)

        # Publishers
        self.status_pub = self.create_publisher(String, p('status_topic'), 10)
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        # Separate publisher for emergency stop, on /cmd_vel, so it
        # overrides Nav2's controller output on /cmd_vel_nav.
        self.stop_pub = self.create_publisher(Twist, self.stop_topic, 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timers: decisions at tick_hz; stop burst and spin at 20 Hz
        self.create_timer(1.0 / self.tick_hz, self.tick)
        self.create_timer(0.05, self.fast_tick)

        self.get_logger().info(
            f'frontier_explorer ready: '
            f'goal={self.goal_topic} cmd_vel={self.cmd_vel_topic} '
            f'stop={self.stop_topic} '
            f'map_topic={self.map_topic} map_frame={self.map_frame} '
            f'min_frontier_size={self.min_frontier_size} '
            f'blind={self.blind_enable} coverage={self.coverage_enable} '
            f'return_to_init={self.return_to_init} '
            f'spin_on_goal={self.spin_on_goal}')

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
            self.stop_spin()
            self.emergency_stop()
            self.cancel_active_goal()
            self.stop_blind()
            self.state = 'IDLE'
        self.publish_status()

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def tick(self):
        if not self.enabled or self.state in ('IDLE', 'DONE'):
            return
        if self.map is None:
            return
        if self.spinning():
            return          # spin-on-goal in progress (fast_tick drives it)

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
    # Emergency stop
    # ------------------------------------------------------------------
    def emergency_stop(self, blocking=False):
        """Publish zero velocity on /cmd_vel (bypasses the smoother).

        Nav2's controller publishes on /cmd_vel_nav, which the smoother
        relays to /cmd_vel. Ten stops over 0.5 s briefly dominate the wire
        and bring the robot to a stop. The first goes out now, the rest from
        fast_tick, so the callback returns at once (blocking only at shutdown,
        when no timer runs any more).
        """
        stop = Twist()
        if blocking:
            for _ in range(10):
                self.stop_pub.publish(stop)
                time.sleep(0.05)
            return
        self.stop_pub.publish(stop)
        self.stop_burst = 9

    def fast_tick(self):
        """20 Hz: remaining stop burst, then spin-on-goal velocity."""
        if self.stop_burst > 0:
            self.stop_burst -= 1
            self.stop_pub.publish(Twist())
            return
        if self.spin_until:
            if time.time() < self.spin_until and self.enabled:
                twist = Twist()
                twist.angular.z = self.spin_rate
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_spin()
                self.get_logger().info('spin_360: complete')

    # ------------------------------------------------------------------
    # Phase 1: Blind lead-in
    # ------------------------------------------------------------------
    def start_blind_step(self, robot_pose):
        self.blind_active = True
        self.blind_start_xy = robot_pose
        self.blind_start_time = time.time()
        self.get_logger().info(
            f'BLIND step {self.blind_attempts + 1}/{self.blind_max_attempts}: '
            f'driving {self.blind_step_distance} m at '
            f'{self.blind_step_speed} m/s')

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
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def stop_blind(self):
        if not self.blind_active:
            return
        self.cmd_vel_pub.publish(Twist())
        self.blind_active = False
        self.blind_start_xy = None
        self.blind_start_time = 0.0

    # ------------------------------------------------------------------
    # Phase 2 & 3: Frontier + Coverage
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

        # PHASE 1: Blind lead-in if the map is too sparse
        if (self.blind_enable
                and known_cells < self.blind_trigger_cells
                and self.blind_attempts < self.blind_max_attempts):
            self.start_blind_step(robot_pose)
            return

        # Require a minimum number of known cells before trusting
        # frontier detection.
        if known_cells < self.min_known_cells:
            self.get_logger().info(
                f'Waiting for map to grow: {known_cells}/{self.min_known_cells}',
                throttle_duration_sec=5.0)
            return

        # PHASE 2: Frontier exploration
        frontiers = self.detect_frontiers(grid, robot_pose)
        self.get_logger().info(
            f'detect_frontiers: known={known_cells} '
            f'frontiers={len(frontiers)}',
            throttle_duration_sec=5.0)

        if frontiers:
            self.empty_frontier_ticks = 0
            frontiers.sort(
                key=lambda f: f.size / max(f.distance, 0.5), reverse=True)
            best = frontiers[0]
            self.get_logger().info(
                f'FRONTIER: ({best.cx:.2f}, {best.cy:.2f}) '
                f'size={best.size} path={best.distance:.2f}m '
                f'goal=({best.gx:.2f}, {best.gy:.2f}) '
                f'({len(frontiers)} candidates)')
            self.publish_goal(best.gx, best.gy, tag=(best.cx, best.cy), yaw=best.yaw)
            return

        # No frontiers
        self.empty_frontier_ticks += 1

        # PHASE 3: Visited-cell coverage
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

        # PHASE 4: Nothing left to do
        if self.empty_frontier_ticks >= 5:
            self.get_logger().info('Exploration complete')
            self.state = 'RETURNING' if self.return_to_init else 'DONE'
            self.empty_frontier_ticks = 0
            self.publish_status()

    # ------------------------------------------------------------------
    # Phase 4: Return to origin
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
            self.get_logger().info('Returned to origin — DONE')
            self.state = 'DONE'
            self.publish_status()
            return

        self.get_logger().info(
            f'RETURN: to origin ({self.origin_xy[0]:.2f}, '
            f'{self.origin_xy[1]:.2f}), {dist:.2f}m away')
        self.publish_goal(self.origin_xy[0], self.origin_xy[1], tag=None)

    # ------------------------------------------------------------------
    # Goal publishing and tracking
    # ------------------------------------------------------------------
    def publish_goal(self, x, y, tag=None, yaw=None):
        """Send a goal. tag = frontier centroid for frontier goals (re-plan
        and blacklist use it). yaw None = face along robot -> goal."""
        if self.active_goal_xy is not None:
            return
        if yaw is None:
            pose = self.get_robot_pose()
            yaw = math.atan2(y - pose[1], x - pose[0]) if pose else 0.0

        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

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
            if self.spin_on_goal:
                self.start_spin()
            return

        # Frontier already seen from here: choose the next one now instead
        # of walking all the way to a place that is no longer unknown.
        if self.replan_on_explored and self.active_goal_tag is not None \
                and self.frontier_explored(self.active_goal_tag):
            self.replans += 1
            self.get_logger().info(
                f'Frontier at ({self.active_goal_tag[0]:.2f}, {self.active_goal_tag[1]:.2f}) '
                f'explored before arrival: re-planning')
            self.clear_active_goal()
            self.publish_status()
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
        """Abandoned frontier: skip frontiers near it for blacklist_s."""
        if self.active_goal_tag is not None:
            self.blacklist[tuple(self.active_goal_tag)] = time.time() + self.blacklist_s
            self.blacklist[tuple(self.active_goal_xy)] = time.time() + self.blacklist_s

    def blacklisted(self, x, y):
        return any(math.hypot(x - bx, y - by) < self.blacklist_radius for bx, by in self.blacklist)

    def frontier_explored(self, centroid):
        """True if no frontier cell is left within frontier_gone_radius."""
        grid = self.to_numpy(self.map)
        if grid is None:
            return False
        res = self.map.info.resolution
        ox, oy = self.map.info.origin.position.x, self.map.info.origin.position.y
        c, r = int((centroid[0] - ox) / res), int((centroid[1] - oy) / res)
        rad = max(1, int(self.frontier_gone_radius / res))
        h, w = grid.shape
        y0, y1, x0, x1 = max(0, r - rad), min(h, r + rad + 1), max(0, c - rad), min(w, c + rad + 1)
        if y0 >= y1 or x0 >= x1:
            return False     # off the current map: cannot tell
        # Pad by one so the window's own unknown border counts.
        win = grid[max(0, y0 - 1):min(h, y1 + 1), max(0, x0 - 1):min(w, x1 + 1)]
        return not frontier_mask(win).any()

    def clear_active_goal(self):
        self.active_goal_xy = None
        self.active_goal_tag = None
        self.active_goal_last_progress_xy = None

    # ------------------------------------------------------------------
    # Spin-on-goal capture
    # ------------------------------------------------------------------
    def start_spin(self, rate=None):
        """Rotate 360 degrees in place for tour capture (non-blocking).

        fast_tick publishes the Twist on /cmd_vel_nav at 20 Hz for one
        revolution at `rate` (spin_rate_rad_s when None); tick() picks no
        new goal meanwhile, and STOP ends the spin at once.
        """
        if rate is None:
            rate = self.spin_rate_rad_s
        if rate <= 0:
            self.get_logger().warn('spin_360: rate must be > 0, skipping')
            return
        duration = 2.0 * math.pi / rate
        self.get_logger().info(
            f'spin_360: starting, rate={rate:.2f} rad/s, '
            f'duration={duration:.1f}s')
        self.spin_rate = rate
        self.spin_until = time.time() + duration

    def spinning(self):
        return self.spin_until > 0.0

    def stop_spin(self):
        if self.spin_until:
            self.spin_until = 0.0
            self.cmd_vel_pub.publish(Twist())

    def cancel_active_goal(self):
        if self.active_goal_xy is not None:
            self.get_logger().info('Cancelling active goal')
            self.emergency_stop()
            self.clear_active_goal()

    # ------------------------------------------------------------------
    # Visited grid
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

        x0 = max(0, cx - r)
        x1 = min(grid.shape[1], cx + r + 1)
        y0 = max(0, cy - r)
        y1 = min(grid.shape[0], cy + r + 1)
        if x0 < x1 and y0 < y1:
            self.visited[y0:y1, x0:x1] = True

    def nearest_unvisited(self, robot_xy):
        if self.visited is None or self.map is None:
            return None
        grid = self.to_numpy(self.map)
        if grid is None or grid.shape != self.visited.shape:
            return None

        free = (grid == 0)
        unvisited_free = free & ~self.visited
        ys, xs = np.where(unvisited_free)
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
    # Frontier detection
    # ------------------------------------------------------------------
    def to_numpy(self, occ):
        if not occ.data:
            return None
        w, h = occ.info.width, occ.info.height
        return np.array(occ.data, dtype=np.int8).reshape(h, w)

    def detect_frontiers(self, grid, robot_xy):
        """Reachable frontiers with path distance, snapped goal and heading."""
        now = time.time()
        self.blacklist = {k: v for k, v in self.blacklist.items() if v > now}

        res = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y
        return find_frontiers(grid, res, (ox, oy), robot_xy,
                              min_size=self.min_frontier_size,
                              trav_max_cost=self.trav_max_cost,
                              snap_radius=self.goal_snap_radius,
                              skip=self.blacklisted)

    # ------------------------------------------------------------------
    # Helpers
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
            f'replans={self.replans} '
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
        node.stop_spin()
        node.emergency_stop(blocking=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()