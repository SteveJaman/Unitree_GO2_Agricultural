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
"""
import math
import signal
import time
from dataclasses import dataclass
from typing import Optional, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy)
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from tf2_ros import Buffer, TransformListener

try:
    from scipy.ndimage import distance_transform_edt
except ImportError:  # wall clearance checks are skipped without scipy
    distance_transform_edt = None

# OccupancyGrid value of a lethal costmap cell (an obstacle seen by the lidar).
LETHAL = 100


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
    # Ignore frontiers closer than this. Must exceed goal_xy_tolerance, or a
    # goal counts as reached the moment it is sent and the robot never moves.
    # The lidar blind ring (obstacle_min_range 0.35 m) always leaves unknown
    # cells right around the robot, so without this the nearest "frontier"
    # is under the robot itself.
    'min_goal_dist':         0.8,
    'min_known_cells':       200,
    'frontier_blacklist_s':  30.0,
    # A frontier whose centroid is within this distance of an abandoned goal
    # is skipped. Keyed in world coordinates because the rolling costmap
    # origin moves, so pixel keys do not survive between ticks.
    'frontier_blacklist_radius_m': 0.5,

    # Goal tracking
    'goal_xy_tolerance':     0.35,
    'goal_timeout_s':        60.0,
    'progress_timeout_s':    20.0,
    'progress_min_m':        0.15,
    # Nav2 actions that go2_goal_pose_nav drives. Their status topics tell
    # us when a leg fails, so we can move on without waiting for
    # progress_timeout_s. Empty string disables the watch.
    'planner_action':        '/compute_path_to_pose',
    'controller_action':     '/follow_path',
    # If the planner fails because the robot itself sits in lethal or
    # inscribed space (costmap value >= this), no goal can succeed, so the
    # goal is not blacklisted. Instead we wait stuck_backoff_s and,
    # if unstick_enable, reverse a little (like Nav2's backup recovery).
    'stuck_cost_threshold':  99,
    'stuck_backoff_s':       5.0,
    'unstick_enable':        False,
    'unstick_distance':      0.3,
    'unstick_speed':         0.1,
    'unstick_timeout_s':     6.0,

    # Wall clearance. Distances are to lethal costmap cells (real obstacles).
    # Robot footprint in base_link, same as go2_nav2/config/go2_footprint.yaml.
    'footprint_front':       0.36,
    'footprint_back':        0.45,
    'footprint_half_width':  0.20,
    # A frontier's goal is moved off the frontier into known free space, to
    # the nearest cell within goal_search_radius_m that is at least
    # goal_clearance_m from any wall and goal_unknown_clearance_m from
    # unknown space. A goal right on the frontier can sit next to a wall the
    # lidar has not seen yet. Frontiers with no such cell are skipped.
    'goal_clearance_m':          0.6,
    'goal_unknown_clearance_m':  0.3,
    'goal_search_radius_m':      1.5,
    # While driving: if a newly seen wall comes within this of the goal,
    # cancel the Nav2 leg and pick another frontier.
    'goal_unsafe_clearance_m':   0.4,
    # Nav2's rotation shim turns in place when a new path starts more than
    # turn_threshold_rad off the robot's heading. Turning sweeps the
    # footprint's far corner (~0.53 m), so when the robot is closer than
    # turn_clearance_m to a wall, prefer frontiers that need no such turn.
    'turn_clearance_m':          0.6,
    'turn_threshold_rad':        0.8,

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


def clearance_maps(grid, res):
    """Distance in metres from every cell to the nearest wall (lethal cell)
    and to the nearest unknown cell. inf where there is none."""
    out = []
    for blocked in (grid >= LETHAL, grid == -1):
        if blocked.any():
            out.append(distance_transform_edt(~blocked) * res)
        else:
            out.append(np.full(grid.shape, np.inf))
    return out[0], out[1]


def snap_goal(grid, clear_wall, clear_unknown, ox, oy, res, x, y,
              min_wall, min_unknown, radius):
    """Nearest free cell to (x, y), within radius, that is at least min_wall
    from walls and min_unknown from unknown space. World (x, y) or None."""
    h, w = grid.shape
    ci, cj = int((y - oy) / res), int((x - ox) / res)
    r = int(math.ceil(radius / res))
    i0, i1 = max(0, ci - r), min(h, ci + r + 1)
    j0, j1 = max(0, cj - r), min(w, cj + r + 1)
    if i0 >= i1 or j0 >= j1:
        return None
    ii, jj = np.mgrid[i0:i1, j0:j1]
    d2 = (ii - ci) ** 2 + (jj - cj) ** 2
    ok = ((grid[i0:i1, j0:j1] == 0)
          & (clear_wall[i0:i1, j0:j1] >= min_wall)
          & (clear_unknown[i0:i1, j0:j1] >= min_unknown)
          & (d2 <= r * r))
    if not ok.any():
        return None
    k = int(np.argmin(np.where(ok, d2, np.iinfo(d2.dtype).max)))
    return (ox + (jj.flat[k] + 0.5) * res, oy + (ii.flat[k] + 0.5) * res)


def angle_diff(a, b):
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


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
        self.map_topic = p('map_topic')
        self.goal_topic = p('goal_topic')
        self.cmd_vel_topic = p('cmd_vel_topic')
        self.stop_topic = p('stop_topic')

        self.min_frontier_size = int(p('min_frontier_size'))
        self.min_goal_dist = float(p('min_goal_dist'))
        self.min_known_cells = int(p('min_known_cells'))
        self.blacklist_s = float(p('frontier_blacklist_s'))
        self.blacklist_radius = float(p('frontier_blacklist_radius_m'))

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

        self.stuck_cost = int(p('stuck_cost_threshold'))
        self.stuck_backoff = float(p('stuck_backoff_s'))
        self.unstick_enable = bool(p('unstick_enable'))
        self.unstick_distance = float(p('unstick_distance'))
        self.unstick_speed = float(p('unstick_speed'))
        self.unstick_timeout = float(p('unstick_timeout_s'))

        self.fp_front = float(p('footprint_front'))
        self.fp_back = float(p('footprint_back'))
        self.fp_half_w = float(p('footprint_half_width'))
        self.goal_clearance = float(p('goal_clearance_m'))
        self.goal_unknown_clearance = float(p('goal_unknown_clearance_m'))
        self.goal_search_radius = float(p('goal_search_radius_m'))
        self.goal_unsafe_clearance = float(p('goal_unsafe_clearance_m'))
        self.turn_clearance = float(p('turn_clearance_m'))
        self.turn_threshold = float(p('turn_threshold_rad'))

        self.tick_hz = float(p('tick_hz'))

        # State
        self.map: Optional[OccupancyGrid] = None
        self.enabled = False
        self.state = 'IDLE'
        self.origin_xy: Optional[Tuple[float, float]] = None
        # (x, y, expiry) in map_frame world coordinates
        self.blacklist: List[Tuple[float, float, float]] = []
        self.empty_frontier_ticks = 0
        self.goals_sent = 0
        self.goals_succeeded = 0
        self.goals_failed = 0
        self.blind_attempts = 0

        # Active goal tracking
        self.active_goal_xy: Optional[Tuple[float, float]] = None
        self.active_goal_tag = None
        self.active_goal_start = 0.0
        self.active_goal_last_progress_xy: Optional[Tuple[float, float]] = None
        self.active_goal_last_progress_time = 0.0

        # Nav2 leg tracking. go2_goal_pose_nav publishes no result, so we
        # watch the planner/controller action status. Goal IDs seen before
        # our goal was sent belong to an older leg and are ignored. A
        # controller goal counts as ours only after a new planner goal
        # appeared, because an older leg queued ahead of ours can still be
        # planning when we publish.
        self.nav_seen_ids = set()
        self.leg_baseline_ids = set()
        self.leg_planner_seen = False

        # Stuck handling: no new goal before this time
        self.next_goal_time = 0.0
        self.unstick_start_xy: Optional[Tuple[float, float]] = None
        self.unstick_start_time = 0.0
        self.unstick_reason = ''

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
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        for kind in ('planner', 'controller'):
            action = p(f'{kind}_action')
            if action:
                self.create_subscription(
                    GoalStatusArray, f'{action}/_action/status',
                    lambda msg, kind=kind: self.on_nav_status(kind, msg),
                    status_qos)

        # Publishers
        self.status_pub = self.create_publisher(String, p('status_topic'), 10)
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        # Separate publisher for emergency stop, on /cmd_vel, so it
        # overrides Nav2's controller output on /cmd_vel_nav.
        self.stop_pub = self.create_publisher(Twist, self.stop_topic, 10)
        # Cancelling through the action servers is the only way to stop a
        # leg go2_goal_pose_nav already started: it does not preempt on a new
        # /goal_pose, and a zero Twist only pauses the controller.
        self.cancel_clients = [
            self.create_client(CancelGoal, f'{p(k)}/_action/cancel_goal')
            for k in ('planner_action', 'controller_action') if p(k)]

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer
        self.create_timer(1.0 / self.tick_hz, self.tick)
        # The velocity smoother times out after ~1 s, so reverse at 10 Hz.
        self.create_timer(0.1, self.tick_unstick)

        self.get_logger().info(
            f'frontier_explorer ready: '
            f'goal={self.goal_topic} cmd_vel={self.cmd_vel_topic} '
            f'stop={self.stop_topic} '
            f'map_topic={self.map_topic} map_frame={self.map_frame} '
            f'min_frontier_size={self.min_frontier_size} '
            f'min_goal_dist={self.min_goal_dist} '
            f'blacklist_radius={self.blacklist_radius} '
            f'unstick={self.unstick_enable} '
            f'goal_clearance={self.goal_clearance} '
            f'clearance_checks={distance_transform_edt is not None} '
            f'blind={self.blind_enable} coverage={self.coverage_enable} '
            f'return_to_init={self.return_to_init} '
            f'spin_on_goal={self.spin_on_goal}')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def on_map(self, msg: OccupancyGrid):
        self.map = msg

    def on_nav_status(self, kind: str, msg: GoalStatusArray):
        terminal = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                    GoalStatus.STATUS_CANCELED)
        for st in msg.status_list:
            gid = (kind, bytes(st.goal_info.goal_id.uuid))
            self.nav_seen_ids.add(gid)
            if self.active_goal_xy is None or gid in self.leg_baseline_ids:
                continue
            if kind == 'planner':
                self.leg_planner_seen = True
            elif not self.leg_planner_seen:
                # Controller goal of an older leg that was still planning
                self.leg_baseline_ids.add(gid)
                continue
            if st.status not in terminal:
                continue
            if kind == 'planner' and st.status == GoalStatus.STATUS_SUCCEEDED:
                # Planning done; wait for follow_path
                self.leg_baseline_ids.add(gid)
                continue
            self.on_leg_done(kind, st.status)
            return

    def on_leg_done(self, kind: str, status: int):
        x, y = self.active_goal_xy
        pose = self.get_robot_pose()
        dist = (math.hypot(pose[0] - x, pose[1] - y)
                if pose is not None else float('nan'))
        if status == GoalStatus.STATUS_SUCCEEDED:
            # Nav2's goal checker decided we arrived. Its tolerance can be
            # looser than goal_xy_tolerance, so trust it rather than wait
            # progress_timeout_s for the last few centimetres.
            self.goals_succeeded += 1
            self.get_logger().info(
                f'Goal reached: ({x:.2f}, {y:.2f}) '
                f'(Nav2 succeeded, {dist:.2f} m off)')
            self.clear_active_goal()
            if self.spin_on_goal:
                self.spin_360()
            return
        name = 'ABORTED' if status == GoalStatus.STATUS_ABORTED else 'CANCELED'
        cost = self.cost_at(pose) if pose is not None else None
        stuck = (kind == 'planner' and cost is not None
                 and cost >= self.stuck_cost)
        if not stuck:
            self.blacklist_tag()
        self.goals_failed += 1
        self.clear_active_goal()
        if stuck:
            # "Starting point in lethal space": the goal is not at fault.
            self.get_logger().warn(
                f'Nav2 planner ABORTED for ({x:.2f}, {y:.2f}): robot is in '
                f'lethal space (cost {cost}) — not blacklisting')
            self.next_goal_time = time.time() + self.stuck_backoff
            if self.unstick_enable:
                self.start_unstick(pose)
            return
        self.get_logger().warn(
            f'Nav2 {kind} {name} for ({x:.2f}, {y:.2f}), {dist:.2f} m away '
            f'— abandoning goal')

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
            self.cancel_nav()
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

        if self.blind_active:
            self.tick_blind()
            return

        if self.unstick_start_xy is not None:
            return  # tick_unstick runs on its own faster timer

        if self.active_goal_xy is not None:
            self.tick_active_goal()
            return

        if time.time() < self.next_goal_time:
            return

        if self.state == 'EXPLORING':
            self.step_explore()
        elif self.state == 'RETURNING':
            self.step_return()

    # ------------------------------------------------------------------
    # Emergency stop
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
    # Unstick: reverse out of lethal space
    # ------------------------------------------------------------------
    def start_unstick(self, pose, reason='in lethal space'):
        """Reverse a little, if the strip behind the robot is free."""
        if not self.body_strip_free(-self.fp_back - self.unstick_distance - 0.05,
                                    -self.fp_back):
            self.get_logger().warn(
                f'UNSTICK ({reason}): wall behind the robot, not reversing',
                throttle_duration_sec=10.0)
            return False
        self.unstick_start_xy = pose
        self.unstick_start_time = time.time()
        self.unstick_reason = reason
        self.get_logger().info(
            f'UNSTICK ({reason}): reversing up to {self.unstick_distance} m at '
            f'{self.unstick_speed} m/s')
        return True

    def body_strip_free(self, x0, x1):
        """True if no wall lies in the body-frame strip x0..x1 (|y| within
        the footprint half width). True when it cannot be checked."""
        pose = self.get_robot_pose_yaw()
        grid = self.to_numpy(self.map) if self.map is not None else None
        if pose is None or grid is None:
            return True
        info = self.map.info
        step = info.resolution / 2.0
        bx, by = np.meshgrid(np.arange(x0, x1 + 1e-6, step),
                             np.arange(-self.fp_half_w, self.fp_half_w + 1e-6, step))
        c, s_ = math.cos(pose[2]), math.sin(pose[2])
        wx = pose[0] + c * bx - s_ * by
        wy = pose[1] + s_ * bx + c * by
        j = ((wx - info.origin.position.x) / info.resolution).astype(int)
        i = ((wy - info.origin.position.y) / info.resolution).astype(int)
        inside = (i >= 0) & (i < grid.shape[0]) & (j >= 0) & (j < grid.shape[1])
        return not bool(np.any(grid[i[inside], j[inside]] >= LETHAL))

    def tick_unstick(self):
        if self.unstick_start_xy is None:
            return
        pose = self.get_robot_pose()
        elapsed = time.time() - self.unstick_start_time
        travelled = (math.hypot(pose[0] - self.unstick_start_xy[0],
                                pose[1] - self.unstick_start_xy[1])
                     if pose is not None else 0.0)
        cost = self.cost_at(pose) if pose is not None else None
        if (pose is None or travelled >= self.unstick_distance
                or elapsed >= self.unstick_timeout
                or (cost is not None and cost < self.stuck_cost
                    and self.unstick_reason == 'in lethal space')
                or not self.body_strip_free(-self.fp_back - 0.1, -self.fp_back)):
            self.cmd_vel_pub.publish(Twist())
            self.unstick_start_xy = None
            self.next_goal_time = time.time() + 1.0
            self.get_logger().info(
                f'UNSTICK done: {travelled:.2f} m in {elapsed:.1f}s, '
                f'cost now {cost}')
            return
        twist = Twist()
        twist.linear.x = -self.unstick_speed
        self.cmd_vel_pub.publish(twist)

    # ------------------------------------------------------------------
    # Phase 1: Blind lead-in
    # ------------------------------------------------------------------
    def start_blind_step(self, robot_pose):
        if not self.body_strip_free(self.fp_front, self.fp_front + self.blind_step_distance):
            self.get_logger().warn('BLIND step skipped: wall ahead')
            self.blind_attempts += 1
            return
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
                or elapsed >= self.blind_step_timeout
                or not self.body_strip_free(self.fp_front, self.fp_front + 0.15)):
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
            choice = self.choose_safe_goal(frontiers, grid, robot_pose)
            if choice is None:
                if self.unstick_start_xy is not None:
                    return  # backing off a wall before turning
                self.get_logger().info(
                    f'No frontier has a goal clear of walls '
                    f'({len(frontiers)} candidates)',
                    throttle_duration_sec=5.0)
                return
            best, gx, gy = choice
            self.get_logger().info(
                f'FRONTIER: ({best.cx:.2f}, {best.cy:.2f}) '
                f'size={best.size} dist={best.distance:.2f}m '
                f'goal=({gx:.2f}, {gy:.2f}) '
                f'({len(frontiers)} candidates)')
            self.publish_goal(gx, gy, tag=(best.cx, best.cy))
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

    def choose_safe_goal(self, frontiers, grid, robot_pose):
        """First frontier (in score order) with a goal clear of walls.

        Returns (frontier, goal_x, goal_y) or None. Without scipy the
        frontier centroid is used as before.
        """
        if distance_transform_edt is None:
            f = frontiers[0]
            return f, f.cx, f.cy
        info = self.map.info
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        clear_wall, clear_unknown = clearance_maps(grid, res)
        pose = self.get_robot_pose_yaw()
        tight = False
        if pose is not None:
            ri, rj = int((pose[1] - oy) / res), int((pose[0] - ox) / res)
            if 0 <= ri < grid.shape[0] and 0 <= rj < grid.shape[1]:
                tight = clear_wall[ri, rj] < self.turn_clearance
        safe, turning = [], []
        for f in frontiers:
            g = snap_goal(grid, clear_wall, clear_unknown, ox, oy, res, f.cx, f.cy,
                          self.goal_clearance, self.goal_unknown_clearance,
                          self.goal_search_radius)
            if g is None:
                continue
            if math.hypot(g[0] - robot_pose[0], g[1] - robot_pose[1]) < self.min_goal_dist:
                continue
            if tight and pose is not None and angle_diff(
                    math.atan2(g[1] - pose[1], g[0] - pose[0]), pose[2]) > self.turn_threshold:
                turning.append((f, g[0], g[1]))
                continue
            safe.append((f, g[0], g[1]))
            break
        if safe:
            return safe[0]
        if turning:
            # Every option needs a turn in place next to a wall. Back off
            # first if allowed, otherwise accept the turn.
            if self.unstick_enable and self.start_unstick(robot_pose, reason='too close to turn'):
                return None
            self.get_logger().warn(
                'Close to a wall and every frontier needs a turn in place',
                throttle_duration_sec=10.0)
            return turning[0]
        return None

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
    def publish_goal(self, x, y, tag=None):
        if self.active_goal_xy is not None:
            return

        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        # Face the direction of travel, so no final turn is needed if the
        # goal checker's yaw tolerance is ever tightened.
        pose = self.get_robot_pose()
        yaw = math.atan2(y - pose[1], x - pose[0]) if pose is not None else 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.leg_baseline_ids = set(self.nav_seen_ids)
        self.leg_planner_seen = False
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
                self.spin_360()
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

        wall = self.goal_wall_clearance()
        if wall is not None and wall < self.goal_unsafe_clearance:
            self.abandon_goal(f'wall {wall:.2f} m from the goal')
            return

        if now - self.active_goal_last_progress_time > self.progress_timeout:
            self.abandon_goal('no progress')
            return

        if now - self.active_goal_start > self.goal_timeout:
            self.abandon_goal('goal timeout')
            return

    def abandon_goal(self, reason):
        """Give up on the active goal and stop Nav2 from still driving it."""
        self.get_logger().warn(f'{reason} — abandoning goal')
        self.cancel_nav()
        self.blacklist_tag()
        self.goals_failed += 1
        self.clear_active_goal()

    def goal_wall_clearance(self):
        """Distance from the active goal to the nearest wall, or None."""
        if distance_transform_edt is None or self.map is None:
            return None
        info = self.map.info
        res = info.resolution
        gi = int((self.active_goal_xy[1] - info.origin.position.y) / res)
        gj = int((self.active_goal_xy[0] - info.origin.position.x) / res)
        r = int(math.ceil((self.goal_unsafe_clearance + res) / res))
        grid = self.to_numpy(self.map)
        if grid is None or not (0 <= gi < grid.shape[0] and 0 <= gj < grid.shape[1]):
            return None  # goal outside the rolling costmap: nothing to check
        win = grid[max(0, gi - r):gi + r + 1, max(0, gj - r):gj + r + 1]
        ys, xs = np.nonzero(win >= LETHAL)
        if len(xs) == 0:
            return None
        ys = ys + max(0, gi - r) - gi
        xs = xs + max(0, gj - r) - gj
        return float(np.min(np.hypot(xs, ys))) * res

    def cancel_nav(self):
        """Cancel every Nav2 planner/controller goal (go2_goal_pose_nav's leg).

        A zero goal ID and zero stamp means "all goals" in the action protocol.
        The request goes out immediately; the response is not awaited.
        """
        for cli in self.cancel_clients:
            if cli.service_is_ready():
                cli.call_async(CancelGoal.Request())

    def blacklist_tag(self):
        """Blacklist the active frontier goal (tag is its world x, y)."""
        if self.active_goal_tag is not None:
            bx, by = self.active_goal_tag
            self.blacklist.append((bx, by, time.time() + self.blacklist_s))
            self.get_logger().info(
                f'Blacklisted ({bx:.2f}, {by:.2f}) for {self.blacklist_s:.0f}s')

    def is_blacklisted(self, x, y):
        r2 = self.blacklist_radius ** 2
        return any((x - bx) ** 2 + (y - by) ** 2 < r2
                   for bx, by, _ in self.blacklist)

    def clear_active_goal(self):
        self.active_goal_xy = None
        self.active_goal_tag = None
        self.active_goal_last_progress_xy = None

    # ------------------------------------------------------------------
    # Spin-on-goal capture
    # ------------------------------------------------------------------
    def spin_360(self, rate=None):
        """Rotate 360 degrees in place for tour capture.

        Publishes continuous Twist on /cmd_vel_nav for the duration
        required to complete one full revolution at the given rate.
        Honours spin_rate_rad_s parameter when rate is None.
        """
        if rate is None:
            rate = self.spin_rate_rad_s
        if rate <= 0:
            self.get_logger().warn('spin_360: rate must be > 0, skipping')
            return

        corner = math.hypot(max(self.fp_front, self.fp_back), self.fp_half_w)
        if not self.room_to_turn(corner):
            self.get_logger().warn('spin_360: too close to a wall, skipping')
            return

        duration = 2.0 * math.pi / rate
        self.get_logger().info(
            f'spin_360: starting, rate={rate:.2f} rad/s, '
            f'duration={duration:.1f}s')

        t0 = time.time()
        twist = Twist()
        twist.angular.z = rate
        while time.time() - t0 < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        self.cmd_vel_pub.publish(Twist())

        self.get_logger().info('spin_360: complete')

    def room_to_turn(self, radius):
        """True if no wall is within radius of the robot (or unknown)."""
        pose = self.get_robot_pose()
        grid = self.to_numpy(self.map) if self.map is not None else None
        if pose is None or grid is None:
            return True
        info = self.map.info
        res = info.resolution
        ci = int((pose[1] - info.origin.position.y) / res)
        cj = int((pose[0] - info.origin.position.x) / res)
        r = int(math.ceil(radius / res))
        if not (0 <= ci < grid.shape[0] and 0 <= cj < grid.shape[1]):
            return True
        win = grid[max(0, ci - r):ci + r + 1, max(0, cj - r):cj + r + 1]
        ys, xs = np.nonzero(win >= LETHAL)
        ys = ys + max(0, ci - r) - ci
        xs = xs + max(0, cj - r) - cj
        return not bool(np.any(np.hypot(xs, ys) * res < radius))

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
        free = (grid == 0)
        unknown = (grid == -1)

        # Use np.pad instead of np.roll to avoid wrap-around.
        # np.roll treats the top row as adjacent to the bottom row,
        # which creates phantom frontiers at the map boundaries.
        padded = np.pad(unknown, 1, mode='constant', constant_values=False)
        unknown_d = (
            padded[:-2, 1:-1] | padded[2:, 1:-1] |
            padded[1:-1, :-2] | padded[1:-1, 2:])
        frontier_mask = free & unknown_d

        now = time.time()
        self.blacklist = [b for b in self.blacklist if b[2] > now]

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

            if self.is_blacklisted(cxw, cyw):
                continue

            dist = math.hypot(cxw - robot_xy[0], cyw - robot_xy[1])
            if dist < self.min_goal_dist:
                continue
            frontiers.append(Frontier(cxw, cyw, len(cluster_pts), dist))

        return frontiers

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

    def get_robot_pose_yaw(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    def cost_at(self, world_xy):
        """Costmap value (0-100, -1 unknown) at a world point, or None."""
        if self.map is None:
            return None
        info = self.map.info
        mx = int((world_xy[0] - info.origin.position.x) / info.resolution)
        my = int((world_xy[1] - info.origin.position.y) / info.resolution)
        if not (0 <= mx < info.width and 0 <= my < info.height):
            return None
        return int(self.map.data[my * info.width + mx])

    def publish_status(self):
        msg = String()
        msg.data = (
            f'state={self.state} enabled={self.enabled} '
            f'goals_sent={self.goals_sent} '
            f'succeeded={self.goals_succeeded} '
            f'failed={self.goals_failed} '
            f'blind_attempts={self.blind_attempts}')
        self.status_pub.publish(msg)


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # rclpy's own signal handler shuts the context down before our finally
    # block runs, so the final stop would fail with "publisher's context is
    # invalid". Handle SIGINT/SIGTERM ourselves and shut down after the stop.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_interrupt)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Launchers can forward a second signal while cleanup is in progress.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.stop_blind()
        node.cancel_nav()
        if node.unstick_start_xy is not None:
            node.cmd_vel_pub.publish(Twist())
        node.emergency_stop()
        node.get_logger().info(
            f'Stopped. goals sent={node.goals_sent} '
            f'reached={node.goals_succeeded} abandoned={node.goals_failed}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()