#!/usr/bin/env python3
"""
go2_stuck_recovery.py: get the Go2 unstuck when Nav2 gives up on a leg.

Watches the controller's follow_path action. When a leg is aborted (e.g.
Regulated Pure Pursuit's "detected collision ahead!" with the nose at a wall,
or "Failed to make progress"), it:

  1. checks the local costmap for a free patch behind the dog, then (with
     allow_sidestep) to its left and right;
  2. steps there: escape_distance metres at escape_speed. With --drive omni
     this goes on /cmd_vel_nav, whose smoother allows vy. With --drive hybrid
     (forward-only Nav2, whose smoother clamps vy to 0) it goes straight to
     /cmd_vel after escape_delay_s, once the smoother has stopped publishing;
  3. re-sends the last /goal_pose (the explorer's current frontier, or a 2D
     Goal Pose) so the planner tries again from the new spot, up to
     max_retries times per goal.

It also watches the planner's compute_path_to_pose action. When the planner
refuses a goal and the dog's own footprint touches inscribed or lethal cost
in the local costmap (Smac: "Starting point in lethal space"), every goal
fails from that spot: the dog stands pressed to a wall, e.g. after MPPI omni
slid it sideways. It then steps into a free patch the same way, at most
max_start_escapes times per start_escape_window. Other planner refusals (a
goal in a wall, no path) are left to the explorer.

Without this, a leg aborted at a wall leaves the dog staring at it: the goal
handler only logs "leg failed" and the explorer waits ~25 s before trying
another frontier, from the same spot.

If no direction is free, it does not move and logs why.

Run in go2-lidar3d by wifi_nav.sh in every session (--no-recovery turns it off).
"""

import math
import time

import numpy as np

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

# Escape patches in base_link (x fwd, y left), metres: (name, vx, vy, x0, x1, y0, y1).
# The footprint is x in [-0.45, 0.36], y in [-0.20, 0.20].
FOOTPRINT = (-0.45, 0.36, -0.20, 0.20)
ESCAPES = [
    ("back", -1.0, 0.0, -0.85, -0.45, -0.25, 0.25),
    ("left", 0.0, 1.0, -0.40, 0.30, 0.20, 0.55),
    ("right", 0.0, -1.0, -0.40, 0.30, -0.55, -0.20),
]


class StuckRecovery(Node):
    def __init__(self):
        super().__init__("go2_stuck_recovery")
        p = self.declare_parameter
        p("status_topic", "/follow_path/_action/status")
        p("planner_status_topic", "/compute_path_to_pose/_action/status")
        p("costmap_topic", "/local_costmap/costmap")
        p("goal_topic", "/goal_pose")
        p("cmd_topic", "/cmd_vel_nav")
        p("base_frame", "base_link")
        p("escape_distance", 0.30)
        p("escape_speed", 0.12)
        # costmap value below which a cell is free. 99 = inscribed: anything
        # lower is inflation, which in a doorway surrounds the dog (cost 50-98)
        p("free_threshold", 99)
        p("retry_goal", True)
        p("allow_sidestep", False)  # needs vy: --drive omni, or hybrid on /cmd_vel
        # Wait before moving. On /cmd_vel (hybrid) the smoother still sends its
        # stop for velocity_timeout (1 s) after the abort; moving earlier fights it.
        p("escape_delay_s", 0.0)
        p("max_retries", 2)
        # Planner refusals while the footprint is in cost >= this (99 = inscribed)
        p("start_blocked_cost", 99)
        p("max_start_escapes", 3)  # per start_escape_window
        p("start_escape_window", 60.0)  # s

        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.costmap = None
        self.goal = None
        self.retries = 0
        self.seen = {}  # goal id -> last status
        self.escape = None  # (vx, vy, start, end) while stepping
        self.resend_at = None
        self._own_goal = False  # the next /goal_pose is our own retry
        self.planner_seen = {}  # planner goal id -> last status
        self.start_escapes = []  # monotonic times of planner-triggered escapes

        self.create_subscription(
            GoalStatusArray,
            self.get_parameter("status_topic").value,
            self.on_status,
            10,
        )
        self.create_subscription(
            GoalStatusArray,
            self.get_parameter("planner_status_topic").value,
            self.on_planner_status,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("costmap_topic").value,
            lambda m: setattr(self, "costmap", m),
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(
            PoseStamped, self.get_parameter("goal_topic").value, self.on_goal, 10
        )
        self.cmd = self.create_publisher(
            Twist, self.get_parameter("cmd_topic").value, 10
        )
        self.goal_pub = self.create_publisher(
            PoseStamped, self.get_parameter("goal_topic").value, 10
        )
        self.create_timer(0.1, self.tick)
        moves = (
            "back / left / right"
            if self.get_parameter("allow_sidestep").value
            else "back"
        )
        self.get_logger().info(
            f"stuck recovery ready ({moves}, then retry goal; also when the planner "
            f"refuses goals with the dog in an obstacle)"
        )

    def on_goal(self, msg):
        if self.resend_at is None and self.escape is None and not self._own_goal:
            self.retries = 0  # a new goal from someone else
        self._own_goal = False
        self.goal = msg

    def on_status(self, msg):
        for s in msg.status_list:
            gid = bytes(s.goal_info.goal_id.uuid)
            prev = self.seen.get(gid)
            self.seen[gid] = s.status
            if (
                s.status == GoalStatus.STATUS_ABORTED
                and prev != GoalStatus.STATUS_ABORTED
            ):
                self.on_abort()
        if len(self.seen) > 50:
            self.seen = dict(list(self.seen.items())[-20:])

    def on_planner_status(self, msg):
        for s in msg.status_list:
            gid = bytes(s.goal_info.goal_id.uuid)
            prev = self.planner_seen.get(gid)
            self.planner_seen[gid] = s.status
            if (
                s.status == GoalStatus.STATUS_ABORTED
                and prev != GoalStatus.STATUS_ABORTED
            ):
                self.on_planner_abort()
        if len(self.planner_seen) > 50:
            self.planner_seen = dict(list(self.planner_seen.items())[-20:])

    def on_planner_abort(self):
        if self.escape is not None or self.resend_at is not None:
            return
        cost = self.footprint_cost()
        if cost is None or cost < self.get_parameter("start_blocked_cost").value:
            return  # the goal or the way is the problem, not where the dog stands
        now = time.monotonic()
        window = self.get_parameter("start_escape_window").value
        self.start_escapes = [t for t in self.start_escapes if now - t < window]
        if len(self.start_escapes) >= self.get_parameter("max_start_escapes").value:
            if not getattr(self, "_start_limit_said", False):
                self._start_limit_said = True
                self.get_logger().warn(
                    f"planner refuses goals with the dog in an obstacle (cost {cost}), "
                    f"but {len(self.start_escapes)} escapes in {window:.0f} s already: waiting"
                )
            return
        self._start_limit_said = False
        self.start_escapes.append(now)
        self.on_abort(
            f"planner refuses goals: the dog stands in an obstacle (footprint cost {cost})"
        )

    def on_abort(self, why="leg aborted by the controller"):
        if self.escape is not None:
            return
        self.get_logger().warn(f"{why}: trying to get unstuck")
        direction = self.free_direction()
        if direction is None:
            where = (
                "behind or beside"
                if self.get_parameter("allow_sidestep").value
                else "behind"
            )
            self.get_logger().warn(
                f"no free space {where} the dog: not moving "
                f"({'; '.join(getattr(self, 'blocked', []))})"
            )
            return
        name, vx, vy = direction
        speed = self.get_parameter("escape_speed").value
        duration = self.get_parameter("escape_distance").value / speed
        start = time.monotonic() + self.get_parameter("escape_delay_s").value
        self.escape = (vx * speed, vy * speed, start, start + duration)
        self.get_logger().info(
            f"stepping {name} {self.get_parameter('escape_distance').value:.2f} m"
        )

    def patch_reader(self):
        """Returns cells(x0, x1, y0, y1): local costmap values of a base_link
        rectangle (None where it leaves the costmap), or None without data."""
        m = self.costmap
        if m is None:
            self.get_logger().warn("no local costmap yet")
            return None
        try:
            t = self.tf.lookup_transform(
                m.header.frame_id, self.get_parameter("base_frame").value, Time()
            )
        except Exception as e:
            self.get_logger().warn(f"no TF for the costmap: {e}")
            return None
        q, tr = t.transform.rotation, t.transform.translation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        grid = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        res, ox, oy = (
            m.info.resolution,
            m.info.origin.position.x,
            m.info.origin.position.y,
        )

        def cells(x0, x1, y0, y1):
            xs, ys = np.meshgrid(np.arange(x0, x1, res / 2), np.arange(y0, y1, res / 2))
            wx = tr.x + c * xs - s * ys
            wy = tr.y + s * xs + c * ys
            j = ((wx - ox) / res).astype(int)
            i = ((wy - oy) / res).astype(int)
            if (
                (i < 0).any()
                or (j < 0).any()
                or (i >= m.info.height).any()
                or (j >= m.info.width).any()
            ):
                return None
            return grid[i, j]

        return cells

    def footprint_cost(self):
        cells = self.patch_reader()
        under = cells(*FOOTPRINT) if cells else None
        return None if under is None else int(under.max())

    def free_direction(self):
        cells_of = self.patch_reader()
        if cells_of is None:
            return None
        thr = self.get_parameter("free_threshold").value
        sidestep = self.get_parameter("allow_sidestep").value
        self.blocked = []
        for name, vx, vy, x0, x1, y0, y1 in ESCAPES:
            if vy and not sidestep:
                continue
            cells = cells_of(x0, x1, y0, y1)
            if cells is None:
                continue
            # Unknown (-1) is not free: the dog never steps blind.
            if (cells >= 0).all() and (cells < thr).all():
                return name, vx, vy
            self.blocked.append(
                f"{name}: max cost {cells.max()}, {(cells < 0).sum()} unknown"
            )
        return None

    def tick(self):
        now = time.monotonic()
        if self.escape is not None:
            vx, vy, start, end = self.escape
            if now < start:
                return
            out = Twist()
            if now < end:
                out.linear.x, out.linear.y = vx, vy
                self.cmd.publish(out)
                return
            self.cmd.publish(out)  # stop
            self.escape = None
            if self.get_parameter("retry_goal").value and self.goal is not None:
                if self.retries < self.get_parameter("max_retries").value:
                    self.resend_at = now + 1.0
                else:
                    self.get_logger().warn(
                        "goal still unreachable after retries: giving up"
                    )
        if self.resend_at is not None and now >= self.resend_at:
            self.resend_at = None
            self.retries += 1
            g = self.goal
            g.header.stamp = self.get_clock().now().to_msg()
            self._own_goal = True
            self.get_logger().info(
                f"retrying goal ({self.retries}/{self.get_parameter('max_retries').value})"
            )
            self.goal_pub.publish(g)


def main():
    rclpy.init()
    node = StuckRecovery()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
