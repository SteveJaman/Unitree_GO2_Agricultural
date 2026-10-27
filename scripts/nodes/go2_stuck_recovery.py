#!/usr/bin/env python3
"""
go2_stuck_recovery.py: get the Go2 unstuck when Nav2 gives up on a leg.

Watches the controller's follow_path action. When a leg is aborted (e.g.
Regulated Pure Pursuit's "detected collision ahead!" with the nose at a wall,
or "Failed to make progress"), it:

  1. checks the local costmap for a free patch behind the dog, then (with
     allow_sidestep, i.e. --omni, whose smoother allows vy) to its left and
     right;
  2. steps there: escape_distance metres at escape_speed, sent on
     /cmd_vel_nav so the velocity smoother still limits acceleration;
  3. re-sends the last /goal_pose (the explorer's current frontier, or a 2D
     Goal Pose) so the planner tries again from the new spot, up to
     max_retries times per goal.

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
        p("allow_sidestep", False)  # only when the smoother allows vy (--omni)
        p("max_retries", 2)

        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.costmap = None
        self.goal = None
        self.retries = 0
        self.seen = {}  # goal id -> last status
        self.escape = None  # (vx, vy, end time) while stepping
        self.resend_at = None
        self._own_goal = False  # the next /goal_pose is our own retry

        self.create_subscription(
            GoalStatusArray,
            self.get_parameter("status_topic").value,
            self.on_status,
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
        self.get_logger().info(f"stuck recovery ready ({moves}, then retry goal)")

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

    def on_abort(self):
        if self.escape is not None:
            return
        self.get_logger().warn("leg aborted by the controller: trying to get unstuck")
        direction = self.free_direction()
        if direction is None:
            where = (
                "behind or beside"
                if self.get_parameter("allow_sidestep").value
                else "behind"
            )
            self.get_logger().warn(
                f"no free space {where} the dog: not moving "
                f"({'; '.join(getattr(self, 'blocked', []))})")
            return
        name, vx, vy = direction
        speed = self.get_parameter("escape_speed").value
        duration = self.get_parameter("escape_distance").value / speed
        self.escape = (vx * speed, vy * speed, time.monotonic() + duration)
        self.get_logger().info(
            f"stepping {name} {self.get_parameter('escape_distance').value:.2f} m"
        )

    def free_direction(self):
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
        thr = self.get_parameter("free_threshold").value
        sidestep = self.get_parameter("allow_sidestep").value
        self.blocked = []
        for name, vx, vy, x0, x1, y0, y1 in ESCAPES:
            if vy and not sidestep:
                continue
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
                continue
            cells = grid[i, j]
            # Unknown (-1) is not free: the dog never steps blind.
            if (cells >= 0).all() and (cells < thr).all():
                return name, vx, vy
            self.blocked.append(
                f"{name}: max cost {cells.max()}, {(cells < 0).sum()} unknown")
        return None

    def tick(self):
        now = time.monotonic()
        if self.escape is not None:
            vx, vy, end = self.escape
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
