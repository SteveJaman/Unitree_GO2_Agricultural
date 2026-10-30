#!/usr/bin/env python3
"""
go2_stuck_recovery.py: get the Go2 unstuck when Nav2 gives up on a leg.

Watches the controller's follow_path action. When a leg is aborted (e.g.
Regulated Pure Pursuit's "detected collision ahead!" with the nose at a wall,
or "Failed to make progress"), it:

  0. if the local costmap is blocked right in front of the dog and the camera
     VLM runs (vlm_reasoning_node.py), asks it first (/vlm_verify_request):
     the LiDAR sometimes reports obstacles that are not there (reflections,
     dust, a smeared scan), and lowering the costmap thresholds would hide
     real ones. The camera's answer decides:
       CLEAR   -> a LiDAR ghost: both costmaps are cleared and the goal is
                  re-sent (at most ghost_retries times per goal, then step 1);
       BLOCKED -> real: sidestep to the side the VLM saw more room on (if that
                  side is free in the costmap, else the other), then step 3;
       no answer within verify_timeout / UNKNOWN -> step 1.
  1. checks the local costmap for a free patch behind the dog, then (with
     allow_sidestep) to its left and right;
  2. steps there: escape_distance metres at escape_speed. With --drive omni
     this goes on /cmd_vel_nav, whose smoother allows vy. With --drive hybrid
     (forward-only Nav2, whose smoother clamps vy to 0) it goes straight to
     /cmd_vel after escape_delay_s, once the smoother has stopped publishing;
  3. re-sends the last /goal_pose (the explorer's current frontier, or a 2D
     Goal Pose) so the planner tries again from the new spot, up to
     max_retries times per goal.

Sidestep on request (/recovery/sidestep {"side": "left"|"right"|"auto"}): the
VLM saw the path blocked although Nav2 is still driving (the LiDAR missed
it). The side (then the other) is checked in the costmap; if free, Nav2 is
paused (explorer: /explore/resume False; else its goals are cancelled), the
dog steps sidestep_distance sideways on sidestep_topic (/cmd_vel, never
clamped), and Nav2 continues from there (explorer resumed, or the goal
re-sent, which does not count as a retry). It is skipped ({"skip": true}: the
VLM then does nothing) unless Nav2 is driving (explorer on, or a follow_path
goal running), and after max_vlm_sidesteps (2) within vlm_sidestep_window
(120 s). The answer goes to /recovery/sidestep_result {"ok", "skip", "side",
"note"}; ok False without skip (no side free) makes the VLM pause and dock.

Nothing is done while the dashboard's Drive pad has manual control
(/teleop/status active). Progress for the dashboard: /recovery/status
(std_msgs/String JSON {"state", "note"}).

If no direction is free, it does not move and logs why.

Run in go2-lidar3d by wifi_nav.sh in every session (--no-recovery turns it off).
"""

import json
import math
import time

import numpy as np

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

try:
    from nav2_msgs.srv import ClearEntireCostmap
except ImportError:  # no Nav2 messages: no ghost clearing, the rest works
    ClearEntireCostmap = None

# Escape patches in base_link (x fwd, y left), metres: (name, vx, vy, x0, x1, y0, y1).
# The footprint is x in [-0.45, 0.36], y in [-0.20, 0.20].
ESCAPES = [
    ("back", -1.0, 0.0, -0.85, -0.45, -0.25, 0.25),
    ("left", 0.0, 1.0, -0.40, 0.30, 0.20, 0.55),
    ("right", 0.0, -1.0, -0.40, 0.30, -0.55, -0.20),
]
# Right in front of the nose: where "collision ahead" comes from.
FRONT = (0.36, 0.80, -0.22, 0.22)
NAV_ACTIONS = ("/follow_path", "/compute_path_to_pose")
CLEAR_SERVICES = (
    "/local_costmap/clear_entirely_local_costmap",
    "/global_costmap/clear_entirely_global_costmap",
)


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
        p("allow_sidestep", False)  # needs vy: --drive omni, or hybrid on /cmd_vel
        # Wait before moving. On /cmd_vel (hybrid) the smoother still sends its
        # stop for velocity_timeout (1 s) after the abort; moving earlier fights it.
        p("escape_delay_s", 0.0)
        p("max_retries", 2)
        p("verify_with_vlm", True)  # ask the camera before trusting a blocked front
        p("verify_timeout", 25.0)  # s (llava:7b: seconds on a GPU, more on CPU)
        p("ghost_retries", 2)  # costmap clears per goal on a CLEAR answer
        p("sidestep_distance", 0.35)  # m, VLM sidesteps
        p("sidestep_topic", "/cmd_vel")  # after the smoother: vy is never clamped
        p("sidestep_delay_s", 1.5)  # s for Nav2 to stop before stepping
        p("max_vlm_sidesteps", 2)  # VLM sidesteps per window, then ignored
        p("vlm_sidestep_window", 120.0)  # s
        gp = lambda k: self.get_parameter(k).value  # noqa: E731

        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.costmap = None
        self.goal = None
        self.retries = 0
        self.ghosts = 0
        self.seen = {}  # goal id -> last status
        # While stepping: {"vx", "vy", "start", "end", "pub", "after"}; after =
        # "retry" (re-send the goal) or "resume" (restart the explorer).
        self.escape = None
        self.resend_at = None
        self.resume_at = None
        self._own_goal = False  # the next /goal_pose is our own retry
        self.verify = (
            None  # (request id, deadline, front-patch note) while asking the VLM
        )
        self.verify_n = 0
        self.manual = False
        self.exploring = False
        self.nav_active = False  # a follow_path goal is accepted / executing
        self.vlm_steps = []  # times of recent VLM sidesteps
        self.resend_free = False  # the pending re-send does not count as a retry

        self.create_subscription(
            GoalStatusArray, gp("status_topic"), self.on_status, 10
        )
        self.create_subscription(
            OccupancyGrid,
            gp("costmap_topic"),
            lambda m: setattr(self, "costmap", m),
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(PoseStamped, gp("goal_topic"), self.on_goal, 10)
        self.create_subscription(
            String, "/vlm_verify_result", self.on_verify_result, 10
        )
        self.create_subscription(String, "/recovery/sidestep", self.on_sidestep, 10)
        self.create_subscription(String, "/teleop/status", self.on_teleop, 10)
        self.create_subscription(String, "/explore/status", self.on_explore, 10)
        self.cmd = self.create_publisher(Twist, gp("cmd_topic"), 10)
        self.side_cmd = (
            self.cmd
            if gp("sidestep_topic") == gp("cmd_topic")
            else self.create_publisher(Twist, gp("sidestep_topic"), 10)
        )
        self.goal_pub = self.create_publisher(PoseStamped, gp("goal_topic"), 10)
        self.verify_pub = self.create_publisher(String, "/vlm_verify_request", 10)
        self.result_pub = self.create_publisher(String, "/recovery/sidestep_result", 10)
        self.status_pub = self.create_publisher(String, "/recovery/status", 10)
        self.resume_pub = self.create_publisher(Bool, "/explore/resume", 10)
        self.cancel = [
            self.create_client(CancelGoal, f"{a}/_action/cancel_goal")
            for a in NAV_ACTIONS
        ]
        self.clear = (
            [self.create_client(ClearEntireCostmap, s) for s in CLEAR_SERVICES]
            if ClearEntireCostmap
            else []
        )
        self.create_timer(0.1, self.tick)
        moves = "back / left / right" if gp("allow_sidestep") else "back"
        check = ", camera check of a blocked front" if gp("verify_with_vlm") else ""
        self.get_logger().info(
            f"stuck recovery ready ({moves}, then retry goal{check})"
        )

    # ---------------------------------------------------------------- inputs
    def on_goal(self, msg):
        if self.resend_at is None and self.escape is None and not self._own_goal:
            self.retries = 0  # a new goal from someone else
            self.ghosts = 0
        self._own_goal = False
        self.goal = msg

    def on_teleop(self, msg):
        try:
            self.manual = bool(json.loads(msg.data).get("active"))
        except ValueError:
            return
        if self.manual and (
            self.escape or self.verify or self.resend_at or self.resume_at
        ):
            # The driver has the dog: drop whatever recovery was doing.
            if self.escape:
                self.escape["pub"].publish(Twist())
            self.escape = self.verify = self.resend_at = self.resume_at = None
            self.report("idle", "manual control: recovery stood down")

    def on_explore(self, msg):
        self.exploring = "enabled=True" in msg.data

    def on_status(self, msg):
        self.nav_active = any(
            s.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
            for s in msg.status_list
        )
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

    def report(self, state, note, warn=False):
        # Two call sites: rclpy refuses one log call with changing severity.
        if warn:
            self.get_logger().warn(note)
        else:
            self.get_logger().info(note)
        self.status_pub.publish(String(data=json.dumps({"state": state, "note": note})))

    # ------------------------------------------------------- aborted legs
    def on_abort(self):
        if self.escape is not None or self.verify is not None or self.manual:
            return
        self.get_logger().warn("leg aborted by the controller: trying to get unstuck")
        front_free, front_note = self.patch_free(*FRONT)
        if (
            not front_free
            and front_note != "no costmap"
            and self.get_parameter("verify_with_vlm").value
            and self.count_subscribers("/vlm_verify_request") > 0
        ):
            self.verify_n += 1
            rid = f"r{self.verify_n}"
            self.verify = (
                rid,
                time.monotonic() + self.get_parameter("verify_timeout").value,
                front_note,
            )
            self.report(
                "verifying",
                f"costmap blocked ahead ({front_note}): asking the camera whether it is real",
            )
            self.verify_pub.publish(String(data=json.dumps({"id": rid})))
            return
        self.escape_from_costmap()

    def on_verify_result(self, msg):
        try:
            r = json.loads(msg.data)
        except ValueError:
            return
        if self.verify is None or r.get("id") != self.verify[0]:
            return
        self.verify = None
        if self.manual:
            return
        decision, side = r.get("decision"), r.get("side")
        if decision == "CLEAR":
            if self.ghosts < self.get_parameter("ghost_retries").value and self.goal:
                self.ghosts += 1
                self.report(
                    "ghost",
                    "camera sees the way ahead clear: treating it as a LiDAR ghost, "
                    f"clearing the costmaps and retrying ({self.ghosts}/"
                    f"{self.get_parameter('ghost_retries').value})",
                    warn=True,
                )
                self.clear_costmaps()
                self.resend_at = time.monotonic() + 1.5  # let the costmaps refill first
                return
            self.report(
                "blocked",
                "camera says clear, but the retry failed again: stepping away",
            )
        elif decision == "BLOCKED":
            order = (
                [side, "right" if side == "left" else "left"]
                if side
                else ["left", "right"]
            )
            got = self.free_side(order, self.get_parameter("escape_distance").value)
            if got:
                self.report(
                    "sidestep", f"camera confirms the obstacle: stepping {got} round it"
                )
                self.start_step(
                    got,
                    self.get_parameter("escape_distance").value,
                    self.side_cmd,
                    "retry",
                    self.get_parameter("escape_delay_s").value,
                )
                return
            self.report(
                "blocked", "camera confirms the obstacle; no free side: trying back"
            )
        else:
            self.report("blocked", f"camera unsure ({decision}): usual recovery")
        self.escape_from_costmap()

    def escape_from_costmap(self):
        direction = self.free_direction()
        if direction is None:
            where = (
                "behind or beside"
                if self.get_parameter("allow_sidestep").value
                else "behind"
            )
            self.report(
                "stuck",
                f"no free space {where} the dog: not moving "
                f"({'; '.join(getattr(self, 'blocked', []))})",
                warn=True,
            )
            return
        name, vx, vy = direction
        speed = self.get_parameter("escape_speed").value
        duration = self.get_parameter("escape_distance").value / speed
        start = time.monotonic() + self.get_parameter("escape_delay_s").value
        self.escape = {
            "vx": vx * speed,
            "vy": vy * speed,
            "start": start,
            "end": start + duration,
            "pub": self.cmd,
            "after": "retry",
        }
        self.report(
            "escape",
            f"stepping {name} {self.get_parameter('escape_distance').value:.2f} m",
        )

    # -------------------------------------------- sidestep on VLM request
    def on_sidestep(self, msg):
        try:
            req = json.loads(msg.data)
        except ValueError:
            req = {"side": msg.data.strip()}
        side = req.get("side") or "auto"

        def answer(ok, note, got=None, skip=False):
            self.result_pub.publish(
                String(
                    data=json.dumps({"ok": ok, "skip": skip, "side": got, "note": note})
                )
            )
            if skip:
                self.get_logger().info(f"VLM sidestep skipped: {note}")
            else:
                self.report("sidestep" if ok else "blocked", note, warn=not ok)

        # "skip" answers make the VLM do nothing at all (no docking either).
        if self.manual:
            return answer(False, "manual control", skip=True)
        if self.escape is not None or self.verify is not None:
            return answer(False, "busy with another recovery", skip=True)
        if not (self.exploring or self.nav_active):
            return answer(False, "the dog is not navigating", skip=True)
        now = time.monotonic()
        window = self.get_parameter("vlm_sidestep_window").value
        self.vlm_steps = [t for t in self.vlm_steps if now - t < window]
        if len(self.vlm_steps) >= self.get_parameter("max_vlm_sidesteps").value:
            return answer(
                False,
                f"already stepped round {len(self.vlm_steps)}x in {window:.0f} s: "
                "ignoring the camera, the LiDAR costmap decides",
                skip=True,
            )
        order = {"left": ["left", "right"], "right": ["right", "left"]}.get(
            side, ["left", "right"]
        )
        dist = self.get_parameter("sidestep_distance").value
        got = self.free_side(order, dist)
        if got is None:
            return answer(
                False,
                f"path blocked, no free side in the costmap "
                f"({'; '.join(getattr(self, 'blocked', []))})",
            )
        # Take the dog from Nav2 first, so the step does not fight it.
        if self.exploring:
            self.resume_pub.publish(Bool(data=False))
            after = "resume"
        else:
            self.cancel_nav()
            after = "resend"  # always re-send the goal we cancelled
        self.vlm_steps.append(now)
        self.start_step(
            got,
            dist,
            self.side_cmd,
            after,
            self.get_parameter("sidestep_delay_s").value,
        )
        answer(
            True,
            f"camera sees the path blocked: stepping {got} {dist:.2f} m round it",
            got,
        )

    def start_step(self, side, dist, pub, after, delay):
        speed = self.get_parameter("escape_speed").value
        start = time.monotonic() + delay
        self.escape = {
            "vx": 0.0,
            "vy": (speed if side == "left" else -speed),
            "start": start,
            "end": start + dist / speed,
            "pub": pub,
            "after": after,
        }

    def cancel_nav(self):
        # A zero goal id and stamp cancels every goal of the action server.
        for cli in self.cancel:
            if cli.service_is_ready():
                cli.call_async(CancelGoal.Request())

    def clear_costmaps(self):
        if not self.clear:
            self.get_logger().warn("nav2_msgs missing: cannot clear the costmaps")
        for cli in self.clear:
            if cli.service_is_ready():
                cli.call_async(ClearEntireCostmap.Request())
            else:
                self.get_logger().warn(f"{cli.srv_name} not available")

    # -------------------------------------------------------- costmap checks
    def patch_free(self, x0, x1, y0, y1):
        """(free?, note) for a base_link rectangle in the local costmap.

        Unknown (-1) is not free: the dog never steps blind.
        """
        m = self.costmap
        if m is None:
            return False, "no costmap"
        try:
            t = self.tf.lookup_transform(
                m.header.frame_id, self.get_parameter("base_frame").value, Time()
            )
        except Exception as e:
            return False, f"no TF for the costmap: {e}"
        q, tr = t.transform.rotation, t.transform.translation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        grid = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        res, ox, oy = (
            m.info.resolution,
            m.info.origin.position.x,
            m.info.origin.position.y,
        )
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
            return False, "outside the costmap"
        cells = grid[i, j]
        if (cells >= 0).all() and (
            cells < self.get_parameter("free_threshold").value
        ).all():
            return True, "free"
        return False, f"max cost {cells.max()}, {(cells < 0).sum()} unknown"

    def free_side(self, order, dist):
        """First of `order` ("left"/"right") with room for a `dist` m sidestep."""
        self.blocked = []
        for side in order:
            y0, y1 = (0.20, 0.20 + dist) if side == "left" else (-0.20 - dist, -0.20)
            ok, note = self.patch_free(-0.40, 0.30, y0, y1)
            if ok:
                return side
            self.blocked.append(f"{side}: {note}")
        return None

    def free_direction(self):
        if self.costmap is None:
            self.get_logger().warn("no local costmap yet")
            return None
        sidestep = self.get_parameter("allow_sidestep").value
        self.blocked = []
        for name, vx, vy, x0, x1, y0, y1 in ESCAPES:
            if vy and not sidestep:
                continue
            ok, note = self.patch_free(x0, x1, y0, y1)
            if ok:
                return name, vx, vy
            self.blocked.append(f"{name}: {note}")
        return None

    # ------------------------------------------------------------------ loop
    def tick(self):
        now = time.monotonic()
        if self.verify is not None and now > self.verify[1]:
            self.verify = None
            self.report("blocked", "no answer from the camera VLM: usual recovery")
            self.escape_from_costmap()
        if self.escape is not None:
            e = self.escape
            if now < e["start"]:
                return
            out = Twist()
            if now < e["end"]:
                out.linear.x, out.linear.y = e["vx"], e["vy"]
                e["pub"].publish(out)
                return
            e["pub"].publish(out)  # stop
            self.escape = None
            if e["after"] == "resume":
                self.resume_at = now + 1.0
            elif e["after"] == "resend" and self.goal is not None:
                self.resend_at, self.resend_free = now + 1.0, True
            elif self.get_parameter("retry_goal").value and self.goal is not None:
                if self.retries < self.get_parameter("max_retries").value:
                    self.resend_at = now + 1.0
                else:
                    self.report(
                        "stuck",
                        "goal still unreachable after retries: giving up",
                        warn=True,
                    )
        if self.resume_at is not None and now >= self.resume_at:
            self.resume_at = None
            self.report("idle", "sidestep done: exploring again")
            self.resume_pub.publish(Bool(data=True))
        if self.resend_at is not None and now >= self.resend_at:
            self.resend_at = None
            if self.goal is None:
                return
            g = self.goal
            g.header.stamp = self.get_clock().now().to_msg()
            self._own_goal = True
            if self.resend_free:
                # After a VLM sidestep: the goal was ours to cancel, not a failure.
                self.resend_free = False
                self.report("retry", "continuing to the goal from the new spot")
            else:
                self.retries += 1
                self.report(
                    "retry",
                    f"retrying goal ({self.retries}/{self.get_parameter('max_retries').value})",
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
