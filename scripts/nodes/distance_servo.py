#!/usr/bin/env python3
"""Standalone test: hold the D1 arm's tool at a set distance from a tracked object.

Subscribes to /object_distance (std_msgs/Float32, metres, from yolo_depth_tracker.py) and
moves the tool straight along the arm base's X axis (base_link: +x ahead of the arm,
d1_go2.urdf.xacro) in small Cartesian steps:

    error = distance - target_distance          (target 0.50 m)
    |error| <= deadband (0.02 m)  -> no move
    error > 0 (object too far)    -> +X, toward it
    error < 0 (object too close)  -> -X, away from it
    step = kp * error, its size clamped to [min_step, max_step] (1-2 cm)

Each step: the tool pose now (TF base_link -> tool_link), the same pose shifted in x,
/compute_cartesian_path (MoveIt, collision-checked), then /execute_trajectory. One
step at a time; the next starts once the last has finished.

Safety:
  - No command unless the latest distance is fresh (< stale_s old) and within
    [min_distance, max_distance] (0.20-0.80 m). This is checked again just before a
    planned step is executed, and a step still moving is cancelled when the distance
    goes stale or out of range.
  - The tool never goes more than max_back / max_forward (0.10 / 0.15 m) along X from
    where it was when the node started: the stop if the arm's motion does not change the
    measured distance (camera fixed on the dog, not on the arm).
  - The path must be planned in full (fraction >= 0.99), or the step is dropped.
  - dry_run:=true plans the steps but executes nothing.

Data: with csv_path set, every control tick (control_hz, 5 Hz) appends a row to that CSV:
    time_s, wall_s, current_distance_m, tool_x_m, error_m, step_m, state
time_s from the node's start (wall_s: Unix time, to line up with other logs); current_distance_m the latest measured distance (empty when
stale: nothing seen for stale_s); tool_x_m the tool's X in frame (MoveIt's view of the
arm); step_m the commanded step (0 when holding); state one of moving, step, hold
(inside the deadband), wait (not yet median_of distances measured since the last
move ended + measure_after_move_s), stale, out_of_range, limit, not_ready, dry_run.
scripts/tools/plot_distance.py turns it into distance_over_time.png.

With steps_csv set, every step is also one "interaction" row (as in the per-action
records of Zhang & Gupta, "Push Past Green", CoRL 2023): step, policy, t_start_s,
t_plan_s, t_end_s, distance_before_m, distance_after_m (first fresh distance after the
move), tool_x_before_m, tool_x_after_m, commanded_m, achieved_m, planned_points,
fraction, outcome (done, plan_failed, rejected, cancelled, error_<code>, dry_run).

Policies (policy parameter), for comparing the servo against simple baselines:
    proportional  step = kp * error clamped to [min_step, max_step]  (the servo)
    fixed_step    always max_step towards the target (bang-bang, no gain)
    open_loop     one move of the whole first error (clamped to the travel limits),
                  then hold: no feedback
stop_when_settled:=true makes the node exit (code 0) once the distance stayed inside
the deadband for settle_s, and run_timeout_s (> 0) ends it in any case (code 3); stopped
from outside (SIGINT/SIGTERM) it exits 130.

Start MoveIt first (arm_moveit.sh, domain 78) and put the arm in the start pose
yourself; this node does not move it there.

Usage (in MoveIt's environment):
    ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1 python3 scripts/nodes/distance_servo.py \
        --ros-args -p dry_run:=true
"""

import copy
import csv
import math
import os
import time

import rclpy
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformException, TransformListener


STEP_FIELDS = [
    "step",
    "policy",
    "t_start_s",
    "t_plan_s",
    "t_end_s",
    "distance_before_m",
    "distance_after_m",
    "tool_x_before_m",
    "tool_x_after_m",
    "commanded_m",
    "achieved_m",
    "planned_points",
    "fraction",
    "outcome",
]


def step_for(distance, target, deadband, kp, min_step, max_step):
    """Signed X step (m) for a measured distance: 0 inside the deadband."""
    error = distance - target
    if abs(error) <= deadband:
        return 0.0
    size = min(max(abs(kp * error), min_step), max_step)
    return math.copysign(size, error)


class DistanceServo(Node):
    def __init__(self):
        super().__init__("distance_servo")
        d = self.declare_parameter
        d("target_distance", 0.50)
        d("deadband", 0.02)
        d("kp", 0.5)  # m of step per m of error, before clamping
        d("min_step", 0.01)
        d("max_step", 0.02)
        d("min_distance", 0.20)  # hard limits on the measured distance
        d("max_distance", 0.80)
        d("stale_s", 0.5)  # distance older than this = no object
        d("max_forward", 0.15)  # tool travel limits along X from the start pose
        d("max_back", 0.10)
        d("control_hz", 5.0)
        d("velocity_scaling", 0.2)
        d("group", "arm")
        d("frame", "base_link")
        d("tool_link", "tool0")
        d("dry_run", False)
        d("csv_path", "")  # '' = no CSV log
        d("steps_csv", "")  # '' = no per-step interaction log
        d("policy", "proportional")  # proportional | fixed_step | open_loop
        d("stop_when_settled", False)
        d("settle_s", 3.0)
        d("run_timeout_s", 0.0)  # 0 = run until stopped
        d(
            "min_run_s", 0.0
        )  # stop_when_settled only after this long (a disturbance is due)
        d(
            "measure_after_move_s", 0.3
        )  # next step only on distances measured this long after a move
        d(
            "median_of", 1
        )  # step on the median of this many fresh distances since then (1 = the latest)
        self.p = lambda n: self.get_parameter(n).value
        self.t0 = time.monotonic()
        self.csv = self.csv_writer = None
        if self.p("csv_path"):
            path = os.path.expanduser(self.p("csv_path"))
            os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
            self.csv = open(path, "w", newline="")
            self.csv_writer = csv.writer(self.csv)
            self.csv_writer.writerow(
                [
                    "time_s",
                    "wall_s",
                    "current_distance_m",
                    "tool_x_m",
                    "error_m",
                    "step_m",
                    "state",
                ]
            )
            self.get_logger().info(f"logging every tick to {path}")
        if self.p("policy") not in ("proportional", "fixed_step", "open_loop"):
            raise ValueError("policy must be proportional, fixed_step or open_loop")
        self.steps_csv = self.steps_writer = None
        self.step_n = 0
        self.step_rec = None  # the interaction being recorded
        self.open_loop_done = False
        self.in_band_since = None
        self.exit_code = None
        self.last_move_end = 0.0
        self.recent = []  # (time, distance)
        if self.p("steps_csv"):
            path = os.path.expanduser(self.p("steps_csv"))
            os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
            self.steps_csv = open(path, "w", newline="")
            self.steps_writer = csv.DictWriter(self.steps_csv, fieldnames=STEP_FIELDS)
            self.steps_writer.writeheader()

        self.distance = None
        self.distance_time = 0.0
        self.home_x = None
        self.busy = False
        self.goal_handle = None

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.execute = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.create_subscription(Float32, "/object_distance", self.on_distance, 10)
        self.create_timer(1.0 / float(self.p("control_hz")), self.tick)
        self.get_logger().info(
            f'target {self.p("target_distance"):.2f} m +/- {self.p("deadband"):.2f}, '
            f'limits {self.p("min_distance"):.2f}-{self.p("max_distance"):.2f} m, '
            f'{"DRY RUN" if self.p("dry_run") else "EXECUTING"}'
        )

    # ------------------------------------------------------------------ inputs
    def on_distance(self, msg):
        self.distance = float(msg.data)
        self.distance_time = time.monotonic()
        self.recent.append((self.distance_time, self.distance))
        del self.recent[:-50]

    def settled_distance(self):
        """Median of the distances measured since the last move ended (+ measure_after_move_s),
        once there are median_of of them; None while still collecting."""
        since = self.last_move_end + float(self.p("measure_after_move_s"))
        n = max(1, int(self.p("median_of")))
        latest = [d for t, d in self.recent if t >= since][
            -n:
        ]  # the n latest, in time order
        if len(latest) < n:
            return None
        return sorted(latest)[len(latest) // 2]

    def safe_distance(self):
        """The latest distance if fresh, finite and inside the hard limits, else None."""
        if self.distance is None or time.monotonic() - self.distance_time > float(
            self.p("stale_s")
        ):
            return None
        dist = self.distance
        if not math.isfinite(dist) or not float(
            self.p("min_distance")
        ) <= dist <= float(self.p("max_distance")):
            self.get_logger().warn(
                f"distance {dist:.3f} m outside the limits: no motion",
                throttle_duration_sec=2.0,
            )
            return None
        return dist

    def tool_pose(self, quiet=False):
        try:
            t = self.tf.lookup_transform(self.p("frame"), self.p("tool_link"), Time())
        except TransformException as e:
            if not quiet:
                self.get_logger().warn(
                    f"no tool pose yet: {e}", throttle_duration_sec=5.0
                )
            return None
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        )
        pose.orientation = t.transform.rotation
        return pose

    # ------------------------------------------------------------------ control
    def log_row(self, state, step=0.0):
        if self.csv_writer is None:
            return
        fresh = (
            self.distance is not None
            and time.monotonic() - self.distance_time <= float(self.p("stale_s"))
        )
        dist = self.distance if fresh else None
        pose = self.tool_pose(quiet=True)
        target = float(self.p("target_distance"))
        self.csv_writer.writerow(
            [
                f"{time.monotonic() - self.t0:.3f}",
                f"{time.time():.3f}",
                f"{dist:.4f}" if dist is not None else "",
                f"{pose.position.x:.4f}" if pose is not None else "",
                f"{dist - target:.4f}" if dist is not None else "",
                f"{step:.4f}",
                state,
            ]
        )
        self.csv.flush()

    def tick(self):
        self.finish_pending_step()
        state, step = self.control()
        self.log_row(state, step)
        self.check_end(state)

    def fresh_distance(self):
        if self.distance is None or time.monotonic() - self.distance_time > float(
            self.p("stale_s")
        ):
            return None
        return self.distance

    def check_end(self, state):
        now = time.monotonic()
        dist = self.fresh_distance()
        target, band = float(self.p("target_distance")), float(self.p("deadband"))
        if dist is not None and abs(dist - target) <= band and not self.busy:
            self.in_band_since = self.in_band_since or now
        else:
            self.in_band_since = None
        if (
            self.p("stop_when_settled")
            and self.in_band_since is not None
            and now - self.in_band_since >= float(self.p("settle_s"))
            and now - self.t0 >= float(self.p("min_run_s"))
        ):
            self.get_logger().info(
                f"settled: {dist:.3f} m, inside {target:.2f} +/- {band:.2f} for {self.p('settle_s')} s"
            )
            self.exit_code = 0
        elif float(self.p("run_timeout_s")) > 0 and now - self.t0 > float(
            self.p("run_timeout_s")
        ):
            self.get_logger().warn(f"run timeout ({self.p('run_timeout_s')} s)")
            self.exit_code = 3

    def finish_pending_step(self, force=False):
        """Close an interaction record once the move ended and a distance newer than its end
        arrived (force: now, with the latest distance, as a new step is about to start)."""
        rec = self.step_rec
        if rec is None or rec.get("t_end_abs") is None:
            return
        # the distance once the arm stopped ringing: the median the next step will use
        after = (
            self.settled_distance() if self.last_move_end >= rec["t_end_abs"] else None
        )
        if after is None and force and self.distance_time > rec["t_end_abs"]:
            after = self.distance
        waited = time.monotonic() - rec["t_end_abs"]
        if after is None and waited < 3.0 and not force:
            return
        pose = self.tool_pose(quiet=True)
        rec["distance_after_m"] = f"{after:.4f}" if after is not None else ""
        if pose is not None:
            rec["tool_x_after_m"] = f"{pose.position.x:.4f}"
            rec["achieved_m"] = f"{pose.position.x - rec['_x0']:.4f}"
        self.write_step()

    def write_step(self):
        rec = self.step_rec
        self.step_rec = None
        if self.steps_writer is None or rec is None:
            return
        self.steps_writer.writerow({k: rec.get(k, "") for k in STEP_FIELDS})
        self.steps_csv.flush()

    def control(self):
        """One control tick; returns (state, commanded step) for the CSV log."""
        dist = self.safe_distance()
        if self.busy:
            if dist is None and self.goal_handle is not None:
                self.get_logger().warn(
                    "distance lost or out of range: cancelling the step"
                )
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
            return "moving", 0.0
        if dist is None:
            fresh = (
                self.distance is not None
                and time.monotonic() - self.distance_time <= float(self.p("stale_s"))
            )
            return ("out_of_range" if fresh else "stale"), 0.0
        pose = self.tool_pose()
        if pose is None:
            return "not_ready", 0.0
        if self.distance_time < self.last_move_end + float(
            self.p("measure_after_move_s")
        ):
            return (
                "wait",
                0.0,
            )  # the distance was measured while (or just after) the arm moved
        settled = self.settled_distance()
        if settled is None:
            return "wait", 0.0  # still collecting median_of distances after the move
        if int(self.p("median_of")) > 1:
            dist = settled
        if self.home_x is None:
            self.home_x = pose.position.x
            self.get_logger().info(
                f'start pose: tool x = {self.home_x:.3f} m in {self.p("frame")}'
            )

        policy = self.p("policy")
        target, band = float(self.p("target_distance")), float(self.p("deadband"))
        if policy == "open_loop":
            if self.open_loop_done:
                return "hold", 0.0
            step = 0.0 if abs(dist - target) <= band else dist - target
        elif policy == "fixed_step":
            step = (
                0.0
                if abs(dist - target) <= band
                else math.copysign(float(self.p("max_step")), dist - target)
            )
        else:
            step = step_for(
                dist,
                target,
                band,
                float(self.p("kp")),
                float(self.p("min_step")),
                float(self.p("max_step")),
            )
        if step == 0.0:
            return "hold", 0.0
        lo = self.home_x - float(self.p("max_back"))
        hi = self.home_x + float(self.p("max_forward"))
        new_x = min(max(pose.position.x + step, lo), hi)
        if abs(new_x - pose.position.x) < 0.002:
            self.get_logger().warn(
                f"tool at its X travel limit ({lo:.3f}-{hi:.3f} m): no motion",
                throttle_duration_sec=2.0,
            )
            return "limit", 0.0
        if not self.cartesian.service_is_ready() or not self.execute.server_is_ready():
            self.get_logger().warn(
                "MoveIt not ready (/compute_cartesian_path, /execute_trajectory)",
                throttle_duration_sec=5.0,
            )
            return "not_ready", 0.0

        target = copy.deepcopy(pose)
        target.position.x = new_x
        req = GetCartesianPath.Request()
        req.header.frame_id = self.p("frame")
        req.header.stamp = self.get_clock().now().to_msg()
        req.start_state = RobotState(is_diff=True)
        req.group_name = self.p("group")
        req.link_name = self.p("tool_link")
        req.waypoints = [target]
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = float(self.p("velocity_scaling"))
        req.max_acceleration_scaling_factor = float(self.p("velocity_scaling"))
        self.finish_pending_step(force=True)  # never overwrite the last step's record
        self.busy = True
        if policy == "open_loop":
            self.open_loop_done = True
        self.step_n += 1
        self.step_rec = {
            "step": self.step_n,
            "policy": policy,
            "t_start_s": f"{time.monotonic() - self.t0:.3f}",
            "distance_before_m": f"{dist:.4f}",
            "tool_x_before_m": f"{pose.position.x:.4f}",
            "commanded_m": f"{new_x - pose.position.x:.4f}",
            "_x0": pose.position.x,
            "t_end_abs": None,
        }
        self.get_logger().info(
            f"distance {dist:.3f} m: tool x {pose.position.x:.3f} -> {new_x:.3f}"
        )
        self.cartesian.call_async(req).add_done_callback(self.on_path)
        return ("dry_run" if self.p("dry_run") else "step"), new_x - pose.position.x

    def on_path(self, future):
        res = future.result()
        if (
            res is None
            or res.error_code.val != MoveItErrorCodes.SUCCESS
            or res.fraction < 0.99
        ):
            frac = res.fraction if res is not None else 0.0
            self.get_logger().warn(
                f"Cartesian step not planned in full ({frac:.0%}): dropped"
            )
            self.end_step("plan_failed", fraction=frac)
            self.busy = False
            return
        n = len(res.solution.joint_trajectory.points)
        if self.step_rec is not None:
            self.step_rec.update(
                t_plan_s=f"{time.monotonic() - self.t0:.3f}",
                planned_points=n,
                fraction=f"{res.fraction:.3f}",
            )
        if self.p("dry_run"):
            self.get_logger().info(f"dry run: step planned ({n} points), not executed")
            self.end_step("dry_run", fraction=res.fraction)
            self.busy = False
            return
        if self.safe_distance() is None:  # changed while planning
            self.end_step("cancelled", fraction=res.fraction)
            self.busy = False
            return
        goal = ExecuteTrajectory.Goal(trajectory=res.solution)
        self.execute.send_goal_async(goal).add_done_callback(self.on_goal)

    def on_goal(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn("trajectory rejected")
            self.end_step("rejected")
            self.busy = False
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.on_done)

    def on_done(self, future):
        result = future.result()
        code = result.result.error_code.val if result is not None else None
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(f"step ended with MoveIt error code {code}")
        if self.step_rec is not None:
            self.step_rec["outcome"] = (
                "done"
                if code == MoveItErrorCodes.SUCCESS
                else (
                    "cancelled"
                    if code == MoveItErrorCodes.PREEMPTED
                    else f"error_{code}"
                )
            )
            self.step_rec["t_end_s"] = f"{time.monotonic() - self.t0:.3f}"
            self.step_rec["t_end_abs"] = time.monotonic()
        self.goal_handle = None
        self.last_move_end = time.monotonic()
        self.busy = False

    def end_step(self, outcome, fraction=None):
        """Close the interaction record of a step that never moved the arm."""
        if self.step_rec is None:
            return
        self.step_rec.update(
            outcome=outcome, t_end_s=f"{time.monotonic() - self.t0:.3f}"
        )
        if fraction is not None:
            self.step_rec["fraction"] = f"{fraction:.3f}"
        self.write_step()


def main():
    rclpy.init()
    node = DistanceServo()
    try:
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node.goal_handle is not None:
            node.goal_handle.cancel_goal_async()
        if node.step_rec is not None:
            node.step_rec.setdefault("outcome", "interrupted")
            node.write_step()
        for f in (node.csv, node.steps_csv):
            if f is not None:
                f.close()
        # interrupted (Ctrl-C, SIGTERM: rclpy shuts the context down) is not "settled"
        code = node.exit_code if node.exit_code is not None else 130
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    import sys

    sys.exit(main())
