#!/usr/bin/env python3
"""
moveit_d1_bridge.py

The real D1 arm as MoveIt's arm_controller. MoveIt speaks ROS 2 trajectories
(radians); the D1 speaks JSON (degrees). Started by arm_moveit.sh --real, with
demo.launch.py hardware:=real, which runs no ros2_control: this node is the
only server of /arm_controller/follow_joint_trajectory and the only publisher
of /joint_states.

Each FollowJointTrajectory goal:
  1. is checked before anything moves: all six joints, within the arm's limits
     and max_vel, and starting where the arm actually is;
  2. is resampled at BRIDGE_RATE (50 Hz) with MoveIt's own timing, so Cartesian
     paths stay straight and the arm never stops between waypoints;
  3. plays in mode 0 (direct). With the arm on the Go2 the whole trajectory goes
     to the robot in one traj_points message, because streamed over Wi-Fi the
     setpoints bunch up and the arm shakes (docs/hardware/d1arm.md). Wired, it
     is streamed from here;
  4. ends with smooth_move() on the last point, whose outer PID removes gravity
     sag, and succeeds only if the arm is within GOAL_TOLERANCE_DEG.

It stops at once on a cancel, when the arm falls PATH_TOLERANCE_DEG behind, or
when telemetry stops, and reports that to MoveIt. The arm then holds where it
is. Shutting down leaves the arm enabled and holding: it never releases by
itself, the arm would fall. Fold it with start_arm.sh fold.

Start-up powers and enables the arm only if it is not already: enable-all sent
to an enabled arm drives it back to its last target (d1arm.md). The gripper is
held at the angle it has on start-up.

With ARM_START_STOW=1 (arm_moveit.sh --real), an arm resting on its fold is then
moved to stow: MoveIt's model counts the real fold as a self-collision and plans
nothing from it. Only from the fold, whatever the roll joints: from any other
pose this straight joint-space move is unchecked, so the arm stays where it is.

Usage (arm_moveit.sh --real does this):
    ARM_JOINT_SIGNS="1 1 1 1 1 1" python3 scripts/nodes/moveit_d1_bridge.py

Environment:
    ARM_JOINT_SIGNS Required: "+1/-1" for each of Joint1..Joint6, firmware angle
                    = sign x MoveIt angle. The D1-550 URDF and Unitree's own
                    d1_description disagree on Joint1 and Joint6: test on the
                    arm (d1arm.md, "Joint signs") before setting this.
    ARM_IFACE       Arm network interface (default: d1_arm_control's auto-detect)
    ARM_ROBOT       Go2 Wi-Fi IP, for an arm plugged into the Go2
    BRIDGE_RATE     Trajectory rate in Hz (default: 50)
    ARM_START_STOW  1: on start-up move from the fold to stow. MOVES THE ARM

Joint mapping: MoveIt Joint1..Joint6 (radians) <-> D1 joint 0..5 (degrees).
"""

import json
import math
import os
import signal
import sys
import threading
import time
from dataclasses import replace

# Add the nodes directory so d1_arm_control and d1_motion can be imported
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

from d1_arm_calibration import FOLDED_ANGLES
from d1_arm_control import (
    ADDR_CMD,
    ADDR_TELEM,
    D1Arm,
    ENABLE_FULL,
    FUNC_JOINTS,
    JOINT_LIMITS,
)
from d1_motion import (
    ARM_JOINTS,
    MotionConfig,
    read_angles,
    robot_has,
    smooth_move,
    steady_gripper,
    stop_stream,
)

MOVEIT_JOINTS = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
FINGER_JOINTS = ["Joint_L", "Joint_R"]  # in the gripper model; not driven from here
RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0

STREAM_RATE = float(os.environ.get("BRIDGE_RATE", "50"))
REAL_MAX_VEL_DEG_S = 30.0  # refuse faster trajectories (d1_motion's limit)
REAL_MAX_ACC_DEG_S2 = 60.0
LIMIT_MARGIN_DEG = 3.0  # the real fold reads joint 2 at +91.8
START_TOLERANCE_DEG = (
    5.0  # mode 0 would jump: refuse a trajectory that starts elsewhere
)
PATH_TOLERANCE_DEG = 20.0  # behind the setpoint: something is blocking the arm
PATH_LAG_S = 1.0  # measured against the setpoint this long ago: Wi-Fi and the
# firmware delay telemetry, at wrist speed 1 s reads as 20 deg (real arm, 2026-11-03)
GOAL_TOLERANCE_DEG = 2.0
TELEMETRY_TIMEOUT_S = 1.0
T_POWER, T_ENABLE = 1.5, 1.0  # s, as d1_arm_calibration.py
STOW_RAD = [0.0, -1.396, 0.785, 0.0, 1.396, 0.0]  # "stow" in d1_go2.srdf, MoveIt angles
# Counts as resting on the fold: shoulder, elbow and wrist pitch decide that; the
# roll joints (base, 3, 5) only turn the wrist in place, e.g. after a sign test.
FOLD_TOLERANCE_DEG = np.array([35.0, 8.0, 8.0, 35.0, 8.0, 35.0])
STOW_TOLERANCE_DEG = 5.0  # already there, nothing to do
STOW_MAX_MOVE_DEG = 65.0  # fold to stow is 57 at most: more means a wrong sign
STOW_VEL_DEG_S, STOW_ACC_DEG_S2 = 15.0, 30.0


def parse_signs(text):
    """ARM_JOINT_SIGNS: six values, each +1 or -1."""
    try:
        signs = [int(v) for v in text.split()]
    except (AttributeError, ValueError):
        signs = []
    if len(signs) != ARM_JOINTS or any(s not in (1, -1) for s in signs):
        raise ValueError(f'ARM_JOINT_SIGNS needs six values of 1 or -1, got "{text}"')
    return signs


def seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


def resample(trajectory, rate):
    """MoveIt's trajectory at a fixed rate, positions in radians, Joint1..Joint6
    in columns, (cubic Hermite through the points where MoveIt gives velocities).
    Raises ValueError if a joint is missing or there are no points."""
    missing = [j for j in MOVEIT_JOINTS if j not in trajectory.joint_names]
    if missing:
        raise ValueError(f"no {', '.join(missing)} in the trajectory")
    if not trajectory.points:
        raise ValueError("the trajectory has no points")
    order = [trajectory.joint_names.index(j) for j in MOVEIT_JOINTS]
    times, pos, vel = [], [], []
    for pt in trajectory.points:
        t = seconds(pt.time_from_start)
        if times and t <= times[-1]:
            continue
        times.append(t)
        pos.append([pt.positions[i] for i in order])
        vel.append([pt.velocities[i] for i in order] if len(pt.velocities) else None)
    times, pos = np.array(times), np.array(pos)
    if len(times) == 1:
        return pos.copy()
    if any(v is None for v in vel):
        vel = np.gradient(pos, times, axis=0)  # no velocities: secant slopes
    vel = np.array(vel, dtype=float)
    samples = np.append(np.arange(times[0], times[-1], 1.0 / rate), times[-1])
    i = np.clip(np.searchsorted(times, samples, side="right") - 1, 0, len(times) - 2)
    h = (times[i + 1] - times[i])[:, None]
    s = (samples - times[i])[:, None] / h
    h00, h10 = 2 * s**3 - 3 * s**2 + 1, s**3 - 2 * s**2 + s
    h01, h11 = -2 * s**3 + 3 * s**2, s**3 - s**2
    return h00 * pos[i] + h10 * h * vel[i] + h01 * pos[i + 1] + h11 * h * vel[i + 1]


def check_points(points, dt, limits=JOINT_LIMITS):
    """Why these firmware-angle setpoints (degrees) must not be played, or None."""
    for j in range(ARM_JOINTS):
        worst = float(np.abs(points[:, j]).max())
        if worst > limits[j] + LIMIT_MARGIN_DEG:
            return f"joint {j} reaches {worst:.1f} deg (limit {limits[j]:.0f})"
    if len(points) > 1:
        fastest = float((np.abs(np.diff(points, axis=0)) / dt).max())
        if fastest > REAL_MAX_VEL_DEG_S:
            return f"{fastest:.0f} deg/s (limit {REAL_MAX_VEL_DEG_S:.0f}): lower the velocity scaling"
    return None


class MoveItD1Bridge(Node):
    """The physical D1 arm as a FollowJointTrajectory controller."""

    def __init__(self, signs, arm=None, start_stow=False):
        super().__init__("moveit_d1_bridge")
        self.signs = np.array(signs, dtype=float)
        self.arm_lock = (
            threading.Lock()
        )  # telemetry is drained on read: one reader at a time
        self.measured = None  # firmware degrees, joints 0-5
        self.measured_at = 0.0
        self.busy = False
        try:
            self.connect(arm, start_stow)
        except (RuntimeError, KeyboardInterrupt):
            self.destroy_node()  # left half-built it crashes Python on exit
            raise

        group = ReentrantCallbackGroup()
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(0.1, self.poll_joint_states, callback_group=group)
        self.action = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=group,
        )
        self.get_logger().info(
            f"Bridge ready: {STREAM_RATE:.0f} Hz, on the robot: {robot_has(self.arm, 'traj_points')}, "
            f"signs {[int(s) for s in signs]}, gripper held at {self.gripper:.1f} deg"
        )

    def connect(self, arm, start_stow):
        """Reach the arm, power and enable it, and move it to stow if asked."""
        self.arm = arm or D1Arm(
            iface=os.environ.get("ARM_IFACE"), robot=os.environ.get("ARM_ROBOT")
        )
        if not self.arm.ok:
            raise RuntimeError(
                "the D1 arm did not answer (powered? ARM_ROBOT / ARM_IFACE?)"
            )
        self.bring_up()
        self.gripper = steady_gripper(self.arm)
        self.settle_cfg = MotionConfig(
            max_vel=REAL_MAX_VEL_DEG_S,
            max_acc=REAL_MAX_ACC_DEG_S2,
            min_duration=0.5,
            tolerance=GOAL_TOLERANCE_DEG,
            settle_time=0.3,
            settle_timeout=2.0,
        )
        if start_stow:
            self.fold_to_stow()

    def bring_up(self):
        """Power and enable, each only if needed: enable-all on an enabled arm
        drives it back to its last target."""
        status = (self.arm.state() or {}).get("data")
        if not status:
            raise RuntimeError("no status from the arm")
        if status.get("power_status") != 1:
            self.get_logger().info("Motor power on")
            self.arm.power(True)
            time.sleep(T_POWER)
            status = (self.arm.state() or {}).get("data", {})
        if status.get("enable_status") == 1:
            self.get_logger().info("Motors already enabled (holding): not re-enabled")
            return
        self.get_logger().info("Enabling the motors")
        self.arm.enable(ENABLE_FULL)
        time.sleep(T_ENABLE)
        if (self.arm.state() or {}).get("data", {}).get("enable_status") != 1:
            raise RuntimeError("the motors did not enable")

    def fold_to_stow(self):
        """From the fold, glide to stow. Any other pose: leave the arm alone."""
        now = read_angles(self.arm, timeout=2.0)
        if now is None:
            raise RuntimeError("no joint angles from the arm")
        now = np.array(now[:ARM_JOINTS])
        stow = np.array(STOW_RAD) * RAD2DEG * self.signs
        if np.abs(now - stow).max() <= STOW_TOLERANCE_DEG:
            self.get_logger().info("Already at stow")
            return
        off = np.abs(now - np.array(FOLDED_ANGLES[:ARM_JOINTS]))
        if (off > FOLD_TOLERANCE_DEG).any():
            joints = ", ".join(
                f"joint {j} off by {off[j]:.1f}"
                for j in np.flatnonzero(off > FOLD_TOLERANCE_DEG)
            )
            self.get_logger().warn(
                f"Not on the fold ({joints} deg, arm at {np.round(now, 1).tolist()}): "
                "staying here. If MoveIt refuses this pose, move the arm to stow"
            )
            return
        if np.abs(stow - now).max() > STOW_MAX_MOVE_DEG:
            raise RuntimeError("stow is too far from the fold: check ARM_JOINT_SIGNS")
        self.get_logger().info(
            f"Moving from the fold to stow at {STOW_VEL_DEG_S:.0f} deg/s"
        )
        cfg = replace(
            self.settle_cfg,
            max_vel=STOW_VEL_DEG_S,
            max_acc=STOW_ACC_DEG_S2,
            settle_timeout=4.0,
        )
        try:
            done = smooth_move(
                self.arm,
                list(stow),
                gripper=self.gripper,
                cfg=cfg,
                limits={
                    j: JOINT_LIMITS[j] + LIMIT_MARGIN_DEG for j in range(ARM_JOINTS)
                },
            )
        except KeyboardInterrupt:
            self.stop_arm()  # on the Go2 the robot would otherwise finish the move
            raise
        if not done.reached:
            raise RuntimeError(f"stow not reached (off by {done.final_error:.1f} deg)")

    # ------------------------------------------------------------------ state
    def poll_angles(self):
        """Newest telemetry without waiting (d1_motion.read_angles sleeps)."""
        for text in self.arm._take():
            try:
                m = json.loads(text)
            except ValueError:
                continue
            if m.get("address") == ADDR_TELEM and m.get("funcode") == 1:
                data = m.get("data", {})
                angles = [data.get(f"angle{j}") for j in range(ARM_JOINTS)]
                if None not in angles:
                    self.measured, self.measured_at = (
                        [float(a) for a in angles],
                        time.monotonic(),
                    )

    def publish_joint_states(self):
        if self.measured is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = MOVEIT_JOINTS + FINGER_JOINTS
        msg.position = list(np.array(self.measured) * self.signs * DEG2RAD) + [0.0, 0.0]
        self.js_pub.publish(msg)

    def poll_joint_states(self):
        """10 Hz. During a move execute() holds the lock and publishes instead."""
        if self.arm_lock.acquire(blocking=False):
            try:
                self.poll_angles()
            finally:
                self.arm_lock.release()
            self.publish_joint_states()

    # ---------------------------------------------------------------- commands
    def send_setpoint(self, angles):
        """One funcode 2 setpoint, mode 0, with no wait (arm._send sleeps 0.1 s)."""
        data = {
            "mode": 0,
            **{f"angle{j}": round(float(a), 2) for j, a in enumerate(angles)},
            "angle6": self.gripper,
        }
        self.arm.seq += 1
        payload = {
            "seq": self.arm.seq,
            "address": ADDR_CMD,
            "funcode": FUNC_JOINTS,
            "data": data,
        }
        self.arm._write(json.dumps(payload, separators=(",", ":")))

    def play(self, points, dt, goal_handle):
        """Play the setpoints. Returns None, or why it stopped early."""
        on_robot = robot_has(self.arm, "traj_points")
        if on_robot:
            self.arm.link.write(
                json.dumps(
                    {
                        "traj_points": {
                            "dt": dt,
                            "points": [[round(float(a), 2) for a in p] for p in points],
                            "gripper": self.gripper,
                            "mode": 0,
                        }
                    },
                    separators=(",", ":"),
                )
            )
        t0 = self.measured_at = time.monotonic()
        last_publish = 0.0
        for i, point in enumerate(points):
            if goal_handle.is_cancel_requested:
                return "cancelled"
            if not on_robot:
                self.send_setpoint(point)
            self.poll_angles()
            now = time.monotonic()
            if now - self.measured_at > TELEMETRY_TIMEOUT_S:
                return "no telemetry from the arm"
            reference = points[max(0, i - int(PATH_LAG_S / dt))]
            behind = float(np.abs(reference - np.array(self.measured)).max())
            if behind > PATH_TOLERANCE_DEG:
                return (
                    f"{behind:.1f} deg behind the path: is something blocking the arm?"
                )
            if now - last_publish >= 0.1:
                last_publish = now
                self.publish_joint_states()
            time.sleep(max(0.0, t0 + (i + 1) * dt - time.monotonic()))
        if on_robot:
            time.sleep(0.25)  # the robot's last sends
        return None

    # ------------------------------------------------------------------ action
    def on_goal(self, _goal):
        if self.busy:
            self.get_logger().warn("Goal refused: the arm is still moving")
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    def execute(self, goal_handle):
        try:
            return self.run_goal(goal_handle)
        except Exception as e:  # e.g. the Wi-Fi link to the Go2 bridge broke
            self.stop_arm()
            return self.fail(
                goal_handle,
                FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                f"{e}",
            )
        finally:
            self.busy = False

    def fail(self, goal_handle, code, why):
        self.get_logger().error(f"Trajectory failed: {why}")
        goal_handle.abort()
        return FollowJointTrajectory.Result(error_code=code, error_string=why)

    def wait_until_still(self, timeout=2.0):
        """After a stop the arm coasts to its last setpoint: planning from a moving
        arm fails MoveIt's start check. Publishes the joints meanwhile."""
        end = time.monotonic() + timeout
        last = None
        while time.monotonic() < end:
            time.sleep(0.2)
            self.poll_angles()
            self.publish_joint_states()
            now = np.array(self.measured)
            if last is not None and np.abs(now - last).max() < 0.3:
                return
            last = now

    def stop_arm(self):
        """Stop the robot's playback; streamed from here, the arm holds the last setpoint."""
        try:
            stop_stream(self.arm)
        except Exception as e:
            self.get_logger().error(f"Could not stop the trajectory: {e}")

    def run_goal(self, goal_handle):
        Result = FollowJointTrajectory.Result
        dt = 1.0 / STREAM_RATE
        try:
            points = resample(goal_handle.request.trajectory, STREAM_RATE)
        except ValueError as e:
            return self.fail(goal_handle, Result.INVALID_JOINTS, str(e))
        points = points * RAD2DEG * self.signs
        why = check_points(points, dt)
        if why:
            return self.fail(goal_handle, Result.INVALID_GOAL, why)

        with self.arm_lock:
            self.poll_angles()
            if (
                self.measured is None
                or time.monotonic() - self.measured_at > TELEMETRY_TIMEOUT_S
            ):
                return self.fail(
                    goal_handle,
                    Result.PATH_TOLERANCE_VIOLATED,
                    "no telemetry from the arm",
                )
            off = float(np.abs(points[0] - np.array(self.measured)).max())
            if off > START_TOLERANCE_DEG:
                return self.fail(
                    goal_handle,
                    Result.PATH_TOLERANCE_VIOLATED,
                    f"the trajectory starts {off:.1f} deg from where the arm is",
                )

            self.get_logger().info(
                f"Playing {len(points)} setpoints, {len(points) * dt:.1f} s"
            )
            why = self.play(points, dt, goal_handle)
            if why:
                self.stop_arm()
                self.wait_until_still()
                if why == "cancelled":
                    self.get_logger().info("Trajectory cancelled: holding")
                    goal_handle.canceled()
                    return Result(error_code=Result.SUCCESSFUL, error_string=why)
                return self.fail(goal_handle, Result.PATH_TOLERANCE_VIOLATED, why)

            done = smooth_move(
                self.arm,
                list(points[-1]),
                gripper=self.gripper,
                cfg=self.settle_cfg,
                limits={
                    j: JOINT_LIMITS[j] + LIMIT_MARGIN_DEG for j in range(ARM_JOINTS)
                },
            )
            self.poll_angles()
            self.publish_joint_states()
        if not done.reached:
            return self.fail(
                goal_handle,
                Result.GOAL_TOLERANCE_VIOLATED,
                f"stopped {done.final_error:.1f} deg from the goal",
            )
        self.get_logger().info(f"Goal reached (off by {done.final_error:.1f} deg)")
        goal_handle.succeed()
        return Result(error_code=Result.SUCCESSFUL)


def main():
    try:
        signs = parse_signs(os.environ.get("ARM_JOINT_SIGNS", ""))
    except ValueError as e:
        sys.exit(f"[bridge] {e}")
    # Python's own handlers, not rclpy's: those only flag the shutdown, and the
    # stow move in the constructor would run on after Ctrl+C. Explicit because
    # a background job (arm_moveit.sh) starts with SIGINT ignored.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    try:
        node = MoveItD1Bridge(signs, start_stow=os.environ.get("ARM_START_STOW") == "1")
    except (RuntimeError, KeyboardInterrupt) as e:
        rclpy.shutdown()
        sys.exit(
            f"[bridge] {e}" if str(e) else "[bridge] stopped: the arm holds where it is"
        )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_arm()
        node.get_logger().info(
            "Bridge shutting down: the arm stays enabled and holding."
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
