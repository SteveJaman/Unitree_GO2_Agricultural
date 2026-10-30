#!/usr/bin/env python3
"""
d1_motion.py

Smooth, shake-free moves for the D1 arm: a minimum-jerk trajectory streamed as
small setpoints, with an outer PID loop on the measured joint angles.

The arm runs its own servo loops; their gains are not reachable from the
protocol. What the host controls is how targets are fed in. One big funcode 2
jump makes the servos accelerate hard, overshoot and ring around the target,
and repeated corrections near the target make it hunt: both look like shaking.
So a move here is:

  1. Trajectory: from the measured angles to the target along a minimum-jerk
     profile (zero speed and acceleration at both ends), timed so no joint
     exceeds max_vel (deg/s) or max_acc (deg/s^2). Every joint arrives at the
     same time.
  2. Setpoints at `rate` Hz, each a small step (<= ~1.5 deg) the servos follow
     without overshoot (stream_mode, 1: the firmware's own smoothing on top).
     With the arm plugged into the Go2, the trajectory runs ON THE ROBOT
     (arm_bridge_wifi.py "traj", 50 Hz over its wired link): Wi-Fi latency
     spikes (200 ms seen) would bunch setpoints streamed from here into bursts
     and shake the arm on every move. Wired, or with an older bridge, they
     are streamed from here (one send each, no triple repeat).
  3. Outer PID per joint on (target - measured), only once the trajectory is
     done (while settling): it corrects what the servos leave, mainly gravity
     sag on the shoulder and elbow. During the move the arm always lags the
     setpoint a little; correcting that lag fights the servos' own loop and
     shakes the arm, so the move itself is pure trajectory.
       - P and D only act outside a `deadband` (default 0.5 deg): telemetry
         noise never makes the arm hunt.
       - I is capped (i_limit), as is the whole correction.
       - D uses the measured rate, low-pass filtered (no derivative kick).
     The wrist joints (3-5) carry almost no load and barely sag, but their
     readings are as noisy as the others: on them P and D only amplify noise
     into a shaking claw. They get their own gentle profile (wrist_kp/ki/kd,
     wrist_deadband): by default no P or D, a slow integral only.
  4. Settle: hold the target (PID still running) until every joint stays
     within `tolerance` for `settle_time`, or `settle_timeout` passes. Once the
     trajectory is done, a command is only re-sent when the correction moves
     by more than resend_deg: re-sending the same target 20 times a second
     makes the firmware restart its own smoothing each time (a buzz at the
     claw). The last command goes out with the usual triple repeat.

hold_jitter() watches a held pose and reports how much each joint wanders
(play_arm.py prints it, to tell which joint shakes).

Telemetry arrives at ~10 Hz; between readings the last one is used.
"""

import json
import math
import time
from dataclasses import dataclass, field

from d1_arm_control import FUNC_JOINTS

ARM_JOINTS = 6


@dataclass
class MotionConfig:
    max_vel: float = 30.0  # deg/s, fastest joint
    max_acc: float = 60.0  # deg/s^2
    min_duration: float = 0.8  # s, even for tiny moves
    rate: float = 20.0  # Hz, setpoint stream
    kp: float = 0.3  # outer PID gains (deg of correction per deg of error, ...)
    ki: float = 0.4  # per s
    kd: float = 0.02  # s
    i_limit: float = 4.0  # deg, integral term cap
    max_correction: float = 6.0  # deg, total correction cap
    deadband: float = 0.5  # deg: no P/D inside it (no hunting)
    d_filter: float = 0.3  # 0..1, low-pass on the measured rate (lower = smoother)
    tolerance: float = 1.5  # deg, counted as reached
    settle_time: float = 0.5  # s within tolerance to finish
    settle_timeout: float = 4.0  # s after the trajectory
    wrist_kp: float = 0.0  # wrist joints 3-5: light, little sag; no P/D by default
    wrist_ki: float = 0.15  # per s, slow sag correction only
    wrist_kd: float = 0.0
    wrist_deadband: float = 1.0  # deg
    resend_deg: float = (
        0.2  # after the trajectory: re-send only if the command moves this much
    )
    cmd_mode: int = 1  # funcode 2 "mode" of the final / settle commands
    stream_mode: int = 1  # funcode 2 "mode" of the trajectory setpoints
    robot_traj: bool = True  # Go2 bridge with "traj": run the trajectory on the robot
    robot_rate: float = 50.0  # Hz, setpoints on the robot
    # funcode 2 "mode" of the robot's setpoints. 0 (direct): at a steady 50 Hz
    # the steps are tiny. Mode 1 restarts the firmware's 0.3 s ramp at every
    # step, so the arm falls behind the path and lurches to catch up at the end
    # (simulated: wrist wobble +/-20 deg after arriving; mode 0: none).
    robot_mode: int = 0
    pid: bool = True  # False: trajectory only, no outer loop


@dataclass
class MoveResult:
    reached: bool
    duration: float
    max_error: float  # deg, worst |setpoint - measured| during the move
    final_error: float  # deg, worst |target - measured| at the end
    overshoot: float  # deg, worst travel past the target
    corrections: list = field(default_factory=list)


def min_jerk(tau):
    """Minimum-jerk blend 0..1 for tau in 0..1 (zero speed and acceleration at both ends)."""
    tau = min(1.0, max(0.0, tau))
    return tau**3 * (10 - 15 * tau + 6 * tau * tau)


def plan_duration(start, target, cfg):
    """Shortest minimum-jerk duration that respects max_vel and max_acc on every joint."""
    d = max(abs(t - s) for s, t in zip(start, target))
    if d < 1e-6:
        return cfg.min_duration
    # Peak speed of a min-jerk move: 1.875 d / T; peak acceleration: 5.774 d / T^2.
    return max(
        cfg.min_duration, 1.875 * d / cfg.max_vel, math.sqrt(5.774 * d / cfg.max_acc)
    )


def read_angles(arm, timeout=0.02):
    """Joints 0-6 from the newest telemetry, or None (non-blocking by default)."""
    d = (arm.angles(timeout=timeout) or {}).get("data", {})
    vals = [d.get(f"angle{i}") for i in range(7)]
    if any(v is None for v in vals[:ARM_JOINTS]):
        return None
    return [float(v) if v is not None else 0.0 for v in vals]


class _JointPID:
    def __init__(self, cfg, wrist=False):
        self.cfg = cfg
        self.kp, self.ki, self.kd = (
            (cfg.wrist_kp, cfg.wrist_ki, cfg.wrist_kd)
            if wrist
            else (cfg.kp, cfg.ki, cfg.kd)
        )
        self.deadband = cfg.wrist_deadband if wrist else cfg.deadband
        self.i = 0.0
        self.rate = 0.0
        self.prev = None

    def update(self, error, measured, dt, integrate=True):
        c = self.cfg
        # Measured rate, low-pass filtered: D on the measurement (no kick when the setpoint moves).
        if self.prev is not None and dt > 0:
            self.rate += c.d_filter * ((measured - self.prev) / dt - self.rate)
        self.prev = measured
        if integrate:
            self.i = max(-c.i_limit, min(c.i_limit, self.i + self.ki * error * dt))
        out = self.i
        if abs(error) > self.deadband:
            out += self.kp * error - self.kd * self.rate
        return max(-c.max_correction, min(c.max_correction, out))


def smooth_move(arm, target, gripper=0.0, cfg=None, limits=None, log=None):
    """Move joints 0-5 to `target` (degrees) smoothly; see the module docstring.

    `limits`: {joint: max |deg|} that commands are clamped to (e.g. JOINT_REACH).
    Returns a MoveResult. Without telemetry it falls back to one funcode 2
    command and reports reached=False.
    """
    cfg = cfg or MotionConfig()
    target = [float(v) for v in target[:ARM_JOINTS]]
    now_angles = read_angles(arm, timeout=2.0)

    def send(cmd_angles, repeat=1, mode=None):
        if limits:
            cmd_angles = [
                max(-limits[j], min(limits[j], a)) for j, a in enumerate(cmd_angles)
            ]
        data = {
            "mode": cfg.cmd_mode if mode is None else mode,
            **{f"angle{j}": round(a, 2) for j, a in enumerate(cmd_angles)},
            "angle6": gripper,
        }
        arm._send(FUNC_JOINTS, data, repeat=repeat)

    if now_angles is None:
        if log:
            log("  [!] no joint telemetry: one direct move, no smoothing")
        send(target, repeat=3)
        return MoveResult(False, 0.0, float("nan"), float("nan"), float("nan"))

    start = now_angles[:ARM_JOINTS]
    if limits:
        target = [max(-limits[j], min(limits[j], a)) for j, a in enumerate(target)]
    duration = plan_duration(start, target, cfg)
    link = getattr(arm, "link", None)
    on_robot = (
        cfg.robot_traj
        and link is not None
        and '"traj"' in (getattr(link, "hello", None) or "")
    )
    if on_robot:
        # One request over Wi-Fi; the robot streams the setpoints itself.
        link.write(
            json.dumps(
                {
                    "traj": {
                        "target": [round(a, 2) for a in target],
                        "gripper": gripper,
                        "max_vel": cfg.max_vel,
                        "max_acc": cfg.max_acc,
                        "rate": cfg.robot_rate,
                        "mode": cfg.robot_mode,
                        "min_duration": cfg.min_duration,
                    }
                }
            )
        )
        duration += 0.3  # the robot's own start and final sends
    pids = [_JointPID(cfg, wrist=j >= 3) for j in range(ARM_JOINTS)]
    last_cmd = None
    direction = [1.0 if t >= s else -1.0 for s, t in zip(start, target)]
    measured = list(start)
    max_err = overshoot = 0.0
    period = 1.0 / cfg.rate
    t0 = last = time.monotonic()
    in_tol_since = None
    end = t0 + duration + cfg.settle_timeout
    corrections = [0.0] * ARM_JOINTS
    while True:
        now = time.monotonic()
        dt, last = now - last, now
        fresh = read_angles(arm)
        if fresh:
            measured = fresh[:ARM_JOINTS]
        s = min_jerk((now - t0) / duration)
        setpoint = [a + (b - a) * s for a, b in zip(start, target)]
        errors = [sp - m for sp, m in zip(setpoint, measured)]
        max_err = max(max_err, max(abs(e) for e in errors))
        overshoot = max(
            overshoot, max((m - t) * d for m, t, d in zip(measured, target, direction))
        )
        moving = now - t0 < duration
        if cfg.pid and not moving:  # settle phase only: no fighting the servos mid-move
            corrections = [
                p.update(e, m, dt) for p, e, m in zip(pids, errors, measured)
            ]
        cmd = [sp + c for sp, c in zip(setpoint, corrections)]
        if moving:
            if not on_robot:
                send(cmd, mode=cfg.stream_mode)
            last_cmd = cmd
        elif (
            last_cmd is None
            or max(abs(a - b) for a, b in zip(cmd, last_cmd)) > cfg.resend_deg
        ):
            send(cmd)
            last_cmd = cmd

        if not moving:
            final = max(abs(t - m) for t, m in zip(target, measured))
            if final <= cfg.tolerance:
                in_tol_since = in_tol_since or now
                if now - in_tol_since >= cfg.settle_time:
                    break
            else:
                in_tol_since = None
            if now >= end:
                break
        time.sleep(max(0.0, period - (time.monotonic() - now)))

    # Last command once more, reliably: the corrected target the arm now holds.
    send([t + c for t, c in zip(target, corrections)], repeat=3)
    final = max(abs(t - m) for t, m in zip(target, measured))
    return MoveResult(
        final <= cfg.tolerance,
        time.monotonic() - t0,
        max_err,
        final,
        max(0.0, overshoot),
        corrections,
    )


def hold_jitter(arm, seconds, period=0.1):
    """Watch a held pose for `seconds`: peak-to-peak wander (deg) per joint 0-6, or None."""
    lo = hi = None
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        a = read_angles(arm, timeout=period)
        if a:
            lo = a if lo is None else [min(x, y) for x, y in zip(lo, a)]
            hi = a if hi is None else [max(x, y) for x, y in zip(hi, a)]
        time.sleep(period)
    return None if lo is None else [h - low for h, low in zip(hi, lo)]


def steady_gripper(arm, samples=5):
    """The claw's angle (joint 6) as the median of a few readings; 0.0 without telemetry."""
    vals = []
    for _ in range(samples):
        a = read_angles(arm, timeout=0.3)
        if a:
            vals.append(a[6])
    vals.sort()
    return round(vals[len(vals) // 2], 1) if vals else 0.0


# --------------------------------------------------------------------------
# Ruckig: jerk-limited (S-curve) trajectories, like an industrial robot.
# --------------------------------------------------------------------------
@dataclass
class RuckigConfig:
    max_vel: float = 60.0  # deg/s per joint
    max_acc: float = 90.0  # deg/s^2
    max_jerk: float = 300.0  # deg/s^3
    rate: float = 50.0  # Hz, streamed setpoints (every 0.02 s)
    mode: int = 0  # funcode 2 "mode": 0 = direct tracking of each setpoint
    robot_side: bool = True  # Go2 bridge with "traj_points": the robot plays the points
    tolerance: float = 2.0  # deg: final error counted as reached


def ruckig_points(start, target, cfg):
    """Time-optimal S-curve from start to target (degrees, joints 0-5), at rest at
    both ends, all joints synchronised. Returns (duration s, [points every 1/rate s])."""
    from ruckig import InputParameter, Result, Ruckig, Trajectory

    n = len(start)
    otg, inp, traj = Ruckig(n), InputParameter(n), Trajectory(n)
    inp.current_position = [float(v) for v in start]
    inp.current_velocity = [0.0] * n
    inp.current_acceleration = [0.0] * n
    inp.target_position = [float(v) for v in target]
    inp.target_velocity = [0.0] * n
    inp.target_acceleration = [0.0] * n
    inp.max_velocity = [cfg.max_vel] * n
    inp.max_acceleration = [cfg.max_acc] * n
    inp.max_jerk = [cfg.max_jerk] * n
    result = otg.calculate(inp, traj)
    if result not in (Result.Working, Result.Finished):
        raise ValueError(f"Ruckig could not plan the move ({result})")
    dt = 1.0 / cfg.rate
    steps = max(1, math.ceil(traj.duration / dt))
    points = [
        list(traj.at_time(min(traj.duration, i * dt))[0]) for i in range(1, steps + 1)
    ]
    points[-1] = [float(v) for v in target]
    return traj.duration, points


def robot_has(arm, feature):
    """True if the arm is reached through a Go2 bridge that advertises `feature`."""
    link = getattr(arm, "link", None)
    return link is not None and f'"{feature}"' in (getattr(link, "hello", None) or "")


def ruckig_move(arm, target, gripper=0.0, cfg=None, limits=None, log=None):
    """Move joints 0-5 to `target` (degrees) along a Ruckig S-curve.

    The trajectory is generated here and streamed at cfg.rate as funcode 2
    (cfg.mode, 0 = direct). Through a Go2 bridge with "traj_points" the whole
    sampled trajectory goes over Wi-Fi in one message and the robot plays it
    at the exact rate (streamed over Wi-Fi, latency spikes bunch the points
    and the arm shakes). The stream runs for exactly the trajectory's
    duration, without waiting on telemetry; one reading afterwards gives the
    final error. Ctrl+C (KeyboardInterrupt) propagates: the caller stops the
    robot's playback (stop_stream) and releases or locks the arm.
    """
    cfg = cfg or RuckigConfig()
    target = [float(v) for v in target[:ARM_JOINTS]]
    if limits:
        target = [max(-limits[j], min(limits[j], a)) for j, a in enumerate(target)]

    def send(angles, repeat=1):
        data = {
            "mode": cfg.mode,
            **{f"angle{j}": round(a, 2) for j, a in enumerate(angles)},
            "angle6": gripper,
        }
        arm._send(FUNC_JOINTS, data, repeat=repeat)

    now_angles = read_angles(arm, timeout=2.0)
    if now_angles is None:
        if log:
            log("  [!] no joint telemetry: one direct move, no trajectory")
        send(target, repeat=3)
        return MoveResult(False, 0.0, float("nan"), float("nan"), float("nan"))

    duration, points = ruckig_points(now_angles[:ARM_JOINTS], target, cfg)
    dt = 1.0 / cfg.rate
    t0 = time.monotonic()
    if cfg.robot_side and robot_has(arm, "traj_points"):
        arm.link.write(
            json.dumps(
                {
                    "traj_points": {
                        "dt": dt,
                        "points": [[round(a, 2) for a in p] for p in points],
                        "gripper": gripper,
                        "mode": cfg.mode,
                    }
                },
                separators=(",", ":"),
            )
        )
        time.sleep(duration + 0.25)  # the robot plays it; its final sends follow
    else:
        for i, p in enumerate(points, 1):
            send(p)
            time.sleep(max(0.0, t0 + i * dt - time.monotonic()))
        send(target, repeat=3)
    final_angles = read_angles(arm, timeout=0.5)
    final = (
        max(abs(t - m) for t, m in zip(target, final_angles[:ARM_JOINTS]))
        if final_angles
        else float("nan")
    )
    return MoveResult(
        final <= cfg.tolerance, time.monotonic() - t0, float("nan"), final, float("nan")
    )


def stop_stream(arm):
    """Stop a trajectory the Go2 bridge is playing (no-op when streaming from here)."""
    link = getattr(arm, "link", None)
    if link is not None and robot_has(arm, "traj"):
        try:
            link.write(json.dumps({"traj_stop": 1}))
        except ConnectionError:
            pass
