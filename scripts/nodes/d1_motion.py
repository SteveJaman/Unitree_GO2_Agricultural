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
  2. Setpoints streamed at `rate` Hz (one send each, no triple repeat), each
     a small step the servos follow without overshoot.
  3. Outer PID per joint on (setpoint - measured): corrects what the servos
     leave, mainly gravity sag on the shoulder and elbow.
       - P and D only act outside a `deadband` (default 0.5 deg): telemetry
         noise never makes the arm hunt.
       - I only builds in the last 10 % of the move and while settling (the
         arm lagging a moving setpoint is not sag; integrating it would
         overshoot), and is capped (i_limit), as is the whole correction.
       - D uses the measured rate, low-pass filtered (no derivative kick).
  4. Settle: hold the target (PID still running) until every joint stays
     within `tolerance` for `settle_time`, or `settle_timeout` passes. The last
     command is sent with the usual triple repeat, so it is not lost.

Telemetry arrives at ~10 Hz; between readings the last one is used.
"""

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
    cmd_mode: int = 1  # funcode 2 "mode" (1: the arm's own smoothing too)
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
    def __init__(self, cfg):
        self.cfg = cfg
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
            self.i = max(-c.i_limit, min(c.i_limit, self.i + c.ki * error * dt))
        out = self.i
        if abs(error) > c.deadband:
            out += c.kp * error - c.kd * self.rate
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

    def send(cmd_angles, repeat=1):
        if limits:
            cmd_angles = [
                max(-limits[j], min(limits[j], a)) for j, a in enumerate(cmd_angles)
            ]
        data = {
            "mode": cfg.cmd_mode,
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
    duration = plan_duration(start, target, cfg)
    pids = [_JointPID(cfg) for _ in range(ARM_JOINTS)]
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
        if cfg.pid:
            near_end = (now - t0) >= 0.9 * duration
            corrections = [
                p.update(e, m, dt, near_end) for p, e, m in zip(pids, errors, measured)
            ]
        send([sp + c for sp, c in zip(setpoint, corrections)])

        if now - t0 >= duration:
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
