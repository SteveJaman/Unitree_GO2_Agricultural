#!/usr/bin/env python3
"""
step_height_probe.py: how high the Go2 lifts its feet, from its own joint angles.

Runs ON THE DOG (Foxy, internal DDS: source ~/go2-nav2-lidar3d-onboard/robot-source-unitree-ros.sh).
Read only: it subscribes to /lowstate (leg joint angles, IMU) and /sportmodestate
(speed) and sends nothing, so it can run beside any walking test.

Each foot's position comes from forward kinematics of its leg (Go2: hip offsets
+-0.1934 / +-0.0465 m, ab/ad link 0.0955 m, thigh and calf 0.213 m), turned by the
body's roll and pitch, so the height is gravity-aligned. Per leg, the stance level
is the foot height while it is on the ground (a low percentile of the recording);
every swing above it by more than --swing-min is one step, and its peak above the
stance level is the step's clearance: how far the foot's lowest point rose over the
floor it stood on. Firmware V2.0 no longer reports foot_raise_height, so this is the
way to measure it (docs/hardware/traversability_handoff.md).

  python3 step_height_probe.py --duration 12            # record, then print per leg
  python3 step_height_probe.py --duration 12 --csv /tmp/steps.csv
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_go.msg import LowState, SportModeState

LEGS = ["FR", "FL", "RR", "RL"]  # /lowstate motor order: hip, thigh, calf per leg
HIP_X, HIP_Y, L_ABAD, L_THIGH, L_CALF = 0.1934, 0.0465, 0.0955, 0.213, 0.213


def foot_height(leg, q0, q1, q2, roll, pitch):
    """Gravity-aligned height (m) of a foot centre relative to the body origin."""
    sx = 1.0 if leg in ("FR", "FL") else -1.0
    sy = 1.0 if leg in ("FL", "RL") else -1.0
    x = -L_THIGH * math.sin(q1) - L_CALF * math.sin(q1 + q2)
    z = -L_THIGH * math.cos(q1) - L_CALF * math.cos(q1 + q2)
    y = sy * L_ABAD
    # ab/ad rotation about x
    y, z = y * math.cos(q0) - z * math.sin(q0), y * math.sin(q0) + z * math.cos(q0)
    px, py, pz = sx * HIP_X + x, sy * HIP_Y + y, z
    # body roll (x) then pitch (y): height component only
    pz_r = py * math.sin(roll) + pz * math.cos(roll)
    py_r = py * math.cos(roll) - pz * math.sin(roll)  # noqa: F841 (kept for clarity)
    return -px * math.sin(pitch) + pz_r * math.cos(pitch)


def percentile(values, p):
    s = sorted(values)
    if not s:
        return float("nan")
    k = min(len(s) - 1, max(0, int(round(p / 100.0 * (len(s) - 1)))))
    return s[k]


def swings(series, ground, swing_min):
    """Peak height above ground of every swing (series above ground + swing_min)."""
    peaks, cur = [], None
    for h in series:
        if h - ground > swing_min:
            cur = max(cur or 0.0, h - ground)
        elif cur is not None:
            peaks.append(cur)
            cur = None
    return peaks


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=12.0, help="s to record")
    ap.add_argument("--rate", type=float, default=100.0, help="samples/s kept (lowstate is ~500)")
    ap.add_argument("--swing-min", type=float, default=0.01, help="m above stance that counts as a swing")
    ap.add_argument("--csv", default="", help="write the samples here")
    ap.add_argument("--walk-min", type=float, default=0.05, help="m/s: samples slower than this are not walking")
    args = ap.parse_args()

    rclpy.init()
    node = Node("step_height_probe")
    rows, speed, last = [], [], {"t": 0.0, "v": 0.0}

    def on_low(m):
        now = time.monotonic()
        if now - last["t"] < 1.0 / args.rate:
            return
        last["t"] = now
        roll, pitch = m.imu_state.rpy[0], m.imu_state.rpy[1]
        row = [now, last["v"]]
        for i, leg in enumerate(LEGS):
            q = [m.motor_state[3 * i + k].q for k in range(3)]
            row.append(foot_height(leg, q[0], q[1], q[2], roll, pitch))
        rows.append(row)

    def on_sport(m):
        last["v"] = math.hypot(m.velocity[0], m.velocity[1])
        speed.append(last["v"])

    node.create_subscription(LowState, "/lowstate", on_low, qos_profile_sensor_data)
    node.create_subscription(SportModeState, "/sportmodestate", on_sport, 10)
    while not rows:  # recording starts with the first sample
        rclpy.spin_once(node, timeout_sec=0.05)
    print("RECORDING %.0f s of /lowstate (read only)..." % args.duration, flush=True)
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()

    n = len(rows)
    if n < 20:
        print("only %d samples from /lowstate: is the robot's DDS environment sourced?" % n)
        return 1
    v = sorted(speed)
    print("samples %d over %.1f s; speed median %.2f m/s, max %.2f m/s"
          % (n, rows[-1][0] - rows[0][0], v[len(v) // 2] if v else float("nan"), v[-1] if v else float("nan")))
    walking = [r for r in rows if r[1] > args.walk_min]
    print("walking samples: %d (%.1f s above %.2f m/s); distinct FR heights while walking: %d"
          % (len(walking), len(walking) / args.rate, args.walk_min, len(set(round(r[2], 4) for r in walking))))
    if len(walking) < args.rate:
        print("less than 1 s of walking recorded: nothing to measure")
        return 1
    print("leg  stance_h   steps  clearance_median  min    max   (m, foot lowest point over its stance floor)")
    all_peaks = []
    for i, leg in enumerate(LEGS):
        series = [r[2 + i] for r in walking]
        ground = percentile(series, 10)
        peaks = swings(series, ground, args.swing_min)
        all_peaks += peaks
        if peaks:
            print("%s   %+.3f   %5d   %.3f             %.3f  %.3f"
                  % (leg, ground, len(peaks), percentile(peaks, 50), min(peaks), max(peaks)))
        else:
            print("%s   %+.3f   %5d   (no swings: standing?)" % (leg, ground, 0))
    if all_peaks:
        print("ALL  steps %d, clearance median %.3f m, 10th percentile %.3f m (use this for step_over_height_max, minus a margin)"
              % (len(all_peaks), percentile(all_peaks, 50), percentile(all_peaks, 10)))
    if args.csv:
        with open(args.csv, "w") as f:
            f.write("t,speed," + ",".join(LEGS) + "\n")
            for r in rows:
                f.write(",".join("%.4f" % x for x in r) + "\n")
        print("samples written to", args.csv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
