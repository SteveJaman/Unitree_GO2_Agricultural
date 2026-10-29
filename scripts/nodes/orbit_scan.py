#!/usr/bin/env python3
"""
orbit_scan.py

Hardcoded "orbit scan" for the D1 arm: look down at a table-top object and
sweep around it with the camera kept on target.

    1. Power on, enable (mode 80000) and home.
    2. Move to SCAN_POSE (camera facing down at the object).
    3. Sweep joint 0 (base) from -90° to +90° in 30 steps. At every step the
       wrist (joint 5) counter-rotates to -base * 0.8 so the camera stays
       centred on the object.
    4. Home and release (mode 0).

Whatever happens (Ctrl+C, an error, a lost link) the finally block homes the
arm and releases the motors. Homing first keeps a limp arm from dropping out
of the scan pose.

Uses the shared D1Arm of d1_arm_control.py: DDS on --iface (auto-detected: the
wired NIC on 192.168.123.x), or the Go2 Wi-Fi bridge when ARM_ROBOT / --robot
is set (arm plugged into the Go2).

Usage:
    python3 orbit_scan.py --dry-run                 # print the plan, no arm
    python3 orbit_scan.py --yes                     # run (wired iface auto-detected)
    python3 orbit_scan.py --robot 192.168.1.4 --yes # arm on the Go2, over Wi-Fi
    ./scripts/runtime/start_arm.sh orbit            # usual way: preflight + confirm
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from d1_arm_control import (  # noqa: E402
    D1Arm,
    ENABLE_FULL,
    FUNC_JOINTS,
    JOINT_LIMITS,
    T_ENABLE,
    T_HOME,
    T_RELEASE,
    default_arm_iface,
    preflight,
    print_state,
)

# Joints 0-5 in degrees: shoulder and elbow fold the camera down over the
# object, the wrist pitch (joint 4) tilts it onto the table.
SCAN_POSE = [0.0, -40.0, 60.0, 0.0, 30.0, 0.0]
BASE_START, BASE_END, STEPS = -90.0, 90.0, 30
WRIST_GAIN = 0.8  # joint 5 = -base * WRIST_GAIN


def sweep_targets(
    pose=SCAN_POSE, start=BASE_START, end=BASE_END, steps=STEPS, gain=WRIST_GAIN
):
    """Joint 0-5 targets for each sweep step: base swept, wrist counter-rotated."""
    out = []
    for i in range(steps):
        base = start + (end - start) * i / (steps - 1)
        target = list(pose)
        target[0] = round(base, 2)
        target[5] = round(-base * gain, 2)
        out.append(target)
    return out


def check_limits(targets):
    """Raise ValueError if any target is outside the arm's joint limits."""
    for n, t in enumerate(targets):
        for j, v in enumerate(t):
            if abs(v) > JOINT_LIMITS[j]:
                raise ValueError(
                    f"step {n}: joint {j} {v:+.1f}° beyond ±{JOINT_LIMITS[j]}°"
                )


def current(arm):
    """All seven reported angles (joints 0-5 + gripper), or None."""
    d = (arm.angles(timeout=2.0) or {}).get("data", {})
    vals = [d.get(f"angle{i}") for i in range(7)]
    return None if any(v is None for v in vals) else [float(v) for v in vals]


def move_to(arm, target, gripper, tol, timeout):
    """One multi-joint move (funcode 2); True once joints 0-5 are within tol."""
    cmd = {
        "mode": 1,
        **{f"angle{j}": v for j, v in enumerate(target)},
        "angle6": gripper,
    }
    arm._send(FUNC_JOINTS, cmd)
    t0 = time.time()
    while time.time() - t0 < timeout:
        now = current(arm)
        if now and max(abs(n - t) for n, t in zip(now[:6], target)) <= tol:
            return True
        time.sleep(0.1)
    return False


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--iface",
        default=default_arm_iface(),
        help="wired interface to the arm (default: auto-detected, %(default)s)",
    )
    ap.add_argument(
        "--domain", type=int, default=int(os.environ.get("ARM_DOMAIN", "0"))
    )
    ap.add_argument("--ip", default=os.environ.get("ARM_IP", "192.168.123.100"))
    ap.add_argument(
        "--robot",
        default=os.environ.get("ARM_ROBOT"),
        help="Go2 Wi-Fi IP: reach the arm through the robot's bridge",
    )
    ap.add_argument(
        "--dwell", type=float, default=0.3, help="s held at each sweep step"
    )
    ap.add_argument(
        "--tolerance", type=float, default=3.0, help="degrees counted as reached"
    )
    ap.add_argument(
        "--move-timeout", type=float, default=15.0, help="s allowed per move"
    )
    ap.add_argument("--yes", action="store_true", help="do not ask before moving")
    ap.add_argument(
        "--dry-run", action="store_true", help="print the plan, do not connect"
    )
    ap.add_argument("--skip-preflight", action="store_true")
    args = ap.parse_args()

    targets = sweep_targets()
    check_limits([SCAN_POSE] + targets)
    print(f"[orbit] scan pose {SCAN_POSE}")
    print(
        f"[orbit] base {BASE_START:+.0f}° -> {BASE_END:+.0f}° in {STEPS} steps, "
        f"wrist = -base * {WRIST_GAIN}"
    )
    if args.dry_run:
        for n, t in enumerate(targets):
            print(f"  {n + 1:2d}: base {t[0]:+7.2f}°  wrist {t[5]:+7.2f}°")
        return 0

    if args.robot:
        os.environ["ARM_ROBOT"] = args.robot
    if not args.yes:
        try:
            if (
                input("The arm will move through the orbit scan. Continue? [y/N] ")
                .strip()
                .lower()
                != "y"
            ):
                print("aborted")
                return 0
        except EOFError:
            print("aborted")
            return 0
    if not args.skip_preflight and not preflight(args.iface, args.ip):
        print("\n[!] Preflight failed")
        return 2
    arm = D1Arm(args.iface, args.domain)
    if not arm.ok:
        print("[!] cannot reach the arm (DDS discovery / Go2 bridge failed)")
        return 2

    enabled = False
    try:
        arm.power(True)
        time.sleep(1.5)
        arm.enable(ENABLE_FULL)
        time.sleep(T_ENABLE)
        if print_state(arm, "after-enable").get("enable_status") != 1:
            print("[!] the arm did not enable; nothing moved")
            return 3
        enabled = True
        print("[orbit] home")
        arm.home()
        time.sleep(T_HOME)
        gripper = (current(arm) or [0.0] * 7)[6]  # keep the end effector as it is

        print("[orbit] scan pose")
        if not move_to(arm, SCAN_POSE, gripper, args.tolerance, args.move_timeout):
            print(
                f"  [!] scan pose not reached within {args.move_timeout:.0f} s; continuing"
            )
        for n, t in enumerate(targets, 1):
            if not move_to(arm, t, gripper, args.tolerance, args.move_timeout):
                print(
                    f"  [!] step {n} not reached within {args.move_timeout:.0f} s; continuing"
                )
            print(f"  step {n:2d}/{STEPS}: base {t[0]:+7.2f}°  wrist {t[5]:+7.2f}°")
            time.sleep(args.dwell)
        print("[orbit] sweep done")
    finally:
        if enabled:
            # Normal end, Ctrl+C or an error: home first so the arm does not
            # drop from the scan pose, then always release (mode 0).
            try:
                print("[orbit] home")
                arm.home()
                time.sleep(T_HOME)
            except Exception as exc:  # still release below
                print(f"  [!] home failed: {exc}")
            try:
                arm.release()
                time.sleep(T_RELEASE)
                print("[orbit] released (mode 0)")
            except Exception as exc:
                print(f"  [!] release failed: {exc}")
            print_state(arm, "final")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\ninterrupted: arm homed and released")
        sys.exit(130)
