#!/usr/bin/env python3
"""
play_arm.py

Replay the plant-scan poses taught with teach_arm.py
(config/arm_waypoints.json): lock, home, then visit every waypoint with a
smooth trajectory (funcode 2, mode 1) and hold it for the camera.

Workflow:
    1. Check every waypoint against the joint limits (nothing moves if one fails)
    2. Power on, lock (funcode 5, {"mode": 80000}), home (funcode 7)
    3. Per waypoint: move, wait until it is reached, hold --dwell s
       ("Waypoint N reached. [Camera should capture photo here]")
    4. Home again. The arm stays locked; --release releases it at home
Ctrl+C or an error locks the arm where it is (it never drops).

Usage:
    python3 scripts/tools/play_arm.py                  # replay once
    python3 scripts/tools/play_arm.py --dwell 5
    python3 scripts/tools/play_arm.py --dry-run        # print the plan, no arm
"""
import argparse
import json
import os
import sys
import time

# Reuse the shared D1Arm class from the control module
_here = os.path.dirname(os.path.abspath(__file__))
_repo = os.path.dirname(os.path.dirname(_here))
sys.path.insert(0, os.path.join(_repo, "scripts", "nodes"))
try:
    from d1_arm_control import (
        D1Arm, ENABLE_FULL, FUNC_JOINTS, JOINT_LIMITS, preflight,
        T_ENABLE, T_HOME, T_RELEASE, default_arm_iface,
    )
except ImportError as e:
    sys.exit(f"cannot import d1_arm_control: {e}")

WAYPOINTS = os.path.join(_repo, "config", "arm_waypoints.json")
ARM_JOINTS = 6


def load_waypoints(path):
    """[[a0..a5], ...] from teach_arm.py's file; exits with a message on errors."""
    if not os.path.isfile(path):
        sys.exit(f"[!] {path} not found: teach poses first (scripts/tools/teach_arm.py)")
    with open(path) as f:
        data = json.load(f)
    wps = [w["angles"] if isinstance(w, dict) else w for w in data.get("waypoints", [])]
    if not wps:
        sys.exit(f"[!] {path} has no waypoints")
    for n, a in enumerate(wps, 1):
        if len(a) != ARM_JOINTS:
            sys.exit(f"[!] waypoint {n}: {len(a)} angles, expected {ARM_JOINTS}")
        bad = [f"joint {j} {v:+.1f}°" for j, v in enumerate(a) if abs(v) > JOINT_LIMITS[j]]
        if bad:
            sys.exit(f"[!] waypoint {n} beyond the joint limits: {', '.join(bad)}")
    return [[float(v) for v in a] for a in wps]


def current(arm):
    """All seven reported angles (joints 0-5 + gripper), or None."""
    d = (arm.angles(timeout=2.0) or {}).get("data", {})
    vals = [d.get(f"angle{i}") for i in range(7)]
    return None if any(v is None for v in vals) else [float(v) for v in vals]


def wait_reached(arm, target, tol, timeout):
    """Wait until joints 0-5 are within tol degrees of target; True if reached."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        now = current(arm)
        if now and max(abs(n - t) for n, t in zip(now[:ARM_JOINTS], target)) <= tol:
            return True
        time.sleep(0.2)
    return False


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--iface", default=default_arm_iface())
    ap.add_argument("--domain", type=int, default=int(os.environ.get("ARM_DOMAIN", "0")))
    ap.add_argument("--ip", default=os.environ.get("ARM_IP", "192.168.123.100"))
    ap.add_argument("--file", default=WAYPOINTS, help="waypoint file (default: %(default)s)")
    ap.add_argument("--dwell", type=float, default=3.0, help="s held at each waypoint")
    ap.add_argument("--tolerance", type=float, default=3.0, help="degrees counted as reached")
    ap.add_argument("--move-timeout", type=float, default=15.0, help="s allowed per move")
    ap.add_argument("--release", action="store_true", help="release the arm at home when done")
    ap.add_argument("--dry-run", action="store_true", help="print the plan, do not connect")
    ap.add_argument("--skip-preflight", action="store_true")
    args = ap.parse_args()

    wps = load_waypoints(args.file)
    print(f"{len(wps)} waypoints from {args.file}")
    for n, a in enumerate(wps, 1):
        print(f"  {n}: {' '.join(f'{v:+7.1f}' for v in a)}")
    if args.dry_run:
        return 0

    if not args.skip_preflight and not preflight(args.iface, args.ip):
        print("\n[!] Preflight failed")
        return 2
    arm = D1Arm(args.iface, args.domain)
    if not arm.ok:
        print("[!] DDS discovery failed")
        return 2

    locked = False
    try:
        arm.power(True)
        time.sleep(1.5)
        arm.enable(ENABLE_FULL)
        time.sleep(T_ENABLE)
        st = (arm.state() or {}).get("data", {})
        if st.get("enable_status") != 1:
            print(f"[!] the arm did not enable (state: {st or 'none'}); nothing moved")
            return 3
        locked = True
        print("[*] home")
        arm.home()
        time.sleep(T_HOME)
        gripper = (current(arm) or [0.0] * 7)[6]   # keep the end effector as it is
        for n, a in enumerate(wps, 1):
            cmd = {"mode": 1, **{f"angle{j}": v for j, v in enumerate(a)}, "angle6": gripper}
            arm._send(FUNC_JOINTS, cmd)
            if not wait_reached(arm, a, args.tolerance, args.move_timeout):
                print(f"  [!] waypoint {n} not reached within {args.move_timeout:.0f} s "
                      f"(±{args.tolerance:.0f}°); holding anyway")
            print(f"Waypoint {n} reached. [Camera should capture photo here]")
            time.sleep(args.dwell)
        print("[*] home")
        arm.home()
        time.sleep(T_HOME)
        if args.release:
            print("[*] release (the arm goes limp at home)")
            arm.release()
            time.sleep(T_RELEASE)
            locked = False
    finally:
        if locked:
            # Normal end, Ctrl+C or an error: hold the current pose.
            arm.enable(ENABLE_FULL)
            print(f"Arm locked (mode {ENABLE_FULL}).")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\ninterrupted: arm locked where it stopped")
        sys.exit(130)
