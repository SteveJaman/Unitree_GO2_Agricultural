#!/usr/bin/env python3
"""
play_arm.py

Replay the home and plant-scan poses taught with teach_arm.py
(config/arm_waypoints.json): lock, go home, then visit every pose and hold it
for the camera.

Moves are shake-free by default (scripts/nodes/d1_motion.py): a minimum-jerk
trajectory from the measured angles, streamed as small setpoints, with an
outer PID loop that removes gravity sag without hunting around the target.
--no-pid sends one funcode 2 jump per pose instead (the old behaviour).
Tune with --max-vel / --max-acc (speed) and --kp / --ki / --kd / --deadband,
or the matching ARM_* environment variables (start_arm.sh play passes them).

Usage:
    python3 scripts/tools/play_arm.py                  # replay once (fast defaults)
    python3 scripts/tools/play_arm.py --dwell 3.0      # wait longer at each pose
    python3 scripts/tools/play_arm.py --mode 0         # use fast mode (might shake)
    python3 scripts/tools/play_arm.py --max-vel 45     # faster smooth moves
    python3 scripts/tools/play_arm.py --no-pid         # one jump per pose (old)
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
        D1Arm, ENABLE_FULL, FUNC_JOINTS, preflight,
        T_ENABLE, T_HOME, T_RELEASE, default_arm_iface,
    )
    from d1_motion import MotionConfig, smooth_move
except ImportError as e:
    sys.exit(f"cannot import d1_arm_control / d1_motion: {e}")

WAYPOINTS = os.path.join(_repo, "config", "arm_waypoints.json")
ARM_JOINTS = 6
MAX_TEACH_REACH = 150.0


def check_pose(label, a):
    if len(a) != ARM_JOINTS:
        sys.exit(f"[!] {label}: {len(a)} angles, expected {ARM_JOINTS}")
    bad = [f"joint {j} {v:+.1f}° (max ±150°)"
           for j, v in enumerate(a) if abs(v) > MAX_TEACH_REACH]
    if bad:
        sys.exit(f"[!] {label} beyond the 150° limit: {', '.join(bad)}")
    return [float(v) for v in a]


def load_waypoints(path):
    if not os.path.isfile(path):
        sys.exit(f"[!] {path} not found: teach poses first (scripts/tools/teach_arm.py)")
    with open(path) as f:
        data = json.load(f)
    raw = data.get("waypoints", [])
    if not raw:
        sys.exit(f"[!] {path} has no waypoints")
    wps = []
    for n, w in enumerate(raw, 1):
        a, name = (w["angles"], w.get("name", "")) if isinstance(w, dict) else (w, "")
        wps.append((name, check_pose(f"waypoint {n}", a)))
    home = data.get("home")
    return (check_pose("home", home["angles"]) if home else None), wps


def current(arm):
    d = (arm.angles(timeout=2.0) or {}).get("data", {})
    vals = [d.get(f"angle{i}") for i in range(7)]
    return None if any(v is None for v in vals) else [float(v) for v in vals]


def wait_reached(arm, target, tol, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        now = current(arm)
        if now and max(abs(n - t) for n, t in zip(now[:ARM_JOINTS], target)) <= tol:
            return True
        time.sleep(0.05) # Check more frequently for faster transitions
    return False


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--iface", default=default_arm_iface())
    ap.add_argument("--domain", type=int, default=int(os.environ.get("ARM_DOMAIN", "0")))
    ap.add_argument("--ip", default=os.environ.get("ARM_IP", "192.168.123.150"))
    ap.add_argument("--file", default=WAYPOINTS)

    # --- FASTER DEFAULTS ---
    ap.add_argument("--dwell", type=float, default=1.5, help="Seconds held at each waypoint (default: 1.0)")
    ap.add_argument("--tolerance", type=float, default=2.0, help="Degrees counted as reached (default: 2.0)")
    ap.add_argument("--move-timeout", type=float, default=8.0, help="Seconds allowed per move (default: 8.0)")
    ap.add_argument("--mode", type=int, default=1, choices=[0, 1],
                    help="Trajectory mode: 0=fast/jerky, 1=smooth/slower (default: 1)")

    # --- Smooth motion + PID (d1_motion.py) ---
    env = os.environ.get
    ap.add_argument("--no-pid", action="store_true", default=env("ARM_PID", "1") == "0",
                    help="one funcode 2 jump per pose instead of the smooth PID move")
    ap.add_argument("--max-vel", type=float, default=float(env("ARM_MAX_VEL", "30")),
                    help="deg/s, fastest joint (default %(default)s)")
    ap.add_argument("--max-acc", type=float, default=float(env("ARM_MAX_ACC", "60")),
                    help="deg/s^2 (default %(default)s)")
    ap.add_argument("--kp", type=float, default=float(env("ARM_KP", "0.3")), help="default %(default)s")
    ap.add_argument("--ki", type=float, default=float(env("ARM_KI", "0.4")), help="per s, default %(default)s")
    ap.add_argument("--kd", type=float, default=float(env("ARM_KD", "0.02")), help="s, default %(default)s")
    ap.add_argument("--deadband", type=float, default=float(env("ARM_DEADBAND", "0.5")),
                    help="deg: no P/D inside it, so the arm never hunts (default %(default)s)")

    ap.add_argument("--release", action="store_true")
    ap.add_argument("--factory-home", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--skip-preflight", action="store_true")
    args = ap.parse_args()

    home, wps = load_waypoints(args.file)
    if args.factory_home:
        home = None

    print(f"{len(wps)} waypoints from {args.file}")
    print(f"  mode: {args.mode} | dwell: {args.dwell}s | timeout: {args.move_timeout}s")
    print("  motion: " + ("one jump per pose (--no-pid)" if args.no_pid else
          f"smooth + PID ({args.max_vel:g} deg/s, kp {args.kp:g} ki {args.ki:g} kd {args.kd:g}, "
          f"deadband {args.deadband:g} deg)"))
    print(f"  home: {' '.join(f'{v:+7.1f}' for v in home) if home else 'factory zero pose'}")

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
        time.sleep(1.0)
        arm.enable(ENABLE_FULL)
        time.sleep(T_ENABLE)

        locked = True
        gripper = (current(arm) or [0.0] * 7)[6]

        cfg = MotionConfig(max_vel=args.max_vel, max_acc=args.max_acc, kp=args.kp, ki=args.ki,
                           kd=args.kd, deadband=args.deadband, tolerance=args.tolerance,
                           settle_timeout=args.move_timeout, cmd_mode=args.mode)
        limits = {j: MAX_TEACH_REACH for j in range(ARM_JOINTS)}

        def move(a, label):
            if not args.no_pid:
                r = smooth_move(arm, a, gripper, cfg, limits=limits, log=print)
                if not r.reached:
                    print(f"  [!] {label} not reached (off by {r.final_error:.1f}°, "
                          f"±{args.tolerance:.0f}° wanted); moving on")
                return
            # Use the requested trajectory mode
            cmd = {"mode": args.mode, **{f"angle{j}": v for j, v in enumerate(a)}, "angle6": gripper}
            arm._send(FUNC_JOINTS, cmd)
            if not wait_reached(arm, a, args.tolerance, args.move_timeout):
                print(f"  [!] {label} not reached within {args.move_timeout:.1f}s (±{args.tolerance:.0f}°); moving on")

        def go_home():
            print("[*] home")
            if home:
                move(home, "home")
            else:
                arm.home()
                time.sleep(T_HOME)

        go_home()

        for n, (name, a) in enumerate(wps, 1):
            move(a, f"waypoint {n}")
            print(f"Waypoint {n}{f' ({name})' if name else ''} reached. Holding {args.dwell}s...")
            time.sleep(args.dwell)

        go_home()

        if args.release:
            print("[*] release")
            arm.release()
            time.sleep(T_RELEASE)
            locked = False

    finally:
        if locked:
            arm.enable(ENABLE_FULL)
            print(f"Arm locked.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\ninterrupted: arm locked where it stopped")
        sys.exit(130)
