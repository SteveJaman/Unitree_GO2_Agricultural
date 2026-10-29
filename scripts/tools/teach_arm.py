#!/usr/bin/env python3
"""
teach_arm.py

Teach the D1 arm a home pose and plant-scan poses by hand (kinesthetic
teaching). The arm goes into compliance mode (enable-all with a low
stiffness, default 5000): it holds the camera's weight but yields to a push.
Joint angles come from telemetry (address 2, funcode 1) and are saved to
config/arm_waypoints.json; play_arm.py replays them.

Workflow:
    1. Power on, compliance mode (funcode 5, {"mode": 5000})
    2. HOME first: guide the arm to its rest / start pose, press Enter.
       play_arm.py starts and ends there (not at the factory zero pose).
       A file that already has a home keeps it (h re-records it, --new
       starts over).
    3. Then each scan pose: guide the arm, press Enter (or type a name, e.g.
       "left leaf", then Enter). Commands at the prompt:
         u = undo the last pose   h = re-record home   l = list   q = quit
    4. q (or Ctrl+C / Ctrl+D) = lock the arm (funcode 5, {"mode": 80000}), exit
The arm is always locked on exit, so it never drops. It stays locked and
powered: support it before `start_arm.sh release`, or run `start_arm.sh fold`.

Usage:
    python3 scripts/tools/teach_arm.py                 # add poses (home kept if saved)
    python3 scripts/tools/teach_arm.py --new           # new home + new poses
    python3 scripts/tools/teach_arm.py --stiffness 8000
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
        D1Arm, ENABLE_FULL, JOINT_LIMITS, preflight, T_ENABLE, default_arm_iface,
    )
except ImportError as e:
    sys.exit(f"cannot import d1_arm_control: {e}")

WAYPOINTS = os.path.join(_repo, "config", "arm_waypoints.json")
COMPLIANT = 5000   # stiffness: holds the camera's weight, yields to a hand
ARM_JOINTS = 6     # joints 0-5; joint 6 (gripper) is not taught


def read_joints(arm, timeout=2.0):
    """The six arm joint angles (degrees) from telemetry, or None."""
    a = arm.angles(timeout=timeout)
    d = (a or {}).get("data", {})
    vals = [d.get(f"angle{i}") for i in range(ARM_JOINTS)]
    return None if any(v is None for v in vals) else [round(float(v), 2) for v in vals]


def load(path):
    """{"home": {...} | absent, "waypoints": [{"angles", "name"?, "saved"}]}"""
    if not os.path.isfile(path):
        return {"waypoints": []}
    with open(path) as f:
        data = json.load(f)
    data.setdefault("waypoints", [])
    return data


def fmt(angles):
    return " ".join(f"{a:+7.1f}" for a in angles)


# The arm's own rest pose can read a little past a command limit (the folded
# pose has joint 2 at +91.8 deg, limit 90): up to this much is clamped, not refused.
CLAMP_DEG = 5.0


def capture(arm):
    """Current joints if telemetry answers and they are (nearly) within the limits,
    else None (says why). Readings up to CLAMP_DEG past a limit are clamped to it."""
    angles = read_joints(arm)
    if angles is None:
        print("  [!] no joint telemetry (address 2, funcode 1): not saved, try again")
        return None
    bad = [f"joint {j} {a:+.1f}° (limit ±{JOINT_LIMITS[j]:.0f}°)"
           for j, a in enumerate(angles) if abs(a) > JOINT_LIMITS[j] + CLAMP_DEG]
    if bad:
        print(f"  [!] beyond the joint limits, not saved: {', '.join(bad)}")
        return None
    clamped = []
    for j, a in enumerate(angles):
        lim = JOINT_LIMITS[j]
        if abs(a) > lim:
            angles[j] = round(max(-lim, min(lim, a)), 2)
            clamped.append(f"joint {j} {a:+.1f}° -> {angles[j]:+.1f}°")
    if clamped:
        print(f"  note: just past the limit, clamped: {', '.join(clamped)}")
    return angles


def record_home(arm, data, path):
    """Prompt until a home pose is saved; False if the user quit instead."""
    while True:
        try:
            ans = input("HOME: move the arm to its home / rest pose, then press [ENTER] "
                        "(q to quit): ").strip().lower()
        except EOFError:
            print()
            return False
        if ans == "q":
            return False
        angles = capture(arm)
        if angles is None:
            continue
        data["home"] = {"angles": angles, "saved": time.strftime("%Y-%m-%dT%H:%M:%S")}
        save(path, data)
        print(f"  saved HOME:        {fmt(angles)}")
        return True


def show(data):
    home = data.get("home")
    print(f"  HOME: {fmt(home['angles']) if home else '(not recorded)'}")
    for n, w in enumerate(data["waypoints"], 1):
        print(f"  {n:2d}: {fmt(w['angles'])}  {w.get('name', '')}")


def save(path, data):
    """Write atomically, so an interrupted save never leaves half a file."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = f"{path}.tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=1)
        f.write("\n")
    os.replace(tmp, path)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--iface", default=default_arm_iface())
    ap.add_argument("--domain", type=int, default=int(os.environ.get("ARM_DOMAIN", "0")))
    ap.add_argument("--ip", default=os.environ.get("ARM_IP", "192.168.123.100"))
    ap.add_argument("--file", default=WAYPOINTS, help="waypoint file (default: %(default)s)")
    ap.add_argument("--new", action="store_true", help="replace the file's waypoints")
    ap.add_argument("--stiffness", type=int, default=COMPLIANT,
                    help="compliance mode value (default %(default)s; 80000 = locked)")
    ap.add_argument("--skip-preflight", action="store_true")
    args = ap.parse_args()

    data = {"waypoints": []} if args.new else load(args.file)
    if not args.skip_preflight and not preflight(args.iface, args.ip):
        print("\n[!] Preflight failed")
        return 2
    arm = D1Arm(args.iface, args.domain)
    if not arm.ok:
        print("[!] DDS discovery failed")
        return 2

    try:
        arm.power(True)
        time.sleep(1.5)
        arm.enable(args.stiffness)
        time.sleep(T_ENABLE)
        print("Arm is compliant. You can now move it by hand.")
        print(f"File: {args.file}")
        if data.get("home"):
            print(f"  HOME already saved: {fmt(data['home']['angles'])}  (h at the prompt re-records it)")
        elif not record_home(arm, data, args.file):
            return 0
        print(f"Scan poses ({len(data['waypoints'])} already saved). "
              "u = undo last, h = re-record home, l = list, q = quit")
        while True:
            try:
                ans = input(f"POSE {len(data['waypoints']) + 1}: move the arm, then [ENTER] "
                            "(or type a name, then ENTER): ").strip()
            except EOFError:
                print()
                break
            cmd = ans.lower()
            if cmd == "q":
                break
            if cmd == "l":
                show(data)
                continue
            if cmd == "h":
                record_home(arm, data, args.file)
                continue
            if cmd == "u":
                if data["waypoints"]:
                    w = data["waypoints"].pop()
                    save(args.file, data)
                    print(f"  removed pose {len(data['waypoints']) + 1}: {fmt(w['angles'])}")
                else:
                    print("  no pose to undo")
                continue
            angles = capture(arm)
            if angles is None:
                continue
            wp = {"angles": angles, "saved": time.strftime("%Y-%m-%dT%H:%M:%S")}
            if ans:
                wp["name"] = ans
            data["waypoints"].append(wp)
            save(args.file, data)
            print(f"  saved pose {len(data['waypoints']):2d}:     {fmt(angles)}  {ans}")
    finally:
        # Ctrl+C included: lock before anything else, so the camera never drops.
        arm.enable(ENABLE_FULL)
        time.sleep(T_ENABLE)
        print(f"Arm locked (mode {ENABLE_FULL}). "
              f"{'HOME + ' if data.get('home') else 'no HOME, '}{len(data['waypoints'])} poses in {args.file}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\ninterrupted: arm locked")
        sys.exit(130)
