#!/usr/bin/env python3
"""
teach_arm.py

Teach the D1 arm plant-scan poses by hand (kinesthetic teaching). The arm
goes into compliance mode (enable-all with a low stiffness, default 5000):
it holds the camera's weight but yields to a push. Guide it until the end
camera frames the plant, press Enter, and the six arm joint angles are
appended to config/arm_waypoints.json. play_arm.py replays them.

Workflow:
    1. Power on, compliance mode (funcode 5, {"mode": 5000})
    2. Enter = save the current pose (telemetry address 2, funcode 1)
    3. q (or Ctrl+C / Ctrl+D) = lock the arm (funcode 5, {"mode": 80000}), exit
The arm is always locked on exit, so it never drops. It stays locked and
powered: support it before `start_arm.sh release`, or run `start_arm.sh fold`.

Usage:
    python3 scripts/tools/teach_arm.py                 # append to the file
    python3 scripts/tools/teach_arm.py --new           # start a new sweep
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
    if not os.path.isfile(path):
        return {"waypoints": []}
    with open(path) as f:
        data = json.load(f)
    data.setdefault("waypoints", [])
    return data


def save(path, data):
    """Write atomically, so an interrupted save never leaves half a file."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = f"{path}.tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=1)
        f.write("\n")
    os.replace(tmp, path)


def out_of_limits(angles):
    return [f"joint {j} {a:+.1f}° (limit ±{JOINT_LIMITS[j]:.0f}°)"
            for j, a in enumerate(angles) if abs(a) > JOINT_LIMITS[j]]


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
        print(f"Waypoints: {args.file} ({len(data['waypoints'])} already saved)")
        while True:
            try:
                ans = input("Move the arm to the desired scan angle, then press [ENTER] "
                            "to save, or type 'q' to quit: ").strip().lower()
            except EOFError:
                print()
                break
            if ans == "q":
                break
            angles = read_joints(arm)
            if angles is None:
                print("  [!] no joint telemetry (address 2, funcode 1): not saved, try again")
                continue
            bad = out_of_limits(angles)
            if bad:
                print(f"  [!] beyond the joint limits, not saved: {', '.join(bad)}")
                continue
            data["waypoints"].append({"angles": angles,
                                      "saved": time.strftime("%Y-%m-%dT%H:%M:%S")})
            save(args.file, data)
            print(f"  saved waypoint {len(data['waypoints'])}: "
                  f"{' '.join(f'{a:+.1f}' for a in angles)}")
    finally:
        # Ctrl+C included: lock before anything else, so the camera never drops.
        arm.enable(ENABLE_FULL)
        time.sleep(T_ENABLE)
        print(f"Arm locked (mode {ENABLE_FULL}). {len(data['waypoints'])} waypoints in {args.file}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\ninterrupted: arm locked")
        sys.exit(130)
