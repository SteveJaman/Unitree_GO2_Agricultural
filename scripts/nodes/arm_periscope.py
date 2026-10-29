#!/usr/bin/env python3
"""
arm_periscope.py

"Periscope" pose for the D1 arm: lift the wrist camera high and look 45 deg
down and forward, over the edge of a table the Go2's own LiDAR and front
camera cannot see onto, and hold it there while the wrist camera maps the
table top (table_search.py / the dashboard's Periscope Scan).

    1. Power on, enable (mode 80000), home.
    2. Move to PERISCOPE_POSE and hold it (the arm stays enabled and stiff).
    3. On Ctrl+C, --hold seconds, or --max-hold: home, then release (mode 0).

PERISCOPE_POSE (protocol degrees, joints 0-5) was chosen on the D1 URDF with
the IK chain of arm_ik_scanner.py: the highest camera position looking 45 deg
down with margin on every joint and clear of the Go2's body. Camera about
0.46 m above the Go2's base_link (~0.78 m above the floor standing), 0.31 m
ahead of the arm's base. The wrist camera is not mounted yet: the pose and the
hold work without it.

Like orbit_scan.py: a try/finally always homes and releases the arm; same
links (wired DDS --iface, or the Go2 bridge --robot / ARM_ROBOT).

Usage:
    python3 arm_periscope.py --dry-run                 # print the pose and its camera height
    ./scripts/runtime/start_arm.sh periscope           # preflight, confirm, hold until Ctrl+C
    ./scripts/runtime/start_arm.sh periscope --hold 10 --yes
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from d1_arm_control import (  # noqa: E402
    D1Arm,
    ENABLE_FULL,
    JOINT_LIMITS,
    T_ENABLE,
    T_HOME,
    T_RELEASE,
    default_arm_iface,
    preflight,
    print_state,
)
from orbit_scan import current, move_to  # noqa: E402

# Joints 0-5 in degrees: shoulder slightly forward, elbow up, wrist pitched
# down so the camera (along Link6 +z) looks 45 deg below the horizon.
PERISCOPE_POSE = [0.0, 10.0, -45.0, 0.0, 80.0, 0.0]


def camera_pose(pose=PERISCOPE_POSE):
    """(camera position in the Go2's base_link, degrees below the horizon), via the IK chain."""
    import numpy as np

    from arm_ik_scanner import MOUNT, load_chain

    chain = load_chain()
    fk = chain.forward_kinematics(np.radians([0.0] + list(pose) + [0.0]))
    axis = fk[:3, 2]
    return fk[:3, 3] + MOUNT, float(
        np.degrees(np.arctan2(-axis[2], np.hypot(axis[0], axis[1])))
    )


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--hold",
        type=float,
        default=0.0,
        help="s to hold the pose (0: until Ctrl+C / stop)",
    )
    ap.add_argument(
        "--max-hold", type=float, default=600.0, help="s: release after this anyway"
    )
    ap.add_argument("--iface", default=default_arm_iface())
    ap.add_argument(
        "--domain", type=int, default=int(os.environ.get("ARM_DOMAIN", "0"))
    )
    ap.add_argument("--ip", default=os.environ.get("ARM_IP", "192.168.123.100"))
    ap.add_argument(
        "--robot",
        default=os.environ.get("ARM_ROBOT"),
        help="Go2 Wi-Fi IP (arm on the Go2)",
    )
    ap.add_argument(
        "--tolerance", type=float, default=3.0, help="degrees counted as reached"
    )
    ap.add_argument("--move-timeout", type=float, default=15.0)
    ap.add_argument("--yes", action="store_true", help="do not ask before moving")
    ap.add_argument(
        "--dry-run", action="store_true", help="print the pose, do not connect"
    )
    ap.add_argument("--skip-preflight", action="store_true")
    args = ap.parse_args()

    bad = [
        f"joint {j} {v:+.1f}"
        for j, v in enumerate(PERISCOPE_POSE)
        if abs(v) > JOINT_LIMITS[j]
    ]
    if bad:
        print(f"[!] PERISCOPE_POSE beyond the joint limits: {', '.join(bad)}")
        return 4
    print(f"[periscope] pose {PERISCOPE_POSE}")
    try:
        pos, pitch = camera_pose()
        print(
            f"[periscope] camera at x {pos[0]:+.2f} y {pos[1]:+.2f} z {pos[2]:+.2f} m (base_link), "
            f"looking {pitch:.0f} deg down"
        )
    except ImportError:
        pass  # ikpy only for this printout
    if args.dry_run:
        return 0

    if args.robot:
        os.environ["ARM_ROBOT"] = args.robot
    if not args.yes:
        try:
            if (
                input(
                    "The arm will rise into the periscope pose and hold. Continue? [y/N] "
                )
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
        print("[periscope] home")
        arm.home()
        time.sleep(T_HOME)
        gripper = (current(arm) or [0.0] * 7)[6]
        if not move_to(arm, PERISCOPE_POSE, gripper, args.tolerance, args.move_timeout):
            print(
                f"  [!] pose not reached within {args.move_timeout:.0f} s; holding anyway"
            )
        limit = min(args.hold, args.max_hold) if args.hold > 0 else args.max_hold
        print(f"[periscope] holding (up to {limit:.0f} s; Ctrl+C to end)", flush=True)
        t0 = time.time()
        while time.time() - t0 < limit:
            time.sleep(0.5)
        print("[periscope] hold ended")
    finally:
        if enabled:
            try:
                print("[periscope] home")
                arm.home()
                time.sleep(T_HOME)
            except Exception as exc:
                print(f"  [!] home failed: {exc}")
            try:
                arm.release()
                time.sleep(T_RELEASE)
                print("[periscope] released (mode 0)")
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
