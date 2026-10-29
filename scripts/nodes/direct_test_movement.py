#!/usr/bin/env python3
"""
direct_test_movement.py

Small, safe movement test for a Unitree D1 arm. Needs d1_arm_control.py in the
same folder. Angles are DEGREES.

    python3 direct_test_movement.py                  # wired iface auto-detected
    python3 direct_test_movement.py --iface enp0s31f6 --peer 192.168.123.100
    python3 direct_test_movement.py --release   # go limp at the end
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from d1_arm_control import (  # noqa: E402
    D1Arm,
    ADDR_TELEMETRY,
    default_arm_iface,
    diagnose_interface,
)


def main() -> int:
    ap = argparse.ArgumentParser(description="Direct movement test for Unitree D1 arm")
    ap.add_argument(
        "--iface",
        default=default_arm_iface(),
        help="wired interface (default: auto-detected, e.g. enp0s31f6)",
    )
    ap.add_argument("--peer", default=None, help="unicast arm IP, e.g. 192.168.123.100")
    ap.add_argument(
        "--angle", type=float, default=20.0, help="test angle for joint 0 (deg)"
    )
    ap.add_argument(
        "--release",
        action="store_true",
        help="release motors at the end (arm will go limp)",
    )
    ap.add_argument("--verbose", "-v", action="store_true")
    args = ap.parse_args()

    diagnose_interface(args.iface, args.verbose)
    arm = D1Arm(args.iface, peer=args.peer, verbose=args.verbose)
    if not arm.connected:
        print("[test] DDS match with arm: NO. Arm not discovered.", file=sys.stderr)
        return 2
    print("[test] DDS match with arm: YES")

    fb = arm.read(ADDR_TELEMETRY, 1, timeout=3.0)
    print(f"[test] initial joint angles: {fb['data'] if fb else 'no feedback'}")

    print("[test] 1. enable motors")
    arm.enable()
    time.sleep(2.0)
    print("[test] 2. home (go to zero)")
    arm.home()
    time.sleep(6.0)
    print(f"[test] 3. joint 0 -> {args.angle} deg")
    arm.joint(0, args.angle)
    time.sleep(3.0)
    print("[test] 4. joint 0 -> 0 deg")
    arm.joint(0, 0.0)
    time.sleep(3.0)

    fb = arm.read(ADDR_TELEMETRY, 1, timeout=3.0)
    print(f"[test] final joint angles: {fb['data'] if fb else 'no feedback'}")

    if args.release:
        print("[test] 5. release motors (arm goes limp)")
        arm.release()
    print("[test] done")
    return 3 if arm.missing_acks else 0


if __name__ == "__main__":
    sys.exit(main())
