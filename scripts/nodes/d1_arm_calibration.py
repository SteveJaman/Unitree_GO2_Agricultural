#!/usr/bin/env python3
"""
d1_arm_calibration.py

Fold the D1 arm into its calibrated resting pose on the Go2's back.
Uses the shared D1Arm of d1_arm_control.py: DDS on --iface, or with
ARM_ROBOT=<Go2 Wi-Fi IP> the arm plugged into the Go2, through its bridge.

    python3 d1_arm_calibration.py --iface enp0s8 --read
    python3 d1_arm_calibration.py --iface enp0s8 --move
    ARM_ROBOT=192.168.1.9 python3 d1_arm_calibration.py --move
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from d1_arm_control import ADDR_ACK, D1Arm as _SharedArm  # noqa: E402


# Paste the FOLDED_ANGLES line printed by --read below:
FOLDED_ANGLES = [1.8, -88.8, 91.8, 1.3, 23.0, 0.6, -29.5]

# Protocol Constants
FUNC_JOINTS = 2
FUNC_ENABLE_ALL = 5
FUNC_POWER = 6
FUNC_ZERO = 7


class D1Arm(_SharedArm):
    """The shared D1Arm (d1_arm_control.py) with this script's helpers. The
    link is DDS on --iface, or the Go2 Wi-Fi bridge when ARM_ROBOT is set."""

    def __init__(self, iface):
        super().__init__(iface, match_timeout=10.0, retries=2)
        self.missing_acks = 0

    def wait_for_connection(self, timeout=5.0):
        return self.ok

    def send(self, funcode, data=None, repeat=3, wait_ack=True):
        self._send(funcode, data, repeat)
        if not wait_ack:
            return True
        ack = self._latest(ADDR_ACK, 1, timeout=2.0)
        if ack and (ack.get("data") or {}).get("recv_status") == 1:
            return True
        self.missing_acks += 1
        return False

    def read_angles(self, timeout=2.0):
        m = self.angles(timeout)
        return m.get("data") if m else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="enp0s8")
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--read", action="store_true")
    mode.add_argument("--move", action="store_true")
    args = ap.parse_args()

    arm = D1Arm(args.iface)
    if not arm.wait_for_connection():
        print("[!] Failed to connect to DDS.")
        sys.exit(1)

    if args.read:
        angles = arm.read_angles()
        if angles:
            angle_list = [round(angles.get(f"angle{i}", 0.0), 2) for i in range(7)]
            print(f"FOLDED_ANGLES = {angle_list}")
        else:
            print("[!] No joint telemetry received (address 2, funcode 1).")
            sys.exit(1)

    elif args.move:
        # 1. Power ON (funcode 6)
        print("[*] Turning motor power ON (funcode=6)...")
        arm.send(FUNC_POWER, {"power": 1})
        time.sleep(1.5)

        # 2. Enable Motors (funcode 5)
        print("[*] Enabling motors (mode 80000)...")
        arm.send(FUNC_ENABLE_ALL, {"mode": 80000})
        time.sleep(1.0)

        # 3. Move (funcode 2)
        print(f"[*] Moving to FOLDED_ANGLES: {FOLDED_ANGLES}")
        data = {"mode": 1}  # mode 1 = large smoothing trajectory
        for i, angle in enumerate(FOLDED_ANGLES):
            data[f"angle{i}"] = angle

        arm.send(FUNC_JOINTS, data)

        print("[*] Waiting 5s for motion to complete...")
        time.sleep(5.0)

        # 4. Release Motors (funcode 5)
        print("[*] Releasing motors (mode 0)...")
        arm.send(FUNC_ENABLE_ALL, {"mode": 0})
        time.sleep(1.0)

        # 5. Power OFF (funcode 6)
        print("[*] Turning motor power OFF (funcode=6)...")
        arm.send(FUNC_POWER, {"power": 0})

        print("[+] Done.")


if __name__ == "__main__":
    main()
