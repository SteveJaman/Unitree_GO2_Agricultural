#!/usr/bin/env python3
"""
d1_arm_calibration.py

Fold the D1 arm into its calibrated resting pose on the Go2's back.
This uses the exact same robust ACK-waiting DDS logic as d1_arm_control.py.

    python3 d1_arm_calibration.py --iface enp0s8 --read
    python3 d1_arm_calibration.py --iface enp0s8 --move
"""

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass
from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.idl import IdlStruct


@dataclass
class ArmString_(IdlStruct, typename="unitree_arm::msg::dds_::ArmString_"):
    data: str = ""


# Paste the FOLDED_ANGLES line printed by --read below:
FOLDED_ANGLES = [1.8, -88.8, 91.8, 1.3, 23.0, 0.6, -29.5]

ADDR_CMD = 1
ADDR_TELEMETRY = 2
ADDR_ACK = 3

# Protocol Constants
FUNC_JOINTS = 2
FUNC_ENABLE_ALL = 5
FUNC_POWER = 6
FUNC_ZERO = 7


def get_cyclonedds_uri(iface="enp0s8"):
    return f'<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any"><General><Interfaces><NetworkInterface name="{iface}" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'


class D1Arm:
    def __init__(self, iface):
        os.environ["CYCLONEDDS_URI"] = get_cyclonedds_uri(iface)
        self.p = DomainParticipant(0)
        self.cmd_writer = DataWriter(
            self.p, Topic(self.p, "rt/arm_Command", ArmString_)
        )
        self.fb_reader = DataReader(
            self.p, Topic(self.p, "rt/arm_Feedback", ArmString_)
        )
        self.seq = 0
        self.missing_acks = 0

    def wait_for_connection(self, timeout=5.0):
        """Block until the arm's endpoints are matched, or fallback to a 2s wait."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                w = self.cmd_writer.get_publication_matched_status().current_count
                r = self.fb_reader.get_subscription_matched_status().current_count
            except AttributeError:
                # Older cyclonedds API fallback
                time.sleep(2.0)
                return True
            if w > 0 or r > 0:
                time.sleep(0.3)
                return True
            time.sleep(0.1)
        return False

    def _wait_ack(self, timeout=2.0):
        """Wait for the 'command received' ack (address 3, funcode 1)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            for s in self.fb_reader.take(N=20):
                try:
                    m = json.loads(s.data)
                except Exception:
                    continue
                if m.get("address") == ADDR_ACK and m.get("funcode") == 1:
                    if m.get("data", {}).get("recv_status") == 1:
                        return True
            time.sleep(0.05)
        self.missing_acks += 1
        return False

    def send(self, funcode, data=None, repeat=3, wait_ack=True):
        self.seq += 1
        payload = {"seq": self.seq, "address": ADDR_CMD, "funcode": funcode}
        if data is not None:
            payload["data"] = data
        text = json.dumps(payload, separators=(",", ":"))
        msg = ArmString_(data=text)

        for _ in range(repeat):
            self.cmd_writer.write(msg)
            time.sleep(0.1)

        if wait_ack:
            return self._wait_ack()
        return True

    def read_angles(self, timeout=2.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            for s in self.fb_reader.take(N=20):
                try:
                    m = json.loads(s.data)
                except Exception:
                    continue
                if m.get("address") == ADDR_TELEMETRY and m.get("funcode") == 1:
                    return m.get("data")
            time.sleep(0.05)
        return None


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
