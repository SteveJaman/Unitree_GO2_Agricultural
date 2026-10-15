#!/usr/bin/env python3
"""
direct_test_movement.py
Standalone test sequence for a directly connected Unitree D1 arm.
"""
import argparse
import json
import os
import sys
import time
from dataclasses import dataclass

try:
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.pub import DataWriter
    from cyclonedds.topic import Topic
    from cyclonedds.idl import IdlStruct
except ImportError:
    sys.exit("CycloneDDS Python bindings not installed. Run: pip3 install cyclonedds")


@dataclass
class ArmString_(IdlStruct, typename="unitree_arm::msg::dds_::ArmString_"):
    data: str = ""


def build_cyclonedds_uri(iface: str) -> str:
    """Force CycloneDDS to bind exclusively to the direct ethernet interface."""
    return f"""<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
        <Domain id="any">
            <General>
                <Interfaces>
                    <NetworkInterface name="{iface}" multicast="default"/>
                </Interfaces>
            </General>
        </Domain>
    </CycloneDDS>"""


class DirectD1Test:
    def __init__(self, iface: str):
        os.environ["CYCLONEDDS_URI"] = build_cyclonedds_uri(iface)
        print(f"[test] Binding CycloneDDS to {iface}...")
        
        self.participant = DomainParticipant(0)
        self.cmd_topic = Topic(self.participant, "rt/arm_Command", ArmString_)
        self.cmd_writer = DataWriter(self.participant, self.cmd_topic)
        self._seq = 0
        
        # Give DDS discovery a moment to find the arm on the direct link
        time.sleep(1.0)

    def send_cmd(self, payload: dict, repeat: int = 2):
        self._seq += 1
        payload["seq"] = self._seq
        msg = ArmString_(data=json.dumps(payload))
        for _ in range(repeat):
            self.cmd_writer.write(msg)
            time.sleep(0.1)

    def run_test_sequence(self):
        print("\n[test] 1. Enabling motors...")
        self.send_cmd({"address": 0, "funcode": 5, "data": ""})
        time.sleep(2.0)

        print("[test] 2. Homing arm (returning to zero position)...")
        self.send_cmd({"address": 0, "funcode": 7, "data": ""})
        print("       Waiting 6 seconds for homing to complete...")
        time.sleep(6.0)

        print("[test] 3. Moving Joint 0 (base) to 0.5 radians...")
        # funcode 1 targets a specific joint address with a radian float
        self.send_cmd({"address": 0, "funcode": 1, "data": "0.5"})
        time.sleep(3.0)

        print("[test] 4. Releasing motors (entering drag-teach mode)...")
        self.send_cmd({"address": 0, "funcode": 5, "data": "release"})
        print("[test] Sequence complete. The arm should now be limp.")


def main():
    ap = argparse.ArgumentParser(description="Direct movement test for Unitree D1 Arm")
    ap.add_argument("--iface", required=True, help="The wired network interface (e.g., enp0s8, eth0)")
    args = ap.parse_args()

    try:
        tester = DirectD1Test(args.iface)
        tester.run_test_sequence()
    except Exception as e:
        print(f"[test] ERROR: {e}", file=sys.stderr)
        return 1
    return 0

if __name__ == "__main__":
    sys.exit(main())