#!/usr/bin/env python3
"""
d1_arm_calibration.py

Fold the D1 arm into its calibrated resting pose on the Go2's back.

  --read   print the current joint angles as a FOLDED_ANGLES line.
           Fold the arm by hand (motors released), run --read, and paste
           the printed line over FOLDED_ANGLES below.
  --move   enable motors, glide to FOLDED_ANGLES with one funcode=2
           multi-joint command, then release the motors.

Angles are DEGREES (joints 0-5 arm, joint 6 gripper). Run without ROS
sourced and with CYCLONEDDS_URI unset; the script binds DDS to --iface.

    python3 d1_arm_calibration.py --iface enp0s8 --read
    python3 d1_arm_calibration.py --iface enp0s8 --move
"""
import argparse, json, os, sys, time
from dataclasses import dataclass
from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.idl import IdlStruct

@dataclass
class ArmString_(IdlStruct, typename="unitree_arm::msg::dds_::ArmString_"):
    data: str = ""

# Default folded angles (User will update this via --read)
FOLDED_ANGLES = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0, 0.0]

ADDR_CMD = 1
ADDR_TELEMETRY = 2
FUNC_JOINTS = 2
FUNC_ENABLE_ALL = 5

def get_cyclonedds_uri(iface="enp0s8"):
    return f'<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any"><General><Interfaces><NetworkInterface name="{iface}" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'

class D1Arm:
    def __init__(self, iface):
        os.environ["CYCLONEDDS_URI"] = get_cyclonedds_uri(iface)
        self.p = DomainParticipant(0)
        self.cmd_writer = DataWriter(self.p, Topic(self.p, "rt/arm_Command", ArmString_))
        self.fb_reader = DataReader(self.p, Topic(self.p, "rt/arm_Feedback", ArmString_))
        self.seq = 0

    def wait_for_connection(self, timeout=6.0):
        print("[*] Connecting to DDS...")
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.cmd_writer.get_publication_matched_status().current_count > 0:
                print("[+] DDS match OK.")
                return True
            time.sleep(0.1)
        print("[!] DDS discovery failed.")
        return False

    def send(self, funcode, data=None):
        self.seq += 1
        payload = {"seq": self.seq, "address": ADDR_CMD, "funcode": funcode}
        if data is not None: payload["data"] = data
        msg = ArmString_(data=json.dumps(payload, separators=(",", ":")))
        for _ in range(3):
            self.cmd_writer.write(msg)
            time.sleep(0.1)

    def read_angles(self, timeout=3.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            for s in self.fb_reader.take(N=10):
                try:
                    m = json.loads(s.data)
                    if m.get("address") == ADDR_TELEMETRY and m.get("funcode") == 1:
                        return m.get("data", {})
                except Exception: continue
            time.sleep(0.1)
        return None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="enp0s8")
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--read", action="store_true")
    mode.add_argument("--move", action="store_true")
    args = ap.parse_args()

    arm = D1Arm(args.iface)
    if not arm.wait_for_connection(): sys.exit(1)

    if args.read:
        angles = arm.read_angles()
        if angles:
            print(f"FOLDED_ANGLES = {[angles.get(f'angle{i}', 0.0) for i in range(7)]}")
        else:
            print("[!] No joint telemetry received (address 2, funcode 1).")
            sys.exit(1)

    elif args.move:
        arm.send(FUNC_ENABLE_ALL, {"mode": 80000})
        time.sleep(1.0)
        data = {"mode": 1}
        for i, angle in enumerate(FOLDED_ANGLES):
            data[f"angle{i}"] = angle
        arm.send(FUNC_JOINTS, data)
        time.sleep(5.0)
        arm.send(FUNC_ENABLE_ALL, {"mode": 0})

if __name__ == "__main__":
    main()
