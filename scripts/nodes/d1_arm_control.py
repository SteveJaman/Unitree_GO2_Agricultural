#!/usr/bin/env python3
"""
d1_arm_control.py

Direct control for the Unitree D1-T arm via the Unitree SDK2 DDS interface.
Sends JSON commands over the rt/arm_Command topic and reads joint feedback
from current_servo_angle.

Usage:
    python3 d1_arm_control.py --iface eth0 home
    python3 d1_arm_control.py --iface eth0 enable
    python3 d1_arm_control.py --iface eth0 release
    python3 d1_arm_control.py --iface eth0 joint 0 1.57
    python3 d1_arm_control.py --iface eth0 joints 0.0 0.5 0.5 0.0 0.0 0.0
    python3 d1_arm_control.py --iface eth0 feedback
    python3 d1_arm_control.py --iface eth0 sequence home enable joints 0 0.3 0.6 0 0.5 0

Requires: unitree_sdk2_python, cyclonedds
"""
import argparse
import json
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_arm.msg.dds_ import ArmString_
from unitree_sdk2py.idl.unitree_arm.msg.dds_ import PubServoInfo_

CMD_TOPIC = "rt/arm_Command"
FB_TOPIC = "current_servo_angle"


class D1Arm:
    def __init__(self, iface="eth0"):
        ChannelFactoryInitialize(0, iface)
        self._pub = ChannelPublisher(CMD_TOPIC, ArmString_)
        self._pub.Init()

        self._feedback = None
        self._sub = ChannelSubscriber(FB_TOPIC, PubServoInfo_)
        self._sub.Init(self._on_feedback, 10)

    def _on_feedback(self, msg):
        self._feedback = list(msg.servo_angle)

    def _send(self, payload: dict, repeat=2):
        """Send a command. D1 recommends sending twice for reliability."""
        msg = ArmString_()
        msg.data = json.dumps(payload)
        for _ in range(repeat):
            self._pub.Write(msg)
            time.sleep(0.5)  # settle between repeats

    def home(self):
        """Return all joints to zero."""
        self._send({"funcode": 7, "address": 0, "seq": 0, "data": ""})

    def enable(self):
        """Power on motors (torque enabled)."""
        self._send({"funcode": 5, "address": 0, "seq": 0, "data": ""})

    def release(self):
        """Power off motors (gravity compensation, drag-teach mode)."""
        self._send({"funcode": 5, "address": 0, "seq": 0, "data": "release"})

    def joint(self, joint_id: int, angle_rad: float):
        """Move a single joint."""
        self._send({
            "funcode": 1,
            "address": joint_id,
            "seq": 0,
            "data": str(angle_rad),
        })

    def joints(self, angles):
        """Move all 6 joints simultaneously."""
        if len(angles) != 6:
            raise ValueError("expected 6 joint angles")
        data = "[" + ",".join(f"{a:.4f}" for a in angles) + "]"
        self._send({"funcode": 2, "address": 0, "seq": 0, "data": data})

    def feedback(self, timeout=2.0):
        """Read current joint angles."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self._feedback is not None:
                return self._feedback
            time.sleep(0.05)
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--iface", default="eth0",
                    help="network interface to the arm (e.g. eth0)")
    ap.add_argument("commands", nargs="+", help="commands to execute")
    args = ap.parse_args()

    # Commands are a mix of keywords and args. Parse sequentially.
    arm = D1Arm(args.iface)
    print(f"connected on {args.iface}")

    cmds = args.commands
    i = 0
    while i < len(cmds):
        c = cmds[i]
        if c == "home":
            print("home"); arm.home(); i += 1
        elif c == "enable":
            print("enable"); arm.enable(); i += 1
        elif c == "release":
            print("release"); arm.release(); i += 1
        elif c == "joint":
            jid = int(cmds[i + 1]); ang = float(cmds[i + 2])
            print(f"joint {jid} -> {ang:.3f} rad")
            arm.joint(jid, ang); i += 3
        elif c == "joints":
            angles = [float(x) for x in cmds[i + 1:i + 7]]
            print(f"joints -> {angles}")
            arm.joints(angles); i += 7
        elif c == "feedback":
            fb = arm.feedback()
            print(f"joint feedback: {fb}")
            i += 1
        elif c == "sleep":
            s = float(cmds[i + 1]); time.sleep(s); i += 2
        else:
            print(f"unknown command: {c}"); i += 1

    print("done")


if __name__ == "__main__":
    main()