#!/usr/bin/env python3
"""
d1_arm_control.py

Control the Unitree D1-T arm via DDS based on official D1Arm services.

DDS topics:
  rt/arm_Command  — command bus (ArmString_, JSON payload)
  rt/arm_Feedback — feedback bus (ArmString_, JSON payload)

Usage:
    python3 d1_arm_control.py --iface enp0s8 home
    python3 d1_arm_control.py --iface enp0s8 enable
    python3 d1_arm_control.py --iface enp0s8 release
    python3 d1_arm_control.py --iface enp0s8 joint 0 1.57
    python3 d1_arm_control.py --iface enp0s8 joints 0 0.5 0.5 0 0 0
    python3 d1_arm_control.py --iface enp0s8 feedback
"""
import argparse
import json
import os
import sys
import time
from dataclasses import dataclass
from typing import Optional

try:
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.pub import DataWriter
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic
    from cyclonedds.idl import IdlStruct
except ImportError:
    sys.exit("cyclonedds Python bindings not installed. Install with: pip3 install cyclonedds")

# ---------------------------------------------------------------------------
# DDS type definitions matching the D1 arm's IDL
# ---------------------------------------------------------------------------

@dataclass
class ArmString_(IdlStruct, typename="unitree_arm::msg::dds_::ArmString_"):
    """D1 arm message. data holds a JSON string for both commands and feedback."""
    data: str = ""

# ---------------------------------------------------------------------------
# DDS URI construction
# ---------------------------------------------------------------------------

def build_cyclonedds_uri(iface: str, peer: Optional[str] = None) -> str:
    """Build a CycloneDDS config URI to bind to the correct network interface."""
    parts = [
        '<CycloneDDS xmlns="https://cdds.io/config" ',
        'xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" ',
        'xsi:schemaLocation="https://cdds.io/config ',
        'https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">',
        '<Domain id="any">',
        "<General>",
        "<Interfaces>",
        f'<NetworkInterface name="{iface}" multicast="default"/>',
        "</Interfaces>",
    ]
    if peer:
        parts.append("<AllowMulticast>false</AllowMulticast>")
    parts.append("</General>")
    if peer:
        parts.extend([
            "<Discovery><Peers>",
            f'<Peer address="{peer}"/>',
            "</Peers></Discovery>",
        ])
    parts.extend(["</Domain>", "</CycloneDDS>"])
    return "".join(parts)

def diagnose_interface(iface: str, verbose: bool) -> bool:
    """Check the interface exists and has an IPv4 address."""
    import subprocess
    try:
        result = subprocess.run(
            ["ip", "-brief", "addr", "show", iface],
            capture_output=True, text=True, timeout=2
        )
        if result.returncode != 0:
            print(f"[d1] WARNING: interface '{iface}' not found", file=sys.stderr)
            return False
        if verbose:
            print(f"[d1] interface {iface}: {result.stdout.strip()}")
        if "inet " not in result.stdout:
            print(f"[d1] WARNING: {iface} has no IPv4 address — DDS will fail", file=sys.stderr)
        return True
    except Exception as e:
        if verbose:
            print(f"[d1] interface check skipped: {e}")
        return True

# ---------------------------------------------------------------------------
# Arm controller
# ---------------------------------------------------------------------------

class D1Arm:
    def __init__(self, iface: str = "enp0s3", domain_id: int = 0, peer: Optional[str] = None, verbose: bool = False):
        uri = build_cyclonedds_uri(iface, peer)
        os.environ["CYCLONEDDS_URI"] = uri

        if verbose:
            print(f"[d1] CYCLONEDDS_URI = {uri}")
            print(f"[d1] DDS domain id  = {domain_id}")
        
        self.participant = DomainParticipant(domain_id)

        # Command Writer
        self.cmd_topic = Topic(self.participant, "rt/arm_Command", ArmString_)
        self.cmd_writer = DataWriter(self.participant, self.cmd_topic)

        # Feedback Reader
        self.fb_topic = Topic(self.participant, "rt/arm_Feedback", ArmString_)
        self.fb_reader = DataReader(self.participant, self.fb_topic)
        
        self._seq = 0

    def _next_seq(self) -> int:
        self._seq += 1
        return self._seq

    def _send(self, payload: dict, repeat: int = 2):
        """Publish a command. D1 recommends sending twice for reliability."""
        msg = ArmString_(data=json.dumps(payload))
        for _ in range(repeat):
            self.cmd_writer.write(msg)
            time.sleep(0.1)

    def home(self):
        self._send({"seq": self._next_seq(), "address": 0, "funcode": 7, "data": ""})

    def enable(self):
        self._send({"seq": self._next_seq(), "address": 0, "funcode": 5, "data": ""})

    def release(self):
        self._send({"seq": self._next_seq(), "address": 0, "funcode": 5, "data": "release"})

    def joint(self, joint_id: int, angle_rad: float):
        self._send({"seq": self._next_seq(), "address": joint_id, "funcode": 1, "data": str(angle_rad)})

    def joints(self, angles):
        if len(angles) not in (6, 7):
            raise ValueError(f"expected 6 or 7 joint angles, got {len(angles)}")
        
        # Formatted exactly as Unitree's arm parser expects
        data_str = "[" + ",".join(f"{a:.4f}" for a in angles) + "]"
        self._send({"seq": self._next_seq(), "address": 0, "funcode": 2, "data": data_str})

    def feedback(self, timeout: float = 2.0) -> Optional[dict]:
        """Read current telemetry from rt/arm_Feedback."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            for sample in self.fb_reader.take():
                if sample is None or not sample.data:
                    continue
                try:
                    return json.loads(sample.data)
                except json.JSONDecodeError:
                    return {"raw_data": sample.data}
            time.sleep(0.05)
        return None

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", default="enp0s3", help="network interface")
    ap.add_argument("--domain", type=int, default=0, help="DDS domain ID")
    ap.add_argument("--peer", default=None, help="unicast peer IP")
    ap.add_argument("--verbose", "-v", action="store_true", help="verbose output")
    ap.add_argument("commands", nargs="+", help="home | enable | release | joint <id> <rad> | joints <a0..a5> | feedback | sleep <s>")
    args = ap.parse_args()

    diagnose_interface(args.iface, args.verbose)

    try:
        arm = D1Arm(args.iface, args.domain, args.peer, args.verbose)
    except Exception as e:
        print(f"[d1] ERROR: failed to initialize DDS: {e}", file=sys.stderr)
        return 1

    cmds = args.commands
    i = 0
    while i < len(cmds):
        c = cmds[i]
        try:
            if c == "home":
                print("[d1] home"); arm.home(); i += 1
            elif c == "enable":
                print("[d1] enable"); arm.enable(); i += 1
            elif c == "release":
                print("[d1] release"); arm.release(); i += 1
            elif c == "joint":
                jid = int(cmds[i + 1]); ang = float(cmds[i + 2])
                print(f"[d1] joint {jid} -> {ang:.3f} rad")
                arm.joint(jid, ang); i += 3
            elif c == "joints":
                angles = [float(x) for x in cmds[i + 1:i + 8] if x.replace('.', '', 1).lstrip('-').isdigit()]
                print(f"[d1] joints -> {angles}")
                arm.joints(angles); i += 1 + len(angles)
            elif c == "feedback":
                fb = arm.feedback()
                print(f"[d1] joint feedback: {json.dumps(fb, indent=2) if fb else 'None'}"); i += 1
            elif c == "sleep":
                s = float(cmds[i + 1])
                print(f"[d1] sleep {s}s"); time.sleep(s); i += 2
            else:
                print(f"[d1] unknown command: {c}", file=sys.stderr); i += 1
        except (IndexError, ValueError) as e:
            print(f"[d1] ERROR parsing command {c!r}: {e}", file=sys.stderr)
            return 1
    return 0

if __name__ == "__main__":
    sys.exit(main())