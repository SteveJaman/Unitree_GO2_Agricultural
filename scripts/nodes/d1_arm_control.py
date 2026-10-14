#!/usr/bin/env python3
"""
d1_arm_control.py

Control the Unitree D1-T arm via DDS without relying on
unitree_sdk2py.idl.unitree_arm (which doesn't exist in the public Python SDK).

Defines the missing DDS types manually using cyclonedds.idl and talks to the
arm's DDS topics directly.

DDS topics:
  rt/arm_Command        — command bus (ArmString_, JSON payload)
  current_servo_angle   — joint telemetry (PubServoInfo_, float array)

Requires:
    pip install cyclonedds

Usage:
    python3 d1_arm_control.py --iface enp0s8 home
    python3 d1_arm_control.py --iface enp0s8 enable
    python3 d1_arm_control.py --iface enp0s8 release
    python3 d1_arm_control.py --iface enp0s8 joint 0 1.57
    python3 d1_arm_control.py --iface enp0s8 joints 0 0.5 0.5 0 0 0
    python3 d1_arm_control.py --iface enp0s8 feedback
    python3 d1_arm_control.py --iface enp0s8 sequence enable sleep 1 home

    # Debug output showing the exact DDS config
    python3 d1_arm_control.py --iface enp0s8 --verbose feedback

    # Fallback if multicast is blocked: add a unicast peer
    python3 d1_arm_control.py --iface enp0s8 --peer 192.168.123.100 feedback
"""
import argparse
import json
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Optional, List

# ---------------------------------------------------------------------------
# DDS type definitions matching the D1 arm's IDL
# ---------------------------------------------------------------------------
try:
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.pub import DataWriter
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic
    from cyclonedds.idl import IdlStruct
    from cyclonedds.idl.types import sequence, float32
    from cyclonedds.core import SampleInfo
except ImportError:
    sys.exit(
        "cyclonedds Python bindings not installed.\n"
        "Install with:\n"
        "    pip3 install cyclonedds\n"
    )


@dataclass
class ArmString_(IdlStruct, typename="unitree_arm::msg::dds_::ArmString_"):
    """D1 arm command message. The data field holds a JSON string.

    Command JSON format:
      {"seq": N, "address": A, "funcode": F, "data": ...}

    funcode values:
      1  — single joint move
      2  — multi joint move
      5  — enable / release motors
      7  — return to zero (home)
    """
    data: str = ""


@dataclass
class PubServoInfo_(IdlStruct, typename="unitree_arm::msg::dds_::PubServoInfo_"):
    """D1 arm joint telemetry.

    Carries an array of float joint angles. Field name is best-effort based
    on the C++ reference; adjust if the arm's IDL differs.
    """
    servo_angle: sequence[float32] = field(default_factory=list)


# ---------------------------------------------------------------------------
# DDS URI construction
# ---------------------------------------------------------------------------

def build_cyclonedds_uri(iface: str, peer: Optional[str] = None) -> str:
    """Build a CycloneDDS config URI.

    Binds discovery and data traffic to a single interface. Optionally adds
    a unicast peer address for cases where multicast is unavailable.
    """
    parts = [
        "<CycloneDDS><Domain><General>",
        "<Interfaces>",
        f"<NetworkInterface name='{iface}'/>",
        "</Interfaces>",
    ]
    if peer:
        parts.append("<AllowMulticast>false</AllowMulticast>")
    parts.append("</General>")
    if peer:
        parts.extend([
            "<Discovery><Peers>",
            f"<Peer address='{peer}'/>",
            "</Peers></Discovery>",
        ])
    parts.append("</Domain></CycloneDDS>")
    return "".join(parts)


def diagnose_interface(iface: str, verbose: bool) -> bool:
    """Check the interface exists. List alternatives if not."""
    import subprocess
    try:
        result = subprocess.run(
            ["ip", "-brief", "addr", "show", iface],
            capture_output=True, text=True, timeout=2
        )
        if result.returncode != 0:
            print(f"[d1] WARNING: interface '{iface}' not found", file=sys.stderr)
            print(f"[d1] available interfaces:", file=sys.stderr)
            all_ifaces = subprocess.run(
                ["ip", "-brief", "link"], capture_output=True, text=True
            ).stdout
            for line in all_ifaces.strip().splitlines():
                print(f"       {line}", file=sys.stderr)
            return False
        if verbose:
            print(f"[d1] interface {iface}: {result.stdout.strip()}")
        return True
    except Exception as e:
        if verbose:
            print(f"[d1] interface check skipped: {e}")
        return True


# ---------------------------------------------------------------------------
# Arm controller
# ---------------------------------------------------------------------------

class D1Arm:
    def __init__(self, iface: str = "enp0s8", domain_id: int = 0,
                 peer: Optional[str] = None, verbose: bool = False):

        uri = build_cyclonedds_uri(iface, peer)
        os.environ["CYCLONEDDS_URI"] = uri

        if verbose:
            print(f"[d1] CYCLONEDDS_URI = {uri}")
            print(f"[d1] DDS domain id  = {domain_id}")

        self.participant = DomainParticipant(domain_id)

        self.cmd_topic = Topic(self.participant, "rt/arm_Command", ArmString_)
        self.cmd_writer = DataWriter(self.participant, self.cmd_topic)

        self.fb_topic = Topic(self.participant, "current_servo_angle", PubServoInfo_)
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
            time.sleep(0.5)

    def home(self):
        self._send({"seq": self._next_seq(), "address": 0,
                    "funcode": 7, "data": ""})

    def enable(self):
        self._send({"seq": self._next_seq(), "address": 0,
                    "funcode": 5, "data": ""})

    def release(self):
        self._send({"seq": self._next_seq(), "address": 0,
                    "funcode": 5, "data": "release"})

    def joint(self, joint_id: int, angle_rad: float):
        self._send({"seq": self._next_seq(), "address": joint_id,
                    "funcode": 1, "data": str(angle_rad)})

    def joints(self, angles):
        if len(angles) != 6:
            raise ValueError("expected 6 joint angles")
        data = "[" + ",".join(f"{a:.4f}" for a in angles) + "]"
        self._send({"seq": self._next_seq(), "address": 0,
                    "funcode": 2, "data": data})

    def feedback(self, timeout: float = 2.0) -> Optional[List[float]]:
        """Read current joint angles. Returns None if no message arrives."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            for sample in self.fb_reader.take():
                if isinstance(sample, SampleInfo):
                    continue
                return list(sample.servo_angle)
            time.sleep(0.05)
        return None


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--iface", default="enp0s8",
                    help="network interface the arm is reachable on "
                         "(default: enp0s8)")
    ap.add_argument("--domain", type=int, default=0,
                    help="DDS domain ID (default: 0)")
    ap.add_argument("--peer", default=None,
                    help="unicast peer IP for discovery when multicast is "
                         "blocked (e.g. 192.168.123.100)")
    ap.add_argument("--verbose", "-v", action="store_true",
                    help="print DDS config and interface details")
    ap.add_argument("commands", nargs="+",
                    help="home | enable | release | joint <id> <rad> | "
                         "joints <a0..a5> | feedback | sleep <s>")
    args = ap.parse_args()

    print(f"[d1] initializing on interface {args.iface}, "
          f"domain {args.domain}")

    diagnose_interface(args.iface, args.verbose)

    try:
        arm = D1Arm(args.iface, args.domain, args.peer, args.verbose)
    except Exception as e:
        print(f"[d1] ERROR: failed to initialize DDS: {e}", file=sys.stderr)
        return 1

    print("[d1] DDS participant created; command topic rt/arm_Command")

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
                angles = [float(x) for x in cmds[i + 1:i + 7]]
                print(f"[d1] joints -> {angles}")
                arm.joints(angles); i += 7
            elif c == "feedback":
                fb = arm.feedback()
                print(f"[d1] joint feedback: {fb}"); i += 1
            elif c == "sleep":
                s = float(cmds[i + 1])
                print(f"[d1] sleep {s}s"); time.sleep(s); i += 2
            else:
                print(f"[d1] unknown command: {c}", file=sys.stderr); i += 1
        except (IndexError, ValueError) as e:
            print(f"[d1] ERROR parsing command {c!r}: {e}", file=sys.stderr)
            return 1

    print("[d1] done")
    return 0


if __name__ == "__main__":
    sys.exit(main())