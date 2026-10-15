#!/usr/bin/env python3
"""
d1_arm_control.py

Control the Unitree D1 arm over DDS (CycloneDDS), following the official
"D1 Mechanical Arm Services Interface" protocol.

Topics (type unitree_arm::msg::dds_::ArmString_, one JSON string):
  rt/arm_Command   host -> arm   (address 1)
  rt/arm_Feedback  arm  -> host  (address 2 = telemetry @10Hz, address 3 = acks)

ALL ANGLES ARE IN DEGREES. Joints 0-5 are the arm, joint 6 is the gripper.

Usage:
    python3 d1_arm_control.py --iface enp0s8 enable sleep 2 home sleep 6
    python3 d1_arm_control.py --iface enp0s8 joint 0 20
    python3 d1_arm_control.py --iface enp0s8 joints 0 -30 30 0 20 0 0
    python3 d1_arm_control.py --iface enp0s8 feedback
    python3 d1_arm_control.py --iface enp0s8 status
    python3 d1_arm_control.py --iface enp0s8 release

Tips:
  * Chain commands in ONE invocation: every new process has to rediscover the
    arm over DDS, and exiting right after a write can drop the message.
  * If multicast discovery fails (VMs, bridges), add  --peer 192.168.123.100
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

try:
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.pub import DataWriter
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic
    from cyclonedds.idl import IdlStruct
except ImportError:
    sys.exit("cyclonedds Python bindings not installed. Run: pip3 install cyclonedds")


# ---------------------------------------------------------------------------
# DDS type (matches unitree_arm/msg/ArmString.idl)
# ---------------------------------------------------------------------------

@dataclass
class ArmString_(IdlStruct, typename="unitree_arm::msg::dds_::ArmString_"):
    data: str = ""


# Protocol constants (from the official doc)
ADDR_CMD = 1          # command address
ADDR_TELEMETRY = 2    # arm -> host telemetry
ADDR_ACK = 3          # arm -> host acknowledgements

FUNC_JOINT = 1
FUNC_JOINTS = 2
FUNC_JOINT_ENABLE = 4
FUNC_ENABLE_ALL = 5
FUNC_POWER = 6
FUNC_ZERO = 7

# Joint limits in degrees (from the spec sheet). Joint 6 = gripper (not limited here).
JOINT_LIMITS_DEG = {0: 135.0, 1: 90.0, 2: 90.0, 3: 135.0, 4: 90.0, 5: 135.0}


# ---------------------------------------------------------------------------
# CycloneDDS config
# ---------------------------------------------------------------------------

def build_cyclonedds_uri(iface: str, peer: Optional[str] = None) -> str:
    """Bind CycloneDDS to one interface; optionally use unicast peer discovery."""
    general = f'<NetworkInterface name="{iface}" multicast="default"/>'
    allow_mc = "<AllowMulticast>false</AllowMulticast>" if peer else ""
    discovery = (
        f'<Discovery><Peers><Peer address="{peer}"/></Peers></Discovery>' if peer else ""
    )
    return (
        '<CycloneDDS xmlns="https://cdds.io/config">'
        "<Domain id=\"any\">"
        f"<General><Interfaces>{general}</Interfaces>{allow_mc}</General>"
        f"{discovery}"
        "</Domain></CycloneDDS>"
    )


def diagnose_interface(iface: str, verbose: bool) -> bool:
    """Check the interface exists and has an IPv4 address (ip -brief has no 'inet')."""
    try:
        res = subprocess.run(
            ["ip", "-4", "-o", "addr", "show", "dev", iface],
            capture_output=True, text=True, timeout=3,
        )
    except Exception as e:  # `ip` missing in container, etc.
        if verbose:
            print(f"[d1] interface check skipped: {e}")
        return True

    addrs = re.findall(r"inet (\d+\.\d+\.\d+\.\d+/\d+)", res.stdout)
    if res.returncode != 0:
        print(f"[d1] WARNING: interface '{iface}' not found", file=sys.stderr)
        return False
    if not addrs:
        print(f"[d1] WARNING: {iface} has no IPv4 address; DDS will fail", file=sys.stderr)
        return False
    if verbose:
        print(f"[d1] interface {iface}: {', '.join(addrs)}")
    return True


# ---------------------------------------------------------------------------
# Arm controller
# ---------------------------------------------------------------------------

class D1Arm:
    def __init__(self, iface: str, domain_id: int = 0, peer: Optional[str] = None,
                 verbose: bool = False, match_timeout: float = 6.0):
        self.verbose = verbose
        uri = build_cyclonedds_uri(iface, peer)
        os.environ["CYCLONEDDS_URI"] = uri
        if verbose:
            print(f"[d1] CYCLONEDDS_URI = {uri}")
            print(f"[d1] DDS domain id  = {domain_id}")

        self.participant = DomainParticipant(domain_id)
        self.cmd_topic = Topic(self.participant, "rt/arm_Command", ArmString_)
        self.cmd_writer = DataWriter(self.participant, self.cmd_topic)
        self.fb_topic = Topic(self.participant, "rt/arm_Feedback", ArmString_)
        self.fb_reader = DataReader(self.participant, self.fb_topic)

        self._seq = 0
        self.missing_acks = 0
        self.connected = self._wait_for_arm(match_timeout)

    # -- discovery ----------------------------------------------------------

    def _wait_for_arm(self, timeout: float) -> bool:
        """Block until the arm's endpoints are matched (i.e. DDS discovery worked)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                w = self.cmd_writer.get_publication_matched_status().current_count
                r = self.fb_reader.get_subscription_matched_status().current_count
            except Exception:
                # Status API unavailable: just give discovery time and carry on.
                time.sleep(min(timeout, 2.0))
                return True
            if w > 0 or r > 0:
                time.sleep(0.3)  # let the other direction finish matching
                return True
            time.sleep(0.1)
        return False

    # -- low level ----------------------------------------------------------

    def _next_seq(self) -> int:
        self._seq += 1
        return self._seq

    def _send(self, funcode: int, data: Any = None, repeat: int = 2,
              wait_ack: bool = True) -> bool:
        payload: Dict[str, Any] = {
            "seq": self._next_seq(), "address": ADDR_CMD, "funcode": funcode,
        }
        if data is not None:
            payload["data"] = data  # nested JSON object, NOT a string
        text = json.dumps(payload, separators=(",", ":"))
        if self.verbose:
            print(f"[d1] -> {text}")
        msg = ArmString_(data=text)
        for _ in range(repeat):
            self.cmd_writer.write(msg)
            time.sleep(0.1)
        if not wait_ack:
            return True
        return self._wait_ack()

    @staticmethod
    def _parse(sample) -> Optional[dict]:
        if sample is None or not getattr(sample, "data", ""):
            return None
        try:
            obj = json.loads(sample.data)
            return obj if isinstance(obj, dict) else None
        except json.JSONDecodeError:
            return None

    def _wait_ack(self, timeout: float = 2.0) -> bool:
        """Wait for the 'command received' ack (address 3, funcode 1)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            for s in self.fb_reader.take(N=20):
                m = self._parse(s)
                if not m or m.get("address") != ADDR_ACK:
                    continue
                data = m.get("data") or {}
                if m.get("funcode") == 1:
                    ok = data.get("recv_status") == 1
                    print(f"[d1]   ack: received={'OK' if ok else 'REJECTED (bad format)'}")
                    if not ok:
                        self.missing_acks += 1
                    return ok
            time.sleep(0.05)
        print("[d1]   ack: none (arm did not confirm receipt)")
        self.missing_acks += 1
        return False

    # -- commands (degrees!) ------------------------------------------------

    def enable(self, mode: int = 1):
        return self._send(FUNC_ENABLE_ALL, {"mode": mode})

    def release(self):
        return self._send(FUNC_ENABLE_ALL, {"mode": 0})

    def home(self):
        return self._send(FUNC_ZERO)  # no "data" key per spec

    def power(self, on: bool):
        return self._send(FUNC_POWER, {"power": 1 if on else 0})

    def joint(self, joint_id: int, angle_deg: float):
        if not 0 <= joint_id <= 6:
            raise ValueError(f"joint id must be 0-6, got {joint_id}")
        lim = JOINT_LIMITS_DEG.get(joint_id)
        if lim is not None and abs(angle_deg) > lim:
            raise ValueError(f"joint {joint_id} limit is +/-{lim:.0f} deg, got {angle_deg}")
        return self._send(FUNC_JOINT, {"id": joint_id, "angle": angle_deg, "delay_ms": 0})

    def joints(self, angles_deg, mode: int = 1):
        if len(angles_deg) != 7:
            raise ValueError(f"need 7 values (6 joints + gripper), got {len(angles_deg)}")
        for i, a in enumerate(angles_deg):
            lim = JOINT_LIMITS_DEG.get(i)
            if lim is not None and abs(a) > lim:
                raise ValueError(f"joint {i} limit is +/-{lim:.0f} deg, got {a}")
        data: Dict[str, Any] = {"mode": mode}
        data.update({f"angle{i}": a for i, a in enumerate(angles_deg)})
        return self._send(FUNC_JOINTS, data)

    # -- feedback -----------------------------------------------------------

    def read(self, address: int, funcode: int, timeout: float = 5.0) -> Optional[dict]:
        """Return the first feedback message matching (address, funcode)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            for s in self.fb_reader.take(N=20):
                m = self._parse(s)
                if m and m.get("address") == address and m.get("funcode") == funcode:
                    return m
            time.sleep(0.05)
        return None


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", default=os.environ.get("ARM_IFACE", "enp0s8"),
                    help="wired interface the arm is on (default $ARM_IFACE or enp0s8)")
    ap.add_argument("--domain", type=int, default=0, help="DDS domain id")
    ap.add_argument("--peer", default=os.environ.get("ARM_PEER"),
                    help="unicast peer IP (disables multicast), e.g. 192.168.123.100")
    ap.add_argument("--enable-mode", type=int, default=1,
                    help="value for enable (1 = enable; doc says up to 80000 = fully locked)")
    ap.add_argument("--verbose", "-v", action="store_true")
    ap.add_argument("commands", nargs="+",
                    help="enable | release | home | power <0|1> | joint <id> <deg> | "
                         "joints <a0..a6> | feedback | status | motors | sleep <s>")
    args = ap.parse_args()

    diagnose_interface(args.iface, args.verbose)

    try:
        arm = D1Arm(args.iface, args.domain, args.peer, args.verbose)
    except Exception as e:
        print(f"[d1] ERROR: failed to initialize DDS: {e}", file=sys.stderr)
        return 1

    if arm.connected:
        print("[d1] DDS match with arm: YES")
    else:
        print("[d1] DDS match with arm: NO")
        print("[d1] The arm was not discovered. Check: cable/interface, host IP on the arm's "
              "subnet (arm default 192.168.123.100), arm power (~1 min boot), "
              "or try --peer <arm ip>.", file=sys.stderr)
        return 2

    cmds = args.commands
    i = 0
    try:
        while i < len(cmds):
            c = cmds[i]
            if c == "enable":
                print("[d1] enable"); arm.enable(args.enable_mode); i += 1
            elif c == "release":
                print("[d1] release (arm goes limp!)"); arm.release(); i += 1
            elif c in ("home", "zero"):
                print("[d1] home"); arm.home(); i += 1
            elif c == "power":
                on = int(cmds[i + 1]) != 0
                print(f"[d1] power {'on' if on else 'off'}"); arm.power(on); i += 2
            elif c == "joint":
                jid, ang = int(cmds[i + 1]), float(cmds[i + 2])
                print(f"[d1] joint {jid} -> {ang:.2f} deg"); arm.joint(jid, ang); i += 3
            elif c == "joints":
                vals = cmds[i + 1:i + 8]
                if len(vals) != 7:
                    raise ValueError("joints needs exactly 7 values (6 joints + gripper)")
                angles = [float(x) for x in vals]
                print(f"[d1] joints (deg) -> {angles}"); arm.joints(angles); i += 8
            elif c == "feedback":
                fb = arm.read(ADDR_TELEMETRY, 1)
                print(f"[d1] joint feedback: {json.dumps(fb) if fb else 'None'}"); i += 1
            elif c == "status":
                st = arm.read(ADDR_TELEMETRY, 3)
                print(f"[d1] arm status: {json.dumps(st) if st else 'None'}"); i += 1
            elif c == "motors":
                st = arm.read(ADDR_TELEMETRY, 4)
                print(f"[d1] motor status: {json.dumps(st) if st else 'None'}"); i += 1
            elif c == "sleep":
                s = float(cmds[i + 1]); print(f"[d1] sleep {s}s"); time.sleep(s); i += 2
            else:
                print(f"[d1] unknown command: {c}", file=sys.stderr); return 1
    except (IndexError, ValueError) as e:
        print(f"[d1] ERROR in command {cmds[i]!r}: {e}", file=sys.stderr)
        return 1

    time.sleep(0.5)  # let the last writes flush before the participant is torn down
    return 3 if arm.missing_acks else 0


if __name__ == "__main__":
    sys.exit(main())