#!/usr/bin/env python3
"""
d1_arm_control.py

Control the Unitree D1 arm over DDS (CycloneDDS), following the official
"D1 Mechanical Arm Services Interface" protocol.

Topics (type unitree_arm::msg::dds_::ArmString_, one JSON string):
  rt/arm_Command   host -> arm   (address 1)
  rt/arm_Feedback  arm  -> host  (address 2 = telemetry @10Hz, address 3 = acks)

ALL ANGLES ARE IN DEGREES. Joints 0-5 are the arm, joint 6 is the gripper
(the gripper command is a servo angle in degrees, not millimetres or metres).

Enable uses mode 80000 (fully locked) by default. The official doc table says
"1 = enable" but also "0..80000, 80000 = fully locked", and the working D1-550
ros2_control driver (SofiaAbal/tfg_go2_arm) sends 80000, so that is the default.

Usage:
    python3 d1_arm_control.py --iface enp0s8 enable sleep 2 home sleep 6
    python3 d1_arm_control.py --iface enp0s8 joint 0 20
    python3 d1_arm_control.py --iface enp0s8 joints 0 -30 30 0 20 0 0
    python3 d1_arm_control.py --iface enp0s8 enable glide 0 -30 30 0 20 0 0
    python3 d1_arm_control.py --iface enp0s8 gripper 60
    python3 d1_arm_control.py --iface enp0s8 angles
    python3 d1_arm_control.py --iface enp0s8 run scan.txt
    python3 d1_arm_control.py --iface enp0s8 feedback
    python3 d1_arm_control.py --iface enp0s8 status
    python3 d1_arm_control.py --iface enp0s8 release

Smooth motion (glide):
  `joints` sends one target and lets the arm's controller choose the motion.
  `glide` starts from the MEASURED position (averaged feedback), streams a
  quintic trajectory (zero velocity/acceleration at both ends) with peak
  joint speed --vmax deg/s, then waits until every joint is within --tolerance. It
  refuses to move if no joint feedback arrives: planning from a guessed start
  would make the first setpoint a jump.

Sequence files (run <file>):
  One command per line, same syntax as the CLI; '#' starts a comment.
      enable
      glide 0 -30 30 0 20 0 0
      gripper 60 2000
      sleep 2

Tips:
  * Chain commands in ONE invocation: every new process has to rediscover the
    arm over DDS, and exiting right after a write can drop the message.
  * If multicast discovery fails (VMs, bridges), add  --peer 192.168.123.100
"""
import argparse
import json
import os
import re
import shlex
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

try:
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.pub import DataWriter
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic
    from cyclonedds.idl import IdlStruct
    from cyclonedds.idl import types as idl_types
    from cyclonedds.qos import Policy, Qos
except ImportError:
    sys.exit("cyclonedds Python bindings not installed. Run: pip3 install cyclonedds")


# ---------------------------------------------------------------------------
# DDS type (matches unitree_arm/msg/ArmString.idl)
# ---------------------------------------------------------------------------

@dataclass
class ArmString_(IdlStruct, typename="unitree_arm::msg::dds_::ArmString_"):
    data: str = ""


# Second joint-angle source (matches unitree_arm/msg/PubServoInfo.idl), published
# on "current_servo_angle". Used when rt/arm_Feedback telemetry is missing.
@dataclass
class PubServoInfo_(IdlStruct, typename="unitree_arm::msg::dds_::PubServoInfo_"):
    servo0_data: idl_types.float32 = 0.0
    servo1_data: idl_types.float32 = 0.0
    servo2_data: idl_types.float32 = 0.0
    servo3_data: idl_types.float32 = 0.0
    servo4_data: idl_types.float32 = 0.0
    servo5_data: idl_types.float32 = 0.0
    servo6_data: idl_types.float32 = 0.0


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

ENABLE_FULL = 80000   # "fully locked" per the official doc; used by working drivers

# Joint limits in degrees (from the spec sheet, matches the D1-550 URDF).
# Joint 6 = gripper (not limited here).
JOINT_LIMITS_DEG = {0: 135.0, 1: 90.0, 2: 90.0, 3: 135.0, 4: 90.0, 5: 135.0}

# Glide defaults. The URDF allows 60 deg/s (joints 0-2) and 99 deg/s (3-5).
# GLIDE_VMAX is the PEAK joint speed. A quintic profile peaks at 1.875x its
# average speed, so 25 deg/s peak ~= the 15 deg/s average reported to avoid
# vibration on hardware at a 50 Hz stream rate.
GLIDE_VMAX_DEG_S = 25.0
QUINTIC_PEAK_RATIO = 1.875
GLIDE_RATE_HZ = 50.0
GLIDE_MIN_DURATION_S = 1.0
GLIDE_TOLERANCE_DEG = 5.0


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

        # Best-effort reader matches both reliable and best-effort writers.
        try:
            self.servo_topic = Topic(self.participant, "current_servo_angle", PubServoInfo_)
            self.servo_reader = DataReader(self.participant, self.servo_topic,
                                           qos=Qos(Policy.Reliability.BestEffort))
        except Exception as e:
            if verbose:
                print(f"[d1] current_servo_angle reader unavailable: {e}")
            self.servo_reader = None

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

    def enable(self, mode: int = ENABLE_FULL):
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

    @staticmethod
    def _check_limits(angles_deg):
        if len(angles_deg) != 7:
            raise ValueError(f"need 7 values (6 joints + gripper), got {len(angles_deg)}")
        for i, a in enumerate(angles_deg):
            lim = JOINT_LIMITS_DEG.get(i)
            if lim is not None and abs(a) > lim:
                raise ValueError(f"joint {i} limit is +/-{lim:.0f} deg, got {a}")

    @staticmethod
    def _joints_data(angles_deg, mode: int = 1) -> Dict[str, Any]:
        data: Dict[str, Any] = {"mode": mode}
        data.update({f"angle{i}": round(float(a), 3) for i, a in enumerate(angles_deg)})
        return data

    def joints(self, angles_deg, mode: int = 1):
        self._check_limits(angles_deg)
        return self._send(FUNC_JOINTS, self._joints_data(angles_deg, mode))

    def gripper(self, angle_deg: float, delay_ms: int = 0):
        """Gripper is joint 6; the value is a servo angle in degrees (0 = closed)."""
        return self._send(FUNC_JOINT, {"id": 6, "angle": angle_deg, "delay_ms": int(delay_ms)})

    def glide(self, target_deg, vmax_deg_s: float = GLIDE_VMAX_DEG_S,
              rate_hz: float = GLIDE_RATE_HZ, tolerance_deg: float = GLIDE_TOLERANCE_DEG,
              settle_timeout: float = 10.0) -> bool:
        """Smooth move from the measured position to target (7 values, degrees).

        Streams funcode-2 setpoints along a quintic time-scaling
        q(s) = q0 + (q1 - q0) * (10 s^3 - 15 s^4 + 6 s^5), s = t / T, which has
        zero velocity and acceleration at both ends. Its peak speed is 1.875x
        the average, so T = 1.875 * (largest joint travel) / vmax keeps every
        joint at or below vmax_deg_s.
        """
        self._check_limits(target_deg)
        if vmax_deg_s <= 0 or rate_hz <= 0:
            raise ValueError("vmax and rate must be > 0")

        start = self.joint_angles()
        if start is None:
            print("[d1]   glide REFUSED: no joint feedback, so the start position is unknown")
            return False

        travel = max(abs(t - s) for s, t in zip(start[:6], target_deg[:6]))
        duration = max(GLIDE_MIN_DURATION_S, QUINTIC_PEAK_RATIO * travel / vmax_deg_s)
        dt = 1.0 / rate_hz
        steps = max(1, int(round(duration / dt)))
        print(f"[d1]   from {[round(a, 1) for a in start]}")
        print(f"[d1]   {travel:.1f} deg max travel, {duration:.1f} s, peak <= {vmax_deg_s:g} deg/s, "
              f"{rate_hz:g} Hz")

        # Setpoints are streamed without waiting for acks: at 50 Hz an ack
        # wait per message would stall the trajectory.
        t0 = time.monotonic()
        for k in range(1, steps + 1):
            s = k / steps
            blend = s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)
            setpoint = [q0 + (q1 - q0) * blend for q0, q1 in zip(start, target_deg)]
            payload = {"seq": self._next_seq(), "address": ADDR_CMD, "funcode": FUNC_JOINTS,
                       "data": self._joints_data(setpoint)}
            self.cmd_writer.write(ArmString_(data=json.dumps(payload, separators=(",", ":"))))
            sleep_for = t0 + k * dt - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)

        return self.wait_for_position(target_deg, tolerance_deg, settle_timeout)

    def wait_for_position(self, target_deg, tolerance_deg: float = GLIDE_TOLERANCE_DEG,
                          timeout: float = 10.0) -> bool:
        """Block until joints 0-5 are within tolerance of target (gripper ignored)."""
        t0 = time.time()
        last_print = 0.0
        worst = None
        while time.time() - t0 < timeout:
            cur = self.joint_angles(samples=1, timeout=0.5)
            if cur is not None:
                worst = max(abs(c - t) for c, t in zip(cur[:6], target_deg[:6]))
                if worst <= tolerance_deg:
                    print(f"[d1]   reached (max error {worst:.1f} deg)")
                    return True
                if time.time() - last_print >= 1.0:
                    print(f"[d1]   settling... max error {worst:.1f} deg")
                    last_print = time.time()
        msg = f"max error {worst:.1f} deg" if worst is not None else "no feedback"
        print(f"[d1]   WARNING: target not reached within {timeout:.0f} s ({msg})")
        return False

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

    def _sample_angles(self) -> Optional[List[float]]:
        """Newest joint-angle sample from either feedback source, or None."""
        latest = None
        for s in self.fb_reader.take(N=20):
            m = self._parse(s)
            if m and m.get("address") == ADDR_TELEMETRY and m.get("funcode") == 1:
                d = m.get("data") or {}
                if all(f"angle{i}" in d for i in range(7)):
                    latest = [float(d[f"angle{i}"]) for i in range(7)]
        if latest is None and self.servo_reader is not None:
            for s in self.servo_reader.take(N=20):
                if s is not None and hasattr(s, "servo0_data"):
                    latest = [float(getattr(s, f"servo{i}_data")) for i in range(7)]
        return latest

    def joint_angles(self, samples: int = 10, timeout: float = 3.0) -> Optional[List[float]]:
        """Average `samples` readings (7 values, degrees). None if no feedback.

        Averaging smooths sensor jitter before planning a move from this pose.
        """
        got: List[List[float]] = []
        t0 = time.time()
        while len(got) < samples and time.time() - t0 < timeout:
            a = self._sample_angles()
            if a is not None:
                got.append(a)
            else:
                time.sleep(0.05)
        if not got:
            return None
        return [sum(col) / len(got) for col in zip(*got)]


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def run_commands(arm: "D1Arm", cmds: List[str], args, depth: int = 0) -> int:
    """Execute CLI-style tokens. Returns 0, or 1 on a bad command."""
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
            elif c in ("joints", "glide"):
                vals = cmds[i + 1:i + 8]
                if len(vals) != 7:
                    raise ValueError(f"{c} needs exactly 7 values (6 joints + gripper)")
                angles = [float(x) for x in vals]
                if c == "joints":
                    print(f"[d1] joints (deg) -> {angles}"); arm.joints(angles)
                else:
                    print(f"[d1] glide (deg) -> {angles}")
                    if not arm.glide(angles, args.vmax, args.rate, args.tolerance):
                        arm.missing_acks += 1
                i += 8
            elif c == "gripper":
                ang = float(cmds[i + 1])
                # optional delay_ms if the next token is a number
                delay = 0
                if i + 2 < len(cmds) and re.fullmatch(r"\d+", cmds[i + 2]):
                    delay = int(cmds[i + 2]); i += 1
                print(f"[d1] gripper -> {ang:.1f} deg (delay {delay} ms)"); arm.gripper(ang, delay); i += 2
            elif c == "angles":
                a = arm.joint_angles()
                if a is None:
                    print("[d1] angles: no joint feedback"); arm.missing_acks += 1
                else:
                    print(f"[d1] angles (deg, 10-sample avg): {[round(x, 1) for x in a]}")
                i += 1
            elif c == "run":
                path = cmds[i + 1]
                if depth > 0:
                    raise ValueError("nested 'run' is not allowed")
                with open(path) as f:
                    tokens = []
                    for line in f:
                        tokens += shlex.split(line, comments=True)
                print(f"[d1] run {path} ({len(tokens)} tokens)")
                rc = run_commands(arm, tokens, args, depth + 1)
                if rc:
                    return rc
                i += 2
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
    except (IndexError, ValueError, OSError) as e:
        print(f"[d1] ERROR in command {cmds[i] if i < len(cmds) else '?'!r}: {e}", file=sys.stderr)
        return 1
    return 0

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", default=os.environ.get("ARM_IFACE", "enp0s8"),
                    help="wired interface the arm is on (default $ARM_IFACE or enp0s8)")
    ap.add_argument("--domain", type=int, default=0, help="DDS domain id")
    ap.add_argument("--peer", default=os.environ.get("ARM_PEER"),
                    help="unicast peer IP (disables multicast), e.g. 192.168.123.100")
    ap.add_argument("--enable-mode", type=int, default=ENABLE_FULL,
                    help="enable value: 0 = release, 80000 = fully locked (default), "
                         "1 = enable per doc table")
    ap.add_argument("--vmax", type=float, default=GLIDE_VMAX_DEG_S,
                    help=f"glide: peak joint speed, deg/s (default {GLIDE_VMAX_DEG_S:g})")
    ap.add_argument("--rate", type=float, default=GLIDE_RATE_HZ,
                    help=f"glide: setpoint stream rate, Hz (default {GLIDE_RATE_HZ:g})")
    ap.add_argument("--tolerance", type=float, default=GLIDE_TOLERANCE_DEG,
                    help=f"glide: 'reached' tolerance, deg (default {GLIDE_TOLERANCE_DEG:g})")
    ap.add_argument("--verbose", "-v", action="store_true")
    ap.add_argument("commands", nargs="+",
                    help="enable | release | home | power <0|1> | joint <id> <deg> | "
                         "joints <a0..a6> | glide <a0..a6> | gripper <deg> [delay_ms] | "
                         "angles | run <file> | feedback | status | motors | sleep <s>")
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

    rc = run_commands(arm, args.commands, args)
    if rc:
        return rc

    time.sleep(0.5)  # let the last writes flush before the participant is torn down
    return 3 if arm.missing_acks else 0


if __name__ == "__main__":
    sys.exit(main())