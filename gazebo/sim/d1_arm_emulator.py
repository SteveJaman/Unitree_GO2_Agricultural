#!/usr/bin/env python3
"""
d1_arm_emulator.py: a simulated Unitree D1 controller.

Speaks the real arm protocol so the unmodified arm scripts (start_arm.sh,
d1_arm_calibration.py, d1_arm_control.py, diag_arm.sh) work against Gazebo:

  rt/arm_Command   (ArmString_ JSON, address 1)  <- scripts
  rt/arm_Feedback  (ArmString_ JSON)             -> scripts
      address 3 funcode 1  {"recv_status": 1}           ack per command
      address 2 funcode 1  {"angle0".."angle6"}  10 Hz  joint angles (deg)
      address 2 funcode 3  {"enable_status", ...} 1 Hz  status (sim values)
  current_servo_angle (PubServoInfo_)            10 Hz  joint angles (deg)

DDS domain 0, bound to --iface (the sim arm interface, simarm0), exactly
like the scripts. Joint targets go to Gazebo JointPositionControllers over
gz-transport; measured angles come back from the JointStatePublisher.

Funcodes: 1 single joint {id, angle, delay_ms}, 2 all joints {mode, angle0..6},
4 single enable (ack only), 5 enable all {mode: 0 = release, else enable},
6 power (ack only), 7 home (all zero).

Simulation differences (documented, deliberate):
  * Motion is rate-limited to --max-speed deg/s (gripper --gripper-speed), or
    paced by delay_ms when given. The real arm's internal profile is unknown.
  * Motion commands while not enabled are acknowledged but ignored.
  * "release" cannot make Gazebo joints limp; the arm holds its pose.
  * Gripper: protocol degrees 0..gripper_open_deg map to finger travel
    0..finger_travel_m (both fingers, opposite directions).

Run WITHOUT ROS sourced (pip cyclonedds and ROS's libddsc must not mix).
--no-gz runs a kinematic arm with no Gazebo (handy for testing the scripts).
"""
import argparse
import json
import math
import os
import sys
import threading
import time
from dataclasses import dataclass

ADDR_CMD, ADDR_TELEMETRY, ADDR_ACK = 1, 2, 3


# ----------------------------------------------------------- protocol logic
class ArmModel:
    """The D1 controller's behaviour, with no I/O (unit-testable)."""

    def __init__(self, limits_deg, gripper_open_deg=65.0, max_speed=60.0, gripper_speed=100.0):
        self.limits = list(limits_deg)              # 6 x (lo, hi) for joints 0-5
        self.gripper_open = gripper_open_deg
        self.max_speed, self.gripper_speed = max_speed, gripper_speed
        self.enabled = False
        self.commanded = [0.0] * 7
        self.targets = [0.0] * 7
        self.speeds = [max_speed] * 6 + [gripper_speed]
        self.measured = None                         # last angles from Gazebo (deg)
        self.notes = []

    def _clamp(self, i, a):
        lo, hi = (0.0, self.gripper_open) if i == 6 else self.limits[i]
        if a < lo or a > hi:
            self.notes.append(f"joint {i}: {a:.1f} deg clamped to [{lo:.1f}, {hi:.1f}]")
        return max(lo, min(hi, a))

    def current(self):
        return list(self.measured) if self.measured is not None else list(self.commanded)

    def handle(self, text):
        """Process one rt/arm_Command string. Returns feedback dicts to send."""
        try:
            msg = json.loads(text)
            if not isinstance(msg, dict) or msg.get("address") != ADDR_CMD:
                raise ValueError("not an address-1 command")
            func = int(msg["funcode"])
            data = msg.get("data")
            if data is not None and not isinstance(data, dict):
                raise ValueError("data must be a JSON object")
        except (ValueError, KeyError, TypeError) as e:
            self.notes.append(f"rejected: {e}")
            return [self._ack(None, 0)]
        seq = msg.get("seq")
        data = data or {}
        ok = 1
        if func == 1:
            jid = int(data.get("id", -1))
            if not 0 <= jid <= 6 or "angle" not in data:
                ok = 0
            elif self._require_enabled("joint"):
                self._set_target(jid, float(data["angle"]), int(data.get("delay_ms", 0) or 0))
        elif func == 2:
            if self._require_enabled("joints"):
                for i in range(7):
                    if f"angle{i}" in data:
                        self._set_target(i, float(data[f"angle{i}"]), 0)
        elif func == 5:
            mode = int(data.get("mode", 1))
            if mode == 0:
                if self.enabled:
                    self.notes.append("release: real arm goes limp; the simulated arm holds its pose")
                self.enabled = False
            else:
                if not self.enabled:
                    here = self.current()           # no jump when torque comes on
                    self.commanded, self.targets = list(here), list(here)
                self.enabled = True
        elif func == 7:
            if self._require_enabled("home"):
                for i in range(6):
                    self._set_target(i, 0.0, 0)
        elif func in (4, 6):
            pass                                     # acknowledged, no simulated effect
        else:
            ok = 0
        return [self._ack(seq, ok)]

    def _require_enabled(self, what):
        if not self.enabled:
            self.notes.append(f"{what} ignored: motors not enabled (send enable first)")
        return self.enabled

    def _set_target(self, i, angle, delay_ms):
        self.targets[i] = self._clamp(i, angle)
        base = self.gripper_speed if i == 6 else self.max_speed
        if delay_ms > 0:
            dist = abs(self.targets[i] - self.commanded[i])
            self.speeds[i] = max(1e-3, min(base, dist / (delay_ms / 1000.0)))
        else:
            self.speeds[i] = base

    def step(self, dt):
        """Advance commanded angles toward targets (rate-limited). Returns commanded."""
        if self.enabled:
            for i in range(7):
                d = self.targets[i] - self.commanded[i]
                lim = self.speeds[i] * dt
                self.commanded[i] += max(-lim, min(lim, d))
        return list(self.commanded)

    @staticmethod
    def _ack(seq, ok):
        return {"seq": seq, "address": ADDR_ACK, "funcode": 1, "data": {"recv_status": ok}}

    def telemetry(self, seq):
        a = self.current()
        return {"seq": seq, "address": ADDR_TELEMETRY, "funcode": 1,
                "data": {f"angle{i}": round(a[i], 2) for i in range(7)}}

    def status(self, seq):
        return {"seq": seq, "address": ADDR_TELEMETRY, "funcode": 3,
                "data": {"enable_status": 1 if self.enabled else 0, "power_status": 1,
                         "error_status": 0, "sim": 1}}


# ----------------------------------------------------------- unit mapping
def to_gz(i, deg, p):
    """Protocol degrees -> Gazebo joint command(s): list of (joint, value)."""
    joints = p["d1_joints"]
    if i < 6:
        return [(joints[i], math.radians(deg))]
    travel = max(0.0, min(1.0, deg / p["gripper_open_deg"])) * p["finger_travel_m"]
    return [(joints[6], travel), (joints[7], -travel)]


def from_gz(positions, p):
    """Gazebo joint positions (rad / m) -> 7 protocol angles (deg), or None."""
    joints = p["d1_joints"]
    if not all(j in positions for j in joints[:7]):
        return None
    out = [math.degrees(positions[j]) for j in joints[:6]]
    out.append(abs(positions[joints[6]]) / p["finger_travel_m"] * p["gripper_open_deg"])
    return out


# ----------------------------------------------------------- I/O
def cyclonedds_uri(iface):
    return ('<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any"><General><Interfaces>'
            f'<NetworkInterface name="{iface}" multicast="default"/>'
            '</Interfaces></General></Domain></CycloneDDS>')


def run(args, params):
    os.environ["CYCLONEDDS_URI"] = cyclonedds_uri(args.iface)
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.idl import IdlStruct
    from cyclonedds.idl import types as idl_types
    from cyclonedds.pub import DataWriter
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic

    @dataclass
    class ArmString_(IdlStruct, typename="unitree_arm::msg::dds_::ArmString_"):
        data: str = ""

    @dataclass
    class PubServoInfo_(IdlStruct, typename="unitree_arm::msg::dds_::PubServoInfo_"):
        servo0_data: idl_types.float32 = 0.0
        servo1_data: idl_types.float32 = 0.0
        servo2_data: idl_types.float32 = 0.0
        servo3_data: idl_types.float32 = 0.0
        servo4_data: idl_types.float32 = 0.0
        servo5_data: idl_types.float32 = 0.0
        servo6_data: idl_types.float32 = 0.0

    limits = [tuple(math.degrees(v) for v in params["d1_limits_rad"][j]) for j in params["d1_joints"][:6]]
    arm = ArmModel(limits, params["gripper_open_deg"], args.max_speed, args.gripper_speed)

    dp = DomainParticipant(0)
    cmd_reader = DataReader(dp, Topic(dp, "rt/arm_Command", ArmString_))
    fb_writer = DataWriter(dp, Topic(dp, "rt/arm_Feedback", ArmString_))
    servo_writer = DataWriter(dp, Topic(dp, "current_servo_angle", PubServoInfo_))

    lock = threading.Lock()
    gz_pubs = {}
    if not args.no_gz:
        from gz.msgs10.double_pb2 import Double
        from gz.msgs10.model_pb2 import Model
        from gz.transport13 import Node
        node = Node()
        for j in params["d1_joints"]:
            gz_pubs[j] = (node.advertise(f"/sim/d1/{j}/cmd_pos", Double), Double)

        def on_joint_state(msg):
            pos = {j.name: j.axis1.position for j in msg.joint}
            angles = from_gz(pos, params)
            if angles is not None:
                with lock:
                    arm.measured = angles

        node.subscribe(Model, "/sim/d1/joint_state", on_joint_state)

    def send(obj):
        fb_writer.write(ArmString_(data=json.dumps(obj, separators=(",", ":"))))

    print(f"D1 ARM EMULATOR READY on {args.iface} (DDS domain 0, "
          f"{'kinematic, no Gazebo' if args.no_gz else 'Gazebo joints'})", flush=True)
    period, seq = 1.0 / 50, 0
    last_tel = last_status = 0.0
    while True:
        t0 = time.monotonic()
        for s in cmd_reader.take(N=50):
            if s is None or not getattr(s, "data", None):
                continue
            with lock:
                replies = arm.handle(s.data)
            for r in replies:
                send(r)
        with lock:
            commanded = arm.step(period)
            if args.no_gz:
                arm.measured = list(commanded)
            notes, arm.notes = arm.notes, []
        for n in notes:
            print(f"[d1-sim] {n}", flush=True)
        for i, deg in enumerate(commanded):
            for joint, value in to_gz(i, deg, params):
                if joint in gz_pubs:
                    pub, Double = gz_pubs[joint]
                    m = Double()
                    m.data = value
                    pub.publish(m)
        now = time.monotonic()
        if now - last_tel >= 0.1:
            seq += 1
            with lock:
                send(arm.telemetry(seq))
                a = arm.current()
            servo_writer.write(PubServoInfo_(*[float(v) for v in a]))
            last_tel = now
        if now - last_status >= 1.0:
            with lock:
                send(arm.status(seq))
            last_status = now
        time.sleep(max(0.0, period - (time.monotonic() - t0)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", default=os.environ.get("SIM_ARM_IFACE", "simarm0"))
    ap.add_argument("--params", required=True, help="sim_params.json from build_model.py")
    ap.add_argument("--max-speed", type=float, default=60.0, help="joint speed limit, deg/s")
    ap.add_argument("--gripper-speed", type=float, default=100.0, help="gripper speed, deg/s")
    ap.add_argument("--no-gz", action="store_true", help="kinematic arm, no Gazebo")
    args = ap.parse_args()
    with open(args.params) as f:
        params = json.load(f)
    try:
        run(args, params)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
