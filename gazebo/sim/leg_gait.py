#!/usr/bin/env python3
"""leg_gait.py: a kinematic trot for the simulated Go2 (SIM_GAIT=1).

Turns the body's commanded velocity and a foot lift height into 12 joint angles
with inverse kinematics. Stance feet move backwards at the body's speed, so they
stay planted on the floor; swing feet travel forward through an arc.

This shows the legs and measures foot clearance. It is NOT a walking simulation:
the body is still driven by VelocityControl, nothing balances on these feet, and
no ground force from them moves the robot.

Uses only the Python standard library. Tested by tests/test_leg_gait.py.
"""
import math

THIGH = 0.213
CALF = 0.213
HIP_OFFSET = {"FL": (0.1934, 0.0465), "FR": (0.1934, -0.0465),
              "RL": (-0.1934, 0.0465), "RR": (-0.1934, -0.0465)}
THIGH_Y = 0.0955
PHASE = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}   # trot: diagonal pairs together
STAND_HEIGHT = 2 * THIGH * math.cos(0.9)
LEGS = ("FR", "FL", "RR", "RL")


def leg_ik(leg, x, y, z):
    """Foot (x, y, z) from the hip joint, body axes -> (hip, thigh, calf) angles."""
    d = THIGH_Y if leg.endswith("L") else -THIGH_Y
    length = math.sqrt(max(y * y + z * z - d * d, 1e-6))
    hip = math.atan2(y, -z) - math.atan2(d, length)
    reach = min(math.hypot(x, length), (THIGH + CALF) * 0.999)
    calf = -math.acos(max(-1.0, min(1.0, (reach * reach - THIGH ** 2 - CALF ** 2) / (2 * THIGH * CALF))))
    thigh = math.atan2(-x, length) - calf / 2
    return hip, thigh, calf


def leg_fk(leg, hip, thigh, calf):
    """Foot position from the hip joint, for tests and clearance checks."""
    d = THIGH_Y if leg.endswith("L") else -THIGH_Y
    x = -THIGH * math.sin(thigh) - CALF * math.sin(thigh + calf)
    length = THIGH * math.cos(thigh) + CALF * math.cos(thigh + calf)
    return x, d * math.cos(hip) + length * math.sin(hip), d * math.sin(hip) - length * math.cos(hip)


class Trot:
    """avoid(leg, x, y) -> True if landing there, in body axes now, would be on an
    object. Each swing tries a few landing offsets and takes the first clear one."""

    OFFSETS = (0.0, 0.04, -0.04, 0.08, -0.08, 0.12, -0.12, 0.16, -0.16)

    def __init__(self, period=0.6, body_height=STAND_HEIGHT, still_speed=0.01, still_yaw=0.02):
        self.period, self.body_height = period, body_height
        self.still_speed, self.still_yaw = still_speed, still_yaw
        self.phase = 0.0
        self.amount = 0.0     # 0 standing, 1 walking: fades the swing in and out
        self.offset = {leg: 0.0 for leg in LEGS}       # current stance foot's landing offset
        self.next_offset = {leg: 0.0 for leg in LEGS}  # the swinging foot's
        self.in_swing = {leg: False for leg in LEGS}

    def plan_landing(self, leg, ux, uy, vx, vy, avoid):
        half = self.period / 4
        hx, hy = HIP_OFFSET[leg]
        side = THIGH_Y if leg.endswith("L") else -THIGH_Y
        lx = hx + ux * half + vx * self.period / 2
        ly = hy + side + uy * half + vy * self.period / 2
        for off in self.OFFSETS:
            if not avoid(leg, lx + off, ly):
                return off
        return 0.0

    def foot(self, leg, vx, vy, wz, lift, avoid=None):
        hx, hy = HIP_OFFSET[leg]
        fy = THIGH_Y if leg.endswith("L") else -THIGH_Y
        # Velocity of this foot's rest point, from the body's motion.
        px, py = hx, hy + fy
        ux, uy = vx - wz * py, vy + wz * px
        half = self.period / 4
        t = (self.phase + PHASE[leg]) % 1.0
        if t < 0.5:          # stance: planted, so it moves back at the body's speed
            if self.in_swing[leg]:
                self.in_swing[leg] = False
                self.offset[leg] = self.next_offset[leg]
            s = 1.0 - 2.0 * (t / 0.5)
            return ux * half * s + self.offset[leg], uy * half * s, 0.0
        if not self.in_swing[leg]:
            self.in_swing[leg] = True
            self.next_offset[leg] = self.plan_landing(leg, ux, uy, vx, vy, avoid) if avoid else 0.0
        u = (t - 0.5) / 0.5  # swing: up first, across while high, then down
        s = -math.cos(math.pi * max(0.0, min(1.0, (u - 0.2) / 0.6)))
        dx = self.offset[leg] + (self.next_offset[leg] - self.offset[leg]) * (1 + s) / 2
        return ux * half * s + dx, uy * half * s, lift * self.amount * min(1.0, 2.5 * math.sin(math.pi * u))

    def step(self, dt, vx, vy, wz, lift, avoid=None):
        """Returns {joint name: angle} and {leg: foot position from the base, in body axes}."""
        moving = math.hypot(vx, vy) > self.still_speed or abs(wz) > self.still_yaw
        self.amount = min(1.0, self.amount + dt / 0.2) if moving else max(0.0, self.amount - dt / 0.2)
        if self.amount > 0.0:
            self.phase = (self.phase + dt / self.period) % 1.0
        joints, feet = {}, {}
        for leg in LEGS:
            fx, fy_off, fz = self.foot(leg, vx, vy, wz, lift, avoid)
            hx, hy = HIP_OFFSET[leg]
            side = THIGH_Y if leg.endswith("L") else -THIGH_Y
            x, y, z = fx, side + fy_off, -self.body_height + fz
            hip, thigh, calf = leg_ik(leg, x, y, z)
            joints[f"{leg}_hip_joint"] = hip
            joints[f"{leg}_thigh_joint"] = thigh
            joints[f"{leg}_calf_joint"] = calf
            feet[leg] = (hx + x, hy + y, z)
        return joints, feet
