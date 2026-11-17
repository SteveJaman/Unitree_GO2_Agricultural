#!/usr/bin/env python3
"""
step_over_crawl.py: per-leg step-over for the real Go2 by a slow, statically stable crawl.

The Go2's sport mode has no foot-height control on this firmware (FootRaiseHeight 3203,
remote L1+up measured no change, 2026-11-16), so the legs are driven directly in
LOW-LEVEL mode: Unitree's balance controller is OFF while this runs, and the dog stays
up only because three feet are always on the floor with the body over them.

  step_over_crawl.py --plan                       # print and check the crawl, moves nothing
  step_over_crawl.py --plan --object-height 0.08 --object-depth 0.10 --csv /tmp/plan.csv
  step_over_crawl.py --check                      # ON THE DOG, read only: joint states and mode
  step_over_crawl.py --stand                      # ON THE DOG: sport mode off, stand, back
  step_over_crawl.py --lift FR --dry              # the one-leg lift's poses, moves nothing
  step_over_crawl.py --lift FR                    # ON THE DOG: stand, body over 3 feet, FR up, back
  step_over_crawl.py --step-over --stop-after 5   # ON THE DOG: the crawl, its first 5 keyframes only
  step_over_crawl.py --step-over                  # ON THE DOG: the whole crawl over the object

--plan     no robot, no SDK: the footholds, body shifts and 12 joint angles for every
           keyframe, checked against the joint limits, the leg reach and the stability
           margin (the body's centre inside the triangle of the three standing feet).
--stand    the first hardware stage. Sport mode lies the dog down, sport mode is
           released (MotionSwitcher), the dog is stood up and held by joint position
           control, laid down again, and sport mode is handed back.

Later stages (not here yet): --lift LEG, then --step-over, each only after the one
before worked with the dog in a harness. docs/hardware/step_over_crawl.md.

SAFETY (--stand): dog on the floor, clear 1 m around it, a harness or a hand on it the
first time, someone on the remote. Ctrl+C, a stale /lowstate (> 0.1 s) or a tilt over
25 deg: the joints go soft (damping) and the dog settles, then sport mode is handed back.
Runs ON THE DOG (the control loop at 500 Hz must not cross Wi-Fi), with
unitree_sdk2_python installed there.
"""

import argparse
import csv
import math
import sys
import time

# --- Go2 geometry (gazebo/models/go2_description, as step_height_probe.py) ---------
LEGS = ("FR", "FL", "RR", "RL")          # /lowstate motor order: hip, thigh, calf per leg
HIP_X, HIP_Y, L_ABAD, L_THIGH, L_CALF = 0.1934, 0.0465, 0.0955, 0.213, 0.213
FOOT_R = 0.022                            # foot sphere radius
LIMITS = {                                # rad, from the URDF
    "hip": (-1.0472, 1.0472),
    "thigh_front": (-1.5708, 3.4907),
    "thigh_rear": (-0.5236, 4.5379),
    "calf": (-2.7227, -0.83776),
}
LIMIT_MARGIN = 0.05                       # rad kept away from every limit
# Unitree's go2_stand_example.py poses (FR, FL, RR, RL x hip, thigh, calf).
LIE_DOWN = [0.0, 1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65]


def side(leg):
    return 1.0 if leg.endswith("L") else -1.0


def front(leg):
    return leg.startswith("F")


def hip_xy(leg):
    return (HIP_X if front(leg) else -HIP_X, side(leg) * HIP_Y)


def leg_fk(leg, q0, q1, q2):
    """Foot centre relative to the hip joint, body axes (x forward, y left, z up)."""
    d = side(leg) * L_ABAD
    x = -L_THIGH * math.sin(q1) - L_CALF * math.sin(q1 + q2)
    zp = -L_THIGH * math.cos(q1) - L_CALF * math.cos(q1 + q2)
    return x, d * math.cos(q0) - zp * math.sin(q0), d * math.sin(q0) + zp * math.cos(q0)


def leg_ik(leg, x, y, z):
    """(hip, thigh, calf) for a foot centre at (x, y, z) from the hip joint; knee backward.
    Raises ValueError when the point is out of reach."""
    d = side(leg) * L_ABAD
    r2 = y * y + z * z - d * d
    if r2 <= 0:
        raise ValueError("foot inside the ab/ad offset")
    zp = -math.sqrt(r2)
    q0 = math.atan2(z, y) - math.atan2(zp, d)
    q0 = (q0 + math.pi) % (2 * math.pi) - math.pi
    reach = math.hypot(x, zp)
    c2 = (reach * reach - L_THIGH ** 2 - L_CALF ** 2) / (2 * L_THIGH * L_CALF)
    if not -1.0 <= c2 <= 1.0:
        raise ValueError(f"out of reach ({reach:.3f} m)")
    q2 = -math.acos(c2)
    q1 = math.atan2(-x, -zp) - math.atan2(L_CALF * math.sin(q2), L_THIGH + L_CALF * math.cos(q2))
    return q0, q1, q2


def limit_violations(leg, q):
    out = []
    names = ("hip", "thigh_front" if front(leg) else "thigh_rear", "calf")
    for name, v in zip(names, q):
        lo, hi = LIMITS[name]
        if not lo + LIMIT_MARGIN <= v <= hi - LIMIT_MARGIN:
            out.append(f"{leg} {name.split('_')[0]} {v:+.2f} rad outside [{lo:+.2f}, {hi:+.2f}]")
    return out


# --- Feed-forward: hold the dog's weight before the legs bend ---------------------------
# Pure position control (kp) only pushes back once a joint has bent away from its target:
# under the dog's weight the legs bent until it tipped over (crawl tests 2 and 3,
# 2026-11-16). This gives each standing leg the joint torque that carries its share.
TAU_MAX = (18.0, 18.0, 30.0)      # N m per hip / thigh / calf (motor peaks 23.7 / 23.7 / 45)
CONTACT_LIFT = 0.015              # m: a foot this far above its standing height carries nothing


def leg_z_jacobian(leg, q3, h=1e-5):
    """d(foot z)/dq for the three joints, by central differences of leg_fk."""
    out = []
    for k in range(3):
        a, b = list(q3), list(q3)
        a[k] += h
        b[k] -= h
        out.append((leg_fk(leg, *a)[2] - leg_fk(leg, *b)[2]) / (2 * h))
    return out


def support_forces(feet, weights, com, total):
    """Vertical forces (N) on the feet that hold `total` N with its centre at com (x, y):
    the smallest forces (weighted by contact) meeting force and moment balance. feet:
    [(x, y)] in body axes; weights 0..1 (0: in the air)."""
    idx = [i for i, w in enumerate(weights) if w > 0.05]
    forces = [0.0] * len(feet)
    if not idx:
        return forces
    if len(idx) < 3:                              # a line or a point: share by weight only
        tw = sum(weights[i] for i in idx)
        for i in idx:
            forces[i] = total * weights[i] / tw
        return forces
    # min sum(F_i^2 / w_i)  s.t.  A F = b  ->  F = W A^T (A W A^T)^-1 b
    a_rows = [[1.0] * len(idx), [feet[i][0] for i in idx], [feet[i][1] for i in idx]]
    b = [total, total * com[0], total * com[1]]
    w = [weights[i] for i in idx]
    m = [[sum(a_rows[r][k] * w[k] * a_rows[c][k] for k in range(len(idx))) for c in range(3)] for r in range(3)]
    lam = solve3(m, b)
    f = [w[k] * sum(a_rows[r][k] * lam[r] for r in range(3)) for k in range(len(idx))]
    f = [max(0.0, v) for v in f]                  # feet can only push
    scale = total / sum(f) if sum(f) > 0 else 0.0
    for k, i in enumerate(idx):
        forces[i] = f[k] * scale
    return forces


def solve3(m, b):
    """3x3 linear solve (Cramer); m is well conditioned for three or more spread feet."""
    def det(a):
        return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
                - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    d = det(m)
    out = []
    for c in range(3):
        mc = [row[:] for row in m]
        for r in range(3):
            mc[r][c] = b[r]
        out.append(det(mc) / d)
    return out


def ff_torques(q12, stand_height, mass, com=(0.0, 0.0), g=9.81):
    """Joint torques (motor order) that hold the dog's weight on the feet that are down,
    for the joint angles q12. A foot's load fades out as it rises CONTACT_LIFT above its
    standing height. Clamped to TAU_MAX."""
    feet, weights, jac = [], [], []
    for i, leg in enumerate(LEGS):
        q3 = q12[3 * i: 3 * i + 3]
        x, y, z = leg_fk(leg, *q3)
        hx, hy = hip_xy(leg)
        feet.append((hx + x, hy + y))
        weights.append(max(0.0, min(1.0, 1.0 - (z + stand_height) / CONTACT_LIFT)))
        jac.append(leg_z_jacobian(leg, q3))
    forces = support_forces(feet, weights, com, mass * g)
    tau = []
    for i in range(4):
        for k in range(3):
            t = -forces[i] * jac[i][k]                 # push the foot down with forces[i]
            tau.append(max(-TAU_MAX[k], min(TAU_MAX[k], t)))
    return tau


# --- Stability -------------------------------------------------------------------------
def triangle_margin(p, tri):
    """Signed distance (m) from p to the nearest edge of triangle tri; > 0 inside."""
    (ax, ay), (bx, by), (cx, cy) = tri
    area = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    s = 1.0 if area > 0 else -1.0
    best = math.inf
    for (x1, y1), (x2, y2) in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
        ex, ey = x2 - x1, y2 - y1
        n = math.hypot(ex, ey) or 1e-9
        best = min(best, s * (ex * (p[1] - y1) - ey * (p[0] - x1)) / n)
    return best


def shift_target(body, tri, margin):
    """Where to move the body before a leg swings: the point on the line from the body to
    the centroid of tri (the three standing feet) nearest the body that is at least
    margin inside tri. A smaller shift than the centroid keeps the legs less stretched
    (the full shift to the centroid stretched a rear leg past its calf limit)."""
    c = (sum(p[0] for p in tri) / 3, sum(p[1] for p in tri) / 3)
    if triangle_margin(body, tri) >= margin:
        return body
    if triangle_margin(c, tri) < margin:
        return c                                   # the best the triangle allows
    lo, hi = 0.0, 1.0                              # fraction of the way to the centroid
    for _ in range(30):
        mid = (lo + hi) / 2
        p = (body[0] + (c[0] - body[0]) * mid, body[1] + (c[1] - body[1]) * mid)
        if triangle_margin(p, tri) >= margin:
            hi = mid
        else:
            lo = mid
    return (body[0] + (c[0] - body[0]) * hi, body[1] + (c[1] - body[1]) * hi)


# --- The crawl plan --------------------------------------------------------------------
class Plan:
    """Keyframes in world axes: origin under the body centre at the start, x forward.

    Each keyframe: name, duration (s), body (x, y), feet {leg: (x, y, z)}, the swing leg
    (or None) and the stability margin at that moment (None when all four are down)."""

    def __init__(self, a):
        self.a = a
        self.h = a.stand_height
        self.keys = []
        out = getattr(a, "stance_out", 0.0)       # feet this far outward: bigger triangles
        self.feet = {leg: (hip_xy(leg)[0], hip_xy(leg)[1] + side(leg) * (L_ABAD + out), 0.0) for leg in LEGS}
        self.body = (0.0, 0.0)
        # Object, in world x: its near edge object_distance ahead of the front feet.
        self.obj_near = HIP_X + a.object_distance
        self.obj_far = self.obj_near + a.object_depth
        self.key("stand", 0.0, None)

    def key(self, name, dur, swing):
        margin = None
        if swing is not None:
            tri = [self.feet[leg][:2] for leg in LEGS if leg != swing]
            margin = triangle_margin(self.body, tri)
        self.keys.append({"name": name, "t": dur, "body": self.body, "feet": dict(self.feet),
                          "swing": swing, "margin": margin})

    def forbidden(self, x):
        m = self.a.landing_margin + FOOT_R
        return self.obj_near - m < x < self.obj_far + m

    def landing(self, leg, x0):
        """Next foothold x for a leg at x0: one stride, shortened or lengthened so it is
        not on the object; (x, crosses_object)."""
        a, m = self.a, self.a.landing_margin + FOOT_R
        before, beyond = self.obj_near - m, self.obj_far + m
        want = x0 + a.stride
        if x0 >= beyond - 1e-9 or want <= before + 1e-9:
            return want, False                          # all on one side of the object
        # The step would reach or pass the object: cross it in one high step, or stop short.
        if beyond - x0 <= a.max_stride:
            return beyond, True
        return max(x0, before), False

    def swing(self, leg):
        a = self.a
        x0, y0, _ = self.feet[leg]
        x1, crosses = self.landing(leg, x0)
        if x1 - x0 < 0.01:
            return False
        lift = (a.object_height + a.clearance) if crosses else a.normal_lift
        tri = [self.feet[o][:2] for o in LEGS if o != leg]
        self.body = shift_target(self.body, tri, 1.5 * a.stability_margin)
        self.key(f"shift for {leg}", a.t_shift, leg)
        if getattr(a, "t_settle", 0) > 0:
            # Hold before the foot lifts: the sag correction catches up with the new load
            # (crawl test 2, 2026-11-16: fast phases left it behind and the dog tipped at FR).
            self.key(f"settle for {leg}", a.t_settle, leg)
        self.feet[leg] = (x0, y0, lift)
        self.key(f"{leg} up {lift * 100:.0f} cm", a.t_lift, leg)
        self.feet[leg] = (x1, y0, lift)
        self.key(f"{leg} forward {(x1 - x0) * 100:.0f} cm{' over the object' if crosses else ''}",
                 a.t_move, leg)
        self.feet[leg] = (x1, y0, 0.0)
        self.key(f"{leg} down", a.t_lower, leg)
        return True

    def build(self):
        a = self.a
        rear_done = self.obj_far + a.landing_margin + FOOT_R
        for cycle in range(a.max_cycles):
            if min(self.feet[leg][0] for leg in ("RR", "RL")) > rear_done:
                break
            moved = False
            for n, leg in enumerate(a.order.split(",")):
                moved |= self.swing(leg)
                if (n + 1) % getattr(a, "recentre_every", 4) == 0:
                    # Re-centre over all four feet, so no leg ends folded far forward or back
                    # (crawl test 3: a rear leg folded under the body bent under the load).
                    self.body = (sum(f[0] for f in self.feet.values()) / 4, 0.0)
                    self.key(f"cycle {cycle + 1}: body over the feet", a.t_shift, None)
            if not moved:
                break
        return self

    def uncrossed(self):
        """Problems for legs that never got past the object (it is too deep to cross in
        one step of at most max_stride)."""
        m = self.a.landing_margin + FOOT_R
        need = self.obj_far + m - (self.obj_near - m)
        return [f"{leg} never crossed the object: it needs one step of {need * 100:.0f} cm, "
                f"--max-stride is {self.a.max_stride * 100:.0f} cm (object too deep)"
                for leg in LEGS if self.feet[leg][0] < self.obj_far + m - 1e-9]

    def joints(self, k):
        """12 joint angles for keyframe k (motor order), and any problems."""
        q, problems = [], []
        bx, by = k["body"]
        for leg in LEGS:
            fx, fy, fz = k["feet"][leg]
            hx, hy = hip_xy(leg)
            x, y, z = fx - bx - hx, fy - by - hy, fz - self.h
            try:
                ql = leg_ik(leg, x, y, z)
            except ValueError as ex:
                problems.append(f"{leg}: {ex}")
                ql = (0.0, 0.0, -1.0)
            problems += limit_violations(leg, ql)
            q += list(ql)
        if k["margin"] is not None and k["margin"] < self.a.stability_margin:
            problems.append(f"stability margin {k['margin'] * 100:.1f} cm < "
                            f"{self.a.stability_margin * 100:.1f} cm")
        return q, problems


def smooth(s):
    return s * s * (3 - 2 * s)


def dense(plan, dt=0.02):
    """The crawl sampled every dt s: body and feet interpolated in real space between
    keyframes (smooth start and stop), joints solved at every sample, so the swinging
    foot moves on the planned path instead of a joint-space blend that can dip.
    [(keyframe index, name, q12, problems, feet)]."""
    out = []
    for i in range(1, len(plan.keys)):
        k0, k1 = plan.keys[i - 1], plan.keys[i]
        n = max(1, int(round(k1["t"] / dt)))
        for j in range(1, n + 1):
            s = smooth(j / n)
            body = tuple(a + (b - a) * s for a, b in zip(k0["body"], k1["body"]))
            feet = {leg: tuple(a + (b - a) * s for a, b in zip(k0["feet"][leg], k1["feet"][leg]))
                    for leg in LEGS}
            q, problems = plan.joints({"body": body, "feet": feet, "margin": None})
            out.append((i, k1["name"], q, problems, feet))
    return out


def over_object_clearance(plan, samples):
    """Lowest height (m) any foot has above the object's top while it is over the object
    (foot centre within FOOT_R of the object along x); None if no foot passes over."""
    worst = None
    for _i, _name, _q, _p, feet in samples:
        for x, _y, z in feet.values():
            if plan.obj_near - FOOT_R < x < plan.obj_far + FOOT_R and z > 0:
                c = z - plan.a.object_height
                worst = c if worst is None else min(worst, c)
    return worst


def stand_joints(stand_height, stance_out=0.0):
    """12 joint angles for the stand pose: every foot under its hip (plus stance_out
    outward), stand_height below."""
    q = []
    for leg in LEGS:
        q += list(leg_ik(leg, 0.0, side(leg) * (L_ABAD + stance_out), -stand_height))
    return q


def print_plan(plan, csv_path=""):
    a = plan.a
    print(f"Object: {a.object_height * 100:.0f} cm high, {a.object_depth * 100:.0f} cm deep, "
          f"near edge {a.object_distance * 100:.0f} cm ahead of the front feet "
          f"(world x {plan.obj_near:.3f}-{plan.obj_far:.3f} m)")
    print(f"Stand height {a.stand_height:.2f} m (hip to foot centre); lift over the object "
          f"{(a.object_height + a.clearance) * 100:.0f} cm, elsewhere {a.normal_lift * 100:.0f} cm")
    print()
    hdr = f"{'#':>3} {'t':>4}  {'keyframe':<38} {'body x,y':>12} {'margin':>7}  joints (rad, FR FL RR RL)"
    print(hdr)
    total, bad, worst = 0.0, 0, math.inf
    rows = []
    for i, k in enumerate(plan.keys):
        q, problems = plan.joints(k)
        total += k["t"]
        m = k["margin"]
        if m is not None:
            worst = min(worst, m)
        ms = f"{m * 100:5.1f}cm" if m is not None else "   -   "
        js = " ".join(f"{v:+.2f}" for v in q)
        print(f"{i:3d} {k['t']:4.1f}  {k['name']:<38} {k['body'][0]:+.3f},{k['body'][1]:+.3f} {ms}  {js}")
        for p in problems:
            print(f"      !! {p}")
        bad += len(problems)
        rows.append([i, k["t"], k["name"], *k["body"], "" if m is None else m, *q])
    for p in plan.uncrossed():
        print(f"      !! {p}")
        bad += 1
    samples = dense(plan)
    sample_bad = [(i, name, p) for i, name, _q, problems, _f in samples for p in problems]
    for i, name, p in sample_bad[:10]:
        print(f"      !! between keyframes {i - 1} and {i} ({name}): {p}")
    bad += len(sample_bad)
    clear = over_object_clearance(plan, samples)
    print(f"Every 20 ms sample checked ({len(samples)}): {len(sample_bad)} problems; lowest foot over "
          f"the object {'-' if clear is None else f'{clear * 100:.1f} cm above its top'}")
    if clear is not None and clear < a.clearance / 2:
        print(f"      !! a foot passes only {clear * 100:.1f} cm over the object")
        bad += 1
    print()
    print(f"{len(plan.keys)} keyframes, {total:.0f} s; worst stability margin "
          f"{worst * 100:.1f} cm; problems: {bad}")
    if csv_path:
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["i", "t", "keyframe", "body_x", "body_y", "margin"]
                       + [f"{leg}_{j}" for leg in LEGS for j in ("hip", "thigh", "calf")])
            w.writerows(rows)
        print(f"written {csv_path}")
    return bad


# --- Hardware: --stand -------------------------------------------------------------------
class LowLevel:
    """unitree_sdk2py low-level control at 500 Hz, with the safety stops."""

    KP, KD = 60.0, 5.0          # Unitree's stand example
    DAMP_KD = 3.0
    DT = 0.002

    def __init__(self, iface):
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.go2.sport.sport_client import SportClient
        from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        ChannelFactoryInitialize(0, iface)
        self.crc = CRC()
        self.cmd = unitree_go_msg_dds__LowCmd_()
        self.cmd.head[0], self.cmd.head[1] = 0xFE, 0xEF
        self.cmd.level_flag, self.cmd.gpio = 0xFF, 0
        for i in range(20):
            m = self.cmd.motor_cmd[i]
            m.mode, m.q, m.dq, m.kp, m.kd, m.tau = 0x01, 2.146e9, 16000.0, 0.0, 0.0, 0.0
        self.state, self.state_t = None, 0.0
        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.pub.Init()
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self._on_state, 10)
        self.sport = SportClient()
        self.sport.SetTimeout(5.0)
        self.sport.Init()
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()
        self.mode_name = ""
        self.kp = self.KP           # raised for the 3-leg phases (--kp-lift)
        # Integral correction (--ki): pure PD sagged ~0.2 rad at the rear thighs and calves
        # under the dog's weight (lift_FR_v2.csv, 2026-11-16). i_corr is added to every
        # command and grows with the remaining error, capped at I_MAX.
        self.ki, self.i_corr = 0.0, [0.0] * 12
        self.I_MAX = 0.35
        self.i_fade = 0.0           # > 0: the correction fades out over this many s (lying down)
        self.q_ref = None
        # Feed-forward (ff_torques): gain ramps 0 -> ff_target at 1 per s, so it never jumps.
        self.ff_target, self.ff_gain = 0.0, 0.0
        self.ff_height, self.ff_mass, self.ff_com = 0.30, 17.0, (0.0, 0.0)
        self.tau = [0.0] * 12
        # Lean guard (check): degrees of roll/pitch change from the stand that abort.
        self.lean_ref, self.lean_limit = None, 0.0
        self.max_temp, self.n_checks = 68, 0
        self.phase = ""
        self.log, self.log_n = None, 0

    def open_log(self, path):
        """CSV at ~50 Hz: time, phase, roll, pitch (deg), then per joint command and lag."""
        self.log = open(path, "w", newline="")
        self.log_w = csv.writer(self.log)
        names = [f"{leg}_{j}" for leg in LEGS for j in ("hip", "thigh", "calf")]
        self.log_w.writerow(["t", "phase", "kp", "ki", "roll_deg", "pitch_deg"]
                            + [f"{n}_ref" for n in names] + [f"{n}_lag" for n in names]
                            + [f"{n}_corr" for n in names] + [f"{n}_tau" for n in names])
        self.log_t0 = time.monotonic()

    def _log(self, q_cmd):
        self.log_n += 1
        if self.log is None or self.log_n % 10 or self.state is None:
            return
        r, p = self.state.imu_state.rpy[0], self.state.imu_state.rpy[1]
        q = self.q_now()
        ref = self.q_ref if self.q_ref is not None else q_cmd     # lag: reference - actual
        self.log_w.writerow([f"{time.monotonic() - self.log_t0:.3f}", self.phase, self.kp, self.ki,
                             f"{math.degrees(r):.2f}", f"{math.degrees(p):.2f}"]
                            + [f"{v:.4f}" for v in ref] + [f"{c - v:+.4f}" for c, v in zip(ref, q)]
                            + [f"{c:+.4f}" for c in self.i_corr] + [f"{t:+.2f}" for t in self.tau])

    def _on_state(self, msg):
        self.state, self.state_t = msg, time.monotonic()

    def q_now(self):
        return [self.state.motor_state[i].q for i in range(12)]

    def tilt_deg(self):
        r, p = self.state.imu_state.rpy[0], self.state.imu_state.rpy[1]
        return math.degrees(max(abs(r), abs(p)))

    def rp_deg(self):
        return (math.degrees(self.state.imu_state.rpy[0]), math.degrees(self.state.imu_state.rpy[1]))

    def wait_state(self, timeout=3.0):
        end = time.monotonic() + timeout
        while self.state is None and time.monotonic() < end:
            time.sleep(0.01)
        if self.state is None:
            raise RuntimeError("no /lowstate: wrong network interface, or not on the dog?")

    def release_sport(self):
        _, res = self.msc.CheckMode()
        self.mode_name = (res or {}).get("name", "")
        print(f"[crawl] sport mode now: '{self.mode_name or 'released'}'; lying the dog down...")
        self.sport.StandDown()
        time.sleep(3.0)
        end = time.monotonic() + 15.0
        while time.monotonic() < end:
            self.msc.ReleaseMode()
            time.sleep(1.0)
            _, res = self.msc.CheckMode()
            if not (res or {}).get("name"):
                print("[crawl] sport mode released: LOW-LEVEL control, Unitree's balance is OFF")
                return
        raise RuntimeError("sport mode did not release")

    def integrate(self, q_ref):
        """q_ref plus the integral correction, updated from the measured error."""
        if self.state is None:
            return list(q_ref)
        if self.ki <= 0:
            if self.i_fade > 0:
                k = max(0.0, 1.0 - self.DT / self.i_fade)
                self.i_corr = [c * k for c in self.i_corr]
            return [r + c for r, c in zip(q_ref, self.i_corr)]
        q = self.q_now()
        for i in range(12):
            c = self.i_corr[i] + self.ki * (q_ref[i] - q[i]) * self.DT
            self.i_corr[i] = max(-self.I_MAX, min(self.I_MAX, c))
        return [r + c for r, c in zip(q_ref, self.i_corr)]

    def feed_forward(self):
        """Torques for this step: ff_torques of the reference pose times the ramped gain."""
        step = self.DT                                  # gain changes 1 per second at most
        self.ff_gain += max(-step, min(step, self.ff_target - self.ff_gain))
        if self.ff_gain <= 1e-6 or self.q_ref is None:
            self.tau = [0.0] * 12
        else:
            t = ff_torques(self.q_ref, self.ff_height, self.ff_mass, self.ff_com)
            self.tau = [v * self.ff_gain for v in t]
        return self.tau

    def send(self, q, kp, kd):
        tau = self.feed_forward() if kp > 0 else [0.0] * 12
        for i in range(12):
            m = self.cmd.motor_cmd[i]
            m.q, m.dq, m.kp, m.kd, m.tau = q[i], 0.0, kp, kd, tau[i]
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.pub.Write(self.cmd)
        self._log(q)

    def lean(self):
        """Degrees of roll or pitch change since lean_ref (the stand)."""
        r, p = self.rp_deg()
        return max(abs(r - self.lean_ref[0]), abs(p - self.lean_ref[1]))

    def max_motor_temp(self):
        return max(self.state.motor_state[i].temperature for i in range(12))

    def check(self, max_tilt):
        if time.monotonic() - self.state_t > 0.1:
            raise RuntimeError("/lowstate stale > 0.1 s")
        if self.tilt_deg() > max_tilt:
            raise RuntimeError(f"tilt {self.tilt_deg():.0f} deg > {max_tilt:.0f}")
        if self.lean_ref is not None and self.lean_limit > 0 and self.lean() > self.lean_limit:
            raise RuntimeError(f"leaning {self.lean():.1f} deg from the stand (> {self.lean_limit:.0f})")
        self.n_checks += 1
        if self.n_checks % 250 == 0 and self.max_motor_temp() > self.max_temp:
            raise RuntimeError(f"a motor is at {self.max_motor_temp()} C (> {self.max_temp})")

    def ramp(self, q_to, seconds, max_tilt, q_from=None):
        q_from = q_from or self.q_now()
        n = max(1, int(seconds / self.DT))
        for k in range(1, n + 1):
            t0 = time.monotonic()
            self.check(max_tilt)
            s = k / n
            s = s * s * (3 - 2 * s)            # smooth start and stop
            ref = [a + (b - a) * s for a, b in zip(q_from, q_to)]
            self.q_ref = ref
            self.send(self.integrate(ref), self.kp, self.KD)
            time.sleep(max(0.0, self.DT - (time.monotonic() - t0)))
        return list(q_to)

    def hold(self, q, seconds, max_tilt):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            t0 = time.monotonic()
            self.check(max_tilt)
            self.q_ref = q
            self.send(self.integrate(q), self.kp, self.KD)
            time.sleep(max(0.0, self.DT - (time.monotonic() - t0)))

    def safe_stop(self, why, crouch_s=0.8, max_crouch_tilt=35.0):
        """Stop without toppling. Going soft at standing height let the dog fall on its side
        (crawl test 2, roll -111 deg, 2026-11-16), so: while it is not already falling, fold
        the legs into the lying pose quickly WITH stiffness, then go soft."""
        try:
            fresh = self.state is not None and time.monotonic() - self.state_t < 0.1
            if fresh and self.tilt_deg() < max_crouch_tilt:
                print(f"\n[crawl] STOP: {why}: lowering the body, then joints soft")
                self.ki = 0.0
                self.i_fade = crouch_s / 3
                self.ff_target = 0.0
                self.lean_ref = None
                self.phase = "safe stop"
                q_from = self.q_now()
                n = max(1, int(crouch_s / self.DT))
                for k in range(1, n + 1):
                    s = smooth(k / n)
                    ref = [a + (b - a) * s for a, b in zip(q_from, LIE_DOWN)]
                    self.q_ref = ref
                    self.send(self.integrate(ref), self.KP, self.KD)
                    time.sleep(self.DT)
            else:
                print(f"\n[crawl] STOP: {why}: joints soft at once "
                      f"({'no fresh joint data' if not fresh else f'tilt {self.tilt_deg():.0f} deg'})")
        except Exception as ex:  # noqa: BLE001 - the soft stop below must run whatever happened
            print(f"[crawl] crouch failed ({ex}); joints soft")
        self.damp(2.5)

    def damp(self, seconds=2.0):
        """Joints soft (kp 0, damping only): the dog settles slowly onto its belly."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send(self.q_now() if self.state else [0.0] * 12, 0.0, self.DAMP_KD)
            time.sleep(self.DT)

    def hand_back(self):
        name = self.mode_name or "normal"
        print(f"[crawl] handing back to sport mode '{name}'...")
        self.msc.SelectMode(name)
        time.sleep(2.0)
        _, res = self.msc.CheckMode()
        print(f"[crawl] sport mode now: '{(res or {}).get('name', '')}' (stand it with the remote, "
              f"or --restand)")


def run_check(a):
    """Read only: /lowstate rate, joint angles, tilt and the current motion mode."""
    ll = LowLevel(a.iface)
    ll.wait_state()
    n0, t0 = 0, time.monotonic()
    last = ll.state_t
    while time.monotonic() - t0 < 2.0:
        if ll.state_t != last:
            n0, last = n0 + 1, ll.state_t
        time.sleep(0.0005)
    q = ll.q_now()
    print(f"[crawl] /lowstate on {a.iface}: ~{n0 / 2.0:.0f} Hz received; tilt {ll.tilt_deg():.1f} deg")
    for i, leg in enumerate(LEGS):
        print(f"        {leg}: hip {q[3 * i]:+.2f} thigh {q[3 * i + 1]:+.2f} calf {q[3 * i + 2]:+.2f} rad")
    code, res = ll.msc.CheckMode()
    print(f"[crawl] motion mode: code {code}, {res}")
    print(f"[crawl] stand pose at {a.stand_height:.2f} m would be: "
          + " ".join(f"{v:+.2f}" for v in stand_joints(a.stand_height)))


def lift_plan(a, leg):
    """Keyframes for --lift: from the stand, move the body over the other three feet (at
    least lift_margin inside their triangle: the D1 arm on the back moves the real centre
    of mass), lift `leg` by lift_height, hold, lower, re-centre. [(name, s, q12, margin,
    problems)]."""
    plan = Plan(a)
    tri = [plan.feet[o][:2] for o in LEGS if o != leg]
    x0, y0, _ = plan.feet[leg]
    plan.body = shift_target(plan.body, tri, a.lift_margin)
    plan.key(f"body over the other three feet (for {leg})", a.lift_shift_s, leg)
    plan.feet[leg] = (x0, y0, a.lift_height)
    plan.key(f"{leg} up {a.lift_height * 100:.0f} cm", a.lift_up_s, leg)
    plan.key(f"{leg} held up", a.hold, leg)
    plan.feet[leg] = (x0, y0, 0.0)
    plan.key(f"{leg} down", a.lift_up_s, leg)
    plan.body = (0.0, 0.0)
    plan.key("body back over all four feet", a.lift_shift_s, None)
    out = []
    for k in plan.keys[1:]:
        q, problems = plan.joints(k)
        if k["margin"] is not None and k["margin"] < a.lift_margin - 1e-3:
            problems.append(f"margin {k['margin'] * 100:.1f} cm < --lift-margin {a.lift_margin * 100:.1f} cm")
        out.append((k["name"], k["t"], q, k["margin"], problems))
    return out


def leg_over_plan(a, leg):
    """--leg-over: one foot steps over a small object and back, the other three standing.
    The object's near edge is object_distance ahead of that foot. Keyframes as Plan, so it
    runs through the same real-space path (dense) as the crawl: the foot rises to
    object_height + clearance BEFORE it moves forward, and comes down only past the far
    edge. Returns the Plan (its keys) for dense() and the checks."""
    plan = Plan(a)
    x0, y0, _ = plan.feet[leg]
    near = x0 + a.object_distance
    far = near + a.object_depth
    x1 = far + a.landing_margin + FOOT_R                 # land clear of the far edge
    lift = a.object_height + a.clearance
    plan.obj_near, plan.obj_far = near, far
    tri = [plan.feet[o][:2] for o in LEGS if o != leg]
    plan.body = shift_target(plan.body, tri, a.lift_margin)
    plan.key(f"body over the other three feet (for {leg})", a.lift_shift_s, leg)
    plan.key("settle", a.t_settle or 1.0, leg)
    plan.feet[leg] = (x0, y0, lift)
    plan.key(f"{leg} up {lift * 100:.0f} cm", a.lift_up_s, leg)
    plan.feet[leg] = (x1, y0, lift)
    plan.key(f"{leg} forward {(x1 - x0) * 100:.0f} cm over the object", a.t_move, leg)
    plan.feet[leg] = (x1, y0, 0.0)
    plan.key(f"{leg} down beyond the object", a.lift_up_s, leg)
    plan.key(f"{leg} held beyond the object", a.hold, leg)
    plan.feet[leg] = (x1, y0, lift)
    plan.key(f"{leg} up again", a.lift_up_s, leg)
    plan.feet[leg] = (x0, y0, lift)
    plan.key(f"{leg} back over the object", a.t_move, leg)
    plan.feet[leg] = (x0, y0, 0.0)
    plan.key(f"{leg} down where it started", a.lift_up_s, leg)
    plan.body = (0.0, 0.0)
    plan.key("body back over all four feet", a.lift_shift_s, None)
    return plan


def check_leg_over(a, plan):
    """Print and count problems for --leg-over: joint limits and reach at every 20 ms
    sample, stability while the foot is up, and the foot's clearance over the object."""
    print(f"[crawl] --leg-over {a.leg_over}: object {a.object_height * 100:.0f} cm high, "
          f"{a.object_depth * 100:.0f} cm deep, {a.object_distance * 100:.0f} cm ahead of the foot; "
          f"stand {a.stand_height:.2f} m")
    bad = 0
    for k in plan.keys[1:]:
        q, problems = plan.joints(k)
        m = k["margin"]
        if m is not None and m < a.lift_margin - 1e-3:
            problems.append(f"margin {m * 100:.1f} cm < {a.lift_margin * 100:.1f} cm")
        ms = f"{m * 100:5.1f}cm" if m is not None else "   -   "
        print(f"  {k['t']:4.1f}s  {k['name']:<42} {ms}  " + " ".join(f"{v:+.2f}" for v in q))
        for p in problems:
            print(f"        !! {p}")
        bad += len(problems)
    samples = dense(plan)
    sb = sum(len(s[3]) for s in samples)
    clear = over_object_clearance(plan, samples)
    print(f"[crawl] {len(samples)} samples: {sb} problems; foot over the object at least "
          f"{'-' if clear is None else f'{clear * 100:.1f} cm above its top'}")
    if clear is None or clear < a.clearance / 2:
        bad += 1
        print("        !! the foot does not clear the object")
    print(f"[crawl] problems: {bad + sb}")
    return bad + sb


def print_lift(a, keys):
    print(f"[crawl] --lift {a.lift}: stand {a.stand_height:.2f} m, foot up {a.lift_height * 100:.0f} cm, "
          f"body at least {a.lift_margin * 100:.1f} cm inside the other three feet")
    bad = 0
    for name, t, q, m, problems in keys:
        ms = f"{m * 100:5.1f}cm" if m is not None else "   -   "
        print(f"  {t:4.1f}s  {name:<42} {ms}  " + " ".join(f"{v:+.2f}" for v in q))
        for p in problems:
            print(f"        !! {p}")
        bad += len(problems)
    print(f"[crawl] problems: {bad}")
    return bad


def run_stand(a):
    q_stand = stand_joints(a.stand_height, getattr(a, "stance_out", 0.0))
    for leg_i, leg in enumerate(LEGS):
        bad = limit_violations(leg, q_stand[3 * leg_i: 3 * leg_i + 3])
        if bad:
            sys.exit(f"[crawl] stand pose outside the joint limits: {bad}")
    lift = lift_plan(a, a.lift) if a.lift else []
    if a.leg_over:
        lo_plan = leg_over_plan(a, a.leg_over)
        if check_leg_over(a, lo_plan):
            sys.exit("[crawl] the leg-over plan has problems: not started")
    if a.lift and print_lift(a, lift):
        sys.exit("[crawl] the lift plan has problems: not started")
    crawl = dense(lo_plan) if a.leg_over else []
    if a.step_over:
        plan = Plan(a).build()
        if print_plan(plan):
            sys.exit("[crawl] the crawl plan has problems: not started")
        crawl = dense(plan)
        last = len(plan.keys) - 1
        if a.stop_after:
            # Stop only with every foot on the floor: at the next "down" or body re-centre.
            last = a.stop_after
            while last < len(plan.keys) - 1 and not (plan.keys[last]["name"].endswith(" down")
                                                     or plan.keys[last]["swing"] is None):
                last += 1
            crawl = [c for c in crawl if c[0] <= last]
        print(f"[crawl] running keyframes 1-{last} of {len(plan.keys) - 1} "
              f"({sum(plan.keys[i]['t'] for i in range(1, last + 1)):.0f} s)")
    what = (("lift " + a.lift) if a.lift else ("leg-over " + a.leg_over) if a.leg_over
            else ("step-over crawl" if a.step_over else "stand"))
    print(f"[crawl] --{what}: sport mode OFF, stand on joint control at {a.stand_height:.2f} m, "
          f"{what}, lie down, sport mode back.")
    print("        Dog on the floor, 1 m clear around it, someone on the remote; a harness or")
    print("        a hand on it the first time. Ctrl+C at any time: joints go soft, the dog settles.")
    if not a.yes and input("Type 'yes' to start: ").strip().lower() != "yes":
        sys.exit("[crawl] not started")
    ll = LowLevel(a.iface)
    ll.wait_state()
    hot = ll.max_motor_temp()
    if hot > a.start_temp:
        sys.exit(f"[crawl] a motor is at {hot} C (> {a.start_temp}): let the dog cool first, not started")
    ll.max_temp = a.max_temp
    if a.log:
        ll.open_log(a.log)
        print(f"[crawl] logging roll, pitch and joint lag to {a.log}")
    released = False
    try:
        ll.release_sport()
        released = True
        q = ll.q_now()
        print(f"[crawl] lying pose read; standing up ({a.ramp:.1f} s)...")
        q = ll.ramp(LIE_DOWN, a.ramp, a.max_tilt, q_from=q)
        q = ll.ramp(q_stand, a.ramp, a.max_tilt, q_from=q)
        ll.ki = a.ki          # integral correction from the stand on (0: off)
        ll.ff_height, ll.ff_mass, ll.ff_com = a.stand_height, a.mass, (a.com_x, a.com_y)
        ll.ff_target = a.ff        # feed-forward ramps in over the hold below
        ll.phase = "stand (feed-forward ramping in)"
        print(f"[crawl] standing on joint control; holding {a.hold:.0f} s "
              f"(tilt {ll.tilt_deg():.1f} deg)")
        ll.hold(q, a.hold if not (lift or crawl) else 2.0, a.max_tilt)
        r0, p0 = ll.rp_deg()
        ll.lean_ref, ll.lean_limit = (r0, p0), a.lean_abort   # abort (lower the body) past this
        print(f"[crawl] feed-forward {ll.ff_gain:.2f} of {a.mass:.0f} kg; lean abort at {a.lean_abort:.0f} deg; "
              f"motors at most {ll.max_motor_temp()} C")
        if crawl:
            ll.kp, shown = a.kp_lift, None

            for i, name, q_k, _p, _f in crawl:
                if i != shown:
                    shown = i
                    ll.phase = f"{i} {name}"
                    r, p = ll.rp_deg()
                    print(f"[crawl] {i:3d} {name:<38} roll {r - r0:+5.1f} pitch {p - p0:+5.1f} deg "
                          "(change since the stand)")
                ll.hold(q_k, 0.02, a.max_tilt)
                q = q_k
            r, p = ll.rp_deg()
            print(f"[crawl] crawl done: roll {r - r0:+.1f} pitch {p - p0:+.1f} deg since the stand")
            ll.phase, ll.kp = "settle", ll.KP
            ll.hold(q, 1.0, a.max_tilt)
        for name, t, q_k, _m, _p in lift:
            ll.phase = name
            if name.startswith("body over"):
                ll.kp = a.kp_lift
            r, p = ll.rp_deg()
            print(f"[crawl] {name} ({t:.1f} s, kp {ll.kp:.0f}); roll {r:+.1f} pitch {p:+.1f} deg "
                  f"(change since the stand: roll {r - r0:+.1f}, pitch {p - p0:+.1f})")
            q = ll.ramp(q_k, t, a.max_tilt, q_from=q) if "held" not in name else q
            if "held" in name:
                ll.hold(q, t, a.max_tilt)
        if lift:
            r, p = ll.rp_deg()
            print(f"[crawl] after the lift: roll {r:+.1f} pitch {p:+.1f} deg "
                  f"(change: roll {r - r0:+.1f}, pitch {p - p0:+.1f})")
            ll.phase, ll.kp = "settle", ll.KP
            ll.hold(q, 1.0, a.max_tilt)
        print("[crawl] lying down...")
        ll.ki, ll.i_fade = 0.0, a.ramp / 3          # the correction fades as the dog lies down
        ll.ff_target, ll.lean_ref = 0.0, None       # feed-forward fades too; no lean abort lying down
        ll.phase = "lying down"
        q = ll.ramp(LIE_DOWN, a.ramp, a.max_tilt, q_from=q)
        ll.damp(1.0)
        print("[crawl] done")
    except (KeyboardInterrupt, RuntimeError) as ex:
        ll.safe_stop(f"{ex or 'Ctrl+C'}")
    finally:
        if ll.log is not None:
            ll.log.close()
        if released:
            ll.hand_back()
            if a.restand or a.resume_session:
                ll.sport.StandUp()
                time.sleep(1.5)
                ll.sport.BalanceStand()
            if a.resume_session:
                # Back to how the navigation session's sport bridge left the dog: ClassicWalk,
                # the remote ignored (wifi_3d_nav.sh --semantic --step-over-crawl).
                time.sleep(1.0)
                ll.sport.ClassicWalk(True)
                time.sleep(0.3)
                ll.sport.SwitchJoystick(False)
                print("[crawl] session resumed: standing, ClassicWalk, remote ignored")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--plan", action="store_true", help="print and check the crawl; no robot")
    mode.add_argument("--check", action="store_true", help="ON THE DOG, read only: joint states and mode")
    mode.add_argument("--stand", action="store_true", help="ON THE DOG: release sport mode, stand, give it back")
    mode.add_argument("--lift", choices=LEGS, help="ON THE DOG: stand, shift, lift this foot, back (--dry: print)")
    mode.add_argument("--leg-over", choices=LEGS,
                      help="ON THE DOG: this foot steps over a small object and back (--dry: print)")
    mode.add_argument("--step-over", action="store_true",
                      help="ON THE DOG: the crawl over the object (--stop-after N: first N keyframes)")
    ap.add_argument("--object-height", type=float, default=0.08, help="m")
    ap.add_argument("--object-depth", type=float, default=0.10, help="m, along the walking direction")
    ap.add_argument("--object-distance", type=float, default=0.12, help="m from the front feet to its near edge")
    ap.add_argument("--stand-height", type=float, default=0.30,
                    help="m, hip joint to foot centre (taller cannot reach the crossing step)")
    ap.add_argument("--clearance", type=float, default=0.05, help="m above the object at the top of the lift")
    ap.add_argument("--normal-lift", type=float, default=0.06, help="m, steps that do not cross the object")
    ap.add_argument("--stride", type=float, default=0.12, help="m per step")
    ap.add_argument("--max-stride", type=float, default=0.26, help="m, the longest step over the object")
    ap.add_argument("--landing-margin", type=float, default=0.04, help="m kept between a foot and the object")
    ap.add_argument("--stability-margin", type=float, default=0.05,
                    help="m the body centre must be inside the standing triangle; the body shifts "
                         "1.5x this before each swing (7.5 cm: the margin that held level in --lift)")
    ap.add_argument("--order", default="RL,FL,RR,FR", help="swing order (crawl: hind, fore, other hind, fore)")
    ap.add_argument("--max-cycles", type=int, default=8)
    ap.add_argument("--recentre-every", type=int, default=2, help="re-centre the body after this many steps")
    # Phase times (s): the lift tests' pace, which held level; the first crawl test at
    # 1.5 / 0.8 / 1.0 / 0.8 s tipped at FR.
    ap.add_argument("--t-shift", type=float, default=3.0)
    ap.add_argument("--t-settle", type=float, default=1.0, help="s held after each body shift")
    ap.add_argument("--t-lift", type=float, default=1.5)
    ap.add_argument("--t-move", type=float, default=2.0)
    ap.add_argument("--t-lower", type=float, default=1.5)
    ap.add_argument("--stance-out", type=float, default=0.0,
                    help="m each foot stands outward of the hip: bigger support triangles")
    ap.add_argument("--lean-abort", type=float, default=6.0,
                    help="deg of roll/pitch change from the stand: lower the body and stop (a hold at "
                         "6 deg let crawl test 3 reach 15 and fall)")
    ap.add_argument("--ff", type=float, default=0.8,
                    help="feed-forward gain: share of the weight torque given up front (0: off)")
    ap.add_argument("--mass", type=float, default=17.0, help="kg, the dog with the D1 arm")
    ap.add_argument("--com-x", type=float, default=0.0, help="m, centre of mass ahead of the body centre")
    ap.add_argument("--com-y", type=float, default=0.0, help="m, centre of mass left of the body centre")
    ap.add_argument("--start-temp", type=int, default=55, help="C: refuse to start above this")
    ap.add_argument("--max-temp", type=int, default=68, help="C: stop (lower the body) above this")
    ap.add_argument("--csv", default="", help="--plan: write the keyframes here")
    ap.add_argument("--iface", default="eth0", help="--stand: the dog's internal network interface")
    ap.add_argument("--hold", type=float, default=5.0, help="--stand: s standing on joint control")
    ap.add_argument("--ramp", type=float, default=2.5, help="--stand: s per pose change")
    ap.add_argument("--max-tilt", type=float, default=12.0, help="deg absolute tilt: stop (lower the body)")
    ap.add_argument("--restand", action="store_true", help="--stand: StandUp + BalanceStand in sport mode after")
    ap.add_argument("--yes", action="store_true", help="--stand: skip the typed confirmation")
    ap.add_argument("--resume-session", action="store_true",
                    help="after: stand up in sport mode, ClassicWalk, remote ignored (the nav session's state)")
    ap.add_argument("--lift-height", type=float, default=0.10, help="--lift: m the foot rises")
    ap.add_argument("--lift-margin", type=float, default=0.05,
                    help="--lift: m the body goes inside the other three feet (the arm moves the CoM)")
    ap.add_argument("--lift-shift-s", type=float, default=3.0, help="--lift: s to move the body")
    ap.add_argument("--lift-up-s", type=float, default=2.0, help="--lift: s to raise or lower the foot")
    ap.add_argument("--dry", action="store_true", help="--lift: print the poses, move nothing")
    ap.add_argument("--kp-lift", type=float, default=80.0,
                    help="--lift: joint stiffness from the body shift on (the stand uses 60)")
    ap.add_argument("--ki", type=float, default=1.0,
                    help="--stand/--lift: integral correction, rad/s per rad of error (0: pure PD)")
    ap.add_argument("--stop-after", type=int, default=0,
                    help="--step-over: run keyframes 1..N (rounded up to feet-down), then lie down")
    ap.add_argument("--log", default="", help="--stand/--lift: CSV of roll, pitch, joint commands and lag")
    a = ap.parse_args()
    if a.plan:
        return 1 if print_plan(Plan(a).build(), a.csv) else 0
    if a.check:
        run_check(a)
        return 0
    if a.leg_over and a.dry:
        return 1 if check_leg_over(a, leg_over_plan(a, a.leg_over)) else 0
    if a.lift and a.dry:
        return 1 if print_lift(a, lift_plan(a, a.lift)) else 0
    run_stand(a)
    return 0


if __name__ == "__main__":
    sys.exit(main())
