#!/usr/bin/env python3
"""
orbit_reach_check.py: reach-only azimuth limits of the orbit (no collisions), observed by IK
and predicted analytically, for the R(theta) comparison of the orbit frontier.

  python3 scripts/tools/orbit_reach_check.py --urdf D1_URDF --h-p 0.844 --out SWEEP/reach.json

Per cell (d, r) and shell (eq 0, top +30, under -20 deg elevation):
  observed   damped least-squares IK of the camera pose (orbit_waypoints.camera_pose's
             geometry), joint limits only, continued in azimuth from 0 (5 deg steps, then 1 deg):
             the last feasible azimuth (no collision checking: the arm's reach alone)
  planar     R(theta) = sqrt((d - r cos)^2 + (r sin)^2) <= 0.605 m (the brief's equation)
  wrist      the same reach test at the wrist centre and in 3D: W = C - a x_c - b z_c (the camera
             at a = 0.163 m ahead of and b = 0.054 m above the Joint5 axis, x_c the viewing
             direction, z_c the camera's up), |W - S| <= L with S the Joint2 axis
             (0.4796 m up) and L = |J2->J3| + |J3->J5| = 0.479 m (upper arm + forearm, URDF)
Writes the limits per cell and shell to --out.
"""
import argparse
import itertools
import json
import math
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
SHELLS = {"eq": 0.0, "top": 30.0, "under": -20.0}
S_Z, L_ARM, CAM_A, CAM_B, REACH = 0.4796, 0.479, 0.163, 0.054, 0.605
_K = {}


def kin(urdf):
    if "k" not in _K:
        src = open(os.path.join(REPO, "src/d1_moveit_config/scripts/full_mission_test.py")).read()
        src = src.split("# ------------------------------------------------------------------ Gazebo side")[0]
        ns = {"__name__": "fm"}
        exec(compile(src, "fm", "exec"), ns)
        _K["ns"], _K["k"] = ns, ns["Kinematics"](urdf)
        lim = _K["k"].limits(urdf)
        J = ns["ARM_JOINTS"]
        _K["lo"] = np.array([lim[j][0] for j in J]) * 0.98
        _K["hi"] = np.array([lim[j][1] for j in J]) * 0.98
    return _K


def cam(d, r, h_p, az, el):
    a, e = math.radians(az), math.radians(el)
    P = np.array([d, 0.0, h_p])
    C = P + r * np.array([-math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
    x = (P - C) / np.linalg.norm(P - C)
    z = np.array([0, 0, 1.0]) - x[2] * x
    z /= np.linalg.norm(z)
    return C, np.column_stack([x, np.cross(z, x), z])


def rot_error(R_target, R_now):
    """Axis-angle rotation vector taking R_now to R_target (world frame), and its angle. A
    sum of axis cross products, the usual shortcut, is zero for every 180 deg rotation as
    well, which let the IK converge on a camera turned the wrong way."""
    Re = R_target @ R_now.T
    c = max(-1.0, min(1.0, (np.trace(Re) - 1.0) / 2.0))
    w = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
    s = np.linalg.norm(w)
    ang = math.atan2(s, c)
    if s > 1e-9:
        return w / s * ang, ang
    if c > 0:
        return np.zeros(3), 0.0
    k = int(np.argmax(np.diag(Re)))                  # 180 deg: the axis from the diagonal
    axis = np.sqrt(np.clip((np.diag(Re) + 1.0) / 2.0, 0.0, None))
    axis[k] = max(axis[k], 1e-9)
    for i in range(3):
        if i != k:
            axis[i] = math.copysign(axis[i], Re[k, i])
    return axis / np.linalg.norm(axis) * math.pi, math.pi


def dls(K, C, R, seeds, iters=100):
    k, ns = K["k"], K["ns"]
    J = ns["ARM_JOINTS"]

    def err(q):
        p, Rq = k.pose(dict(zip(J, q)))
        dr = rot_error(R, Rq)[0]
        return np.concatenate([C - p, 0.3 * dr])
    for s0 in seeds:
        q = np.clip(np.array(s0, float), K["lo"], K["hi"])
        for _ in range(iters):
            e = err(q)
            if np.linalg.norm(e[:3]) < 5e-4 and np.linalg.norm(e[3:]) < 3e-3:
                break
            Jm = np.empty((6, 6))
            for i in range(6):
                dq = np.zeros(6)
                dq[i] = 1e-5
                Jm[:, i] = (err(q + dq) - e) / -1e-5
            q = np.clip(q + Jm.T @ np.linalg.solve(Jm @ Jm.T + 1e-4 * np.eye(6), e), K["lo"], K["hi"])
        p_now, R_now = k.pose(dict(zip(J, q)))
        if np.linalg.norm(C - p_now) < 1e-3 and rot_error(R, R_now)[1] < 0.02:
            return q
    return None


def observed(args):
    urdf, d, r, h_p, shell = args
    K = kin(urdf)
    rng = np.random.default_rng(5)
    seeds = [np.array([0, 0.25, 0.30, 0, -0.55, 0]), np.zeros(6)] + [rng.uniform(K["lo"], K["hi"]) for _ in range(10)]
    el = SHELLS[shell]
    out = {}
    for sign in (1, -1):
        q, last, fail = None, None, None
        for k in range(0, 181, 5):
            C, R = cam(d, r, h_p, sign * k, el)
            s = dls(K, C, R, ([q] if q is not None else []) + seeds)
            if s is None:
                fail = k
                break
            q, last = s, k
        if fail is not None and last is not None:
            for k in range(last + 1, fail):
                C, R = cam(d, r, h_p, sign * k, el)
                s = dls(K, C, R, [q] + seeds)
                if s is None:
                    break
                q, last = s, k
        out["left" if sign > 0 else "right"] = last
    vals = [v for v in out.values()]
    return (d, r, shell, None if None in vals else min(vals))


def envelope(args):
    """(d, r, [reachable elevations at az 0]): the camera's elevation range over the plant,
    1 deg steps from -40 to +40, reach only (joint limits, no collisions)."""
    urdf, d, r, h_p = args
    K = kin(urdf)
    rng = np.random.default_rng(7)
    seeds = [np.array([0, 0.25, 0.30, 0, -0.55, 0]), np.zeros(6)] + [rng.uniform(K["lo"], K["hi"]) for _ in range(16)]
    ok, q = [], None
    for el in range(-40, 41):
        C, R = cam(d, r, h_p, 0, el)
        s = dls(K, C, R, ([q] if q is not None else []) + seeds)
        if s is not None:
            ok.append(el)
            q = s
    return d, r, ok


def planar(d, r):
    c = (d * d + r * r - REACH * REACH) / (2 * d * r)
    return 180.0 if c <= -1 else (0.0 if c >= 1 else math.degrees(math.acos(c)))


def wrist(d, r, h_p, el):
    best = None
    for k in range(0, 181):
        C, R = cam(d, r, h_p, k, el)
        W = C - CAM_A * R[:, 0] - CAM_B * R[:, 2]
        if np.linalg.norm(W - np.array([0, 0, S_Z])) <= L_ARM:
            best = k
        else:
            break
    return best


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--h-p", type=float, default=0.844)
    ap.add_argument("--ds", type=float, nargs="+", default=[0.35, 0.40, 0.45, 0.50])
    ap.add_argument("--rs", type=float, nargs="+", default=[0.25, 0.30, 0.35, 0.40])
    ap.add_argument("--out", required=True)
    ap.add_argument("--jobs", type=int, default=12)
    ap.add_argument("--envelope", action="store_true", help="also the reachable elevations at az 0 per cell")
    a = ap.parse_args()
    jobs = [(a.urdf, d, r, a.h_p, s) for d, r, s in itertools.product(a.ds, a.rs, SHELLS)]
    with Pool(a.jobs) as pool:
        res = pool.map(observed, jobs)
    out = {}
    for d, r, shell, lim in res:
        key = f"d{d:.2f}_r{r:.2f}"
        cell = out.setdefault(key, {"d": d, "r": r, "planar_deg": planar(d, r), "shells": {}})
        cell["shells"][shell] = {"observed_reach_deg": lim, "wrist_analytic_deg": wrist(d, r, a.h_p, SHELLS[shell])}
    if a.envelope:
        with Pool(a.jobs) as pool:
            env = pool.map(envelope, [(a.urdf, d, r, a.h_p) for d, r in itertools.product(a.ds, a.rs)])
        for d, r, els in env:
            out[f"d{d:.2f}_r{r:.2f}"]["elevations_reachable_az0"] = els
            print(f"d{d:.2f}_r{r:.2f} az 0 reachable elevations:", f"{min(els)}..{max(els)} deg" if els else "none",
                  f"({len(els)} of 81)" if els else "")
    json.dump(out, open(a.out, "w"), indent=1)
    for key, c in sorted(out.items()):
        print(key, f"planar {c['planar_deg']:.1f}", {s: (v["observed_reach_deg"], v["wrist_analytic_deg"]) for s, v in c["shells"].items()})


if __name__ == "__main__":
    sys.exit(main())
