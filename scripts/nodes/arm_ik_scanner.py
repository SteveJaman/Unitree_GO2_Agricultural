#!/usr/bin/env python3
"""
arm_ik_scanner.py

Point the D1 arm's wrist camera at a target and sweep across it, with inverse
kinematics instead of hardcoded angles (the dynamic counterpart of
orbit_scan.py).

The target is given in the Go2's base_link frame (metres: x forward, y left,
z up), e.g. by inspection_mission.py, which turns the 3D point clicked in the
dashboard into base_link once the dog has arrived:

    1. Load the D1 URDF with ikpy (chain base_link -> Link6, plus a fixed
       camera point CAMERA_OFFSET along Link6 +z, the gripper axis).
    2. Aim points: the target, then the target at y +/- SIDE (look left and
       right of it), in STEPS from left to right.
    3. For each, solve for joints 0-5 so the camera's axis passes through the
       aim point, from spots above and in front of it looking down (or on
       the shoulder -> target line for distant targets). Several start guesses
       and spots are tried; the best
       aims within MAX_AIM_ERR_DEG and moves least from the previous step.
       Solutions at a joint limit (the solver giving up) or with a link inside
       the Go2's body box are refused, and if no pose is left nothing moves.
    4. Power on, enable, home, then each pose as one multi-joint move
       (funcode 2, mode 1: the arm's smoothed trajectory) with STEP_DELAY s
       between steps, then home and release.

Like orbit_scan.py, a try/finally homes the arm and always releases it
(mode 0) on Ctrl+C, an error or a lost link.

Joint mapping: protocol angle i (degrees) = URDF Joint(i+1) (radians), as in
gazebo/sim/d1_arm_emulator.py. That mapping comes from the simulator; check
the aim on the real arm with --dry-run and a slow first run.

Needs ikpy (pip3 install --user ikpy). Same links as start_arm.sh: wired DDS
(--iface, auto-detected) or the Go2 bridge (--robot / ARM_ROBOT).

Usage:
    python3 arm_ik_scanner.py --target 0.55 0.0 0.0 --dry-run   # solve + print, no arm
    ./scripts/runtime/start_arm.sh scan --target 0.55 0.0 0.0     # preflight, confirm, run
"""

import argparse
import os
import sys
import time
import warnings

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from d1_arm_control import (  # noqa: E402
    D1Arm,
    ENABLE_FULL,
    JOINT_LIMITS,
    T_ENABLE,
    T_HOME,
    T_RELEASE,
    default_arm_iface,
    preflight,
    print_state,
)
from orbit_scan import current, move_to  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
URDF = os.path.join(
    REPO, "twin", "webapp", "public", "d1_description", "urdf", "d1_description.urdf"
)

MOUNT = np.array(
    [0.0, 0.0, 0.057]
)  # D1 base on the Go2 trunk (as Scene.jsx / gazebo --mount)
SHOULDER = np.array([0.0, 0.0, 0.11])  # roughly Joint2, in the arm's base frame
CAMERA_OFFSET = 0.09  # camera along Link6 +z (the dashboard's cone starts here)
REACH = (0.22, 0.42)  # m from the shoulder to the camera (distant targets)
ELEVATIONS = (45, 30, 60, 15, 75)  # deg above the target the camera looks down from
VIEW_DIST = (0.30, 0.22, 0.38)  # m from the camera to the target
MIN_CAM_DIST = 0.15  # m: closer and the camera (and wrist) nearly touch the object
LIMIT_MARGIN_DEG = 2.0  # a solution this close to a joint limit is the solver giving up
MAX_AIM_ERR_DEG = 3.0
SIDE, STEPS = 0.10, 5  # look +/- SIDE m left/right of the target, STEPS aim points
STEP_DELAY = 0.5  # s between sweep steps
# The Go2 in its base_link frame (trunk + head + legs, small margin): no arm link inside.
BODY_BOX = ((-0.38, 0.36), (-0.15, 0.15), (-0.40, 0.08))


def load_chain(urdf=URDF):
    """ikpy chain base_link -> Link6 -> fixed camera point; 6 active joints."""
    from ikpy.chain import Chain
    from ikpy.link import URDFLink

    warnings.filterwarnings("ignore", module="ikpy")
    elems = ["base_link"] + sum([[f"Joint{i}", f"Link{i}"] for i in range(1, 7)], [])
    full = Chain.from_urdf_file(urdf, base_elements=elems)
    cam = URDFLink(
        name="camera",
        origin_translation=[0, 0, CAMERA_OFFSET],
        origin_orientation=[0, 0, 0],
        rotation=None,
        joint_type="fixed",
    )
    return Chain(
        full.links[:7] + [cam], active_links_mask=[False] + [True] * 6 + [False]
    )


def aim_error(chain, q, target):
    """(degrees off, camera-to-target distance) for joints q aimed at target (arm frame)."""
    fk = chain.forward_kinematics(q)
    cam, axis = fk[:3, 3], fk[:3, 2]
    to = target - cam
    dist = float(np.linalg.norm(to))
    if dist < 1e-6:
        return 180.0, 0.0
    cosang = float(np.clip(axis @ (to / dist), -1.0, 1.0))
    return float(np.degrees(np.arccos(cosang))), dist


def hits_body(chain, q):
    """True if a link origin from Link2 to the camera falls inside BODY_BOX."""
    for frame in chain.forward_kinematics(q, full_kinematics=True)[2:]:
        p = frame[:3, 3] + MOUNT
        if all(lo <= v <= hi for v, (lo, hi) in zip(p, BODY_BOX)):
            return True
    return False


def at_limit(chain, q):
    """True if a joint sits within LIMIT_MARGIN_DEG of its URDF limit."""
    m = np.radians(LIMIT_MARGIN_DEG)
    return any(
        v <= lo + m or v >= hi - m
        for v, (lo, hi) in ((q[i], chain.links[i].bounds) for i in range(1, 7))
    )


def camera_spots(t):
    """Candidate (camera position, viewing direction) to look at t (arm frame).

    Mostly above and in front of the target, looking down at it from the arm's
    side (ELEVATIONS x VIEW_DIST): a table-top object is seen from above, not
    from the shoulder through the dog's head. For distant targets, also spots
    on the shoulder -> target line at a reachable radius (REACH)."""
    back = -t[:2] / (
        np.linalg.norm(t[:2]) or 1.0
    )  # from the target towards the arm, horizontal
    spots = []
    for el in np.radians(ELEVATIONS):
        for d in VIEW_DIST:
            off = np.array([back[0] * np.cos(el), back[1] * np.cos(el), np.sin(el)]) * d
            spots.append((t + off, -off / d))
    u = t - SHOULDER
    span = float(np.linalg.norm(u))
    u /= span
    for r in np.linspace(*REACH, 3):
        if span - r >= MIN_CAM_DIST:
            spots.append((SHOULDER + u * r, u))
    return spots


def solve(chain, target_go2, seeds, prev=None, hint=None):
    """Joints (radians, chain layout) aiming the camera at target_go2.

    Returns (q, err, dist, spot index); q is None if nothing is feasible.
    `hint` (the previous step's spot) is tried first from `prev`: neighbouring
    sweep steps then reuse one arm posture instead of searching again."""
    t = np.asarray(target_go2, float) - MOUNT
    spots = camera_spots(t)
    best, best_cost = (None, 180.0, 0.0, None), float("inf")

    def attempt(i, seed):
        nonlocal best, best_cost
        pos, u = spots[i]
        q = chain.inverse_kinematics(
            pos, target_orientation=u, orientation_mode="Z", initial_position=seed
        )
        err, dist = aim_error(chain, q, t)
        if (
            err > MAX_AIM_ERR_DEG
            or dist < MIN_CAM_DIST
            or at_limit(chain, q)
            or hits_body(chain, q)
        ):
            return False
        # Aim first, then the smallest move from the previous pose (no thrashing).
        cost = err + (
            0.02 * float(np.degrees(np.abs(q[1:7] - prev[1:7])).sum())
            if prev is not None
            else 0.0
        )
        if cost < best_cost:
            best, best_cost = (q, err, dist, i), cost
        return True

    if hint is not None and prev is not None and attempt(hint, prev) and best[1] < 1.0:
        return best
    for seed in seeds:
        for i in range(len(spots)):
            attempt(i, seed)
    return best


def plan(target, side=SIDE, steps=STEPS, chain=None):
    """[(aim point, protocol degrees joints 0-5, aim error, distance)] centre, left..right, centre."""
    chain = chain or load_chain()
    x, y, z = target
    aims = (
        [(x, y, z)]
        + [(x, y + dy, z) for dy in np.linspace(side, -side, steps)]
        + [(x, y, z)]
    )
    seeds = [
        np.zeros(8),
        np.radians([0, 0, 45, 60, 0, -60, 0, 0]),
        np.radians([0, 0, -45, 60, 0, 60, 0, 0]),
    ]
    out, prev, hint = [], None, None
    for a in aims:
        q, err, dist, hint = solve(
            chain, a, ([prev] if prev is not None else []) + seeds, prev, hint
        )
        if q is None or err > MAX_AIM_ERR_DEG:
            raise ValueError(
                f"cannot aim the camera at ({a[0]:.2f}, {a[1]:.2f}, {a[2]:.2f}) m: "
                "no pose within the joint limits that clears the Go2's body "
                "(too close, behind or below the dog?)"
            )
        deg = [float(v) for v in np.degrees(q[1:7])]
        bad = [
            f"joint {j} {v:+.1f}" for j, v in enumerate(deg) if abs(v) > JOINT_LIMITS[j]
        ]
        if bad:
            raise ValueError(f"solution beyond the joint limits: {', '.join(bad)}")
        out.append((a, deg, err, dist))
        prev = q
    return out


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--target",
        type=float,
        nargs=3,
        required=True,
        metavar=("DX", "DY", "DZ"),
        help="target in the Go2's base_link frame, metres",
    )
    ap.add_argument(
        "--side",
        type=float,
        default=SIDE,
        help="look this far left/right of the target, m",
    )
    ap.add_argument(
        "--steps", type=int, default=STEPS, help="aim points from left to right"
    )
    ap.add_argument(
        "--delay", type=float, default=STEP_DELAY, help="s between sweep steps"
    )
    ap.add_argument("--iface", default=default_arm_iface())
    ap.add_argument(
        "--domain", type=int, default=int(os.environ.get("ARM_DOMAIN", "0"))
    )
    ap.add_argument("--ip", default=os.environ.get("ARM_IP", "192.168.123.100"))
    ap.add_argument(
        "--robot",
        default=os.environ.get("ARM_ROBOT"),
        help="Go2 Wi-Fi IP (arm on the Go2)",
    )
    ap.add_argument(
        "--tolerance", type=float, default=3.0, help="degrees counted as reached"
    )
    ap.add_argument("--move-timeout", type=float, default=15.0)
    ap.add_argument("--yes", action="store_true", help="do not ask before moving")
    ap.add_argument(
        "--dry-run", action="store_true", help="solve and print, do not connect"
    )
    ap.add_argument("--skip-preflight", action="store_true")
    args = ap.parse_args()

    try:
        poses = plan(args.target, args.side, max(1, args.steps))
    except ImportError:
        print("[!] ikpy is missing: pip3 install --user ikpy")
        return 2
    except ValueError as exc:
        print(f"[!] {exc}; nothing moved")
        return 4
    print(
        f"[ik-scan] target ({args.target[0]:+.2f}, {args.target[1]:+.2f}, {args.target[2]:+.2f}) m "
        f"in base_link, {len(poses)} poses"
    )
    for n, (a, deg, err, dist) in enumerate(poses, 1):
        print(
            f"  {n}: aim y {a[1]:+.2f}  joints {' '.join(f'{v:+6.1f}' for v in deg)}  "
            f"({err:.1f} deg off, camera {dist:.2f} m away)"
        )
    if args.dry_run:
        return 0

    if args.robot:
        os.environ["ARM_ROBOT"] = args.robot
    if not args.yes:
        try:
            if (
                input(
                    "The arm will aim at the target and sweep across it. Continue? [y/N] "
                )
                .strip()
                .lower()
                != "y"
            ):
                print("aborted")
                return 0
        except EOFError:
            print("aborted")
            return 0
    if not args.skip_preflight and not preflight(args.iface, args.ip):
        print("\n[!] Preflight failed")
        return 2
    arm = D1Arm(args.iface, args.domain)
    if not arm.ok:
        print("[!] cannot reach the arm (DDS discovery / Go2 bridge failed)")
        return 2

    enabled = False
    try:
        arm.power(True)
        time.sleep(1.5)
        arm.enable(ENABLE_FULL)
        time.sleep(T_ENABLE)
        if print_state(arm, "after-enable").get("enable_status") != 1:
            print("[!] the arm did not enable; nothing moved")
            return 3
        enabled = True
        print("[ik-scan] home")
        arm.home()
        time.sleep(T_HOME)
        gripper = (current(arm) or [0.0] * 7)[6]
        # First pose: a long move, wait for it. Then the sweep, one step per delay.
        if not move_to(arm, poses[0][1], gripper, args.tolerance, args.move_timeout):
            print(
                f"  [!] first pose not reached within {args.move_timeout:.0f} s; continuing"
            )
        for n, (a, deg, _, _) in enumerate(poses[1:], 2):
            move_to(arm, deg, gripper, args.tolerance, args.delay)
            print(f"  step {n}/{len(poses)}: aim y {a[1]:+.2f}")
            time.sleep(args.delay)
        print("[ik-scan] sweep done")
    finally:
        if enabled:
            try:
                print("[ik-scan] home")
                arm.home()
                time.sleep(T_HOME)
            except Exception as exc:
                print(f"  [!] home failed: {exc}")
            try:
                arm.release()
                time.sleep(T_RELEASE)
                print("[ik-scan] released (mode 0)")
            except Exception as exc:
                print(f"  [!] release failed: {exc}")
            print_state(arm, "final")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\ninterrupted: arm homed and released")
        sys.exit(130)
