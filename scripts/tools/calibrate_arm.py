#!/usr/bin/env python3
"""
calibrate_arm.py

Measure per-joint offset for the Unitree D1 arm by commanding small test
moves and comparing commanded vs reported angles.

Procedure (per joint):
    1. Home all joints to 0°  (so the joint starts at a known pose)
    2. Read reported angle at 0°        -> baseline
    3. Command +test_deg, read angle    -> offset_+ = reported − (+test_deg)
    4. Command −test_deg, read angle    -> offset_− = reported − (−test_deg)
    5. Command 0°, read angle           -> return-to-zero sanity check
Always starts and ends with motors RELEASED.

Writes calibration JSON if --output is given.

Usage:
    python3 calibrate_arm.py                     # default: ±10°, all joints
    python3 calibrate_arm.py --test-deg 15       # bigger test move
    python3 calibrate_arm.py --joints 0,1,2      # only some joints
    python3 calibrate_arm.py --output cal.json   # save results
"""
import argparse
import json
import os
import sys
import time

# Reuse the shared D1Arm class from the control module
_here = os.path.dirname(os.path.abspath(__file__))
_repo = os.path.dirname(os.path.dirname(_here))
sys.path.insert(0, os.path.join(_repo, "scripts", "nodes"))
try:
    from d1_arm_control import (
        D1Arm, JOINT_LIMITS, JOINT_NAMES, preflight,
        T_ENABLE, T_HOME, T_RELEASE, T_MOVE, DEG_WAIT,
    )
except ImportError as e:
    sys.exit(f"cannot import d1_arm_control: {e}")


def read_angle(arm, jid, timeout=2.0):
    a = arm.angles(timeout=timeout)
    if not a:
        return None
    d = a.get("data", {})
    v = d.get(f"angle{jid}")
    return float(v) if v is not None else None


def calibrate_joint(arm, jid, test_deg, settle):
    print(f"\n--- joint {jid} ({JOINT_NAMES[jid]}) ±{test_deg:.0f}° ---")
    out = {"joint": jid, "name": JOINT_NAMES[jid]}

    arm.joint(jid, 0.0); time.sleep(settle)
    base = read_angle(arm, jid)
    out["baseline"] = base
    print(f"  baseline       : {base}")

    arm.joint(jid, +test_deg); time.sleep(settle + test_deg * DEG_WAIT)
    pos = read_angle(arm, jid)
    out["at_plus"] = pos
    if pos is not None:
        out["offset_plus"] = round(pos - (+test_deg), 3)
        print(f"  +{test_deg:.0f}° -> read {pos:+.2f}   "
              f"(offset {out['offset_plus']:+.2f})")
    else:
        print(f"  +{test_deg:.0f}° -> (no reading)")

    arm.joint(jid, -test_deg); time.sleep(settle + test_deg * DEG_WAIT)
    neg = read_angle(arm, jid)
    out["at_minus"] = neg
    if neg is not None:
        out["offset_minus"] = round(neg - (-test_deg), 3)
        print(f"  -{test_deg:.0f}° -> read {neg:+.2f}   "
              f"(offset {out['offset_minus']:+.2f})")
    else:
        print(f"  -{test_deg:.0f}° -> (no reading)")

    arm.joint(jid, 0.0); time.sleep(settle)
    back = read_angle(arm, jid)
    out["back_to_zero"] = back
    print(f"  back to 0°     : {back}")

    if out.get("offset_plus") is not None and out.get("offset_minus") is not None:
        out["offset_mean"] = round(
            (out["offset_plus"] + out["offset_minus"]) / 2.0, 3)
        print(f"  mean offset    : {out['offset_mean']:+.2f}°")
    return out


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", default=os.environ.get("ARM_IFACE", "enp0s8"))
    ap.add_argument("--domain", type=int, default=int(os.environ.get("ARM_DOMAIN", "0")))
    ap.add_argument("--ip", default=os.environ.get("ARM_IP", "192.168.123.100"))
    ap.add_argument("--test-deg", type=float, default=10.0)
    ap.add_argument("--joints", default="all")
    ap.add_argument("--settle", type=float, default=2.0)
    ap.add_argument("--output", default=None,
                    help="write calibration JSON to this path")
    ap.add_argument("--yes", "-y", action="store_true")
    ap.add_argument("--skip-preflight", action="store_true")
    args = ap.parse_args()

    joints = (sorted(JOINT_LIMITS.keys())
              if args.joints.lower() == "all"
              else [int(x) for x in args.joints.split(",")])
    for j in joints:
        if j not in JOINT_LIMITS:
            sys.exit(f"unknown joint {j}")
        if abs(args.test_deg) > JOINT_LIMITS[j]:
            sys.exit(f"test {args.test_deg}° exceeds joint {j} limit")

    if not args.skip_preflight and not preflight(args.iface, args.ip):
        return 2

    print("=" * 64)
    print(" D1 ARM CALIBRATION")
    print("=" * 64)
    print(f" iface={args.iface}  domain={args.domain}  test_deg=±{args.test_deg:.0f}°")
    print(f" joints={joints}  settle={args.settle:.1f}s")
    print()

    if not args.yes:
        print("!" * 64)
        print(" !! ARM WILL MOVE through ±test_deg on each joint.")
        print(" !! Clear a workspace.")
        print("!" * 64)
        try:
            if input("Continue? [y/N] ").strip().lower() != "y":
                print("aborted"); return 0
        except EOFError:
            print("aborted"); return 0

    arm = D1Arm(args.iface, args.domain)
    if not arm.ok:
        print("[!] DDS discovery failed")
        return 2
    print("[*] DDS match OK\n")

    results = []
    enabled_by_us = False
    try:
        arm.enable(80000); time.sleep(T_ENABLE)
        st = arm.state()
        if not st or st.get("data", {}).get("enable_status") != 1:
            print("[!] enable failed")
            return 3
        enabled_by_us = True

        for jid in joints:
            results.append(calibrate_joint(arm, jid, args.test_deg, args.settle))

    except KeyboardInterrupt:
        print("\n[!] interrupted")
    finally:
        if enabled_by_us:
            try: arm.home(); time.sleep(T_HOME)
            except Exception: pass
            try: arm.release(); time.sleep(T_RELEASE)
            except Exception: pass

    print()
    print("=" * 64)
    print(" CALIBRATION SUMMARY")
    print("=" * 64)
    print(f" {'joint':>6}  {'name':<12}  {'offset+':>9}  {'offset-':>9}  {'mean':>8}")
    for r in results:
        print(f" {r['joint']:>6}  {r['name']:<12}  "
              f"{str(r.get('offset_plus')):>9}  "
              f"{str(r.get('offset_minus')):>9}  "
              f"{str(r.get('offset_mean')):>8}")

    if args.output:
        payload = {
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "test_deg": args.test_deg,
            "joints": results,
        }
        with open(args.output, "w") as f:
            json.dump(payload, f, indent=2)
        print(f"\n[+] calibration written to {args.output}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
