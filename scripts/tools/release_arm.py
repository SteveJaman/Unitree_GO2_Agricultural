#!/usr/bin/env python3
"""
release_arm.py

Release the D1 arm's holding tension: the motors stop holding and the arm
goes limp (enable-all mode 0, funcode 5).

Released at once, the arm drops under its own and the camera's weight. So by
default the stiffness is ramped down first (mode 80000 -> ... -> 0 over
--ramp seconds): the arm sinks slowly onto its mount instead of falling.
Support it, or fold it first (start_arm.sh fold) if it is up in the air.

    1. Ask (unless --yes): the arm will go limp.
    2. Ramp the stiffness down in --steps steps, then release (mode 0).
    3. --power-off also switches motor power off (funcode 6).

Reaches the arm like the other tools: wired DDS (--iface, auto-detected), or
through the Go2's bridge when the arm is plugged into the dog (ARM_ROBOT, or
found by itself when the wired port has no cable link).

Usage:
    python3 scripts/tools/release_arm.py                # ramp down over 3 s, release
    python3 scripts/tools/release_arm.py --now --yes    # release at once (arm drops!)
    python3 scripts/tools/release_arm.py --power-off
"""
import argparse
import os
import sys
import time

_here = os.path.dirname(os.path.abspath(__file__))
_repo = os.path.dirname(os.path.dirname(_here))
sys.path.insert(0, os.path.join(_repo, "scripts", "nodes"))
try:
    from d1_arm_control import D1Arm, ENABLE_FULL, T_RELEASE, default_arm_iface, preflight, print_state
except ImportError as e:
    sys.exit(f"cannot import d1_arm_control: {e}")


def ramp_modes(steps, top=ENABLE_FULL):
    """Stiffness values from just below `top` down to 0 (mode 0 = released)."""
    steps = max(1, steps)
    return [int(top * (steps - i) / (steps + 1)) for i in range(steps)] + [0]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ramp", type=float, default=3.0, help="s to ramp the stiffness down (default %(default)s)")
    ap.add_argument("--steps", type=int, default=6, help="ramp steps (default %(default)s)")
    ap.add_argument("--now", action="store_true", help="release at once, no ramp (the arm drops)")
    ap.add_argument("--power-off", action="store_true", help="also switch motor power off")
    ap.add_argument("--yes", action="store_true", help="do not ask first")
    ap.add_argument("--iface", default=default_arm_iface())
    ap.add_argument("--domain", type=int, default=int(os.environ.get("ARM_DOMAIN", "0")))
    ap.add_argument("--ip", default=os.environ.get("ARM_IP", "192.168.123.100"))
    ap.add_argument("--skip-preflight", action="store_true")
    args = ap.parse_args()

    modes = [0] if args.now else ramp_modes(args.steps)
    if not args.yes:
        how = "at once: it drops" if args.now else f"slowly over {args.ramp:.0f} s"
        try:
            if input(f"The arm will go limp ({how}). Support it if it is up. Continue? [y/N] "
                     ).strip().lower() != "y":
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

    print_state(arm, "before")
    pause = 0.0 if args.now else args.ramp / max(1, len(modes) - 1)
    for mode in modes:
        arm.enable(mode)  # enable-all with this stiffness; 0 = released
        print(f"  stiffness {mode}" + ("  (released)" if mode == 0 else ""), flush=True)
        if mode:
            time.sleep(pause)
    time.sleep(T_RELEASE)
    if args.power_off:
        arm.power(False)
        time.sleep(T_RELEASE)
        print("  motor power off")
    print_state(arm, "after")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        # Mid-ramp: the last stiffness sent stays; run again to finish releasing.
        print("\ninterrupted: the arm keeps the last stiffness sent")
        sys.exit(130)
