#!/usr/bin/env python3
"""
orbit_frontier_table.py: the frontier table of a run_orbit_sweep.sh output folder.

  python3 scripts/tools/orbit_frontier_table.py SWEEP_DIR                 # markdown table
  python3 scripts/tools/orbit_frontier_table.py SWEEP_DIR --list runnable # "d r" of cells to orbit
  python3 scripts/tools/orbit_frontier_table.py SWEEP_DIR --list full     # cells reaching every waypoint

Per cell (SWEEP_DIR/cells.tsv: analytic class; SWEEP_DIR/frontier/d*_r*.json: IK limits):
  feasible  every orbit waypoint reachable: eq to +-90, top and under to +-60 deg
  all-shells every shell has at least its az = 0 waypoint (runnable), not all waypoints
  partial   (unused since the all-shells class; kept for older tables)
  fail      some shell has no reachable waypoint
  blocked   the parked dog's rear legs would hit the reservoir under the rack (not orbited)
  skipped   analytic R(90) > 0.625 m (not run)
The analytic limit is the planar R(theta) = sqrt((d - r cos)^2 + (r sin)^2) <= 0.605 m; the
3D limit adds the height of the camera over the D1's shoulder (|C - S| <= 0.605 m with S
the Joint2 axis at 0.4796 m above the floor, the camera's position as in orbit_waypoints).
"""
import argparse
import glob
import json
import math
import os

REACH, SHOULDER_Z = 0.605, 0.4796
NEED = {"eq": 90, "top": 60, "under": 60}


def analytic_3d(d, r, h_p, el_deg, reach=REACH, shoulder_z=SHOULDER_Z):
    """Largest azimuth (deg, 1 deg steps) with the camera within reach of the shoulder."""
    el = math.radians(el_deg)
    best = None
    for k in range(0, 181):
        az = math.radians(k)
        x = d - r * math.cos(el) * math.cos(az)
        y = r * math.cos(el) * math.sin(az)
        z = h_p + r * math.sin(el) - shoulder_z
        if math.sqrt(x * x + y * y + z * z) <= reach:
            best = k
        else:
            break
    return best


def load(sweep):
    cells = []
    for line in open(os.path.join(sweep, "cells.tsv")):
        d, r, cls = line.rstrip("\n").split("\t")
        name, r90 = cls.split()
        f = os.path.join(sweep, "frontier", f"d{d}_r{r}.json")
        fr = json.load(open(f)) if os.path.exists(f) else None
        cells.append(dict(d=float(d), r=float(r), analytic_class=name, R90=float(r90), frontier=fr))
    return cells


def classify(c):
    fr = c["frontier"]
    if c["analytic_class"] == "skip":
        return "skipped", "", ""
    if fr is None:
        return "missing", "", ""
    lim = {s: fr["shells"][s]["limit_deg"] for s in NEED}
    if not fr.get("base_clear", True):
        return "blocked", "", "dog body vs rack"
    if any(v is None for v in lim.values()):
        bad = [s for s, v in lim.items() if v is None]
        return "fail", "", ",".join(bad)
    if all(lim[s] >= NEED[s] for s in NEED):
        return "feasible", "", ""
    if all(lim[s] >= 0 for s in NEED):              # every shell has at least its az 0 waypoint
        worst = min(NEED, key=lambda s: lim[s] - NEED[s])
        return "all-shells", lim[worst], worst
    worst = min(NEED, key=lambda s: lim[s] - NEED[s])
    return "partial", lim[worst], worst


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sweep")
    ap.add_argument("--list", choices=("runnable", "full", "best"))
    a = ap.parse_args()
    cells = load(a.sweep)
    if a.list == "best":                             # the 5 runnable cells with the widest reach
        run = [c for c in cells if classify(c)[0] in ("feasible", "all-shells")]
        score = lambda c: sum(c["frontier"]["shells"][s]["limit_deg"] for s in NEED)  # noqa: E731
        for c in sorted(run, key=score, reverse=True)[:5]:
            print(f"{c['d']:.2f} {c['r']:.2f}")
        return
    if a.list:
        for c in cells:
            k = classify(c)[0]
            if (a.list == "full" and k == "feasible") or (a.list == "runnable" and k in ("feasible", "all-shells")):
                print(f"{c['d']:.2f} {c['r']:.2f}")
        return
    rp = os.path.join(a.sweep, "reach.json")
    reach = json.load(open(rp)) if os.path.exists(rp) else {}
    fmt = lambda v: "none" if v is None else f"±{v}"  # noqa: E731
    rows = ["| d (m) | r (m) | R(90) (m) | class | collision-free eq / top / under | reach-only eq / top / under | "
            "az 0 elevations reachable | planar R(θ) limit | planar − reach-only (eq) | dog clears rack |",
            "|---|---|---|---|---|---|---|---|---|---|"]
    for c in cells:
        k, lim_az, lim_shell = classify(c)
        fr = c["frontier"]
        rc = reach.get(f"d{c['d']:.2f}_r{c['r']:.2f}", {})
        rs = rc.get("shells", {})
        ro = " / ".join(fmt(rs.get(s, {}).get("observed_reach_deg")) for s in ("eq", "top", "under")) if rs else ""
        els = rc.get("elevations_reachable_az0")
        env = f"{min(els)}..{max(els)}°" if els else ("none" if els is not None else "")
        plan = rc.get("planar_deg", (fr or {}).get("analytic_limit_deg"))
        plan_s = "" if plan is None else (">=180" if plan >= 180 else f"{plan:.1f}°")
        eq_r = rs.get("eq", {}).get("observed_reach_deg") if rs else None
        diff = "" if plan is None or eq_r is None else (f"{plan - eq_r:+.0f}°" if plan < 180 else f"≥ {180 - eq_r:+.0f}°")
        cf = " / ".join(fmt(fr["shells"][s]["limit_deg"]) for s in ("eq", "top", "under")) if fr else ""
        cls = k + (f" ({lim_shell} {lim_az})" if lim_shell else "")
        rows.append(f"| {c['d']:.2f} | {c['r']:.2f} | {c['R90']:.3f} | {cls} | {cf} | {ro} | {env} | {plan_s} | {diff} | "
                    f"{'' if not fr else ('yes' if fr['base_clear'] else 'no')} |")
    print("\n".join(rows))


if __name__ == "__main__":
    main()
