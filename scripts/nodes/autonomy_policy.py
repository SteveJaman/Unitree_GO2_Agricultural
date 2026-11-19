#!/usr/bin/env python3
"""
autonomy_policy.py: per-plant stopping policies for the 3-shell orbit, evaluated against the
full sweep on the same recorded orbits (offline replay; host side).

  python3 scripts/nodes/autonomy_policy.py --mode early_stop \
      --trials SWEEP/orbit_trials.csv [more ...] --out results/early_stop.csv

Inputs: coverage_eval.py's outputs for recorded full orbits: <csv> (one row per waypoint: the
energy E3 of executing and holding it) and <csv>_frames.csv (one row per wrist frame, in capture
order: the detector confidence on the plant and the visible-half samples it saw).

Modes (all replay the waypoints in their recorded, fixed order; none reorders them):
  full        every executed waypoint and frame (the reference)
  three_stop  the az 0 waypoint of each shell only
  early_stop  E5: per frame, c = the detection confidence on the plant (single-class detector:
              the posterior is (c, 1 - c), c clipped to [0.02, 0.98]; no detection -> 0.02),
              fused over the shell's frames so far on the log-odds, b = sigmoid(sum logit c);
              H(b) = -b log2 b - (1-b) log2 (1-b). The current shell stops when H < --h-stop
              (0.3 bit) for --k (3) consecutive frames, or when its waypoints are done; the next
              shell starts with a fresh posterior. The waypoint where it stops is paid in full
              (moved to, settled, held); the rest of its frames and the shell's later waypoints
              are not.

Per run and mode: waypoints and frames used, energy (sum of exec_energy_j + hold_energy_j of the
waypoints used: a relative index, see docs/theory/equations.md E3), coverage fraction (E4, union
of the used frames' seen samples / visible half), and the detection set: the shells in which the
plant was detected (conf >= 0.25 in at least one used frame). Acceptance of early_stop against
full (per run): energy saving >= 30 %, relative coverage loss <= 2 %, identical detection set.
The energy of skipped waypoints is removed; the move from the stop to the next shell's first
waypoint is taken as recorded (an approximation, noted in docs/theory/optimization.md).
"""

import argparse
import csv
import gzip
import math
import os
from collections import defaultdict

SHELL_ORDER = ["eq", "top", "under"]


def open_csv(path):
    """A CSV, or its gzipped copy (path + '.gz': the per-frame tables are stored gzipped)."""
    if not os.path.exists(path) and os.path.exists(path + ".gz"):
        return gzip.open(path + ".gz", "rt", newline="")
    return open(path, newline="")


def fnum(v, d=0.0):
    try:
        return float(v)
    except (TypeError, ValueError):
        return d


def entropy(b):
    if b <= 0.0 or b >= 1.0:
        return 0.0
    return -b * math.log2(b) - (1 - b) * math.log2(1 - b)


def load(trials_csv):
    wps = defaultdict(list)
    for r in csv.DictReader(open(trials_csv)):
        wps[r["run_id"]].append(r)
    frames = defaultdict(list)
    for r in csv.DictReader(open_csv(trials_csv.replace(".csv", "_frames.csv"))):
        frames[(r["run_id"], r["waypoint_k"])].append(r)
    vis = {}
    runs_csv = trials_csv.replace(".csv", "_runs.csv")
    if os.path.exists(runs_csv):
        for r in csv.DictReader(open(runs_csv)):
            if r.get("shell", "all") == "all":
                vis[r["run_id"]] = int(r["visible_samples"])
    return wps, frames, vis


def replay(wp_rows, frames, mode, h_stop=0.3, k=3, det_conf=0.25):
    """The waypoints and frames a policy uses on one recorded run."""
    used_wp, used_fr, stops = [], [], {}
    by_shell = defaultdict(list)
    for w in sorted(wp_rows, key=lambda r: int(r["waypoint_k"])):
        by_shell[w["shell"]].append(w)
    for sh in [s for s in SHELL_ORDER if s in by_shell] + [
        s for s in by_shell if s not in SHELL_ORDER
    ]:
        logit, run_low, stopped = 0.0, 0, False
        for w in by_shell[sh]:
            fr = sorted(
                frames.get((w["run_id"], w["waypoint_k"]), []), key=lambda r: r["frame"]
            )
            if not fr:  # not executed (IK/plan/exec failure): no cost, no frames
                continue
            if mode == "three_stop" and abs(fnum(w["azimuth_deg"])) > 1e-6:
                continue
            used_wp.append(w)
            for f in fr:
                used_fr.append(f)
                if mode != "early_stop":
                    continue
                c = min(0.98, max(0.02, fnum(f["detection_confidence"]) or 0.02))
                logit += math.log(c / (1 - c))
                b = 1.0 / (1.0 + math.exp(-logit))
                run_low = run_low + 1 if entropy(b) < h_stop else 0
                if run_low >= k:
                    stopped = True
                    stops[sh] = f"{w['waypoint_k']}:{f['frame']}"
                    break
            if stopped:
                break
    return used_wp, used_fr, stops


def metrics(used_wp, used_fr, nvis, det_conf=0.25):
    energy = sum(fnum(w["exec_energy_j"]) + fnum(w["hold_energy_j"]) for w in used_wp)
    seen = set()
    for f in used_fr:
        if f.get("seen_idx"):
            seen |= set(map(int, f["seen_idx"].split()))
    dets = sorted(
        {f["shell"] for f in used_fr if fnum(f["detection_confidence"]) >= det_conf}
    )
    return {
        "waypoints": len(used_wp),
        "frames": len(used_fr),
        "energy": energy,
        "coverage": len(seen) / nvis if nvis else 0.0,
        "detections": dets,
    }


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--mode", default="early_stop", choices=["early_stop", "three_stop", "full"]
    )
    ap.add_argument(
        "--trials", nargs="+", required=True, help="coverage_eval.py --csv outputs"
    )
    ap.add_argument(
        "--species",
        nargs="*",
        default=None,
        help="species per --trials file (default: from the path)",
    )
    ap.add_argument("--h-stop", type=float, default=0.3)
    ap.add_argument("--k", type=int, default=3)
    ap.add_argument("--min-saving", type=float, default=0.30)
    ap.add_argument(
        "--max-cov-loss", type=float, default=0.02, help="relative coverage loss"
    )
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    out = []
    for i, path in enumerate(a.trials):
        sp = (
            a.species[i]
            if a.species
            else next(
                (
                    s
                    for s in ["lettuce", "basil", "spinach", "tomato", "strawberry"]
                    if s in os.path.basename(path)
                ),
                "lettuce",
            )
        )
        wps, frames, vis = load(path)
        for run, rows in sorted(wps.items()):
            nvis = vis.get(run, 0)
            ref = metrics(*replay(rows, frames, "full")[:2], nvis)
            uw, uf, stops = replay(rows, frames, a.mode, a.h_stop, a.k)
            pol = metrics(uw, uf, nvis)
            if ref["frames"] == 0:
                continue
            saving = 1 - pol["energy"] / ref["energy"] if ref["energy"] else 0.0
            loss_rel = (
                (ref["coverage"] - pol["coverage"]) / ref["coverage"]
                if ref["coverage"]
                else 0.0
            )
            same = pol["detections"] == ref["detections"]
            out.append(
                {
                    "species": sp,
                    "run_id": run,
                    "mode": a.mode,
                    "full_waypoints": ref["waypoints"],
                    "full_frames": ref["frames"],
                    "full_energy": f"{ref['energy']:.4f}",
                    "full_coverage": f"{ref['coverage']:.4f}",
                    "full_detections": " ".join(ref["detections"]),
                    "policy_waypoints": pol["waypoints"],
                    "policy_frames": pol["frames"],
                    "policy_energy": f"{pol['energy']:.4f}",
                    "policy_coverage": f"{pol['coverage']:.4f}",
                    "policy_detections": " ".join(pol["detections"]),
                    "energy_saving": f"{saving:.4f}",
                    "coverage_loss_abs": f"{ref['coverage'] - pol['coverage']:.4f}",
                    "coverage_loss_rel": f"{loss_rel:.4f}",
                    "same_detection_set": int(same),
                    "stops": " ".join(f"{s}@{v}" for s, v in stops.items()),
                    "accept": int(
                        saving >= a.min_saving and loss_rel <= a.max_cov_loss and same
                    ),
                }
            )
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(out[0].keys()))
        w.writeheader()
        w.writerows(out)
    by = defaultdict(list)
    for r in out:
        by[r["species"]].append(r)
    print(
        "| Species | Runs | Energy saving (mean) | Coverage loss rel. (mean) | Same detection set | Accepted runs |"
    )
    print("|---|---|---|---|---|---|")
    for sp, rs in by.items():
        n = len(rs)
        print(
            f"| {sp} | {n} | {sum(float(r['energy_saving']) for r in rs) / n:.1%} | "
            f"{sum(float(r['coverage_loss_rel']) for r in rs) / n:.1%} | "
            f"{sum(r['same_detection_set'] for r in rs)}/{n} | {sum(r['accept'] for r in rs)}/{n} |"
        )
    print(f"[policy] {len(out)} runs -> {a.out}")


if __name__ == "__main__":
    main()
