#!/usr/bin/env python3
"""
unique_detections.py: does each elevation shell see parts of the plant no other shell detects?

  python3 scripts/tools/unique_detections.py RUN_DIR/orbit_trials_frames.csv [more ...] --out results/unique.csv

Input: the per-frame table of coverage_eval.py (<csv>_frames.csv): for every wrist frame, the
plant surface samples (indices of the visible-half samples, 3000 per plant, fixed seed) that are
seen (inside the image, not occluded: rendered depth within 1.5 cm) AND inside the frame's YOLO
detection box for that plant (IoU >= 0.1 with the plant's projected box) - the frame's
"detection region". Frames with no detection have an empty region.

Region match (the definition): for a frame f of shell s in run k, let U(k, not s) be the union of
the detection regions of all frames of the OTHER shells of the same run (same plant). f is a
unique detection when its region has at least 20 samples outside U(k, not s) and those are at
least 20 % of its region:
    |R_f \\ U| >= 20  and  |R_f \\ U| / |R_f| >= 0.2
i.e. this shell's detection covers a part of the plant that no other shell's detection covered.
Thresholds: --min-samples, --min-frac.

Output: one row per run and shell: frames, frames with a detection, unique detections, the
shell's unique samples (|union of its regions \\ U|) as a fraction of the visible half.
"""
import argparse
import csv
import gzip
import os
from collections import defaultdict


def open_csv(path):
    """A CSV, or its gzipped copy (path + '.gz': the per-frame tables are stored gzipped)."""
    if not os.path.exists(path) and os.path.exists(path + ".gz"):
        return gzip.open(path + ".gz", "rt", newline="")
    return open(path, newline="")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frames_csv", nargs="+")
    ap.add_argument("--out", required=True)
    ap.add_argument("--min-samples", type=int, default=20)
    ap.add_argument("--min-frac", type=float, default=0.2)
    ap.add_argument("--visible", type=int, default=0, help="visible-half samples per plant (0: from the runs CSV)")
    a = ap.parse_args()
    rows = []
    for path in a.frames_csv:
        path = path[:-3] if path.endswith(".gz") else path
        vis = {}
        runs_csv = path.replace("_frames.csv", "_runs.csv")
        if os.path.exists(runs_csv):
            for r in csv.DictReader(open(runs_csv)):
                vis[r["run_id"]] = int(r["visible_samples"])
        frames = defaultdict(list)                    # run -> [(shell, region set)]
        for r in csv.DictReader(open_csv(path)):
            reg = set(map(int, r["det_region"].split())) if r.get("det_region") else set()
            frames[r["run_id"]].append((r["shell"], reg))
        for run, fl in frames.items():
            shells = sorted({s for s, _ in fl})
            for s in shells:
                other = set().union(*[reg for sh, reg in fl if sh != s]) if len(shells) > 1 else set()
                mine = [reg for sh, reg in fl if sh == s]
                det = [reg for reg in mine if reg]
                uniq = 0
                for reg in det:
                    new = reg - other
                    if len(new) >= a.min_samples and len(new) / len(reg) >= a.min_frac:
                        uniq += 1
                union_new = set().union(*mine) - other if mine else set()
                nv = a.visible or vis.get(run, 0)
                rows.append({"run_id": run, "shell": s, "frames": len(mine), "frames_detected": len(det),
                             "unique_detections": uniq, "unique_samples": len(union_new),
                             "unique_fraction_of_visible": f"{len(union_new) / nv:.4f}" if nv else ""})
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else ["run_id"])
        w.writeheader()
        w.writerows(rows)
    print(f"[unique] {len(rows)} run x shell rows -> {a.out}")


if __name__ == "__main__":
    main()
