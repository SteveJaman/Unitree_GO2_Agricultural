#!/usr/bin/env python3
"""
make_stats_table.py: the orbit study's statistics table (markdown), from the result CSVs.

  python3 scripts/tools/make_stats_table.py --out results/orbit_stats.md

Reads (repo paths): results/orbit_trials.csv (Phase 2, lettuce control world, per waypoint),
results/species/<species>_trials.csv (Phase 3, per waypoint), results/policy/early_stop.csv and
results/policy/three_stop.csv (Phase 4, per run: the policy against the full sweep).

Tests, the same as the servo report's (scripts/tools/servo_trials_report.py: its wilson() and
ci95() are imported, and scipy's mannwhitneyu / fisher_exact are called the same way):
  - success rates (waypoint executed per IK-feasible waypoint; early-stop acceptance) with 95 %
    Wilson score intervals;
  - two-sided Mann-Whitney U on the per-run energy (full sweep vs each policy);
  - Fisher's exact test on the detection sets (runs whose detection set equals the full
    sweep's, policy vs full) and on per-shell detection (shells with a detection).
Energy is the simulation's relative index (docs/theory/equations.md E3), not hardware joules.
"""
import argparse
import csv
import os
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
from servo_trials_report import ci95, fnum, rows, wilson  # noqa: E402

SPECIES = ["lettuce", "basil", "spinach", "tomato", "strawberry"]


def rate(k, n):
    if not n:
        return "0/0 | – | –"
    lo, hi = wilson(k, n)
    return f"{k}/{n} | {k / n:.0%} | [{lo:.0%}, {hi:.0%}]"


def mean_ci(v):
    m, lo, hi = ci95(v, n=2000)
    return f"{m:.2f} [{lo:.2f}, {hi:.2f}]" if v else "–"


def waypoint_success(trials):
    """Executed (plan + execution succeeded) out of the IK-feasible waypoints."""
    feas = [t for t in trials if t.get("ik_success") == "1"]
    k = sum(1 for t in feas if t.get("plan_success") == "1" and not t.get("fail_reason"))
    return k, len(feas)


def section_cells(out, path):
    tr = rows(path)
    if not tr:
        return
    out += ["", "## Waypoint execution per cell (Phase 2, lettuce control)", "",
            "| Cell (d, r) | Runs | Executed / IK-feasible | Rate | 95 % CI (Wilson) | Energy per run (mean [95 % CI]) |",
            "|---|---|---|---|---|---|"]
    cells = defaultdict(list)
    for t in tr:
        cells[(t["d"], t["r"])].append(t)
    for (d, r), ts in sorted(cells.items(), key=lambda x: (float(x[0][0]), float(x[0][1]))):
        k, n = waypoint_success(ts)
        e = defaultdict(float)
        for t in ts:
            e[t["run_id"]] += (fnum(t["exec_energy_j"]) if t["exec_energy_j"] else 0) + \
                (fnum(t["hold_energy_j"]) if t["hold_energy_j"] else 0)
        out.append(f"| {float(d):.2f}, {float(r):.2f} | {len(e)} | {rate(k, n)} | {mean_ci(list(e.values()))} |")


def section_species(out):
    out += ["", "## Waypoint execution per species (Phase 3, best cell)", "",
            "| Species | Executed / IK-feasible | Rate | 95 % CI (Wilson) |", "|---|---|---|---|"]
    for sp in SPECIES:
        tr = rows(os.path.join(REPO, "results", "species", f"{sp}_trials.csv"))
        if tr:
            k, n = waypoint_success(tr)
            out.append(f"| {sp} | {rate(k, n)} |")


def section_policy(out):
    try:
        from scipy.stats import fisher_exact, mannwhitneyu
    except ImportError:
        out += ["", "(scipy not installed: no significance tests)"]
        return
    pol = {m: rows(os.path.join(REPO, "results", "policy", f"{m}.csv")) for m in ("early_stop", "three_stop")}
    if not any(pol.values()):
        return
    out += ["", "## Stopping policies against the full sweep (Phase 4, per run)", "",
            "| Species | Policy | Runs | Energy saving (mean [95 % CI]) | Coverage loss, rel. (mean [95 % CI]) | "
            "Same detection set | Rate | 95 % CI (Wilson) | Accepted | Rate | 95 % CI (Wilson) |",
            "|---|---|---|---|---|---|---|---|---|---|---|"]
    for sp in SPECIES:
        for m, rs in pol.items():
            rs = [r for r in rs if r["species"] == sp]
            if not rs:
                continue
            same = sum(int(r["same_detection_set"]) for r in rs)
            acc = sum(int(r["accept"]) for r in rs)
            out.append(f"| {sp} | {m} | {len(rs)} | {mean_ci([fnum(r['energy_saving']) * 100 for r in rs])} % | "
                       f"{mean_ci([fnum(r['coverage_loss_rel']) * 100 for r in rs])} % | {rate(same, len(rs))} | "
                       f"{rate(acc, len(rs))} |")
    out += ["", "## Tests (two-sided): full sweep vs policy", "",
            "| Species | Policy | Energy, Mann-Whitney U | Detection set equal to full, Fisher | "
            "Shells with a detection, Fisher |", "|---|---|---|---|---|"]
    for sp in SPECIES:
        for m, rs in pol.items():
            rs = [r for r in rs if r["species"] == sp]
            if not rs:
                continue
            ef = [fnum(r["full_energy"]) for r in rs]
            ep = [fnum(r["policy_energy"]) for r in rs]
            mw = f"p = {mannwhitneyu(ef, ep, alternative='two-sided').pvalue:.3g}" if len(rs) >= 3 else "n too small"
            n = len(rs)
            same = sum(int(r["same_detection_set"]) for r in rs)
            fs = fisher_exact([[n, 0], [same, n - same]])[1]
            sf = sum(len(r["full_detections"].split()) for r in rs)
            sp_ = sum(len(r["policy_detections"].split()) for r in rs)
            nf = sum(int(r["full_waypoints"]) > 0 for r in rs) * 3
            fsh = fisher_exact([[sf, nf - sf], [sp_, nf - sp_]])[1]
            out.append(f"| {sp} | {m} | {mw} | p = {fs:.3g} ({same}/{n} equal) | p = {fsh:.3g} ({sp_} vs {sf} of {nf}) |")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.join(REPO, "results", "orbit_stats.md"))
    a = ap.parse_args()
    out = ["# Orbit study: statistics", "",
           "Generated by `scripts/tools/make_stats_table.py` from `results/`. Rates with 95 % Wilson score "
           "intervals; means with 95 % bootstrap intervals; tests two-sided (Mann-Whitney U, Fisher's exact), "
           "as in the servo report. Energy is the simulation's relative index (E3), not hardware joules."]
    section_cells(out, os.path.join(REPO, "results", "orbit_trials.csv"))
    section_species(out)
    section_policy(out)
    open(a.out, "w").write("\n".join(out) + "\n")
    print("\n".join(out))
    print(f"[stats] -> {a.out}")


if __name__ == "__main__":
    main()
