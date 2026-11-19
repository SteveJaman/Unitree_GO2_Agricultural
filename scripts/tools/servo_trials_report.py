#!/usr/bin/env python3
"""
servo_trials_report.py: figures and a results table from servo_demo_trials.py's output.

  python3 scripts/tools/servo_trials_report.py RUN_DIR            # e.g. ~/files/go2-nav2-wifi/ws/log/servo_demo/<run>

Reporting follows the evaluation protocol of Zhang & Gupta, "Push Past Green" (CoRL 2023):
several trials per policy in shuffled order, each metric as the mean with a 95 %
confidence interval (bootstrap, 5000 resamples), and the metric plotted against the
interaction (step) index, all policies on one axis. The metrics are computed from logged
data only; ground truth comes from Gazebo, not from the tracker.

Per trial:
  start_m / final_m       measured distance (tracker) at the first and last logged tick
  final_err_cm            |final measured distance - target|
  true_final_err_cm       |true distance to the front of the lettuce at the end - target| (Gazebo
                          depth over the lettuce's projected outline, no YOLO)
  meas_err_cm             mean |measured - true depth| while the lettuce was seen (tracker accuracy)
  settle_s                time from the servo's start until the distance entered the deadband for
                          good (the servo then holds it for its settle_s before it stops)
  steps                   arm moves commanded; overshoot_cm: furthest past the target
  target_px_gain          visible lettuce pixels near the target, end / start (colour + depth mask)

Writes into RUN_DIR: results.csv (one row per trial), results.md (table per policy),
distance_over_time.png (one proportional trial: the servo demo graph), error_over_time.png,
error_vs_step.png, measured_vs_true.png, visible_plant.png.
"""
import argparse
import csv
import json
import math
import os
import random
import subprocess
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
COLORS = {"proportional": "#264653", "fixed_step": "#e76f51", "open_loop": "#e9c46a"}


def fnum(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else math.nan
    except (TypeError, ValueError):
        return math.nan


def rows(path):
    return list(csv.DictReader(open(path))) if os.path.exists(path) else []


def ci95(values, n=5000, seed=0):
    v = [x for x in values if x is not None and math.isfinite(x)]
    if not v:
        return math.nan, math.nan, math.nan
    rng = random.Random(seed)
    means = sorted(sum(rng.choice(v) for _ in v) / len(v) for _ in range(n))
    return sum(v) / len(v), means[int(0.025 * n)], means[int(0.975 * n) - 1]


def wilson(k, n, z=1.96):
    """95 % Wilson score interval of a success rate k/n."""
    if n == 0:
        return math.nan, math.nan
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def statistics_section(trials, policies, target):
    """Markdown: success rates with Wilson CIs, pairwise tests between policies (two-sided
    Mann-Whitney U on the continuous metrics, Fisher's exact test on success), and the
    results split by start (far: the arm approaches; near: it backs off)."""
    try:
        from scipy.stats import fisher_exact, mannwhitneyu
    except ImportError:
        return ["", "(scipy not installed: no significance tests)"]
    out = ["", "## Success rate (settled within the trial time)", "",
           "| Policy | Settled | Rate | 95 % CI (Wilson) |", "|---|---|---|---|"]
    for p in policies:
        n = sum(t["policy"] == p for t in trials)
        k = sum(t["settled"] for t in trials if t["policy"] == p)
        lo, hi = wilson(k, n)
        out.append(f"| {p} | {k}/{n} | {k / n:.0%} | [{lo:.0%}, {hi:.0%}] |" if n else f"| {p} | 0/0 | – | – |")
    out += ["", "## Pairwise tests (two-sided; Mann-Whitney U, Fisher's exact for success)", "",
            "| Comparison | Final error, true | Arm moves | Time to settle | Success |", "|---|---|---|---|---|"]
    for i, a_ in enumerate(policies):
        for b_ in policies[i + 1:]:
            cells = []
            for k in ("true_final_err_cm", "steps", "settle_s"):
                xa = [float(t[k]) for t in trials if t["policy"] == a_ and math.isfinite(float(t[k]))]
                xb = [float(t[k]) for t in trials if t["policy"] == b_ and math.isfinite(float(t[k]))]
                if len(xa) >= 3 and len(xb) >= 3:
                    cells.append(f"p = {mannwhitneyu(xa, xb, alternative='two-sided').pvalue:.3g}")
                else:
                    cells.append("n too small")
            ka = sum(t["settled"] for t in trials if t["policy"] == a_)
            na = sum(t["policy"] == a_ for t in trials)
            kb = sum(t["settled"] for t in trials if t["policy"] == b_)
            nb = sum(t["policy"] == b_ for t in trials)
            cells.append(f"p = {fisher_exact([[ka, na - ka], [kb, nb - kb]])[1]:.3g}")
            out.append(f"| {a_} vs {b_} | " + " | ".join(cells) + " |")
    fails = [t for t in trials if not t["settled"]]
    if fails:
        out += ["", "## Trials that did not settle", "", "| Policy | Start | Disturbance | Cause |", "|---|---|---|---|"]
        for t in sorted(fails, key=lambda t: (t["policy"], t["trial"])):
            out.append(f"| {t['policy']} | {'far' if t['offset_cm'] > 0 else 'near'} ({t['offset_cm']:+.1f} cm) | "
                       f"{t['disturb_cm']:+.1f} cm | {t['failure']} |")
    out += ["", "## By start", "",
            "| Policy | Start | n | Settled | Final error, true (cm) | Arm moves | Time to settle (s) |",
            "|---|---|---|---|---|---|---|"]
    for p in policies:
        for name, sel in (("far", lambda t: t["offset_cm"] > 0), ("near", lambda t: t["offset_cm"] <= 0)):
            ts = [t for t in trials if t["policy"] == p and sel(t)]
            if not ts:
                continue
            cell = lambda k: "{:.2f} [{:.2f}, {:.2f}]".format(*ci95([float(t[k]) for t in ts], n=2000))  # noqa: E731
            out.append(f"| {p} | {name} | {len(ts)} | {sum(t['settled'] for t in ts)}/{len(ts)} | "
                       f"{cell('true_final_err_cm')} | {cell('steps')} | {cell('settle_s')} |")
    return out


def trial_metrics(tdir, info, target, band):
    log = rows(os.path.join(tdir, "servo_log.csv"))
    gt = rows(os.path.join(tdir, "ground_truth.csv"))
    steps = rows(os.path.join(tdir, "servo_steps.csv"))
    frames = rows(os.path.join(tdir, "frames.csv"))
    meas = [(fnum(r["wall_s"]), fnum(r["current_distance_m"])) for r in log]
    seen = [(t, d) for t, d in meas if math.isfinite(d)]
    t0 = fnum(log[0]["wall_s"]) if log else math.nan
    # tracker vs truth: pair each measurement with the nearest ground-truth sample
    gts = [(fnum(r["wall_s"]), fnum(r["gt_depth_m"]), fnum(r["gt_axial_m"])) for r in gt]
    pairs = []
    k = 0
    for t, d in seen:
        while k + 1 < len(gts) and abs(gts[k + 1][0] - t) <= abs(gts[k][0] - t):
            k += 1
        if gts and abs(gts[k][0] - t) < 0.25 and math.isfinite(gts[k][1]):
            pairs.append((d, gts[k][1]))
    true_end = next((g[1] for g in reversed(gts) if math.isfinite(g[1])), math.nan)
    settle = math.nan
    if info.get("settled") and seen:
        # the start of the last run inside the deadband (the servo then confirms for settle_s)
        k = len(seen)
        while k > 0 and abs(seen[k - 1][1] - target) <= band:
            k -= 1
        if k < len(seen):
            settle = seen[k][0] - t0
    over = 0.0
    if seen:
        sign0 = math.copysign(1, seen[0][1] - target)
        over = max([sign0 * (target - d) for _, d in seen] + [0.0])
    tpx = [fnum(r["target_px"]) for r in frames]
    # why a trial did not settle: the servo's state over its last 5 s
    tail = [r["state"] for r in log if fnum(r["wall_s"]) >= fnum(log[-1]["wall_s"]) - 5.0] if log else []
    if info.get("settled"):
        cause = ""
    elif tail and tail.count("limit") >= len(tail) / 3:
        cause = "travel limit"
    elif tail and tail.count("hold") >= len(tail) / 2 and info.get("policy") == "open_loop":
        cause = "no feedback (open loop)"
    elif tail and (tail.count("stale") + tail.count("out_of_range")) >= len(tail) / 2:
        cause = "target lost"
    else:
        cause = "still moving at timeout"
    return {
        "trial": info["trial"], "policy": info["policy"], "offset_cm": 100 * info["offset_m"],
        "disturb_cm": 100 * info.get("disturb_m", 0.0), "failure": cause,
        "start_m": seen[0][1] if seen else math.nan, "final_m": seen[-1][1] if seen else math.nan,
        "final_err_cm": 100 * abs(seen[-1][1] - target) if seen else math.nan,
        "true_final_err_cm": 100 * abs(true_end - target) if math.isfinite(true_end) else math.nan,
        "meas_err_cm": 100 * sum(abs(a - b) for a, b in pairs) / len(pairs) if pairs else math.nan,
        "meas_bias_cm": 100 * sum(a - b for a, b in pairs) / len(pairs) if pairs else math.nan,
        "settle_s": settle, "settled": bool(info.get("settled")), "steps": len(steps),
        "overshoot_cm": 100 * over, "seen_frac": len(seen) / len(meas) if meas else math.nan,
        "target_px_gain": (tpx[-1] / tpx[0]) if len(tpx) > 1 and tpx[0] > 0 else math.nan,
        "_seen": [(t - t0, d) for t, d in seen], "_pairs": pairs,
        "_steps": [(int(s["step"]), fnum(s["distance_after_m"])) for s in steps],
        "_frames": [(r["tag"], tpx_i) for r, tpx_i in zip(frames, tpx)],
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("--target", type=float, default=0.50)
    ap.add_argument("--deadband", type=float, default=0.02)
    a = ap.parse_args()
    run = os.path.abspath(a.run_dir)
    meta = json.load(open(os.path.join(run, "trials.json")))
    trials = [trial_metrics(os.path.join(run, t["dir"]), t, a.target, a.deadband) for t in meta["trials"]]
    if not trials:
        sys.exit("no trials in trials.json")
    policies = [p for p in ("proportional", "fixed_step", "open_loop") if any(t["policy"] == p for t in trials)]
    policies += sorted({t["policy"] for t in trials} - set(policies))

    keys = ["trial", "policy", "offset_cm", "disturb_cm", "failure", "start_m", "final_m", "final_err_cm", "true_final_err_cm", "meas_err_cm",
            "meas_bias_cm", "settle_s", "settled", "steps", "overshoot_cm", "seen_frac", "target_px_gain"]
    with open(os.path.join(run, "results.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(keys)
        for t in sorted(trials, key=lambda t: t["trial"]):
            w.writerow([f"{t[k]:.4f}" if isinstance(t[k], float) else t[k] for k in keys])

    metrics = [("final_err_cm", "Final error, measured (cm)"), ("true_final_err_cm", "Final error, true (cm)"),
               ("settle_s", "Time to settle (s)"), ("steps", "Arm moves"), ("overshoot_cm", "Overshoot (cm)"),
               ("meas_err_cm", "Tracker error vs truth (cm)"), ("target_px_gain", "Visible lettuce, end / start")]
    lines = [f"# Servo demo results: {meta.get('world', '')}", "",
             f"Target {a.target:.2f} m ± {a.deadband * 100:.0f} cm. Mean [95 % CI] over trials (bootstrap); "
             "settled = stayed in the deadband for the servo's settle time.", "",
             "| Metric | " + " | ".join(f"{p} (n={sum(t['policy'] == p for t in trials)})" for p in policies) + " |",
             "|---|" + "---|" * len(policies)]
    lines.append("| Settled | " + " | ".join(
        f"{sum(t['settled'] for t in trials if t['policy'] == p)}/{sum(t['policy'] == p for t in trials)}" for p in policies) + " |")
    for k, label in metrics:
        cells = []
        for p in policies:
            m, lo, hi = ci95([float(t[k]) for t in trials if t["policy"] == p])
            cells.append("–" if not math.isfinite(m) else f"{m:.2f} [{lo:.2f}, {hi:.2f}]")
        lines.append(f"| {label} | " + " | ".join(cells) + " |")
    lines += statistics_section(trials, policies, a.target)
    open(os.path.join(run, "results.md"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))

    # 1. the single-trial graph (distance_over_time.png): the first settled proportional trial
    pick = next((t for t in sorted(trials, key=lambda t: t["trial"]) if t["policy"] == "proportional" and t["settled"]),
                next((t for t in trials if t["policy"] == "proportional"), trials[0]))
    tdir = os.path.join(run, next(x["dir"] for x in meta["trials"] if x["trial"] == pick["trial"]))
    subprocess.run([sys.executable, os.path.join(HERE, "plot_distance.py"), os.path.join(tdir, "servo_log.csv"),
                    "--out", os.path.join(run, "distance_over_time.png"), "--target", str(a.target),
                    "--deadband", str(a.deadband),
                    "--title", f"Arm distance servo in Gazebo (trial {pick['trial']}, {pick['policy']})"], check=False)

    # 2. |error| over time per policy, mean and 95 % CI on a 0.25 s grid
    fig, ax = plt.subplots(figsize=(9, 4.8))
    for p in policies:
        series = [t["_seen"] for t in trials if t["policy"] == p and t["_seen"]]
        if not series:
            continue
        tmax = max(s[-1][0] for s in series)
        grid = [i * 0.25 for i in range(int(tmax / 0.25) + 1)]
        mean, lo, hi, gx = [], [], [], []
        for g in grid:
            vals = []
            for s in series:
                past = [d for tt, d in s if tt <= g]
                if past:
                    vals.append(100 * abs(past[-1] - a.target))
            if len(vals) >= max(2, len(series) // 2):
                m, l_, h_ = ci95(vals, n=1000)
                gx.append(g); mean.append(m); lo.append(l_); hi.append(h_)
        ax.plot(gx, mean, color=COLORS.get(p), lw=2, label=f"{p} (n={len(series)})")
        ax.fill_between(gx, lo, hi, color=COLORS.get(p), alpha=0.2)
    ax.axhline(100 * a.deadband, color="#2a9d8f", ls="--", lw=1, label="deadband")
    ax.set_xlabel("time since the servo started (s)")
    ax.set_ylabel("|distance − target| (cm)")
    ax.set_title("Distance error over time, mean and 95 % CI")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(run, "error_over_time.png"), dpi=130)
    plt.close(fig)

    # 3. error after each interaction (step index), as the paper's metric-vs-time-step curves
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for p in policies:
        per = {}
        for t in trials:
            if t["policy"] != p:
                continue
            per.setdefault(0, []).append(100 * abs(t["start_m"] - a.target) if math.isfinite(t["start_m"]) else math.nan)
            last = t["start_m"]
            for k in range(1, 13):
                after = next((d for s, d in t["_steps"] if s == k and math.isfinite(d)), None)
                last = after if after is not None else last   # no more steps: the error stays
                per.setdefault(k, []).append(100 * abs(last - a.target) if math.isfinite(last) else math.nan)
        ks = sorted(per)
        stats = [ci95(per[k], n=1000) for k in ks]
        ax.errorbar(ks, [s[0] for s in stats], yerr=[[s[0] - s[1] for s in stats], [s[2] - s[0] for s in stats]],
                    color=COLORS.get(p), marker="o", ms=4, capsize=3, label=p)
    ax.axhline(100 * a.deadband, color="#2a9d8f", ls="--", lw=1, label="deadband")
    ax.set_xlabel("arm move (interaction) index")
    ax.set_ylabel("|distance − target| after the move (cm)")
    ax.set_title("Error after each move, mean and 95 % CI")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(run, "error_vs_step.png"), dpi=130)
    plt.close(fig)

    # 4. tracker (YOLO box + depth) against the true distance to the front of the lettuce
    fig, ax = plt.subplots(figsize=(5.2, 5))
    allp = [pr for t in trials for pr in t["_pairs"]]
    if allp:
        ax.scatter([b for _, b in allp], [m for m, _ in allp], s=6, alpha=0.4, color="#264653")
        lo_, hi_ = min(min(x) for x in allp) - 0.02, max(max(x) for x in allp) + 0.02
        ax.plot([lo_, hi_], [lo_, hi_], color="#e76f51", lw=1, label="measured = true")
        ax.legend(fontsize=8)
    ax.set_xlabel("true distance to the lettuce's front (m)")
    ax.set_ylabel("tracker distance (m)")
    ax.set_title("Tracker against Gazebo ground truth")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(run, "measured_vs_true.png"), dpi=130)
    plt.close(fig)

    # 5. visible lettuce pixels in the wrist view, before/after each move
    fig, ax = plt.subplots(figsize=(8, 4.2))
    for p in policies:
        for t in trials:
            if t["policy"] == p and t["_frames"]:
                ax.plot(range(len(t["_frames"])), [v for _, v in t["_frames"]], color=COLORS.get(p), alpha=0.5, lw=1)
        ax.plot([], [], color=COLORS.get(p), label=p)
    ax.set_xlabel("wrist frame (start, before/after each move, end)")
    ax.set_ylabel("lettuce pixels near the target")
    ax.set_title("Visible target lettuce (colour + depth mask)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(run, "visible_plant.png"), dpi=130)
    plt.close(fig)
    print(f"[report] {run}: results.md, results.csv, distance_over_time.png, error_over_time.png, "
          "error_vs_step.png, measured_vs_true.png, visible_plant.png")


if __name__ == "__main__":
    main()
