#!/usr/bin/env python3
"""
servo_paper_figures.py: figures and tables for a paper from two servo_demo_trials.py runs,
one without and one with the mid-trial disturbance (run_sim_demo.sh --disturb).

  python3 scripts/tools/servo_paper_figures.py NOMINAL_RUN DISTURB_RUN --out docs/images/servo_demo

Writes into --out:
  fig_success.png    success rate per policy and condition, 95 % Wilson intervals
  fig_error.png      final true error (distance to the lettuce's front vs the 0.50 m target),
                     every trial as a dot over a box plot, per policy and condition
  fig_effort.png     arm moves and time to settle (settled trials), per policy and condition
  fig_recovery.png   one disturbed trial per policy: measured distance and tool X over time,
                     the slide marked
  fig_tracker.png    tracker distance against Gazebo ground truth over all trials, and the
                     distribution of its error
  all_trials.csv     one row per trial, both conditions
  summary.md         the tables (mean [95 % CI], success with Wilson CI, tests)
Uses the metrics of servo_trials_report.py.
"""
import argparse
import csv
import json
import math
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import servo_trials_report as rep  # noqa: E402

POLICIES = ["proportional", "fixed_step", "open_loop"]
LABEL = {"proportional": "Proportional", "fixed_step": "Fixed step", "open_loop": "Open loop"}
COND_COLOR = {"nominal": "#2a9d8f", "disturbed": "#e76f51"}
plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})


def load(run, cond, target=0.50, band=0.02):
    meta = json.load(open(os.path.join(run, "trials.json")))
    out = []
    for t in meta["trials"]:
        m = rep.trial_metrics(os.path.join(run, t["dir"]), t, target, band)
        m["condition"], m["dir"] = cond, os.path.join(run, t["dir"])
        out.append(m)
    return out


def fig_success(trials, out):
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    w = 0.36
    for j, cond in enumerate(("nominal", "disturbed")):
        for i, p in enumerate(POLICIES):
            ts = [t for t in trials if t["policy"] == p and t["condition"] == cond]
            if not ts:
                continue
            k, n = sum(t["settled"] for t in ts), len(ts)
            lo, hi = rep.wilson(k, n)
            x = i + (j - 0.5) * w
            ax.bar(x, 100 * k / n, w * 0.92, color=COND_COLOR[cond], label=cond if i == 0 else None)
            ax.errorbar(x, 100 * k / n, yerr=[[100 * (k / n - lo)], [100 * (hi - k / n)]], color="#333", capsize=4)
            ax.text(x, 100 * k / n + 2, f"{k}/{n}", ha="center", va="bottom", fontsize=8)
    ax.set_xticks(range(len(POLICIES)), [LABEL[p] for p in POLICIES])
    ax.set_ylabel("trials settled (%)")
    ax.set_ylim(0, 118)
    ax.set_title("Success: inside ±2 cm of 0.50 m, held 3 s, within 60 s")
    ax.legend(frameon=False, loc="upper center", ncol=2, bbox_to_anchor=(0.5, 1.0))
    ax.set_ylim(0, 128)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_success.png"), dpi=150)
    plt.close(fig)


def fig_error(trials, out):
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    pos, data, cols = [], [], []
    for i, p in enumerate(POLICIES):
        for j, cond in enumerate(("nominal", "disturbed")):
            v = [t["true_final_err_cm"] for t in trials if t["policy"] == p and t["condition"] == cond
                 and math.isfinite(t["true_final_err_cm"])]
            if v:
                pos.append(i + (j - 0.5) * 0.36)
                data.append(v)
                cols.append(COND_COLOR[cond])
    bp = ax.boxplot(data, positions=pos, widths=0.28, patch_artist=True, showfliers=False)
    for b, c in zip(bp["boxes"], cols):
        b.set_facecolor(c)
        b.set_alpha(0.35)
    for x, v, c in zip(pos, data, cols):
        ax.scatter([x + (k % 5 - 2) * 0.02 for k in range(len(v))], v, s=12, color=c, zorder=3)
    ax.axhline(2.0, color="#555", ls="--", lw=1)
    ax.text(len(POLICIES) - 0.45, 2.1, "deadband", fontsize=8, color="#555", ha="right")
    ax.set_xticks(range(len(POLICIES)), [LABEL[p] for p in POLICIES])
    ax.set_ylabel("final error, true (cm)")
    ax.set_title("Distance error at the end of each trial (Gazebo ground truth)")
    for cond, c in COND_COLOR.items():
        ax.scatter([], [], color=c, label=cond)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_error.png"), dpi=150)
    plt.close(fig)


def fig_effort(trials, out):
    fig, axes = plt.subplots(1, 2, figsize=(8.6, 3.4))
    for ax, key, ylabel in ((axes[0], "steps", "arm moves per trial"), (axes[1], "settle_s", "time to settle (s)")):
        w = 0.36
        for j, cond in enumerate(("nominal", "disturbed")):
            for i, p in enumerate(POLICIES):
                v = [float(t[key]) for t in trials if t["policy"] == p and t["condition"] == cond
                     and (key != "settle_s" or t["settled"]) and math.isfinite(float(t[key]))]
                if not v:
                    continue
                m, lo, hi = rep.ci95(v, n=2000)
                x = i + (j - 0.5) * w
                ax.bar(x, m, w * 0.92, color=COND_COLOR[cond], label=cond if i == 0 else None)
                ax.errorbar(x, m, yerr=[[m - lo], [hi - m]], color="#333", capsize=4)
        ax.set_xticks(range(len(POLICIES)), [LABEL[p] for p in POLICIES], fontsize=9)
        ax.set_ylabel(ylabel)
    axes[0].legend(frameon=False)
    axes[1].set_title("settled trials only", fontsize=9)
    fig.suptitle("Effort: mean and 95 % CI", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_effort.png"), dpi=150)
    plt.close(fig)


def series(tdir):
    rows = rep.rows(os.path.join(tdir, "servo_log.csv"))
    if not rows:
        return [], [], []
    t0 = rep.fnum(rows[0]["wall_s"])
    t = [rep.fnum(r["wall_s"]) - t0 for r in rows]
    return t, [rep.fnum(r["current_distance_m"]) for r in rows], [rep.fnum(r["tool_x_m"]) for r in rows]


def fig_recovery(trials, out, disturb_at=4.0):
    fig, axes = plt.subplots(2, 1, figsize=(7.4, 5.0), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
    colors = {"proportional": "#264653", "fixed_step": "#e76f51", "open_loop": "#e9c46a"}
    for p in POLICIES:
        # a far start with the slide away from the rack: the same situation for every policy
        cand = [t for t in trials if t["condition"] == "disturbed" and t["policy"] == p and t["offset_cm"] > 0
                and t.get("disturb_cm", 0) > 0]
        cand = cand or [t for t in trials if t["condition"] == "disturbed" and t["policy"] == p]
        if not cand:
            continue
        t, d, x = series(cand[0]["dir"])
        axes[0].plot(t, d, color=colors[p], lw=1.6, label=f"{LABEL[p]} ({'settled' if cand[0]['settled'] else 'not settled'})")
        axes[1].plot(t, x, color=colors[p], lw=1.6)
    for ax in axes:
        ax.axvline(disturb_at, color="#888", ls=":", lw=1.2)
    axes[0].axhspan(0.48, 0.52, color="#2a9d8f", alpha=0.15)
    axes[0].axhline(0.50, color="#2a9d8f", ls="--", lw=1)
    axes[0].annotate("dog slides 6 cm away", xy=(disturb_at, 0.49), xytext=(disturb_at + 6, 0.488), fontsize=8,
                     color="#555", arrowprops=dict(arrowstyle="->", color="#888"))
    axes[0].set_ylabel("measured distance (m)")
    axes[1].set_ylabel("tool X (m)")
    axes[1].set_xlabel("time since the servo started (s)")
    axes[0].legend(frameon=False, fontsize=8)
    axes[0].set_title("Recovery from a disturbance (far start, slide away from the rack)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_recovery.png"), dpi=150)
    plt.close(fig)


def fig_tracker(trials, out):
    pairs = [pr for t in trials for pr in t["_pairs"]]
    fig, axes = plt.subplots(1, 2, figsize=(8.6, 3.6))
    if pairs:
        m, g = [a for a, _ in pairs], [b for _, b in pairs]
        axes[0].scatter(g, m, s=3, alpha=0.25, color="#264653")
        lo, hi = min(g + m) - 0.02, max(g + m) + 0.02
        axes[0].plot([lo, hi], [lo, hi], color="#e76f51", lw=1)
        err = [100 * (a - b) for a, b in pairs]
        axes[1].hist(err, bins=40, color="#264653", alpha=0.8)
        mu = sum(err) / len(err)
        sd = math.sqrt(sum((e - mu) ** 2 for e in err) / max(1, len(err) - 1))
        axes[1].set_title(f"error: mean {mu:+.2f} cm, sd {sd:.2f} cm (n = {len(err)})", fontsize=9)
    axes[0].set_xlabel("true distance to the lettuce's front (m)")
    axes[0].set_ylabel("tracker distance (m)")
    axes[1].set_xlabel("tracker − truth (cm)")
    fig.suptitle("Detector + depth against Gazebo ground truth, all trials", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_tracker.png"), dpi=150)
    plt.close(fig)


def summary(trials, out):
    lines = ["# Distance servo in simulation: results", "",
             "Target 0.50 m ± 2 cm (camera to the front of the lettuce). Mean [95 % CI] (bootstrap); success "
             "with the 95 % Wilson interval; two-sided Mann-Whitney U and Fisher's exact tests.", ""]
    for cond in ("nominal", "disturbed"):
        ts = [t for t in trials if t["condition"] == cond]
        if not ts:
            continue
        lines += [f"## {cond.capitalize()}" + (" (the dog slides ±6 cm 4 s into each trial)" if cond == "disturbed" else ""),
                  "", "| Metric | " + " | ".join(f"{LABEL[p]} (n={sum(t['policy'] == p for t in ts)})" for p in POLICIES) + " |",
                  "|---|" + "---|" * len(POLICIES)]
        row = []
        for p in POLICIES:
            k = sum(t["settled"] for t in ts if t["policy"] == p)
            n = sum(t["policy"] == p for t in ts)
            lo, hi = rep.wilson(k, n)
            row.append(f"{k}/{n} ({k / n:.0%}, [{lo:.0%}, {hi:.0%}])" if n else "–")
        lines.append("| Success | " + " | ".join(row) + " |")
        for key, label, only_settled in (("true_final_err_cm", "Final error, true (cm)", False),
                                         ("final_err_cm", "Final error, measured (cm)", False),
                                         ("steps", "Arm moves", False), ("settle_s", "Time to settle (s)", True),
                                         ("overshoot_cm", "Overshoot (cm)", False),
                                         ("meas_err_cm", "Tracker error vs truth (cm)", False)):
            cells = []
            for p in POLICIES:
                v = [float(t[key]) for t in ts if t["policy"] == p and (t["settled"] or not only_settled)]
                m, lo, hi = rep.ci95(v)
                cells.append("–" if not math.isfinite(m) else f"{m:.2f} [{lo:.2f}, {hi:.2f}]")
            lines.append(f"| {label} | " + " | ".join(cells) + " |")
        lines += [""] + rep.statistics_section(ts, POLICIES, 0.50)[1:]
        lines.append("")
    open(os.path.join(out, "summary.md"), "w").write("\n".join(lines) + "\n")
    keys = ["condition", "trial", "policy", "offset_cm", "disturb_cm", "settled", "failure", "start_m", "final_m",
            "final_err_cm", "true_final_err_cm", "meas_err_cm", "meas_bias_cm", "settle_s", "steps", "overshoot_cm",
            "seen_frac", "target_px_gain"]
    with open(os.path.join(out, "all_trials.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(keys)
        for t in trials:
            w.writerow([f"{t[k]:.4f}" if isinstance(t.get(k), float) else t.get(k, "") for k in keys])
    return lines


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("nominal")
    ap.add_argument("disturbed")
    ap.add_argument("--out", default="servo_paper")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    trials = load(a.nominal, "nominal") + load(a.disturbed, "disturbed")
    fig_success(trials, a.out)
    fig_error(trials, a.out)
    fig_effort(trials, a.out)
    fig_recovery(trials, a.out)
    fig_tracker(trials, a.out)
    print("\n".join(summary(trials, a.out)))
    print(f"[paper] {a.out}: fig_success, fig_error, fig_effort, fig_recovery, fig_tracker (.png), summary.md, all_trials.csv")


if __name__ == "__main__":
    main()
