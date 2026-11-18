#!/usr/bin/env python3
"""Poster figures from the phase-4 n = 10 data (docs/simulation/results/phase4/data).

    python3 tests/traversability/poster_figures.py [DATA_DIR] [OUT_DIR]

height_scatter.png   true object height vs LiDAR-measured height (fused 3D boxes)
clearance_bars.png   foot passes over items <= 0.10 m that cleared, per mode, Wilson 95 % CI
"""
import csv
import math
import os
import statistics
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DATA = sys.argv[1] if len(sys.argv) > 1 else os.path.join(REPO, "docs/simulation/results/phase4/data")
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(REPO, "docs/images/poster")

INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
plt.rcParams.update({
    "font.size": 13, "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2,
    "ytick.color": INK2, "axes.spines.top": False, "axes.spines.right": False,
    "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
})


def wilson(k, n, z=1.96):
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return c - h, c + h


def height_scatter():
    rows = [r for r in csv.DictReader(open(os.path.join(DATA, "boxes.csv"))) if r["est_h"]]
    t = [float(r["true_h"]) * 100 for r in rows]
    e = [float(r["est_h"]) * 100 for r in rows]
    med = statistics.median(abs(float(r["err_h"])) for r in rows) * 100
    fig, axes = plt.subplots(1, 2, figsize=(12, 6.4))
    for ax, hi, title in ((axes[0], 200, "All objects"), (axes[1], 35, "Zoom: the step-over decision range")):
        ax.plot([0, hi], [0, hi], color=INK2, lw=1.5, ls="--", zorder=1)
        ax.text(hi * 0.97, hi * 0.90, "1:1 (perfect)", color=INK2, ha="right", fontsize=11)
        for lim, name in ((12, "step-over limit 12 cm"), (22, "lethal from 22 cm")):
            if lim < hi:
                ax.axhline(lim, color=GRID, lw=1.2, zorder=0)
                ax.text(hi * 0.02, lim + hi * 0.012, name, color=INK2, fontsize=10)
        ax.scatter(t, e, s=46 if hi > 50 else 70, color=BLUE, alpha=0.5, edgecolor=SURF, linewidth=1, zorder=2)
        ax.set_xlim(0, hi)
        ax.set_ylim(0, hi)
        ax.set_aspect("equal")
        ax.set_xlabel("True object height (cm)")
        ax.set_ylabel("LiDAR-measured height (cm)")
        ax.set_title(title, loc="left", color=INK, fontsize=13)
    axes[0].annotate("tall furniture under-read:\nLiDAR misses the top,\nstill far above lethal", xy=(185, 70),
                     xytext=(95, 25), fontsize=10, color=INK2, arrowprops=dict(arrowstyle="->", color=INK2))
    fig.suptitle(f"Estimated, not exact: median height error {med:.1f} cm", x=0.06, ha="left", fontsize=16)
    fig.text(0.06, 0.02, f"{len(rows)} fused-box measurements from 20 Gazebo trials (phase 4, traversability modes). "
             "Each object is measured in several trials, so points overlap.", fontsize=9.5, color=INK2)
    fig.tight_layout(rect=(0, 0.06, 1, 0.95))
    fig.savefig(os.path.join(OUT, "height_scatter.png"), dpi=200)
    plt.close(fig)


def clearance_bars():
    rows = [r for r in csv.DictReader(open(os.path.join(DATA, "foot_passes.csv"))) if float(r["height"]) <= 0.10]
    modes = [("baseline", "Baseline\n(4 cm step)", INK2), ("traversability", "Traversability", BLUE),
             ("traversability_semantic", "Traversability\n+ semantic", AQUA)]
    fig, ax = plt.subplots(figsize=(8, 6))
    for i, (m, label, col) in enumerate(modes):
        sub = [r for r in rows if r["mode"] == m]
        k, n = sum(r["cleared"] == "True" for r in sub), len(sub)
        lo, hi = wilson(k, n)
        p = 100 * k / n
        ax.bar(i, p, width=0.6, color=col, zorder=2)
        ax.errorbar(i, p, yerr=[[p - 100 * lo], [100 * hi - p]], color=INK, capsize=8, lw=1.5, zorder=3)
        ax.text(i, 100 * hi + 3, f"{p:.0f} %\n{k} / {n} passes", ha="center", va="bottom", color=INK, fontsize=13)
    # significance bracket: traversability vs baseline only
    y = 92
    ax.plot([0, 0, 1, 1], [y - 2, y, y, y - 2], color=INK, lw=1.2)
    ax.text(0.5, y + 1, "p = 0.008, h = 0.57", ha="center", va="bottom", fontsize=12)
    ax.set_xticks(range(3), [m[1] for m in modes])
    ax.set_ylim(0, 100)
    ax.set_yticks(range(0, 101, 20), [f"{v} %" for v in range(0, 101, 20)])
    ax.yaxis.grid(True, color=GRID, zorder=0)
    ax.set_ylabel("Foot passes that stayed above the item")
    ax.set_title("Foot clearance over items 10 cm or lower", loc="left", fontsize=15)
    fig.text(0.02, 0.02, "n = 10 trials per mode, seeds 1234-1243. Error bars: Wilson 95 % CI.\n"
             "Semantic vs baseline p = 0.20, semantic vs traversability p = 0.35: not significant.", fontsize=9.5, color=INK2)
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    fig.savefig(os.path.join(OUT, "clearance_bars.png"), dpi=200)
    plt.close(fig)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    height_scatter()
    clearance_bars()
    print("wrote", OUT)
