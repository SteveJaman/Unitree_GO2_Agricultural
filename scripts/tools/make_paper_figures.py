#!/usr/bin/env python3
"""
make_paper_figures.py: every orbit-study figure (F1-F6) from the result CSVs, one command.

  python3 scripts/tools/make_paper_figures.py            # -> docs/images/orbit_paper/F*.png

Inputs (repo paths unless noted):
  F1  results/orbit_frontier/frontier/*.json (collision-free limits), results/orbit_frontier/reach.json
  F2  results/species/<species>_trials.csv (waypoints executed per species, 5 plants each)
  F3  results/species/<species>_trials_runs.csv (coverage per shell), results/species/unique.csv
  F4  results/policy/early_stop.csv and three_stop.csv (per run: full sweep and policy)
  F5  results/species/<species>_trials_frames.csv(.gz) to pick frames; the images from --frames-root
      (default ~/files/go2-nav2-wifi/ws/log/orbit/species/<species>/runs); the YOLO box is redrawn
      with ~/.cache/go2-yolo/<species>_yolo11s.pt
  F6  the servo study's figures (docs/images/servo_demo/fig_*.png, commit 4f36620), tiled as they
      are: reused, not regenerated
Energy is the simulation's relative index (docs/theory/equations.md E3).
"""
import argparse
import csv
import glob
import gzip
import json
import math
import os
import sys
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
from servo_trials_report import ci95  # noqa: E402

SPECIES = ["lettuce", "basil", "spinach", "tomato", "strawberry"]
SHELLS = [("eq", 0.0, (-10, 10), list(range(-90, 91, 30))),
          ("top", 15.0, (10, 20), list(range(-60, 61, 30))),
          ("under", -20.0, (-25, -10), list(range(-60, 61, 30)))]
SLOT = ["#2a78d6", "#eb6834", "#1baf7a"]          # categorical slots 1-3 (fixed order)
SHELL_COLOR = {"eq": SLOT[0], "top": SLOT[1], "under": SLOT[2]}
POLICY_COLOR = {"full": SLOT[0], "three_stop": SLOT[1], "early_stop": SLOT[2]}
INK, MUTED, GRID = "#1f1f1e", "#6b6a66", "#e4e3df"
BLUES = LinearSegmentedColormap.from_list("blues", ["#cde2fb", "#86b6ef", "#3987e5", "#1c5cab", "#0d366b"])
plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                     "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "axes.axisbelow": True,
                     "savefig.dpi": 200, "savefig.bbox": "tight"})


def rows(path):
    if os.path.exists(path):
        return list(csv.DictReader(open(path, newline="")))
    if os.path.exists(path + ".gz"):
        return list(csv.DictReader(gzip.open(path + ".gz", "rt", newline="")))
    return []


def fnum(v, d=math.nan):
    try:
        return float(v)
    except (TypeError, ValueError):
        return d


def res(*p):
    return os.path.join(REPO, "results", *p)


def f1_frontier(out):
    cells = {}
    for p in glob.glob(res("orbit_frontier", "frontier", "*.json")):
        j = json.load(open(p))
        cells[(round(j["d"], 2), round(j["r"], 2))] = j
    if not cells:
        return None
    ds = sorted({k[0] for k in cells} | {0.50})
    rs = sorted({k[1] for k in cells} | {0.40})
    fig, axes = plt.subplots(1, 3, figsize=(10.5, 3.4), constrained_layout=True)
    for ax, (sh, el, _, _) in zip(axes, SHELLS):
        Z = np.full((len(rs), len(ds)), np.nan)
        for (d, r), j in cells.items():
            lim = (j.get("shells", {}).get(sh) or {}).get("limit_deg")
            Z[rs.index(r), ds.index(d)] = lim if lim is not None else 0
        im = ax.imshow(Z, origin="lower", cmap=BLUES, vmin=0, vmax=50, aspect="auto",
                       extent=[ds[0] - 0.025, ds[-1] + 0.025, rs[0] - 0.025, rs[-1] + 0.025])
        for (d, r), j in cells.items():
            z = Z[rs.index(r), ds.index(d)]
            txt = f"±{z:.0f}" if z > 0 else "none"
            if not j.get("base_clear", True):
                txt += "\nblocked"
            ax.text(d, r, txt, ha="center", va="center", fontsize=7, color="white" if z > 30 else INK)
        for d in ds:
            for r in rs:
                if (d, r) not in cells:
                    ax.add_patch(Rectangle((d - 0.025, r - 0.025), 0.05, 0.05, fill=False, hatch="///",
                                           edgecolor=MUTED, lw=0))
                    ax.text(d, r, "skipped", ha="center", va="center", fontsize=6.5, color=MUTED)
        # analytic planar R(theta) <= 0.605 limit, as contours over a fine (d, r) grid
        dd, rr = np.meshgrid(np.linspace(ds[0] - 0.025, ds[-1] + 0.025, 120), np.linspace(rs[0] - 0.025, rs[-1] + 0.025, 120))
        c = np.clip((dd ** 2 + rr ** 2 - 0.605 ** 2) / (2 * dd * rr), -1, 1)
        ax.contour(dd, rr, np.degrees(np.arccos(c)), levels=[90, 105, 120, 135], colors=MUTED, linewidths=0.8,
                   linestyles="--")
        ax.set_title(f"{sh} shell (el {el:+.0f}°)", color=INK, fontsize=9.5, loc="left")
        ax.set_xlabel("stand-off d (m)")
        ax.set_xticks(ds)
        ax.set_yticks(rs)
        ax.grid(False)
    axes[0].set_ylabel("orbit radius r (m)")
    fig.colorbar(im, ax=axes, shrink=0.85, label="collision-free azimuth limit (±deg)")
    fig.suptitle("F1  Reach frontier: observed collision-free azimuth limit (cells) vs the analytic planar limit "
                 "R(θ) ≤ 0.605 m (dashed: 135°, 120°, 105°, 90° from lower left)",
                 x=0.0, ha="left", fontsize=10, color=INK)
    p = os.path.join(out, "F1_reach_frontier.png")
    fig.savefig(p)
    plt.close(fig)
    return p


def f2_semisphere(out):
    data = {sp: rows(res("species", f"{sp}_trials.csv")) for sp in SPECIES}
    if not any(data.values()):
        return None
    fig, axes = plt.subplots(1, len(SPECIES), figsize=(13, 3.0), sharey=True, constrained_layout=True)
    for ax, sp in zip(axes, SPECIES):
        tr = data[sp]
        runs = sorted({t["run_id"] for t in tr}) or [None]
        ax.add_patch(Rectangle((-105, -25), 210, 45, fill=False, hatch="///", edgecolor=GRID, lw=0))
        for sh, el, (lo, hi), azs in SHELLS:
            for az in azs:
                ex = sum(1 for t in tr if t["shell"] == sh and int(float(t["azimuth_deg"])) == az and t.get("exec_time_s"))
                frac = ex / len(runs) if tr else 0
                if frac > 0:
                    ax.add_patch(Rectangle((az - 15, lo), 30, hi - lo, color=BLUES(0.25 + 0.75 * frac), lw=0))
                    ax.text(az, el + 3.2, f"{ex}/{len(runs)}", ha="center", va="center", fontsize=6.5,
                            color="white" if frac > 0.6 else INK)
                ax.plot(az, el, "o", ms=2.5, color=INK if frac > 0 else MUTED, mfc="none" if frac == 0 else INK)
        if not any(t.get("exec_time_s") for t in tr):
            ax.text(0, -3, "no waypoint reachable", ha="center", va="center", fontsize=8, color=INK,
                    bbox={"facecolor": "white", "edgecolor": GRID, "pad": 3})
        ax.set_xlim(-105, 105)
        ax.set_ylim(-27, 22)
        ax.set_xticks([-90, -60, -30, 0, 30, 60, 90])
        ax.set_title(sp, loc="left", color=INK, fontsize=9.5)
        ax.set_xlabel("azimuth θ (deg)")
        ax.grid(False)
    axes[0].set_ylabel("elevation φ (deg)")
    fig.suptitle("F2  Observable semi-sphere per species: shaded = waypoint executed (runs out of 5), hatched = "
                 "not reached", x=0.0, ha="left", fontsize=10, color=INK)
    p = os.path.join(out, "F2_semisphere.png")
    fig.savefig(p)
    plt.close(fig)
    return p


def f3_coverage(out):
    uniq = rows(res("species", "unique.csv"))
    cov = {sp: rows(res("species", f"{sp}_trials_runs.csv")) for sp in SPECIES}
    if not any(cov.values()):
        return None
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 3.4), constrained_layout=True)
    x = np.arange(len(SPECIES))
    w = 0.26
    for i, (sh, *_rest) in enumerate(SHELLS):
        m, lo, hi, um = [], [], [], []
        for sp in SPECIES:
            v = [fnum(r["coverage_fraction"]) for r in cov[sp] if r["shell"] == sh]
            mm, l_, h_ = ci95(v, n=2000) if v else (0, 0, 0)
            m.append(mm)
            lo.append(mm - l_)
            hi.append(h_ - mm)
            u = [int(r["unique_detections"]) for r in uniq if r["shell"] == sh and f"_{sp}_" in r["run_id"]]
            um.append(np.mean(u) if u else 0)
        a1.bar(x + (i - 1) * w, m, w * 0.92, yerr=[lo, hi], color=SHELL_COLOR[sh], label=sh,
               error_kw={"ecolor": INK, "elinewidth": 0.8, "capsize": 2})
        a2.bar(x + (i - 1) * w, um, w * 0.92, color=SHELL_COLOR[sh], label=sh)
    for ax, yl in ((a1, "coverage fraction of the visible half"), (a2, "unique detections per orbit (mean)")):
        ax.set_xticks(x)
        ax.set_xticklabels(SPECIES)
        ax.set_ylabel(yl)
        ax.grid(axis="x", visible=False)
    fig.legend(*a1.get_legend_handles_labels(), frameon=False, ncol=3, loc="upper right", bbox_to_anchor=(1.0, 1.06))
    ti = SPECIES.index("tomato")
    for ax in (a1, a2):
        ax.text(ti, ax.get_ylim()[1] * 0.04, "not\nreachable", ha="center", va="bottom", fontsize=7, color=MUTED)
    a1.set_title("Coverage per shell (mean, 95 % CI over 5 plants)", loc="left", fontsize=9.5, color=INK)
    a2.set_title("Frames whose detection covers a part no other shell detected", loc="left", fontsize=9.5, color=INK)
    fig.suptitle("F3  Species × shell", x=0.0, ha="left", fontsize=10, color=INK)
    p = os.path.join(out, "F3_species_shell.png")
    fig.savefig(p)
    plt.close(fig)
    return p


def f4_pareto(out):
    pol = {m: rows(res("policy", f"{m}.csv")) for m in ("early_stop", "three_stop")}
    if not any(pol.values()):
        return None
    fig, axes = plt.subplots(1, len(SPECIES), figsize=(13, 3.1), sharey=True, sharex=True, constrained_layout=True)
    off = {"full": (6, -10), "early_stop": (8, 4), "three_stop": (8, -14)}
    for ax, sp in zip(axes, SPECIES):
        pts = {}
        es = [r for r in pol["early_stop"] if r["species"] == sp]
        ts = [r for r in pol["three_stop"] if r["species"] == sp]
        if es:
            pts["full"] = ([fnum(r["full_waypoints"]) for r in es], [fnum(r["full_energy"]) for r in es],
                           [fnum(r["full_coverage"]) for r in es])
            pts["early_stop"] = ([fnum(r["policy_waypoints"]) for r in es], [fnum(r["policy_energy"]) for r in es],
                                 [fnum(r["policy_coverage"]) for r in es])
        if ts:
            pts["three_stop"] = ([fnum(r["policy_waypoints"]) for r in ts], [fnum(r["policy_energy"]) for r in ts],
                                 [fnum(r["policy_coverage"]) for r in ts])
        for m, (n, e, c) in pts.items():
            xm, xl, xh = ci95(n, n=2000)
            ym, yl, yh = ci95(e, n=2000)
            cm = np.mean(c)
            ax.errorbar(xm, ym, xerr=[[xm - xl], [xh - xm]], yerr=[[ym - yl], [yh - ym]], fmt="o", ms=7,
                        color=POLICY_COLOR[m], ecolor=POLICY_COLOR[m], elinewidth=1, capsize=2,
                        mec="white", mew=1.5, label={"full": "full sweep", "three_stop": "3-stop", "early_stop": "early stop"}[m])
            ax.annotate(f"cov {cm:.2f}", (xm, ym), textcoords="offset points", xytext=off[m], fontsize=6.5,
                        color=MUTED)
        if not pts:
            ax.text(0.5, 0.5, "no waypoint reachable", transform=ax.transAxes, ha="center", va="center", fontsize=8,
                    color=MUTED)
        ax.set_xlim(0, 8.5)
        ax.set_title(sp, loc="left", fontsize=9.5, color=INK)
        ax.set_xlabel("stops (executed waypoints)")
    axes[0].set_ylabel("energy index per orbit (relative, E3)")
    axes[0].legend(frameon=False, fontsize=7, loc="upper left")
    fig.suptitle("F4  Energy vs stop count: full sweep vs 3-stop (az 0 per shell) vs entropy early stop "
                 "(mean, 95 % CI; coverage annotated)", x=0.0, ha="left", fontsize=10, color=INK)
    p = os.path.join(out, "F4_energy_pareto.png")
    fig.savefig(p)
    plt.close(fig)
    return p


def f5_frames(out, frames_root):
    import cv2

    picks = {}
    for sp in SPECIES:
        fr = rows(res("species", f"{sp}_trials_frames.csv"))
        for sh, *_ in SHELLS:
            cand = [r for r in fr if r["shell"] == sh and fnum(r["detection_confidence"], 0) > 0]
            if cand:
                picks[(sp, sh)] = max(cand, key=lambda r: (abs(fnum(r["azimuth_deg"])) < 1, fnum(r["detection_confidence"])))
    if not picks:
        return None
    try:
        from ultralytics import YOLO
    except ImportError:
        YOLO = None
    fig, axes = plt.subplots(3, len(SPECIES), figsize=(13, 6.6), constrained_layout=True)
    for j, sp in enumerate(SPECIES):
        model = None
        wts = os.path.expanduser(f"~/.cache/go2-yolo/{sp}_yolo11s.pt")
        if YOLO and os.path.exists(wts):
            model = YOLO(wts)
        for i, (sh, *_r) in enumerate(SHELLS):
            ax = axes[i, j]
            ax.axis("off")
            r = picks.get((sp, sh))
            if i == 0:
                ax.set_title(sp, fontsize=9.5, color=INK, loc="left")
            if r is None:
                ax.text(0.5, 0.5, f"{sh}: not reached", ha="center", va="center", color=MUTED, transform=ax.transAxes)
                continue
            img_p = os.path.join(frames_root.format(species=sp), r["run_id"], "frames", r["frame"] + ".jpg")
            if not os.path.exists(img_p):
                ax.text(0.5, 0.5, "frame not on disk", ha="center", va="center", color=MUTED, transform=ax.transAxes)
                continue
            img = cv2.cvtColor(cv2.imread(img_p), cv2.COLOR_BGR2RGB)
            if model is not None:
                for b in model.predict(img_p, conf=0.5, verbose=False)[0].boxes:
                    x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                    cv2.rectangle(img, (x1, y1), (x2, y2), (235, 104, 52), 3)
                    cv2.putText(img, f"{sp} {float(b.conf):.2f}", (x1 + 4, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (235, 104, 52), 2)
            ax.imshow(img)
            ax.text(0.01, 0.99, f"{sh}, az {fnum(r['azimuth_deg']):+.0f}°", transform=ax.transAxes, va="top",
                    fontsize=7.5, color="white", bbox={"facecolor": "#000000aa", "lw": 0, "pad": 2})
    fig.suptitle("F5  Wrist frames per shell per species (az 0), YOLO boxes at conf ≥ 0.5 redrawn; boxes on neighbouring plants included", x=0.0, ha="left", fontsize=10, color=INK)
    p = os.path.join(out, "F5_wrist_frames.jpg")  # photographs: JPEG keeps it under the repo's 500 KB limit
    fig.savefig(p, dpi=130, pil_kwargs={"quality": 85})
    plt.close(fig)
    return p


def f6_servo(out):
    src = [os.path.join(REPO, "docs", "images", "servo_demo", f"fig_{k}.png") for k in ("success", "error", "effort", "recovery")]
    src = [s for s in src if os.path.exists(s)]
    if not src:
        return None
    fig, axes = plt.subplots(2, 2, figsize=(11, 7.5), constrained_layout=True)
    for ax in axes.flat:
        ax.axis("off")
    for ax, s in zip(axes.flat, src):
        ax.imshow(plt.imread(s))
    fig.suptitle("F6  Distance-servo robustness (reused from the servo study, commit 4f36620; not regenerated)",
                 x=0.0, ha="left", fontsize=10, color=INK)
    p = os.path.join(out, "F6_servo_robustness.png")
    fig.savefig(p, dpi=150)
    plt.close(fig)
    return p


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.join(REPO, "docs", "images", "orbit_paper"))
    ap.add_argument("--frames-root", default=os.path.expanduser("~/files/go2-nav2-wifi/ws/log/orbit/species/{species}/runs"))
    ap.add_argument("--only", nargs="*", default=None, help="F1 ... F6")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    jobs = {"F1": lambda: f1_frontier(a.out), "F2": lambda: f2_semisphere(a.out), "F3": lambda: f3_coverage(a.out),
            "F4": lambda: f4_pareto(a.out), "F5": lambda: f5_frames(a.out, a.frames_root), "F6": lambda: f6_servo(a.out)}
    made = defaultdict(str)
    for k, fn in jobs.items():
        if a.only and k not in a.only:
            continue
        made[k] = fn() or "(no input data)"
        print(f"[figures] {k}: {made[k]}")


if __name__ == "__main__":
    main()
