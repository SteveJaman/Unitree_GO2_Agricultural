#!/usr/bin/env python3
"""Score traversability trials against the world's true geometry.

  analyze.py RESULTS_DIR OUT_DIR WORLD_SDF

Writes OUT_DIR/{trials.csv, stats.md, per_trial.md, graph_data.md, data/*.csv, *.png}.
Runs in the go2-3d image (matplotlib, scipy).
"""
import csv
import heapq
import json
import math
import os
import re
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon, Rectangle  # noqa: E402
import numpy as np  # noqa: E402
from scipy import ndimage, stats  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "scripts", "nodes"))
from sim_box_detector import corners, load_objects  # noqa: E402

BLUE, ORANGE = "#2a78d6", "#eb6834"
INK, INK2, MUTED, GRID, AXIS, SURFACE = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#fcfcfb"
BAND = {"step_over": "#0ca30c", "caution": "#fab219", "blocked": "#d03b3b"}
BAND_LABEL = {"step_over": "Step over (≤ 0.15 m)", "caution": "Caution (0.15–0.25 m)", "blocked": "Blocked (≥ 0.25 m)"}
GAIT_ORDER = ["NORMAL", "HIGH_STEP", "CAREFUL", "STOP"]
GAIT_COLOR = {"NORMAL": "#c3c2b7", "HIGH_STEP": "#1baf7a", "CAREFUL": "#eda100", "STOP": "#6250d6"}
MODES = ["baseline", "traversability"]
MODE_COLOR = {"baseline": BLUE, "traversability": ORANGE}
FOOT_HIGH, MARGIN, LETHAL = 0.10, 0.02, 0.25
BODY = (0.70, 0.31)
SCAN_D, SCAN_W = 1.5, 0.3
XLIM, YLIM = (-2.3, 12.3), (-4.8, 4.8)

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "text.color": INK, "xtick.color": MUTED,
    "ytick.color": MUTED, "axes.grid": False, "grid.color": GRID, "grid.linewidth": 0.7,
    "font.size": 10, "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False,
    "axes.titlesize": 11, "axes.titlelocation": "left",
})


def band(height):
    if height <= 0.15:
        return "step_over"
    return "caution" if height < 0.25 else "blocked"


def gait_for_height(h):
    if h + MARGIN <= FOOT_HIGH:
        return "HIGH_STEP"
    return "CAREFUL" if h < LETHAL else "STOP"


def load_walls(path):
    text = re.sub(r"@\w+@", "ogre2", open(path).read())
    walls = []
    for m in ET.fromstring(text).iter("model"):
        if m.get("name", "").startswith("wall_"):
            x, y, *_ = (float(v) for v in m.find("pose").text.split())
            sx, sy, _ = (float(v) for v in m.find("link/collision/geometry/box/size").text.split())
            walls.append({"name": m.get("name"), "x": x, "y": y, "sx": sx, "sy": sy})
    return walls


def footprint(o):
    c, s = math.cos(o["yaw"]), math.sin(o["yaw"])
    hx, hy = o["sx"] / 2, o["sy"] / 2
    return [(o["x"] + c * a - s * b, o["y"] + s * a + c * b)
            for a, b in ((hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy))]


class Raster:
    """Occupancy of the true footprints at 2 cm, with a distance transform."""
    RES = 0.02

    def __init__(self, objects, walls, keep):
        self.x0, self.y0 = XLIM[0], YLIM[0]
        self.nx = int((XLIM[1] - XLIM[0]) / self.RES)
        self.ny = int((YLIM[1] - YLIM[0]) / self.RES)
        yy, xx = np.mgrid[0:self.ny, 0:self.nx]
        wx, wy = self.x0 + (xx + 0.5) * self.RES, self.y0 + (yy + 0.5) * self.RES
        occ = np.zeros((self.ny, self.nx), bool)
        for o in objects:
            if keep(o):
                c, s = math.cos(o["yaw"]), math.sin(o["yaw"])
                dx, dy = wx - o["x"], wy - o["y"]
                occ |= (np.abs(c * dx + s * dy) <= o["sx"] / 2) & (np.abs(-s * dx + c * dy) <= o["sy"] / 2)
        for w in walls:
            occ |= (np.abs(wx - w["x"]) <= w["sx"] / 2) & (np.abs(wy - w["y"]) <= w["sy"] / 2)
        self.occ = occ
        self.dist = ndimage.distance_transform_edt(~occ) * self.RES

    def clearance(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        hx, hy = BODY[0] / 2, BODY[1] / 2
        lx = np.concatenate([np.linspace(-hx, hx, 15), np.linspace(-hx, hx, 15),
                             np.full(7, -hx), np.full(7, hx)])
        ly = np.concatenate([np.full(15, -hy), np.full(15, hy),
                             np.linspace(-hy, hy, 7), np.linspace(-hy, hy, 7)])
        px, py = x + c * lx - s * ly, y + s * lx + c * ly
        i = np.clip(((px - self.x0) / self.RES).astype(int), 0, self.nx - 1)
        j = np.clip(((py - self.y0) / self.RES).astype(int), 0, self.ny - 1)
        return float(self.dist[j, i].min())

    def shortest(self, start, goal, margin=0.3):
        """Dijkstra on an 8-connected 10 cm grid over free space with a margin."""
        step = 5
        free = (self.dist > margin)[::step, ::step]
        res = self.RES * step
        ny, nx = free.shape

        def cell(p):
            return (int(np.clip((p[1] - self.y0) / res, 0, ny - 1)),
                    int(np.clip((p[0] - self.x0) / res, 0, nx - 1)))
        s, g = cell(start), cell(goal)
        best = {s: 0.0}
        heap = [(0.0, s)]
        while heap:
            d, u = heapq.heappop(heap)
            if u == g:
                return d * res
            if d > best[u]:
                continue
            for dj in (-1, 0, 1):
                for di in (-1, 0, 1):
                    v = (u[0] + dj, u[1] + di)
                    if (dj or di) and 0 <= v[0] < ny and 0 <= v[1] < nx and (free[v] or v == g):
                        nd = d + math.hypot(dj, di)
                        if nd < best.get(v, 1e9):
                            best[v] = nd
                            heapq.heappush(heap, (nd, v))
        return math.hypot(goal[0] - start[0], goal[1] - start[1])


def load_trial(path):
    t = {"name": os.path.basename(path), "mode": re.sub(r"\d+$", "", os.path.basename(path))}
    rows = list(csv.DictReader(open(os.path.join(path, "trajectory.csv"))))
    t["traj"] = np.array([[float(r[k]) for k in ("t", "x", "y", "yaw")] for r in rows])
    t["legs"] = json.load(open(os.path.join(path, "legs.json")))
    t["rtf"] = float(open(os.path.join(path, "rtf.txt")).read())
    log = open(os.path.join(path, "goal_nav.log")).read()
    m = re.search(r"\[(\d+\.\d+)\] \[go2_goal_pose_nav\]: Goal received", log)
    t["t0"] = float(m.group(1)) if m else 0.0
    t["events"] = [json.loads(line) for line in open(os.path.join(path, "events.jsonl")) if line.strip()]
    contacts = [json.loads(line) for line in open(os.path.join(path, "contacts.jsonl")) if line.strip()]
    for c in contacts:
        c["t_rel"] = c["t"] - t["t0"]
    t["contacts"] = contacts
    return t


def pose_at(traj, time):
    i = int(np.clip(np.searchsorted(traj[:, 0], time), 0, len(traj) - 1))
    return traj[i, 1:]


def contact_episodes(trial, gap=2.0):
    """One episode per object per continuous touch: {object, t0, t1, n}."""
    per = defaultdict(list)
    for c in trial["contacts"]:
        per[c["other"].split("::")[0]].append(c["t_rel"])
    eps = []
    for name, ts in per.items():
        ts.sort()
        start = prev = ts[0]
        for x in ts[1:] + [1e18]:
            if x - prev > gap:
                eps.append({"object": name, "t0": start, "t1": prev})
                start = x
            prev = x
    return sorted(eps, key=lambda e: e["t0"])


def path_len(traj, a, b):
    seg = traj[(traj[:, 0] >= a) & (traj[:, 0] <= b)]
    return float(np.hypot(*np.diff(seg[:, 1:3], axis=0).T).sum()) if len(seg) > 1 else 0.0


def truth_gait(pose, objects):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    hs = []
    for o in objects:
        dx, dy = o["x"] - x, o["y"] - y
        fw, lat = c * dx + s * dy, -s * dx + c * dy
        reach = max(o["sx"], o["sy"]) / 2
        if -0.45 - reach < fw < SCAN_D + reach and abs(lat) < SCAN_W + reach:
            hs.append(o["height"])
    return gait_for_height(max(hs)) if hs else "NORMAL"


def gait_series(trial):
    return [(e["t"], json.loads(e["data"])) for e in trial["events"] if e["kind"] == "gait"]


def box_snapshots(trial):
    return [(e["t"], json.loads(e["data"])["boxes"]) for e in trial["events"] if e["kind"] == "boxes"]


def match_box(o, boxes):
    best, dist = None, 1.0
    for b in boxes:
        d = math.hypot(b["x"] - o["x"], b["y"] - o["y"])
        if b["class"] == o["class"] and d < dist:
            best, dist = b, d
    return best, dist


def trial_metrics(t, objects, tall, low_all):
    legs, traj = t["legs"], t["traj"]
    spl, walked = [], 0.0
    leg_rows = []
    for leg in legs:
        d = path_len(traj, leg["t_start"], leg["t_end"])
        walked += d
        short = tall.shortest((leg["start_x"], leg["start_y"]), (leg["goal_x"], leg["goal_y"]))
        ok = leg["result"] == "reached"
        s = short / max(d, short) if ok else 0.0
        spl.append(s)
        leg_rows.append({"trial": t["name"], "mode": t["mode"], "goal": leg["goal"], "result": leg["result"],
                         "time_s": leg["time_s"], "path_m": round(d, 2), "shortest_m": round(short, 2),
                         "spl": round(s, 2), "resends": leg["resends"]})
    samples = traj[::4]
    clear_tall = [tall.clearance(*p[1:]) for p in samples]
    eps = contact_episodes(t)
    msgs = Counter(e["msg"] for e in t["events"] if e["kind"] == "nav2")
    aborts = sum(v for k, v in msgs.items() if "collision ahead" in k)
    plan_fail = sum(v for k, v in msgs.items() if "Starting point in lethal" in k)
    return {"trial": t["name"], "mode": t["mode"],
            "reached": sum(1 for l in legs if l["result"] == "reached"),
            "time_s": round(sum(l["time_s"] for l in legs), 1),
            "walked_m": round(walked, 2), "spl": round(float(np.mean(spl)), 3),
            "contact_episodes": len(eps),
            "objects_hit": len({e["object"] for e in eps}),
            "collision_aborts": aborts, "start_in_lethal": plan_fail,
            "resends": sum(l["resends"] for l in legs),
            "min_clear_tall_m": round(min(clear_tall), 3),
            "rtf": t["rtf"]}, leg_rows, clear_tall


def contact_rows(t, objects):
    """Per first touch of each object: what the system knew before contact."""
    by_name = {o["name"]: o for o in objects}
    gait = gait_series(t)
    snaps = box_snapshots(t)
    rows = []
    seen = set()
    for ep in contact_episodes(t):
        o = by_name.get(ep["object"])
        if o is None or ep["object"] in seen:
            continue
        seen.add(ep["object"])
        row = {"trial": t["name"], "mode": t["mode"], "object": o["name"], "true_h": round(o["height"], 3),
               "band": band(o["height"]), "t_contact": round(ep["t0"], 1),
               "right_gait": gait_for_height(o["height"]), "gait": "", "costmap_gait": "",
               "known": False, "lead_s": "", "lead_m": "", "est_h": ""}
        if gait:
            before = [g for g in gait if g[0] <= ep["t0"]]
            if before:
                row["gait"], row["costmap_gait"] = before[-1][1]["gait"], before[-1][1]["costmap_gait"]
        for ts, boxes in snaps:
            if ts > ep["t0"]:
                break
            b, _ = match_box(o, boxes)
            if b is not None:
                if not row["known"]:
                    p = pose_at(t["traj"], ts)
                    row.update(known=True, lead_s=round(ep["t0"] - ts, 1),
                               lead_m=round(math.hypot(p[0] - o["x"], p[1] - o["y"]), 2))
                row["est_h"] = b["height"]
        rows.append(row)
    return rows


def box_rows(trials, objects):
    rows = []
    for t in trials:
        snaps = box_snapshots(t)
        last = snaps[-1][1] if snaps else []
        for o in objects:
            b, d = match_box(o, last)
            rows.append({"trial": t["name"], "object": o["name"], "class": o["class"],
                         "true_h": round(o["height"], 3), "true_band": band(o["height"]),
                         "seen": b is not None,
                         "est_h": b["height"] if b else "", "err_h": round(b["height"] - o["height"], 3) if b else "",
                         "err_xy": round(d, 3) if b else "", "est_band": b["traverse"] if b else "",
                         "n_points": b["n_points"] if b else "", "n_obs": b["n_obs"] if b else ""})
    return rows


def track_audit(trials, objects):
    """Every final track, including duplicates: is its band right for the nearest object of its class?"""
    rows = []
    for t in trials:
        snaps = box_snapshots(t)
        for b in (snaps[-1][1] if snaps else []):
            same = [o for o in objects if o["class"] == b["class"]]
            o = min(same, key=lambda s: math.hypot(s["x"] - b["x"], s["y"] - b["y"]))
            true_band = band(o["height"])
            rank = {"step_over": 0, "caution": 1, "blocked": 2}
            rows.append({"trial": t["name"], "track_class": b["class"], "object": o["name"],
                         "true_h": round(o["height"], 3), "est_h": b["height"], "true_band": true_band,
                         "est_band": b["traverse"], "dist_m": round(math.hypot(o["x"] - b["x"], o["y"] - b["y"]), 2),
                         "n_obs": b["n_obs"], "n_points": b["n_points"],
                         "unsafe": rank[b["traverse"]] < rank[true_band]})
    return rows


def gait_agreement(trials, objects):
    rows = []
    for t in trials:
        for ts, g in gait_series(t):
            truth = truth_gait(pose_at(t["traj"], ts), objects)
            rows.append({"trial": t["name"], "t": ts, "truth": truth, "boxes": g["gait"], "costmap": g["costmap_gait"]})
    return rows


def write_csv(path, rows):
    if not rows:
        open(path, "w").close()
        return
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def md_table(rows, keys=None):
    if not rows:
        return "(none)\n"
    keys = keys or list(rows[0].keys())
    out = ["| " + " | ".join(keys) + " |", "|" + "---|" * len(keys)]
    out += ["| " + " | ".join(str(r.get(k, "")) for k in keys) + " |" for r in rows]
    return "\n".join(out) + "\n"


def draw_world(ax, objects, walls, labels=False):
    for w in walls:
        ax.add_patch(Rectangle((w["x"] - w["sx"] / 2, w["y"] - w["sy"] / 2), w["sx"], w["sy"], fc="#898781", ec="none"))
    for o in objects:
        ax.add_patch(Polygon(footprint(o), fc=BAND[band(o["height"])], ec=SURFACE, lw=0.6, alpha=0.9))
        if labels:
            off = {"cable_1": (-40, -46), "barrier_center": (-46, 4), "hose_1": (12, 30),
                   "pipe_low": (8, -40), "toolbox_1": (6, -22), "cable_2": (34, 14)}.get(o["name"], (0, 9))
            ax.annotate(f"{o['name']}\n{o['height']:.2f} m", (o["x"], o["y"]), xytext=off,
                        textcoords="offset points", ha="center", fontsize=6.5, color=INK2)
    ax.set_xlim(*XLIM)
    ax.set_ylim(*YLIM)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")


def band_legend(ax, **kw):
    from matplotlib.patches import Patch
    return ax.legend(handles=[Patch(fc=BAND[b], label=BAND_LABEL[b]) for b in BAND], **kw)


def fig_floorplan(out, objects, walls, goals):
    fig, ax = plt.subplots(figsize=(11, 6.2))
    draw_world(ax, objects, walls, labels=True)
    for i, (x, y, n) in enumerate(goals):
        ax.plot(x, y, "o", ms=9, mfc=BLUE, mec=SURFACE, mew=1.5, zorder=5)
        ax.annotate(n, (x, y), xytext=(6, -13), textcoords="offset points", fontsize=8, color=INK)
    ax.plot(-4.5, 0, marker=None)
    ax.annotate("The old spawn (−4.5, 0) was west of this wall, outside the arena", (-1.8, -4.2), fontsize=8, color=INK2)
    band_legend(ax, loc="upper center", bbox_to_anchor=(0.5, 1.09), ncol=3, fontsize=8)
    fig.savefig(os.path.join(out, "floorplan.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)


def fig_trajectories(out, trials, objects, walls):
    fig, axes = plt.subplots(2, 1, figsize=(10, 10.5))
    for ax, mode in zip(axes, MODES):
        draw_world(ax, objects, walls)
        ts = [t for t in trials if t["mode"] == mode]
        for t in ts:
            ax.plot(t["traj"][:, 1], t["traj"][:, 2], lw=1.6, color=MODE_COLOR[mode], alpha=0.75)
            ax.plot(*t["traj"][-1, 1:3], "o", ms=8, mfc=MODE_COLOR[mode], mec=SURFACE, mew=1.5, zorder=6)
            for c in contact_points(t):
                ax.plot(c[0], c[1], "X", ms=9, mfc=INK, mec=SURFACE, mew=1, zorder=7)
        ax.set_title(f"{mode} (n = {len(ts)}): dot marks where the trial ended, X marks first contact with an object")
    band_legend(axes[0], loc="upper right", fontsize=7.5)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "trajectories.png"), dpi=150)
    plt.close(fig)


def contact_points(t):
    pts, seen = [], set()
    for c in t["contacts"]:
        k = c["other"].split("::")[0]
        if k not in seen:
            seen.add(k)
            pts.append((c.get("x", 0.0), c.get("y", 0.0)))
    return pts


def fig_legs(out, legs):
    goals = list(dict.fromkeys(l["goal"] for l in legs))
    fig, ax = plt.subplots(figsize=(7.5, 3.4))
    w = 0.34
    for k, mode in enumerate(MODES):
        ns = [sum(1 for l in legs if l["goal"] == g and l["mode"] == mode and l["result"] == "reached") for g in goals]
        tot = [sum(1 for l in legs if l["goal"] == g and l["mode"] == mode) for g in goals]
        xs = np.arange(len(goals)) + (k - 0.5) * (w + 0.03)
        ax.bar(xs, ns, w, color=MODE_COLOR[mode], label=mode)
        for x, n, m in zip(xs, ns, tot):
            ax.annotate(f"{n}/{m}", (x, n), xytext=(0, 3), textcoords="offset points", ha="center", fontsize=9, color=INK2)
    ax.set_xticks(range(len(goals)), goals)
    ax.set_ylabel("trials reaching goal")
    ax.set_ylim(0, max(l for l in [sum(1 for x in legs if x["goal"] == goals[0] and x["mode"] == MODES[0])]) + 0.8)
    ax.yaxis.set_major_locator(plt.MaxNLocator(integer=True))
    ax.grid(axis="y")
    ax.set_axisbelow(True)
    ax.legend(loc="upper right", ncol=2)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "goals.png"), dpi=150)
    plt.close(fig)


def fig_boxes(out, rows):
    pts = [r for r in rows if r["seen"]]
    if not pts:
        return
    fig, ax = plt.subplots(figsize=(6.2, 5.4))
    lim = max(max(r["true_h"] for r in pts), max(float(r["est_h"]) for r in pts)) + 0.05
    ax.axhspan(0, 0.15, color=BAND["step_over"], alpha=0.08, lw=0)
    ax.axhspan(0.15, 0.25, color=BAND["caution"], alpha=0.10, lw=0)
    ax.axhspan(0.25, lim, color=BAND["blocked"], alpha=0.08, lw=0)
    ax.plot([0, lim], [0, lim], color=MUTED, lw=1)
    ax.annotate("perfect estimate", (lim * 0.62, lim * 0.68), color=MUTED, fontsize=8, rotation=38)
    rng = np.random.default_rng(0)
    for r in pts:
        ax.plot(r["true_h"] + rng.uniform(-0.004, 0.004), float(r["est_h"]), "o", ms=7, mfc=BAND[r["true_band"]],
                mec=SURFACE, mew=1.2, alpha=0.9)
    ax.set_xlim(0, lim)
    ax.set_ylim(0, lim)
    ax.set_xlabel("true height (m)")
    ax.set_ylabel("height from fused 3D box (m)")
    ax.set_title("One point per object per trial (final box)")
    band_legend(ax, loc="lower right", fontsize=8, title="Point color: true band", title_fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "box_accuracy.png"), dpi=150)
    plt.close(fig)


def fig_gait_agreement(out, rows):
    if not rows:
        return
    fig, axes = plt.subplots(1, 2, figsize=(9.2, 3.8), sharey=True)
    for ax, key, title in zip(axes, ("boxes", "costmap"),
                              ("3D boxes (camera + LiDAR)", "Costmap only (Phase 4)")):
        m = np.zeros((4, 4), int)
        for r in rows:
            m[GAIT_ORDER.index(r["truth"]), GAIT_ORDER.index(r[key])] += 1
        share = m / np.maximum(m.sum(axis=1, keepdims=True), 1)
        ax.imshow(share, cmap=matplotlib.colors.LinearSegmentedColormap.from_list("b", ["#fcfcfb", "#2a78d6", "#104281"]), vmin=0, vmax=1)
        for i in range(4):
            for j in range(4):
                if m[i, j]:
                    ax.text(j, i, f"{share[i, j] * 100:.0f}%\n({m[i, j]})", ha="center", va="center", fontsize=8,
                            color="#ffffff" if share[i, j] > 0.55 else INK)
        ax.set_xticks(range(4), GAIT_ORDER, fontsize=8)
        ax.set_yticks(range(4), GAIT_ORDER, fontsize=8)
        ax.set_xlabel("gait chosen")
        ax.tick_params(length=0)
        for s in ax.spines.values():
            s.set_visible(False)
        acc = sum(m[i, i] for i in range(4)) / max(m.sum(), 1)
        ax.set_title(f"{title}: {acc * 100:.0f}% match")
    axes[0].set_ylabel("right gait (true geometry)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "gait_agreement.png"), dpi=150)
    plt.close(fig)


def fig_gait_timeline(out, trial, objects):
    g = gait_series(trial)
    if not g:
        return
    tt = np.array([x[0] for x in g])
    fig, ax = plt.subplots(figsize=(9.5, 3.4))
    low = [o for o in objects]
    traj = trial["traj"]
    d = []
    for row in traj[::4]:
        px, py = row[1], row[2]
        d.append(min(math.hypot(px - o["x"], py - o["y"]) for o in low))
    ax.plot(traj[::4, 0], d, color=INK2, lw=1.4)
    ymax = max(d) * 1.05
    for (t0, s0), t1 in zip(g, list(tt[1:]) + [tt[-1]]):
        ax.axvspan(t0, t1, ymin=0, ymax=0.1, color=GAIT_COLOR[s0["gait"]], lw=0)
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(fc=GAIT_COLOR[k], label=k) for k in GAIT_ORDER], loc="upper center",
              bbox_to_anchor=(0.5, 1.2), ncol=4, fontsize=8)
    for ep in contact_episodes(trial):
        ax.axvline(ep["t0"], color=INK, lw=1, ls=":")
        ax.annotate(ep["object"], (ep["t0"], ymax * 0.92), rotation=90, fontsize=7, ha="right", va="top", color=INK2)
    ax.set_ylim(0, ymax)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("distance to nearest\nobject centre (m)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "gait_timeline.png"), dpi=150)
    plt.close(fig)


def summary_stats(ms):
    keys = [("reached", "Goals reached (of 3)"), ("time_s", "Total time (s)"), ("walked_m", "Distance walked (m)"),
            ("spl", "SPL"), ("contact_episodes", "Contact episodes"), ("objects_hit", "Objects touched"),
            ("collision_aborts", "Nav2 collision aborts"), ("start_in_lethal", "Plans refused: start in lethal space"),
            ("resends", "Goals re-sent"), ("min_clear_tall_m", "Closest to a tall object or wall (m)"), ("rtf", "Sim real-time factor")]
    a = [m for m in ms if m["mode"] == MODES[0]]
    b = [m for m in ms if m["mode"] == MODES[1]]
    rows = []
    for k, label in keys:
        x, y = np.array([m[k] for m in a], float), np.array([m[k] for m in b], float)
        p = ""
        if len(x) > 1 and len(y) > 1 and (x.std() + y.std()) > 0:
            p = f"{stats.ttest_ind(x, y, equal_var=False).pvalue:.3f}"
        f = lambda v: "" if len(v) == 0 else f"{v.mean():.2f} ± {v.std(ddof=1) if len(v) > 1 else 0:.2f} [{v.min():.2f}–{v.max():.2f}]"  # noqa: E731
        rows.append({"Metric": label, f"`baseline` (n={len(a)})": f(x), f"`traversability` (n={len(b)})": f(y), "p (Welch t)": p})
    return rows


def main():
    res, out, world = sys.argv[1:4]
    os.makedirs(os.path.join(out, "data"), exist_ok=True)
    objects, walls = load_objects(world), load_walls(world)
    names = sorted(d for d in os.listdir(res) if os.path.exists(os.path.join(res, d, "legs.json")))
    trials = [load_trial(os.path.join(res, d)) for d in names]
    tall = Raster(objects, walls, lambda o: o["height"] > 0.15)
    ms, legs, clear = [], [], {}
    for t in trials:
        m, leg_rows, c = trial_metrics(t, objects, tall, None)
        ms.append(m)
        legs += leg_rows
        clear[t["name"]] = c
    contacts = [r for t in trials for r in contact_rows(t, objects)]
    trav = [t for t in trials if t["mode"] == "traversability"]
    boxes = box_rows(trav, objects)
    gait = gait_agreement(trav, objects)
    audit = track_audit(trav, objects)
    write_csv(os.path.join(out, "data", "tracks.csv"), audit)
    print(json.dumps({"tracks": len(audit), "per_trial": round(len(audit) / max(len(trav), 1), 1),
                      "band_wrong": sum(r["true_band"] != r["est_band"] for r in audit),
                      "unsafe_too_low": sum(r["unsafe"] for r in audit),
                      "unsafe_by_object": Counter(r["object"] for r in audit if r["unsafe"]).most_common()}))

    per = defaultdict(list)
    for r in boxes:
        if r["seen"]:
            per[r["object"]].append(r)
    obj_rows = []
    for o in objects:
        rs = per.get(o["name"], [])
        hs = [float(r["est_h"]) for r in rs]
        obj_rows.append({"object": o["name"], "class": o["class"], "true_h": round(o["height"], 3),
                         "true_band": band(o["height"]), "seen_in_trials": f"{len(rs)}/{len(trav)}",
                         "mean_est_h": round(float(np.mean(hs)), 3) if hs else "",
                         "mean_err_xy": round(float(np.mean([r["err_xy"] for r in rs])), 3) if rs else "",
                         "band_correct": f"{sum(r['est_band'] == r['true_band'] for r in rs)}/{len(rs)}" if rs else "",
                         "mean_points": round(float(np.mean([r["n_points"] for r in rs])), 1) if rs else ""})

    write_csv(os.path.join(out, "trials.csv"), ms)
    write_csv(os.path.join(out, "data", "legs.csv"), legs)
    write_csv(os.path.join(out, "data", "contacts.csv"), contacts)
    write_csv(os.path.join(out, "data", "boxes.csv"), boxes)
    write_csv(os.path.join(out, "data", "box_objects.csv"), obj_rows)
    write_csv(os.path.join(out, "data", "gait_samples.csv"), gait)

    seen = [r for r in boxes if r["seen"]]
    n = len(gait)
    fusion = {
        "box_observations": len(seen),
        "mean_abs_height_err_m": round(float(np.mean([abs(r["err_h"]) for r in seen])), 3) if seen else None,
        "median_abs_height_err_m": round(float(np.median([abs(r["err_h"]) for r in seen])), 3) if seen else None,
        "mean_xy_err_m": round(float(np.mean([r["err_xy"] for r in seen])), 3) if seen else None,
        "band_correct": sum(r["est_band"] == r["true_band"] for r in seen),
        "gait_samples": n,
        "gait_match_boxes": round(sum(r["truth"] == r["boxes"] for r in gait) / max(n, 1), 3),
        "gait_match_costmap": round(sum(r["truth"] == r["costmap"] for r in gait) / max(n, 1), 3),
    }
    json.dump(fusion, open(os.path.join(out, "data", "fusion_summary.json"), "w"), indent=1)

    with open(os.path.join(out, "stats.md"), "w") as f:
        f.write("Per mode over trials: mean ± SD [min–max]; p from Welch's t-test.\n\n")
        f.write(md_table(summary_stats(ms)) + "\n")
        for mode in MODES:
            ls = [l for l in legs if l["mode"] == mode]
            f.write(f"`{mode}` legs reached: {sum(l['result'] == 'reached' for l in ls)}/{len(ls)}\n\n")
        a = [sum(l["result"] == "reached" for l in legs if l["mode"] == m) for m in MODES]
        c = [sum(l["mode"] == m for l in legs) for m in MODES]
        if min(c):
            f.write(f"Fisher exact, per leg: p = {stats.fisher_exact([[a[0], c[0] - a[0]], [a[1], c[1] - a[1]]])[1]:.3f}\n\n")
        f.write("Fusion summary: " + json.dumps(fusion) + "\n")
    with open(os.path.join(out, "per_trial.md"), "w") as f:
        f.write(md_table(ms))
    with open(os.path.join(out, "graph_data.md"), "w") as f:
        f.write("### Every leg (goals.png)\n\n" + md_table(legs))
        f.write("\n### First contact with each object (trajectories.png)\n\n" + md_table(contacts))
        f.write("\n### Fused boxes per object (box_accuracy.png)\n\n" + md_table(obj_rows))

    goals = [(5, 0, "goal 1"), (10, 0, "goal 2"), (0, 0, "start / goal 3")]
    fig_floorplan(out, objects, walls, goals)
    fig_trajectories(out, trials, objects, walls)
    fig_legs(out, legs)
    fig_boxes(out, boxes)
    fig_gait_agreement(out, gait)
    if trav:
        fig_gait_timeline(out, trav[0], objects)
    print(md_table(summary_stats(ms)))
    print(json.dumps(fusion, indent=1))
    print(md_table(obj_rows))
    print(md_table(contacts, ["trial", "object", "true_h", "t_contact", "right_gait", "gait", "costmap_gait", "known", "lead_s", "lead_m", "est_h"]))


if __name__ == "__main__":
    main()
