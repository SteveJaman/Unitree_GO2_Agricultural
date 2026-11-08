#!/usr/bin/env python3
"""Final cost per object and backstop events, per trial.

  object_costs.py RESULTS_DIR WORLD_JSON [OUT_JSON]

For every trial directory with costmap/*.npz (save_costmap.py, raw costs):
  costmap_cost   median over snapshots of the highest raw cost on the object's
                 true footprint, using snapshots where the whole footprint is
                 inside the local costmap and within 2.5 m of its centre (the robot)
  global_cost    the same in the global costmap, same snapshots
  measured       share of semantic messages for it that carry a LiDAR height
  rpp_collision_aborts  RPP "collision ahead" aborts, with the robot position at each
  label          published semantic label held longest (semantic.jsonl), if any
  vlm            the VLM answer given most often for it, if any
  backstop_events  /gait/status samples with backstop_raised (events.jsonl), and
                 how many distinct episodes (gaps > 1 s split them)
"""
import glob
import json
import math
import os
import sys
from collections import Counter

import numpy as np

COST = {"STEP_OVER": 40, "CAUTION": 200, "AVOID": 254, "UNKNOWN": 0}
MATCH_M = 0.35


def footprint_dist(o, x, y):
    c, s = math.cos(o.get("yaw", 0.0)), math.sin(o.get("yaw", 0.0))
    dx, dy = x - o["x"], y - o["y"]
    ex = max(0.0, abs(c * dx + s * dy) - o["sx"] / 2)
    ey = max(0.0, abs(-s * dx + c * dy) - o["sy"] / 2)
    return math.hypot(ex, ey)


def footprint_cost(grid, origin, res, o, near=None):
    h, w = grid.shape
    cx, cy = near if near else (origin[0] + w * res / 2, origin[1] + h * res / 2)
    if math.hypot(o["x"] - cx, o["y"] - cy) > 2.5:
        return None
    r = math.hypot(o["sx"], o["sy"]) / 2
    i0, i1 = int((o["x"] - r - origin[0]) / res), int((o["x"] + r - origin[0]) / res)
    j0, j1 = int((o["y"] - r - origin[1]) / res), int((o["y"] + r - origin[1]) / res)
    if i0 < 0 or j0 < 0 or i1 >= w or j1 >= h:
        return None
    best = 0
    for j in range(j0, j1 + 1):
        for i in range(i0, i1 + 1):
            px, py = origin[0] + (i + 0.5) * res, origin[1] + (j + 0.5) * res
            if footprint_dist(o, px, py) == 0.0:
                best = max(best, int(grid[j, i]))
    return best


def trial_costs(d, objects):
    snaps = [np.load(f) for f in sorted(glob.glob(os.path.join(d, "costmap", "*.npz")))]
    labels, vlm = {o["name"]: Counter() for o in objects}, {o["name"]: Counter() for o in objects}
    meas = {o["name"]: [0, 0] for o in objects}
    sem = os.path.join(d, "semantic.jsonl")
    if os.path.exists(sem):
        for line in open(sem):
            for ob in json.loads(line).get("obstacles", []):
                near = min(objects, key=lambda o: footprint_dist(o, ob["x"], ob["y"]))
                if footprint_dist(near, ob["x"], ob["y"]) < MATCH_M:
                    labels[near["name"]][ob["label"]] += 1
                    meas[near["name"]][0] += ob.get("height") is not None
                    meas[near["name"]][1] += 1
                    if ob.get("vlm") and ob.get("label_age_s") is not None:
                        vlm[near["name"]][ob["vlm"]] += 1
    rows = []
    for o in objects:
        cs, gs = [], []
        for s in snaps:
            c = footprint_cost(s["grid"], s["origin"], float(s["resolution"]), o)
            if c is None:
                continue
            cs.append(c)
            if "global_grid" in s:
                g, r = s["grid"], float(s["resolution"])
                centre = (s["origin"][0] + g.shape[1] * r / 2, s["origin"][1] + g.shape[0] * r / 2)
                gc = footprint_cost(s["global_grid"], s["global_origin"], float(s["global_resolution"]), o, centre)
                if gc is not None:
                    gs.append(gc)
        lab = labels[o["name"]].most_common(1)
        rows.append({"object": o["name"], "height": o["height"], "band": o.get("band"),
                     "snapshots": len(cs), "costmap_cost": int(np.median(cs)) if cs else None,
                     "global_cost": int(np.median(gs)) if gs else None,
                     "measured": round(meas[o["name"]][0] / meas[o["name"]][1], 2) if meas[o["name"]][1] else None,
                     "label": lab[0][0] if lab else None, "label_cost": COST.get(lab[0][0]) if lab else None,
                     "vlm": vlm[o["name"]].most_common(1)[0][0] if vlm[o["name"]] else None})
    ts = []
    ev = os.path.join(d, "events.jsonl")
    if os.path.exists(ev):
        for line in open(ev):
            e = json.loads(line)
            if e.get("kind") == "gait" and json.loads(e["data"]).get("backstop_raised"):
                ts.append(e["t"])
    episodes = sum(1 for k, t in enumerate(ts) if k == 0 or t - ts[k - 1] > 1.0)
    aborts = []
    if os.path.exists(ev):
        traj = np.genfromtxt(os.path.join(d, "trajectory.csv"), delimiter=",", names=True)
        for line in open(ev):
            e = json.loads(line)
            if e.get("kind") == "nav2" and "collision ahead" in e.get("msg", ""):
                k = int(np.clip(np.searchsorted(traj["t"], e["t"]), 0, len(traj) - 1))
                aborts.append((round(float(traj["x"][k]), 2), round(float(traj["y"][k]), 2)))
    return {"objects": rows, "backstop_samples": len(ts), "backstop_episodes": episodes,
            "rpp_collision_aborts": len(aborts), "abort_positions": aborts}


def main():
    res, world = sys.argv[1:3]
    objects = json.load(open(world))["objects"]
    out = {}
    for d in sorted(glob.glob(os.path.join(res, "*"))):
        if os.path.isdir(os.path.join(d, "costmap")):
            out[os.path.basename(d)] = trial_costs(d, objects)
    if len(sys.argv) > 3:
        json.dump(out, open(sys.argv[3], "w"), indent=1)
    for name, t in out.items():
        print(f"== {name}: backstop samples {t['backstop_samples']}, episodes {t['backstop_episodes']}, "
              f"RPP collision aborts {t['rpp_collision_aborts']} at {t['abort_positions']}")
        for r in t["objects"]:
            if r["snapshots"] or r["label"]:
                print(f"  {r['object']:15s} h={r['height']:.3f} band={r['band'] or '-':8s} "
                      f"local={r['costmap_cost']} global={r['global_cost']} (n={r['snapshots']}) "
                      f"measured={r['measured']} label={r['label']} vlm={r['vlm']}")


if __name__ == "__main__":
    main()
