#!/usr/bin/env python3
"""Which true objects the LiDAR-only clusterer found, and which backstop events it caused.

  cluster_attribution.py TRIAL_DIR WORLD_JSON

Reads low_clusters.jsonl (record_json_topic.py), events.jsonl (/gait/status) and
trajectory.csv. Clusters are matched to the true object whose footprint is within
0.15 m of the centroid. A backstop event is a /gait/status sample with
backstop_raised; its source is costmap, cluster or costmap+cluster, and cluster
events are attributed to the true objects in the 1.5 m x 0.3 m corridor then.
"""
import json
import math
import os
import sys
from collections import Counter, defaultdict

import numpy as np

SCAN_D, SCAN_W = 1.5, 0.3


def footprint_dist(o, x, y):
    c, s = math.cos(o.get("yaw", 0.0)), math.sin(o.get("yaw", 0.0))
    dx, dy = x - o["x"], y - o["y"]
    return math.hypot(max(0.0, abs(c * dx + s * dy) - o["sx"] / 2), max(0.0, abs(-s * dx + c * dy) - o["sy"] / 2))


def main():
    d, world = sys.argv[1:3]
    objects = json.load(open(world))["objects"]
    found, heights = Counter(), defaultdict(list)
    unmatched = 0
    n_msgs = 0
    for line in open(os.path.join(d, "low_clusters.jsonl")):
        n_msgs += 1
        for c in json.loads(line)["data"]["clusters"]:
            o = min(objects, key=lambda o: footprint_dist(o, c["x"], c["y"]))
            if footprint_dist(o, c["x"], c["y"]) <= 0.15:
                found[o["name"]] += 1
                heights[o["name"]].append(c["height"])
            else:
                unmatched += 1
    traj = np.genfromtxt(os.path.join(d, "trajectory.csv"), delimiter=",", names=True)
    sources, cluster_objs = Counter(), Counter()
    for line in open(os.path.join(d, "events.jsonl")):
        e = json.loads(line)
        if e.get("kind") != "gait":
            continue
        g = json.loads(e["data"])
        if not g.get("backstop_raised"):
            continue
        src = g.get("backstop_source") or "?"
        sources[src] += 1
        if "cluster" in src:
            k = int(np.clip(np.searchsorted(traj["t"], e["t"]), 0, len(traj) - 1))
            x, y, yaw = float(traj["x"][k]), float(traj["y"][k]), float(traj["yaw"][k])
            cy, sy = math.cos(yaw), math.sin(yaw)
            for o in objects:
                dx, dy = o["x"] - x, o["y"] - y
                reach = max(o["sx"], o["sy"]) / 2
                if -0.45 - reach < cy * dx + sy * dy < SCAN_D + reach and abs(-sy * dx + cy * dy) < SCAN_W + reach:
                    cluster_objs[o["name"]] += 1
    print(json.dumps({
        "cluster_messages": n_msgs,
        "objects_found": {k: {"messages": v, "median_height": round(float(np.median(heights[k])), 3),
                              "true_height": next(o["height"] for o in objects if o["name"] == k)}
                          for k, v in found.most_common()},
        "unmatched_cluster_detections": unmatched,
        "backstop_samples_by_source": dict(sources),
        "cluster_sourced_samples_by_object_in_corridor": dict(cluster_objs.most_common())}, indent=1))


if __name__ == "__main__":
    main()
