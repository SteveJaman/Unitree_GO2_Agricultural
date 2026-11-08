#!/usr/bin/env python3
"""Local costmap snapshot with the semantic obstacles and the true objects.

  fig_semantic_costmap.py TRIAL_DIR WORLD_JSON OUT_PNG [SNAPSHOT]

Without SNAPSHOT, the snapshot with the most semantic AVOID/CAUTION marks.
"""
import glob
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Circle, Polygon  # noqa: E402

COLOR = {"AVOID": "#d03b3b", "CAUTION": "#eda100", "STEP_OVER": "#0ca30c"}


def rect(x, y, sx, sy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * dx - s * dy, y + s * dx + c * dy)
            for dx, dy in ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2))]


def main():
    trial, world, out = sys.argv[1:4]
    files = sorted(glob.glob(os.path.join(trial, "costmap", "*.npz")))
    if len(sys.argv) > 4:
        pick = os.path.join(trial, "costmap", f"{int(sys.argv[4]):04d}.npz")
    else:
        pick = max(files, key=lambda f: sum(o["label"] in ("AVOID", "CAUTION")
                                            for o in json.loads(str(np.load(f)["semantic"]))))
    d = np.load(pick)
    g, (ox, oy), res = d["grid"].astype(float), d["origin"], float(d["resolution"])
    g[g < 0] = np.nan
    fig, ax = plt.subplots(figsize=(7, 7))
    ext = [ox, ox + g.shape[1] * res, oy, oy + g.shape[0] * res]
    ax.imshow(g, origin="lower", extent=ext, cmap="Greys", vmin=0, vmax=100)
    for o in json.load(open(world))["objects"]:
        ax.add_patch(Polygon(rect(o["x"], o["y"], o["sx"], o["sy"], o.get("yaw", 0.0)), closed=True,
                             fill=False, ec="#2a78d6", lw=1.2, ls="--"))
        if ext[0] <= o["x"] <= ext[1] and ext[2] <= o["y"] <= ext[3]:
            ax.annotate(o["name"], (o["x"], o["y"]), fontsize=7, color="#2a78d6", ha="center")
    for o in json.loads(str(d["semantic"])):
        col = COLOR.get(o["label"], "#898781")
        if o["size_x"] > 0 and o["size_y"] > 0:
            ax.add_patch(Polygon(rect(o["x"], o["y"], o["size_x"] + 0.1, o["size_y"] + 0.1, o["yaw"]),
                                 closed=True, fill=False, ec=col, lw=2))
        else:
            ax.add_patch(Circle((o["x"], o["y"]), o["radius"], fill=False, ec=col, lw=2))
        ax.annotate(f"{o['class']}: {o['label']}", (o["x"], o["y"]), xytext=(0, -12),
                    textcoords="offset points", fontsize=7, color=col, ha="center")
    ax.set_xlim(ext[0], ext[1])
    ax.set_ylim(ext[2], ext[3])
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"Local costmap {os.path.basename(pick)}: grey = cost, dashed blue = true object,\n"
                 "solid = semantic mark (red AVOID, amber CAUTION)", fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    print(pick)


if __name__ == "__main__":
    main()
