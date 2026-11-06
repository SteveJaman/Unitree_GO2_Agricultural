#!/usr/bin/env python3
"""Turn run_demo.sh's chase-camera frames into the report's walking clips.

  make_video.py DEMO_DIR OUT_DIR

walk_baseline.mp4, walk_traversability.mp4  full clips, 12 fps (one frame per 1/12 s of sim time)
walk_compare.gif   the two side by side, for the report page
walk_strip.png     four moments of each run

Runs in the go2-3d image (OpenCV, Pillow).
"""
import glob
import os
import re
import sys

import cv2
import numpy as np
from PIL import Image


def frames(folder):
    paths = glob.glob(os.path.join(folder, "*.png"))
    return sorted(paths, key=lambda p: int(re.search(r"_(\d+)\.png$", p).group(1)))


def caption(img, text):
    img = img.copy()
    cv2.rectangle(img, (0, 0), (img.shape[1], 34), (30, 30, 30), -1)
    cv2.putText(img, text, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    return img


def write_mp4(paths, path, text):
    out = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), 12, (640, 360))
    for p in paths:
        out.write(cv2.resize(caption(cv2.imread(p), text), (640, 360), interpolation=cv2.INTER_AREA))
    out.release()


def main():
    demo, out = sys.argv[1:3]
    os.makedirs(out, exist_ok=True)
    base = frames(os.path.join(demo, "baseline", "frames"))
    trav = frames(os.path.join(demo, "traversability", "frames"))
    print("frames", len(base), len(trav))
    write_mp4(base, os.path.join(out, "walk_baseline.mp4"), "Baseline: low step, 4 cm lift")
    write_mp4(trav, os.path.join(out, "walk_traversability.mp4"), "Traversability: 3D boxes choose the step")

    n = min(len(base), len(trav))
    gif = []
    for i in range(0, n, 3):
        a = caption(cv2.imread(base[int(i * len(base) / n)]), "Baseline")
        b = caption(cv2.imread(trav[int(i * len(trav) / n)]), "Traversability")
        both = cv2.resize(np.hstack([a, b]), None, fx=0.42, fy=0.42, interpolation=cv2.INTER_AREA)
        gif.append(Image.fromarray(both[:, :, ::-1]).quantize(colors=96))
    gif[0].save(os.path.join(out, "walk_compare.gif"), save_all=True, append_images=gif[1:],
                duration=250, loop=0, optimize=True)

    rows = []
    for name, paths in (("Baseline", base), ("Traversability", trav)):
        picks = [paths[int(len(paths) * f)] for f in (0.2, 0.4, 0.6, 0.8)]
        rows.append(np.hstack([caption(cv2.resize(cv2.imread(p), (640, 360)), name) for p in picks]))
    cv2.imwrite(os.path.join(out, "walk_strip.png"), np.vstack(rows))


if __name__ == "__main__":
    main()
