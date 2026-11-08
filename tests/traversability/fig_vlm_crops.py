#!/usr/bin/env python3
"""Contact sheet of the VLM evaluation crops (eval_vlm_frames.py output).

  fig_vlm_crops.py EVAL_DIR OUT_PNG

One crop per object (the first), titled with the true label, the VLM's answer
and the published (combined) label; red when the published label is unsafe.
"""
import csv
import os
import sys
import zlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def read_png(path):
    """The RGB PNGs semantic_labeler.png_bytes writes (filter 0, 8-bit RGB)."""
    data = open(path, "rb").read()
    w, h = int.from_bytes(data[16:20], "big"), int.from_bytes(data[20:24], "big")
    n = int.from_bytes(data[33:37], "big")
    raw = np.frombuffer(zlib.decompress(data[41:41 + n]), np.uint8).reshape(h, 1 + 3 * w)
    return raw[:, 1:].reshape(h, w, 3)


def main():
    ev, out = sys.argv[1:3]
    rows, seen = [], set()
    for r in csv.DictReader(open(os.path.join(ev, "vlm_eval.csv"))):
        if r["object"] not in seen:
            seen.add(r["object"])
            rows.append(r)
    sev = {"STEP_OVER": 0, "CAUTION": 1, "AVOID": 2, "UNKNOWN": -1}
    cols = 5
    fig, axes = plt.subplots((len(rows) + cols - 1) // cols, cols, figsize=(3.0 * cols, 3.3 * ((len(rows) + cols - 1) // cols)))
    for ax in axes.flat:
        ax.axis("off")
    for ax, r in zip(axes.flat, rows):
        ax.imshow(read_png(os.path.join(ev, "crops", r["crop"] + ".png")))
        unsafe = r["true"] == "AVOID" and r["final"] in ("STEP_OVER", "CAUTION")
        low_vlm = sev[r["vlm_raw"]] < sev[r["true"]] and r["vlm_raw"] != "UNKNOWN"
        ax.set_title(f"{r['object']} (true {r['true']})\nVLM {r['vlm_raw']} {float(r['vlm_conf']):.2f}"
                     f"{' (low)' if low_vlm else ''}\npublished {r['final']} [{r['source']}]",
                     fontsize=8, color="#d03b3b" if unsafe else "#0b0b0b")
    fig.tight_layout()
    fig.savefig(out, dpi=110)


if __name__ == "__main__":
    main()
