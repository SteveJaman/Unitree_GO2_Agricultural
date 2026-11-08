#!/usr/bin/env python3
"""Offline accuracy of the semantic labeller on saved camera frames (Phase 2d).

  eval_vlm_frames.py --world cluttered_items.json --out DIR --frames N RUN_DIR [RUN_DIR ...]

Each RUN_DIR/frames/*.npz (save_frames.py) holds an image, the map->camera
transform, the fused boxes and the latest 2D detections at that moment. Every
box with a matching detection in that image is cropped from the detection and sent to the VLM with the same code the node uses
(scripts/nodes/semantic_labeler.py), then matched to the world's true object
(nearest within MATCH_M). True label is the height band of the true object.

Writes to --out: crops/*.png, vlm_eval.csv, vlm_eval.json, vlm_eval.md with
confusion matrices for the raw VLM label, the LiDAR geometry label and the
combined label, plus the safety counts:
  unsafe_tall_labelled_low   LiDAR-measured, true >= LETHAL_MIN, labelled STEP_OVER or CAUTION
  step_over_when_true_blocked  LiDAR-measured, true >= LETHAL_MIN, labelled STEP_OVER
  unmeasured_objects         crops with no LiDAR height (held at CAUTION by design)
"""
import argparse
import csv
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "scripts", "nodes"))
import semantic_labeler as sl  # noqa: E402

STEP_OVER_MAX = float(os.environ.get("STEP_OVER_MAX", "0.12"))
LETHAL_MIN = float(os.environ.get("LETHAL_MIN", "0.22"))
MATCH_M = 0.6
K = (468.0, 468.0, 320.0, 180.0)
TRUE = ["STEP_OVER", "CAUTION", "AVOID"]
PRED = ["STEP_OVER", "CAUTION", "AVOID", "UNKNOWN"]


def true_label(o):
    if o.get("band"):
        return {"step_over": "STEP_OVER", "caution": "CAUTION"}.get(o["band"], "AVOID")
    h = o["height"]
    return "STEP_OVER" if h <= STEP_OVER_MAX else ("CAUTION" if h < LETHAL_MIN else "AVOID")


def match(box, objects):
    best, dist = None, MATCH_M
    for o in objects:
        d = math.hypot(o["x"] - box["x"], o["y"] - box["y"])
        if d < dist:
            best, dist = o, d
    return best


def scores(rows, key):
    m = {t: {p: 0 for p in PRED} for t in TRUE}
    for r in rows:
        m[r["true"]][r[key]] += 1
    n = len(rows)
    correct = sum(m[t][t] for t in TRUE)
    # Unsafe counts LiDAR-measured objects only (unmeasured ones are held at CAUTION by design).
    meas = [r for r in rows if r.get("measured_height") is not None]
    out = {"matrix": m, "n": n, "accuracy": round(correct / n, 3) if n else None,
           "unsafe_tall_labelled_low": sum(r["true"] == "AVOID" and r[key] in ("STEP_OVER", "CAUTION") for r in meas),
           "step_over_when_true_blocked": sum(r["true"] == "AVOID" and r[key] == "STEP_OVER" for r in meas),
           "unmeasured_objects": n - len(meas),
           "unknown": sum(m[t]["UNKNOWN"] for t in TRUE)}
    for c in TRUE:
        tp = m[c][c]
        pp = sum(m[t][c] for t in TRUE)
        ap = sum(m[c].values())
        out[f"precision_{c}"] = round(tp / pp, 3) if pp else None
        out[f"recall_{c}"] = round(tp / ap, 3) if ap else None
    return out


def md_matrix(s, title):
    lines = [f"**{title}** (n = {s['n']}, accuracy {s['accuracy']})", "",
             "| true \\ label | " + " | ".join(PRED) + " |", "|---" * (len(PRED) + 1) + "|"]
    for t in TRUE:
        lines.append(f"| {t} | " + " | ".join(str(s["matrix"][t][p]) for p in PRED) + " |")
    lines += ["", f"unsafe_tall_labelled_low = {s['unsafe_tall_labelled_low']}, "
              f"step_over_when_true_blocked = {s['step_over_when_true_blocked']}, UNKNOWN = {s['unknown']}", ""]
    return lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--world", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames", type=int, default=24, help="frames sampled, spread evenly over all runs")
    ap.add_argument("--url", default="http://172.30.160.1:11434")
    ap.add_argument("--model", default="qwen2.5vl")
    ap.add_argument("--prompt", default=sl.PROMPT_ID, choices=sorted(sl.PROMPTS))
    ap.add_argument("--timeout", type=float, default=60.0, help="the first call also loads the model")
    ap.add_argument("--confidence-min", type=float, default=0.7)
    a = ap.parse_args()
    objects = json.load(open(a.world))["objects"]
    os.makedirs(os.path.join(a.out, "crops"), exist_ok=True)

    files = []
    for run in a.runs:
        files += sorted(glob.glob(os.path.join(run, "frames", "*.npz")))
    # Only frames where some box projects into view are worth sampling.
    usable = []
    for f in files:
        d = np.load(f)
        rot, trans = sl.rotation_matrix(d["quat"]), np.array(d["trans"])
        dets = json.loads(str(d["dets"]))
        boxes = [b for b in json.loads(str(d["boxes"]))
                 if sl.associate(b, dets, rot, trans, K) and match(b, objects)]
        if boxes:
            usable.append(f)
    pick = [usable[int(i)] for i in np.linspace(0, len(usable) - 1, min(a.frames, len(usable)))]
    print(f"{len(files)} frames, {len(usable)} with a box in view, using {len(pick)}", flush=True)

    rows = []
    for f in pick:
        d = np.load(f)
        rgb = sl.to_rgb(d["image"], d["encoding"])
        rot, trans = sl.rotation_matrix(d["quat"]), np.array(d["trans"])
        dets = json.loads(str(d["dets"]))
        for b in json.loads(str(d["boxes"])):
            o = match(b, objects)
            det = sl.associate(b, dets, rot, trans, K)
            got = det and sl.crop_bbox(rgb, det["bbox"])
            if not o or not got:
                continue
            crop, win = got
            tag = f"{os.path.basename(os.path.dirname(os.path.dirname(f)))}_{os.path.basename(f)[:-4]}_t{b['id']}"
            with open(os.path.join(a.out, "crops", tag + ".png"), "wb") as fh:
                fh.write(sl.png_bytes(sl.upscale(crop)))
            try:
                label, conf, reason, lat = sl.ask_ollama(a.url, a.model, crop, det["class"], det.get("score", 0.0), a.timeout, a.prompt)
            except Exception as ex:  # noqa: BLE001 - recorded, counted as UNKNOWN
                label, conf, reason, lat = "UNKNOWN", 0.0, f"error: {ex}", None
            geo = sl.geometry_label(b, STEP_OVER_MAX, LETHAL_MIN)
            final, source, _ = sl.combine(label, conf, geo, a.confidence_min)
            vlm_used = label if conf >= a.confidence_min else "UNKNOWN"
            r = {"crop": tag, "object": o["name"], "class": b["class"], "true_height": o["height"],
                 "true": true_label(o), "measured_height": b.get("height"), "vlm_raw": label,
                 "vlm_conf": round(conf, 2), "vlm": vlm_used, "geometry": geo, "final": final,
                 "source": source, "latency_s": None if lat is None else round(lat, 2),
                 "crop_px": f"{win[2] - win[0]}x{win[3] - win[1]}", "reason": reason}
            rows.append(r)
            print(json.dumps({k: r[k] for k in ("crop", "true", "vlm_raw", "vlm_conf", "geometry",
                                                 "final", "latency_s")}), flush=True)

    with open(os.path.join(a.out, "vlm_eval.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()) if rows else ["crop"])
        w.writeheader()
        w.writerows(rows)
    lat = [r["latency_s"] for r in rows if r["latency_s"] is not None]
    low = [r for r in rows if r["true"] == "STEP_OVER"]
    summary = {"model": a.model, "prompt": a.prompt, "frames": len(pick),
               "avoid_on_low_items": sum(r["vlm_raw"] == "AVOID" for r in low), "low_items": len(low), "crops": len(rows),
               "objects": len({r["object"] for r in rows}), "confidence_min": a.confidence_min,
               "errors": sum(1 for r in rows if r["reason"].startswith("error")),
               "latency_s": {"median": round(float(np.median(lat)), 2), "p90": round(float(np.percentile(lat, 90)), 2),
                             "max": round(max(lat), 2)} if lat else None,
               "vlm": scores(rows, "vlm"), "geometry": scores(rows, "geometry"), "final": scores(rows, "final")}
    json.dump(summary, open(os.path.join(a.out, "vlm_eval.json"), "w"), indent=1)
    md = [f"# VLM labeller on saved frames ({a.model}, prompt {a.prompt})", "",
          f"AVOID on true step-over crops: {summary['avoid_on_low_items']} of {summary['low_items']}", "",
          f"{summary['frames']} frames, {summary['crops']} crops of {summary['objects']} objects, "
          f"confidence_min {a.confidence_min}, latency {summary['latency_s']}, errors {summary['errors']}", ""]
    md += md_matrix(summary["vlm"], "VLM alone (below confidence_min counts as UNKNOWN)")
    md += md_matrix(summary["geometry"], "LiDAR height band alone")
    md += md_matrix(summary["final"], "Combined (published label)")
    open(os.path.join(a.out, "vlm_eval.md"), "w").write("\n".join(md))
    print(json.dumps({k: summary[k] for k in ("frames", "crops", "latency_s")}))
    for k in ("vlm", "geometry", "final"):
        s = summary[k]
        print(k, "acc", s["accuracy"], "unsafe", s["unsafe_tall_labelled_low"],
              "step_over_when_blocked", s["step_over_when_true_blocked"], "unknown", s["unknown"])


if __name__ == "__main__":
    main()
