#!/usr/bin/env python3
"""Camera figures for the report, from the frames save_frames.py recorded.

  make_figures.py FRAMES_DIR OUT_DIR

fusion_view.png  sim camera frames with each 2D detection and its fused 3D box
yolo_real.png    the same world through stock YOLO11s (COCO), the real detector
data/yolo_real.csv  what YOLO11s reported on every frame

Runs in the go2-3d image (OpenCV, ultralytics).
"""
import collections
import csv
import glob
import json
import math
import os
import sys

import cv2
import numpy as np

BAND_BGR = {"step_over": (12, 163, 12), "caution": (25, 178, 250), "blocked": (59, 59, 208)}
K = (468.0, 468.0, 320.0, 180.0)
SCALE = 2


def load(path):
    z = np.load(path, allow_pickle=False)
    img = z["image"]
    if str(z["encoding"]) == "rgb8":
        img = img[:, :, ::-1]
    return {"img": img.copy(), "t": z["trans"], "q": z["quat"], "boxes": json.loads(str(z["boxes"])),
            "dets": json.loads(str(z["dets"]))}


def rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def box_corners(b):
    c, s = math.cos(b["yaw"]), math.sin(b["yaw"])
    pts = []
    for dz in (0.0, b["sz"]):
        for a, d in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
            dx, dy = a * b["sx"] / 2, d * b["sy"] / 2
            pts.append((b["x"] + c * dx - s * dy, b["y"] + s * dx + c * dy, dz))
    return np.array(pts)


def pixels(points, R, t):
    cam = points @ R.T + t
    if (cam[:, 0] < 0.15).any():
        return None
    u = K[2] - K[0] * cam[:, 1] / cam[:, 0]
    v = K[3] - K[1] * cam[:, 2] / cam[:, 0]
    return np.stack([u, v], axis=1) * SCALE


def label(img, text, x, y, color):
    x, y = int(np.clip(x, 2, img.shape[1] - 8 * len(text))), int(np.clip(y, 16, img.shape[0] - 4))
    (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    cv2.rectangle(img, (x - 2, y - h - 4), (x + w + 2, y + 3), color, -1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)


def draw(frame):
    img = cv2.resize(frame["img"], None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_LINEAR)
    R, t = rot(frame["q"]), np.asarray(frame["t"], float)
    for d in frame["dets"]:
        x1, y1, x2, y2 = [int(v * SCALE) for v in d["bbox"]]
        cv2.rectangle(img, (x1, y1), (x2, y2), (235, 235, 235), 1, cv2.LINE_AA)
    drawn = 0
    for b in frame["boxes"]:
        px = pixels(box_corners(b), R, t)
        if px is None or px[:, 0].max() < 0 or px[:, 0].min() > img.shape[1]:
            continue
        col = BAND_BGR[b["traverse"]]
        p = px.astype(int)
        for i in range(4):
            j = (i + 1) % 4
            cv2.line(img, tuple(p[i]), tuple(p[j]), col, 2, cv2.LINE_AA)
            cv2.line(img, tuple(p[i + 4]), tuple(p[j + 4]), col, 2, cv2.LINE_AA)
            cv2.line(img, tuple(p[i]), tuple(p[i + 4]), col, 2, cv2.LINE_AA)
        label(img, f"{b['class']} {b['height']:.2f} m {b['traverse']}", p[:, 0].min(), p[:, 1].min() - 6, col)
        drawn += 1
    return img, drawn


def main():
    frames_dir, out = sys.argv[1:3]
    os.makedirs(os.path.join(out, "data"), exist_ok=True)
    paths = sorted(glob.glob(os.path.join(frames_dir, "*.npz")))
    frames = [(p, load(p)) for p in paths]
    scored = []
    for i, (p, f) in enumerate(frames):
        _, n = draw(f)
        scored.append((n, i))
    picks = []
    for n, i in sorted(scored, reverse=True):
        if all(abs(i - j) > 12 for j in picks):
            picks.append(i)
        if len(picks) == 4:
            break
    tiles = [draw(frames[i][1])[0] for i in sorted(picks)]
    while len(tiles) < 4:
        tiles.append(np.zeros_like(tiles[0]))
    cv2.imwrite(os.path.join(out, "fusion_view.png"),
                np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])]))

    from ultralytics import YOLO
    model = YOLO("/opt/yolo/yolo11s.pt")
    rows, classes, examples = [], collections.Counter(), []
    for p, f in frames:
        res = model.predict(f["img"], conf=0.25, verbose=False, imgsz=640)[0]
        found = [(model.names[int(c)], float(s)) for c, s in zip(res.boxes.cls, res.boxes.conf)]
        for name, score in found:
            classes[name] += 1
            rows.append({"frame": os.path.basename(p), "class": name, "score": round(score, 3)})
        if found:
            examples.append((max(s for _, s in found), res.plot()))
    with open(os.path.join(out, "data", "yolo_real.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["frame", "class", "score"])
        w.writeheader()
        w.writerows(rows)
    examples.sort(key=lambda e: -e[0])
    tiles = [cv2.resize(e[1], None, fx=SCALE, fy=SCALE) for e in examples[:2]]
    if len(tiles) < 2:
        plain = cv2.resize(frames[sorted(picks)[0]][1]["img"], None, fx=SCALE, fy=SCALE)
        tiles = (tiles + [plain, plain])[:2]
    cv2.imwrite(os.path.join(out, "yolo_real.png"), np.hstack(tiles))
    print(json.dumps({"frames": len(frames), "frames_with_detection": len(examples),
                      "detections": len(rows), "classes": classes.most_common()}))


if __name__ == "__main__":
    main()
