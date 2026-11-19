#!/usr/bin/env python3
"""
coverage_eval.py: how much of a plant an orbit saw, from its wrist frames (host side).

  python3 scripts/nodes/coverage_eval.py --runs SWEEP/runs --world-json gazebo/worlds/cropking_nft46.json \
      --model ~/.cache/go2-yolo/lettuce_yolo11s.pt --per-shell --csv SWEEP/orbit_trials.csv

Inputs per run (orbit_waypoints.py --mode orbit): waypoints.csv, run.json and frames/ (jpg,
depth npz in metres, json with the achieved camera pose and the intrinsics). The camera
pose is the ground truth the run logged at capture (the dog's Gazebo pose + the joints +
the arm's kinematics), the same transform /tf carries.

Plant surface: N points (3000) sampled on the plant mesh, area-weighted, placed with the
plant's pose and scale in the world SDF. "Visible half": the samples on the side of the
plant facing the dog (x below the plant centre's: the robot side; the far side is behind
the plant for every waypoint of a front orbit).

Covered: a sample counts as seen in a frame when (all of)
  - it projects inside the image, in front of the camera (> 0.1 m);
  - the rendered depth at its pixel is within 1.5 cm of its own depth (not occluded);
  - the plant's projected extent in that frame is >= 30 px on its shorter side
    ("sampled at >= 30 px": enough pixels on the plant to inspect it);
  - the frame is sharp: variance of the Laplacian of the grey image >= --sharpness
    (default 15; frames below are dropped and counted);
coverage_fraction = seen visible-half samples / visible-half samples, per waypoint (its 5
frames), per shell (--per-shell: union over the shell's waypoints) and per run (all).

Detection: YOLO (--model) on every frame, conf >= 0.25; detection_confidence of a waypoint
is the highest confidence of a box overlapping the plant's projected box (IoU >= 0.1) over
its frames (0 when none), box_area_px that box's area.

Outputs: --csv (one row per waypoint, the orbit_trials schema), and next to it
<csv>_runs.csv (per run and per shell coverage) and <csv>_frames.csv (per frame: sharpness,
detection, the samples seen, for the per-shell unique-detection analysis).
"""

import argparse
import csv
import glob
import json
import math
import os
import re
import xml.etree.ElementTree as ET

import cv2
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCHEMA = [
    "run_id",
    "d",
    "r",
    "shell",
    "waypoint_k",
    "azimuth_deg",
    "elevation_deg",
    "ik_success",
    "plan_success",
    "fail_reason",
    "plan_time_s",
    "exec_time_s",
    "settle_time_s",
    "hold_energy_j",
    "exec_energy_j",
    "camera_x",
    "camera_y",
    "camera_z",
    "dist_to_plant_m",
    "detection_confidence",
    "box_area_px",
    "frame_sharpness",
    "coverage_fraction",
]


def load_obj(path):
    V, F = [], []
    for line in open(path):
        p = line.split()
        if not p:
            continue
        if p[0] == "v":
            V.append([float(x) for x in p[1:4]])
        elif p[0] == "f":
            idx = [int(t.split("/")[0]) - 1 for t in p[1:]]
            F += [[idx[0], idx[k], idx[k + 1]] for k in range(1, len(idx) - 1)]
    return np.array(V), np.array(F)


def sample_mesh(V, F, n, seed=0):
    rng = np.random.default_rng(seed)
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    pick = rng.choice(len(F), n, p=area / area.sum())
    u, v = rng.random(n), rng.random(n)
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    return a[pick] + u[:, None] * (b[pick] - a[pick]) + v[:, None] * (c[pick] - a[pick])


def plant_samples(plant, world_sdf, n=3000):
    """World-frame surface samples of the plant: plant = {'model': name in the SDF} or a run's
    plant record {'mesh': path, 'pose': [x, y, z, yaw], 'scale': [sx, sy, sz]}."""
    if "mesh" in plant:
        mesh, (x, y, z, yaw), sc = (
            plant["mesh"],
            plant["pose"],
            plant.get("scale", [1, 1, 1]),
        )
    else:
        root = ET.parse(world_sdf).getroot()
        m = next(e for e in root.iter("model") if e.get("name") == plant["model"])
        x, y, z, _, _, yaw = (float(v) for v in m.find("pose").text.split())
        mesh_el = m.find(".//mesh")
        uri = mesh_el.find("uri").text
        sc = (
            [float(v) for v in mesh_el.find("scale").text.split()]
            if mesh_el.find("scale") is not None
            else [1, 1, 1]
        )
        mesh = os.path.join(
            REPO, "gazebo", "worlds", "models", uri.replace("model://", "")
        )
        if not os.path.exists(mesh):
            mesh = os.path.join(REPO, "gazebo", "models", uri.replace("model://", ""))
    V, F = load_obj(mesh)
    P = sample_mesh(V * np.array(sc), F, n)
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return P @ Rz.T + np.array([x, y, z])


def project(meta, pts):
    """(u, v, depth) of world points in a frame (camera_link: x out of the lens, y left, z up)."""
    R = np.array(meta["R_world"])
    cam = np.array(meta["camera_world"])
    pc = (pts - cam) @ R  # in camera_link axes
    x = pc[:, 0]
    fx, W, H = meta["fx"], meta["w"], meta["h"]
    with np.errstate(divide="ignore", invalid="ignore"):
        u = W / 2 - fx * pc[:, 1] / x
        v = H / 2 - fx * pc[:, 2] / x
    return u, v, x


def sharpness(img):
    return float(cv2.Laplacian(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var())


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    un = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / un if un > 0 else 0.0


def eval_frame(meta, img, depth, samples, visible, model, a):
    W, H = meta["w"], meta["h"]
    u, v, x = project(meta, samples)
    ok = (x > 0.1) & (u >= 0) & (u < W) & (v >= 0) & (v < H)
    ui, vi = np.clip(u.astype(int), 0, W - 1), np.clip(v.astype(int), 0, H - 1)
    dz = np.abs(depth[vi, ui].astype(np.float32) - x)
    seen = ok & (dz < 0.015)
    sharp = sharpness(img)
    box = None
    if ok.any():
        box = [
            float(u[ok].min()),
            float(v[ok].min()),
            float(u[ok].max()),
            float(v[ok].max()),
        ]
    extent = min(box[2] - box[0], box[3] - box[1]) if box else 0.0
    valid = sharp >= a.sharpness and extent >= a.min_px
    conf, area, det_box = 0.0, 0.0, None
    if model is not None and box is not None:
        r = model(img, conf=0.25, verbose=False)[0]
        for b, c in zip(r.boxes.xyxy.tolist(), r.boxes.conf.tolist()):
            if iou(b, box) >= 0.1 and c > conf:
                conf, area, det_box = float(c), (b[2] - b[0]) * (b[3] - b[1]), b
    covered = seen & visible if valid else np.zeros(len(samples), bool)
    # samples inside the detection box and seen: the "plant region" this detection covers
    in_det = np.zeros(len(samples), bool)
    if det_box is not None:
        in_det = (
            seen
            & (u >= det_box[0])
            & (u <= det_box[2])
            & (v >= det_box[1])
            & (v <= det_box[3])
        )
    return dict(
        sharp=sharp,
        valid=valid,
        extent=extent,
        conf=conf,
        area=area,
        covered=covered,
        det_region=in_det,
    )


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--runs", required=True, help="folder of run folders")
    ap.add_argument(
        "--world-json", default=os.path.join(REPO, "gazebo/worlds/cropking_nft46.json")
    )
    ap.add_argument("--world-sdf", default=None)
    ap.add_argument(
        "--model", default=os.path.expanduser("~/.cache/go2-yolo/lettuce_yolo11s.pt")
    )
    ap.add_argument(
        "--sharpness", type=float, default=15.0, help="variance-of-Laplacian threshold"
    )
    ap.add_argument("--min-px", type=float, default=30.0)
    ap.add_argument("--samples", type=int, default=3000)
    ap.add_argument("--per-shell", action="store_true")
    ap.add_argument("--csv", required=True)
    a = ap.parse_args()
    sdf = a.world_sdf or a.world_json.replace(".json", ".sdf")
    model = None
    if a.model and os.path.exists(a.model):
        from ultralytics import YOLO

        model = YOLO(a.model)
    world = json.load(open(a.world_json))
    tp = world["target_plant"]
    default_plant = {"model": f"lettuce_{tp['channel']}_{tp['site']}"}
    rows, run_rows, frame_rows = [], [], []
    dropped = total = 0
    for rdir in sorted(glob.glob(os.path.join(a.runs, "*"))):
        wp_csv = os.path.join(rdir, "waypoints.csv")
        if not os.path.exists(wp_csv):
            continue
        run = json.load(open(os.path.join(rdir, "run.json")))
        plant = run.get("plant_sample_spec") or default_plant
        samples = plant_samples(plant, sdf, a.samples)
        centre = np.array(run.get("plant_center_world", tp["center"]))
        visible = samples[:, 0] <= centre[0]
        nvis = int(visible.sum())
        union_run = np.zeros(len(samples), bool)
        union_shell = {}
        for w in csv.DictReader(open(wp_csv)):
            tag = f"k{int(w['waypoint_k']):02d}_{w['shell']}_az{int(float(w['azimuth_deg'])):+04d}"
            cov = np.zeros(len(samples), bool)
            confs, areas, sharps = [], [], []
            for fj in sorted(glob.glob(os.path.join(rdir, "frames", tag + "_f*.json"))):
                meta = json.load(open(fj))
                img = cv2.imread(fj.replace(".json", ".jpg"))
                depth = np.load(fj.replace(".json", ".npz"))["depth"].astype(np.float32)
                fr = eval_frame(meta, img, depth, samples, visible, model, a)
                total += 1
                dropped += 0 if fr["sharp"] >= a.sharpness else 1
                cov |= fr["covered"]
                confs.append(fr["conf"])
                areas.append(fr["area"])
                sharps.append(fr["sharp"])
                frame_rows.append(
                    {
                        "run_id": w["run_id"],
                        "shell": w["shell"],
                        "waypoint_k": w["waypoint_k"],
                        "azimuth_deg": w["azimuth_deg"],
                        "frame": meta["frame"],
                        "sharpness": f"{fr['sharp']:.1f}",
                        "valid": int(fr["valid"]),
                        "plant_extent_px": f"{fr['extent']:.0f}",
                        "detection_confidence": f"{fr['conf']:.3f}",
                        "box_area_px": f"{fr['area']:.0f}",
                        "seen_visible": int((fr["covered"]).sum()),
                        "det_region": " ".join(
                            map(str, np.flatnonzero(fr["det_region"] & visible))
                        ),
                    }
                )
            union_run |= cov
            union_shell.setdefault(w["shell"], np.zeros(len(samples), bool))
            union_shell[w["shell"]] |= cov
            row = {k: w.get(k, "") for k in SCHEMA}
            row.update(
                detection_confidence=f"{max(confs):.3f}" if confs else "",
                box_area_px=f"{max(areas):.0f}" if areas else "",
                frame_sharpness=f"{np.mean(sharps):.1f}" if sharps else "",
                coverage_fraction=f"{cov.sum() / nvis:.4f}" if sharps else "",
            )
            rows.append(row)
        rr = {
            "run_id": run["run_id"],
            "d": run["d"],
            "r": run["r"],
            "shell": "all",
            "coverage_fraction": f"{union_run.sum() / nvis:.4f}",
            "visible_samples": nvis,
        }
        run_rows.append(rr)
        if a.per_shell:
            for sh, u in union_shell.items():
                run_rows.append(
                    {**rr, "shell": sh, "coverage_fraction": f"{u.sum() / nvis:.4f}"}
                )
        print(
            f"[coverage] {run['run_id']}: coverage {union_run.sum() / nvis:.3f}"
            + (
                "".join(f", {sh} {u.sum() / nvis:.3f}" for sh, u in union_shell.items())
                if a.per_shell
                else ""
            )
        )
    os.makedirs(os.path.dirname(os.path.abspath(a.csv)), exist_ok=True)
    with open(a.csv, "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=SCHEMA)
        wr.writeheader()
        wr.writerows(rows)
    base = re.sub(r"\.csv$", "", a.csv)
    if run_rows:
        with open(base + "_runs.csv", "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=list(run_rows[0].keys()))
            wr.writeheader()
            wr.writerows(run_rows)
    if frame_rows:
        with open(base + "_frames.csv", "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=list(frame_rows[0].keys()))
            wr.writeheader()
            wr.writerows(frame_rows)
    print(
        f"[coverage] {len(rows)} waypoints, {total} frames, {dropped} below the sharpness threshold "
        f"{a.sharpness} (var of Laplacian) dropped; plant extent >= {a.min_px} px -> {a.csv}"
    )


if __name__ == "__main__":
    main()
