#!/usr/bin/env python3
"""
species_eval.py: the species ladder's detector table (mAP50 / mAP50-95 per species).

  python3 scripts/tools/species_eval.py --out results/species_map.csv

For each species: ~/.cache/go2-yolo/<species>_yolo11s.pt evaluated with ultralytics' val on
the held-out split of ~/.cache/go2-yolo/<species>_dataset (every 8th generated frame,
never trained on). Lettuce is the control: lettuce_yolo11s.pt on its own RGB-D validation set,
not retrained. The floors (mAP50-95) are the plan's: lettuce 0.80, basil 0.75, spinach 0.70,
tomato 0.70, strawberry 0.65. Writes the CSV and a markdown table next to it.
"""
import argparse
import csv
import glob
import os

CACHE = os.path.expanduser("~/.cache/go2-yolo")
FLOORS = {"lettuce": 0.80, "basil": 0.75, "spinach": 0.70, "tomato": 0.70, "strawberry": 0.65}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--species", nargs="+", default=list(FLOORS))
    ap.add_argument("--out", default="results/species_map.csv")
    a = ap.parse_args()
    from ultralytics import YOLO
    rows = []
    for sp in a.species:
        weights = os.path.join(CACHE, f"{sp}_yolo11s.pt")
        data = os.path.join(CACHE, f"{sp}_dataset", "data.yaml")
        if not (os.path.exists(weights) and os.path.exists(data)):
            print(f"[eval] {sp}: missing {weights if not os.path.exists(weights) else data}")
            continue
        n_val = len(glob.glob(os.path.join(CACHE, f"{sp}_dataset", "images", "val", "*.jpg")))
        n_train = len(glob.glob(os.path.join(CACHE, f"{sp}_dataset", "images", "train", "*.jpg")))
        m = YOLO(weights).val(data=data, split="val", imgsz=640, batch=16, verbose=False, plots=False,
                              project=os.path.join(CACHE, "runs"), name=f"eval_{sp}", exist_ok=True)
        r = {"species": sp, "train_frames": n_train, "val_frames": n_val, "mAP50": round(float(m.box.map50), 4),
             "mAP50_95": round(float(m.box.map), 4), "precision": round(float(m.box.mp), 4),
             "recall": round(float(m.box.mr), 4), "floor_mAP50_95": FLOORS.get(sp, ""),
             "pass": float(m.box.map) >= FLOORS.get(sp, 0)}
        rows.append(r)
        print(f"[eval] {sp}: mAP50 {r['mAP50']:.3f}, mAP50-95 {r['mAP50_95']:.3f} on {n_val} held-out frames "
              f"(floor {r['floor_mAP50_95']}) -> {'PASS' if r['pass'] else 'FAIL'}")
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    md = ["| Species | Train frames | Held-out frames | mAP50 | mAP50-95 | Floor (mAP50-95) | Result |",
          "|---|---|---|---|---|---|---|"]
    md += [f"| {r['species']} | {r['train_frames']} | {r['val_frames']} | {r['mAP50']:.3f} | {r['mAP50_95']:.3f} | "
           f"{r['floor_mAP50_95']} | {'pass' if r['pass'] else 'FAIL'} |" for r in rows]
    open(a.out.replace(".csv", ".md"), "w").write("\n".join(md) + "\n")
    print("\n".join(md))


if __name__ == "__main__":
    main()
