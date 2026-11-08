#!/usr/bin/env python3
"""Score the VLM's semantic labels against the world's ground truth.

  analyze_semantic.py RESULTS_DIR OUT_DIR WORLD_JSON

Reads each trial's semantic.jsonl (record_semantic.py: one /semantic/obstacles
message per line). Each obstacle is matched to the nearest true object within
MATCH_M; its label is compared with the label that object's band calls for:

  step_over -> STEP_OVER    caution -> CAUTION    blocked -> AVOID

Writes OUT_DIR/semantic_confusion.csv (rows: true label, columns: VLM label),
semantic_summary.json and semantic.md. The safety metric counts labels on tall
objects (true height >= TALL_M) that say STEP_OVER or CAUTION.
"""
import csv
import json
import math
import os
import sys
from collections import Counter, defaultdict

LABELS = ["STEP_OVER", "CAUTION", "AVOID", "UNKNOWN"]
TRUE_LABELS = ["STEP_OVER", "CAUTION", "AVOID", "NO_OBJECT"]
MATCH_M = 0.6
TALL_M = 0.25
STEP_OVER_MAX = float(os.environ.get("STEP_OVER_MAX", "0.15"))
LETHAL_MIN = float(os.environ.get("LETHAL_MIN", "0.25"))
COST_LABEL = {254: "AVOID", 253: "AVOID", 200: "CAUTION", 40: "STEP_OVER", 0: "UNKNOWN"}


def true_label(o):
    b = o.get("band")
    if b is None:
        b = "step_over" if o["height"] <= STEP_OVER_MAX else ("caution" if o["height"] < LETHAL_MIN else "blocked")
    return {"step_over": "STEP_OVER", "caution": "CAUTION", "blocked": "AVOID"}[b]


def obstacle_label(ob):
    """The VLM label if the message carries one, else the one its cost encodes."""
    if ob.get("label"):
        return ob["label"]
    cost = int(ob.get("cost", 0))
    return COST_LABEL.get(cost, "AVOID" if cost > 200 else "CAUTION" if cost > 40 else "STEP_OVER")


def load_semantic(path):
    f = os.path.join(path, "semantic.jsonl")
    if not os.path.exists(f):
        return []
    out = []
    for line in open(f):
        try:
            out.append(json.loads(line))
        except ValueError:
            pass
    return out


def match(ob, objects):
    best, dist = None, MATCH_M
    for o in objects:
        d = math.hypot(o["x"] - ob["x"], o["y"] - ob["y"])
        if d < dist:
            best, dist = o, d
    return best


def score(trials, objects):
    """One vote per (trial, object or track) per label: the label it held longest."""
    votes = defaultdict(Counter)
    for name, msgs in trials.items():
        for msg in msgs:
            for ob in msg.get("obstacles", []):
                o = match(ob, objects)
                key = (name, o["name"] if o else f"track{ob.get('track_id', '?')}")
                votes[key][obstacle_label(ob)] += 1
    rows = []
    for (trial, obj), c in sorted(votes.items()):
        o = next((x for x in objects if x["name"] == obj), None)
        rows.append({"trial": trial, "object": obj, "true": true_label(o) if o else "NO_OBJECT",
                     "true_h": o["height"] if o else "", "vlm": c.most_common(1)[0][0], "messages": sum(c.values())})
    return rows


def summarize(rows):
    m = {t: {p: 0 for p in LABELS} for t in TRUE_LABELS}
    for r in rows:
        m[r["true"]][r["vlm"]] += 1
    n = len(rows)
    correct = sum(m[t][t] for t in ("STEP_OVER", "CAUTION", "AVOID"))
    per_class = {}
    for c in ("STEP_OVER", "CAUTION", "AVOID"):
        tp = m[c][c]
        predicted = sum(m[t][c] for t in TRUE_LABELS)
        actual = sum(m[c].values())
        per_class[c] = {"precision": round(tp / predicted, 3) if predicted else None,
                        "recall": round(tp / actual, 3) if actual else None, "n_true": actual}
    unsafe = [r for r in rows if r["true_h"] != "" and r["true_h"] >= TALL_M and r["vlm"] in ("STEP_OVER", "CAUTION")]
    return m, {"objects_labelled": n, "accuracy": round(correct / n, 3) if n else None, "per_class": per_class,
               "unsafe_tall_labelled_low": len(unsafe),
               "step_over_when_true_blocked": sum(1 for r in rows if r["true"] == "AVOID" and r["vlm"] == "STEP_OVER"),
               "unsafe_examples": unsafe[:10]}


def main():
    res, out, world = sys.argv[1:4]
    os.makedirs(out, exist_ok=True)
    objects = json.load(open(world))["objects"]
    names = sorted(d for d in os.listdir(res) if os.path.isdir(os.path.join(res, d)))
    trials = {n: load_semantic(os.path.join(res, n)) for n in names}
    trials = {n: v for n, v in trials.items() if v or os.path.exists(os.path.join(res, n, "semantic.jsonl"))}
    rows = score(trials, objects)
    m, summary = summarize(rows)
    summary["trials_with_semantic_log"] = len(trials)
    with open(os.path.join(out, "semantic_confusion.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["true \\ vlm"] + LABELS)
        for t in TRUE_LABELS:
            w.writerow([t] + [m[t][p] for p in LABELS])
    with open(os.path.join(out, "semantic_objects.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["trial", "object", "true", "true_h", "vlm", "messages"])
        w.writeheader()
        w.writerows(rows)
    json.dump(summary, open(os.path.join(out, "semantic_summary.json"), "w"), indent=1)
    with open(os.path.join(out, "semantic.md"), "w") as f:
        f.write("| true \\ VLM | " + " | ".join(LABELS) + " |\n|" + "---|" * (len(LABELS) + 1) + "\n")
        for t in TRUE_LABELS:
            f.write(f"| {t} | " + " | ".join(str(m[t][p]) for p in LABELS) + " |\n")
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
