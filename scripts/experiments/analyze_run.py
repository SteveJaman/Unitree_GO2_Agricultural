import json
import math
import statistics
import sys

# true footprints from build_lab_world.py: name, cx, cy, sx, sy, expected
OBJ = [("cable", -2.0, 0.0, 0.03, 0.9, "traversable"), ("mat", -1.0, 0.0, 0.8, 1.0, "traversable"),
       ("toolbox", 0.2, 0.9, 0.3, 0.5, "avoid"), ("phone", 0.2, -0.9, 0.15, 0.08, "avoid"),
       ("vase", 1.2, 0.0, 0.12, 0.12, "avoid")]
M = 0.2
RANK = {"traversable": 0, "caution": 1, "avoid": 2}
path, label = sys.argv[1], sys.argv[2]
recs = [json.loads(l) for l in open(path) if l.strip()]
t0 = recs[0]["t"]


def which(x, y):
    for n, cx, cy, sx, sy, _ in OBJ:
        if abs(x - cx) <= sx / 2 + M and abs(y - cy) <= sy / 2 + M:
            return n
    return "wall" if (x < -3.2 or abs(y) > 1.8) else "other"


per = {n: {"hs": [], "ticks": 0, "ids": set(), "dec": {"traversable": 0, "caution": 0, "avoid": 0},
           "events": [], "first": None, "labels": {}, "tr": {}} for n, *_ in OBJ}
unm = {"wall": set(), "other": set()}
for r in recs:
    t = r["t"] - t0
    cur = {}
    for k in r["tracks"]:
        if k["seen"] < 3:
            continue
        w = which(k["x"], k["y"])
        if w in unm:
            unm[w].add(k["id"])
            continue
        p = per[w]
        p["ids"].add(k["id"])
        p["hs"].append(k["h"] * 100)
        d = cur.get(w)
        if d is None or RANK[k["dec"]] > RANK[d]:
            cur[w] = k["dec"]
        if k["v"]:
            key = f"{k['v']['label']}/{'fragile' if k['v'].get('fragile') else 'ok'}"
            p["labels"][key] = p["labels"].get(key, 0) + 1
        p["tr"][k["id"]] = (k["x"], k["y"], round(k["h"] * 100), k["dec"], (k["v"] or {}).get("name", ""))
    for w, d in cur.items():
        p = per[w]
        p["ticks"] += 1
        p["dec"][d] += 1
        if p["first"] is None:
            p["first"] = t
        if not p["events"] or p["events"][-1][1] != d:
            p["events"].append((round(t), d))
print(f"OBJECTS,{label},vlm={recs[0]['vlm']},duration_s={round(recs[-1]['t'] - t0)},ticks={len(recs)}")
print("object,expected,tracks,seen_ticks,first_seen_s,h_min,h_med,h_max,pct_trav,pct_caution,pct_avoid,final,vlm_labels")
for n, *_, exp in OBJ:
    p = per[n]
    if not p["ticks"]:
        print(f"{n},{exp},0,0,,,,,,,,not_seen,")
        continue
    tot = p["ticks"]
    fin = p["events"][-1][1]
    print(f"{n},{exp},{len(p['ids'])},{tot},{round(p['first'])},{min(p['hs']):.0f},{statistics.median(p['hs']):.0f},"
          f"{max(p['hs']):.0f},{100 * p['dec']['traversable'] / tot:.0f},{100 * p['dec']['caution'] / tot:.0f},"
          f"{100 * p['dec']['avoid'] / tot:.0f},{fin},{' '.join(f'{k}:{v}' for k, v in p['labels'].items())}")
print(f"UNMATCHED,{label},wall_tracks={len(unm['wall'])},other_tracks={len(unm['other'])}")
print("EVENTS," + label)
for n, *_ in OBJ:
    if per[n]["events"]:
        print(n + "," + ">".join(f"{d}@{t}" for t, d in per[n]["events"][:40]))
print("MATCHED_TRACKS," + label)
print("object,x,y,h_cm,final,vlm_name")
for n, *_ in OBJ:
    for i, (x, y, h, d, nm) in sorted(per[n]["tr"].items()):
        print(f"{n},{x},{y},{h},{d},{nm}")
print("PATH," + label)
last = None
for r in recs:
    b = r["base"]
    if last is None or math.hypot(b[0] - last[0], b[1] - last[1]) > 0.15:
        print(f"{round(r['t'] - t0)},{b[0]},{b[1]}")
        last = b
