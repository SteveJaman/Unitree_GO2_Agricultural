import json
import sys

path, label = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else "run")
recs = [json.loads(l) for l in open(path) if l.strip()]
if not recs:
    sys.exit("empty log")
t0 = recs[0]["t"]
tr = {}
for r in recs:
    t = round(r["t"] - t0, 1)
    for k in r["tracks"]:
        d = tr.setdefault(k["id"], {"first": t, "conf": None, "verdict_t": None, "dec_hist": []})
        if k["seen"] >= 3 and d["conf"] is None:
            d["conf"] = t
        if k["v"] and d["verdict_t"] is None:
            d["verdict_t"] = t
        if not d["dec_hist"] or d["dec_hist"][-1][1] != k["dec"]:
            d["dec_hist"].append((t, k["dec"]))
        d["last"] = k
print(f"TRACKS,{label},vlm={recs[0]['vlm']},step_limit={recs[0]['step_limit']},duration_s={round(recs[-1]['t'] - t0)}")
print("id,x,y,h_cm,n,final,vlm_name,vlm_label,vlm_fragile,t_first,t_confirmed,t_verdict,decision_changes")
for i, d in sorted(tr.items()):
    k = d["last"]
    v = k["v"] or {}
    hist = ">".join(f"{h[1]}@{h[0]}" for h in d["dec_hist"])
    print(f"{i},{k['x']},{k['y']},{k['h'] * 100:.0f},{k['n']},{k['dec']},{v.get('name', '')},{v.get('label', '')},"
          f"{v.get('fragile', '')},{d['first']},{d['conf'] if d['conf'] is not None else ''},"
          f"{d['verdict_t'] if d['verdict_t'] is not None else ''},{hist}")
print(f"PATH,{label}")
print("t_s,x,y,yaw")
last = -9
for r in recs:
    t = r["t"] - t0
    if t - last >= 1.0:
        print(f"{t:.0f},{r['base'][0]},{r['base'][1]},{r['base'][2]}")
        last = t
