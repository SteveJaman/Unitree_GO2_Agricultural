#!/usr/bin/env python3
"""Replay a run log with and without the operator lists (no sim needed).

  python3 replay_rules.py RUN.jsonl LABEL [--lab2] [--avoid a,b] [--safe c,d] [--unknown vlm|strict] [--skip FROM_S TO_S]

Every confirmed track is matched to the true object whose footprint it lies in. Four decisions are
compared per object, as the share of ticks spent at each decision (trav/caution/avoid, %):
  recorded                    what the run actually did
  oracle, no lists            a recogniser that names the object correctly but calls everything benign
                              (label step_over, not fragile): the height rule alone
  oracle, with lists          the same correct names, with the operator lists applied
  logged VLM name, with lists the VLM's own name from the log (VLM runs only), with the lists applied
The oracle columns show what the lists change when recognition is right. The sim objects are plain
boxes, so the real VLM cannot name most of them.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'nodes'))
import traversability_core as tc

OBJ = [("cable", -2.0, 0.0, 0.03, 0.9, "traversable"), ("mat", -1.0, 0.0, 0.8, 1.0, "traversable"),
       ("toolbox", 0.2, 0.9, 0.3, 0.5, "avoid"), ("phone", 0.2, -0.9, 0.15, 0.08, "avoid"),
       ("vase", 1.2, 0.0, 0.12, 0.12, "avoid")]
EXTRA = [("laptop", 0.8, -0.5, 0.33, 0.23, "avoid"), ("glasses case", 0.6, 0.4, 0.16, 0.06, "avoid"),
         ("power strip", 1.6, -1.1, 0.30, 0.06, "traversable"), ("book", -0.2, -1.3, 0.22, 0.15, "traversable")]
RANK = {"traversable": 0, "caution": 1, "avoid": 2}
M = 0.2

ap = argparse.ArgumentParser()
ap.add_argument('log')
ap.add_argument('label')
ap.add_argument('--lab2', action='store_true')
ap.add_argument('--avoid', default='phone,laptop,glasses,vase,toolbox')
ap.add_argument('--safe', default='cable,mat,power strip,book,rug')
ap.add_argument('--unknown', default='vlm')
ap.add_argument('--skip', nargs=2, type=float, metavar=('FROM_S', 'TO_S'),
                help='ignore this time window (seconds from the start of the log)')
a = ap.parse_args()
objs = OBJ + (EXTRA if a.lab2 else [])
words = lambda s: [w.strip().lower() for w in s.split(',') if w.strip()]
rules = {'avoid': words(a.avoid), 'safe': words(a.safe), 'unknown': a.unknown}
recs = [json.loads(l) for l in open(a.log) if l.strip()]
t0 = recs[0]['t']


def which(x, y):
    for n, cx, cy, sx, sy, _ in objs:
        if abs(x - cx) <= sx / 2 + M and abs(y - cy) <= sy / 2 + M:
            return n
    return None


MODES = ['recorded', 'oracle, no lists', 'oracle, with lists', 'logged VLM name, with lists']
acc = {o[0]: {m: {'traversable': 0, 'caution': 0, 'avoid': 0} for m in MODES} for o in objs}
ticks = {o[0]: 0 for o in objs}
for r in recs:
    t = r['t'] - t0
    if a.skip and a.skip[0] <= t <= a.skip[1]:
        continue
    sl = r.get('step_limit', 0.08)
    cur = {}
    for k in r['tracks']:
        if k['seen'] < 3:
            continue
        w = which(k['x'], k['y'])
        if w is None:
            continue
        v = {'name': w, 'label': 'step_over', 'fragile': False}
        d = {'recorded': k['dec'],
             'oracle, no lists': tc.decide(k['h'], v, sl, True, None),
             'oracle, with lists': tc.decide(k['h'], v, sl, True, rules)}
        if k.get('v'):
            d['logged VLM name, with lists'] = tc.decide(k['h'], k['v'], sl, True, rules)
        best = cur.setdefault(w, {})
        for m, dec in d.items():
            if m not in best or RANK[dec] > RANK[best[m]]:
                best[m] = dec
    for w, best in cur.items():
        ticks[w] += 1
        for m, dec in best.items():
            acc[w][m][dec] += 1

print(f'REPLAY,{a.label},avoid={a.avoid},safe={a.safe},unknown={a.unknown}')
print('object,expected,ticks,' + ','.join(f'{m} (trav/caution/avoid %)' for m in MODES))
for n, _cx, _cy, _sx, _sy, exp in objs:
    cols = []
    for m in MODES:
        c = acc[n][m]
        tot = sum(c.values())
        cols.append('-' if tot == 0 else '/'.join(str(round(100 * c[k] / tot)) for k in ('traversable', 'caution', 'avoid')))
    print(f'{n},{exp},{ticks[n]},' + ','.join(cols))
