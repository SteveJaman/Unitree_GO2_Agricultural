#!/usr/bin/env python3
"""Offline test of traversability_core.py on a synthetic scene (no ROS needed).

  python3 scripts/tools/test_traversability_core.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'nodes'))
import traversability_core as tc  # noqa: E402

rng = np.random.default_rng(1)


def grid(x0, x1, y0, y1, z, step=0.04):
    xs, ys = np.arange(x0, x1, step), np.arange(y0, y1, step)
    X, Y = np.meshgrid(xs, ys)
    return np.stack([X.ravel(), Y.ravel(), np.full(X.size, z)], axis=1)


def box(cx, cy, sx, sy, h, step=0.03):
    """Top face plus the four sides of a box standing on the z=0 floor."""
    top = grid(cx - sx / 2, cx + sx / 2 + 1e-6, cy - sy / 2, cy + sy / 2 + 1e-6, h, step)
    zs = np.arange(0.0, h + 1e-6, step)
    sides = []
    for z in zs:
        for x in np.arange(cx - sx / 2, cx + sx / 2 + 1e-6, step):
            sides += [(x, cy - sy / 2, z), (x, cy + sy / 2, z)]
        for y in np.arange(cy - sy / 2, cy + sy / 2 + 1e-6, step):
            sides += [(cx - sx / 2, y, z), (cx + sx / 2, y, z)]
    return np.vstack([top, np.array(sides)]) if sides else top


floor = grid(0.5, 5.0, -2.5, 2.5, 0.0, 0.05)
scene = np.vstack([
    floor,
    box(2.0, 0.0, 0.9, 0.03, 0.03),      # cable
    box(3.0, 1.0, 1.0, 0.8, 0.02),       # mat
    box(2.5, -1.0, 0.15, 0.08, 0.08),    # phone
    box(3.5, -1.5, 0.12, 0.12, 0.30),    # vase
    box(3.0, -0.3, 0.5, 0.3, 0.12),      # toolbox (taller than the step limit)
    box(4.4, 0.5, 0.6, 0.6, 0.6),        # crate (tall: must be ignored here)
])
scene[:, 2] += rng.normal(0, 0.002, len(scene))

guess = 0.012                              # TF guess is 12 mm off on purpose
floor_z = tc.estimate_floor(scene, guess)
print(f'floor estimate {floor_z:+.4f} m (truth 0.0, guess {guess})')
assert abs(floor_z) < 0.006, 'floor estimate off by more than 6 mm'

cands = tc.find_candidates(scene, floor_z)
print(f'{len(cands)} candidates')
for c in sorted(cands, key=lambda c: c['x']):
    print(f"  x={c['x']:.2f} y={c['y']:.2f} h={c['height'] * 100:.1f} cm cells={len(c['cells'])} pts={c['n_pts']}")


def nearest(x, y):
    return min(cands, key=lambda c: (c['x'] - x) ** 2 + (c['y'] - y) ** 2)


expect = {'cable': (2.0, 0.0, 0.03), 'mat': (3.0, 1.0, 0.02), 'phone': (2.5, -1.0, 0.08),
          'vase': (3.5, -1.5, 0.30), 'toolbox': (3.0, -0.3, 0.12)}
for name, (x, y, h) in expect.items():
    c = nearest(x, y)
    assert np.hypot(c['x'] - x, c['y'] - y) < 0.3, f'{name} not found'
    assert abs(c['height'] - h) < 0.012, f"{name}: height {c['height']:.3f} vs {h}"
assert not any(np.hypot(c['x'] - 4.4, c['y'] - 0.5) < 0.5 for c in cands), 'crate leaked in'

# a floor height that is 2 cm too low must not produce floor-sized "objects"
bad = tc.find_candidates(scene, floor_z - 0.02)
assert all(len(c['cells']) <= 400 for c in bad), 'oversized cluster leaked through'

# decision rules
D = tc.decide
step_ok = {'label': 'step_over', 'fragile': False, 'name': 'cable'}
assert D(0.03, step_ok, 0.08) == 'traversable'
assert D(0.03, None, 0.08) == 'caution'                                   # VLM pending
assert D(0.12, step_ok, 0.08) == 'avoid'                                  # geometry wins over VLM
assert D(0.03, {'label': 'step_over', 'fragile': True, 'name': 'x'}, 0.08) == 'avoid'
assert D(0.03, {'label': 'avoid', 'fragile': False, 'name': 'x'}, 0.08) == 'avoid'
assert D(0.03, {'label': 'caution', 'fragile': False, 'name': 'x'}, 0.08) == 'caution'
assert D(0.03, None, 0.08, use_vlm=False) == 'traversable'
assert D(0.30, None, 0.08, use_vlm=False) == 'avoid'

# verdict parsing
assert tc.parse_verdict('{"name":"cable","fragile":false,"label":"step_over"}')['label'] == 'step_over'
assert tc.parse_verdict('Sure! {"label": "step-over", "fragile": "true"} done')['fragile'] is True
assert tc.parse_verdict('no json here') is None
assert tc.parse_verdict('{"label":"jump"}') is None

# tracker keeps ids stable and averages height
tr = tc.Tracker()
t1 = tr.update(cands, 0.0)
t2 = tr.update(cands, 0.5)
assert len(t1) == len(t2) and all(t.seen_count == 2 for t in t2)
tr.update([], 100.0)
assert tr.tracks == {}, 'tracks should expire'
print('ALL TESTS PASSED')
