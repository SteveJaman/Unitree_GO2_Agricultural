"""Pure-numpy logic for traversability_node.py (no ROS imports, unit-testable).

Pipeline: points in the map frame -> floor height -> low-object candidates
(grid clusters) -> tracks -> VLM verdict -> decision.

Safety rule (decide): geometry always wins. The VLM can only make an object
MORE restricted than its measured height allows, never less.
"""
import json
import math
import re

import numpy as np

LABELS = ('step_over', 'avoid', 'caution')


# ----------------------------------------------------------------- floor
def estimate_floor(pts, guess, window=0.15, bin_w=0.005):
    """Floor z = most populated 5 mm z-bin within +-window of the TF guess."""
    if len(pts) == 0:
        return float(guess)
    z = pts[:, 2]
    z = z[np.abs(z - guess) < window]
    if len(z) < 20:
        return float(guess)
    bins = np.arange(guess - window, guess + window + bin_w, bin_w)
    counts, edges = np.histogram(z, bins=bins)
    k = int(np.argmax(counts))
    centre = (edges[k] + edges[k + 1]) / 2
    near = z[np.abs(z - centre) < 0.01]
    return float(np.median(near)) if len(near) else float(centre)


# ----------------------------------------------------------------- clustering
def find_candidates(pts, floor_z, cell=0.08, h_min=0.012, h_tall=0.5, h_scan=2.0,
                    min_cell_pts=2, max_cells=400):
    """Low-object clusters in an Nx3 map-frame cloud.

    Cells whose highest point exceeds h_tall (walls, crates) and their 8
    neighbours are excluded, so only free-standing low objects remain.
    Returns a list of dicts: x, y, height, n_pts, cells [(x, y)], bbox
    ((x0, y0), (x1, y1)). Clusters with more than max_cells cells are dropped
    (a floor-estimate error would otherwise turn the whole floor into one; 400 cells = 2.5 m^2 at 8 cm).
    """
    if len(pts) == 0:
        return []
    h = pts[:, 2] - floor_z
    keep = (h > h_min) & (h < h_scan)
    pts, h = pts[keep], h[keep]
    if len(pts) == 0:
        return []
    ij = np.floor(pts[:, :2] / cell).astype(np.int64)
    uniq, inv = np.unique(ij, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    cmax = np.full(len(uniq), -np.inf)
    np.maximum.at(cmax, inv, h)
    cnt = np.bincount(inv, minlength=len(uniq))
    index = {(int(a), int(b)): k for k, (a, b) in enumerate(uniq)}

    blocked = set()
    for key, k in index.items():
        if cmax[k] > h_tall:
            a, b = key
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    blocked.add((a + da, b + db))
    cand = {key for key, k in index.items() if key not in blocked and cnt[k] >= min_cell_pts}

    lab = np.full(len(uniq), -1, dtype=np.int64)
    clusters, seen = [], set()
    for start in sorted(cand):
        if start in seen:
            continue
        cid, queue, members = len(clusters), [start], []
        seen.add(start)
        while queue:
            a, b = queue.pop()
            members.append((a, b))
            lab[index[(a, b)]] = cid
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    nb = (a + da, b + db)
                    if nb in cand and nb not in seen:
                        seen.add(nb)
                        queue.append(nb)
        clusters.append(members)

    plab = lab[inv]
    out = []
    for cid, members in enumerate(clusters):
        if len(members) > max_cells:
            continue
        sel = plab == cid
        hs, xy = h[sel], pts[sel, :2]
        height = float(np.percentile(hs, 95)) if len(hs) >= 8 else float(hs.max())
        out.append({
            'x': float(xy[:, 0].mean()), 'y': float(xy[:, 1].mean()),
            'height': height, 'n_pts': int(sel.sum()),
            'cells': [((a + 0.5) * cell, (b + 0.5) * cell) for a, b in members],
            'bbox': ((float(xy[:, 0].min()), float(xy[:, 1].min())),
                     (float(xy[:, 0].max()), float(xy[:, 1].max()))),
        })
    return out


# ----------------------------------------------------------------- tracking
class Track:
    def __init__(self, tid, c, now):
        self.id = tid
        self.x, self.y, self.height = c['x'], c['y'], c['height']
        self.cells, self.bbox, self.n_pts = c['cells'], c['bbox'], c['n_pts']
        self.first_seen = self.last_seen = now
        self.seen_count = 1
        self.verdict = None      # {'label', 'fragile', 'name'} from the VLM
        self.retry_at = 0.0      # monotonic time before which the VLM is not asked again
        self.decision = 'caution'

    def confirmed(self, min_seen):
        return self.seen_count >= min_seen


class Tracker:
    """Nearest-centroid association; objects are static, so tracks persist for ttl s."""

    def __init__(self, match_dist=0.35, ttl=30.0, alpha=0.3):
        self.match_dist, self.ttl, self.alpha = match_dist, ttl, alpha
        self.tracks = {}
        self._next = 1

    def update(self, cands, now):
        used = set()
        for c in cands:
            best, bd = None, self.match_dist
            for t in self.tracks.values():
                if t.id in used:
                    continue
                d = math.hypot(t.x - c['x'], t.y - c['y'])
                if d < bd:
                    best, bd = t, d
            if best is None:
                best = Track(self._next, c, now)
                self.tracks[best.id] = best
                self._next += 1
            else:
                a = self.alpha
                best.height = (1 - a) * best.height + a * c['height']
                best.x, best.y = (1 - a) * best.x + a * c['x'], (1 - a) * best.y + a * c['y']
                best.cells, best.bbox, best.n_pts = c['cells'], c['bbox'], c['n_pts']
                best.last_seen = now
                best.seen_count += 1
            used.add(best.id)
        for tid in [k for k, t in self.tracks.items() if now - t.last_seen > self.ttl]:
            del self.tracks[tid]
        return list(self.tracks.values())


# ----------------------------------------------------------------- decision
def decide(height, verdict, step_limit, use_vlm=True):
    """'traversable' | 'caution' | 'avoid'. Geometry first; the VLM only restricts.

    caution and avoid are both published as obstacles in v1 (a Nav2
    ObstacleLayer marks lethal cost either way).
    """
    if height >= step_limit:
        return 'avoid'
    if not use_vlm:
        return 'traversable'
    if verdict is None:
        return 'caution'
    if verdict['label'] == 'avoid' or verdict.get('fragile'):
        return 'avoid'
    if verdict['label'] == 'step_over':
        return 'traversable'
    return 'caution'


# ----------------------------------------------------------------- VLM text
def build_prompt(height_m):
    return (
        'This is a close-up camera crop of one object lying on the floor in front of a small '
        f'quadruped robot (Unitree Go2, about 30 cm tall). Measured height: {height_m * 100:.0f} cm. '
        'Decide whether the robot may step over it. '
        'Answer with JSON only: {"name": "<one or two words>", "fragile": true or false, '
        '"label": "step_over" or "avoid" or "caution"}. '
        'step_over = harmless low flat item (cable, cord, mat, rug, paper, tape). '
        'avoid = fragile, valuable, breakable, liquid-filled, or alive (phone, glass, cup, plant, pet). '
        'caution = you cannot tell.')


def parse_verdict(text):
    """Model reply -> {'label', 'fragile', 'name'} or None when unusable."""
    try:
        m = re.search(r'\{.*\}', text, re.S)
        d = json.loads(m.group(0))
    except (AttributeError, ValueError, TypeError):
        return None
    if not isinstance(d, dict):
        return None
    label = str(d.get('label', '')).strip().lower().replace('-', '_').replace(' ', '_')
    if label not in LABELS:
        return None
    fragile = d.get('fragile') in (True, 'true', 'True', 'yes', 'Yes')
    return {'label': label, 'fragile': fragile, 'name': str(d.get('name', ''))[:40]}
