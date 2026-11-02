#!/usr/bin/env python3
"""
analyze.py: compare navigation trials recorded with record.sh / sim_trial.sh.

  analyze.py TRIAL_DIR [TRIAL_DIR ...] [--out DIR] [--rooms agri_rooms|lab_office]

Per trial (from frontier.log, recovery.log, pose.csv, cmd.csv, costmap.npz):
duration, goals sent / reached / dropped early, Nav2 aborts, no-progress and
timeout drops, stuck-recovery moves, mapped area, distance walked, time
standing still, the command mix (forward / turn in place / reverse /
sidestep) and, with --rooms agri_rooms, which rooms of the sim building the
dog entered.

With --rooms NAME and a gazebo/worlds/NAME.json (build_lab_world.py writes
lab_office.json) also, from the world's ground truth (the sim map frame is
the world frame):
  coverage     mapped area as a share of the building's floor; t90 = time
               to 90 % of the final mapped area
  clearance    the dog's body (BODY_L x BODY_W around base_link) against every
               wall and furniture footprint: minimum, contacts (episodes at
               < 2 cm), time within 10 cm
  tour         from tour.csv (nav_tour.py): goals reached, time and path per
               leg, path efficiency against the shortest path on the floor
               plan for a disc of SHORTEST_R, and SPL (success weighted by
               path length, Anderson et al. 2018)

Writes to --out (default: the current directory):
  trials.csv           one row per trial
  summary.md           the same as a Markdown table, plus per-drive means
  coverage.png         mapped area over time
  outcomes.png         goal outcomes per trial
  commands.png         command mix per drive
  trajectories.png     paths over the final map, one panel per drive
  stats.md             per-drive mean ± SD of the main metrics
  tour.png, clearance.png   (worlds with a JSON, see above)

Needs numpy + matplotlib. If the host's matplotlib is broken, run.sh runs this
inside the go2-3d image, which has both.
"""

import argparse
import csv
import heapq
import json
import math
import os
import re
import sys

import numpy as np

DRIVE_COLOR = {'diff': '#2a78d6', 'hybrid': '#1baf7a', 'omni': '#eb6834'}   # reference palette slots 1, 3, 2
TEXT = '#0b0b0b'
TEXT_2 = '#52514e'
GRID = '#e4e3df'
SURFACE = '#fcfcfb'

# Gazebo agri_rooms (gazebo/worlds/build_rooms_world.py); the sim map frame is
# the world frame.
ROOMS = {
    'agri_rooms': {
        'corridor': (-6.05, 6.05, 4.05, 6.05),
        'room 1': (-6.05, -2.05, 6.05, 11.05),
        'room 2': (-2.05, 2.05, 6.05, 11.05),
        'room 3': (2.05, 6.05, 6.05, 11.05),
    },
}

TS = re.compile(r'\[(\d{9,}\.\d+)\]')

HERE = os.path.dirname(os.path.abspath(__file__))
WORLDS = os.path.normpath(os.path.join(HERE, '..', '..', '..', 'gazebo', 'worlds'))
BODY_L, BODY_W = 0.70, 0.31     # Go2 body (m), centred on base_link
SHORTEST_R = 0.30               # disc for the shortest-path reference (robot_radius 0.22 + margin)
GRID_RES = 0.05


def load_world(name):
    """gazebo/worlds/NAME.json (rooms, bounds, obstacles, tour), or None."""
    path = os.path.join(WORLDS, f'{name}.json')
    if not name or not os.path.exists(path):
        return None
    w = json.load(open(path))
    ROOMS.setdefault(name, {k: tuple(v) for k, v in w['rooms'].items()})
    return w


def rect_points(cx, cy, yaw, sx, sy, n=4):
    """Points on a rotated rectangle's outline (n per side)."""
    t = np.linspace(-0.5, 0.5, n + 1)[:-1]
    loc = np.concatenate([np.stack([t * sx, np.full_like(t, -sy / 2)], 1),
                          np.stack([np.full_like(t, sx / 2), t * sy], 1),
                          np.stack([-t * sx, np.full_like(t, sy / 2)], 1),
                          np.stack([np.full_like(t, -sx / 2), -t * sy], 1)])
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([cx + c * loc[:, 0] - s * loc[:, 1], cy + s * loc[:, 0] + c * loc[:, 1]], 1)


def point_rect_dist(px, py, cx, cy, yaw, hx, hy):
    """Signed distance from points to rotated rectangles (negative inside); broadcasts."""
    c, s = np.cos(yaw), np.sin(yaw)
    dx, dy = px - cx, py - cy
    u, v = np.abs(c * dx + s * dy) - hx, np.abs(-s * dx + c * dy) - hy
    out = np.hypot(np.maximum(u, 0), np.maximum(v, 0))
    return np.where((u < 0) & (v < 0), np.maximum(u, v), out)


def clearance(pose, world):
    """Per pose: distance from the dog's body to the nearest wall or furniture footprint."""
    if len(pose) == 0:
        return np.zeros(0)
    obs = world['obstacles']
    ocx, ocy, oyaw = (np.array([o[k] for o in obs]) for k in ('cx', 'cy', 'yaw'))
    ohx, ohy = np.array([o['sx'] / 2 for o in obs]), np.array([o['sy'] / 2 for o in obs])
    opts = np.concatenate([rect_points(o['cx'], o['cy'], o['yaw'], o['sx'], o['sy'], 6) for o in obs])
    out = np.empty(len(pose))
    for i, (_, x, y, yaw) in enumerate(pose):
        body = rect_points(x, y, yaw, BODY_L, BODY_W, 6)
        d1 = point_rect_dist(body[:, 0:1], body[:, 1:2], ocx, ocy, oyaw, ohx, ohy).min()
        d2 = point_rect_dist(opts[:, 0], opts[:, 1], x, y, yaw, BODY_L / 2, BODY_W / 2).min()
        out[i] = min(d1, d2)
    return out


_GEO = {}


def geodesic(world, goal):
    """Shortest-path distance (m) from every free cell to goal, for a disc of SHORTEST_R."""
    key = (world['world'], goal)
    if key in _GEO:
        return _GEO[key]
    x0, x1, y0, y1 = world['bounds']
    nx, ny = int(round((x1 - x0) / GRID_RES)), int(round((y1 - y0) / GRID_RES))
    gx, gy = np.meshgrid(x0 + (np.arange(nx) + 0.5) * GRID_RES, y0 + (np.arange(ny) + 0.5) * GRID_RES)
    occ = np.zeros((ny, nx), bool)
    for o in world['obstacles']:
        occ |= point_rect_dist(gx, gy, o['cx'], o['cy'], o['yaw'], o['sx'] / 2, o['sy'] / 2) < SHORTEST_R
    dist = np.full((ny, nx), np.inf)
    gi, gj = int((goal[0] - x0) / GRID_RES), int((goal[1] - y0) / GRID_RES)
    dist[gj, gi] = 0.0
    heap = [(0.0, gi, gj)]
    steps = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
    while heap:
        d, i, j = heapq.heappop(heap)
        if d > dist[j, i]:
            continue
        for di, dj, w in steps:
            a, b = i + di, j + dj
            if 0 <= a < nx and 0 <= b < ny and not occ[b, a]:
                nd = d + w * GRID_RES
                if nd < dist[b, a]:
                    dist[b, a] = nd
                    heapq.heappush(heap, (nd, a, b))
    _GEO[key] = (dist, x0, y0)
    return _GEO[key]


def shortest(world, start, goal):
    dist, x0, y0 = geodesic(world, goal)
    i, j = int((start[0] - x0) / GRID_RES), int((start[1] - y0) / GRID_RES)
    win = dist[max(0, j - 6):j + 7, max(0, i - 6):i + 7]   # the start may sit in the inflated band
    return float(win.min()) if np.isfinite(win).any() else float('nan')


def read_tour(d, world, pose_all):
    path = os.path.join(d, 'tour.csv')
    if not os.path.exists(path) or world is None:
        return []
    legs = []
    for r in csv.DictReader(open(path)):
        t0 = float(r['t_start'])
        k = np.searchsorted(pose_all[:, 0], t0) if len(pose_all) else 0
        start = tuple(pose_all[min(k, len(pose_all) - 1), 1:3]) if len(pose_all) else (math.nan, math.nan)
        sp = shortest(world, start, (float(r['gx']), float(r['gy'])))
        p = float(r['path_m'])
        ok = r['result'] == 'reached'
        # A leg that starts within reach of its goal (exploration ended there)
        # counts as a trivial success: SPL 1, no path efficiency.
        trivial = ok and max(p, sp) < 0.05
        legs.append(dict(name=r['name'], result=r['result'], time_s=float(r['t_end']) - t0, path_m=p,
                         shortest_m=sp, efficiency=(sp / p if ok and p > 0 and not trivial else math.nan),
                         spl=(1.0 if trivial else sp / max(p, sp) if ok and sp == sp else 0.0),
                         aborts=int(r['planner_aborts']) + int(r['controller_aborts']),
                         t_start=t0, t_end=float(r['t_end'])))
    return legs


def read_meta(d):
    meta = {}
    path = os.path.join(d, 'meta.env')
    if os.path.exists(path):
        for line in open(path):
            if '=' in line:
                k, v = line.strip().split('=', 1)
                meta[k] = v.strip('"')
    return meta


def read_csv(path):
    if not os.path.exists(path):
        return np.zeros((0, 4))
    rows = list(csv.reader(open(path)))[1:]
    rows = [r for r in rows if len(r) == 4]
    return np.array(rows, dtype=float) if rows else np.zeros((0, 4))


def count(lines, text):
    return sum(text in line for line in lines)


def trial_metrics(d, rooms, res, world=None):
    meta = read_meta(d)
    flog = os.path.join(d, 'frontier.log')
    lines = open(flog, errors='replace').read().splitlines() if os.path.exists(flog) else []
    rlog = os.path.join(d, 'recovery.log')
    rlines = open(rlog, errors='replace').read().splitlines() if os.path.exists(rlog) else []

    def ts(line):
        m = TS.search(line)
        return float(m.group(1)) if m else None

    t0 = next((ts(l) for l in lines if 'Exploration ENABLED' in l), None)
    t_done = next((ts(l) for l in lines if 'Exploration complete' in l), None)
    t_end = t_done or float(meta.get('EXPLORE_END', 0)) or float(meta.get('END', 0)) \
        or (ts(lines[-1]) if lines else None)
    # known= counts global costmap cells; their size is in the saved costmap
    # (0.05 m in go2-nav2-wifi; --resolution is only the fallback).
    for name in ('costmap.npz', 'costmap_explore.npz'):
        if os.path.exists(os.path.join(d, name)):
            res = float(np.load(os.path.join(d, name))['resolution'])
            break
    coverage = []
    for l in lines:
        m = re.search(r'known=(\d+)', l)
        t = ts(l)
        if m and t is not None and t0 is not None and t >= t0:
            coverage.append((t - t0, int(m.group(1)) * res * res))

    pose = read_csv(os.path.join(d, 'pose.csv'))
    cmd = read_csv(os.path.join(d, 'cmd.csv'))
    pose_all = pose
    legs = read_tour(d, world, pose_all)
    if t0 is not None and t_end:
        pose = pose[(pose[:, 0] >= t0) & (pose[:, 0] <= t_end)]
        cmd = cmd[(cmd[:, 0] >= t0) & (cmd[:, 0] <= t_end)]

    dist, still = 0.0, 0.0
    if len(pose) > 1:
        dxy = np.hypot(np.diff(pose[:, 1]), np.diff(pose[:, 2]))
        dt = np.diff(pose[:, 0])
        dyaw = np.abs(np.angle(np.exp(1j * np.diff(pose[:, 3]))))
        ok = dxy < 1.0                     # drop TF jumps
        dist = float(dxy[ok].sum())
        speed = np.where(dt > 0, dxy / np.maximum(dt, 1e-3), 0)
        turn = np.where(dt > 0, dyaw / np.maximum(dt, 1e-3), 0)
        still = float(dt[(speed < 0.03) & (turn < 0.05)].sum())

    # Each command by its main direction; drift = any real sideways part.
    mix = {'forward': 0, 'turn': 0, 'reverse': 0, 'sidestep': 0}
    drift = 0
    moving = cmd[(np.abs(cmd[:, 1]) > 0.02) | (np.abs(cmd[:, 2]) > 0.02) | (np.abs(cmd[:, 3]) > 0.05)] \
        if len(cmd) else cmd
    for _, vx, vy, wz in moving:
        drift += abs(vy) > 0.05
        if abs(vy) > 0.02 and abs(vy) > abs(vx):
            mix['sidestep'] += 1
        elif vx < -0.02:
            mix['reverse'] += 1
        elif vx > 0.02:
            mix['forward'] += 1
        else:
            mix['turn'] += 1

    gt = {}
    if world is not None:
        area = (world['bounds'][1] - world['bounds'][0]) * (world['bounds'][3] - world['bounds'][2])
        gt['coverage_pct'] = round(100 * coverage[-1][1] / area, 0) if coverage else float('nan')
        snap = os.path.join(d, 'costmap_explore.npz')
        gt['_room_cov'] = {}
        if os.path.exists(snap):
            # Known cells inside the building, and per room, when exploration ended
            z = np.load(snap)
            data, r, (ox, oy) = z['data'], float(z['resolution']), z['origin']
            ys, xs = np.mgrid[0:data.shape[0], 0:data.shape[1]]
            X, Y = ox + (xs + 0.5) * r, oy + (ys + 0.5) * r
            known = data >= 0
            b = world['bounds']
            inside = (X > b[0]) & (X < b[1]) & (Y > b[2]) & (Y < b[3])
            gt['coverage_pct'] = round(100 * float(known[inside].mean()), 0)
            for room, (x0, x1, y0, y1) in world['rooms'].items():
                m = (X > x0) & (X < x1) & (Y > y0) & (Y < y1)
                gt['_room_cov'][room] = 100 * float(known[m].mean())
            gt['rooms_mapped'] = sum(v >= 80 for k, v in gt['_room_cov'].items() if k != 'corridor')
        t90 = next((t for t, a in coverage if a >= 0.9 * coverage[-1][1]), float('nan')) if coverage else float('nan')
        gt['t90_s'] = round(t90, 0)
        tour_pose = (pose_all[(pose_all[:, 0] >= legs[0]['t_start']) & (pose_all[:, 0] <= legs[-1]['t_end'])]
                     if legs and len(pose_all) else np.zeros((0, 4)))
        gt['_tour_pose'] = tour_pose
        for phase, pp in (('explore', pose), ('tour', tour_pose)):
            c = clearance(pp, world)
            gt[f'{phase}_min_clear_m'] = round(float(c.min()), 2) if len(c) else float('nan')
            close = c < 0.02
            gt[f'{phase}_contacts'] = int(np.sum(close[1:] & ~close[:-1]) + close[0]) if len(c) else 0
            dt = np.diff(pp[:, 0]) if len(pp) > 1 else np.zeros(0)
            gt[f'{phase}_near_s'] = round(float(dt[c[1:] < 0.10].sum()), 0) if len(dt) else 0.0
            gt[f'_{phase}_clear'] = c
        if legs:
            ok = [l for l in legs if l['result'] == 'reached']
            gt['tour_reached'] = len(ok)
            gt['tour_goals'] = len(legs)
            gt['tour_time_s'] = round(sum(l['time_s'] for l in legs), 0)
            gt['tour_leg_s'] = round(float(np.mean([l['time_s'] for l in ok])), 1) if ok else float('nan')
            gt['tour_eff'] = round(float(np.nanmean([l['efficiency'] for l in ok])), 2) if ok else float('nan')
            gt['tour_spl'] = round(float(np.mean([l['spl'] for l in legs])), 2)
            gt['tour_aborts'] = sum(l['aborts'] for l in legs)

    visited = []
    for room, (x0, x1, y0, y1) in (ROOMS.get(rooms) or {}).items():
        m = 0.3
        if len(pose) and np.any((pose[:, 1] > x0 + m) & (pose[:, 1] < x1 - m)
                                & (pose[:, 2] > y0 + m) & (pose[:, 2] < y1 - m)):
            visited.append(room)

    duration = (t_end - t0) if (t0 and t_end) else float('nan')
    return {
        'name': meta.get('NAME', os.path.basename(d.rstrip('/'))),
        'drive': meta.get('DRIVE', '?'),
        'duration_s': round(duration, 0),
        'finished': t_done is not None,
        'goals': count(lines, 'Goal sent'),
        'reached': count(lines, 'Goal reached'),
        'dropped_early': count(lines, 'explored before arrival'),
        'nav2_aborts': count(lines, 'controller ABORTED') + count(lines, 'planner ABORTED'),
        'recovery_retries': count(lines, 'Stuck recovery re-sent'),
        'recovery_moves': count(rlines, 'stepping '),
        'recovery_blocked': count(rlines, 'no free space'),
        'no_progress': count(lines, 'no progress'),
        'timeouts': count(lines, 'goal timeout'),
        'wall_drops': count(lines, 'from the goal'),
        'area_m2': round(coverage[-1][1], 1) if coverage else float('nan'),
        'distance_m': round(dist, 1),
        'still_pct': round(100 * still / duration, 0) if duration == duration and duration > 0 else float('nan'),
        'rooms_entered': len([r for r in visited if r != 'corridor']),
        'rooms': ' '.join(r.replace('room ', 'R').split(' ')[0] for r in visited if r != 'corridor'),
        **{f'cmd_{k}_pct': round(100 * v / max(1, len(moving)), 0) for k, v in mix.items()},
        'cmd_sideways_drift_pct': round(100 * drift / max(1, len(moving)), 0),
        **gt,
        '_coverage': coverage,
        '_pose': pose,
        '_legs': legs,
        '_dir': d,
    }


def style(ax):
    ax.set_facecolor(SURFACE)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=TEXT_2, labelsize=9)
    ax.grid(color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def draw_world(ax, world):
    """Walls and furniture footprints from the world JSON, under the paths."""
    from matplotlib.patches import Polygon
    for o in world['obstacles']:
        pts = rect_points(o['cx'], o['cy'], o['yaw'], o['sx'], o['sy'], 1)
        ax.add_patch(Polygon(pts, closed=True, fc='#4a4946' if o['kind'] == 'wall' else '#cfcdc6',
                             ec='none', zorder=1))


def plot_world(trials, out, world):
    """tour.png (per-goal outcome and time) and clearance.png (distance to the nearest obstacle)."""
    import matplotlib.pyplot as plt
    drives = [d for d in ('diff', 'hybrid', 'omni') if any(t['drive'] == d for t in trials)]
    goals = [g['name'] for g in world['tour']]
    width = 0.8 / max(1, len(drives))

    if any(t['_legs'] for t in trials):
        fig, axes = plt.subplots(1, 2, figsize=(12, 3.8))
        for ax in axes:
            style(ax)
            ax.grid(axis='x', visible=False)
        for k, d in enumerate(drives):
            group = [t for t in trials if t['drive'] == d and t['_legs']]
            n = len(group)
            reached, times, sds = [], [], []
            for g in goals:
                legs = [l for t in group for l in t['_legs'] if l['name'] == g]
                ok = [l['time_s'] for l in legs if l['result'] == 'reached']
                reached.append(100 * len(ok) / max(1, n))
                times.append(np.mean(ok) if ok else np.nan)
                sds.append(np.std(ok, ddof=1) if len(ok) > 1 else 0)
            xs = np.arange(len(goals)) + (k - (len(drives) - 1) / 2) * width
            axes[0].bar(xs, reached, width=width, color=DRIVE_COLOR[d], edgecolor=SURFACE, linewidth=2,
                        label=f'--drive {d} (n={n})', zorder=2)
            axes[1].bar(xs, times, width=width, color=DRIVE_COLOR[d], edgecolor=SURFACE, linewidth=2, zorder=2)
            axes[1].errorbar(xs, times, yerr=sds, fmt='none', ecolor=TEXT_2, elinewidth=1, capsize=2, zorder=3)
            for x, v in zip(xs, reached):
                axes[0].text(x, v + 2, f'{v:.0f}', ha='center', va='bottom', fontsize=7, color=TEXT_2)
        for ax in axes:
            ax.set_xticks(range(len(goals)), [g.replace(' ', '\n', 1) for g in goals], fontsize=8)
        axes[0].set_ylim(0, 115)
        axes[0].set_ylabel('goal reached (% of trials)')
        axes[0].set_title('Navigation tour: goals reached', loc='left', fontsize=11, color=TEXT)
        axes[1].set_ylabel('time to reach (s), mean ± SD')
        axes[1].set_title('Navigation tour: time per goal (reached legs)', loc='left', fontsize=11, color=TEXT)
        axes[0].legend(frameon=False, loc='lower left', fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(out, 'tour.png'), dpi=140, bbox_inches='tight')
        plt.close(fig)

    if any(t.get('_room_cov') for t in trials):
        rooms = list(world['rooms'])
        fig, ax = plt.subplots(figsize=(11, 3.4))
        style(ax)
        ax.grid(axis='x', visible=False)
        for k, d in enumerate(drives):
            group = [t for t in trials if t['drive'] == d and t.get('_room_cov')]
            if not group:
                continue
            vals = [[t['_room_cov'][r] for t in group] for r in rooms]
            xs = np.arange(len(rooms)) + (k - (len(drives) - 1) / 2) * width
            ax.bar(xs, [np.mean(v) for v in vals], width=width, color=DRIVE_COLOR[d], edgecolor=SURFACE,
                   linewidth=2, label=f'--drive {d} (n={len(group)})', zorder=2)
            for x, v in zip(xs, vals):      # every trial as a dot: the spread at n = 3
                ax.plot([x] * len(v), v, 'o', color=TEXT, markersize=3, zorder=3)
        ax.axhline(80, color=TEXT_2, linewidth=1, linestyle=':')
        ax.set_xticks(range(len(rooms)), [r.replace(' ', '\n', 1) for r in rooms], fontsize=8)
        ax.set_ylim(0, 105)
        ax.set_ylabel('cells known (%)')
        ax.set_title('Mapped when the explorer reported "complete", per room (bar = mean, dot = trial)',
                     loc='left', fontsize=11, color=TEXT)
        ax.legend(frameon=False, loc='lower left', fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(out, 'room_coverage.png'), dpi=140, bbox_inches='tight')
        plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(11, 3.6), sharey=True)
    for ax, phase, title in ((axes[0], 'explore', 'Exploration'), (axes[1], 'tour', 'Navigation tour')):
        style(ax)
        for d in drives:
            c = np.concatenate([t.get(f'_{phase}_clear', np.zeros(0)) for t in trials if t['drive'] == d] or [np.zeros(0)])
            if not len(c):
                continue
            c = np.sort(c)
            ax.plot(c, 100 * np.arange(1, len(c) + 1) / len(c), color=DRIVE_COLOR[d], linewidth=2,
                    label=f'--drive {d}')
        ax.axvline(0.10, color=TEXT_2, linewidth=1, linestyle=':')
        ax.text(0.105, 4, '10 cm', fontsize=8, color=TEXT_2)
        ax.set_xlim(0, 1.0)
        ax.set_ylim(0, 100)
        ax.set_xlabel('distance from the body to the nearest wall or furniture (m)')
        ax.set_title(f'{title}: share of time closer than x', loc='left', fontsize=11, color=TEXT)
    axes[0].set_ylabel('time (%)')
    axes[0].legend(frameon=False, loc='lower right')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'clearance.png'), dpi=140, bbox_inches='tight')
    plt.close(fig)


def plot_all(trials, out, rooms, world=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size': 10, 'text.color': TEXT, 'axes.labelcolor': TEXT_2,
                         'figure.facecolor': SURFACE, 'savefig.facecolor': SURFACE})
    drives = [d for d in ('diff', 'hybrid', 'omni') if any(t['drive'] == d for t in trials)]

    # Mapped area over time
    fig, ax = plt.subplots(figsize=(7.5, 4))
    style(ax)
    for t in trials:
        if not t['_coverage']:
            continue
        x, y = zip(*t['_coverage'])
        ax.plot(x, y, color=DRIVE_COLOR.get(t['drive'], TEXT_2), linewidth=2, alpha=0.9,
                label=t['drive'])
    handles, labels = ax.get_legend_handles_labels()
    uniq = dict(zip(labels, handles))
    ax.legend(uniq.values(), [f'--drive {k}' for k in uniq], frameon=False, loc='lower right')
    if world is not None:
        b = world['bounds']
        floor = (b[1] - b[0]) * (b[3] - b[2])
        ax.axhline(floor, color=TEXT_2, linewidth=1, linestyle='--')
        ax.text(0, floor, f' building floor {floor:.0f} m²', va='bottom', fontsize=8, color=TEXT_2)
    ax.set_xlabel('time since Auto Explore (s)')
    ax.set_ylabel('mapped area (m²)')
    ax.set_title('Mapped area over time', loc='left', fontsize=12, color=TEXT)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'coverage.png'), dpi=140, bbox_inches='tight')
    plt.close(fig)

    # Goal outcomes per trial (small multiples, one shared y scale)
    keys = [('reached', 'reached'), ('dropped_early', 'dropped: frontier seen'),
            ('nav2_aborts', 'Nav2 aborts'), ('no_progress', 'dropped: no progress'),
            ('recovery_retries', 'recovery retries')]
    fig, axes = plt.subplots(1, len(keys), figsize=(11, 3.4), sharey=True)
    ymax = max(1, max(t[k] for t in trials for k, _ in keys))
    for ax, (k, label) in zip(axes, keys):
        style(ax)
        ax.grid(axis='x', visible=False)
        names = [t['name'] for t in trials]
        vals = [t[k] for t in trials]
        ax.bar(range(len(trials)), vals, width=0.7,
               color=[DRIVE_COLOR.get(t['drive'], TEXT_2) for t in trials],
               edgecolor=SURFACE, linewidth=2)
        for i, v in enumerate(vals):
            ax.text(i, v + ymax * 0.02, str(v), ha='center', va='bottom', fontsize=8, color=TEXT_2)
        ax.set_xticks(range(len(trials)), names, rotation=45, ha='right', fontsize=8)
        ax.set_title(label, fontsize=10, color=TEXT)
        ax.set_ylim(0, ymax * 1.15)
    axes[0].set_ylabel('count per trial')
    from matplotlib.patches import Patch
    fig.legend([Patch(color=DRIVE_COLOR[d]) for d in drives], [f'--drive {d}' for d in drives],
               frameon=False, loc='upper right', ncol=len(drives))
    fig.suptitle('Goal outcomes per trial', x=0.01, ha='left', fontsize=12, color=TEXT)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(os.path.join(out, 'outcomes.png'), dpi=140, bbox_inches='tight')
    plt.close(fig)

    # Command mix per drive (stacked 100 % bars)
    parts = [('forward', '#2a78d6'), ('turn', '#1baf7a'), ('reverse', '#eb6834'), ('sidestep', '#6250d6')]
    fig, ax = plt.subplots(figsize=(7, 2.6))
    style(ax)
    ax.grid(axis='y', visible=False)
    for i, d in enumerate(drives):
        group = [t for t in trials if t['drive'] == d]
        left = 0
        for k, c in parts:
            v = float(np.mean([t[f'cmd_{k}_pct'] for t in group]))
            ax.barh(i, v, left=left, color=c, edgecolor=SURFACE, linewidth=2, height=0.6,
                    label=k if i == 0 else None)
            if v >= 6:
                ax.text(left + v / 2, i, f'{v:.0f}%', ha='center', va='center', fontsize=8, color='white')
            left += v
    ax.set_yticks(range(len(drives)), [f'--drive {d}' for d in drives])
    ax.set_xlim(0, 100)
    ax.set_xlabel('share of non-zero velocity commands, by main direction (%)')
    ax.legend(frameon=False, ncol=4, loc='upper center', bbox_to_anchor=(0.5, -0.32))
    ax.set_title('What the dog was told to do', loc='left', fontsize=12, color=TEXT)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'commands.png'), dpi=140, bbox_inches='tight')
    plt.close(fig)

    # Paths over the final map
    fig, axes = plt.subplots(1, len(drives), figsize=(5.2 * len(drives), 5.6), squeeze=False)
    for ax, d in zip(axes[0], drives):
        ax.set_facecolor(SURFACE)
        group = [t for t in trials if t['drive'] == d]
        cm = next((os.path.join(t['_dir'], 'costmap.npz') for t in reversed(group)
                   if os.path.exists(os.path.join(t['_dir'], 'costmap.npz'))), None)
        if cm:
            z = np.load(cm)
            data, r, (ox, oy) = z['data'], float(z['resolution']), z['origin']
            img = np.full(data.shape, 1.0)
            img[data >= 0] = 0.93
            img[data >= 99] = 0.35
            h, w = data.shape
            ax.imshow(img, cmap='gray', vmin=0, vmax=1, origin='lower',
                      extent=(ox, ox + w * r, oy, oy + h * r), interpolation='nearest')
        if world is not None:
            draw_world(ax, world)
        for (x0, x1, y0, y1) in (ROOMS.get(rooms) or {}).values():
            ax.plot([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0], color=TEXT_2, linewidth=0.6,
                    linestyle=':')
        placed = []    # end points already labelled: stack labels of nearby stops
        for t in group:
            tp = t.get('_tour_pose', np.zeros((0, 4)))
            if len(tp):
                ax.plot(tp[:, 1], tp[:, 2], color=DRIVE_COLOR[d], linewidth=1.2, alpha=0.6,
                        linestyle='--', zorder=2)
            p = t['_pose']
            if len(p):
                ax.plot(p[:, 1], p[:, 2], color=DRIVE_COLOR[d], linewidth=2, alpha=0.75, zorder=3)
                ax.plot(p[-1, 1], p[-1, 2], 'o', color=DRIVE_COLOR[d], markersize=8,
                        markeredgecolor=SURFACE, markeredgewidth=2)
                near = sum(1 for q in placed if np.hypot(q[0] - p[-1, 1], q[1] - p[-1, 2]) < 1.5)
                placed.append((p[-1, 1], p[-1, 2]))
                ax.annotate(t['name'], (p[-1, 1], p[-1, 2]), xytext=(6, 4 - 11 * near),
                            textcoords='offset points', fontsize=8, color=TEXT,
                            bbox=dict(boxstyle='round,pad=0.15', fc=SURFACE, ec='none', alpha=0.8))
        if rooms == 'agri_rooms':
            ax.set_xlim(-7.5, 7.5)
            ax.set_ylim(-3, 12)
        elif world is not None:
            b = world['bounds']
            ax.set_xlim(b[0] - 0.4, b[1] + 0.4)
            ax.set_ylim(b[2] - 0.4, b[3] + 0.4)
        ax.set_aspect('equal')
        tour_note = ', dashed = tour' if any(len(t.get('_tour_pose', ())) for t in group) else ''
        ax.set_title(f'--drive {d}: {len(group)} trial(s), dot = where exploring stopped{tour_note}',
                     loc='left', fontsize=10, color=TEXT)
        ax.tick_params(colors=TEXT_2, labelsize=8)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'trajectories.png'), dpi=140, bbox_inches='tight')
    plt.close(fig)
    if world is not None:
        plot_world(trials, out, world)
        plot_maps(trials, out, world)


def plot_maps(trials, out, world):
    """maps.png: each trial's costmap when exploration ended, ground-truth footprints and the path."""
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch, Polygon
    shown = [t for t in trials if os.path.exists(os.path.join(t['_dir'], 'costmap_explore.npz'))]
    if not shown:
        return
    drives = [d for d in ('diff', 'hybrid', 'omni') if any(t['drive'] == d for t in shown)]
    cols = max(sum(t['drive'] == d for t in shown) for d in drives)
    fig, axes = plt.subplots(len(drives), cols, figsize=(5.6 * cols, 4.6 * len(drives)), squeeze=False)
    b = world['bounds']
    for row, d in enumerate(drives):
        group = [t for t in shown if t['drive'] == d]
        for col in range(cols):
            ax = axes[row][col]
            if col >= len(group):
                ax.axis('off')
                continue
            t = group[col]
            z = np.load(os.path.join(t['_dir'], 'costmap_explore.npz'))
            data, r, (ox, oy) = z['data'], float(z['resolution']), z['origin']
            img = np.ones(data.shape + (3,))
            img[data < 0] = (0.98, 0.84, 0.80)                        # unknown
            img[(data >= 0) & (data < 50)] = (0.96, 0.96, 0.95)       # free / low cost
            img[(data >= 50) & (data < 99)] = (0.78, 0.78, 0.80)      # inflation
            img[data >= 99] = (0.25, 0.25, 0.27)                      # obstacle
            h, w = data.shape
            ax.imshow(img, origin='lower', extent=(ox, ox + w * r, oy, oy + h * r), interpolation='nearest')
            for o in world['obstacles']:
                ax.add_patch(Polygon(rect_points(o['cx'], o['cy'], o['yaw'], o['sx'], o['sy'], 1), closed=True,
                                     fill=False, ec='#2a78d6', lw=0.6))
            p = t['_pose']
            if len(p):
                ax.plot(p[:, 1], p[:, 2], color=DRIVE_COLOR[d], linewidth=2)
                ax.plot(p[-1, 1], p[-1, 2], 'o', color=DRIVE_COLOR[d], markersize=7,
                        markeredgecolor=SURFACE, markeredgewidth=1.5)
            ax.set_xlim(b[0] - 0.4, b[1] + 0.4)
            ax.set_ylim(b[2] - 0.4, b[3] + 0.4)
            ax.set_aspect('equal')
            ax.tick_params(colors=TEXT_2, labelsize=7)
            ax.set_title(f"{t['name']}: {t['duration_s']:.0f} s, floor {t.get('coverage_pct', float('nan')):.0f} % known",
                         loc='left', fontsize=9, color=TEXT)
    fig.legend([Patch(color=(0.98, 0.84, 0.80)), Patch(color=(0.78, 0.78, 0.80)), Patch(color=(0.25, 0.25, 0.27)),
                Patch(fill=False, ec='#2a78d6'), Patch(color=DRIVE_COLOR.get('hybrid')), Patch(color=DRIVE_COLOR.get('omni'))],
               ['unknown', 'inflation', 'obstacle', 'true furniture / walls', 'hybrid path', 'omni path'],
               frameon=False, loc='upper center', ncol=6, fontsize=9, bbox_to_anchor=(0.5, 1.02))
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(os.path.join(out, 'maps.png'), dpi=110, bbox_inches='tight')
    plt.close(fig)


def write_graph_data(trials, out, world):
    """The numbers behind every plot, as CSV in OUT/data/ and as Markdown tables in OUT/graph_data.md."""
    dd = os.path.join(out, 'data')
    os.makedirs(dd, exist_ok=True)
    drives = [d for d in ('diff', 'hybrid', 'omni') if any(t['drive'] == d for t in trials)]

    def save(name, header, rows):
        with open(os.path.join(dd, name), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(rows)
        return header, rows

    tables = []
    tables.append(('Mapped area over time (coverage.png)', 'coverage_curves.csv', save(
        'coverage_curves.csv', ['trial', 'drive', 't_s', 'area_m2'],
        [[t['name'], t['drive'], round(x, 1), round(y, 1)] for t in trials for x, y in t['_coverage']])))
    mix = ['forward', 'turn', 'reverse', 'sidestep']
    tables.append(('Command mix per drive, % of non-zero commands (commands.png)', 'commands.csv', save(
        'commands.csv', ['drive'] + mix,
        [[d] + [round(float(np.mean([t[f'cmd_{k}_pct'] for t in trials if t['drive'] == d])), 1) for k in mix]
         for d in drives])))
    keys = ['reached', 'dropped_early', 'nav2_aborts', 'no_progress', 'recovery_retries']
    tables.append(('Explorer goal outcomes per trial (outcomes.png)', 'outcomes.csv', save(
        'outcomes.csv', ['trial', 'drive'] + keys, [[t['name'], t['drive']] + [t[k] for k in keys] for t in trials])))
    if world is not None:
        rooms = list(world['rooms'])
        tables.append(('Floor known per room when exploration ended, % (room_coverage.png)', 'room_coverage.csv', save(
            'room_coverage.csv', ['trial', 'drive'] + rooms,
            [[t['name'], t['drive']] + [round(t['_room_cov'].get(r, float('nan')), 1) for r in rooms]
             for t in trials if t.get('_room_cov')])))
        tables.append(('Every tour leg (tour.png)', 'tour_legs.csv', save(
            'tour_legs.csv', ['trial', 'drive', 'goal', 'result', 'time_s', 'path_m', 'shortest_m', 'efficiency',
                              'spl', 'nav2_aborts'],
            [[t['name'], t['drive'], l['name'], l['result'], round(l['time_s'], 1), round(l['path_m'], 2),
              round(l['shortest_m'], 2), round(l['efficiency'], 2) if l['efficiency'] == l['efficiency'] else '',
              round(l['spl'], 2), l['aborts']] for t in trials for l in t['_legs']])))
        q = [0, 1, 5, 10, 25, 50, 75, 90]
        rows = []
        for d in drives:
            for phase in ('explore', 'tour'):
                c = np.concatenate([t.get(f'_{phase}_clear', np.zeros(0)) for t in trials if t['drive'] == d]
                                   or [np.zeros(0)])
                if len(c):
                    rows.append([d, phase, len(c)] + [round(float(np.percentile(c, x)), 3) for x in q]
                                + [round(100 * float(np.mean(c < 0.10)), 2), round(100 * float(np.mean(c < 0.25)), 1)])
        tables.append(('Distance from the body to the nearest wall or furniture, m (clearance.png)', 'clearance.csv', save(
            'clearance.csv', ['drive', 'phase', 'samples'] + [f'p{x}' if x else 'min' for x in q]
            + ['pct_below_0.10m', 'pct_below_0.25m'], rows)))
    md = ['# Graph data (analyze.py)', '',
          'The numbers behind each plot; the same as CSV in [`data/`](data/).']
    for title, fname, (header, rows) in tables:
        md += ['', f'## {title}', '', f'[`data/{fname}`](data/{fname})', '',
               '| ' + ' | '.join(header) + ' |', '|' + '---|' * len(header)]
        md += ['| ' + ' | '.join(str(v) for v in r) + ' |' for r in rows]
    open(os.path.join(out, 'graph_data.md'), 'w', encoding='utf-8').write('\n'.join(md) + '\n')


COLUMNS = ['name', 'drive', 'duration_s', 'finished', 'rooms_entered', 'rooms', 'area_m2',
           'distance_m', 'still_pct', 'goals', 'reached', 'dropped_early', 'nav2_aborts',
           'recovery_retries', 'recovery_moves', 'recovery_blocked', 'no_progress', 'timeouts',
           'wall_drops', 'cmd_forward_pct', 'cmd_turn_pct', 'cmd_reverse_pct', 'cmd_sidestep_pct',
           'cmd_sideways_drift_pct']
WORLD_COLUMNS = ['coverage_pct', 'rooms_mapped', 't90_s', 'explore_min_clear_m', 'explore_contacts', 'explore_near_s',
                 'tour_reached', 'tour_goals', 'tour_time_s', 'tour_leg_s', 'tour_eff', 'tour_spl',
                 'tour_aborts', 'tour_min_clear_m', 'tour_contacts', 'tour_near_s']
# (column, label) for stats.md, in reading order
STATS = [('duration_s', 'Exploration time (s)'), ('finished', 'Reported "complete" (share)'),
         ('area_m2', 'Mapped area (m²)'), ('coverage_pct', 'Coverage of the floor (%)'),
         ('rooms_mapped', 'Rooms ≥ 80 % mapped at "complete" (of 7)'),
         ('t90_s', 'Time to 90 % of the final map (s)'), ('rooms_entered', 'Rooms walked into'),
         ('distance_m', 'Distance walked (m)'), ('still_pct', 'Time standing still (%)'),
         ('goals', 'Explorer goals sent'), ('nav2_aborts', 'Nav2 aborts'),
         ('recovery_retries', 'Stuck-recovery retries'), ('no_progress', 'Goals dropped: no progress'),
         ('explore_min_clear_m', 'Exploration: closest approach (m)'),
         ('explore_contacts', 'Exploration: contacts (< 2 cm)'),
         ('explore_near_s', 'Exploration: time within 10 cm (s)'),
         ('tour_reached', 'Tour: goals reached'), ('tour_time_s', 'Tour: total time (s)'),
         ('tour_leg_s', 'Tour: time per reached goal (s)'), ('tour_eff', 'Tour: path efficiency'),
         ('tour_spl', 'Tour: SPL'), ('tour_aborts', 'Tour: Nav2 aborts'),
         ('tour_min_clear_m', 'Tour: closest approach (m)'), ('tour_contacts', 'Tour: contacts (< 2 cm)'),
         ('tour_near_s', 'Tour: time within 10 cm (s)'),
         ('cmd_forward_pct', 'Commands mainly forward (%)'), ('cmd_reverse_pct', 'Commands mainly reverse (%)'),
         ('cmd_sidestep_pct', 'Commands mainly sideways (%)')]


def write_stats(trials, path, world):
    drives = [d for d in ('diff', 'hybrid', 'omni') if any(t['drive'] == d for t in trials)]
    groups = {d: [t for t in trials if t['drive'] == d] for d in drives}
    try:   # significance tests when comparing two drives and scipy is there
        from scipy import stats as sps
    except ImportError:
        sps = None
    test = sps is not None and len(drives) == 2
    md = ['| Metric | ' + ' | '.join(f'`{d}` (n={len(groups[d])}) mean ± SD [min–max]' for d in drives)
          + (' | p (Welch t) |' if test else ' |'),
          '|---|' + '---|' * (len(drives) + test)]
    for col, label in STATS:
        cells, samples = [], []
        for d in drives:
            vals = [float(t[col]) for t in groups[d] if col in t and isinstance(t[col], (int, float, bool))
                    and t[col] == t[col]]
            if not vals:
                cells.append('')
                continue
            m, sd = np.mean(vals), (np.std(vals, ddof=1) if len(vals) > 1 else 0.0)
            f = '.2f' if col.endswith('_m') or col in ('tour_eff', 'tour_spl', 'finished') else \
                '.1f' if max(abs(v) for v in vals) < 20 else '.0f'
            cells.append(f'{m:{f}} ± {sd:{f}} [{min(vals):{f}}–{max(vals):{f}}]')
            samples.append(vals)
        if any(cells):
            p = ''
            if test and len(samples) == 2 and min(len(s) for s in samples) > 1 \
                    and (np.std(samples[0]) > 0 or np.std(samples[1]) > 0):
                p = f'{sps.ttest_ind(samples[0], samples[1], equal_var=False).pvalue:.3f}'
            md.append(f'| {label} | ' + ' | '.join(cells) + (f' | {p} |' if test else ' |'))
    if test and world is not None and all(any(t['_legs'] for t in groups[d]) for d in drives):
        # Per tour leg (n = trials x goals): more samples than per trial
        legs = {d: [l for t in groups[d] for l in t['_legs']] for d in drives}
        ok = {d: sum(l['result'] == 'reached' for l in legs[d]) for d in drives}
        p_ok = sps.fisher_exact([[ok[d], len(legs[d]) - ok[d]] for d in drives]).pvalue
        times = {d: [l['time_s'] for l in legs[d] if l['result'] == 'reached' and l['path_m'] > 0.05]
                 for d in drives}
        p_t = sps.mannwhitneyu(times[drives[0]], times[drives[1]]).pvalue
        a, b = drives
        md += ['', f'Per tour leg: reached {ok[a]}/{len(legs[a])} (`{a}`) vs {ok[b]}/{len(legs[b])} (`{b}`), '
               f'Fisher exact p = {p_ok:.3f}; time per reached leg, median {np.median(times[a]):.0f} s '
               f'(n={len(times[a])}) vs {np.median(times[b]):.0f} s (n={len(times[b])}), '
               f'Mann-Whitney U p = {p_t:.4f}.']
    if world is not None and any(t.get('_room_cov') for t in trials):
        md += ['', '| Room: mapped when exploration ended | ' + ' | '.join(f'`{d}` mean % [min–max]' for d in drives) + ' |',
               '|---|' + '---|' * len(drives)]
        for room in world['rooms']:
            cells = []
            for d in drives:
                v = [t['_room_cov'][room] for t in groups[d] if t.get('_room_cov')]
                cells.append(f'{np.mean(v):.0f} [{min(v):.0f}–{max(v):.0f}]' if v else '')
            md.append(f'| {room} | ' + ' | '.join(cells) + ' |')
    if world is not None and any(t['_legs'] for t in trials):
        md += ['', '| Tour goal | ' + ' | '.join(f'`{d}` reached · mean time' for d in drives) + ' |',
               '|---|' + '---|' * len(drives)]
        for g in (x['name'] for x in world['tour']):
            cells = []
            for d in drives:
                legs = [l for t in groups[d] for l in t['_legs'] if l['name'] == g]
                ok = [l['time_s'] for l in legs if l['result'] == 'reached']
                other = sorted({l['result'] for l in legs if l['result'] != 'reached'})
                cells.append(f"{len(ok)}/{len(legs)} · {np.mean(ok):.0f} s" if ok else f"0/{len(legs)}"
                             + (f" ({', '.join(other)})" if other else ''))
            md.append(f'| {g} | ' + ' | '.join(cells) + ' |')
    head = ['# Trial statistics (analyze.py)', '',
            'Per drive mode over trials: mean ± SD [min–max]'
            + ('; p from Welch\'s t-test.' if test else '.'), '']
    open(path, 'w', encoding='utf-8').write('\n'.join(head + md) + '\n')
    return md


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('trials', nargs='+')
    ap.add_argument('--out', default='.')
    ap.add_argument('--rooms', default='', help='sim world: agri_rooms, or a world with a '
                    'gazebo/worlds/NAME.json (lab_office) for ground-truth coverage, clearance and the tour')
    ap.add_argument('--resolution', type=float, default=0.1, help='global costmap cell size (m)')
    ap.add_argument('--no-plots', action='store_true')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    world = load_world(a.rooms)
    trials = [trial_metrics(d, a.rooms, a.resolution, world) for d in a.trials if os.path.isdir(d)]
    if not trials:
        sys.exit('no trial directories')
    columns = COLUMNS + (WORLD_COLUMNS if world is not None else [])

    with open(os.path.join(a.out, 'trials.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(columns)
        for t in trials:
            w.writerow([t.get(c, '') for c in columns])

    md = ['| ' + ' | '.join(columns) + ' |', '|' + '---|' * len(columns)]
    md += ['| ' + ' | '.join(str(t.get(c, '')) for c in columns) + ' |' for t in trials]
    md += ['', '| drive | trials | ' + ' | '.join(columns[2:]) + ' |',
           '|' + '---|' * (len(columns)) ]
    for d in sorted({t['drive'] for t in trials}):
        g = [t for t in trials if t['drive'] == d]
        cells = []
        for c in columns[2:]:
            vals = [t[c] for t in g if isinstance(t.get(c), (int, float, bool)) and t[c] == t[c]]
            cells.append(f'{np.mean([float(v) for v in vals]):.1f}' if vals else '')
        md.append(f'| {d} | {len(g)} | ' + ' | '.join(cells) + ' |')
    open(os.path.join(a.out, 'summary.md'), 'w', encoding='utf-8').write(
        '\n'.join(['# Trial summary (analyze.py)', '', 'One row per trial, then the mean per drive mode.', ''] + md) + '\n')
    print('\n'.join(md))
    print('\n'.join(write_stats(trials, os.path.join(a.out, 'stats.md'), world)))
    write_graph_data(trials, a.out, world)

    if not a.no_plots:
        plot_all(trials, a.out, a.rooms, world)
        print(f'plots in {a.out}')


if __name__ == '__main__':
    main()
