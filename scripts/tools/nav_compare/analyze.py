#!/usr/bin/env python3
"""
analyze.py: compare navigation trials recorded with record.sh / sim_trial.sh.

  analyze.py TRIAL_DIR [TRIAL_DIR ...] [--out DIR] [--rooms agri_rooms]

Per trial (from frontier.log, recovery.log, pose.csv, cmd.csv, costmap.npz):
duration, goals sent / reached / dropped early, Nav2 aborts, no-progress and
timeout drops, stuck-recovery moves, mapped area, distance walked, time
standing still, the command mix (forward / turn in place / reverse /
sidestep) and, with --rooms agri_rooms, which rooms of the sim building the
dog entered.

Writes to --out (default: the current directory):
  trials.csv           one row per trial
  summary.md           the same as a Markdown table, plus per-drive means
  coverage.png         mapped area over time
  outcomes.png         goal outcomes per trial
  commands.png         command mix per drive
  trajectories.png     paths over the final map, one panel per drive

Needs numpy + matplotlib. If the host's matplotlib is broken, run.sh runs this
inside the go2-3d image, which has both.
"""

import argparse
import csv
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


def trial_metrics(d, rooms, res):
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
    t_end = t_done or float(meta.get('END', 0)) or (ts(lines[-1]) if lines else None)
    coverage = []
    for l in lines:
        m = re.search(r'known=(\d+)', l)
        t = ts(l)
        if m and t is not None and t0 is not None and t >= t0:
            coverage.append((t - t0, int(m.group(1)) * res * res))

    pose = read_csv(os.path.join(d, 'pose.csv'))
    cmd = read_csv(os.path.join(d, 'cmd.csv'))
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
        'rooms_entered': len([r for r in visited if r.startswith('room')]),
        'rooms': ' '.join(r.replace('room ', 'R') for r in visited if r.startswith('room')),
        **{f'cmd_{k}_pct': round(100 * v / max(1, len(moving)), 0) for k, v in mix.items()},
        'cmd_sideways_drift_pct': round(100 * drift / max(1, len(moving)), 0),
        '_coverage': coverage,
        '_pose': pose,
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


def plot_all(trials, out, rooms):
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
        for (x0, x1, y0, y1) in (ROOMS.get(rooms) or {}).values():
            ax.plot([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0], color=TEXT_2, linewidth=0.6,
                    linestyle=':')
        placed = []    # end points already labelled: stack labels of nearby stops
        for t in group:
            p = t['_pose']
            if len(p):
                ax.plot(p[:, 1], p[:, 2], color=DRIVE_COLOR[d], linewidth=2, alpha=0.75)
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
        ax.set_aspect('equal')
        ax.set_title(f'--drive {d}: {len(group)} trial(s), dot = where it stopped',
                     loc='left', fontsize=10, color=TEXT)
        ax.tick_params(colors=TEXT_2, labelsize=8)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'trajectories.png'), dpi=140, bbox_inches='tight')
    plt.close(fig)


COLUMNS = ['name', 'drive', 'duration_s', 'finished', 'rooms_entered', 'rooms', 'area_m2',
           'distance_m', 'still_pct', 'goals', 'reached', 'dropped_early', 'nav2_aborts',
           'recovery_retries', 'recovery_moves', 'recovery_blocked', 'no_progress', 'timeouts',
           'wall_drops', 'cmd_forward_pct', 'cmd_turn_pct', 'cmd_reverse_pct', 'cmd_sidestep_pct',
           'cmd_sideways_drift_pct']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('trials', nargs='+')
    ap.add_argument('--out', default='.')
    ap.add_argument('--rooms', default='', help='agri_rooms: count rooms entered in the sim building')
    ap.add_argument('--resolution', type=float, default=0.1, help='global costmap cell size (m)')
    ap.add_argument('--no-plots', action='store_true')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    trials = [trial_metrics(d, a.rooms, a.resolution) for d in a.trials if os.path.isdir(d)]
    if not trials:
        sys.exit('no trial directories')

    with open(os.path.join(a.out, 'trials.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        for t in trials:
            w.writerow([t[c] for c in COLUMNS])

    md = ['| ' + ' | '.join(COLUMNS) + ' |', '|' + '---|' * len(COLUMNS)]
    md += ['| ' + ' | '.join(str(t[c]) for c in COLUMNS) + ' |' for t in trials]
    md += ['', '| drive | trials | ' + ' | '.join(COLUMNS[2:]) + ' |',
           '|' + '---|' * (len(COLUMNS)) ]
    for d in sorted({t['drive'] for t in trials}):
        g = [t for t in trials if t['drive'] == d]
        cells = []
        for c in COLUMNS[2:]:
            vals = [t[c] for t in g if isinstance(t[c], (int, float, bool)) and t[c] == t[c]]
            cells.append(f'{np.mean([float(v) for v in vals]):.1f}' if vals else '')
        md.append(f'| {d} | {len(g)} | ' + ' | '.join(cells) + ' |')
    open(os.path.join(a.out, 'summary.md'), 'w').write('\n'.join(md) + '\n')
    print('\n'.join(md))

    if not a.no_plots:
        plot_all(trials, a.out, a.rooms)
        print(f'plots in {a.out}')


if __name__ == '__main__':
    main()
