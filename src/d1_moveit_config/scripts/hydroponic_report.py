#!/usr/bin/env python3
"""
hydroponic_report.py: Mission Score, graphs and tables from full-mission trials
(tests/hydroponic_trial.py -> full_mission_test.py), for
docs/simulation/hydroponic_trial.md.

  python3 hydroponic_report.py TRIAL_DIR [TRIAL_DIR ...] --out docs/images/hydroponic_trial

Mission Score (0-100), per trial:
  40  collisions   40 with no robot contact during the mission (Gazebo contact
                   sensors on every link), -10 per collision (contacts with one
                   model, a new one after 1 s without contact)
  40  completion   40 x the mean over the six phases of: 1 for a successful
                   phase; for a failed Cartesian phase the fraction MoveIt
                   could plan; else 0
  20  accuracy     20 when every executed Cartesian phase (Extend, Orbit,
                   Retract) stays within 5 % of its planned path (RMS
                   cross-track error / path length), down to 0 at 10 %; 0 if
                   none was executed

Writes score.json, summary.md and the graphs: trajectory_3d_NAME.png,
phase_times.png, joint_velocities_NAME.png, deviation_NAME.png.
"""
import argparse
import collections
import csv
import json
import math
import os

import numpy as np

PHASES = ['Navigation', 'Approach', 'Extend', 'Orbit', 'Retract', 'Home']
CARTESIAN = ('Extend', 'Orbit', 'Retract')
JOINTS = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
VMAX = [1.05, 1.05, 1.05, 1.73, 1.73, 1.73]
PHASE_COLOR = {'Setup': '#a8a7a2', 'Navigation': '#52514e', 'Approach': '#2a78d6', 'Extend': '#1baf7a',
               'Orbit': '#eb6834', 'Retract': '#6250d6', 'Home': '#c4a21b'}
TRIAL_COLOR = ['#2a78d6', '#eb6834', '#1baf7a', '#6250d6']
TEXT, TEXT_2, GRID, SURFACE = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float('nan')


def load(d):
    rows = list(csv.DictReader(open(os.path.join(d, 'hydroponic_trial.csv'))))
    ph = json.load(open(os.path.join(d, 'phases.json')))
    z = np.load(os.path.join(d, 'samples.npz'))
    contacts = []
    cp = os.path.join(d, 'contacts.jsonl')
    if os.path.exists(cp):
        contacts = [json.loads(l) for l in open(cp) if l.strip()]
    return dict(name=os.path.basename(d.rstrip('/')), dir=d, rows=rows, phases=ph, s=z, contacts=contacts)


def collisions(t):
    """Contact episodes per model during the mission, assigned to phases."""
    off = t['phases']['wall_minus_mono']
    spans = [(p['name'], p['t0'] + off, p['t1'] + off) for p in t['phases']['phases']]
    if not spans:
        return []
    t_lo, t_hi = spans[0][1], spans[-1][2]
    by = collections.defaultdict(list)
    for c in t['contacts']:
        if t_lo - 1 <= c['t'] <= t_hi + 1:
            parts = c['other'].split('::')
            by['::'.join(parts[:2]) if parts[0] in ('cropking_nft_8_4', 'lettuce', 'bay') else parts[0]].append(c)
    events = []
    for model, cs in by.items():
        cs.sort(key=lambda c: c['t'])
        cur = None
        for c in cs:
            if cur is None or c['t'] - cur['end'] > 1.0:
                phase = next((n for n, a, b in spans if a <= c['t'] <= b), 'between phases')
                cur = dict(model=model, start=c['t'], end=c['t'], phase=phase, links=set())
                events.append(cur)
            cur['end'] = c['t']
            cur['links'].add(c['link'])
    return events


def score(t):
    rows = {r['phase']: r for r in t['rows']}
    ev = collisions(t)
    col = max(0.0, 40.0 - 10.0 * len(ev))
    parts = []
    for p in PHASES:
        r = rows.get(p)
        if r is None:
            parts.append(0.0)
        elif r['success'] == 'True':
            parts.append(1.0)
        elif p in CARTESIAN and r['cartesian_fraction'] not in ('', None):
            parts.append(max(0.0, min(1.0, num(r['cartesian_fraction']))))
        else:
            parts.append(0.0)
    completion = float(np.mean(parts))
    devs = [num(rows[p]['dev_pct']) for p in CARTESIAN if p in rows and rows[p]['success'] == 'True'
            and rows[p]['dev_pct'] not in ('', None)]
    worst = max(devs) if devs else float('nan')
    acc = 0.0 if not devs else (20.0 if worst <= 5 else max(0.0, 20.0 * (10 - worst) / 5))
    return dict(score=round(col + 40 * completion + acc, 1), collisions=len(ev), collision_pts=col,
                completion_pct=round(100 * completion, 1), completion_pts=round(40 * completion, 1),
                worst_dev_pct=None if worst != worst else round(worst, 2), accuracy_pts=round(acc, 1),
                phases_ok=sum(1 for p in PHASES if rows.get(p, {}).get('success') == 'True'),
                events=[dict(model=e['model'], phase=e['phase'], links=sorted(e['links']),
                             duration_s=round(e['end'] - e['start'], 2)) for e in ev])


def style(ax):
    ax.set_facecolor(SURFACE)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=TEXT_2, labelsize=9)
    ax.grid(color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def plots(trials, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size': 10, 'text.color': TEXT, 'axes.labelcolor': TEXT_2,
                         'figure.facecolor': SURFACE, 'savefig.facecolor': SURFACE})

    # execution time per phase, trials side by side; failed phases hatched
    fig, ax = plt.subplots(figsize=(10, 3.8))
    style(ax)
    ax.grid(axis='x', visible=False)
    w = 0.8 / len(trials)
    for k, t in enumerate(trials):
        rows = {r['phase']: r for r in t['rows']}
        xs = np.arange(len(PHASES)) + (k - (len(trials) - 1) / 2) * w
        vals = [num(rows[p]['exec_time_s']) if p in rows else 0 for p in PHASES]
        ok = [rows.get(p, {}).get('success') == 'True' for p in PHASES]
        bars = ax.bar(xs, vals, width=w, color=TRIAL_COLOR[k], edgecolor=SURFACE, linewidth=2, zorder=2,
                      label=f"trial {t['name']}")
        for b, good, v in zip(bars, ok, vals):
            if not good:
                b.set_hatch('///')
                b.set_alpha(0.45)
            ax.text(b.get_x() + b.get_width() / 2, v + 1, f"{v:.0f}" + ('' if good else ' ✗'),
                    ha='center', va='bottom', fontsize=8, color=TEXT_2)
    ax.set_xticks(range(len(PHASES)), PHASES)
    ax.set_ylabel('execution time (s)')
    ax.set_title('Execution time per phase (hatched ✗ = failed or skipped)', loc='left', fontsize=11)
    ax.legend(frameon=False, fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'phase_times.png'), dpi=140, bbox_inches='tight')
    plt.close(fig)

    for t in trials:
        s, ph = t['s'], t['phases']
        tt, phase, ee, v = s['t'], s['phase'], s['ee'], s['v']
        plant = np.array(ph['plant_dog_frame'])
        # 3D camera path
        fig = plt.figure(figsize=(8.5, 6.5))
        ax = fig.add_subplot(111, projection='3d')
        for p in ['Approach', 'Extend', 'Orbit', 'Retract', 'Home']:
            m = phase == p
            if m.any():
                ax.plot(ee[m, 0], ee[m, 1], ee[m, 2], color=PHASE_COLOR[p], linewidth=2.2, label=f'{p} (executed)')
        for rec in ph['phases']:
            ref = rec.get('ideal') or rec.get('planned')
            if rec['name'] in CARTESIAN and ref:
                r = np.array(ref)
                ax.plot(r[:, 0], r[:, 1], r[:, 2], color=TEXT_2, linewidth=1, linestyle='--')
        u, vv = np.mgrid[0:2 * np.pi:24j, 0:np.pi:12j]
        rad = 0.055
        ax.plot_wireframe(plant[0] + rad * np.cos(u) * np.sin(vv), plant[1] + rad * np.sin(u) * np.sin(vv),
                          plant[2] + rad * np.cos(vv), color='#3a9a3a', linewidth=0.5)
        ax.scatter([0], [0], [0.347], color=TEXT, s=25)
        ax.text(0, 0, 0.30, 'D1 base', fontsize=8)
        ax.set_xlabel('x forward (m)')
        ax.set_ylabel('y left (m)')
        ax.set_zlabel('z (m)')
        ax.set_title(f"Trial {t['name']}: camera path in the dog frame (dashed = planned Cartesian path)",
                     loc='left', fontsize=10)
        ax.legend(fontsize=8, loc='upper left', frameon=False)
        ax.view_init(elev=22, azim=-125)
        lim = [min(ee[:, 0].min(), plant[0]) - 0.05, max(ee[:, 0].max(), plant[0]) + 0.08] if len(ee) else [0, 1]
        ax.set_xlim(*lim)
        ax.set_ylim(-0.3, 0.3)
        ax.set_zlim(0.3, 1.1)
        fig.savefig(os.path.join(out, f"trajectory_3d_{t['name']}.png"), dpi=130, bbox_inches='tight')
        plt.close(fig)

        # joint velocities with phase shading
        fig, ax = plt.subplots(figsize=(10, 3.8))
        style(ax)
        t0 = tt[0] if len(tt) else 0
        for rec in ph['phases']:
            if rec['name'] in PHASE_COLOR:
                ax.axvspan(rec['t0'] - t0, rec['t1'] - t0, color=PHASE_COLOR[rec['name']], alpha=0.10, linewidth=0)
                ax.text((rec['t0'] + rec['t1']) / 2 - t0, 1.95, rec['name'], ha='center', fontsize=7, color=TEXT_2)
        for j in range(6):
            ax.plot(tt - t0, v[:, j], linewidth=1.2, label=JOINTS[j])
        ax.axhline(1.05, color=TEXT_2, linewidth=0.8, linestyle=':')
        ax.axhline(-1.05, color=TEXT_2, linewidth=0.8, linestyle=':')
        ax.set_ylim(-2.1, 2.1)
        ax.set_xlabel('mission time (s)')
        ax.set_ylabel('joint velocity (rad/s)')
        ax.set_title(f"Trial {t['name']}: joint velocities (dotted = J1-J3 limit, 1.05 rad/s)", loc='left', fontsize=11)
        ax.legend(frameon=False, ncol=6, fontsize=8, loc='lower right')
        fig.tight_layout()
        fig.savefig(os.path.join(out, f"joint_velocities_{t['name']}.png"), dpi=140, bbox_inches='tight')
        plt.close(fig)

        # deviation from the planned Cartesian path
        fig, ax = plt.subplots(figsize=(10, 3.2))
        style(ax)
        drawn = False
        for rec in ph['phases']:
            if rec['name'] not in CARTESIAN or not (rec.get('ideal') or rec.get('planned')):
                continue
            ref = np.array(rec.get('ideal') or rec.get('planned'))
            m = (tt >= rec['t0']) & (tt <= rec['t1'])
            row = next((r for r in t['rows'] if r['phase'] == rec['name']), {})
            if not m.any() or row.get('success') != 'True':
                continue
            a, b = ref[:-1], ref[1:]
            ab = b - a
            L2 = np.maximum((ab ** 2).sum(1), 1e-12)
            d = []
            for p in ee[m]:
                tt_ = np.clip(((p - a) * ab).sum(1) / L2, 0, 1)
                d.append(np.min(np.linalg.norm(a + ab * tt_[:, None] - p, axis=1)))
            ax.plot(tt[m] - t0, np.array(d) * 1000, color=PHASE_COLOR[rec['name']], linewidth=1.6, label=rec['name'])
            drawn = True
        ax.set_xlabel('mission time (s)')
        ax.set_ylabel('distance to planned path (mm)')
        ax.set_title(f"Trial {t['name']}: camera deviation from the planned Cartesian path", loc='left', fontsize=11)
        if drawn:
            ax.legend(frameon=False, fontsize=8)
        else:
            ax.text(0.5, 0.5, 'no Cartesian phase executed', transform=ax.transAxes, ha='center', color=TEXT_2)
        fig.tight_layout()
        fig.savefig(os.path.join(out, f"deviation_{t['name']}.png"), dpi=140, bbox_inches='tight')
        plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('trials', nargs='+')
    ap.add_argument('--out', default='.')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    trials = [load(d) for d in a.trials]
    scores = {t['name']: score(t) for t in trials}
    json.dump(scores, open(os.path.join(a.out, 'score.json'), 'w'), indent=1)
    md = ['# Hydroponic trial summary (hydroponic_report.py)', '']
    for t in trials:
        sc = scores[t['name']]
        md += [f"## Trial {t['name']}: Mission Score {sc['score']:.1f} / 100", '',
               f"Collisions {sc['collisions']} ({sc['collision_pts']:.0f}/40) · completion {sc['completion_pct']:.1f} % "
               f"({sc['completion_pts']:.1f}/40) · worst Cartesian deviation "
               f"{'n/a' if sc['worst_dev_pct'] is None else str(sc['worst_dev_pct']) + ' %'} ({sc['accuracy_pts']:.1f}/20)", '',
               '| ' + ' | '.join(t['rows'][0].keys()) + ' |', '|' + '---|' * len(t['rows'][0])]
        md += ['| ' + ' | '.join(str(v) for v in r.values()) + ' |' for r in t['rows']]
        md.append('')
    open(os.path.join(a.out, 'summary.md'), 'w').write('\n'.join(md))
    print('\n'.join(md))
    plots(trials, a.out)
    print(f'graphs in {a.out}')


if __name__ == '__main__':
    main()
