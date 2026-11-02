#!/usr/bin/env python3
"""
sim_benchmark.py: automated exploration benchmark in the Gazebo lab world.

  python3 tests/sim_benchmark.py [--drive hybrid|omni|diff] [--sim-seconds 300]
                                 [--world lab_office] [--out DIR]

One unattended run, on the host (needs Docker, the go2-sim image, the
furniture models and a desktop session for the sim's software rendering):

  1. Starts wifi_nav.sh --sim --no-camera with SIM_WORLD=lab_office,
     SIM_GUI=0 (no Gazebo window) and SIM_CONTACTS=1 (contact sensors on every
     robot link: body, legs, head, D1 arm). RViz still opens with the stack.
  2. Presses Auto Explore and lets the frontier explorer run until it
     reports "Exploration complete" or --sim-seconds of *simulated* time
     (Gazebo's /stats clock) have passed.
  3. Meanwhile records every Gazebo contact between the robot and anything
     but the floor (gazebo/sim/contact_monitor.py) and the robot pose and
     costmaps (scripts/tools/nav_compare/record.sh).
  4. Scores the run:
       start at 100
       -10 per collision: contact episodes with one model (one wall), a new episode
           after 1 s without contact
       -5  per room (of the world's rooms, corridor excluded) with less than
           --room-min (90) % of its cells known in the final global costmap
     and prints, before the last line, what caused each deduction, then
       SCORE: X/100

Writes OUT/report.md, OUT/score.json, OUT/contacts.jsonl and the record.sh
trial files (pose.csv, cmd.csv, costmap.npz, frontier.log, session.log ...).
OUT defaults to ~/files/go2-nav2-wifi/ws/log/sim_benchmark/<time>-<drive>.
Exit code 0 when the score is >= --pass (99), else 1.
"""

import argparse
import collections
import json
import os
import re
import signal
import subprocess
import sys
import time

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAV2_WIFI_DIR = os.environ.get('NAV2_WIFI_DIR', os.path.expanduser('~/files/go2-nav2-wifi'))
FLOG = os.path.join(NAV2_WIFI_DIR, 'ws', 'log', 'frontier.log')
RECORD = os.path.join(REPO, 'scripts', 'tools', 'nav_compare', 'record.sh')
MONITOR = os.path.join(REPO, 'gazebo', 'sim', 'contact_monitor.py')
SIM = 'go2-sim'
LIDAR = 'go2-lidar3d'
DDS = 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///tmp/go2-wifi-cyclonedds.xml; ' \
      'source /opt/ros/humble/setup.bash; '


def log(msg):
    print(f'[benchmark] {msg}', flush=True)


def run(cmd, timeout=60, **kw):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, **kw)
    except subprocess.TimeoutExpired:
        return subprocess.CompletedProcess(cmd, 124, '', 'timeout')


def sim_seconds():
    """Gazebo's simulated time (s), or None."""
    r = run(['docker', 'exec', SIM, 'bash', '-c',
             'GZ_PARTITION=go2sim GZ_IP=127.0.0.1 timeout 3 gz topic -e -n 1 -t /stats'], timeout=10)
    m = re.search(r'sim_time\s*\{\s*sec:\s*(\d+)(?:\s*nsec:\s*(\d+))?', r.stdout)
    return int(m.group(1)) + int(m.group(2) or 0) / 1e9 if m else None


def flog_has(text):
    try:
        return text in open(FLOG, errors='replace').read()
    except OSError:
        return False


def resume_explorer():
    run(['docker', 'exec', LIDAR, 'bash', '-c',
         DDS + "ros2 topic pub --once -w 1 /explore/resume std_msgs/msg/Bool '{data: true}'"], timeout=30)


# ------------------------------------------------------------------ session
def start_session(a, log_path):
    env = dict(os.environ, SIM_WORLD=a.world, SIM_CONTACTS='1', SIM_GUI='0',
               ARM_IFACE=os.environ.get('ARM_IFACE', 'simarm0'))
    cmd = ['./scripts/runtime/wifi_nav.sh', '--sim', '--host-ip', '127.0.0.1', '--no-camera', '--drive', a.drive]
    # New session (process group) for a clean Ctrl+C later; SIGINT restored
    # in case this script itself was started with it ignored.
    return subprocess.Popen(cmd, cwd=REPO, env=env, stdout=open(log_path, 'w'), stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, start_new_session=True,
                            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))


def stop_session(proc):
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=120)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
    run([os.path.join(REPO, 'gazebo', 'run_sim.sh'), 'down'], timeout=120)


# ------------------------------------------------------------------ scoring
def short(collision):
    """go2::base::base_fixed_joint_lump__FL_calf_collision_3 -> FL_calf"""
    name = collision.split('::')[-1]
    name = re.sub(r'^base_fixed_joint_lump__', '', name)
    return re.sub(r'_collision(_\d+)?$', '', name)


def collision_events(contacts, gap=1.0):
    by_model = collections.defaultdict(list)
    for c in contacts:
        parts = c['other'].split('::')
        # every wall piece is a link of one "walls" model: count each wall on its own
        by_model['::'.join(parts[:2]) if parts[0] == 'walls' else parts[0]].append(c)
    events = []
    for model, cs in by_model.items():
        cs.sort(key=lambda c: c['sim_t'])
        cur = None
        for c in cs:
            if cur is None or c['sim_t'] - cur['end'] > gap:
                cur = dict(model=model, start=c['sim_t'], end=c['sim_t'], parts=set(), x=c['x'], y=c['y'],
                           max_depth=c['depth'] or 0.0)
                events.append(cur)
            cur['end'] = c['sim_t']
            cur['parts'].add(short(c['robot']))
            cur['max_depth'] = max(cur['max_depth'], c['depth'] or 0.0)
    return sorted(events, key=lambda e: e['start'])


def room_coverage(costmap, world):
    z = np.load(costmap)
    data, r, (ox, oy) = z['data'], float(z['resolution']), z['origin']
    ys, xs = np.mgrid[0:data.shape[0], 0:data.shape[1]]
    X, Y = ox + (xs + 0.5) * r, oy + (ys + 0.5) * r
    known = data >= 0
    out = {}
    for room, (x0, x1, y0, y1) in world['rooms'].items():
        m = (X > x0) & (X < x1) & (Y > y0) & (Y < y1)
        out[room] = 100 * float(known[m].mean()) if m.any() else 0.0
    b = world['bounds']
    inside = (X > b[0]) & (X < b[1]) & (Y > b[2]) & (Y < b[3])
    return out, 100 * float(known[inside].mean())


def top_warnings(session_log, n=6):
    counts = collections.Counter()
    for line in open(session_log, errors='replace'):
        if re.search(r'\[(WARN|ERROR)\]', line) and 'Failed to set parameters atomically' not in line:
            key = re.sub(r'\[\d{9,}\.\d+\]', '', line.strip())
            counts[re.sub(r'-?\d+\.\d+', '#', key)[:160]] += 1
    return counts.most_common(n)


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--drive', default='hybrid', choices=('diff', 'hybrid', 'omni'))
    ap.add_argument('--world', default='lab_office')
    ap.add_argument('--sim-seconds', type=float, default=300.0, help='simulated time limit for exploring')
    ap.add_argument('--room-min', type=float, default=90.0, help='%% of a room known to count as explored')
    ap.add_argument('--pass', dest='pass_score', type=float, default=99.0)
    ap.add_argument('--out', help='output directory')
    a = ap.parse_args()

    world_json = os.path.join(REPO, 'gazebo', 'worlds', f'{a.world}.json')
    if not os.path.exists(world_json):
        sys.exit(f'no {world_json}: the benchmark needs the world JSON (rooms, bounds)')
    world = json.load(open(world_json))
    if not os.path.isdir(os.path.join(REPO, 'gazebo', 'models', 'furniture')):
        sys.exit('furniture models missing: ./gazebo/setup_gazebo.sh --furniture')
    if run(['docker', 'ps', '-q', '--filter', f'name=^{SIM}$']).stdout.strip():
        sys.exit(f'{SIM} is already running: stop it first (./gazebo/run_sim.sh down), '
                 'the benchmark needs a sim started with SIM_CONTACTS=1')
    out = os.path.abspath(a.out or os.path.join(NAV2_WIFI_DIR, 'ws', 'log', 'sim_benchmark',
                                                f"{time.strftime('%Y%m%d-%H%M%S')}-{a.drive}"))
    os.makedirs(out, exist_ok=True)
    name = os.path.basename(out)
    rec_env = dict(os.environ, NAV_COMPARE_DIR=os.path.dirname(out))
    session_log = os.path.join(out, 'session.log')

    try:
        os.remove(FLOG)
    except OSError:
        pass
    log(f'{a.world}, --drive {a.drive}, up to {a.sim_seconds:.0f} simulated s; output {out}')
    proc = start_session(a, session_log)
    result = {}
    try:
        t0 = time.time()
        while not flog_has('frontier_explorer ready'):
            if proc.poll() is not None:
                raise SystemExit(f'session exited early; see {session_log}')
            if time.time() - t0 > 900:
                raise SystemExit(f'explorer not ready after 15 min; see {session_log}')
            time.sleep(2)
        links = run(['docker', 'exec', SIM, 'cat', '/tmp/sim-gen/contact_topics.txt']).stdout.split()
        if not links:
            raise SystemExit('the sim has no contact sensors (SIM_CONTACTS=1 not applied?)')
        # A score of 100 must mean "no contact", not "no sensor": every sensor needs a publisher.
        silent = [t for t in links if 'Publishers' not in run(
            ['docker', 'exec', SIM, 'bash', '-c', f'GZ_PARTITION=go2sim GZ_IP=127.0.0.1 gz topic -i -t {t}'],
            timeout=15).stdout]
        if silent:
            raise SystemExit(f'contact sensors not publishing: {silent}')
        log(f'explorer ready; contact sensors on {len(links)} links; letting the costmap settle (30 s)')
        time.sleep(30)

        run(['docker', 'cp', MONITOR, f'{SIM}:/tmp/contact_monitor.py'])
        run(['docker', 'exec', SIM, 'rm', '-f', '/tmp/contacts.jsonl'])
        run(['docker', 'exec', '-d', SIM, 'bash', '-c',
             'GZ_PARTITION=go2sim GZ_IP=127.0.0.1 python3 -u /tmp/contact_monitor.py '
             f'--out /tmp/contacts.jsonl --topics {" ".join(links)} > /tmp/contact_monitor.log 2>&1'])
        r = run([RECORD, 'start', name, '--drive', a.drive, '--note', f'sim_benchmark {a.world}'], env=rec_env)
        if r.returncode:
            raise SystemExit(f'record.sh start failed: {r.stderr.strip()}')

        resume_explorer()
        s0 = None
        while s0 is None:
            s0 = sim_seconds()
        w0 = time.time()
        done, elapsed = False, 0.0
        while True:
            time.sleep(5)
            s = sim_seconds()
            if s is not None:          # keep the last reading if /stats did not answer
                elapsed = s - s0
            if flog_has('Exploration complete'):
                done = True
                break
            if elapsed >= a.sim_seconds:
                break
            if not flog_has('Exploration ENABLED'):
                resume_explorer()
            if proc.poll() is not None:
                raise SystemExit(f'session exited during the run; see {session_log}')
        result.update(sim_s=round(elapsed, 1), wall_s=round(time.time() - w0, 1), complete=done)
        log(f"exploration {'complete' if done else 'stopped at the limit'} after "
            f"{elapsed:.0f} simulated s ({time.time() - w0:.0f} wall s)")
        run([RECORD, 'snapshot', name], env=rec_env)
        run([RECORD, 'stop', name], env=rec_env, timeout=120)
        run(['docker', 'exec', SIM, 'pkill', '-f', 'contact_monitor.py'])
        time.sleep(1)
        run(['docker', 'cp', f'{SIM}:/tmp/contacts.jsonl', os.path.join(out, 'contacts.jsonl')])
    finally:
        log('stopping the session')
        stop_session(proc)

    # ---------------------------------------------------------------- score
    contacts = []
    cpath = os.path.join(out, 'contacts.jsonl')
    if os.path.exists(cpath):
        contacts = [json.loads(l) for l in open(cpath) if l.strip()]
    events = collision_events(contacts)
    costmap = next((os.path.join(out, f) for f in ('costmap_explore.npz', 'costmap.npz')
                    if os.path.exists(os.path.join(out, f))), None)
    rooms, floor = room_coverage(costmap, world) if costmap else ({}, 0.0)
    unexplored = {k: v for k, v in rooms.items() if k != 'corridor' and v < a.room_min}
    if not costmap:
        unexplored = {k: 0.0 for k in world['rooms'] if k != 'corridor'}

    score = max(0, 100 - 10 * len(events) - 5 * len(unexplored))
    lines = [f'# Sim benchmark: {a.world}, --drive {a.drive}', '',
             f"Exploration {'complete' if result.get('complete') else 'NOT complete'} after "
             f"{result.get('sim_s', 0):.0f} simulated s; floor {floor:.0f} % known.", '',
             '## Deductions', '']
    for e in events:
        lines.append(f"- -10: collision with **{e['model']}** at sim t {e['start']:.1f} s for "
                     f"{e['end'] - e['start']:.1f} s ({', '.join(sorted(e['parts']))}; "
                     f"contact at x {e['x']}, y {e['y']}; depth up to {e['max_depth'] * 100:.1f} cm)")
    for k, v in unexplored.items():
        lines.append(f'- -5: room **{k}** only {v:.0f} % mapped (needs {a.room_min:.0f} %)')
    if not events and not unexplored:
        lines.append('- none')
    lines += ['', '## Rooms mapped (% of cells known)', '', '| Room | % |', '|---|---|']
    lines += [f'| {k} | {v:.0f} |' for k, v in rooms.items()]
    warn = top_warnings(session_log)
    if warn:
        lines += ['', '## Most frequent warnings and errors in the session log', '']
        lines += [f'- {n} x `{w}`' for w, n in warn]
    lines += ['', f'SCORE: {score:.0f}/100']
    open(os.path.join(out, 'report.md'), 'w').write('\n'.join(lines) + '\n')
    json.dump(dict(score=score, drive=a.drive, world=a.world, **result, floor_pct=round(floor, 1),
                   rooms=rooms, collisions=[{**e, 'parts': sorted(e['parts'])} for e in events],
                   unexplored=unexplored), open(os.path.join(out, 'score.json'), 'w'), indent=1)
    print('\n'.join(lines[2:]))
    print(f'[benchmark] report: {out}/report.md')
    print(f'SCORE: {score:.0f}/100')
    sys.exit(0 if score >= a.pass_score else 1)


if __name__ == '__main__':
    main()
