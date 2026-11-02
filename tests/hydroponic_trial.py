#!/usr/bin/env python3
"""
hydroponic_trial.py: one unattended full-mission trial in the Gazebo
hydroponic station (CropKing NFT 8-4): the Go2 walks from the dock to the
rack with Nav2, then the D1-550 (MoveIt 2 through gz_ros2_control, camera
instead of the gripper) approaches, extends to, orbits and retracts from a
lettuce, and folds back. docs/simulation/hydroponic_trial.md

  python3 tests/hydroponic_trial.py --name A                    # the specified 0.6 m waypoint
  python3 tests/hydroponic_trial.py --name B --standoff 0.333   # dog nose at the bench edge
  ... then: python3 src/d1_moveit_config/scripts/hydroponic_report.py TRIAL_DIR...

On the host (Docker, the go2-sim-moveit image from ./gazebo/setup_gazebo.sh
--moveit, ~/files/unitree-D1-550-Robot-ARM, a desktop session):
  1. wifi_nav.sh --sim --no-camera --no-explore with SIM_WORLD=hydroponic_station,
     SIM_SPAWN at the dock, SIM_MOVEIT=1, SIM_CONTACTS=1, SIM_ARM=0;
  2. waits for Nav2 (go2-lidar3d) and MoveIt (go2-sim) to be up;
  3. logs every Gazebo contact of the robot (gazebo/sim/contact_monitor.py);
  4. runs src/d1_moveit_config/scripts/full_mission_test.py in go2-sim;
  5. collects everything in OUT (default ~/files/go2-nav2-wifi/ws/log/hydroponic_trial/NAME),
     copies the phase CSV to ~/files/go2-nav2-wifi/ws/log/hydroponic_trial[_NAME].csv,
     and stops the session.
"""
import argparse
import os
import signal
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAV2_WIFI_DIR = os.environ.get('NAV2_WIFI_DIR', os.path.expanduser('~/files/go2-nav2-wifi'))
SIM = 'go2-sim'
MONITOR = os.path.join(REPO, 'gazebo', 'sim', 'contact_monitor.py')


def log(msg):
    print(f'[trial] {msg}', flush=True)


def run(cmd, timeout=60, **kw):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, **kw)
    except subprocess.TimeoutExpired:
        return subprocess.CompletedProcess(cmd, 124, '', 'timeout')


def wait_for(what, check, timeout, proc):
    t0 = time.time()
    while not check():
        if proc.poll() is not None:
            raise SystemExit(f'session exited while waiting for {what}')
        if time.time() - t0 > timeout:
            raise SystemExit(f'{what} not up after {timeout} s')
        time.sleep(3)
    log(f'{what} up after {time.time() - t0:.0f} s')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--name', default='A')
    ap.add_argument('--standoff', type=float, default=None,
                    help="dog base distance from the stand's front edge (default: the world's 0.6 m waypoint)")
    ap.add_argument('--look-up-deg', type=float, default=30.0)
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    out = os.path.abspath(a.out or os.path.join(NAV2_WIFI_DIR, 'ws', 'log', 'hydroponic_trial', a.name))
    os.makedirs(out, exist_ok=True)
    if run(['docker', 'ps', '-q', '--filter', f'name=^{SIM}$']).stdout.strip():
        sys.exit(f'{SIM} is already running: stop it first (./gazebo/run_sim.sh down)')

    env = dict(os.environ, SIM_WORLD='hydroponic_station', SIM_SPAWN='0 0 0', SIM_MOVEIT='1', SIM_ARM='0',
               SIM_CONTACTS='1', SIM_GUI='0', SIM_TIMEOUT='400', ARM_IFACE=os.environ.get('ARM_IFACE', 'simarm0'))
    session_log = os.path.join(out, 'session.log')
    log(f'trial {a.name}: starting the sim session (log {session_log})')
    proc = subprocess.Popen(['./scripts/runtime/wifi_nav.sh', '--sim', '--host-ip', '127.0.0.1', '--no-camera', '--no-explore'],
                            cwd=REPO, env=env, stdout=open(session_log, 'w'), stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, start_new_session=True,
                            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))
    code = 1
    try:
        # Nav2 is up when both lifecycle managers report their nodes active (session log)
        wait_for('Nav2', lambda: open(session_log, errors='replace').read().count('Managed nodes are active') >= 2,
                 900, proc)
        wait_for('MoveIt', lambda: 'Configured and activated' in run(
            ['docker', 'exec', SIM, 'bash', '-c', 'grep "arm_controller" /tmp/sim-gen/moveit.log']).stdout, 300, proc)
        time.sleep(20)   # costmap settles

        topics = run(['docker', 'exec', SIM, 'cat', '/tmp/sim-gen/contact_topics.txt']).stdout.split()
        run(['docker', 'cp', MONITOR, f'{SIM}:/tmp/contact_monitor.py'])
        run(['docker', 'exec', SIM, 'rm', '-f', '/tmp/contacts.jsonl'])
        run(['docker', 'exec', '-d', SIM, 'bash', '-c', 'GZ_PARTITION=go2sim GZ_IP=127.0.0.1 python3 -u /tmp/contact_monitor.py '
             f'--out /tmp/contacts.jsonl --topics {" ".join(topics)} > /tmp/contact_monitor.log 2>&1'])
        log(f'contact monitor on {len(topics)} links')

        mission_out = f'/tmp/hydroponic_trial_{a.name}'
        args = f'--out {mission_out} --look-up-deg {a.look_up_deg}' + (f' --standoff {a.standoff}' if a.standoff is not None else '')
        log(f'mission: {args}')
        r = run(['docker', 'exec', SIM, 'bash', '-c',
                 'source /opt/ros/humble/setup.bash; source /tmp/moveit_ws/install/setup.bash; '
                 'export CYCLONEDDS_URI=file:///tmp/sim-gen/cyclonedds.xml GZ_PARTITION=go2sim GZ_IP=127.0.0.1; '
                 f'rm -rf {mission_out}; python3 -u /moveit_src/d1_moveit_config/scripts/full_mission_test.py {args}'],
                timeout=1800)
        open(os.path.join(out, 'mission.log'), 'w').write(r.stdout + r.stderr)
        for line in r.stdout.splitlines():
            if line.startswith('[mission]'):
                print(line, flush=True)
        time.sleep(1)
        run(['docker', 'exec', SIM, 'pkill', '-f', 'contact_monitor.py'])
        run(['docker', 'cp', f'{SIM}:{mission_out}/.', out])
        run(['docker', 'cp', f'{SIM}:/tmp/contacts.jsonl', os.path.join(out, 'contacts.jsonl')])
        run(['docker', 'cp', f'{SIM}:/tmp/sim-gen/moveit.log', os.path.join(out, 'moveit.log')])
        csv_src = os.path.join(out, 'hydroponic_trial.csv')
        if os.path.exists(csv_src):
            dst = os.path.join(NAV2_WIFI_DIR, 'ws', 'log', 'hydroponic_trial.csv' if a.name == 'A' else f'hydroponic_trial_{a.name}.csv')
            open(dst, 'w').write(open(csv_src).read())
            log(f'phase CSV: {dst}')
            code = 0
    finally:
        log('stopping the session')
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=120)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        run([os.path.join(REPO, 'gazebo', 'run_sim.sh'), 'down'], timeout=120)
    log(f'trial {a.name} saved to {out}')
    return code


if __name__ == '__main__':
    sys.exit(main())
