#!/usr/bin/env python3
"""
orbit_trial.py: the 180 degree orbit sweep in Gazebo, on the D1 arm driven by
MoveIt through gz_ros2_control, measured by the simulator.

  python3 tests/orbit_trial.py                         # low plant beside the dog
  python3 tests/orbit_trial.py --target 0 0.40 0.85 --radius 0.15 --elevation -15   # rack height

On the host (Docker, go2-sim-moveit from ./gazebo/setup_gazebo.sh --moveit,
~/files/unitree-D1-550-Robot-ARM, a desktop session):
  1. ./gazebo/run_sim.sh up in the hydroponic world, SIM_MOVEIT=1, SIM_CONTACTS=1;
  2. spawns a stem and canopy at the target, the same shape orbit_scanner.py
     adds to the planning scene, so a touch is a real Gazebo contact;
  3. logs every contact of the robot (gazebo/sim/contact_monitor.py);
  4. runs orbit_scanner.py, recording /joint_states, and saves the wrist
     camera's frame at each capture point;
  5. reports waypoints reached, contacts, and the camera's distance, aim and
     angle around the plant from the recorded joints.

Writes to OUT (default ~/files/go2-nav2-wifi/ws/log/orbit_trial/NAME): scanner.log,
contacts.jsonl, joints.csv, frames/*.png, summary.json. Exit 0 only with every
waypoint reached and no contact.
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM = "go2-sim"
RUN_SIM = os.path.join(REPO, "gazebo", "run_sim.sh")
MONITOR = os.path.join(REPO, "gazebo", "sim", "contact_monitor.py")
WORLD = "hydroponic_station"
D1_URDF = os.path.expanduser(
    os.path.join(
        os.environ.get("D1_ARM_DIR", "~/files/unitree-D1-550-Robot-ARM"),
        "d1_550_description/urdf/d1_550_description.urdf",
    )
)
SETUP = (
    "source /opt/ros/humble/setup.bash; source /tmp/moveit_ws/install/setup.bash; "
    "export ROS_DOMAIN_ID=66 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp "
    "CYCLONEDDS_URI=file:///tmp/sim-gen/cyclonedds.xml GZ_PARTITION=go2sim GZ_IP=127.0.0.1; "
)

# Runs inside the container next to Gazebo: records the joints, runs the
# scanner, and saves the wrist camera's next frame at each capture message.
INSIDE = r"""
import os, re, subprocess, sys, threading, time
import numpy as np, rclpy
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node
from sensor_msgs.msg import JointState
out, scanner_args = sys.argv[1], sys.argv[2:]
JOINTS = [f"Joint{i}" for i in range(1, 7)]
rclpy.init()
ros = rclpy.create_node("orbit_trial_recorder")
rows = open(out + "/joints.csv", "w")
def on_joints(m):
    d = dict(zip(m.name, m.position))
    if all(j in d for j in JOINTS):
        rows.write(",".join(f"{d[j]:.5f}" for j in JOINTS) + "\n"); rows.flush()
ros.create_subscription(JointState, "/joint_states", on_joints, 50)
threading.Thread(target=rclpy.spin, args=(ros,), daemon=True).start()
gz = Node()
latest = {}
gz.subscribe(Image, "/wrist_camera/image", lambda m: latest.__setitem__("m", m))
def grab(path, timeout=1.5):
    # A frame from after the stop: the sensor runs at 5 Hz.
    latest.pop("m", None)
    time.sleep(timeout)
    if "m" not in latest:
        print("[trial] no wrist camera frame", flush=True); return
    m = latest["m"]
    img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.imsave(path, img)
p = subprocess.Popen(["python3", "-u", "/moveit_src/d1_active_perception/d1_active_perception/orbit_scanner.py",
                      "--ros-args"] + scanner_args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
for line in p.stdout:
    print(line, end="", flush=True)
    w = re.search(r"AT WAYPOINT (\d+)", line)
    if w:
        grab(f"{out}/frames/wp{int(w.group(1)):02d}.png")
code = p.wait()
sys.stdout.flush()
os._exit(code)
"""


def log(msg):
    print(f"[trial] {msg}", flush=True)


def run(cmd, timeout=60, **kw):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, **kw)
    except subprocess.TimeoutExpired:
        return subprocess.CompletedProcess(cmd, 124, "", "timeout")


def in_sim(script, timeout=60):
    return run(["docker", "exec", SIM, "bash", "-c", script], timeout=timeout)


def plant_sdf(x, y, z, canopy):
    """A static stem and canopy, the shape orbit_scanner.py adds to the planning scene."""
    material = "<material><diffuse>0.2 0.6 0.2 1</diffuse><ambient>0.2 0.6 0.2 1</ambient></material>"
    return (
        f"<sdf version='1.9'><model name='orbit_plant'><static>true</static><pose>{x} {y} 0 0 0 0</pose>"
        f"<link name='plant'>"
        f"<collision name='stem'><pose>0 0 {z / 2} 0 0 0</pose><geometry><cylinder><radius>0.02</radius>"
        f"<length>{z}</length></cylinder></geometry></collision>"
        f"<visual name='stem'><pose>0 0 {z / 2} 0 0 0</pose><geometry><cylinder><radius>0.02</radius>"
        f"<length>{z}</length></cylinder></geometry>{material}</visual>"
        f"<collision name='canopy'><pose>0 0 {z} 0 0 0</pose><geometry><sphere><radius>{canopy}</radius>"
        f"</sphere></geometry></collision>"
        f"<visual name='canopy'><pose>0 0 {z} 0 0 0</pose><geometry><sphere><radius>{canopy}</radius>"
        f"</sphere></geometry>{material}</visual>"
        f"</link></model></sdf>"
    )


def spawn_plant(target, dog, canopy, out):
    """True once Gazebo lists the plant: the create service answers true even for
    an SDF it then rejects."""
    path = os.path.join(out, "plant.sdf")
    open(path, "w").write(plant_sdf(dog[0] + target[0], dog[1] + target[1], target[2], canopy))
    run(["docker", "cp", path, f"{SIM}:/tmp/plant.sdf"])
    gz = "export GZ_PARTITION=go2sim GZ_IP=127.0.0.1; "
    in_sim(
        gz + f"gz service -s /world/{WORLD}/create --reqtype gz.msgs.EntityFactory "
        "--reptype gz.msgs.Boolean --timeout 10000 --req 'sdf_filename: \"/tmp/plant.sdf\"'"
    )
    t0 = time.time()
    while time.time() - t0 < 15:
        if "orbit_plant" in in_sim(gz + "gz model --list 2>/dev/null").stdout:
            return True
        time.sleep(1)
    return False


def rotation(axis, angle):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def arm_chain():
    root, chain = ET.parse(D1_URDF).getroot(), []
    for name in [f"Joint{i}" for i in range(1, 7)]:
        j = next(x for x in root.findall("joint") if x.get("name") == name)
        chain.append(
            (
                np.array(j.find("origin").get("xyz").split(), float),
                np.array(j.find("axis").get("xyz").split(), float),
            )
        )
    return chain


def camera_pose(chain, joints):
    """camera_link position and view axis in MoveIt's world (ground under the dog),
    mount yaw 180 degrees, arm base 0.347 m up (d1_go2.urdf.xacro)."""
    R, p = np.eye(3), np.zeros(3)
    for (origin, axis), q in zip(chain, joints):
        p = p + R @ origin
        R = R @ rotation(axis, q)
    p = p + R @ np.array([0.06, -0.0004, 0.0031])
    x = R[:, 0]
    return np.array([-p[0], -p[1], p[2] + 0.347]), np.array([-x[0], -x[1], x[2]])


def stops(path):
    """The joint poses the arm rested at, in order."""
    q = np.loadtxt(path, delimiter=",")
    moving = np.abs(np.diff(q, axis=0)).max(axis=1) > 2e-4
    rests, i = [], 0
    while i < len(moving):
        if moving[i]:
            i += 1
            continue
        k = i
        while k < len(moving) and not moving[k]:
            k += 1
        if k - i >= 10 and (not rests or np.abs(q[k] - rests[-1]).max() > 0.01):
            rests.append(q[k])
        i = k
    return rests


def measure(path, target):
    target = np.asarray(target, float)
    facing = math.atan2(-target[1], -target[0])
    chain, rows = arm_chain(), []
    for q in stops(path):
        p, view = camera_pose(chain, q)
        v = target - p
        d = float(np.linalg.norm(v))
        az = math.atan2(-v[1], -v[0]) - facing
        rows.append(
            {
                "distance": round(d, 3),
                "aim_deg": round(math.degrees(math.acos(np.clip(view @ (v / d), -1, 1))), 1),
                "azimuth_deg": round(math.degrees((az + math.pi) % (2 * math.pi) - math.pi), 1),
                "elevation_deg": round(math.degrees(math.asin(-v[2] / d)), 1),
            }
        )
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", default="low_plant")
    ap.add_argument("--target", type=float, nargs=3, default=[0.0, 0.40, 0.15])
    ap.add_argument("--radius", type=float, default=0.25)
    ap.add_argument("--elevation", type=float, default=60.0)
    ap.add_argument("--plant-radius", type=float, default=0.08)
    ap.add_argument(
        "--gazebo-plant-radius",
        type=float,
        default=None,
        help="canopy in Gazebo, default --plant-radius. Larger than the planned one = a crash test: "
        "--plant-radius 0 --gazebo-plant-radius 0.30 must log contacts",
    )
    ap.add_argument("--spawn", type=float, nargs=3, default=[0.5, -1.0, 0.0], help="dog x y yaw; away from the dock and rack")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    out = os.path.abspath(a.out or os.path.expanduser(f"~/files/go2-nav2-wifi/ws/log/orbit_trial/{a.name}"))
    os.makedirs(out, exist_ok=True)
    if run(["docker", "ps", "-q", "--filter", f"name=^{SIM}$"]).stdout.strip():
        sys.exit(f"{SIM} is already running: stop it first (./gazebo/run_sim.sh down)")

    env = dict(
        os.environ,
        SIM_WORLD=WORLD,
        SIM_SPAWN=" ".join(map(str, a.spawn)),
        SIM_MOVEIT="1",
        SIM_ARM="0",
        SIM_CONTACTS="1",
        SIM_GUI="0",
        SIM_TIMEOUT="400",
        GO2_HOST_IP="127.0.0.1",
    )
    code, summary = 1, {}
    try:
        log("starting Gazebo with MoveIt")
        up = run([RUN_SIM, "up"], timeout=500, env=env)
        open(os.path.join(out, "sim_up.log"), "w").write(up.stdout + up.stderr)
        if up.returncode != 0:
            sys.exit(f"the simulator did not start: see {out}/sim_up.log")
        t0 = time.time()
        while "Configured and activated" not in in_sim('grep "arm_controller" /tmp/sim-gen/moveit.log').stdout:
            if time.time() - t0 > 300:
                sys.exit("MoveIt did not come up")
            time.sleep(3)
        log(f"MoveIt up after {time.time() - t0:.0f} s")

        gazebo_radius = a.plant_radius if a.gazebo_plant_radius is None else a.gazebo_plant_radius
        if gazebo_radius > 0 and not spawn_plant(a.target, a.spawn, gazebo_radius, out):
            sys.exit("could not spawn the plant in Gazebo")
        topics = in_sim("cat /tmp/sim-gen/contact_topics.txt").stdout.split()
        run(["docker", "cp", MONITOR, f"{SIM}:/tmp/contact_monitor.py"])
        in_sim("rm -f /tmp/contacts.jsonl; rm -rf /tmp/orbit; mkdir -p /tmp/orbit/frames")
        in_sim(
            "GZ_PARTITION=go2sim GZ_IP=127.0.0.1 nohup python3 -u /tmp/contact_monitor.py "
            f"--out /tmp/contacts.jsonl --topics {' '.join(topics)} > /tmp/contact_monitor.log 2>&1 &"
        )
        log(f"contact monitor on {len(topics)} links")
        time.sleep(3)

        inside = os.path.join(out, "inside.py")
        open(inside, "w").write(INSIDE)
        run(["docker", "cp", inside, f"{SIM}:/tmp/orbit_inside.py"])
        target = ",".join(str(v) for v in a.target)
        args = [
            f"-p target:=[{target}]",
            f"-p radius:={float(a.radius)}",
            f"-p elevation_deg:={float(a.elevation)}",
            f"-p plant_radius:={float(a.plant_radius)}",
            "-p planning_time:=2.0",
            "-p dwell_s:=2.0",
            "-p use_sim_time:=false",
        ]
        log("running the sweep")
        t0 = time.time()
        r = in_sim(
            SETUP + "python3 -u /tmp/orbit_inside.py /tmp/orbit " + " ".join(args) + " 2>&1", timeout=1500
        )
        took = time.time() - t0
        open(os.path.join(out, "scanner.log"), "w").write(r.stdout + r.stderr)
        time.sleep(2)
        in_sim("pkill -f contact_monitor.py")
        run(["docker", "cp", f"{SIM}:/tmp/orbit/.", out])
        run(["docker", "cp", f"{SIM}:/tmp/contacts.jsonl", os.path.join(out, "contacts.jsonl")])

        contacts = []
        path = os.path.join(out, "contacts.jsonl")
        if os.path.exists(path):
            contacts = [json.loads(line) for line in open(path) if line.strip()]
        pairs = sorted({(c["link"], c["other"].split("::")[0]) for c in contacts})
        reached = r.stdout.count("CAMERA CAPTURE TRIGGERED")
        poses = measure(os.path.join(out, "joints.csv"), a.target)
        n = a.__dict__.get("waypoints", 13)
        ends = poses[1:] if len(poses) > n else poses
        summary = dict(
            reached=reached,
            contacts=len(contacts),
            contact_pairs=pairs,
            minutes=round(took / 60, 1),
            frames=len(os.listdir(os.path.join(out, "frames"))),
            poses=ends,
        )
        json.dump(summary, open(os.path.join(out, "summary.json"), "w"), indent=1)
        log(f"{reached}/13 waypoints, {len(contacts)} contacts {pairs}, {took / 60:.1f} min, {summary['frames']} frames")
        for i, p in enumerate(ends, 1):
            log(f"stop {i:2d}: {p}")
        code = 0 if reached == 13 and not contacts else 1
    finally:
        log("stopping the simulator")
        run([RUN_SIM, "down"], timeout=120, env=env)
    log(f"saved to {out}")
    return code


if __name__ == "__main__":
    sys.exit(main())
