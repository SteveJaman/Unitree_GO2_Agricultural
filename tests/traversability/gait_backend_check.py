#!/usr/bin/env python3
"""Node-level checks of gait_supervisor's sport_client backend, without a robot.

  gait_backend_check.py GAIT_SUPERVISOR_PY CASE
    CASE dry       backend sport_client, dry_run true, enable_hardware_backend false
    CASE nosdk     both gates open, no unitree_sdk2py installed
    CASE heartbeat both gates open with a fake SDK (temp dir, records calls): inputs
                   for 6 s, then none for 6 s; expect CAREFUL sent once, then silence

Publishes /lidar3d/body_odom and /local_costmap/costmap (an empty grid) while
inputs are on. Prints the supervisor's [DRY_RUN]/[FATAL]/[WARN] lines, the
backend_state values seen on /gait/status, and (heartbeat) the fake SDK's calls.
Run on its own ROS_DOMAIN_ID.
"""
import json
import os
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

FAKE_SDK = {
    "unitree_sdk2py/__init__.py": "",
    "unitree_sdk2py/core/__init__.py": "",
    "unitree_sdk2py/core/channel.py": "def ChannelFactoryInitialize(*a):\n    pass\n",
    "unitree_sdk2py/go2/__init__.py": "",
    "unitree_sdk2py/go2/sport/__init__.py": "",
    "unitree_sdk2py/go2/sport/sport_client.py": (
        "import os, time\n"
        "class SportClient:\n"
        "    def SetTimeout(self, t): pass\n"
        "    def Init(self): pass\n"
        "    def FootRaiseHeight(self, d):\n"
        "        with open(os.environ['FAKE_SDK_LOG'], 'a') as f:\n"
        "            f.write(f'{time.time():.2f} FootRaiseHeight({d:+.3f})\\n')\n"
        "        return 0\n"),
}


def main():
    script, case = sys.argv[1:3]
    env = dict(os.environ)
    args = ["-p", "backend:=sport_client", "-p", "heartbeat_timeout_s:=2.0"]
    sdk_log = None
    if case == "dry":
        args += ["-p", "dry_run:=true", "-p", "enable_hardware_backend:=false"]
    else:
        args += ["-p", "dry_run:=false", "-p", "enable_hardware_backend:=true"]
    if case == "heartbeat":
        root = tempfile.mkdtemp(prefix="fake_sdk_")
        for rel, text in FAKE_SDK.items():
            os.makedirs(os.path.dirname(os.path.join(root, rel)), exist_ok=True)
            open(os.path.join(root, rel), "w").write(text)
        sdk_log = os.path.join(root, "calls.log")
        env["PYTHONPATH"] = root + os.pathsep + env.get("PYTHONPATH", "")
        env["FAKE_SDK_LOG"] = sdk_log
    log_path = f"/tmp/gait_backend_{case}.log"
    proc = subprocess.Popen(["python3", script, "--ros-args"] + args, env=env, start_new_session=True,
                            stdout=open(log_path, "w"), stderr=subprocess.STDOUT)
    rclpy.init()
    node = Node("gait_backend_check")
    odom_pub = node.create_publisher(Odometry, "/lidar3d/body_odom", 10)
    cm_pub = node.create_publisher(OccupancyGrid, "/local_costmap/costmap", QoSProfile(
        depth=5, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    states = []
    node.create_subscription(String, "/gait/status", lambda m: states.append(
        (time.time(), json.loads(m.data).get("gait"), json.loads(m.data).get("backend_state"))), 10)
    grid = OccupancyGrid()
    grid.header.frame_id = "lidar3d_map"
    grid.info.resolution, grid.info.width, grid.info.height = 0.05, 60, 60
    grid.info.origin.position.x = grid.info.origin.position.y = -1.5
    grid.data = [0] * 3600
    t0 = time.time()
    inputs_on = 6.0 if case == "heartbeat" else 4.0
    try:
        while time.time() - t0 < (12.0 if case == "heartbeat" else 6.0):
            if time.time() - t0 < inputs_on:
                o = Odometry()
                o.header.frame_id = "lidar3d_map"
                o.pose.pose.orientation.w = 1.0
                odom_pub.publish(o)
                cm_pub.publish(grid)
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=10)
    lines = [l.rstrip() for l in open(log_path) if any(k in l for k in ("[DRY_RUN]", "[FATAL]", "[WARN]",
                                                                         "inputs back", "sport_client", "gait ->"))]
    print(f"== {case}: crashed={proc.returncode not in (0, -15, None) and 'Traceback' in open(log_path).read()}")
    for l in lines:
        print("   ", l[l.find("]: ") + 3:] if "]: " in l else l)
    seq = []
    for t, g, st in states:
        tag = (round(t - t0), g, st)
        if not seq or seq[-1][1:] != tag[1:]:
            seq.append(tag)
    print("   status (s since start, gait, backend_state) on change:", seq)
    if sdk_log:
        calls = open(sdk_log).read().split("\n") if os.path.exists(sdk_log) else []
        print("   fake SDK calls:", [c.split(" ", 1)[1] + f" @{float(c.split()[0]) - t0:.1f}s" for c in calls if c])


if __name__ == "__main__":
    main()
