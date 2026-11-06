#!/usr/bin/env python3
"""Drive the sim robot through waypoints over TCP :17999, for leg videos: no Nav2.

  drive_line.py x,y x,y ... [--speed 0.3]

Pose comes from gz /model/go2/odometry through `gz topic`, read in the go2-sim
container. Steers with a simple heading controller, and stops at the last point.
"""
import argparse
import json
import math
import socket
import struct
import subprocess
import time

VEL = struct.Struct("!fff")
POSE_CMD = ("export GZ_PARTITION=go2sim GZ_IP=127.0.0.1; "
            "gz topic -e -n 1 --json-output -t /model/go2/odometry")


def pose():
    out = subprocess.run(["docker", "exec", "go2-sim", "bash", "-c", POSE_CMD],
                         capture_output=True, text=True, timeout=10).stdout
    p = json.loads(out.strip().splitlines()[0])["pose"]
    q = p["orientation"]
    z, w = q.get("z", 0.0), q.get("w", 1.0)
    return p["position"].get("x", 0.0), p["position"].get("y", 0.0), 2 * math.atan2(z, w)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("points", nargs="+")
    ap.add_argument("--speed", type=float, default=0.3)
    a = ap.parse_args()
    pts = [tuple(float(v) for v in s.split(",")) for s in a.points]
    sock = socket.create_connection(("127.0.0.1", 17999), timeout=5)
    try:
        for gx, gy in pts:
            while True:
                x, y, yaw = pose()
                d = math.hypot(gx - x, gy - y)
                if d < 0.08:
                    break
                err = math.atan2(math.sin(math.atan2(gy - y, gx - x) - yaw),
                                 math.cos(math.atan2(gy - y, gx - x) - yaw))
                vx = 0.0 if abs(err) > 0.6 else a.speed
                wz = max(-0.7, min(0.7, 1.5 * err))
                end = time.time() + 0.25
                while time.time() < end:
                    sock.sendall(VEL.pack(vx, 0.0, wz))
                    time.sleep(0.05)
                print(f"{x:.2f} {y:.2f} yaw {yaw:.2f} -> {gx},{gy}", flush=True)
    finally:
        for _ in range(5):
            sock.sendall(VEL.pack(0.0, 0.0, 0.0))
            time.sleep(0.05)
        sock.close()


if __name__ == "__main__":
    main()
