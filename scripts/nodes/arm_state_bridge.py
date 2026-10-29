#!/usr/bin/env python3
"""
arm_state_bridge.py

The D1 arm's live joint angles for the web dashboard, and its Orbit Scan button.

The arm speaks JSON on raw DDS (rt/arm_Feedback), not ROS JointState. This
runs on the host and turns its telemetry (angle0..angle6, degrees) into
sensor_msgs/JointState on /d1_arm/joint_states, published through rosbridge
(ws://localhost:9090), so the dashboard can animate the 3D arm.

Arm source, like start_arm.sh:
    wired   DDS on the arm's Ethernet (192.168.123.100), the shared D1Arm
            (pip cyclonedds); --iface auto-detects the 192.168.123.x NIC
    --robot the arm plugged into the Go2: the read-only monitor port of
            arm_bridge_wifi.py (ARM_BRIDGE_PORT + 1), which never blocks
            start_arm.sh's own connection

It also listens on /d1_arm/command (std_msgs/String, JSON):
    {"op": "orbit"}   run ./scripts/runtime/start_arm.sh orbit --yes
    {"op": "stop"}    Ctrl+C that run: orbit_scan.py homes, then releases
and reports on /d1_arm/status (std_msgs/String, JSON) every second:
    {"arm": true|false, "orbit": "idle"|"running"|"done"|"failed"|"stopped"}
--no-commands turns the button off (status only).

Joint names are the D1 URDF's (Joint1..Joint6, fingers Joint7_1/Joint7_2):
protocol angle i -> Joint(i+1) in radians, as in gazebo/sim/d1_arm_emulator.py.

ROS stays out of this process on purpose: the pip cyclonedds the wired link
needs clashes with ROS's own libddsc. rosbridge is reached with a minimal
standard-library WebSocket client.

Usage:
    python3 arm_state_bridge.py                       # wired arm
    python3 arm_state_bridge.py --robot 192.168.1.4   # arm on the Go2
(wifi_3d_nav.sh starts it with the session.)
"""

import argparse
import base64
import json
import math
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
START_ARM = os.path.join(REPO, "scripts", "runtime", "start_arm.sh")

JOINTS = [
    "Joint1",
    "Joint2",
    "Joint3",
    "Joint4",
    "Joint5",
    "Joint6",
    "Joint7_1",
    "Joint7_2",
]
GRIPPER_OPEN_DEG, FINGER_TRAVEL_M = 65.0, 0.03  # as gazebo/sim/build_model.py
ADDR_TELEM = 2


def joint_positions(angles):
    """Seven protocol angles (deg; 6 = gripper) -> URDF joint positions."""
    pos = [math.radians(a) for a in angles[:6]]
    travel = (
        max(0.0, min(1.0, angles[6] / GRIPPER_OPEN_DEG)) * FINGER_TRAVEL_M
        if len(angles) > 6
        else 0.0
    )
    return pos + [travel, -travel]


def angles_from_feedback(line):
    """angle0..angle6 from one rt/arm_Feedback JSON line, or None."""
    try:
        msg = json.loads(line)
    except ValueError:
        return None
    if (
        not isinstance(msg, dict)
        or msg.get("address") != ADDR_TELEM
        or msg.get("funcode") != 1
    ):
        return None
    d = msg.get("data") or {}
    vals = [d.get(f"angle{i}") for i in range(7)]
    if any(v is None for v in vals[:6]):
        return None
    return [float(v) if v is not None else 0.0 for v in vals]


# ---------------------------------------------------------------- rosbridge
class Rosbridge:
    """Just enough RFC 6455 for rosbridge: text frames out (masked), in."""

    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=5)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall(
            (
                f"GET / HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\n"
                f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
                "Sec-WebSocket-Version: 13\r\n\r\n"
            ).encode()
        )
        head = b""
        while b"\r\n\r\n" not in head:
            chunk = self.sock.recv(1024)
            if not chunk:
                raise ConnectionError("rosbridge closed during the handshake")
            head += chunk
        status = head.split(b"\r\n", 1)[0]
        if b" 101 " not in status:
            raise ConnectionError(f"rosbridge refused: {status!r}")
        self.buf = head.split(b"\r\n\r\n", 1)[1]
        self.sock.settimeout(None)
        self.wlock = threading.Lock()

    def _frame(self, opcode, payload):
        n = len(payload)
        head = bytes([0x80 | opcode])
        if n < 126:
            head += bytes([0x80 | n])
        elif n < 65536:
            head += bytes([0x80 | 126]) + struct.pack("!H", n)
        else:
            head += bytes([0x80 | 127]) + struct.pack("!Q", n)
        mask = os.urandom(4)
        body = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        with self.wlock:
            self.sock.sendall(head + mask + body)

    def send(self, obj):
        self._frame(0x1, json.dumps(obj, separators=(",", ":")).encode())

    def _read(self, n):
        while len(self.buf) < n:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("rosbridge closed")
            self.buf += chunk
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def recv(self):
        """Next text message (dict); answers pings, raises on close."""
        while True:
            b0, b1 = self._read(2)
            n = b1 & 0x7F
            if n == 126:
                n = struct.unpack("!H", self._read(2))[0]
            elif n == 127:
                n = struct.unpack("!Q", self._read(8))[0]
            mask = self._read(4) if b1 & 0x80 else None
            data = self._read(n)
            if mask:
                data = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
            op = b0 & 0x0F
            if op == 0x8:
                raise ConnectionError("rosbridge closed the WebSocket")
            if op == 0x9:
                self._frame(0xA, data)
            elif op == 0x1:
                try:
                    return json.loads(data)
                except ValueError:
                    continue

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


# ---------------------------------------------------------------- arm source
class ArmFeed:
    """Latest arm angles, kept fresh by a background thread (reconnects)."""

    def __init__(self, args):
        self.args = args
        self.angles = None
        self.stamp = 0.0
        threading.Thread(target=self._run, daemon=True).start()

    def alive(self):
        return self.angles is not None and time.time() - self.stamp < 2.0

    def _set(self, a):
        self.angles, self.stamp = a, time.time()

    def _run(self):
        while True:
            try:
                (self._monitor if self.args.robot else self._dds)()
            except Exception as exc:  # keep retrying: the arm comes and goes
                print(f"[arm_state] arm feed: {exc}; retrying in 3 s", flush=True)
            time.sleep(3.0)

    def _monitor(self):
        port = self.args.monitor_port
        with socket.create_connection((self.args.robot, port), timeout=5) as s:
            print(
                f"[arm_state] arm via the Go2 bridge monitor {self.args.robot}:{port}",
                flush=True,
            )
            s.settimeout(5.0)  # telemetry is ~10 Hz: silence means a dead link
            buf = b""
            while True:
                chunk = s.recv(65536)
                if not chunk:
                    raise ConnectionError("monitor closed")
                *lines, buf = (buf + chunk).split(b"\n")
                for raw in lines:
                    a = angles_from_feedback(raw.decode(errors="replace"))
                    if a:
                        self._set(a)

    def _dds(self):
        sys.path.insert(0, HERE)
        from d1_arm_control import (
            D1Arm,
        )  # pip cyclonedds, imported only for the wired link

        arm = D1Arm(self.args.iface, self.args.domain)
        if not arm.ok:
            raise ConnectionError(f"no DDS match on {self.args.iface}")
        print(f"[arm_state] arm via DDS on {self.args.iface}", flush=True)
        misses = 0
        while misses < 20:
            d = (arm.angles(timeout=0.5) or {}).get("data", {})
            vals = [d.get(f"angle{i}") for i in range(7)]
            if all(v is not None for v in vals[:6]):
                self._set([float(v) if v is not None else 0.0 for v in vals])
                misses = 0
            else:
                misses += 1
            time.sleep(0.1)
        raise ConnectionError("telemetry stopped")


# ---------------------------------------------------------------- orbit runs
class Orbit:
    """The one arm job at a time the dashboard / mission can start: orbit or IK scan."""

    def __init__(self, args):
        self.args = args
        self.proc = None
        self.state = "idle"
        self.job = None

    def poll(self):
        if self.proc and self.proc.poll() is not None:
            code = self.proc.returncode
            self.state = (
                "done" if code == 0 else "stopped" if code in (130, -2) else "failed"
            )
            print(
                f"[arm_state] {self.job} ended ({self.state}, exit {code})", flush=True
            )
            self.proc = None
        return self.state

    def stop(self, wait=30.0):
        """Ctrl+C a running job (the scan script homes, then releases) and wait for it."""
        if self.proc and self.proc.poll() is None:
            print(
                f"[arm_state] stopping the running {self.job} (homes, then releases)",
                flush=True,
            )
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=wait)
            except subprocess.TimeoutExpired:
                self.proc.kill()

    def _run(self, job, extra):
        cmd = (
            [START_ARM]
            + (["--robot", self.args.robot] if self.args.robot else [])
            + extra
            + ["--yes"]
        )
        log = open(self.args.orbit_log, "a")
        print(
            f"[arm_state] {job}: {' '.join(cmd)} (log {self.args.orbit_log})",
            flush=True,
        )
        self.proc = subprocess.Popen(
            cmd, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL
        )
        self.job, self.state = job, "running"

    def command(self, text):
        try:
            msg = json.loads(text) or {}
            op = msg.get("op")
        except (ValueError, AttributeError):
            return
        if self.args.no_commands:
            print(
                f"[arm_state] /d1_arm/command {op!r} ignored (--no-commands)",
                flush=True,
            )
            return
        if op == "orbit" and not self.proc:
            self._run("orbit", ["orbit"])
        elif op == "scan_target" and not self.proc:
            try:
                target = [float(v) for v in msg.get("target", [])]
            except (TypeError, ValueError):
                target = []
            if len(target) != 3 or any(abs(v) > 3.0 for v in target):
                print(
                    f"[arm_state] scan_target ignored: bad target {msg.get('target')!r}",
                    flush=True,
                )
                return
            self._run("scan", ["scan", "--target"] + [f"{v:.3f}" for v in target])
        elif op == "stop" and self.proc:
            print(f"[arm_state] stop the {self.job} (homes, then releases)", flush=True)
            self.proc.send_signal(signal.SIGINT)


def _terminate(*_):
    raise KeyboardInterrupt


def main():
    sys.path.insert(0, HERE)
    try:
        from d1_arm_control import default_arm_iface

        iface = default_arm_iface()
    except Exception:
        iface = "eth0"
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--robot",
        default=os.environ.get("ARM_ROBOT"),
        help="Go2 Wi-Fi IP (arm plugged into the Go2)",
    )
    ap.add_argument(
        "--monitor-port",
        type=int,
        default=int(
            os.environ.get(
                "ARM_BRIDGE_MONITOR_PORT",
                int(os.environ.get("ARM_BRIDGE_PORT", "18100")) + 1,
            )
        ),
    )
    ap.add_argument(
        "--iface", default=iface, help="wired arm interface (default: %(default)s)"
    )
    ap.add_argument(
        "--domain", type=int, default=int(os.environ.get("ARM_DOMAIN", "0"))
    )
    ap.add_argument(
        "--rosbridge",
        default=os.environ.get("TWIN_ROSBRIDGE", "localhost:9090"),
        help="host:port",
    )
    ap.add_argument("--rate", type=float, default=10.0, help="JointState rate, Hz")
    ap.add_argument(
        "--no-commands",
        action="store_true",
        help="ignore the dashboard's Orbit Scan button",
    )
    ap.add_argument("--orbit-log", default="/tmp/arm_orbit.log")
    args = ap.parse_args()

    host, _, port = args.rosbridge.rpartition(":")
    feed = ArmFeed(args)
    orbit = Orbit(args)
    # wifi_3d_nav.sh ends the session with SIGTERM: stop a running scan first.
    signal.signal(signal.SIGTERM, _terminate)
    try:
        serve(args, host, port, feed, orbit)
    finally:
        orbit.stop()


def serve(args, host, port, feed, orbit):
    """Publish to rosbridge forever, reconnecting when it goes away."""
    while True:
        try:
            ws = Rosbridge(host or "localhost", int(port))
        except (OSError, ConnectionError) as exc:
            print(
                f"[arm_state] rosbridge {args.rosbridge}: {exc}; retrying in 3 s",
                flush=True,
            )
            time.sleep(3.0)
            continue
        print(
            f"[arm_state] rosbridge {args.rosbridge} connected: /d1_arm/joint_states, /d1_arm/status",
            flush=True,
        )
        try:
            ws.send(
                {
                    "op": "advertise",
                    "topic": "/d1_arm/joint_states",
                    "type": "sensor_msgs/msg/JointState",
                }
            )
            ws.send(
                {
                    "op": "advertise",
                    "topic": "/d1_arm/status",
                    "type": "std_msgs/msg/String",
                }
            )
            ws.send(
                {
                    "op": "subscribe",
                    "topic": "/d1_arm/command",
                    "type": "std_msgs/msg/String",
                }
            )

            def reader():
                try:
                    while True:
                        m = ws.recv()
                        if (
                            m.get("op") == "publish"
                            and m.get("topic") == "/d1_arm/command"
                        ):
                            orbit.command((m.get("msg") or {}).get("data", ""))
                except (OSError, ConnectionError):
                    ws.close()

            threading.Thread(target=reader, daemon=True).start()
            last_status, last_sent = 0.0, None
            while True:
                now = time.time()
                if feed.alive() and feed.stamp != last_sent:
                    last_sent = feed.stamp
                    sec = int(feed.stamp)
                    ws.send(
                        {
                            "op": "publish",
                            "topic": "/d1_arm/joint_states",
                            "msg": {
                                "header": {
                                    "stamp": {
                                        "sec": sec,
                                        "nanosec": int((feed.stamp - sec) * 1e9),
                                    },
                                    "frame_id": "",
                                },
                                "name": JOINTS,
                                "position": joint_positions(feed.angles),
                                "velocity": [],
                                "effort": [],
                            },
                        }
                    )
                if now - last_status >= 1.0:
                    last_status = now
                    ws.send(
                        {
                            "op": "publish",
                            "topic": "/d1_arm/status",
                            "msg": {
                                "data": json.dumps(
                                    {
                                        "arm": feed.alive(),
                                        "orbit": orbit.poll(),
                                        "job": orbit.job,
                                        "commands": not args.no_commands,
                                    }
                                )
                            },
                        }
                    )
                time.sleep(1.0 / args.rate)
        except (OSError, ConnectionError) as exc:
            print(f"[arm_state] rosbridge lost ({exc}); reconnecting", flush=True)
            ws.close()
            time.sleep(1.0)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
