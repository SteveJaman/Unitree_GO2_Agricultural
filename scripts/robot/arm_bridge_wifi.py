#!/usr/bin/env python3
"""
arm_bridge_wifi.py: ON THE ROBOT. The D1 arm, plugged into the Go2's
Ethernet port, controlled from the laptop over Wi-Fi.

    arm (192.168.123.100) --eth0, DDS domain 0--> this bridge --Wi-Fi, TCP--> laptop D1Arm

The arm sits on the Go2's internal network (the Jetson is 192.168.123.18) and
speaks JSON strings on rt/arm_Command / rt/arm_Feedback. This bridge talks
to it with rclpy (message unitree_arm/ArmString = DDS
unitree_arm::msg::dds_::ArmString_) and serves one laptop over TCP, one JSON
object per line:

    laptop -> bridge   arm commands {"seq":N,"address":1,"funcode":F,"data":{...}}
                       and {"ping":1} about every second
    bridge -> laptop   {"bridge":"arm_bridge_wifi",...} once, then every
                       rt/arm_Feedback message (telemetry, status, acks)

Monitor port (ARM_BRIDGE_PORT + 1): read-only. Any number of clients get
every rt/arm_Feedback line and nothing they send is read, so the dashboard's
arm_state_bridge.py can watch the arm while start_arm.sh drives it.

Trajectories on the robot: {"traj": {"target": [6 deg], "gripper": g,
"max_vel": deg/s, "max_acc": deg/s^2, "rate": Hz, "mode": 0|1}} from the laptop
runs a minimum-jerk move here, streaming setpoints to the arm over the wired
DDS link (d1_motion.py's profile). Streamed over Wi-Fi instead, latency
spikes (200 ms) bunch the setpoints into bursts and the arm shakes on every
move. {"traj_stop": 1}, any arm command, or the laptop leaving stops it.

TCP instead of DDS over Wi-Fi: arm traffic is small JSON at 10 Hz, and TCP
needs no discovery and loses nothing. Safety: when the laptop disconnects or
is silent for ARM_BRIDGE_TIMEOUT s while the arm is compliant (teach mode:
last enable-all mode between 0 and 80000), the bridge stiffens it (mode
80000), so a Wi-Fi drop never lets the camera sag. A fully enabled arm
already holds by itself and is never sent anything: re-enabling it during a
move sets a new target on this firmware. Only GO2_HOST_IP may connect (if
set), and only arm commands (address 1) are forwarded.

Started by arm_bridge_wifi.sh (wifi_nav.sh --arm-wifi copies and runs it).
Env: ARM_BRIDGE_PORT=18100  ARM_BRIDGE_MONITOR_PORT=18101  ARM_BRIDGE_TIMEOUT=3
     GO2_HOST_IP=<laptop>
"""

import json
import math
import os
import signal
import socket
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from unitree_arm.msg import ArmString

PORT = int(os.environ.get("ARM_BRIDGE_PORT", "18100"))
MONITOR_PORT = int(os.environ.get("ARM_BRIDGE_MONITOR_PORT", str(PORT + 1)))
TIMEOUT = float(os.environ.get("ARM_BRIDGE_TIMEOUT", "3"))
ALLOW = os.environ.get("GO2_HOST_IP", "")
LOCK_MODE = 80000
TRAJ_REACH = 150.0  # deg: no trajectory target beyond this (play_arm.py's taught-pose limit)


def min_jerk(tau):
    tau = min(1.0, max(0.0, tau))
    return tau ** 3 * (10 - 15 * tau + 6 * tau * tau)


class Bridge(Node):
    def __init__(self):
        super().__init__("arm_bridge_wifi")
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.cmd_pub = self.create_publisher(ArmString, "arm_Command", reliable)
        self.create_subscription(
            ArmString, "arm_Feedback", self.on_feedback, best_effort
        )
        self.conn = None
        self.conn_lock = threading.Lock()
        self.monitors = []  # read-only clients of the monitor port
        self.mon_lock = threading.Lock()
        self.feedback = 0
        self.mode = None  # last enable-all mode sent (None: unknown, 0: released)
        self.angles = None  # newest joints 0-6 from telemetry (address 2, funcode 1)
        self.traj_stop = threading.Event()
        self.traj_thread = None
        self.seq = int(time.time()) % 1000000 * 1000

    def on_feedback(self, msg):
        self.feedback += 1
        try:
            fb = json.loads(msg.data)
            if fb.get("address") == 2 and fb.get("funcode") == 1:
                d = fb.get("data") or {}
                vals = [d.get(f"angle{i}") for i in range(7)]
                if all(v is not None for v in vals[:6]):
                    self.angles = [float(v) if v is not None else 0.0 for v in vals]
        except (ValueError, AttributeError, TypeError):
            pass
        line = (msg.data.strip() + "\n").encode()
        with self.mon_lock:
            for mon in list(self.monitors):
                try:
                    mon.sendall(line)
                except OSError:  # gone, or too slow (send timeout): drop it
                    self.monitors.remove(mon)
                    mon.close()
        with self.conn_lock:
            if self.conn is None:
                return
            try:
                self.conn.sendall(line)
            except OSError:
                self.conn = None  # the serving loop notices and cleans up

    def command(self, text):
        self.cmd_pub.publish(ArmString(data=text))

    def lock_arm(self, why):
        """Hold the arm where it is (enable all, mode 80000), sent three times."""
        self.seq += 1
        text = json.dumps(
            {"seq": self.seq, "address": 1, "funcode": 5, "data": {"mode": LOCK_MODE}},
            separators=(",", ":"),
        )
        for _ in range(3):
            self.command(text)
            time.sleep(0.1)
        self.mode = LOCK_MODE
        self.get_logger().warn(f"{why}: compliant arm locked (mode {LOCK_MODE})")

    def stop_traj(self):
        if self.traj_thread and self.traj_thread.is_alive():
            self.traj_stop.set()
            self.traj_thread.join(timeout=1.0)

    def start_traj(self, spec):
        """Run a minimum-jerk move to spec["target"] here, streaming setpoints to the arm."""
        self.stop_traj()
        try:
            target = [float(v) for v in spec["target"]][:6]
            gripper = float(spec.get("gripper", 0.0))
            vmax = max(1.0, float(spec.get("max_vel", 30.0)))
            amax = max(1.0, float(spec.get("max_acc", 60.0)))
            rate = min(100.0, max(5.0, float(spec.get("rate", 50.0))))
            mode = int(spec.get("mode", 1))
            tmin = float(spec.get("min_duration", 0.8))
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn(f"bad traj request ignored: {spec!r}"[:120])
            return
        if len(target) != 6 or any(abs(v) > TRAJ_REACH for v in target):
            self.get_logger().warn(f"traj target refused (6 joints within +/-{TRAJ_REACH:.0f}): {target}")
            return
        self.traj_stop.clear()
        self.traj_thread = threading.Thread(
            target=self._run_traj, args=(target, gripper, vmax, amax, rate, mode, tmin), daemon=True)
        self.traj_thread.start()

    def _send_joints(self, angles, gripper, mode):
        self.seq += 1
        data = {"mode": mode, **{f"angle{j}": round(a, 2) for j, a in enumerate(angles)}, "angle6": gripper}
        self.command(json.dumps({"seq": self.seq, "address": 1, "funcode": 2, "data": data},
                                separators=(",", ":")))

    def _run_traj(self, target, gripper, vmax, amax, rate, mode, tmin):
        start = list(self.angles[:6]) if self.angles else None
        if start is None:   # no telemetry: one direct command
            for _ in range(3):
                self._send_joints(target, gripper, mode)
                time.sleep(0.1)
            return
        d = max(abs(t - s) for s, t in zip(start, target))
        duration = max(tmin, 1.875 * d / vmax, math.sqrt(5.774 * d / amax)) if d > 1e-6 else tmin
        t0 = time.monotonic()
        while not self.traj_stop.is_set():
            k = min_jerk((time.monotonic() - t0) / duration)
            self._send_joints([a + (b - a) * k for a, b in zip(start, target)], gripper, mode)
            if k >= 1.0:
                break
            time.sleep(1.0 / rate)
        if not self.traj_stop.is_set():
            for _ in range(2):   # the target, reliably
                time.sleep(0.1)
                self._send_joints(target, gripper, mode)

    def on_line(self, line):
        """One line from the laptop: a ping, a trajectory request, or an arm command to forward."""
        try:
            msg = json.loads(line)
        except ValueError:
            self.get_logger().warn(f"not JSON, ignored: {line[:80]!r}")
            return
        if not isinstance(msg, dict) or "ping" in msg:
            return
        if "traj" in msg:
            self.start_traj(msg["traj"] if isinstance(msg["traj"], dict) else {})
            return
        if "traj_stop" in msg:
            self.stop_traj()
            return
        if msg.get("address") != 1 or not isinstance(msg.get("funcode"), int):
            self.get_logger().warn(f"not an arm command, ignored: {line[:80]!r}")
            return
        self.stop_traj()   # the laptop drives the arm directly again
        if msg["funcode"] == 5:
            self.mode = (msg.get("data") or {}).get("mode", 0)
        self.command(line)

    def serve(self, conn, addr):
        conn.settimeout(0.5)
        hello = {
            "bridge": "arm_bridge_wifi",
            "port": PORT,
            "feedback_msgs": self.feedback,
            "features": ["traj"],
        }
        conn.sendall((json.dumps(hello) + "\n").encode())
        with self.conn_lock:
            self.conn = conn
        self.get_logger().info(f"laptop {addr} connected")
        buf, last = b"", time.monotonic()
        why = "laptop disconnected"
        try:
            while rclpy.ok():
                try:
                    chunk = conn.recv(4096)
                except socket.timeout:
                    chunk = None
                if chunk == b"":
                    break
                if chunk:
                    last = time.monotonic()
                    buf += chunk
                    *lines, buf = buf.split(b"\n")
                    for raw in lines:
                        if raw.strip():
                            self.on_line(raw.decode(errors="replace").strip())
                if time.monotonic() - last > TIMEOUT:
                    why = f"laptop silent for {TIMEOUT:.0f} s"
                    break
                with self.conn_lock:
                    if self.conn is None:
                        why = "laptop unreachable"
                        break
        except OSError as e:
            why = f"connection error ({e})"
        except KeyboardInterrupt:
            why = "bridge stopping"
            raise
        finally:
            self.stop_traj()   # no move keeps running for a laptop that is gone
            with self.conn_lock:
                self.conn = None
            conn.close()
            # Only a compliant arm (teach mode, 0 < mode < 80000) can sag; it is
            # stiffened. A fully enabled arm already holds, and re-enabling it
            # mid-motion sets a new target on this firmware (tested: the
            # shoulder swung to its limit), so nothing is ever sent to it.
            if self.mode and self.mode < LOCK_MODE:
                self.lock_arm(why)
            else:
                state = {None: "unknown", 0: "released"}.get(
                    self.mode, f"mode {self.mode}"
                )
                self.get_logger().info(f"{why} (arm {state}: nothing sent)")


def serve_monitors(node):
    """Accept read-only monitor clients; feedback reaches them in on_feedback."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", MONITOR_PORT))
    srv.listen(4)
    while True:
        conn, (addr, _) = srv.accept()
        if ALLOW and addr != ALLOW:
            conn.close()
            continue
        conn.settimeout(0.5)  # a stalled monitor is dropped, never blocks the arm link
        with node.mon_lock:
            node.monitors.append(conn)
        node.get_logger().info(f"monitor {addr} connected (read-only, :{MONITOR_PORT})")


def _stop(signum, frame):
    raise KeyboardInterrupt  # SIGTERM (wifi_nav.sh teardown) runs the same cleanup as Ctrl+C


def main():
    # rclpy's own SIGINT/SIGTERM handlers shut ROS down before the arm could be
    # locked: turn them off (Humble+), and install ours after init (Foxy).
    try:
        from rclpy.signals import SignalHandlerOptions

        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    node = Bridge()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    threading.Thread(target=serve_monitors, args=(node,), daemon=True).start()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", PORT))
    srv.listen(1)
    node.get_logger().info(
        f"TCP :{PORT}, client {ALLOW or 'any'}, timeout {TIMEOUT:.0f} s; "
        "waiting for rt/arm_Feedback and the laptop"
    )
    try:
        while rclpy.ok():
            conn, (addr, _) = srv.accept()
            if ALLOW and addr != ALLOW:
                node.get_logger().warn(f"refused {addr} (only {ALLOW} may connect)")
                conn.close()
                continue
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            node.serve(conn, addr)
    except KeyboardInterrupt:
        pass
    finally:
        srv.close()
        time.sleep(0.3)  # let the lock leave before ROS goes down
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
