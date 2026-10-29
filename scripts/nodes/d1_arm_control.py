#!/usr/bin/env python3
"""
d1_arm_control.py

Control the Unitree D1 arm over DDS (CycloneDDS).

Verified config (proven with move_test.py):
    iface    = the wired NIC on 192.168.123.x (auto-detected; ARM_IFACE overrides)
    domain   = 0
    discovery = MULTICAST  (do NOT pass --peer on this firmware)

Topics:
    rt/arm_Command   (write)  — JSON in ArmString_.data
    rt/arm_Feedback  (read)   — JSON in ArmString_.data

All angles are in DEGREES. Joints 0–5 are the arm, joint 6 is the gripper.
Enable uses mode 80000 (fully locked) by default.

Over Wi-Fi: with the arm plugged into the Go2's Ethernet port, --robot <Go2
Wi-Fi IP> (or ARM_ROBOT) talks to scripts/robot/arm_bridge_wifi.py on the
robot over TCP instead of DDS; every command and reading is the same. Every
script that uses D1Arm (teach, play, calibration, fold) follows ARM_ROBOT.

Usage (chain-style):
    python3 d1_arm_control.py state
    python3 d1_arm_control.py enable sleep 2 joint 0 20 sleep 3
    python3 d1_arm_control.py home sleep 6 release
    python3 d1_arm_control.py lift --deg 45
    python3 d1_arm_control.py sweep --pct 90
    python3 d1_arm_control.py --robot 192.168.1.9 state     # over Wi-Fi
"""

import argparse
import collections
import json
import os
import re
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass

# DDS is only needed for the wired link; over Wi-Fi (ARM_ROBOT) the robot's
# bridge speaks DDS and the laptop only TCP.
try:
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.pub import DataWriter
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic
    from cyclonedds.idl import IdlStruct

    @dataclass
    class ArmString_(IdlStruct, typename="unitree_arm::msg::dds_::ArmString_"):
        data: str = ""
except ImportError:
    DomainParticipant = None


ADDR_CMD, ADDR_TELEM, ADDR_ACK = 1, 2, 3
FUNC_JOINT, FUNC_JOINTS, FUNC_ENABLE_ALL, FUNC_POWER, FUNC_ZERO = 1, 2, 5, 6, 7
ENABLE_FULL = 80000

JOINT_LIMITS = {0: 135.0, 1: 90.0, 2: 90.0, 3: 135.0, 4: 90.0, 5: 135.0}
JOINT_NAMES = {
    0: "base",
    1: "shoulder",
    2: "elbow",
    3: "wrist_roll",
    4: "wrist_pitch",
    5: "wrist_yaw",
}

T_MOVE, T_HOME, T_ENABLE, T_RELEASE = 3.0, 6.0, 1.5, 0.5
DEG_WAIT = 0.04

BRIDGE_PORT = int(os.environ.get("ARM_BRIDGE_PORT", "18100"))


def default_arm_iface():
    """ARM_IFACE, else the wired NIC on 192.168.123.x, else the first wired NIC.

    Same rule as go2_net.sh: interface names differ per machine (enp0s31f6 on
    the native Ubuntu workstation), so they are detected, not hardcoded.
    """
    if os.environ.get("ARM_IFACE"):
        return os.environ["ARM_IFACE"]
    net = "/sys/class/net"
    try:
        wired = sorted(
            d
            for d in os.listdir(net)
            if os.path.exists(os.path.join(net, d, "device"))
            and not os.path.isdir(os.path.join(net, d, "wireless"))
        )
    except OSError:
        wired = []
    for d in wired:
        res = subprocess.run(
            ["ip", "-4", "-o", "addr", "show", "dev", d],
            capture_output=True,
            text=True,
        )
        if " 192.168.123." in res.stdout:
            return d
    return wired[0] if wired else "eth0"


def robot_host():
    """The Go2's Wi-Fi IP when the arm is reached through its bridge, else None."""
    return os.environ.get("ARM_ROBOT") or None


class TcpLink:
    """rt/arm_Command / rt/arm_Feedback through arm_bridge_wifi.py on the Go2:
    one JSON object per line over TCP. A ping every second tells the bridge the
    laptop is alive; if the pings stop it locks the arm."""

    def __init__(self, host, port=BRIDGE_PORT, timeout=5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(None)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.lines = collections.deque(maxlen=500)
        self.hello = None
        self.alive = True
        self.wlock = threading.Lock()
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._pinger, daemon=True).start()

    def _reader(self):
        buf = b""
        try:
            while self.alive:
                chunk = self.sock.recv(65536)
                if not chunk:
                    break
                *lines, buf = (buf + chunk).split(b"\n")
                for raw in lines:
                    text = raw.decode(errors="replace").strip()
                    if text.startswith('{"bridge"'):
                        self.hello = text
                    elif text:
                        self.lines.append(text)
        except OSError:
            pass
        self.alive = False

    def _pinger(self):
        while self.alive:
            try:
                self.write('{"ping":1}')
            except ConnectionError:
                return
            time.sleep(1.0)

    def write(self, text):
        try:
            with self.wlock:
                self.sock.sendall((text + "\n").encode())
        except OSError as e:
            self.alive = False
            raise ConnectionError(f"arm bridge connection lost: {e}") from e

    def take(self):
        out = []
        while self.lines:
            out.append(self.lines.popleft())
        return out

    def close(self):
        self.alive = False
        try:
            self.sock.close()
        except OSError:
            pass


def build_uri(iface, peer=None):
    iface_xml = f'<NetworkInterface name="{iface}" multicast="default"/>'
    if peer:
        mc = "<AllowMulticast>false</AllowMulticast>"
        disc = f'<Discovery><Peers><Peer address="{peer}"/></Peers></Discovery>'
    else:
        mc, disc = "", ""
    return (
        '<CycloneDDS xmlns="https://cdds.io/config">'
        f'<Domain id="any"><General><Interfaces>{iface_xml}'
        f"</Interfaces>{mc}</General>{disc}</Domain></CycloneDDS>"
    )


def preflight(iface, arm_ip, want_ping=True):
    robot = robot_host()
    if robot:
        print(f"[preflight] Arm through the Go2 bridge at {robot}:{BRIDGE_PORT}:")
        try:
            socket.create_connection((robot, BRIDGE_PORT), timeout=3).close()
        except OSError as e:
            print(
                f"    [!] bridge not reachable ({e}). Start it with "
                "wifi_nav.sh --arm-wifi, or on the robot: "
                "GO2_HOST_IP=<laptop IP> bash ~/go2-agri-arm/arm_bridge_wifi.sh"
            )
            return False
        print("    [ok] bridge reachable")
        return True
    print("[preflight] Network diagnostics:")
    try:
        res = subprocess.run(
            ["ip", "-4", "-o", "addr", "show", "dev", iface],
            capture_output=True,
            text=True,
            timeout=3,
        )
    except FileNotFoundError:
        print("    [!] `ip` not found")
        return True
    if res.returncode != 0:
        print(f"    [!] interface {iface} does not exist")
        return False
    addrs = re.findall(r"inet (\d+\.\d+\.\d+\.\d+/\d+)", res.stdout)
    if not addrs:
        print(f"    [!] {iface} has no IPv4 address")
        return False
    print(f"    [ok] {iface} = {', '.join(addrs)}")

    res = subprocess.run(["ip", "route", "get", arm_ip], capture_output=True, text=True)
    if iface not in res.stdout:
        print(f"    [!] route to {arm_ip} does NOT go via {iface}")
        return False
    print(f"    [ok] route to {arm_ip} via {iface}")

    if want_ping:
        res = subprocess.run(
            ["ping", "-c", "2", "-W", "2", arm_ip], capture_output=True, text=True
        )
        if res.returncode != 0:
            print(f"    [!] {arm_ip} does NOT reply to ping")
            return False
        print(f"    [ok] {arm_ip} replies to ping")
    return True


class D1Arm:
    def __init__(
        self,
        iface=None,
        domain=0,
        peer=None,
        verbose=False,
        match_timeout=20.0,
        retries=3,
        robot=None,
    ):
        self.verbose = verbose
        self.seq = 0
        self.link = None
        robot = robot or robot_host()
        if robot:
            self.ok = self._connect_bridge(robot, match_timeout, retries)
            return
        if DomainParticipant is None:
            sys.exit(
                "cyclonedds not installed. Run: pip3 install cyclonedds "
                "(or use the Go2 bridge: --robot / ARM_ROBOT)"
            )
        os.environ["CYCLONEDDS_URI"] = build_uri(iface or default_arm_iface(), peer)
        if verbose:
            print(f"      CYCLONEDDS_URI = {os.environ['CYCLONEDDS_URI']}")
        self.ok = False
        for attempt in range(1, retries + 1):
            if verbose:
                print(
                    f"      discovery attempt {attempt}/{retries} "
                    f"(timeout {match_timeout:.0f}s)..."
                )
            if self._init(domain, match_timeout):
                self.ok = True
                return
        self.p = self.cmd_w = self.fb_r = None

    def _connect_bridge(self, robot, timeout, retries):
        """TCP to arm_bridge_wifi.py, then wait for arm feedback through it."""
        for attempt in range(1, retries + 1):
            try:
                self.link = TcpLink(robot, BRIDGE_PORT)
            except OSError as e:
                if self.verbose:
                    print(
                        f"      bridge {robot}:{BRIDGE_PORT} attempt {attempt}/{retries}: {e}"
                    )
                time.sleep(2.0)
                continue
            t0 = time.time()
            while time.time() - t0 < timeout and self.link.alive:
                if self.link.lines:
                    if self.verbose:
                        print(f"      bridge {robot}:{BRIDGE_PORT}: arm feedback OK")
                    return True
                time.sleep(0.1)
            print(
                f"[!] bridge at {robot} connected, but no arm feedback in {timeout:.0f} s: "
                "is the arm plugged into the Go2 and powered?"
            )
            self.link.close()
            self.link = None
        return False

    def _write(self, text):
        if self.link:
            self.link.write(text)
        else:
            self.cmd_w.write(ArmString_(data=text))

    def _take(self):
        if self.link:
            return self.link.take()
        return [s.data for s in self.fb_r.take(N=20)]

    def _init(self, domain, timeout):
        try:
            self.p = DomainParticipant(domain)
            self.cmd_w = DataWriter(self.p, Topic(self.p, "rt/arm_Command", ArmString_))
            self.fb_r = DataReader(self.p, Topic(self.p, "rt/arm_Feedback", ArmString_))
        except Exception as e:
            if self.verbose:
                print(f"      participant init failed: {e}")
            return False
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                w = self.cmd_w.get_publication_matched_status().current_count
                r = self.fb_r.get_subscription_matched_status().current_count
            except Exception:
                time.sleep(1.0)
                return True
            if w > 0 or r > 0:
                time.sleep(0.5)
                return True
            time.sleep(0.15)
        return False

    def _send(self, funcode, data=None, repeat=3):
        self.seq += 1
        payload = {"seq": self.seq, "address": ADDR_CMD, "funcode": funcode}
        if data is not None:
            payload["data"] = data
        text = json.dumps(payload, separators=(",", ":"))
        if self.verbose:
            print(f"      -> {text}")
        for _ in range(repeat):
            self._write(text)
            time.sleep(0.1)

    def _latest(self, address, funcode, timeout=2.0):
        found = None
        t0 = time.time()
        while time.time() - t0 < timeout:
            for data in self._take():
                try:
                    m = json.loads(data)
                except Exception:
                    continue
                if m.get("address") == address and m.get("funcode") == funcode:
                    found = m
            if found:
                break
            time.sleep(0.05)
        return found

    def state(self, timeout=2.0):
        return self._latest(ADDR_TELEM, 3, timeout)

    def angles(self, timeout=2.0):
        return self._latest(ADDR_TELEM, 1, timeout)

    def ack(self):
        return self._latest(ADDR_ACK, 1)

    def enable(self, mode=ENABLE_FULL):
        self._send(FUNC_ENABLE_ALL, {"mode": mode})

    def release(self):
        self._send(FUNC_ENABLE_ALL, {"mode": 0})

    def home(self):
        self._send(FUNC_ZERO)

    def power(self, on):
        self._send(FUNC_POWER, {"power": 1 if on else 0})

    def joint(self, jid, angle):
        if jid not in JOINT_LIMITS:
            raise ValueError(f"joint {jid} has no known limit (valid 0..5)")
        lim = JOINT_LIMITS[jid]
        if abs(angle) > lim:
            raise ValueError(f"joint {jid} limit ±{lim}°, got {angle}°")
        self._send(FUNC_JOINT, {"id": jid, "angle": angle, "delay_ms": 0})


def print_state(arm, label):
    st = arm.state()
    if st:
        d = st.get("data", {})
        print(
            f"    [{label}] enable={d.get('enable_status')}  "
            f"power={d.get('power_status')}  error={d.get('error_status')}"
        )
        return d
    print(f"    [{label}] (no state)")
    return {}


def _do_sweep(arm, joints, pct, continue_on_error, yes):
    print(f"[sweep] joints={joints}  pct={pct}%")
    if not yes:
        try:
            if input("Continue sweep? [y/N] ").strip().lower() != "y":
                print("aborted")
                return 0
        except EOFError:
            print("aborted")
            return 0
    enabled_by_us = False
    errors = []
    try:
        arm.enable(80000)
        time.sleep(T_ENABLE)
        st = print_state(arm, "after-enable")
        if st.get("enable_status") != 1:
            print("[!] enable failed")
            return 3
        enabled_by_us = True
        for i, jid in enumerate(joints, 1):
            lim = JOINT_LIMITS[jid]
            tgt = round(lim * pct / 100.0, 1)
            print(f"\n[{i}/{len(joints)}] joint {jid} ({JOINT_NAMES[jid]}) ±{tgt:.0f}°")
            try:
                arm.home()
                time.sleep(T_HOME)
                arm.joint(jid, +tgt)
                time.sleep(T_MOVE + tgt * DEG_WAIT)
                print_state(arm, f"j{jid}=+{tgt:.0f}")
                arm.joint(jid, -tgt)
                time.sleep(T_MOVE + tgt * DEG_WAIT)
                print_state(arm, f"j{jid}=-{tgt:.0f}")
                arm.joint(jid, 0.0)
                time.sleep(T_MOVE)
                print_state(arm, "zero")
            except Exception as e:
                print(f"[!] joint {jid}: {e}")
                errors.append((jid, str(e)))
                if not continue_on_error:
                    raise
    except KeyboardInterrupt:
        print("\n[!] interrupted")
    finally:
        if enabled_by_us:
            try:
                arm.home()
                time.sleep(T_HOME)
            except Exception:
                pass
            try:
                arm.release()
                time.sleep(T_RELEASE)
            except Exception:
                pass
            print_state(arm, "final")
    if errors:
        for jid, msg in errors:
            print(f"  joint {jid}: {msg}")
        return 1
    return 0


def _do_lift(arm, deg, yes):
    print(f"[lift] shoulder (joint 1) -> {deg:+.1f}°")
    if not yes:
        try:
            if input("Continue lift? [y/N] ").strip().lower() != "y":
                print("aborted")
                return 0
        except EOFError:
            print("aborted")
            return 0
    enabled_by_us = False
    try:
        arm.enable(80000)
        time.sleep(T_ENABLE)
        if print_state(arm, "after-enable").get("enable_status") != 1:
            print("[!] enable failed")
            return 3
        enabled_by_us = True
        arm.home()
        time.sleep(T_HOME)
        print_state(arm, "after-home")
        arm.joint(1, deg)
        time.sleep(T_MOVE + abs(deg) * DEG_WAIT)
        print_state(arm, "after-lift")
    finally:
        if enabled_by_us:
            try:
                arm.home()
                time.sleep(T_HOME)
            except Exception:
                pass
            try:
                arm.release()
                time.sleep(T_RELEASE)
            except Exception:
                pass
            print_state(arm, "final")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--iface", default=default_arm_iface())
    ap.add_argument(
        "--domain", type=int, default=int(os.environ.get("ARM_DOMAIN", "0"))
    )
    ap.add_argument("--ip", default=os.environ.get("ARM_IP", "192.168.123.100"))
    ap.add_argument(
        "--robot",
        default=robot_host(),
        help="Go2 Wi-Fi IP: reach the arm through arm_bridge_wifi.py (env ARM_ROBOT)",
    )
    ap.add_argument(
        "--peer",
        default=None,
        help="unicast peer — DO NOT USE on this firmware; multicast works",
    )
    ap.add_argument("--enable-mode", type=int, default=ENABLE_FULL)
    ap.add_argument("--match-timeout", type=float, default=20.0)
    ap.add_argument("--retries", type=int, default=3)
    ap.add_argument("--verbose", "-v", action="store_true")
    ap.add_argument("--yes", "-y", action="store_true")
    ap.add_argument("--skip-preflight", action="store_true")
    ap.add_argument("--deg", type=float, default=45.0, help="for 'lift' verb")
    ap.add_argument("--pct", type=float, default=90.0, help="for 'sweep' verb")
    ap.add_argument(
        "--joints",
        default="all",
        help="for 'sweep': 'all' or comma-separated joint ids",
    )
    ap.add_argument("--continue-on-error", action="store_true")
    ap.add_argument(
        "commands",
        nargs="*",
        help="chain commands: state|enable|release|home|power 0|1|"
        "joint J A|joints A0..A6|sleep S|feedback|status|motors|"
        "lift|sweep",
    )

    args = ap.parse_args()
    if args.robot:
        os.environ["ARM_ROBOT"] = args.robot

    if not args.skip_preflight and not preflight(args.iface, args.ip):
        print("\n[!] Preflight failed")
        return 2

    arm = D1Arm(
        args.iface,
        args.domain,
        args.peer,
        verbose=args.verbose,
        match_timeout=args.match_timeout,
        retries=args.retries,
    )
    if not arm.ok:
        print(
            "[!] no connection to the arm"
            + (" (Go2 bridge)" if args.robot else " (DDS discovery failed)")
        )
        return 2
    print(f"[*] arm connected ({'Go2 bridge ' + args.robot if arm.link else 'DDS'})")

    if not args.commands:
        # Default: just print state
        print_state(arm, "state")
        return 0

    # Dispatch sweep / lift directly (they're long-running)
    if args.commands[0] == "sweep":
        if args.joints.lower() == "all":
            joints = sorted(JOINT_LIMITS.keys())
        else:
            joints = [int(x) for x in args.joints.split(",")]
        return _do_sweep(arm, joints, args.pct, args.continue_on_error, args.yes)
    if args.commands[0] == "lift":
        return _do_lift(arm, args.deg, args.yes)

    # Chain-style
    cmds = args.commands
    i = 0
    try:
        while i < len(cmds):
            c = cmds[i]
            if c == "state":
                print_state(arm, "state")
                i += 1
            elif c == "enable":
                print("[d1] enable")
                arm.enable(args.enable_mode)
                i += 1
            elif c in ("release", "disable"):
                print("[d1] release")
                arm.release()
                i += 1
            elif c in ("home", "zero"):
                print("[d1] home")
                arm.home()
                i += 1
            elif c == "power":
                on = int(cmds[i + 1]) != 0
                print(f"[d1] power {'on' if on else 'off'}")
                arm.power(on)
                i += 2
            elif c == "joint":
                jid, ang = int(cmds[i + 1]), float(cmds[i + 2])
                print(f"[d1] joint {jid} -> {ang:+.2f}°")
                arm.joint(jid, ang)
                i += 3
            elif c == "joints":
                vals = [float(x) for x in cmds[i + 1 : i + 8]]
                if len(vals) != 7:
                    raise ValueError("joints needs 7 values (6 joints + gripper)")
                # send one at a time to enforce limits
                for jid in range(6):
                    if vals[jid] != 0:
                        arm.joint(jid, vals[jid])
                        time.sleep(0.5)
                i += 8
            elif c == "feedback":
                a = arm.angles()
                print(f"[d1] angles: {json.dumps(a) if a else 'None'}")
                i += 1
            elif c == "status":
                s = arm.state()
                print(f"[d1] state: {json.dumps(s) if s else 'None'}")
                i += 1
            elif c == "motors":
                m = arm._latest(ADDR_TELEM, 4, timeout=2.0)
                print(f"[d1] motors: {json.dumps(m) if m else 'None'}")
                i += 1
            elif c == "sleep":
                s = float(cmds[i + 1])
                print(f"[d1] sleep {s}s")
                time.sleep(s)
                i += 2
            else:
                print(f"[d1] unknown command: {c}", file=sys.stderr)
                return 1
    except (IndexError, ValueError) as e:
        print(f"[d1] error in command {cmds[i]!r}: {e}", file=sys.stderr)
        return 1

    time.sleep(0.3)
    return 0


if __name__ == "__main__":
    sys.exit(main())
