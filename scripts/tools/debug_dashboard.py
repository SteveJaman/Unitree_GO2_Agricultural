#!/usr/bin/env python3
"""
debug_dashboard.py: a local debug page for the Go2 Wi-Fi navigation stack,
http://localhost:3001. Python standard library only; runs on the host next to
a wifi_nav.sh / wifi_3d_nav.sh session (and tells you when there is none).

  * "Why isn't it moving?": the chain from LiDAR to legs, checked in order
    (session, LiDAR scans, localization TF, manual control, goal, planner,
    footprint in an obstacle, controller output, smoother, the dog actually
    walking, recovery, battery), each green / amber / red with what to do.
  * Topic health: rate and age of the topics Nav2 depends on.
  * Command chain: /cmd_vel_nav -> /cmd_vel -> speed walked (from TF).
  * Local costmap with the footprint and the plan, as Nav2 sees it.
  * ROS warnings and errors (/rosout, every node), counts per node.
  * Logs: every session log (host, go2-lidar3d, the robot) in one viewer.
  * System: containers, robot ping, Wi-Fi signal, UDP buffers, Ollama.

Live data come from scripts/nodes/go2_debug_probe.py, started here inside
go2-lidar3d with the session's DDS settings (read from a running Nav2
process). The probe only subscribes: this page cannot move the dog.

Usage:
    python3 scripts/tools/debug_dashboard.py              # http://localhost:3001
    python3 scripts/tools/debug_dashboard.py --port 3002
    python3 scripts/tools/debug_dashboard.py --host 0.0.0.0   # reachable from the LAN
Env: NAV2_WIFI_DIR (~/files/go2-nav2-wifi), GO2_CONTAINER (go2-lidar3d).
"""

import argparse
import glob
import json
import os
import re
import shlex
import subprocess
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
PAGE = os.path.join(REPO, "twin", "debug", "index.html")
PROBE_SRC = os.path.join(REPO, "scripts", "nodes", "go2_debug_probe.py")
NAV2 = os.path.expanduser(os.environ.get("NAV2_WIFI_DIR", "~/files/go2-nav2-wifi"))
CONTAINER = os.environ.get("GO2_CONTAINER", "go2-lidar3d")
IP_CACHE = os.path.expanduser(
    os.path.join(os.environ.get("XDG_CACHE_HOME", "~/.cache"), "go2_wifi_nav", "robot_ip")
)
SSH = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=4", "-o", "HostKeyAlias=go2-robot",
       "-o", "ServerAliveInterval=3", "-o", "ServerAliveCountMax=2"]
ROS_LOG = "/tmp/go2-lidar3d-ros-log"


def run(cmd, timeout=6, text=True):
    """stdout of a command, or "" (never raises, never hangs)."""
    try:
        return subprocess.run(cmd, capture_output=True, text=text, timeout=timeout).stdout
    except (subprocess.TimeoutExpired, OSError):
        return ""


def robot_ip():
    try:
        return open(IP_CACHE).readline().strip() or None
    except OSError:
        return None


# ---------------------------------------------------------------- probe
class ProbeRunner:
    """Keeps go2_debug_probe.py running in the container; holds its latest line."""

    def __init__(self):
        self.state, self.state_time = None, 0.0
        self.note = "starting"
        self.proc = None
        threading.Thread(target=self.loop, daemon=True).start()

    def session_env(self):
        """DDS environment of a running Nav2 / session node in the container."""
        pids = run(["docker", "exec", CONTAINER, "pgrep", "-f",
                    "controller_server|planner_server|go2_odom_tf|go2_urdf_self_filter"]).split()
        for pid in pids:
            env = run(["docker", "exec", CONTAINER, "cat", f"/proc/{pid}/environ"])
            vals = dict(kv.split("=", 1) for kv in env.split("\0") if "=" in kv)
            if "ROS_DOMAIN_ID" in vals or "CYCLONEDDS_URI" in vals:
                keep = ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "CYCLONEDDS_URI", "ROS_LOCALHOST_ONLY")
                return {k: vals[k] for k in keep if k in vals}
        return None

    def loop(self):
        while True:
            running = CONTAINER in run(["docker", "ps", "--format", "{{.Names}}"]).split()
            if not running:
                self.note = f"{CONTAINER} is not running: no navigation session"
                time.sleep(3)
                continue
            env = self.session_env()
            if env is None:
                self.note = f"{CONTAINER} runs, but no Nav2 session in it yet"
                time.sleep(3)
                continue
            run(["docker", "cp", PROBE_SRC, f"{CONTAINER}:/tmp/go2_debug_probe.py"])
            cmd = ["docker", "exec", "-i"]
            for k, v in env.items():
                cmd += ["-e", f"{k}={v}"]
            cmd += [CONTAINER, "bash", "-c",
                    "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash 2>/dev/null; "
                    "exec python3 -u /tmp/go2_debug_probe.py 2>/tmp/go2_debug_probe.err"]
            self.note = "probe starting (DDS discovery takes a few seconds)"
            self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                         stderr=subprocess.DEVNULL, text=True, bufsize=1)
            for line in self.proc.stdout:
                try:
                    self.state, self.state_time = json.loads(line), time.time()
                    self.note = "live"
                except ValueError:
                    pass
            self.proc.wait()
            self.state = None
            self.note = "probe stopped (session ended?); restarting"
            time.sleep(2)

    def stop(self):
        if self.proc and self.proc.poll() is None:
            try:
                self.proc.stdin.close()  # the probe exits when its stdin closes
            except OSError:
                pass
            self.proc.terminate()

    def snapshot(self):
        age = time.time() - self.state_time if self.state else None
        return {"note": self.note, "age": None if age is None else round(age, 1),
                "state": self.state if self.note == "live" and age is not None and age < 5 else None}


# --------------------------------------------------------------- health
class Health:
    def __init__(self):
        self.data = {}
        threading.Thread(target=self.loop, daemon=True).start()

    def loop(self):
        while True:
            try:
                self.data = self.collect()
            except Exception as e:  # noqa: BLE001 - keep the page alive
                self.data = {"error": str(e)}
            time.sleep(5)

    def collect(self):
        d = {"t": time.time()}
        d["containers"] = [dict(zip(("name", "status"), line.split("\t", 1)))
                           for line in run(["docker", "ps", "-a", "--format", "{{.Names}}\t{{.Status}}"]).splitlines()
                           if line.strip()]
        ip = robot_ip()
        d["robot_ip"] = ip
        if ip:
            out = run(["ping", "-c", "3", "-i", "0.2", "-W", "1", ip], timeout=5)
            m = re.search(r"([\d.]+)% packet loss", out)
            r = re.search(r"= [\d.]+/([\d.]+)/([\d.]+)", out)
            d["ping"] = {"loss": round(float(m.group(1))) if m else 100,
                         "avg_ms": float(r.group(1)) if r else None,
                         "max_ms": float(r.group(2)) if r else None}
            dev = re.search(r"dev (\S+)", run(["ip", "route", "get", ip]))
            if dev:
                d["iface"] = dev.group(1)
                link = run(["iw", "dev", dev.group(1), "link"])
                sig = re.search(r"signal: (-?\d+)", link)
                rate = re.search(r"tx bitrate: ([\d.]+ \S+)", link)
                ssid = re.search(r"SSID: (.+)", link)
                d["wifi"] = {"signal_dbm": int(sig.group(1)) if sig else None,
                             "tx": rate.group(1) if rate else None,
                             "ssid": ssid.group(1).strip() if ssid else None}
                if not link:  # no `iw`: NetworkManager's view (signal in %)
                    for line in run(["nmcli", "-t", "-f", "IN-USE,SSID,SIGNAL,RATE", "dev", "wifi",
                                     "list", "ifname", dev.group(1), "--rescan", "no"]).splitlines():
                        if line.startswith("*"):
                            # -t escapes ':' inside the SSID as '\:'
                            parts = re.split(r"(?<!\\):", line)
                            if len(parts) >= 4:
                                d["wifi"] = {"signal_pct": int(parts[2]) if parts[2].isdigit() else None,
                                             "tx": parts[3], "ssid": parts[1].replace("\\:", ":")}
        try:
            d["rmem_max"] = int(open("/proc/sys/net/core/rmem_max").read())
        except OSError:
            d["rmem_max"] = None
        d["sessions"] = [line for line in run(["pgrep", "-af", r"wifi_3d_nav\.sh|wifi_nav\.sh|mapping\.sh"]).splitlines()
                         if "pgrep" not in line][:6]
        try:
            with urllib.request.urlopen("http://localhost:11434/api/tags", timeout=1) as r:
                d["ollama"] = [m["name"] for m in json.loads(r.read()).get("models", [])]
        except Exception:  # noqa: BLE001
            d["ollama"] = None
        return d


# ----------------------------------------------------------------- logs
def log_sources():
    """id -> (label, where, path). Only these can be read: no arbitrary paths."""
    src = {}
    for path in sorted(glob.glob(os.path.join(NAV2, "ws", "log", "3d", "*.log"))):
        name = os.path.basename(path)[:-4]
        src[f"3d:{name}"] = (f"3D twin: {name}", "host", path)
    frontier = os.path.join(NAV2, "ws", "log", "frontier.log")
    if os.path.exists(frontier):
        src["explorer"] = ("Explorer (frontier.log)", "host", frontier)
    runs = sorted(glob.glob(os.path.join(NAV2, "ws", "log", "mapping.*")), key=os.path.getmtime)
    if runs:
        for name in ("slam", "motion", "rviz"):
            path = os.path.join(runs[-1], f"{name}.log")
            if os.path.exists(path):
                src[f"mapping:{name}"] = (f"Session {name} ({os.path.basename(runs[-1])})", "host", path)
    for name, path in (("recovery", "/tmp/stuck_recovery.log"), ("camera_relay", "/tmp/camera_relay.log"),
                       ("floor_filter", "/tmp/floor_filter.log"), ("debug_probe", "/tmp/go2_debug_probe.err")):
        src[f"c:{name}"] = (f"go2-lidar3d: {name}", "container", path)
    for node in ("controller_server", "planner_server", "velocity_smoother", "lifecycle_manager"):
        src[f"ros:{node}"] = (f"Nav2: {node} (newest)", "container-ros", node)
    src["ros:self_filter"] = ("LiDAR self-filter (newest)", "container-ros", "go2_urdf_self_filter")
    for name, path in (("camera", "/tmp/go2_camera_wifi.log"), ("arm_bridge", "/tmp/arm_bridge_wifi.log")):
        src[f"robot:{name}"] = (f"Robot: {name}", "robot", path)
    return src


def read_log(log_id, n):
    src = log_sources().get(log_id)
    if not src:
        return None
    _, where, path = src
    n = max(20, min(int(n), 2000))
    if where == "host":
        return run(["tail", "-n", str(n), path])
    if where == "container":
        return run(["docker", "exec", CONTAINER, "tail", "-n", str(n), path])
    if where == "container-ros":
        if path.startswith("go2_"):  # a Python node: find its log by content
            script = (f"cd {ROS_LOG} 2>/dev/null && f=$(grep -l {shlex.quote(path)} $(ls -t python3_*.log 2>/dev/null "
                      f"| head -40) 2>/dev/null | head -1) && tail -n {n} \"$f\"")
        else:
            script = f"cd {ROS_LOG} 2>/dev/null && f=$(ls -t {path}_*.log 2>/dev/null | head -1) && tail -n {n} \"$f\""
        return run(["docker", "exec", CONTAINER, "bash", "-c", script]) or f"(no {path} log in {ROS_LOG})"
    if where == "robot":
        ip = robot_ip()
        if not ip:
            return "(robot IP unknown)"
        return run(SSH + [f"unitree@{ip}", f"tail -n {n} {path}"], timeout=10) or "(no answer from the robot)"
    return None


# ------------------------------------------------------------ diagnosis
def check(level, title, detail=""):
    return {"level": level, "title": title, "detail": detail}


def status_json(st, key):
    try:
        return json.loads(st["status"][key]["data"])
    except (KeyError, TypeError, ValueError):
        return None


def diagnose(probe, health):
    """Checks from the LiDAR to the legs, in order. level: ok | warn | bad | info."""
    out = []
    st = probe.get("state")
    if not st:
        out.append(check("bad", "No live data from the navigation session", probe.get("note", "")))
        sess = health.get("sessions") or []
        if not sess:
            out.append(check("info", "No wifi_nav.sh / wifi_3d_nav.sh running",
                             "Start one: ./scripts/runtime/wifi_3d_nav.sh"))
        return out
    topics, nodes = st["topics"], set(n.split("/")[-1] for n in st.get("nodes", []))

    missing = [n for n in ("controller_server", "planner_server", "velocity_smoother") if n not in nodes]
    if missing:
        out.append(check("bad", "Nav2 nodes missing: " + ", ".join(missing),
                         "Nav2 did not start or crashed: see the Nav2 logs below."))
    else:
        out.append(check("ok", "Nav2 is running", "controller, planner and velocity smoother are up"))

    lidar = topics.get("/lidar3d/obstacle_points", {})
    if lidar.get("age") is None:
        out.append(check("bad", "No LiDAR scans at all", "Nothing on /lidar3d/obstacle_points: the robot's "
                         "SLAM (slam.log) or the LiDAR self-filter is not running."))
    elif lidar["age"] > 3:
        out.append(check("bad", f"LiDAR stopped {lidar['age']:.0f} s ago",
                         "The costmap is frozen on the last scans. Usually Wi-Fi: check the robot ping below "
                         "and slam.log; the self-filter log shows 0 scans."))
    elif lidar["hz"] < 2:
        out.append(check("warn", f"LiDAR slow: {lidar['hz']:.1f} scans/s",
                         "Under 2 Hz the costmap clears slowly and the dog hesitates. Weak Wi-Fi or a busy Jetson."))
    else:
        out.append(check("ok", f"LiDAR: {lidar['hz']:.1f} scans/s"))

    js = topics.get("/joint_states", {})
    if js.get("age") is None or js["age"] > 3:
        out.append(check("warn", "No /joint_states", "The self-filter cannot remove the dog's own legs; "
                         "they may show as obstacles right around it."))

    pose = st.get("pose")
    if not pose:
        out.append(check("bad", "No position (TF lidar3d_map -> base_link)",
                         "Localization is not running or lost: nothing can navigate."))
    elif pose.get("tf_age") is not None and pose["tf_age"] > 1.0:
        out.append(check("bad", f"Position is {pose['tf_age']:.1f} s old",
                         "TF stale: a clock skew between robot and laptop, or Point-LIO stalled (slam.log)."))
    else:
        out.append(check("ok", "Localization", f"x {pose['x']:.2f}  y {pose['y']:.2f}"))

    teleop = status_json(st, "/teleop/status")
    if teleop and teleop.get("active"):
        out.append(check("warn", "Manual control is on (Drive pad)",
                         "Nav2 and the explorer are paused until you press Release on the dashboard."))

    nav, cmd = st["nav"], st["cmd"]
    follow, plan = nav.get("/follow_path", {}), nav.get("/compute_path_to_pose", {})
    explore = (st["status"].get("/explore/status") or {}).get("data", "")
    exploring = "enabled=True" in explore
    driving = follow.get("state") in ("executing", "accepted")
    if not driving and not exploring:
        out.append(check("info", "No goal: Nav2 is idle",
                         "Press Auto Explore, Tap to go, or set a 2D Goal Pose in RViz."))
    if plan.get("state") == "aborted" and (plan.get("for") or 99) < 60:
        out.append(check("bad", "Planner found no path",
                         "The goal or the robot is in an obstacle or unknown space. Pick a goal in free floor; "
                         "check the costmap picture (ghost obstacles?)."))
    if follow.get("abort_ago") is not None and follow["abort_ago"] < 30:
        out.append(check("warn", f"Controller aborted a leg {follow['abort_ago']:.0f} s ago",
                         "Usually 'collision ahead' or 'failed to make progress': see ROS errors below. "
                         "Stuck recovery should step away and retry."))

    cm = st.get("costmap")
    if cm and cm.get("under"):
        u = cm["under"]
        if u["lethal"]:
            out.append(check("bad", "The dog stands in an obstacle (costmap)",
                             f"{u['lethal']} lethal cells under the footprint. Nav2 refuses to move. Often its "
                             "own legs or a ghost point: clear the costmap or step it away by hand."))
        elif u["inscribed"]:
            out.append(check("warn", "Footprint touches inflated cells",
                             "The dog is very close to an obstacle; turning in place may be refused."))
        a = cm.get("ahead") or {}
        if a.get("lethal"):
            out.append(check("warn", "Obstacle right in front (costmap)",
                             f"{a['lethal']} lethal cells within 0.45 m ahead: Nav2 stops ('collision ahead'). "
                             "If nothing is really there, it is a LiDAR ghost; the camera check clears it with --ai."))
    elif cm is None:
        out.append(check("warn", "No local costmap", "The controller's costmap does not publish."))

    nav_cmd, out_cmd = cmd.get("/cmd_vel_nav"), cmd.get("/cmd_vel")
    moving_cmd = lambda c: c and c["age"] < 1.5 and (abs(c["vx"]) > 0.02 or abs(c["vy"]) > 0.02 or abs(c["wz"]) > 0.05)  # noqa: E731
    speed = (pose or {}).get("speed") or {}
    if driving and not moving_cmd(nav_cmd):
        out.append(check("warn", "Goal active, but the controller sends no motion",
                         "RPP is stopped: obstacle ahead, robot pose stale, or waiting for the rotation shim."))
    if moving_cmd(nav_cmd) and not moving_cmd(out_cmd):
        out.append(check("bad", "Controller commands, but nothing on /cmd_vel",
                         "The velocity smoother is not running or not active: see its log."))
    if moving_cmd(out_cmd):
        if speed and speed.get("lin", 0) < 0.03 and abs(speed.get("ang", 0)) < 0.05:
            out.append(check("bad", "Commands sent, but the dog does not move",
                             "Motion bridge (session motion.log) or the robot: is it standing, in sport mode, "
                             "not overridden by the remote / app, and not in damping? Battery?"))
        else:
            out.append(check("ok", "The dog is walking", f"{speed.get('lin', 0):.2f} m/s"))

    rec = status_json(st, "/recovery/status")
    if rec and rec.get("state") in ("stuck", "blocked"):
        out.append(check("warn", "Stuck recovery: " + rec.get("state"), rec.get("note", "")))

    bat = st.get("battery")
    if bat and bat["pct"] is not None and bat["pct"] < 20:
        out.append(check("bad" if bat["pct"] < 15 else "warn", f"Battery {bat['pct']:.0f} %",
                         "Below 15 % the battery manager docks and blocks exploring."))

    errs = [e for e in st.get("rosout", []) if e["level"] in ("ERROR", "FATAL") and time.time() - e["t"] < 60]
    if errs:
        out.append(check("warn", f"{len(errs)} ROS errors in the last minute",
                         f"latest: [{errs[-1]['node']}] {errs[-1]['msg'][:160]}"))

    ping = health.get("ping")
    if ping and ping["loss"] >= 100:
        out.append(check("bad", "The robot does not answer ping",
                         f"{health.get('robot_ip')} is off, rebooting, on another network, or got a new IP."))
    elif ping and (ping["loss"] > 0 or (ping["max_ms"] or 0) > 150):
        out.append(check("warn", f"Wi-Fi to the robot: {ping['loss']}% loss, max {ping['max_ms']:.0f} ms",
                         "Lost or late packets drop LiDAR scans and commands."))
    rmem = health.get("rmem_max")
    if rmem is not None and rmem < 8_000_000:
        out.append(check("warn", "UDP receive buffers small",
                         "Run once: printf 'net.core.rmem_max=16777216\\nnet.core.rmem_default=8388608\\n' | "
                         "sudo tee /etc/sysctl.d/60-go2-dds.conf && sudo sysctl --system"))
    return out


# ----------------------------------------------------------------- HTTP
class Handler(BaseHTTPRequestHandler):
    probe = health = None

    def log_message(self, *args):
        pass

    def send(self, code, body, ctype="application/json"):
        data = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        url = urlparse(self.path)
        q = parse_qs(url.query)
        if url.path in ("/", "/index.html"):
            try:
                return self.send(200, open(PAGE, "rb").read(), "text/html; charset=utf-8")
            except OSError:
                return self.send(500, f"missing {PAGE}", "text/plain")
        if url.path == "/api/state":
            p, h = self.probe.snapshot(), self.health.data
            return self.send(200, json.dumps({"probe": p, "health": h, "checks": diagnose(p, h)}))
        if url.path == "/api/logs":
            return self.send(200, json.dumps([{"id": k, "label": v[0], "where": v[1]}
                                              for k, v in log_sources().items()]))
        if url.path == "/api/log":
            text = read_log(q.get("id", [""])[0], q.get("n", ["300"])[0])
            if text is None:
                return self.send(404, json.dumps({"error": "unknown log"}))
            return self.send(200, json.dumps({"text": text}))
        return self.send(404, json.dumps({"error": "not found"}))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1", help="bind address (default %(default)s: this computer only)")
    ap.add_argument("--port", type=int, default=3001)
    args = ap.parse_args()
    Handler.probe, Handler.health = ProbeRunner(), Health()
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    shown = "localhost" if args.host in ("127.0.0.1", "0.0.0.0") else args.host
    print(f"Go2 debug page: http://{shown}:{args.port}   (Ctrl+C to stop)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        Handler.probe.stop()
        server.server_close()


if __name__ == "__main__":
    main()
