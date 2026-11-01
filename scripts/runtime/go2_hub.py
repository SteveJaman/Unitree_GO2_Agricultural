#!/usr/bin/env python3
"""
go2_hub.py: the web dashboard's backend, started by wifi_3d_nav.sh.

One address for everything, http://<this computer>:3000:

  /                the dashboard (twin/webapp), served from the moment the
                   script starts; until the go2-3d image is available, a
                   small start-up page instead
  /maps/ /config/  saved maps and config (as go2-3d's own server served them)
  /api/boot        start-up progress: stages written by wifi_3d_nav.sh
                   (--status-file) plus the live session, and every warning
                   in boot.log with the fix that follows it
  /api/state       health and the "why isn't it moving?" checks
  /api/logs, /api/log?id=   every session log (this computer, both
                   containers, the robot)
  /api/report      a Markdown debug report of all of the above
  /api/settings    GET: the tunable Nav2 values; POST {values, save}: change
                   them live (needs the session code); POST /api/settings/reset

It runs on this computer, not in a container: it reads both containers' logs
and the robot's, and sets Nav2 parameters inside go2-lidar3d through
scripts/debug/go2_param_tool.py. Health, logs and diagnosis come from
scripts/debug/debug_dashboard.py, which still runs on its own as well.

Changes need the session code (--code, printed by wifi_3d_nav.sh; the
dashboard takes it from #code= in its address): anyone who can open the page
can look, only someone with the code can change how the dog drives.
"""

import argparse
import datetime
import hmac
import json
import mimetypes
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from html import escape
from urllib.parse import parse_qs, quote, unquote, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "scripts", "debug"))
import debug_dashboard as dd  # noqa: E402  (health, logs, diagnosis)

PARAM_TOOL = os.path.join(REPO, "scripts", "debug", "go2_param_tool.py")
LOCAL, GLOBAL = "/local_costmap/local_costmap", "/global_costmap/global_costmap"
CTRL, SMOOTH = "/controller_server", "/velocity_smoother"

# The settings the dashboard may change: whitelisted Nav2 parameters that take
# effect at runtime (checked in the sim), each with a safe range.
SETTINGS = [
    {"key": "inflation", "name": "Inflation radius", "unit": "m", "min": 0.30, "max": 1.00, "step": 0.05,
     "params": [(LOCAL, "inflation_layer.inflation_radius"), (GLOBAL, "inflation_layer.inflation_radius")],
     "help": "How far Nav2 keeps the dog from obstacles (both costmaps)."},
    {"key": "padding", "name": "Footprint padding", "unit": "m", "min": 0.00, "max": 0.10, "step": 0.01,
     "params": [(LOCAL, "footprint_padding"), (GLOBAL, "footprint_padding")],
     "help": "Extra margin around the dog's outline."},
    {"key": "speed", "name": "Driving speed", "unit": "m/s", "min": 0.10, "max": 0.40, "step": 0.05,
     "params": [(CTRL, "FollowPath.desired_linear_vel")], "smoother": 0,
     "help": "Cruising speed; the velocity smoother's forward limit follows it."},
    {"key": "turn", "name": "Turn speed", "unit": "rad/s", "min": 0.30, "max": 1.00, "step": 0.05,
     "params": [(CTRL, "FollowPath.rotate_to_heading_angular_vel")], "smoother": 2,
     "help": "Turning on the spot before a path; the smoother's turn limit is raised if needed."},
    {"key": "lookahead", "name": "Stop-for-obstacle look-ahead", "unit": "s", "min": 0.5, "max": 3.0, "step": 0.1,
     "params": [(CTRL, "FollowPath.max_allowed_time_to_collision_up_to_carrot")],
     "help": "How far ahead (in seconds of driving) a collision stops the dog."},
]
PRESETS = {"safe": {"inflation": 0.70, "padding": 0.05}, "wide": {"inflation": 0.85, "padding": 0.05},
           "tight": {"inflation": 0.45, "padding": 0.02}}
STAGES = [("checks", "Checks"), ("image", "3D image"), ("session", "Navigation session"),
          ("position", "Robot position"), ("twin", "3D twin")]

# boot.log lines worth showing (the same idea as the terminal filter), and
# ROS launch output, which is shown only for errors.
SHOW = re.compile(r"WARN|ERROR|FAIL|PROBLEM|TOO LARGE|MISSING|NOT SET UP|Traceback")
ROS_LINE = re.compile(r"^\s*(\[[\w./-]+-\d+\]|\d{9,}\.\d+ \[\d+\]|\[(INFO|WARN|ERROR|FATAL|DEBUG)\])")
FOLLOW = re.compile(r"^\s{4,}\S|^\s*(Fix|Install|Run|Cause)\s*:")


def now_s():
    return datetime.datetime.now().strftime("%H:%M:%S")


class Hub:
    def __init__(self, args):
        self.args = args
        self.probe, self.health = dd.ProbeRunner(), dd.Health()
        self.lock = threading.Lock()
        self.history = []          # settings changes, newest first
        self.defaults = None       # values at session start (first read)
        self.default_smoother = None
        self.cached, self.cached_t = None, 0.0   # last read (a read takes a few seconds)
        self.raw = {}                            # its raw "/node|param" values
        self.env, self.env_t = None, 0.0         # session DDS env (finding it takes ~1 s)
        self.applied_saved = False
        self.webroot_cache = None
        threading.Thread(target=self.webroot_loop, daemon=True).start()
        threading.Thread(target=self.saved_loop, daemon=True).start()

    # ------------------------------------------------------------ web app
    def webroot_loop(self):
        """Copy the dashboard build out of the go2-3d image (once per image)."""
        while True:
            img = dd.run(["docker", "image", "inspect", "-f", "{{.Id}}", self.args.image]).strip()
            if img:
                dest = os.path.join(self.args.cache, img.split(":")[-1][:16])
                if not os.path.isfile(os.path.join(dest, "index.html")):
                    tmp = dest + ".tmp"
                    shutil.rmtree(tmp, ignore_errors=True)
                    cid = dd.run(["docker", "create", self.args.image, "true"]).strip()
                    if cid:
                        dd.run(["docker", "cp", f"{cid}:/opt/webapp", tmp], timeout=60)
                        dd.run(["docker", "rm", cid])
                    if os.path.isfile(os.path.join(tmp, "index.html")):
                        shutil.rmtree(dest, ignore_errors=True)
                        os.replace(tmp, dest)
                if os.path.isfile(os.path.join(dest, "index.html")):
                    self.webroot_cache = dest
            time.sleep(5)

    def webroot(self):
        """twin/webapp/dist when built on this computer and newer, else the image's build."""
        dist = os.path.join(REPO, "twin", "webapp", "dist")
        img = self.webroot_cache
        di, ii = os.path.join(dist, "index.html"), img and os.path.join(img, "index.html")
        if os.path.isfile(di) and (not ii or os.path.getmtime(di) > os.path.getmtime(ii)):
            return dist
        return img

    # --------------------------------------------------------------- boot
    def stages_from_file(self):
        st = {}
        try:
            with open(self.args.status_file) as f:
                for line in f:
                    try:
                        e = json.loads(line)
                        st[e["stage"]] = e
                    except (ValueError, KeyError):
                        pass
        except OSError:
            pass
        return st

    def boot_warnings(self, limit=15):
        try:
            with open(self.args.boot_log, errors="replace") as f:
                lines = f.read().splitlines()[-4000:]
        except OSError:
            return []
        out, cur, seen = [], None, set()
        for raw in lines:
            line = re.sub(r"\x1b\[[0-9;]*m", "", raw).rstrip()
            if cur is not None and FOLLOW.match(line) and not ROS_LINE.match(line):
                cur["more"].append(line.strip())
                continue
            cur = None
            if not line.strip():
                continue
            if ROS_LINE.match(line) and not re.search(r"ERROR|FATAL", line):
                continue
            if SHOW.search(line):
                key = re.sub(r"[\d.]+", "#", line)
                if key in seen:
                    continue
                seen.add(key)
                cur = {"text": line.strip(), "more": []}
                out.append(cur)
        return out[-limit:]

    def boot(self):
        filed = self.stages_from_file()
        checks = dd.diagnose(self.probe.snapshot(), self.health.data)
        titles = {c["title"]: c for c in checks}
        stages = []
        for key, name in STAGES:
            e = dict(filed.get(key) or {})
            state, detail = e.get("state", "wait"), e.get("detail", "")
            if key == "session":
                nav = titles.get("Nav2 is running")
                if nav:
                    state, detail = "ok", "Nav2 planner, controller and smoother up"
                elif any(c["level"] == "bad" and c["title"].startswith("Nav2 nodes missing") for c in checks):
                    state = "run"
            if key == "position":
                loc = next((c for c in checks if c["title"] == "Localization"), None)
                if loc:
                    state, detail = "ok", loc["detail"]
                elif filed.get("session"):
                    state, detail = "run", "waiting for the dog's position"
            stages.append({"key": key, "name": name, "state": state, "detail": detail})
        ready = all(s["state"] in ("ok", "warn") for s in stages)
        failed = [s for s in stages if s["state"] == "fail"]
        return {"stages": stages, "ready": ready, "failed": bool(failed),
                "warnings": self.boot_warnings(), "webapp": bool(self.webroot()),
                "started": filed.get("checks", {}).get("t"), "code_required": bool(self.args.code)}

    # ----------------------------------------------------------- settings
    def param_tool(self, req):
        if self.env is None or time.time() - self.env_t > 60:
            self.env, self.env_t = self.probe.session_env(), time.time()
        if self.env is None:
            return None, "no navigation session yet"
        cmd = ["docker", "exec", "-i"]
        for k, v in self.env.items():
            cmd += ["-e", f"{k}={v}"]
        cmd += [dd.CONTAINER, "bash", "-c",
                "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash 2>/dev/null; "
                "exec python3 - \"$0\"", json.dumps(req)]
        try:   # the script goes in on stdin: no docker cp per call
            with open(PARAM_TOOL) as f:
                out = subprocess.run(cmd, stdin=f, capture_output=True, text=True, timeout=30).stdout
        except (subprocess.TimeoutExpired, OSError):
            out = ""
        for line in reversed(out.splitlines()):
            if line.startswith("{"):
                try:
                    return json.loads(line), None
                except ValueError:
                    break
        self.env = None   # a new session may have started: look again next time
        return None, "the parameter tool gave no answer (Nav2 busy or not up)"

    def read_settings(self):
        req = {"get": [list(p) for s in SETTINGS for p in s["params"]]
               + [[SMOOTH, "max_velocity"], [SMOOTH, "min_velocity"]]}
        res, err = self.param_tool(req)
        if res is None:
            return None, err
        self.raw = dict(res["values"])
        return self.from_raw(self.raw, res["errors"]), None

    def from_raw(self, vals, errors):
        """The settings view of raw {"/node|param": value}; remembers it as the cache."""
        out = {}
        for s in SETTINGS:
            v = next((vals.get(f"{n}|{p}") for n, p in s["params"] if f"{n}|{p}" in vals), None)
            out[s["key"]] = round(v, 3) if isinstance(v, (int, float)) else None
        out["_smoother"] = {"max": vals.get(f"{SMOOTH}|max_velocity"), "min": vals.get(f"{SMOOTH}|min_velocity")}
        out["_errors"] = errors
        if self.defaults is None and all(out[s["key"]] is not None for s in SETTINGS[:2]):
            self.defaults = {s["key"]: out[s["key"]] for s in SETTINGS if out[s["key"]] is not None}
            self.default_smoother = out["_smoother"]
        self.cached, self.cached_t = out, time.time()
        return out

    def settings_now(self, max_age=45):
        """The last read if recent (the page loads at once), else a fresh one."""
        if self.cached is not None and time.time() - self.cached_t < max_age:
            return self.cached, None
        return self.read_settings()

    def apply_settings(self, values, why, restore_smoother=False):
        # One parameter-tool run: the recent read for "before", the tool's own
        # read-back after the change for "after" (each run takes a few seconds).
        cur, err = self.settings_now(max_age=15)
        if cur is None:
            return None, err
        sets, changed = [], []
        smax = list(cur["_smoother"]["max"] or [])
        smin = list(cur["_smoother"]["min"] or [])
        if restore_smoother and self.default_smoother and self.default_smoother.get("max"):
            # a reset also undoes a raised turn limit
            smax, smin = list(self.default_smoother["max"]), list(self.default_smoother["min"] or smin)
        for s in SETTINGS:
            if s["key"] not in values:
                continue
            try:
                v = float(values[s["key"]])
            except (TypeError, ValueError):
                return None, f"{s['name']}: not a number"
            if not s["min"] - 1e-9 <= v <= s["max"] + 1e-9:
                return None, f"{s['name']} must be {s['min']}–{s['max']} {s['unit']}"
            if cur.get(s["key"]) is not None and abs(cur[s["key"]] - v) < 1e-6:
                continue
            sets += [[n, p, v] for n, p in s["params"]]
            changed.append(f"{s['name']} {v:g} {s['unit']}")
            i = s.get("smoother")
            if i is not None and len(smax) == 3 and len(smin) == 3:
                if i == 0:
                    smax[0] = v
                elif v > smax[2]:
                    smax[2], smin[2] = v, -v
        smoother_changed = len(smax) == 3 and smax != cur["_smoother"]["max"]
        if not sets and not smoother_changed:
            return cur, None
        if not changed:
            changed.append("velocity smoother limits")
        if smoother_changed:
            sets += [[SMOOTH, "max_velocity", smax], [SMOOTH, "min_velocity", smin]]
        res, err = self.param_tool({"set": sets})
        if res is None:
            return None, err
        failed = sorted({k.split("|")[1] for k in res["errors"]})
        msg = f"{now_s()} {why}: " + ", ".join(changed)
        if failed:
            msg += f" (not applied: {', '.join(failed)})"
        self.history = ([msg] + self.history)[:20]
        self.raw = {**self.raw, **res["values"]}   # what Nav2 reports after the change
        out = self.from_raw(self.raw, res["errors"])
        out["_set_errors"] = res["errors"]
        return out, None

    def saved_path(self):
        return os.path.join(self.args.config_dir, "dashboard_settings.json")

    def saved(self):
        try:
            with open(self.saved_path()) as f:
                return json.load(f).get("values") or {}
        except (OSError, ValueError):
            return {}

    def save(self, values):
        tmp = self.saved_path() + ".tmp"
        with open(tmp, "w") as f:
            json.dump({"values": values, "saved": datetime.datetime.now().isoformat(timespec="seconds")}, f, indent=2)
        os.replace(tmp, self.saved_path())

    def saved_loop(self):
        """Apply config/dashboard_settings.json once Nav2 is up."""
        while not self.applied_saved:
            time.sleep(3)
            snap = self.probe.snapshot()
            if not snap.get("state"):
                continue
            nodes = {n.split("/")[-1] for n in snap["state"].get("nodes", [])}
            if not {"controller_server", "velocity_smoother"} <= nodes:
                continue
            time.sleep(5)  # costmaps finish activating after the controller
            saved = self.saved()
            with self.lock:
                if saved:
                    _, err = self.apply_settings(saved, "your saved defaults")
                    if err:
                        self.history = ([f"{now_s()} saved defaults not applied: {err}"] + self.history)[:20]
                        continue
                else:
                    self.read_settings()   # remember the session's starting values
            self.applied_saved = True
        while True:                        # keep the page's copy fresh (tune_costmap.sh, rqt …)
            time.sleep(30)
            with self.lock:
                self.read_settings()

    def settings_json(self):
        with self.lock:
            cur, err = self.settings_now()
        return {"defs": SETTINGS_PUBLIC, "presets": PRESETS, "values": cur and {k: v for k, v in cur.items() if not k.startswith("_")},
                "errors": cur and cur["_errors"], "defaults": self.defaults, "saved": self.saved(),
                "history": self.history, "error": err, "code_required": bool(self.args.code)}

    # ------------------------------------------------------------- report
    def report(self):
        p, h = self.probe.snapshot(), self.health.data
        checks = dd.diagnose(p, h)
        st = p.get("state") or {}
        git = dd.run(["git", "-C", REPO, "rev-parse", "--abbrev-ref", "HEAD"]).strip()
        sha = dd.run(["git", "-C", REPO, "rev-parse", "--short", "HEAD"]).strip()
        lines = ["# Go2 debug report",
                 f"{datetime.datetime.now():%Y-%m-%d %H:%M} · {git or '?'} @ {sha or '?'} · drive {os.environ.get('GO2_DRIVE', '?')}",
                 "", "## Checks"]
        for c in checks:
            mark = {"ok": "OK     ", "warn": "CHECK  ", "bad": "BLOCKED", "info": "INFO   "}.get(c["level"], c["level"])
            lines.append(f"- {mark} {c['title']}" + (f": {c['detail']}" if c.get("detail") else ""))
        b = self.boot()
        lines += ["", "## Start-up"] + [f"- {s['name']}: {s['state']} {s['detail']}".rstrip() for s in b["stages"]]
        if b["warnings"]:
            lines += ["", "## Start-up warnings"]
            for w in b["warnings"]:
                lines.append(f"- {w['text']}")
                lines += [f"  {m}" for m in w["more"]]
        topics = st.get("topics") or {}
        if topics:
            lines += ["", "## Topics", "| topic | rate | age |", "|---|---|---|"]
            for t, v in topics.items():
                age = "never" if v.get("age") is None else f"{v['age']} s"
                lines.append(f"| {t} | {v.get('hz', 0)} Hz | {age} |")
        ping, wifi = h.get("ping") or {}, h.get("wifi") or {}
        lines += ["", "## System",
                  f"- robot {h.get('robot_ip') or '?'}: ping {ping.get('avg_ms', '?')} ms avg, {ping.get('loss', '?')}% lost",
                  f"- Wi-Fi: {wifi.get('ssid', '?')} {wifi.get('signal_dbm', wifi.get('signal_pct', '?'))}",
                  "- containers: " + ", ".join(f"{c['name']} ({c['status']})" for c in h.get("containers", [])),
                  f"- Ollama models: {h.get('ollama')}"]
        cur = self.cached   # the last read: a report should not wait for Nav2
        if cur:
            lines += ["", "## Settings", "- " + " · ".join(
                f"{s['name']} {cur.get(s['key'])} {s['unit']}" for s in SETTINGS)]
        errs = [e for e in st.get("rosout", []) if e.get("level") in ("ERROR", "FATAL", "WARN")][-12:]
        if errs:
            lines += ["", "## Recent ROS warnings and errors"] + [f"- [{e['node']}] {e['level']}: {e['msg'][:200]}" for e in errs]
        return "\n".join(lines) + "\n"


SETTINGS_PUBLIC = [{k: v for k, v in s.items() if k not in ("params",)} | {"params": [p for _, p in s["params"]]}
                   for s in SETTINGS]

BOOT_PAGE = """<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Go2 Hub: starting</title><style>
body{margin:0;background:#05070b;color:#f1f5f9;font:15px/1.5 system-ui,sans-serif;display:grid;place-items:center;min-height:100vh}
.card{width:min(520px,92vw);padding:24px;border-radius:20px;background:#0e131c;border:1px solid #ffffff29}
h1{font-size:19px;margin:0 0 14px}.s{display:flex;gap:10px;margin:8px 0}.m{width:22px;text-align:center}
.d{color:#94a3b8;font-size:13px}.w{margin-top:12px;padding:10px;border-radius:12px;background:#f5a5241a;border:1px solid #f5a52466;font-size:13px}
</style></head><body><div class="card"><h1>Starting the robot stack</h1><div id="st">…</div><div id="wa"></div>
<p class="d">The dashboard opens here by itself once its image is ready (first run: it is being built).</p></div>
<script>
const M={ok:'✓',warn:'!',fail:'✕',run:'…',wait:'·'};
async function tick(){try{const b=await (await fetch('/api/boot')).json();
if(b.webapp){location.reload();return}
document.getElementById('st').innerHTML=b.stages.map(s=>`<div class="s"><span class="m">${M[s.state]||'·'}</span><span>${s.name}<div class="d">${s.detail||s.state}</div></span></div>`).join('');
document.getElementById('wa').innerHTML=b.warnings.map(w=>`<div class="w">${w.text.replace(/</g,'&lt;')}${w.more.map(m=>'<br>'+m.replace(/</g,'&lt;')).join('')}</div>`).join('');
}catch(e){}setTimeout(tick,1500)}tick();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    hub = None

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

    def json(self, obj, code=200):
        return self.send(code, json.dumps(obj))

    def authorized(self):
        code = self.hub.args.code
        return not code or hmac.compare_digest(self.headers.get("X-Go2-Code", ""), code)

    def static(self, root, rel, listing=False):
        root = os.path.realpath(root)
        path = os.path.realpath(os.path.join(root, rel.lstrip("/")))
        if not (path == root or path.startswith(root + os.sep)):
            return self.send(403, "forbidden", "text/plain")
        if os.path.isdir(path) and listing:
            # Like python3 -m http.server: the dashboard reads maps/ (meshes)
            # and config/ (which files exist) from this listing.
            names = sorted(os.listdir(path))
            links = "".join(f'<li><a href="{quote(n)}">{escape(n)}</a></li>\n' for n in names)
            return self.send(200, f"<!DOCTYPE html><html><body><ul>\n{links}</ul></body></html>",
                             "text/html; charset=utf-8")
        if os.path.isdir(path):
            path = os.path.join(path, "index.html")
        if not os.path.isfile(path):
            return self.send(404, "not found", "text/plain")
        ctype = mimetypes.guess_type(path)[0] or "application/octet-stream"
        with open(path, "rb") as f:
            return self.send(200, f.read(), ctype)

    def do_GET(self):
        url = urlparse(self.path)
        q, path = parse_qs(url.query), unquote(url.path)
        h = self.hub
        if path == "/api/boot":
            return self.json(h.boot())
        if path == "/api/state":
            p, hh = h.probe.snapshot(), h.health.data
            return self.json({"probe": p, "health": hh, "checks": dd.diagnose(p, hh)})
        if path == "/api/logs":
            return self.json([{"id": k, "label": v[0], "where": v[1]} for k, v in dd.log_sources().items()])
        if path == "/api/log":
            text = dd.read_log(q.get("id", [""])[0], q.get("n", ["300"])[0])
            return self.json({"text": text}) if text is not None else self.json({"error": "unknown log"}, 404)
        if path == "/api/report":
            return self.send(200, h.report(), "text/markdown; charset=utf-8")
        if path == "/api/settings":
            return self.json(h.settings_json())
        if path == "/api/auth":
            return self.json({"code_required": bool(h.args.code), "ok": self.authorized()})
        if path.startswith("/api/"):
            return self.json({"error": "not found"}, 404)
        if path.startswith("/maps/"):
            return self.static(h.args.db_dir, path[len("/maps/"):], listing=True)
        if path.startswith("/config/"):
            return self.static(h.args.config_dir, path[len("/config/"):], listing=True)
        root = h.webroot()
        if not root:
            return self.send(200, BOOT_PAGE, "text/html; charset=utf-8")
        return self.static(root, path)

    def do_POST(self):
        path = urlparse(self.path).path
        if not path.startswith("/api/settings"):
            return self.json({"error": "not found"}, 404)
        if not self.authorized():
            return self.json({"error": "the session code is needed to change settings"}, 403)
        try:
            body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0) or 0)) or b"{}")
        except ValueError:
            return self.json({"error": "bad request"}, 400)
        h = self.hub
        with h.lock:
            if path == "/api/settings/reset":
                if not h.defaults:
                    return self.json({"error": "session defaults not known yet"}, 409)
                values, why, restore = dict(h.defaults), "reset to the session defaults", True
            else:
                values, why, restore = body.get("values") or {}, "applied", False
            out, err = h.apply_settings(values, why, restore_smoother=restore)
            if out is not None and body.get("save"):
                h.save({k: out[k] for k in (s["key"] for s in SETTINGS) if out.get(k) is not None})
                h.history = ([f"{now_s()} saved as your defaults (config/dashboard_settings.json)"] + h.history)[:20]
        if out is None:
            return self.json({"error": err}, 503)
        return self.json(h.settings_json() | {"set_errors": out.get("_set_errors", {})})


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=3000)
    ap.add_argument("--code", default="", help="session code needed to change settings ('' = none needed)")
    ap.add_argument("--image", default="go2-3d:local")
    ap.add_argument("--db-dir", default=os.path.join(dd.NAV2, "ws", "maps", "rtabmap"))
    ap.add_argument("--config-dir", default=os.path.join(REPO, "config"))
    ap.add_argument("--boot-log", default=os.path.join(dd.NAV2, "ws", "log", "3d", "boot.log"))
    ap.add_argument("--status-file", default=os.path.join(dd.NAV2, "ws", "log", "3d", "boot_status.jsonl"))
    ap.add_argument("--cache", default=os.path.expanduser("~/.cache/go2_hub"))
    args = ap.parse_args()
    os.makedirs(args.cache, exist_ok=True)

    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)   # wifi_3d_nav.sh stops the hub with SIGTERM
    Handler.hub = Hub(args)
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"[hub] serving http://{args.host}:{args.port} (settings {'need the session code' if args.code else 'open'})", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        Handler.hub.probe.stop()
        server.server_close()


if __name__ == "__main__":
    main()
