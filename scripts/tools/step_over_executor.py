#!/usr/bin/env python3
"""
step_over_executor.py: runs the step-over crawl on the dog when the session asks for it
(wifi_3d_nav.sh --semantic --step-over-crawl crawl). Runs on the laptop, no ROS: the
laptop has the passwordless SSH to the dog that the containers do not.

scripts/nodes/step_over_coordinator.py (in go2-lidar3d) writes
REQUEST_DIR/request_<id>.json after lining the dog up; this runs

  ~/go2-agri-lowlevel/crawl.sh --step-over --object-height H --object-depth D
      --object-distance GAP --resume-session --yes --log ~/go2-agri-lowlevel/nav_<id>.csv

on the dog and writes REQUEST_DIR/result_<id>.json: {"ok", "returncode", "summary",
"tail"}. ok only when the crawl printed "crawl done" and nothing stopped it. A request
older than --max-age s is not run (a stale file from an earlier session).

  step_over_executor.py REQUEST_DIR [--robot unitree@IP] [--dry]
"""

import argparse
import glob
import json
import os
import subprocess
import sys
import time

DEFAULT_IP_FILE = os.path.expanduser("~/.cache/go2_wifi_nav/robot_ip")


def robot_target(arg):
    if arg:
        return arg
    try:
        ip = open(DEFAULT_IP_FILE).read().strip()
    except OSError:
        ip = ""
    return f"unitree@{ip}" if ip else ""


def crawl_command(req):
    """The command run on the dog for a request (values range-checked)."""
    h, d, gap = float(req["height"]), float(req["depth"]), float(req["gap"])
    if not (0.02 <= h <= 0.15 and 0.01 <= d <= 0.15 and 0.0 <= gap <= 0.30):
        raise ValueError(f"out of range: height {h}, depth {d}, gap {gap}")
    rid = "".join(c for c in str(req["id"]) if c.isalnum())
    return (f"~/go2-agri-lowlevel/crawl.sh --step-over --object-height {h:.3f} --object-depth {d:.3f} "
            f"--object-distance {gap:.3f} --resume-session --yes --log ~/go2-agri-lowlevel/nav_{rid}.csv")


def judge(returncode, output):
    """(ok, summary) from the crawl's exit code and output."""
    lines = [ln for ln in output.splitlines() if ln.startswith("[crawl]")]
    stop = [ln for ln in lines if "STOP" in ln or "not started" in ln or "problems" in ln and "problems: 0" not in ln]
    done = any("crawl done" in ln for ln in lines)
    ok = returncode == 0 and done and not any("STOP" in ln for ln in lines)
    summary = (stop[0] if stop else next((ln for ln in lines if "crawl done" in ln), lines[-1] if lines else ""))
    return ok, summary.replace("[crawl] ", "")


def handle(path, target, dry, max_age):
    req = json.load(open(path))
    rid = req["id"]
    out_path = os.path.join(os.path.dirname(path), f"result_{rid}.json")
    os.replace(path, path + ".taken")                  # never run a request twice
    if time.time() - float(req.get("t", 0)) > max_age:
        res = {"ok": False, "returncode": None, "summary": f"request older than {max_age:.0f} s: not run"}
    else:
        try:
            cmd = crawl_command(req)
        except (KeyError, ValueError) as ex:
            cmd, res = None, {"ok": False, "returncode": None, "summary": f"bad request: {ex}"}
        if cmd and dry:
            print(f"[executor] DRY: would run on {target}: {cmd}", flush=True)
            res = {"ok": True, "returncode": 0, "summary": "dry run: nothing run on the dog"}
        elif cmd:
            print(f"[executor] {req.get('object')}: running the crawl on {target}", flush=True)
            p = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", target, cmd],
                               capture_output=True, text=True, timeout=900)
            output = p.stdout + p.stderr
            ok, summary = judge(p.returncode, output)
            res = {"ok": ok, "returncode": p.returncode, "summary": summary,
                   "tail": output.splitlines()[-25:]}
    with open(out_path + ".tmp", "w") as f:
        json.dump(res, f)
    os.replace(out_path + ".tmp", out_path)
    print(f"[executor] result {rid}: {'OK' if res['ok'] else 'FAILED'}: {res['summary']}", flush=True)
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("request_dir")
    ap.add_argument("--robot", default="", help="user@ip (default: the session's cached robot IP)")
    ap.add_argument("--dry", action="store_true", help="answer requests without running anything")
    ap.add_argument("--max-age", type=float, default=30.0)
    a = ap.parse_args()
    os.makedirs(a.request_dir, exist_ok=True)
    target = robot_target(a.robot)
    if not target and not a.dry:
        sys.exit("[executor] robot IP unknown (no ~/.cache/go2_wifi_nav/robot_ip): --robot unitree@IP")
    print(f"[executor] watching {a.request_dir} for step-over requests ({'dry' if a.dry else target})", flush=True)
    while True:
        for path in sorted(glob.glob(os.path.join(a.request_dir, "request_*.json"))):
            try:
                handle(path, target, a.dry, a.max_age)
            except Exception as ex:  # noqa: BLE001 - keep serving; the coordinator times out
                print(f"[executor] {path}: {ex}", flush=True)
        time.sleep(0.5)


if __name__ == "__main__":
    main()
