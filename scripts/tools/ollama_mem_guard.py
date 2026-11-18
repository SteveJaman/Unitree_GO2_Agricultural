#!/usr/bin/env python3
"""
ollama_mem_guard.py: unload the VLM before Ollama's llama-server eats the workstation's RAM.

With llava:7b fully on the GPU, llama-server's own RAM (RssAnon) still grows about 1 GB a
minute while the session sends it camera images (2026-11-18: 3.4 -> 3.9 GB in 40 s,
free RAM 7.2 -> 3.2 GB in 3 min). At ~6.8 GB the kernel killed it (2026-11-17), and
before that the desktop froze. Unloading the model frees that RAM; the VLM's next request
loads it again (7 s measured; llama-server starts at 0.2 GB again).

  ollama_mem_guard.py                      # watch until Ctrl+C
  ollama_mem_guard.py --parent PID         # stop when that process (the session) ends
  ollama_mem_guard.py --once               # one check, print it, exit

Every --period s (5): RssAnon of every llama-server / "ollama runner" process. Above
--max-gb (GO2_VLM_MAX_GB, default 4), or with less than --min-free-gb of the computer's
RAM available (GO2_VLM_MIN_FREE_GB, default 1.5) while llama-server holds over 1 GB:
POST {"model": M, "keep_alive": 0} to --url for every loaded model. Logs one line per
unload and one a minute with the current size.
Only stdlib; reads /proc (llama-server runs as the ollama user, its status is readable).
"""

import argparse
import json
import os
import sys
import time
import urllib.request

GB = 1024**3


def runner_pids():
    pids = []
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            with open(f"/proc/{d}/cmdline", "rb") as f:
                argv = f.read().split(b"\0")
        except OSError:
            continue
        exe = os.path.basename(argv[0].decode(errors="replace")) if argv and argv[0] else ""
        if exe == "llama-server" or (exe == "ollama" and len(argv) > 1 and argv[1] == b"runner"):
            pids.append(int(d))
    return pids


def rss_anon(pid):
    """Bytes of anonymous RAM (what the model's host buffers grow), 0 if gone."""
    try:
        with open(f"/proc/{pid}/status") as f:
            for line in f:
                if line.startswith("RssAnon:"):
                    return int(line.split()[1]) * 1024
    except OSError:
        pass
    return 0


def mem_available():
    with open("/proc/meminfo") as f:
        for line in f:
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) * 1024
    return 0


def http_json(url, body=None, timeout=10):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read() or b"{}")


def unload_all(base, log):
    try:
        models = [m["name"] for m in http_json(f"{base}/api/ps").get("models", [])]
    except Exception as e:  # noqa: BLE001
        log(f"cannot list models at {base}: {e}")
        return False
    for name in models:
        try:
            http_json(f"{base}/api/generate", {"model": name, "keep_alive": 0}, timeout=30)
            log(f"unloaded {name}")
        except Exception as e:  # noqa: BLE001
            log(f"unload {name} failed: {e}")
    return bool(models)


def alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def check(args, log):
    """One check; returns (bytes held by llama-server, unloaded?)."""
    held = sum(rss_anon(p) for p in runner_pids())
    avail = mem_available()
    reason = None
    if held > args.max_gb * GB:
        reason = f"llama-server holds {held / GB:.1f} GB (> {args.max_gb:g} GB)"
    elif held > GB and avail < args.min_free_gb * GB:
        reason = f"only {avail / GB:.1f} GB RAM free (< {args.min_free_gb:g} GB), llama-server {held / GB:.1f} GB"
    if reason:
        log(f"{reason}: unloading the VLM model")
        unload_all(args.url, log)
        return held, True
    return held, False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default=os.environ.get("OLLAMA_URL", "http://localhost:11434"))
    ap.add_argument("--max-gb", type=float, default=float(os.environ.get("GO2_VLM_MAX_GB", "4")))
    ap.add_argument("--min-free-gb", type=float, default=float(os.environ.get("GO2_VLM_MIN_FREE_GB", "1.5")))
    ap.add_argument("--period", type=float, default=5.0)
    ap.add_argument("--parent", type=int, default=0, help="exit when this pid ends")
    ap.add_argument("--once", action="store_true")
    args = ap.parse_args()
    args.url = args.url.split("/api/")[0].rstrip("/")

    def log(msg):
        print(f"[vlm-guard {time.strftime('%H:%M:%S')}] {msg}", flush=True)

    if args.once:
        held, _ = check(args, log)
        log(f"llama-server {held / GB:.2f} GB, RAM free {mem_available() / GB:.1f} GB")
        return 0
    log(f"watching llama-server: unload above {args.max_gb:g} GB or under {args.min_free_gb:g} GB free ({args.url})")
    last_report = 0.0
    unloads = 0
    try:
        while not args.parent or alive(args.parent):
            held, unloaded = check(args, log)
            unloads += unloaded
            if time.monotonic() - last_report > 60:
                last_report = time.monotonic()
                log(f"llama-server {held / GB:.2f} GB, RAM free {mem_available() / GB:.1f} GB, unloads {unloads}")
            if unloaded:
                time.sleep(max(args.period, 15.0))  # let it free the RAM before looking again
            else:
                time.sleep(args.period)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
