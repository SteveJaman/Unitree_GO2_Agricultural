#!/usr/bin/env python3
"""
session_rviz_respawn.py: an RViz crash no longer ends the navigation session.

go2-nav2-wifi's ws/scripts/go2_session.py supervises the session's processes and
stops all of them when any one exits. RViz is one of them, and RViz 2 segfaults
now and then (5 of 109 recorded sessions, again 2026-11-11 with --semantic
--voxel): Nav2, SLAM and the motion link went down with it. This edit restarts
RViz when it dies abnormally (a signal or a non-zero exit), up to 5 times per
session; closing the RViz window normally (exit 0) still ends the session, as
before. Every other process keeps its old behaviour.

  session_rviz_respawn.py NAV2_WIFI_DIR          # apply (idempotent)
  session_rviz_respawn.py --check NAV2_WIFI_DIR

Edits tagged with TAG; the original is kept as go2_session.py.pre-rviz-respawn.
Run by wifi_nav.sh before every session, like mapping_stop_timeouts.py.
"""

import os
import shutil
import sys

TAG = "  # go2-agri: rviz respawn"
FILE = os.path.join("ws", "scripts", "go2_session.py")
MAX_RESTARTS = 5

EDITS = [
    (
        "        print(f'{mode}: {state}; use go2-session.sh stop {mode} or Ctrl+C', flush=True)\n"
        "        while not stopping:\n",
        "        print(f'{mode}: {state}; use go2-session.sh stop {mode} or Ctrl+C', flush=True)\n"
        "        rviz_restarts = 0" + TAG + "\n"
        "        while not stopping:\n",
    ),
    (
        "                print(f'{mode}: child exited; stopping owned session', flush=True)\n",
        "                crashed = not (info.si_code == os.CLD_EXITED and info.si_status == 0)" + TAG + "\n"
        "                if child.args == normal_exit_command and crashed and rviz_restarts < "
        + str(MAX_RESTARTS) + ":" + TAG + "\n"
        "                    child.wait()" + TAG + "\n"
        "                    children.remove(child)" + TAG + "\n"
        "                    rviz_restarts += 1" + TAG + "\n"
        "                    print(f'{mode}: RViz crashed (code {info.si_status}); restarting it "
        "({rviz_restarts}/" + str(MAX_RESTARTS) + "), the session keeps running', flush=True)" + TAG + "\n"
        "                    children.append(subprocess.Popen(child.args, start_new_session=True))" + TAG + "\n"
        "                    continue" + TAG + "\n"
        "                print(f'{mode}: child exited; stopping owned session', flush=True)\n",
    ),
]


def main():
    args = sys.argv[1:]
    check = "--check" in args
    args = [a for a in args if a != "--check"]
    if len(args) != 1:
        sys.exit(__doc__)
    path = os.path.join(args[0], FILE)
    try:
        text = open(path).read()
    except OSError as e:
        print(f"[rviz] WARN: {path}: {e}; RViz respawn not applied")
        return 1
    todo = [(a, b) for a, b in EDITS if b not in text]
    if not todo:
        print("[rviz] go2_session.py RViz respawn ........ OK")
        return 0
    missing = [a for a, _ in todo if text.count(a) != 1]
    if missing:
        print("[rviz] WARN: go2_session.py changed upstream; RViz respawn not applied for:")
        for a in missing:
            print("         " + a.strip().splitlines()[-1][:90])
        return 1
    if check:
        print(f"[rviz] go2_session.py RViz respawn ........ MISSING ({len(todo)} edits)")
        return 1
    backup = path + ".pre-rviz-respawn"
    if not os.path.exists(backup):
        shutil.copy2(path, backup)
    for a, b in todo:
        text = text.replace(a, b)
    compile(text, path, "exec")  # never write a file that does not parse
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(text)
    shutil.copymode(path, tmp)
    os.replace(tmp, path)
    print(f"[rviz] go2_session.py: RViz respawn added ({len(todo)} edits; original {os.path.basename(backup)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
