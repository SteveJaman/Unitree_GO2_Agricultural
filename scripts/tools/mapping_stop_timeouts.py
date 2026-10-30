#!/usr/bin/env python3
"""
mapping_stop_timeouts.py: make go2-nav2-wifi's mapping.sh stop on Ctrl+C even
when the Wi-Fi link to the robot has stalled.

mapping.sh opens one shared SSH connection (ControlMaster) to the robot and
sends every command through it. Its cleanup ignores Ctrl+C on purpose (the
Jetson must finish saving the map), then asks the robot to stop over that
connection. The master was opened without keep-alives: after a short Wi-Fi
stall it is dead but never notices, every command through it waits forever,
and Ctrl+C does nothing (seen 2026-10-30: cleanup stuck > 4 min, robot fine).

  mapping_stop_timeouts.py NAV2_WIFI_DIR   # apply (idempotent)
  mapping_stop_timeouts.py --check NAV2_WIFI_DIR

Edits (each tagged with TAG, original kept as mapping.sh.pre-timeouts):
  * the master: ServerAliveInterval=5, ServerAliveCountMax=3 (a dead link
    fails in ~15 s, and so does every command through it);
  * cleanup: a message saying what it waits for, and time limits on the
    local session stop (60 s), the Jetson session stop and map save
    (GO2_STOP_TIMEOUT, 120 s), the result transfer (GO2_FETCH_TIMEOUT,
    600 s) and closing the master (10 s). A transfer that times out leaves
    the results on the Jetson, as mapping.sh already reports.
Run by wifi_nav.sh before every session.
"""

import os
import shutil
import sys

TAG = "  # go2-agri: stop timeouts"
FILE = "mapping.sh"

EDITS = [
    (
        'ssh -o ConnectTimeout=8 -o ControlMaster=yes -o ControlPersist=60 -S "$ssh_socket" "$target" true',
        'ssh -o ConnectTimeout=8 -o ControlMaster=yes -o ControlPersist=60 '
        '-o ServerAliveInterval=5 -o ServerAliveCountMax=3 -S "$ssh_socket" "$target" true' + TAG,
    ),
    (
        '    docker exec "$GO2_CONTAINER" bash /ws/scripts/go2-session.sh stop "$session_mode" --owner "$owner" >/dev/null 2>&1 || true',
        '    echo "Stopping the local Nav2/RViz session (up to 60 s)..." >&2' + TAG + '\n'
        '    timeout -k 5 60 docker exec "$GO2_CONTAINER" bash /ws/scripts/go2-session.sh stop "$session_mode" --owner "$owner" >/dev/null 2>&1 || true' + TAG,
    ),
    (
        '    ssh -o BatchMode=yes -S "$ssh_socket" "$target" "docker exec go2-lidar3d-onboard bash /ws/scripts/go2-session.sh stop lidar3d --owner $owner" >/dev/null 2>&1 || true',
        '    echo "Stopping SLAM on the Jetson and saving its map (up to ${GO2_STOP_TIMEOUT:-120} s; Ctrl+C is ignored until then)..." >&2' + TAG + '\n'
        '    timeout -k 5 "${GO2_STOP_TIMEOUT:-120}" ssh -o BatchMode=yes -S "$ssh_socket" "$target" "docker exec go2-lidar3d-onboard bash /ws/scripts/go2-session.sh stop lidar3d --owner $owner" >/dev/null 2>&1 '
        '|| echo "Jetson stop did not finish (link down?): run ./scripts/runtime/wifi_nav.sh --clean before the next session" >&2' + TAG,
    ),
    (
        '      python3 "$MAPPING_ROOT/ws/scripts/fetch-lidar3d.py" --target "$target" --ssh-socket "$ssh_socket" \\',
        # A comment after a trailing backslash would end the command: tag the line above.
        '     ' + TAG + '\n'
        '      timeout -k 5 "${GO2_FETCH_TIMEOUT:-600}" python3 "$MAPPING_ROOT/ws/scripts/fetch-lidar3d.py" --target "$target" --ssh-socket "$ssh_socket" \\',
    ),
    (
        '  ssh -S "$ssh_socket" -O exit "$target" >/dev/null 2>&1 || true',
        '  timeout -k 2 10 ssh -S "$ssh_socket" -O exit "$target" >/dev/null 2>&1 || true' + TAG,
    ),
]


def main():
    args = sys.argv[1:]
    check = "--check" in args
    args = [a for a in args if a != "--check"]
    if len(args) != 1:
        sys.exit(__doc__)
    path = os.path.join(args[0], FILE)
    if not os.path.isfile(path):
        sys.exit(f"[stop] {path} not found")
    text = open(path).read()
    todo = [(a, b) for a, b in EDITS if b not in text]
    if not todo:
        print("[stop] mapping.sh stop timeouts ........... OK")
        return 0
    missing = [a for a, _ in todo if text.count(a) != 1]
    if missing:
        print("[stop] WARN: mapping.sh changed upstream; stop timeouts not applied for:")
        for a in missing:
            print("         " + a.strip()[:90])
        return 1
    if check:
        print(f"[stop] mapping.sh stop timeouts ........... MISSING ({len(todo)} edits)")
        return 1
    backup = path + ".pre-timeouts"
    if not os.path.exists(backup):
        shutil.copy2(path, backup)
    for a, b in todo:
        text = text.replace(a, b)
    # New file + rename: a running bash reads its script as it goes, so an
    # in-place rewrite would corrupt a session that is still running.
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(text)
    shutil.copymode(path, tmp)
    os.replace(tmp, path)
    print(f"[stop] mapping.sh: stop timeouts added ({len(todo)} edits; original {os.path.basename(backup)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
