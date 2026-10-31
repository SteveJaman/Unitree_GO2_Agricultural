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
  * the master: ServerAliveInterval=10, ServerAliveCountMax=6: a link dead
    for a minute fails, and so does every command through it. Not shorter:
    the master also carries the SLAM and motion sessions, and losing it ends
    the run (RViz closes). 5 s x 3 ended runs on ordinary Wi-Fi stalls
    (2026-10-30 14:47). The cleanup steps have their own time limits.
  * cleanup: a message saying what it waits for, and time limits on the
    local session stop (60 s), the Jetson session stop and map save
    (GO2_STOP_TIMEOUT, 120 s), the result transfer (GO2_FETCH_TIMEOUT,
    600 s) and closing the master (10 s). A transfer that times out leaves
    the results on the Jetson, as mapping.sh already reports.
  * keep the session (KEEP_TAG): with GO2_KEEPALIVE set (wifi_nav.sh sets
    it), the end of the SLAM or motion link no longer ends the whole run:
    scripts/runtime/session_keepalive.sh halts the robot, prints the cause
    and the fix, and waits for Ctrl+C. Without it, mapping.sh is unchanged.
Run by wifi_nav.sh before every session.
"""

import os
import shutil
import sys

TAG = "  # go2-agri: stop timeouts"
KEEP_TAG = "  # go2-agri: keep session"
FILE = "mapping.sh"

LOCAL_STOP = 'docker exec "$GO2_CONTAINER" bash /ws/scripts/go2-session.sh stop "$session_mode" --owner "$owner" >/dev/null 2>&1'
FETCH_CMD = 'python3 "$MAPPING_ROOT/ws/scripts/fetch-lidar3d.py" --target "$target" --ssh-socket "$ssh_socket" \\'
FETCH_ARGS = '        --run "$saved_run" --destination "$MAPPING_ROOT/ws/maps/lidar3d"'
FETCH_FAIL = '        echo "Transfer incomplete: results remain on Jetson; see $log_dir/slam.log" >&2'

# The fetch step, interruptible: the cleanup ignores Ctrl+C, and `timeout`
# runs its command in its own process group (no terminal Ctrl+C reaches it),
# so it runs in the background while this shell's own INT trap can end it.
FETCH_NEW = (
    '      echo "Copying the 3D map from the Jetson (up to ${GO2_FETCH_TIMEOUT:-600} s). Ctrl+C skips this: the map stays on the Jetson; fetch it later with:" >&2' + TAG + '\n'
    '      echo "        python3 $MAPPING_ROOT/ws/scripts/fetch-lidar3d.py --target $target --run $saved_run" >&2' + TAG + '\n'
    '     ' + TAG + '\n'
    '      timeout -k 5 "${GO2_FETCH_TIMEOUT:-600}" ' + FETCH_CMD + '\n'
    + FETCH_ARGS + ' &' + TAG + '\n'
    '      fetch_pid=$!' + TAG + '\n'
    '      trap \'kill -TERM "$fetch_pid" 2>/dev/null; echo "Transfer skipped (Ctrl+C): the map stays on the Jetson." >&2\' INT' + TAG + '\n'
    '      fetch_rc=0; wait "$fetch_pid" || fetch_rc=$?' + TAG + '\n'
    '      while kill -0 "$fetch_pid" 2>/dev/null; do fetch_rc=0; wait "$fetch_pid" || fetch_rc=$?; done' + TAG + '\n'
    "      trap '' INT" + TAG + '\n'
    '      [[ "$fetch_rc" == 0 ]] ||' + TAG + '\n'
    + FETCH_FAIL
)

EDITS = [
    (
        'ssh -o ConnectTimeout=8 -o ControlMaster=yes -o ControlPersist=60 -S "$ssh_socket" "$target" true',
        'ssh -o ConnectTimeout=8 -o ControlMaster=yes -o ControlPersist=60 '
        '-o ServerAliveInterval=10 -o ServerAliveCountMax=6 -S "$ssh_socket" "$target" true' + TAG,
    ),
    # First thing on Ctrl+C: say what happens, and start the 3D twin's map save
    # (GO2_STOP_EARLY, set by wifi_3d_nav.sh) alongside everything below.
    (
        "  trap '' INT TERM HUP\n",
        "  trap '' INT TERM HUP\n"
        '  echo "Stopping safely: the robot first, then the maps are saved (a minute at most). A second Ctrl+C only skips copying the map from the Jetson." >&2' + TAG + '\n'
        '  [[ -z "${GO2_STOP_EARLY:-}" ]] || { docker stop -t 60 $GO2_STOP_EARLY >/dev/null 2>&1 & }' + TAG + '\n',
    ),
    # The local Nav2/RViz stop runs alongside the Jetson stop (independent).
    (
        '    ' + LOCAL_STOP + ' || true',
        '    echo "Stopping the local Nav2/RViz session (up to 60 s, alongside the Jetson)..." >&2' + TAG + '\n'
        '    { timeout -k 5 60 ' + LOCAL_STOP + ' || true; } &' + TAG + '\n'
        '    local_stop_pid=$!' + TAG,
    ),
    (
        '    ssh -o BatchMode=yes -S "$ssh_socket" "$target" "docker exec go2-lidar3d-onboard bash /ws/scripts/go2-session.sh stop lidar3d --owner $owner" >/dev/null 2>&1 || true',
        '    echo "Stopping SLAM on the Jetson and saving its map (up to ${GO2_STOP_TIMEOUT:-120} s)..." >&2' + TAG + '\n'
        '    timeout -k 5 "${GO2_STOP_TIMEOUT:-120}" ssh -o BatchMode=yes -S "$ssh_socket" "$target" "docker exec go2-lidar3d-onboard bash /ws/scripts/go2-session.sh stop lidar3d --owner $owner" >/dev/null 2>&1 '
        '|| echo "Jetson stop did not finish (link down?): run ./scripts/runtime/wifi_nav.sh --clean before the next session" >&2' + TAG,
    ),
    (
        '      ' + FETCH_CMD + '\n' + FETCH_ARGS + ' || \\\n' + FETCH_FAIL,
        FETCH_NEW,
    ),
    (
        '  ssh -S "$ssh_socket" -O exit "$target" >/dev/null 2>&1 || true',
        '  [[ -z "${local_stop_pid:-}" ]] || wait "$local_stop_pid" 2>/dev/null || true' + TAG + '\n'
        '  timeout -k 2 10 ssh -S "$ssh_socket" -O exit "$target" >/dev/null 2>&1 || true' + TAG,
    ),
    (
        '    if [[ -n "$motion_pid" ]]; then\n'
        '      wait -n "$mapping_pid" "$viz_pid" "$motion_pid"\n',
        '    if [[ -n "${GO2_KEEPALIVE:-}" && -f "$GO2_KEEPALIVE" ]]; then' + KEEP_TAG + '\n'
        '      source "$GO2_KEEPALIVE"; go2_keepalive_wait' + KEEP_TAG + '\n'
        '    elif [[ -n "$motion_pid" ]]; then\n'
        '      wait -n "$mapping_pid" "$viz_pid" "$motion_pid"\n',
    ),
]


# (earlier edit, current edit): files patched by an older version of this script.
OLD_VERSIONS = [
    ("-o ServerAliveInterval=5 -o ServerAliveCountMax=3 -S \"$ssh_socket\" \"$target\" true" + TAG,
     "-o ServerAliveInterval=10 -o ServerAliveCountMax=6 -S \"$ssh_socket\" \"$target\" true" + TAG),
    # 2026-10-30: local stop in the foreground -> alongside the Jetson stop.
    ('    echo "Stopping the local Nav2/RViz session (up to 60 s)..." >&2' + TAG + '\n'
     '    timeout -k 5 60 ' + LOCAL_STOP + ' || true' + TAG,
     '    echo "Stopping the local Nav2/RViz session (up to 60 s, alongside the Jetson)..." >&2' + TAG + '\n'
     '    { timeout -k 5 60 ' + LOCAL_STOP + ' || true; } &' + TAG + '\n'
     '    local_stop_pid=$!' + TAG),
    ('    echo "Stopping SLAM on the Jetson and saving its map (up to ${GO2_STOP_TIMEOUT:-120} s; Ctrl+C is ignored until then)..." >&2' + TAG,
     '    echo "Stopping SLAM on the Jetson and saving its map (up to ${GO2_STOP_TIMEOUT:-120} s)..." >&2' + TAG),
    # 2026-10-30: fetch in the foreground (Ctrl+C could not skip it) -> interruptible.
    ('     ' + TAG + '\n'
     '      timeout -k 5 "${GO2_FETCH_TIMEOUT:-600}" ' + FETCH_CMD + '\n' + FETCH_ARGS + ' || \\\n' + FETCH_FAIL,
     FETCH_NEW),
    ('  timeout -k 2 10 ssh -S "$ssh_socket" -O exit "$target" >/dev/null 2>&1 || true' + TAG,
     '  [[ -z "${local_stop_pid:-}" ]] || wait "$local_stop_pid" 2>/dev/null || true' + TAG + '\n'
     '  timeout -k 2 10 ssh -S "$ssh_socket" -O exit "$target" >/dev/null 2>&1 || true' + TAG),
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
    # Upgrade an earlier version of these edits in place.
    # An upgrade applies once: some new edits contain the old text they replace.
    upgraded = [old for old, new in OLD_VERSIONS if old in text and new not in text]
    for old, new in OLD_VERSIONS:
        if old in text and new not in text:
            text = text.replace(old, new)
    todo = [(a, b) for a, b in EDITS if b not in text]
    if not todo and not upgraded:
        print("[stop] mapping.sh stop timeouts ........... OK")
        return 0
    missing = [a for a, _ in todo if text.count(a) != 1]
    if missing:
        print("[stop] WARN: mapping.sh changed upstream; stop timeouts not applied for:")
        for a in missing:
            print("         " + a.strip()[:90])
        return 1
    if check:
        print(f"[stop] mapping.sh stop timeouts ........... MISSING ({len(todo) + len(upgraded)} edits)")
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
    print(f"[stop] mapping.sh: stop timeouts added ({len(todo)} edits, {len(upgraded)} upgraded; "
          f"original {os.path.basename(backup)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
