#!/usr/bin/env python3
"""
desktop_capture.py: list, tile and capture the simulation's desktop windows (X11).

  python3 scripts/tools/desktop_capture.py list                    # window titles (xwininfo)
  python3 scripts/tools/desktop_capture.py tile --wait 60          # 2x2: Gazebo, Nav2 RViz, MoveIt RViz, camera RViz
  python3 scripts/tools/desktop_capture.py shot OUT.png            # screenshot of the whole screen
  python3 scripts/tools/desktop_capture.py record OUT.mkv          # screen video until SIGINT/SIGTERM (H.264, 15 fps)

Window titles matched (case-insensitive): "gazebo" (not RViz) top left, "lidar3d_navigation"
(Nav2 RViz) bottom left, "moveit" (MoveIt RViz) top right, "plant_detection" (camera RViz)
bottom right. Tiling sends EWMH _NET_MOVERESIZE_WINDOW requests to the window manager and
needs python-xlib (pip install python-xlib, or PYTHONPATH to a folder with it). Capture uses
ffmpeg's x11grab: $FFMPEG, else ffmpeg on PATH, else the imageio-ffmpeg wheel's binary.
Record to .mkv (survives a hard stop), then remux to .mp4 with `ffmpeg -c copy`.
"""
import argparse
import os
import re
import shutil
import signal
import subprocess
import sys
import time

TILES = (("gazebo", 0, 0), ("lidar3d_navigation", 0, 1), ("moveit", 1, 0), ("plant_detection", 1, 1))


def ffmpeg():
    if os.environ.get("FFMPEG"):
        return os.environ["FFMPEG"]
    if shutil.which("ffmpeg"):
        return shutil.which("ffmpeg")
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError:
        sys.exit("no ffmpeg: set FFMPEG, install ffmpeg, or pip install imageio-ffmpeg")


def screen_size():
    out = subprocess.run(["xdpyinfo"], capture_output=True, text=True).stdout
    m = re.search(r"dimensions:\s+(\d+)x(\d+)", out)
    return (int(m.group(1)), int(m.group(2))) if m else (1920, 1080)


def list_windows():
    """Titles of the top-level named windows (xwininfo, no extra package)."""
    out = subprocess.run(["xwininfo", "-root", "-tree"], capture_output=True, text=True).stdout
    names = re.findall(r'^\s+0x[0-9a-f]+ "([^"]+)":', out, re.MULTILINE)
    return sorted({n for n in names if n.strip()})


def key_of(title):
    low = title.lower()
    if "rviz" in low:
        for k in ("lidar3d_navigation", "moveit", "plant_detection"):
            if k in low:
                return k
        return None
    return "gazebo" if "gazebo" in low else None


def tile(wait):
    from Xlib import X, display
    from Xlib.protocol import event
    d = display.Display()
    root = d.screen().root
    W, H = d.screen().width_in_pixels, d.screen().height_in_pixels
    atom = d.intern_atom
    client_list, wm_name, utf8 = atom("_NET_CLIENT_LIST"), atom("_NET_WM_NAME"), atom("UTF8_STRING")

    def send(w, a, data):
        ev = event.ClientMessage(window=w, client_type=a, data=(32, data + [0] * (5 - len(data))))
        root.send_event(ev, event_mask=X.SubstructureRedirectMask | X.SubstructureNotifyMask)

    done, deadline = set(), time.time() + wait
    while time.time() < deadline and len(done) < len(TILES):
        prop = root.get_full_property(client_list, X.AnyPropertyType)
        for wid in (prop.value if prop else []):
            w = d.create_resource_object("window", wid)
            try:
                p = w.get_full_property(wm_name, utf8)
                title = p.value.decode(errors="replace") if p else (w.get_wm_name() or "")
            except Exception:  # noqa: BLE001  (a window closing while we look)
                continue
            k = key_of(title)
            if not k or k in done:
                continue
            col, row = next((c, r) for kk, c, r in TILES if kk == k)
            send(w, atom("_NET_WM_STATE"), [0, atom("_NET_WM_STATE_MAXIMIZED_VERT"), atom("_NET_WM_STATE_MAXIMIZED_HORZ"), 1])
            flags = (1 << 8) | (1 << 9) | (1 << 10) | (1 << 11) | 1
            send(w, atom("_NET_MOVERESIZE_WINDOW"), [flags, col * W // 2, row * H // 2, W // 2, H // 2])
            send(w, atom("_NET_ACTIVE_WINDOW"), [2, X.CurrentTime])  # to the front: a capture records the screen
            d.flush()
            done.add(k)
        time.sleep(1)
    return sorted(done)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list")
    t = sub.add_parser("tile")
    t.add_argument("--wait", type=float, default=60)
    s = sub.add_parser("shot")
    s.add_argument("out")
    r = sub.add_parser("record")
    r.add_argument("out")
    r.add_argument("--fps", type=int, default=15)
    a = ap.parse_args()
    disp = os.environ.get("DISPLAY", ":0")
    if a.cmd == "list":
        print("\n".join(list_windows()))
    elif a.cmd == "tile":
        print("tiled:", ", ".join(tile(a.wait)) or "none")
    elif a.cmd == "shot":
        w, h = screen_size()
        subprocess.run([ffmpeg(), "-loglevel", "error", "-y", "-f", "x11grab", "-video_size", f"{w}x{h}", "-i", disp,
                        "-frames:v", "1", a.out], check=True)
        print(a.out)
    elif a.cmd == "record":
        w, h = screen_size()
        p = subprocess.Popen([ffmpeg(), "-loglevel", "error", "-y", "-f", "x11grab", "-framerate", str(a.fps),
                              "-video_size", f"{w}x{h}", "-i", disp, "-c:v", "libx264", "-preset", "ultrafast",
                              "-crf", "28", "-pix_fmt", "yuv420p", a.out], stdin=subprocess.PIPE)
        # SIGINT makes ffmpeg finish the file and exit; its "q" key on stdin is ignored when it runs in
        # the background (a script's `&` job), which left recordings running for hours
        stop = lambda *_: p.send_signal(signal.SIGINT)  # noqa: E731
        signal.signal(signal.SIGINT, lambda *_: stop())
        signal.signal(signal.SIGTERM, lambda *_: stop())
        p.wait()


if __name__ == "__main__":
    main()
