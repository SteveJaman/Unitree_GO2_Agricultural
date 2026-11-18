#!/usr/bin/env python3
"""
rviz_gpu.py: draw RViz on the GPU instead of the CPU (go2-nav2-wifi's go2-lidar3d).

RViz runs in the go2-lidar3d container, which go2-nav2-wifi creates without /dev/dri and
starts RViz in with LIBGL_ALWAYS_SOFTWARE=1. Mesa then draws the point clouds, the
costmaps and the robot model on the CPU, on every core, next to RTAB-Map, YOLO and the
VLM, and the workstation froze (2026-11-18, during auto explore).

  rviz_gpu.py NAV2_WIFI_DIR          # apply (idempotent)
  rviz_gpu.py --check NAV2_WIFI_DIR

Edits (tagged TAG; originals kept as <file>.pre-gpu):
  * docker/docker-compose.lidar3d.yml: the container gets /dev/dri (the Intel graphics
    the desktop runs on) and the video/render groups (GO2_VIDEO_GID, GO2_RENDER_GID),
    and LIBGL_ALWAYS_SOFTWARE=${GO2_RVIZ_SOFTWARE_GL:-0}.
  * mapping.sh: RViz's docker exec passes ${GO2_RVIZ_SOFTWARE_GL:-0} instead of 1.

GO2_RVIZ_SOFTWARE_GL=1 brings back software drawing. Without /dev/dri in the container
Mesa falls back to software by itself. wifi_nav.sh recreates a go2-lidar3d made before
this change (no /dev/dri), and runs this tool before every session.
"""

import os
import shutil
import sys

TAG = "# go2-agri: rviz gpu"
COMPOSE = os.path.join("docker", "docker-compose.lidar3d.yml")
MAPPING = "mapping.sh"
SOFT = "${GO2_RVIZ_SOFTWARE_GL:-0}"

EDITS = {
    COMPOSE: [
        (
            '    user: "${GO2_UID:-1000}:${GO2_GID:-1000}"\n',
            '    user: "${GO2_UID:-1000}:${GO2_GID:-1000}"\n'
            f"    devices:  {TAG}\n"
            "      - /dev/dri:/dev/dri\n"
            "    group_add:\n"
            '      - "${GO2_VIDEO_GID:-44}"\n'
            '      - "${GO2_RENDER_GID:-110}"\n',
        ),
        (
            "      - GO2_AUTO_ENV=0\n",
            f"      - GO2_AUTO_ENV=0\n      - LIBGL_ALWAYS_SOFTWARE={SOFT}  {TAG}\n",
        ),
    ],
    # Both RViz launches (sensor view and the navigation view): every occurrence.
    MAPPING: [("-e LIBGL_ALWAYS_SOFTWARE=1 ", f"-e LIBGL_ALWAYS_SOFTWARE={SOFT} ")],
}


def patch(root, rel, edits, check):
    path = os.path.join(root, rel)
    if not os.path.isfile(path):
        print(f"[gpu] {path} not found")
        return 1
    text = open(path).read()
    # Pending: the anchor is there and either the result is not, or the edit replaces the
    # anchor outright (mapping.sh: every occurrence goes, so any left is still to do).
    todo = [(a, b) for a, b in edits if a in text and (b not in text or a not in b)]
    missing = [b for a, b in edits if a not in text and b not in text]
    if missing:
        print(f"[gpu] WARN: {rel} changed upstream; not applied for:")
        for b in missing:
            print("         " + b.strip().splitlines()[0][:90])
        return 1
    if not todo:
        return 0
    if check:
        print(f"[gpu] {rel}: MISSING ({len(todo)} edits)")
        return 1
    backup = path + ".pre-gpu"
    if not os.path.exists(backup):
        shutil.copy2(path, backup)
    for a, b in todo:
        text = text.replace(a, b)
    # New file + rename: a running bash reads its script as it goes.
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(text)
    shutil.copymode(path, tmp)
    os.replace(tmp, path)
    print(f"[gpu] {rel}: {len(todo)} edits (original {os.path.basename(backup)})")
    return 0


def main():
    args = sys.argv[1:]
    check = "--check" in args
    args = [a for a in args if a != "--check"]
    if len(args) != 1:
        sys.exit(__doc__)
    bad = sum(patch(args[0], rel, edits, check) for rel, edits in EDITS.items())
    if not bad:
        print("[gpu] RViz on the GPU (GO2_RVIZ_SOFTWARE_GL=0) ... OK")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
