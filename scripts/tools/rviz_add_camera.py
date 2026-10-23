#!/usr/bin/env python3
"""
rviz_add_camera.py: add the camera and fusion displays to RViz layouts.

Inserts, once per file (skipped if already there):
  * "Front camera"          Image  /camera/image_raw       (Best Effort, as the relay publishes)
  * "LiDAR on camera"       Image  /camera/lidar_overlay   (wifi_3d_nav.sh --fusion / lidar_overlay.sh)
  * "3D map (RTAB-Map)"     PointCloud2 /rtabmap/cloud_map, coloured from the camera with --fusion

wifi_nav.sh runs it on go2-nav2-wifi's lidar3d_planning.rviz and
lidar3d_navigation.rviz before RViz starts. Text insertion only: the rest of
each file is left byte-for-byte as it is.

    python3 scripts/tools/rviz_add_camera.py FILE.rviz [FILE.rviz ...]
"""

import sys

MARKER = "Name: Front camera"

DISPLAYS = """\
- Class: rviz_default_plugins/Image
  Name: Front camera
  Enabled: true
  Topic:
    Value: /camera/image_raw
    Reliability Policy: Best Effort
    Durability Policy: Volatile
    History Policy: Keep Last
    Depth: 1
  Normalize Range: true
  Median window: 5
  Min Value: 0
  Max Value: 1
- Class: rviz_default_plugins/Image
  Name: LiDAR on camera
  Enabled: true
  Topic:
    Value: /camera/lidar_overlay
    Reliability Policy: Reliable
    Durability Policy: Volatile
    History Policy: Keep Last
    Depth: 1
  Normalize Range: true
  Median window: 5
  Min Value: 0
  Max Value: 1
- Class: rviz_default_plugins/PointCloud2
  Name: 3D map (RTAB-Map)
  Enabled: true
  Topic:
    Value: /rtabmap/cloud_map
    Reliability Policy: Reliable
    Durability Policy: Transient Local
    History Policy: Keep Last
    Depth: 1
  Style: Points
  Size (Pixels): 3
  Color Transformer: RGB8
  Position Transformer: XYZ
  Decay Time: 0
"""


def add(path):
    with open(path) as f:
        lines = f.read().split("\n")
    if any(MARKER in line for line in lines):
        return "already has the camera displays"
    try:
        i = next(n for n, line in enumerate(lines) if line.strip() == "Displays:" and line.startswith("  "))
    except StopIteration:
        return "no 'Displays:' list found, left unchanged"
    # Indent like the existing list items under Displays:.
    item = lines[i + 1] if i + 1 < len(lines) else ""
    indent = item[: len(item) - len(item.lstrip())] if item.lstrip().startswith("- ") else lines[i][:2]
    block = [indent + line if line else line for line in DISPLAYS.rstrip("\n").split("\n")]
    lines[i + 1 : i + 1] = block
    with open(path, "w") as f:
        f.write("\n".join(lines))
    return "added Front camera, LiDAR on camera, 3D map (RTAB-Map)"


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for path in sys.argv[1:]:
        try:
            print(f"[rviz] {path}: {add(path)}")
        except OSError as e:
            print(f"[rviz] {path}: {e}")


if __name__ == "__main__":
    main()
