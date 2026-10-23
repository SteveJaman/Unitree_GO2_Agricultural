#!/usr/bin/env python3
"""
rviz_add_camera.py: add the camera and fusion displays to RViz layouts.

Inserts, once per file (skipped if already there):
  * "Front camera"          Image  /camera/image_raw       (Reliable, as the relay publishes)
  * "LiDAR on camera"       Image  /camera/lidar_overlay   (wifi_3d_nav.sh --fusion / lidar_overlay.sh)
  * "3D map (RTAB-Map)"     PointCloud2 /rtabmap/cloud_map, coloured from the camera with --fusion
  * "Camera obstacles"      PointCloud2 /camera/obstacle_points (--camera-obstacles)
  * "Camera depth"          Image  /camera/obstacle_depth  (metric depth + fit quality)

wifi_nav.sh runs it on go2-nav2-wifi's lidar3d_planning.rviz and
lidar3d_navigation.rviz before RViz starts. Text insertion only: the rest of
each file is left byte-for-byte as it is.

    python3 scripts/tools/rviz_add_camera.py FILE.rviz [FILE.rviz ...]
"""

import sys

MARKER = "Name: Front camera"
OBSTACLE_MARKER = "Name: Camera obstacles"
OBSTACLE_DISPLAYS = """\
- Class: rviz_default_plugins/PointCloud2
  Name: Camera obstacles
  Enabled: true
  Topic:
    Value: /camera/obstacle_points
    Reliability Policy: Reliable
    Durability Policy: Volatile
    History Policy: Keep Last
    Depth: 1
  Style: Points
  Size (Pixels): 3
  Color Transformer: FlatColor
  Color: 255; 0; 0
  Position Transformer: XYZ
  Decay Time: 0
- Class: rviz_default_plugins/Image
  Name: Camera depth
  Enabled: false
  Topic:
    Value: /camera/obstacle_depth
    Reliability Policy: Reliable
    Durability Policy: Volatile
    History Policy: Keep Last
    Depth: 1
  Normalize Range: true
  Median window: 5
  Min Value: 0
  Max Value: 1
"""

DISPLAYS = """\
- Class: rviz_default_plugins/Image
  Name: Front camera
  Enabled: true
  Topic:
    Value: /camera/image_raw
    Reliability Policy: Reliable
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


def insert(lines, text):
    """Insert display items right after the top-level 'Displays:' list key."""
    i = next(n for n, line in enumerate(lines) if line.strip() == "Displays:" and line.startswith("  "))
    item = lines[i + 1] if i + 1 < len(lines) else ""
    indent = item[: len(item) - len(item.lstrip())] if item.lstrip().startswith("- ") else lines[i][:2]
    lines[i + 1 : i + 1] = [indent + line if line else line for line in text.rstrip("\n").split("\n")]


def add_obstacles(path):
    with open(path) as f:
        lines = f.read().split("\n")
    if any(OBSTACLE_MARKER in line for line in lines):
        return "already has the camera obstacle displays"
    try:
        insert(lines, OBSTACLE_DISPLAYS)
    except StopIteration:
        return "no 'Displays:' list found, left unchanged"
    with open(path, "w") as f:
        f.write("\n".join(lines))
    return "added Camera obstacles, Camera depth"


def add(path):
    with open(path) as f:
        lines = f.read().split("\n")
    if any(MARKER in line for line in lines):
        # Earlier versions read the camera Best Effort; the relay now publishes
        # reliable and a best-effort reader drops most large frames.
        for n, line in enumerate(lines):
            if line.strip() == "Value: /camera/image_raw" and "Best Effort" in lines[n + 1]:
                lines[n + 1] = lines[n + 1].replace("Best Effort", "Reliable")
                with open(path, "w") as f:
                    f.write("\n".join(lines))
                return "Front camera switched to Reliable"
        return "already has the camera displays"
    try:
        insert(lines, DISPLAYS)
    except StopIteration:
        return "no 'Displays:' list found, left unchanged"
    with open(path, "w") as f:
        f.write("\n".join(lines))
    return "added Front camera, LiDAR on camera, 3D map (RTAB-Map)"


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for path in sys.argv[1:]:
        try:
            print(f"[rviz] {path}: {add(path)}; {add_obstacles(path)}")
        except OSError as e:
            print(f"[rviz] {path}: {e}")


if __name__ == "__main__":
    main()
