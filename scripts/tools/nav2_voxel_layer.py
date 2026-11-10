#!/usr/bin/env python3
"""
nav2_voxel_layer.py: switch the 3D stack's obstacle layer to a Nav2 VoxelLayer.

  nav2_voxel_layer.py --enable  FILE   # ObstacleLayer -> VoxelLayer
  nav2_voxel_layer.py --disable FILE   # back to the ObstacleLayer (no-op if not enabled)

FILE is go2-nav2-wifi's go2_nav2/config/lidar3d_planner.yaml. Its global_costmap
section is the base of both the planner's costmap and the controller's local
costmap (lidar3d_controller.launch.py copies it), so one change covers both.

What changes, in global_costmap's obstacle_layer:
  * plugin nav2_costmap_2d::ObstacleLayer -> nav2_costmap_2d::VoxelLayer: obstacles
    are kept as a 3D grid and cleared by 3D raytracing, so a ray passing above a
    low mark (or under a table top) no longer wipes or keeps the wrong column;
  * a 3D grid from the floor: origin_z 0.0, z_resolution 0.1, z_voxels 16 (1.6 m;
    VoxelLayer allows at most 16 voxels per column), max_obstacle_height 1.5;
  * mark_threshold 0: a column is an obstacle as soon as one voxel in it is
    occupied (VoxelLayer marks when the count EXCEEDS the threshold);
  * publish_voxel_map true: nav2_msgs/VoxelGrid on /<costmap>/voxel_grid (for RViz,
    `ros2 run nav2_costmap_2d nav2_costmap_2d_cloud --ros-args -r voxel_grid:=/local_costmap/voxel_grid`
    turns it into the PointCloud2 voxel_marked_cloud);
  * the lidar source reads the self-filtered /lidar3d/obstacle_points instead of
    /lidar3d/registered, which still contains the robot's own legs and arm (the
    local costmap already used it; the global one did not), and its
    max_obstacle_height becomes 1.5 to match the grid.
The other sources stay as they are: "yolo" (add-yolo-costmap-source.sh) and the
front-camera source "camera" (nav2_camera_source.py, wifi_3d_nav.sh
--camera-obstacles) feed the voxel grid the same way they fed the obstacle layer.

Every inserted or changed line carries the marker below, so --disable restores
the file byte for byte, and other text patches to the file keep working.
"""

import argparse
import re
import sys

TAG = "  # go2-voxel-layer"
VOXEL_PARAMS = [
    "publish_voxel_map: true",
    "origin_z: 0.0",
    "z_resolution: 0.1",
    "z_voxels: 16",
    "max_obstacle_height: 1.5",
    "mark_threshold: 0",
    "unknown_threshold: 15",
    "combination_method: 1",
    "footprint_clearing_enabled: true",
]
LIDAR_TOPIC_OLD, LIDAR_TOPIC_NEW = "/lidar3d/registered", "/lidar3d/obstacle_points"


def indent_of(line):
    return len(line) - len(line.lstrip())


def block_end(lines, start):
    """Index after the block that starts at `start` (lines indented deeper than it)."""
    ind = indent_of(lines[start])
    end = start + 1
    while end < len(lines) and (not lines[end].strip() or indent_of(lines[end]) > ind):
        end += 1
    while end > start + 1 and not lines[end - 1].strip():
        end -= 1
    return end


def find_key(lines, key, start, end, indent=None):
    for i in range(start, end):
        m = re.match(r"^(\s*)" + re.escape(key) + r":\s*(#.*)?$", lines[i])
        if m and (indent is None or len(m.group(1)) == indent):
            return i
    return None


def changed(old, new_value_line):
    """old line rewritten to new_value_line (same indent), remembering the old one."""
    ind = old[: indent_of(old)]
    return f"{ind}{new_value_line}{TAG} was: {old.strip()}"


def enable(lines):
    if any(TAG in line for line in lines):
        return lines, "already enabled"
    top = next((i for i, l in enumerate(lines) if l.strip() == "global_costmap:" and indent_of(l) == 0), None)
    if top is None:
        raise SystemExit("global_costmap section not found")
    top_end = block_end(lines, top)
    layer = find_key(lines, "obstacle_layer", top, top_end)
    if layer is None:
        raise SystemExit("global_costmap obstacle_layer not found")
    lend = block_end(lines, layer)
    plugin = next((i for i in range(layer + 1, lend)
                   if re.match(r"^\s*plugin:\s*[\"']?nav2_costmap_2d::ObstacleLayer[\"']?\s*$", lines[i])), None)
    if plugin is None:
        raise SystemExit("obstacle_layer is not a plain nav2_costmap_2d::ObstacleLayer; nothing changed")
    ind = lines[plugin][: indent_of(lines[plugin])]
    lidar = find_key(lines, "lidar", layer + 1, lend, indent_of(lines[plugin]))
    if lidar is None:
        raise SystemExit("obstacle_layer has no 'lidar' source")
    lidar_end = block_end(lines, lidar)
    for i in range(lidar + 1, lidar_end):
        s = lines[i].strip()
        if re.match(r"^topic:\s*" + re.escape(LIDAR_TOPIC_OLD) + r"\s*$", s):
            lines[i] = changed(lines[i], f"topic: {LIDAR_TOPIC_NEW}")
        elif re.match(r"^max_obstacle_height:\s*[0-9.]+\s*$", s) and s != "max_obstacle_height: 1.5":
            lines[i] = changed(lines[i], "max_obstacle_height: 1.5")
    lines[plugin] = changed(lines[plugin], "plugin: nav2_costmap_2d::VoxelLayer")
    lines[plugin + 1:plugin + 1] = [ind + p + TAG for p in VOXEL_PARAMS]
    return lines, "enabled (VoxelLayer in global/local costmaps, lidar /lidar3d/obstacle_points)"


def disable(lines):
    if not any(TAG in line for line in lines):
        return lines, "not enabled"
    out = []
    for line in lines:
        if TAG not in line:
            out.append(line)
        elif "was: " in line:
            out.append(line[: indent_of(line)] + line.split("was: ", 1)[1])
    return out, "disabled"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--enable", action="store_true")
    mode.add_argument("--disable", action="store_true")
    ap.add_argument("file")
    a = ap.parse_args()
    with open(a.file) as f:
        text = f.read()
    lines, msg = (enable if a.enable else disable)(text.split("\n"))
    new = "\n".join(lines)
    if new != text:
        import yaml   # the result must still be valid YAML before it is written
        yaml.safe_load(new)
        with open(a.file, "w") as f:
            f.write(new)
    print(f"[nav2] {a.file}: voxel layer {msg}")


if __name__ == "__main__":
    sys.exit(main())
