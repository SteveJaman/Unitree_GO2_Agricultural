#!/usr/bin/env python3
"""
nav2_voxel_layer.py: switch the 3D stack's obstacle layer to a 3D voxel layer.

  nav2_voxel_layer.py --enable  FILE                  # ObstacleLayer -> VoxelLayer
  nav2_voxel_layer.py --enable --plugin stvl FILE     # -> SpatioTemporalVoxelLayer
  nav2_voxel_layer.py --disable FILE   # back to the ObstacleLayer (no-op if not enabled)

--plugin (or VOXEL_PLUGIN): "stvl" is the Spatio-Temporal Voxel Layer
(ros-humble-spatio-temporal-voxel-layer; wifi_nav.sh installs it and uses it for
--voxel): every voxel expires VOXEL_DECAY s (3) after the LiDAR last saw it, and a
3D-LiDAR frustum clears what it sees free, so people and moved chairs leave the
costmap. Nav2's VoxelLayer ("voxel") clears only voxels a ray happens to cross:
in the sim a removed box kept one voxel of its front face (0.4-0.5 m, 1 m ahead)
lethal for good, floor rays and origin_z -0.1 notwithstanding (2026-11-11).

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
  * the lidar source's topic becomes the self-filtered /lidar3d/obstacle_points in
    the file: that is what already runs (lidar3d_planning.launch.py and
    lidar3d_controller.launch.py override the topic at launch), so the file matches
    the running costmaps; /lidar3d/registered still contains the robot's own legs
    and arm. Its max_obstacle_height becomes 1.5 to match the grid.
The other sources stay as they are: "yolo" (add-yolo-costmap-source.sh) and the
front-camera source "camera" (nav2_camera_source.py, wifi_3d_nav.sh
--camera-obstacles) feed the voxel grid the same way they fed the obstacle layer.

Every inserted or changed line carries the marker below, so --disable restores
the file byte for byte, and other text patches to the file keep working.
"""

import argparse
import os
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
STVL_PLUGIN = "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
STVL_PARAMS = [
    "voxel_decay: " + os.environ.get("VOXEL_DECAY", "3.0") + "  # s a voxel lives unseen",
    "decay_model: 0",
    "voxel_size: 0.05",
    "track_unknown_space: true",
    "mark_threshold: 0",
    "update_footprint_enabled: true",
    "combination_method: 1",
    "publish_voxel_map: true",
    "mapping_mode: false",
    "map_save_duration: 60.0",
]
# The LiDAR (Unitree L1, 360 deg dome) as a clearing frustum: what it sees free is cleared
# faster than the decay (decay_acceleration).
STVL_LIDAR = [
    "model_type: 1",
    "horizontal_fov_angle: 6.29",
    "vertical_fov_angle: 1.57",
    "vertical_fov_padding: 0.05",
    "min_z: 0.15",
    "max_z: 8.0",
    "decay_acceleration: 5.0",
]


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


def stvl_sources(lines, layer):
    """STVL needs obstacle_range (ObstacleLayer: obstacle_max_range) on every source,
    clear_after_reading, and the LiDAR's clearing frustum. Tagged lines only."""
    lend = block_end(lines, layer)
    src = next((i for i in range(layer + 1, lend) if re.match(r"^\s*observation_sources:", lines[i])), None)
    if src is None:
        return lines
    names = lines[src].split("#", 1)[0].split(":", 1)[1].split()
    blocks = []
    for name in names:
        k = find_key(lines, name, layer + 1, lend, None)
        if k is not None:
            blocks.append((k, block_end(lines, k), name))
    for k, kend, name in sorted(blocks, reverse=True):  # bottom up: indices stay valid
        vi = lines[k][: indent_of(lines[k])] + "  "
        rng = "6.0"
        for i in range(k + 1, kend):
            m = re.match(r"^\s*obstacle_max_range:\s*([0-9.]+)", lines[i])
            if m:
                rng = m.group(1)
        extra = ["obstacle_range: " + rng, "clear_after_reading: true", "inf_is_valid: false"]
        if name == "lidar":
            extra += STVL_LIDAR
        lines[kend:kend] = [vi + e + TAG for e in extra]
    return lines


def current_plugin(lines):
    if not any(TAG in line for line in lines):
        return None
    return "stvl" if any(STVL_PLUGIN in line and TAG in line for line in lines) else "voxel"


def enable(lines, plugin="voxel"):
    if current_plugin(lines) not in (None, plugin):
        lines, _ = disable(lines)  # the other plugin: back to the original first
    if current_plugin(lines) == "stvl":
        return lines, "already enabled (SpatioTemporalVoxelLayer)"
    if current_plugin(lines) == "voxel":
        return lines, "already enabled"
    top = next((i for i, l in enumerate(lines) if l.strip() == "global_costmap:" and indent_of(l) == 0), None)
    if top is None:
        raise SystemExit("global_costmap section not found")
    top_end = block_end(lines, top)
    layer = find_key(lines, "obstacle_layer", top, top_end)
    if layer is None:
        raise SystemExit("global_costmap obstacle_layer not found")
    lend = block_end(lines, layer)
    plugin_i = next((i for i in range(layer + 1, lend)
                     if re.match(r"^\s*plugin:\s*[\"']?nav2_costmap_2d::ObstacleLayer[\"']?\s*$", lines[i])), None)
    if plugin_i is None:
        raise SystemExit("obstacle_layer is not a plain nav2_costmap_2d::ObstacleLayer; nothing changed")
    ind = lines[plugin_i][: indent_of(lines[plugin_i])]
    lidar = find_key(lines, "lidar", layer + 1, lend, indent_of(lines[plugin_i]))
    if lidar is None:
        raise SystemExit("obstacle_layer has no 'lidar' source")
    lidar_end = block_end(lines, lidar)
    for i in range(lidar + 1, lidar_end):
        s = lines[i].strip()
        if re.match(r"^topic:\s*" + re.escape(LIDAR_TOPIC_OLD) + r"\s*$", s):
            lines[i] = changed(lines[i], f"topic: {LIDAR_TOPIC_NEW}")
        elif re.match(r"^max_obstacle_height:\s*[0-9.]+\s*$", s) and s != "max_obstacle_height: 1.5":
            lines[i] = changed(lines[i], "max_obstacle_height: 1.5")
    if plugin == "stvl":
        lines[plugin_i] = changed(lines[plugin_i], "plugin: " + STVL_PLUGIN)
        lines[plugin_i + 1:plugin_i + 1] = [ind + p + TAG for p in STVL_PARAMS + ["max_obstacle_height: 1.5"]]
        lines = stvl_sources(lines, layer)
        return lines, "enabled (SpatioTemporalVoxelLayer, decay " + os.environ.get("VOXEL_DECAY", "3.0") + " s, lidar /lidar3d/obstacle_points)"
    lines[plugin_i] = changed(lines[plugin_i], "plugin: nav2_costmap_2d::VoxelLayer")
    lines[plugin_i + 1:plugin_i + 1] = [ind + p + TAG for p in VOXEL_PARAMS]
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
    ap.add_argument("--plugin", choices=["voxel", "stvl"], default=os.environ.get("VOXEL_PLUGIN", "voxel"))
    ap.add_argument("file")
    a = ap.parse_args()
    with open(a.file) as f:
        text = f.read()
    lines, msg = enable(text.split("\n"), a.plugin) if a.enable else disable(text.split("\n"))
    new = "\n".join(lines)
    if new != text:
        import yaml   # the result must still be valid YAML before it is written
        yaml.safe_load(new)
        with open(a.file, "w") as f:
            f.write(new)
    print(f"[nav2] {a.file}: voxel layer {msg}")


if __name__ == "__main__":
    sys.exit(main())
