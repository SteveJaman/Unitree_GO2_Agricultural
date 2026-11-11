#!/usr/bin/env python3
"""
nav2_camera_source.py: switch the camera obstacle source in the Nav2 costmap.

  nav2_camera_source.py --enable  FILE   # add "camera" to the obstacle layer
  nav2_camera_source.py --enable --min-height 0.15 FILE   # with the --voxel floor cut
  nav2_camera_source.py --disable FILE   # remove it again (no-op if absent)

FILE is go2-nav2-wifi's go2_nav2/config/lidar3d_planner.yaml. Its
global_costmap section is the base of both the planner's costmap and the
controller's local costmap (lidar3d_planning/lidar3d_controller.launch.py), so
one source covers both.

The source reads /camera/obstacle_points (go2_camera_obstacles.py): marking
only, so LiDAR rays passing through glass never clear what the camera sees.
expected_update_rate makes Nav2 treat the costmap as out of date, and stop the
controller, if camera obstacles stop arriving; that is why wifi_nav.sh removes
the source again in every session without --camera-obstacles.

--min-height: the source's min_obstacle_height (default -10.0: the node already
keeps only points 0.08-1.0 m above the floor). wifi_nav.sh passes 0.15 with
--voxel: a VoxelLayer clears per height, and the LiDAR source (min 0.15) never
ray-traces below 0.15 m, so camera marks lower than that would never be cleared
and pile up into phantom walls. Enabling with another value rebuilds the block.

Every inserted line carries the marker below, so --disable restores the file
byte-for-byte.
"""

import argparse
import re
import sys

TAG = "  # go2-camera-obstacles"
CAMERA_BLOCK = """\
camera:
  topic: /camera/obstacle_points
  # The cloud is in lidar3d_map; ranges are measured from the robot.
  sensor_frame: base_footprint
  data_type: PointCloud2
  marking: true
  clearing: false
  # Points are already height-filtered above the floor in base_footprint.
  min_obstacle_height: {min_height}
  max_obstacle_height: 10.0
  obstacle_max_range: 3.5
  observation_persistence: 0.0
  expected_update_rate: 5.0
"""


def fmt_height(h):
    return repr(float(h))


def enable(lines, min_height=-10.0):
    want = f"min_obstacle_height: {fmt_height(min_height)}"
    if any(TAG in line for line in lines):
        if any(TAG in line and line.split("#")[0].strip() == want for line in lines):
            return lines, "already enabled"
        lines, _ = disable(lines)   # enabled with another min height: rebuild the block
    try:
        start = next(i for i, line in enumerate(lines) if line.strip() == "global_costmap:")
        # Any source list that has lidar: other patches add sources too ("lidar yolo").
        src = next(i for i in range(start, len(lines))
                   if re.match(r"\s*observation_sources:\s*[\"']?(\w+\s+)*lidar(\s+\w+)*[\"']?\s*(#.*)?$", lines[i]))
    except StopIteration:
        raise SystemExit("global_costmap obstacle_layer 'observation_sources: ... lidar ...' not found")
    indent = lines[src][: len(lines[src]) - len(lines[src].lstrip())]
    old = lines[src]
    m = re.match(r"\s*observation_sources:\s*([\"']?)([^\"'#]*?)\1\s*(#.*)?$", old)
    q, names = m.group(1), m.group(2).split()
    lines[src] = f"{indent}observation_sources: {q}{' '.join(names + ['camera'])}{q}{TAG} was: {old.strip()}"
    # Insert right after observation_sources, as a sibling of the lidar block.
    end = src + 1
    while end < len(lines) and (not lines[end].strip() or len(lines[end]) - len(lines[end].lstrip()) > len(indent)):
        end += 1
    while lines[end - 1].strip() == "":
        end -= 1
    body = CAMERA_BLOCK.format(min_height=fmt_height(min_height))
    block = [indent + line + TAG if line else line for line in body.rstrip("\n").split("\n")]
    lines[end:end] = block
    return lines, f"enabled (camera source in global/local costmaps, min height {fmt_height(min_height)} m)"


def disable(lines):
    if not any(TAG in line for line in lines):
        return lines, "not enabled"
    out = []
    for line in lines:
        if TAG not in line:
            out.append(line)
        elif "was: " in line:
            indent = line[: len(line) - len(line.lstrip())]
            out.append(indent + line.split("was: ", 1)[1])
    return out, "disabled"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--enable", action="store_true")
    mode.add_argument("--disable", action="store_true")
    ap.add_argument("--min-height", type=float, default=-10.0,
                    help="min_obstacle_height of the source (wifi_nav.sh: 0.15 with --voxel)")
    ap.add_argument("file")
    a = ap.parse_args()
    with open(a.file) as f:
        text = f.read()
    if a.enable:
        lines, msg = enable(text.split("\n"), a.min_height)
    else:
        lines, msg = disable(text.split("\n"))
    new = "\n".join(lines)
    if new != text:
        with open(a.file, "w") as f:
            f.write(new)
    print(f"[nav2] {a.file}: camera obstacle source {msg}")


if __name__ == "__main__":
    sys.exit(main())
