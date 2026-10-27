#!/usr/bin/env bash
# add-yolo-costmap-source.sh — let Nav2 avoid what YOLO sees (wifi_3d_nav.sh).
#
# yolo_node.py publishes /yolo/obstacles (PointCloud2, lidar3d_map), but a Nav2
# costmap only uses topics listed in its obstacle layer's observation_sources.
# This adds a "yolo" source to every ObstacleLayer / VoxelLayer of the global
# costmap in go2-nav2-wifi's go2_nav2/config/lidar3d_planner.yaml (the local
# costmap is copied from it at launch), then rebuilds go2_nav2.
#
#   marking: true, clearing: false — YOLO marks; the LiDAR's own raytracing
#   clears the cells once YOLO stops reporting the object.
#
# Usage:
#   ./scripts/tools/add-yolo-costmap-source.sh            # apply (safe to run twice)
#   ./scripts/tools/add-yolo-costmap-source.sh --undo     # restore the backup
#   NAV2_WIFI_DIR=~/files/go2-nav2-wifi (default)
set -euo pipefail

NAV2_WIFI_DIR="${NAV2_WIFI_DIR:-$HOME/files/go2-nav2-wifi}"
CONTAINER="${GO2_CONTAINER:-go2-lidar3d}"
CFG="${NAV2_WIFI_DIR}/go2_nav2/config/lidar3d_planner.yaml"
BACKUP="${CFG}.pre-yolo"
say()  { printf '\033[1;32m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m!!\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31mERROR:\033[0m %s\n' "$*" >&2; exit 1; }

[ -f "$CFG" ] || die "$CFG not found (set NAV2_WIFI_DIR to your go2-nav2-wifi clone)."
python3 -c 'import yaml' 2>/dev/null || die "python3-yaml is required (sudo apt install python3-yaml)."

rebuild() {
    if command -v docker >/dev/null && docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
        say "Rebuilding go2_nav2 inside $CONTAINER..."
        docker exec "$CONTAINER" bash -c 'source /opt/ros/humble/setup.bash && cd /ws && colcon build --symlink-install --base-paths src/go2_nav2 --packages-select go2_nav2' >/dev/null
    else
        warn "$CONTAINER not running: the change applies at the next session start (wifi_nav.sh rebuilds)."
        warn "If it does not, rebuild inside the container: cd /ws && colcon build --symlink-install --packages-select go2_nav2"
    fi
}

if [ "${1:-}" = "--undo" ]; then
    [ -f "$BACKUP" ] || die "no backup $BACKUP — nothing to undo."
    mv "$BACKUP" "$CFG"
    say "Restored $CFG"
    rebuild
    exit 0
fi

[ -f "$BACKUP" ] || cp "$CFG" "$BACKUP"

# Text edit (keeps comments and layout), then a YAML check of the result.
python3 - "$CFG" <<'PY'
import re, sys, yaml

path = sys.argv[1]
text = open(path).read()
doc = yaml.safe_load(text)

try:
    params = doc['global_costmap']['global_costmap']['ros__parameters']
except (KeyError, TypeError):
    sys.exit('ERROR: no global_costmap.global_costmap.ros__parameters in ' + path)

layers = [name for name in params.get('plugins', [])
          if isinstance(params.get(name), dict)
          and str(params[name].get('plugin', '')).split('::')[-1] in ('ObstacleLayer', 'VoxelLayer')]
if not layers:
    sys.exit('ERROR: no ObstacleLayer/VoxelLayer in global_costmap plugins: '
             + str(params.get('plugins')))

todo = [l for l in layers if 'yolo' not in str(params[l].get('observation_sources', '')).split()]
if not todo:
    print(f'already applied ({", ".join(layers)})')
    sys.exit(0)

lines = text.splitlines(keepends=True)
out, layer, layer_indent, done = [], None, None, set()
in_global = False
rx_key = re.compile(r'^(\s*)([A-Za-z0-9_]+):\s*(#.*)?$')
rx_src = re.compile(r'^(\s*)observation_sources:\s*(["\']?)([^"\'#\n]*)\2\s*(#.*)?$')
for line in lines:
    m = rx_key.match(line)
    if m and m.group(1) == '':               # top-level section
        in_global, layer = m.group(2) == 'global_costmap', None
    if m and in_global and m.group(2) in todo:
        layer, layer_indent = m.group(2), len(m.group(1))
    elif m and layer is not None and len(m.group(1)) <= layer_indent:
        layer = None
    s = rx_src.match(line)
    if s and layer in todo and layer not in done:
        ind, q, val, comment = s.group(1), s.group(2), s.group(3).strip(), s.group(4)
        out.append(f'{ind}observation_sources: {q}{(val + " yolo").strip()}{q}'
                   + (f'   {comment}' if comment else '') + '\n')
        out += [f'{ind}{l}\n' for l in (
            'yolo:   # wifi_3d_nav.sh: YOLO obstacles (add-yolo-costmap-source.sh)',
            '  topic: /yolo/obstacles',
            '  data_type: PointCloud2',
            '  marking: true',
            '  clearing: false',
            '  min_obstacle_height: -10.0',
            '  max_obstacle_height: 10.0',
            '  obstacle_max_range: 6.0',
            '  expected_update_rate: 0.0')]
        done.add(layer)
        continue
    out.append(line)

missing = set(todo) - done
if missing:
    sys.exit('ERROR: could not find an observation_sources line for ' + ', '.join(sorted(missing)))

new = ''.join(out)
check = yaml.safe_load(new)['global_costmap']['global_costmap']['ros__parameters']
for l in todo:
    assert 'yolo' in check[l]['observation_sources'].split(), l
    assert check[l]['yolo']['topic'] == '/yolo/obstacles', l
open(path, 'w').write(new)
print('added yolo source to: ' + ', '.join(todo))
PY

say "Config: $CFG (original kept as $BACKUP)"
rebuild
cat <<'DONE'

Done. Run ./scripts/runtime/wifi_3d_nav.sh; YOLO obstacles now show in the
costmap (RViz/Foxglove: /global_costmap/costmap). Undo: --undo.
Note: a VoxelLayer only keeps points inside its z range (origin_z ..
origin_z + z_voxels * z_resolution); check yolo boxes land inside it.
DONE
