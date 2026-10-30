#!/usr/bin/env bash
# relax-costmap.sh — a less "sticky" Nav2 costmap for the sparse Wi-Fi LiDAR (wifi_3d_nav.sh).
#
# What counts as an obstacle is unchanged (same heights, footprint, inflation
# radius): lowering those would hide real obstacles. Two things made the
# costmap fill up with obstacles that are not there and the dog refuse to move:
#
#   1. Marks that never clear. The LiDAR marks out to obstacle_max_range
#      (12 m) but clears (raytraces) only out to raytrace_max_range (5 m). A
#      stray point 5-12 m away (dust, a reflection, a smeared scan from the
#      ~230-point Wi-Fi clouds) stays in the global costmap for good, and
#      every one becomes an inflated blob. -> raytrace_max_range is raised to
#      obstacle_max_range, so the next scan through that spot clears it.
#   2. Inflation that decays slowly. cost_scaling_factor 2.5 keeps cells
#      ~0.5 m from any mark at high cost; the planner treats a 1 m wide
#      corridor as nearly blocked. -> 5.0: cost falls off twice as fast past
#      the footprint; the lethal and inscribed cells are the same.
#
# Edits every costmap in go2-nav2-wifi's go2_nav2/config/lidar3d_planner.yaml
# (the controller's local costmap is copied from the global one at launch),
# then rebuilds go2_nav2. Run by wifi_3d_nav.sh at every start (safe to run
# twice).
#
# Usage:
#   ./scripts/tools/relax-costmap.sh            # apply
#   ./scripts/tools/relax-costmap.sh --undo     # restore the backup
#   NAV2_WIFI_DIR=~/files/go2-nav2-wifi (default)   COST_SCALING=5.0
set -euo pipefail

NAV2_WIFI_DIR="${NAV2_WIFI_DIR:-$HOME/files/go2-nav2-wifi}"
CONTAINER="${GO2_CONTAINER:-go2-lidar3d}"
CFG="${NAV2_WIFI_DIR}/go2_nav2/config/lidar3d_planner.yaml"
BACKUP="${CFG}.pre-relax"
COST_SCALING="${COST_SCALING:-5.0}"
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
    fi
}

if [ "${1:-}" = "--undo" ]; then
    [ -f "$BACKUP" ] || die "no backup $BACKUP — nothing to undo."
    mv "$BACKUP" "$CFG"
    say "Restored $CFG"
    rebuild
    exit 0
fi

# Text edit (keeps comments and layout), then a YAML check of the result.
# Prints "unchanged" when there is nothing to do.
result="$(python3 - "$CFG" "$COST_SCALING" <<'PY'
import re, sys, yaml

path, scaling = sys.argv[1], float(sys.argv[2])
text = open(path).read()
rx = re.compile(r'^(\s*)(obstacle_max_range|raytrace_max_range|cost_scaling_factor):\s*([0-9.]+)(\s*(#.*)?)$')
out, changes = [], []
block_range = {}  # indent -> obstacle_max_range of the block being read
for line in text.splitlines(keepends=True):
    m = rx.match(line.rstrip('\n'))
    if m:
        ind, key, val = m.group(1), m.group(2), float(m.group(3))
        tail = f'  {m.group(5)}' if m.group(5) else ''
        if key == 'obstacle_max_range':
            block_range[len(ind)] = val
        elif key == 'raytrace_max_range' and block_range.get(len(ind), 0) > val:
            new = block_range[len(ind)]
            changes.append(f'raytrace_max_range {val:g} -> {new:g}')
            line = f'{ind}{key}: {new:.1f}   # relax-costmap.sh: clear as far as it marks (was {val:g}){tail}\n'
        elif key == 'cost_scaling_factor' and val < scaling:
            changes.append(f'cost_scaling_factor {val:g} -> {scaling:g}')
            line = f'{ind}{key}: {scaling:.1f}   # relax-costmap.sh: faster cost decay (was {val:g}){tail}\n'
    elif line.strip() and not line.lstrip().startswith('#'):
        # A key at a shallower indent ends the blocks nested deeper.
        depth = len(line) - len(line.lstrip())
        block_range = {k: v for k, v in block_range.items() if k <= depth}
    out.append(line)

if not changes:
    print('unchanged')
    sys.exit(0)
new = ''.join(out)
doc = yaml.safe_load(new)
for section in ('global_costmap', 'local_costmap'):
    params = (doc.get(section) or {}).get(section, {}).get('ros__parameters', {})
    for name, layer in params.items():
        if not isinstance(layer, dict):
            continue
        for src in layer.values():
            if isinstance(src, dict) and 'raytrace_max_range' in src:
                assert src['raytrace_max_range'] >= src.get('obstacle_max_range', 0), (section, name)
        if 'cost_scaling_factor' in layer:
            assert layer['cost_scaling_factor'] >= scaling, (section, name)
open(path + '.new', 'w').write(new)
print('; '.join(changes))
PY
)"

if [ "$result" = "unchanged" ]; then
    echo "already applied (clearing range = marking range, cost scaling >= ${COST_SCALING})"
    exit 0
fi
[ -f "$BACKUP" ] || cp "$CFG" "$BACKUP"
mv "${CFG}.new" "$CFG"
say "Changed: $result"
say "Config: $CFG (original kept as $BACKUP)"
rebuild
