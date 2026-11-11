#!/usr/bin/env bash
# add-traversability-layers.sh — the traversability costmap layout for the real Go2
# (wifi_3d_nav.sh --semantic). EDITS THE EXTERNAL go2-nav2-wifi REPO, not this one.
#
#   Global costmap (go2_nav2/config/lidar3d_planner.yaml):
#       obstacle, semantic, inflation           (applied)
#       obstacle, inflation                     (--undo; standard Nav2)
#   Local costmap (go2_nav2/launch/lidar3d_controller.launch.py copies the global
#   costmap with deepcopy; the patch merges go2_nav2/config/lidar3d_local_layers.yaml
#   on top, local only):
#       obstacle, height band (0.12 / 0.22 m, histogram floor), semantic, inflation
#   go2_nav2/setup.py installs the new lidar3d_local_layers.yaml.
#
# Legacy cleanup: a HeightBandLayer in the GLOBAL costmap (the 2026-11-06 experiment,
# 0.15 / 0.25 m bands) is removed on apply and on --undo, and never restored; the
# file as it was is kept once as lidar3d_planner.yaml.pre-legacy-heightband.
# Everything else is backed up as *.pre-traversability and --undo restores it
# byte for byte. Safe to run twice either way.
#
# Usage:
#   ./scripts/tools/add-traversability-layers.sh            # apply
#   ./scripts/tools/add-traversability-layers.sh --undo     # restore (no-op when not applied)
#   ./scripts/tools/add-traversability-layers.sh --check    # print the state, change nothing
# Environment:
#   TARGET_REPO=path   the go2-nav2-wifi clone (default $NAV2_WIFI_DIR, else ~/files/go2-nav2-wifi)
#   GO2_CONTAINER=go2-lidar3d   container that rebuilds go2_nav2 after a change
#   SKIP_BUILD=1       never rebuild (also implied by a .mock-go2-nav2-wifi file in TARGET_REPO)
set -euo pipefail

TARGET_REPO="${TARGET_REPO:-${NAV2_WIFI_DIR:-$HOME/files/go2-nav2-wifi}}"
CONTAINER="${GO2_CONTAINER:-go2-lidar3d}"
PKG="${TARGET_REPO}/go2_nav2"
CFG="${PKG}/config/lidar3d_planner.yaml"
LAUNCH="${PKG}/launch/lidar3d_controller.launch.py"
SETUP="${PKG}/setup.py"
LOCAL="${PKG}/config/lidar3d_local_layers.yaml"
BK=".pre-traversability"
LEGACY_BK="${CFG}.pre-legacy-heightband"
MARK="add-traversability-layers.sh"

say()  { printf '\033[1;32m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m!!\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31mERROR:\033[0m %s\n' "$*" >&2; exit 1; }

MODE="apply"
case "${1:-}" in
    "") ;;
    --undo) MODE="undo" ;;
    --check) MODE="check" ;;
    -h|--help) awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; exit 0 ;;
    *) die "unknown option '$1' (use --undo, --check or no option)" ;;
esac

for f in "$CFG" "$LAUNCH" "$SETUP"; do
    [ -f "$f" ] || die "$f not found (TARGET_REPO=${TARGET_REPO}: set it to your go2-nav2-wifi clone)."
done
python3 -c 'import yaml' 2>/dev/null || die "python3-yaml is required (sudo apt install python3-yaml)."

MOCK=0
[ -f "${TARGET_REPO}/.mock-go2-nav2-wifi" ] && MOCK=1
[ "${SKIP_BUILD:-0}" = 1 ] && MOCK=1

banner() {
    warn "This modifies the EXTERNAL repo ${TARGET_REPO} (go2-nav2-wifi), not Unitree_GO2_Agricultural:"
    warn "  ${CFG#${TARGET_REPO}/}, ${LAUNCH#${TARGET_REPO}/}, ${SETUP#${TARGET_REPO}/}, ${LOCAL#${TARGET_REPO}/}"
    warn "  backups next to them (*${BK}, *.pre-legacy-heightband); undo with --undo."
    [ "$MOCK" = 1 ] && warn "  mock target: go2_nav2 is not rebuilt."
    return 0
}

rebuild() {
    if [ "$MOCK" = 1 ]; then
        say "Skipping the go2_nav2 rebuild (mock target or SKIP_BUILD=1)."
        return 0
    fi
    if command -v docker >/dev/null && docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
        say "Rebuilding go2_nav2 inside $CONTAINER..."
        docker exec "$CONTAINER" bash -c 'source /opt/ros/humble/setup.bash && cd /ws && colcon build --symlink-install --base-paths src/go2_nav2 --packages-select go2_nav2' >/dev/null
    else
        warn "$CONTAINER not running: the change applies at the next session start (wifi_nav.sh rebuilds)."
        warn "If it does not, rebuild inside the container: cd /ws && colcon build --symlink-install --packages-select go2_nav2"
    fi
}

# Python helpers: one entry point, a sub-command per edit. Everything else stays text.
py() {
    python3 - "$@" <<'PY'
import os
import re
import sys

import yaml

cmd, args = sys.argv[1], sys.argv[2:]
MARK = "add-traversability-layers.sh"


def load(path):
    return yaml.safe_load(open(path))


# The planner YAML is edited as text, never re-dumped: other patches mark their lines
# with comments (nav2_camera_source.py, nav2_voxel_layer.py) to undo them byte for
# byte, and a YAML dump would drop those comments. Each edit is checked by loading
# the result before it is written.
def indent_of(line):
    return len(line) - len(line.lstrip())


def block_end(lines, start):
    ind = indent_of(lines[start])
    end = start + 1
    while end < len(lines) and (not lines[end].strip() or lines[end].lstrip().startswith("#")
                                or indent_of(lines[end]) > ind):
        end += 1
    while end > start + 1 and not lines[end - 1].strip():
        end -= 1
    return end


def params_block(lines, path):
    """(start, end, key_indent) of global_costmap.global_costmap.ros__parameters children."""
    top = next((i for i, l in enumerate(lines) if l.rstrip() == "global_costmap:"), None)
    if top is None:
        sys.exit(f"ERROR: no global_costmap section in {path}")
    rp = next((i for i in range(top, block_end(lines, top)) if lines[i].strip() == "ros__parameters:"), None)
    if rp is None:
        sys.exit(f"ERROR: no global_costmap ros__parameters in {path}")
    end = block_end(lines, rp)
    child = next(indent_of(lines[i]) for i in range(rp + 1, end) if lines[i].strip())
    return rp + 1, end, child


def key_line(lines, start, end, key, ind):
    return next((i for i in range(start, end)
                 if indent_of(lines[i]) == ind and re.match(re.escape(key) + r":(\s|$)", lines[i].strip())), None)


def plugin_items(lines, start, end, ind):
    """('flow', line) or ('block', [item line indexes]) for the plugins key."""
    k = key_line(lines, start, end, "plugins", ind)
    if k is None:
        sys.exit("ERROR: no plugins list in global_costmap")
    if "[" in lines[k]:
        return "flow", k
    # Items may sit at the key's own indent ("plugins:\n- a", PyYAML's style) or deeper.
    items, i = [], k + 1
    while i < len(lines) and (not lines[i].strip() or lines[i].lstrip().startswith("#")
                              or (re.match(r"\s*-\s", lines[i]) and indent_of(lines[i]) >= ind)):
        if lines[i].strip().startswith("-"):
            items.append(i)
        i += 1
    return "block", items


def set_flow(line, names):
    return re.sub(r"\[.*\]", "[" + ", ".join(names) + "]", line, count=1)


def write_checked(path, lines, check):
    text = "\n".join(lines)
    check(gparams(yaml.safe_load(text), path))
    open(path, "w").write(text)


def gparams(doc, path):
    try:
        return doc["global_costmap"]["global_costmap"]["ros__parameters"]
    except (KeyError, TypeError):
        sys.exit(f"ERROR: no global_costmap.global_costmap.ros__parameters in {path}")


def is_height_band(params, name):
    return str((params.get(name) or {}).get("plugin", "")).endswith("HeightBandLayer")


SEMANTIC_GLOBAL = {
    "plugin": "semantic_traversability_layer/SemanticCostmapLayer",
    "enabled": True, "topic": "/semantic/obstacles", "max_range": 15.0, "confidence_min": 0.7,
    "avoid_cost": 254, "caution_cost": 200, "step_over_cost": 0,
}

if cmd == "legacy-present":                  # exit 0 when a global HeightBandLayer exists
    p = gparams(load(args[0]), args[0])
    sys.exit(0 if any(is_height_band(p, n) for n in p.get("plugins", [])) else 1)

elif cmd == "legacy-remove":
    path = args[0]
    p = gparams(load(path), path)
    names = [n for n in p.get("plugins", []) if is_height_band(p, n)]
    lines = open(path).read().split("\n")
    for n in names:
        start, end, ind = params_block(lines, path)
        kind, items = plugin_items(lines, start, end, ind)
        if kind == "flow":
            lines[items] = set_flow(lines[items], [x for x in p["plugins"] if x != n])
        else:
            for i in items:
                if lines[i].strip().lstrip("-").strip().strip("'\"") == n:
                    del lines[i]
                    break
        start, end, ind = params_block(lines, path)
        k = key_line(lines, start, end, n, ind)
        if k is not None:
            del lines[k:block_end(lines, k)]
        p["plugins"] = [x for x in p["plugins"] if x != n]

    def check(q):
        assert not any(is_height_band(q, x) for x in q["plugins"]) and not any(x in q for x in names)
    write_checked(path, lines, check)
    print("removed global " + ", ".join(names))

elif cmd == "semantic-present":
    p = gparams(load(args[0]), args[0])
    sys.exit(0 if "semantic_layer" in p.get("plugins", []) else 1)

elif cmd == "semantic-add":
    path = args[0]
    p = gparams(load(path), path)
    plugins = list(p.get("plugins", []))
    lines = open(path).read().split("\n")
    start, end, ind = params_block(lines, path)
    if "semantic_layer" not in plugins:
        at = plugins.index("inflation_layer") if "inflation_layer" in plugins else len(plugins)
        plugins.insert(at, "semantic_layer")
        kind, items = plugin_items(lines, start, end, ind)
        if kind == "flow":
            lines[items] = set_flow(lines[items], plugins)
        else:
            infl = next((i for i in items if lines[i].strip().lstrip("-").strip().strip("'\"") == "inflation_layer"),
                        None)
            ref = infl if infl is not None else items[-1]
            item = lines[ref][: lines[ref].index("-")] + "- semantic_layer"
            lines.insert(infl if infl is not None else items[-1] + 1, item)
    start, end, ind = params_block(lines, path)
    if key_line(lines, start, end, "semantic_layer", ind) is None:
        pad = " " * ind
        child = pad + "  "
        block = [pad + "semantic_layer:"] + [
            child + f"{k}: {str(v).lower() if isinstance(v, bool) else v}" for k, v in SEMANTIC_GLOBAL.items()]
        infl = key_line(lines, start, end, "inflation_layer", ind)
        at = infl if infl is not None else end
        lines[at:at] = block

    def check(q):
        assert "semantic_layer" in q["plugins"] and q["semantic_layer"] == SEMANTIC_GLOBAL, q.get("semantic_layer")
    write_checked(path, lines, check)
    print("global plugins: " + ", ".join(plugins))

elif cmd == "launch-patch":
    path = args[0]
    text = open(path).read()
    if MARK in text:
        print("launch file already patched")
        sys.exit(0)
    rx = re.compile(r"^(?P<ind>[ \t]*)local = copy\.deepcopy\(yaml\.safe_load\(stream\)\['global_costmap'\]"
                    r"\['global_costmap'\]\)[ \t]*\n", re.M)
    m = rx.search(text)
    if not m:
        sys.exit("ERROR: the deepcopy line was not found in " + path +
                 " (expected: local = copy.deepcopy(yaml.safe_load(stream)['global_costmap']['global_costmap']))")
    i = m.group("ind")
    # The block sits right after the deepcopy, still inside its `with open(...)` block,
    # and opens its own file; dedent it to the with-statement's level.
    base = i[:-4] if i.endswith("    ") else i
    block = "\n".join(base + l if l else "" for l in [
        f"# {MARK}: local-only layers (height band, semantic) merged on top of the",
        "# copy of the global costmap; undo with add-traversability-layers.sh --undo.",
        "local_layers = pkg + '/config/lidar3d_local_layers.yaml'",
        "if os.path.exists(local_layers):",
        "    with open(local_layers) as stream:",
        "        extra = yaml.safe_load(stream)['local_costmap_layers']",
        "    layers = local['ros__parameters']",
        "    plugins = layers['plugins']",
        "    for name, layer in extra.items():  # in file order, before inflation_layer",
        "        layers[name] = layer",
        "        if name in plugins:",
        "            plugins.remove(name)",
        "        at = plugins.index('inflation_layer') if 'inflation_layer' in plugins else len(plugins)",
        "        plugins.insert(at, name)",
    ]) + "\n"
    text = text[:m.end()] + block + text[m.end():]
    if "import os\n" not in text:
        text = "import os\n" + text
    open(path, "w").write(text)
    compile(text, path, "exec")
    print("launch file patched")

elif cmd == "setup-patch":
    path, entry = args[0], "config/lidar3d_local_layers.yaml"
    text = open(path).read()
    if entry in text:
        print("setup.py already lists " + entry)
        sys.exit(0)
    rx = re.compile(r'^(?P<ind>[ \t]*)(?P<q>["\'])config/lidar3d_planner\.yaml(?P=q),[ \t]*\n', re.M)
    m = rx.search(text)
    if not m:
        sys.exit("ERROR: no \"config/lidar3d_planner.yaml\", line in " + path + " data_files")
    line = f'{m.group("ind")}{m.group("q")}{entry}{m.group("q")},  # {MARK}\n'
    text = text[:m.end()] + line + text[m.end():]
    open(path, "w").write(text)
    compile(text, path, "exec")
    print("setup.py installs " + entry)

elif cmd == "write-local":
    path = args[0]
    doc = {"local_costmap_layers": {
        "height_band_layer": {
            "plugin": "semantic_traversability_layer/HeightBandLayer",
            "enabled": True, "topic": "/lidar3d/obstacle_points",
            "ground_z": 0.0, "ground_tolerance": 0.02, "floor_estimation": "histogram",
            "step_over_height_max": float(os.environ.get("TRAV_STEP_OVER", "0.12")),
            "lethal_height_min": 0.22,
            "min_range": 0.25, "max_range": 5.0, "step_over_cost": 40, "caution_cost": 200,
        },
        "semantic_layer": {
            "plugin": "semantic_traversability_layer/SemanticCostmapLayer",
            "enabled": True, "topic": "/semantic/obstacles", "max_range": 5.0, "confidence_min": 0.7,
            "avoid_cost": 254, "caution_cost": 200, "step_over_cost": 0,
        },
    }}
    with open(path, "w") as f:
        f.write(f"# Written by Unitree_GO2_Agricultural scripts/tools/{MARK}; removed by --undo.\n"
                "# Local-costmap-only layers, merged by lidar3d_controller.launch.py on top of the\n"
                "# copy of the global costmap. Step over <= TRAV_STEP_OVER (wifi_3d_nav.sh: 0.03 m\n"
                "# on the robot, measured; 0.12 m in the sim), lethal >= 0.22 m.\n")
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=False)
    print("wrote " + path)

elif cmd == "state":
    cfg, launch, setup, local = args
    p = gparams(load(cfg), cfg)
    print("global plugins : " + ", ".join(p.get("plugins", [])))
    print("legacy band    : " + ("present" if any(is_height_band(p, n) for n in p.get("plugins", [])) else "none"))
    print("launch patched : " + ("yes" if MARK in open(launch).read() else "no"))
    print("setup.py lists : " + ("yes" if "lidar3d_local_layers.yaml" in open(setup).read() else "no"))
    print("local layers   : " + (", ".join(load(local)["local_costmap_layers"]) if os.path.exists(local) else "none"))
else:
    sys.exit("unknown helper " + cmd)
PY
}

backup_once() { [ -f "$1${BK}" ] || cp -p "$1" "$1${BK}"; }

legacy_cleanup() {
    if py legacy-present "$CFG"; then
        [ -f "$LEGACY_BK" ] || cp -p "$CFG" "$LEGACY_BK"
        say "Legacy cleanup: $(py legacy-remove "$CFG") (original kept as ${LEGACY_BK##*/}; never restored)"
        return 0
    fi
    return 1
}

if [ "$MODE" = check ]; then
    say "State of ${TARGET_REPO}:"
    py state "$CFG" "$LAUNCH" "$SETUP" "$LOCAL"
    exit 0
fi

banner
changed=0

if [ "$MODE" = undo ]; then
    for f in "$CFG" "$LAUNCH" "$SETUP"; do
        if [ -f "$f${BK}" ]; then
            mv -f "$f${BK}" "$f"
            say "Restored ${f#${TARGET_REPO}/}"
            changed=1
        fi
    done
    if [ -f "$LOCAL" ]; then
        rm -f "$LOCAL"
        say "Removed ${LOCAL#${TARGET_REPO}/}"
        changed=1
    fi
    legacy_cleanup && changed=1
    [ "$changed" = 1 ] || say "Nothing to undo: ${TARGET_REPO} has the standard layout."
    py state "$CFG" "$LAUNCH" "$SETUP" "$LOCAL"
    [ "$changed" = 1 ] && rebuild
    exit 0
fi

# Apply. The legacy band goes first, so the *.pre-traversability backup (what --undo
# restores) is the clean global costmap.
legacy_cleanup && changed=1
if ! py semantic-present "$CFG"; then
    backup_once "$CFG"
    say "$(py semantic-add "$CFG")"
    changed=1
fi
if ! grep -q "$MARK" "$LAUNCH"; then
    backup_once "$LAUNCH"
    say "$(py launch-patch "$LAUNCH")"
    changed=1
fi
if ! grep -q "lidar3d_local_layers.yaml" "$SETUP"; then
    backup_once "$SETUP"
    say "$(py setup-patch "$SETUP")"
    changed=1
fi
before="$(cat "$LOCAL" 2>/dev/null || true)"
py write-local "$LOCAL" >/dev/null
[ "$(cat "$LOCAL")" = "$before" ] || { say "Wrote ${LOCAL#${TARGET_REPO}/}"; changed=1; }

[ "$changed" = 1 ] || say "Already applied: nothing changed."
py state "$CFG" "$LAUNCH" "$SETUP" "$LOCAL"
[ "$changed" = 1 ] && rebuild
exit 0
