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


def dump(path, doc):
    with open(path, "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=False)


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
    doc = load(path)
    p = gparams(doc, path)
    names = [n for n in p.get("plugins", []) if is_height_band(p, n)]
    for n in names:
        p["plugins"].remove(n)
        p.pop(n, None)
    dump(path, doc)
    print("removed global " + ", ".join(names))

elif cmd == "semantic-present":
    p = gparams(load(args[0]), args[0])
    sys.exit(0 if "semantic_layer" in p.get("plugins", []) else 1)

elif cmd == "semantic-add":
    path = args[0]
    doc = load(path)
    p = gparams(doc, path)
    plugins = p.setdefault("plugins", [])
    if "semantic_layer" not in plugins:
        at = plugins.index("inflation_layer") if "inflation_layer" in plugins else len(plugins)
        plugins.insert(at, "semantic_layer")
    p["semantic_layer"] = dict(SEMANTIC_GLOBAL)
    dump(path, doc)
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
            "step_over_height_max": 0.12, "lethal_height_min": 0.22,
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
                "# copy of the global costmap. Bands from the sim: step over <= 0.12 m (the Go2's\n"
                "# foot-lift ceiling, TODO: measure on the dog), lethal >= 0.22 m.\n")
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
