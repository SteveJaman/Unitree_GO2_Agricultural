#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# render_world.sh: still images of a Gazebo world for the docs, rendered by
# Gazebo itself (go2-sim image, same renderer as the simulation).
#
#   ./gazebo/worlds/render_world.sh [WORLD] [OUT_DIR]
#   ./gazebo/worlds/render_world.sh lab_office docs/images/lab_office
#   CAM_OVERVIEW="x y z r p y" CAM_INSIDE=... FOV_OVERVIEW=1.0 FOV_INSIDE=1.5 ./gazebo/worlds/render_world.sh WORLD DIR
#
# Two views, each from its own copy of the world with one camera that saves
# frames to disk:
#   OUT_DIR/world_overview.png  cutaway from above the south-west corner: no
#                               ceiling, walls cut to CUT_H (0.9 m) so nothing
#                               standing against a wall is hidden
#   OUT_DIR/world_inside.png    inside the open office at about head height,
#                               full walls and ceiling (what the dog is in)
# Gazebo runs until the camera saved FRAMES frames (meshes keep loading for a
# few seconds after the first one); the last frame is kept. Needs a desktop
# ($DISPLAY), like the simulation. Do not run it during a sim trial: it
# competes for the CPU. Furniture renders untextured: Harmonic does not read
# the models' Gazebo Classic material scripts.
# ---------------------------------------------------------------------------
set -euo pipefail

GZ_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD="${1:-lab_office}"
OUT="$(realpath -m "${2:-docs/images/${WORLD}}")"
IMAGE="${SIM_IMAGE:-go2-sim:local}"
CUT_H="${CUT_H:-0.9}"
[ -f "${GZ_DIR}/worlds/${WORLD}.sdf" ] || { echo "no world ${WORLD}" >&2; exit 1; }
mkdir -p "${OUT}"
tmp="$(mktemp -d)"
trap 'rm -rf "${tmp}"' EXIT

# world_copy CUTAWAY(0|1) "x y z roll pitch yaw" HFOV > world.sdf
world_copy() {
    python3 - "${GZ_DIR}/worlds/${WORLD}.sdf" "$1" "$2" "$3" "${CUT_H}" <<'PY'
import re, sys
src, cut, pose, fov, cut_h = sys.argv[1:]
s = open(src).read().replace("@RENDER_ENGINE@", "ogre2")
if cut == "1":
    # the ceiling is a model (lab_office) or a link (hydroponic_station)
    s = re.sub(r'\s*<model name="ceiling">.*?</model>', '', s, flags=re.S)
    s = re.sub(r'\s*<link name="ceiling">.*?</link>', '', s, flags=re.S)
    def cut_walls(w):
        w = re.sub(r'(<size>[\d.]+ [\d.]+ )([\d.]+)(</size>)', lambda m: m.group(1) + cut_h + m.group(3), w)
        return re.sub(r'(<pose>[-\d.]+ [-\d.]+ )([\d.]+)( )', lambda m: m.group(1) + str(float(cut_h) / 2) + m.group(3), w)
    walls = re.search(r'<model name="walls">.*?\n    </model>', s, flags=re.S)
    if walls:
        s = s.replace(walls.group(0), cut_walls(walls.group(0)))
    s = re.sub(r'<link name="wall_[^"]*">.*?</link>', lambda m: cut_walls(m.group(0)), s, flags=re.S)
camera = f"""    <model name="cam"><static>true</static><pose>{pose}</pose><link name="l">
      <sensor name="cam" type="camera"><update_rate>2</update_rate><always_on>true</always_on><topic>render/cam</topic>
        <camera><horizontal_fov>{fov}</horizontal_fov><image><width>1600</width><height>1000</height></image>
          <clip><near>0.05</near><far>60</far></clip>
          <save enabled="true"><path>/out/cam</path></save></camera></sensor></link></model>
"""
print(s.replace("  </world>\n</sdf>", camera + "  </world>\n</sdf>"), end="")
PY
}

# render NAME: run Gazebo on $tmp/NAME/world.sdf, keep the last frame as OUT/world_NAME.png
render() {
    local dir="${tmp}/$1" dev_args=() f
    [ -d /dev/dri ] && dev_args=(--device /dev/dri --group-add video)
    # As run_sim.sh: host network and IPC (X shared memory), the GPU if any, and
    # a gz-transport partition of its own so a running simulation is not touched.
    docker run --rm --network host --ipc host "${dev_args[@]}" -e GZ_PARTITION="render$$$1" -e GZ_IP=127.0.0.1 \
        -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -e GZ_SIM_RESOURCE_PATH=/sim/models/furniture:/sim/models/items -e HOME=/tmp \
        -v "${GZ_DIR}:/sim:ro" -v "${dir}:/w" -v "${dir}:/out" --user "$(id -u):$(id -g)" \
        -e RENDER_TIMEOUT="${RENDER_TIMEOUT:-300}" -e FRAMES="${FRAMES:-30}" "${IMAGE}" bash -c '
            gz sim -s -r --render-engine ogre2 /w/world.sdf > /w/gz.log 2>&1 &
            # a camera renders (and saves) only while its topic has a subscriber
            gz topic -e -t /render/cam > /dev/null 2>&1 &
            # software rendering: the first frame can take minutes
            for _ in $(seq 1 "$RENDER_TIMEOUT"); do
                [ "$(ls /out/cam 2>/dev/null | wc -l)" -ge "$FRAMES" ] && break
                sleep 1
            done
            pkill -f "gz topic" || true; pkill -f "gz sim" || true; sleep 2'
    f="$(find "${dir}/cam" -name '*.png' 2>/dev/null | sort -V | tail -1 || true)"
    [ -n "${f}" ] || { echo "no $1 frame; Gazebo log:" >&2; grep -v CustomMeshShape "${dir}/gz.log" | tail -20 >&2; exit 1; }
    cp "${f}" "${OUT}/world_$1.png"
    echo "wrote ${OUT}/world_$1.png"
}

mkdir -p "${tmp}/overview" "${tmp}/inside"
# Camera poses "x y z roll pitch yaw" (defaults: lab_office); other worlds set CAM_* / FOV_*.
world_copy 1 "${CAM_OVERVIEW:--10.5 -10.0 11.0 0 0.72 0.76}" "${FOV_OVERVIEW:-1.0}" > "${tmp}/overview/world.sdf"
world_copy 0 "${CAM_INSIDE:--1.4 2.2 1.0 0 0.22 2.906}" "${FOV_INSIDE:-1.5}" > "${tmp}/inside/world.sdf"
render overview
render inside
