#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# wifi_3d_nav.sh
#
# 3D "digital twin" session: the normal wifi_nav.sh session (relay,
# Point-LIO, Nav2, RViz, explorer, camera), plus a separate go2-3d container:
#   * RTAB-Map            dense 3D map of the run, saved to a .db file;
#                         coloured from the Go2 camera when it publishes
#   * Web dashboard       http://<host IP>:3000 (or http://localhost:3000 on this computer) (React + three.js, twin/webapp):
#                         3D map, robot, plan, camera, battery, click-to-go,
#                         over rosbridge ws://<host IP>:9090
#   * go2_click_to_goal   a point clicked in the dashboard becomes a Nav2 goal
#   * battery monitor     /battery_state (in go2-lidar3d)
#   * rooms + survey      room segmentation; the autonomous survey (explore,
#                         rooms, visit + name each, report, dock)
# and the AI (on by default, --no-ai turns it off):
#   * YOLO11s             camera detections -> /yolo/detections, /yolo/obstacles
#   * VLM (Ollama llava)  camera view -> /vlm_decision (HYDROPONIC/BLOCKED/CLEAR)
#   * battery dock        < 15 %: pause the explorer, go to the dock (map
#                         origin); explorer blocked until 20 %
#
# ONE COMMAND: with the dog standing at the dock,
#   ./scripts/runtime/wifi_3d_nav.sh
# builds (or refreshes) the go2-3d image when needed, starts Ollama and pulls
# the VLM model, starts the session, localizes at the dock, and runs the
# autonomous survey with no user input: the dog maps the area, splits it into
# rooms, names each, writes config/survey_report.json and returns to the dock.
#
# wifi_nav.sh runs unchanged and every option not listed below is passed to
# it (--robot-ip, --sim, ...). RTAB-Map lives in its own container so the
# wifi_nav.sh teardown never kills it before it saves.
#
# Usage:
#   ./scripts/runtime/wifi_3d_nav.sh                 # everything: AI + unattended survey
#   ./scripts/runtime/wifi_3d_nav.sh --manual        # AI on, but you start/steer (dashboard Auto Survey, Auto Explore, clicks)
#   ./scripts/runtime/wifi_3d_nav.sh --no-explore    # click-to-go only (no explorer, so no survey)
#   ./scripts/runtime/wifi_3d_nav.sh --no-ai         # 3D map only: no YOLO, VLM or battery docking
#   ./scripts/runtime/wifi_3d_nav.sh --no-yolo       # leave one AI node out (also --no-vlm, --no-battery,
#                                                    #   --no-inspect, --no-periscope)
#   ./scripts/runtime/wifi_3d_nav.sh --wrist-camera  # D1 wrist camera mounted: its depth cloud
#                                                    #   (/wrist_camera/depth/points) marks the costmap
#                      The AI also: adds /yolo/obstacles to the Nav2 costmap config
#                      (add-yolo-costmap-source.sh), VLM BLOCKED -> pause + dock,
#                      and auto-localization at the dock.
#   ./scripts/runtime/wifi_3d_nav.sh --no-localize   # no /initialpose at start
#   ./scripts/runtime/wifi_3d_nav.sh --no-build      # never build the image by itself (offline)
#   ./scripts/runtime/wifi_3d_nav.sh --resume        # continue the newest map (after a battery swap)
#   ./scripts/runtime/wifi_3d_nav.sh --save-location # (2nd terminal) label clicked points
#   ./scripts/runtime/wifi_3d_nav.sh --lidar-only    # no camera fusion (no YOLO/VLM)
#   ./scripts/runtime/wifi_3d_nav.sh --camera        # require the Go2 camera
#   ./scripts/runtime/wifi_3d_nav.sh --fusion        # calibrated camera instead: undistorted,
#                                                    #   coloured map, /camera/lidar_overlay check
#   ./scripts/runtime/wifi_3d_nav.sh --camera-obstacles  # + camera depth (LiDAR-scaled)
#                                                    #   as Nav2 obstacles: glass, legs
#   ./scripts/runtime/wifi_3d_nav.sh --rgbd          # RealSense colour/depth instead
#   ./scripts/runtime/wifi_3d_nav.sh --foxglove      # also foxglove_bridge :8765 (alternative viewer)
#   ./scripts/runtime/wifi_3d_nav.sh --no-web        # rosbridge only, no dashboard server
#   ./scripts/runtime/wifi_3d_nav.sh --no-bridge     # RTAB-Map only (no dashboard)
#   ./scripts/runtime/wifi_3d_nav.sh --export [DB]   # newest (or DB) -> PLY
#   ./scripts/runtime/wifi_3d_nav.sh --export --mesh # + solid Poisson mesh <db>_mesh.ply (dashboard "Mesh")
#   ./scripts/runtime/wifi_3d_nav.sh --view          # dashboard + saved meshes only, no robot (presentations)
#   ./scripts/runtime/wifi_3d_nav.sh --stop          # stop go2-3d, save DB
#   ./scripts/runtime/wifi_3d_nav.sh --build         # rebuild the image now
#   (--ai and --survey are the defaults now; still accepted.)
#
# Environment:
#   TWIN_WEB_PORT=3000  TWIN_ROSBRIDGE_PORT=9090  TWIN_FOXGLOVE_PORT=8765
#   TWIN_ARM=1          D1 arm in the dashboard + its Orbit Scan button
#                       (arm_state_bridge.py; 0 = off)
#   TWIN_CLOUD_TOPIC=/lidar3d/registered
#   TWIN_RGB_TOPIC=/camera/image_raw   TWIN_CAMERA_WAIT=180 (s, camera auto-detect)
#   TWIN_CAMERA_TF="0.327 0 0.043 -1.5708 0 -1.5708"  base_link -> camera optical
#                                      frame (x y z yaw pitch roll; "none" = skip)
#   TWIN_CAMERA_HFOV_DEG=120 (68.75 with --sim)        approximate intrinsics
#   TWIN_CAMERA_INFO_FILE=config/camera/front_camera.yaml       (--fusion, --camera-obstacles; with
#                            --sim: config/camera/sim_front_camera.yaml)
#   TWIN_YOLO_MODEL=/opt/yolo/yolo11s.pt  TWIN_YOLO_RATE=2  TWIN_YOLO_CLASSES=chair,...|all
#   TWIN_VLM_URL=http://localhost:11434/api/generate  TWIN_VLM_MODEL=llava:7b  TWIN_VLM_PERIOD=5
#   TWIN_BATTERY_LOW=15  TWIN_BATTERY_RESUME=20  TWIN_DOCK="0 0 0" (x y yaw in lidar3d_map)
#   TWIN_VLM_ACT=1 (0 = VLM only reports)  TWIN_VLM_CONFIRM=2 (BLOCKED answers in a row)
#   TWIN_ROOMS=1 (room segmentation)  TWIN_ROOM_DOOR_WIDTH=0.9  TWIN_ROOM_MIN_AREA=1.5
#   TWIN_SURVEY=1 (survey node)  TWIN_SURVEY_AUTO=1 (start it unattended; --manual = 0)
#   TWIN_AUTO_BUILD=1 (build a missing/outdated image; --no-build = 0)
#   TWIN_DEPTH_TOPIC / TWIN_CAMINFO_TOPIC                       (--rgbd)
#   TWIN_MAP_FRAME=lidar3d_map   TWIN_BASE_FRAME=base_link
#   TWIN_RTAB_ARGS="--Grid/CellSize 0.1"   extra RTAB-Map parameters
#   TWIN_EXPORT_ARGS="--voxel 0.02"        extra rtabmap-export options (--export)
#   TWIN_MESH_TRIANGLES=300000  TWIN_MESH_ARGS="--depth 10 --trim 0.08"   (--mesh)
#   TWIN_DB_DIR=$NAV2_WIFI_DIR/ws/maps/rtabmap   TWIN_LOG_DIR=$NAV2_WIFI_DIR/ws/log/3d
#   TWIN_START_TIMEOUT=300   TWIN_STOP_TIMEOUT=60 (seconds RTAB-Map gets to save)
#   TWIN_NODE_SETTLE=10      seconds the session's go2_odom_tf must be up before its
#                            DDS settings are copied (it is restarted once at start)
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_DIR="$( cd "${SCRIPT_DIR}/../.." &> /dev/null && pwd )"
NODES_DIR="${REPO_DIR}/scripts/nodes"
TWIN_DIR="${REPO_DIR}/twin"
WIFI_NAV="${SCRIPT_DIR}/wifi_nav.sh"

# --- Configuration ---------------------------------------------------------
NAV2_WIFI_DIR="${NAV2_WIFI_DIR:-$HOME/files/go2-nav2-wifi}"
CONTAINER_NAME="${CONTAINER_NAME:-go2-lidar3d}"
TWIN_CONTAINER="${TWIN_CONTAINER:-go2-3d}"
TWIN_IMAGE="${TWIN_IMAGE:-go2-3d:local}"
TWIN_DB_DIR="${TWIN_DB_DIR:-${NAV2_WIFI_DIR}/ws/maps/rtabmap}"
TWIN_LOG_DIR="${TWIN_LOG_DIR:-${NAV2_WIFI_DIR}/ws/log/3d}"
TWIN_START_TIMEOUT="${TWIN_START_TIMEOUT:-300}"
TWIN_STOP_TIMEOUT="${TWIN_STOP_TIMEOUT:-60}"
TWIN_NODE_SETTLE="${TWIN_NODE_SETTLE:-10}"
TWIN_CLOUD_TOPIC="${TWIN_CLOUD_TOPIC:-/lidar3d/registered}"
TWIN_MAP_FRAME="${TWIN_MAP_FRAME:-lidar3d_map}"
TWIN_BASE_FRAME="${TWIN_BASE_FRAME:-base_link}"
TWIN_CAMERA_WAIT="${TWIN_CAMERA_WAIT:-180}"

# --- Parse args ------------------------------------------------------------
ACTION="run"
TWIN_MODE="auto"
TWIN_CAMERA_OBSTACLES=0
TWIN_ROSBRIDGE="1"
TWIN_WEB="1"
TWIN_FOXGLOVE="0"
AI="true"            # one-and-done: YOLO + VLM + battery dock unless --no-ai
SURVEY_AUTO="1"      # ... and the unattended survey unless --manual
NO_AI=()
RESUME="false"
SIM="false"
EXPORT_DB=""
PASSTHRU=()

while [ $# -gt 0 ]; do
    case "$1" in
        --rgbd)       TWIN_MODE="rgbd" ;;
        --lidar-only) TWIN_MODE="lidar" ;;
        --camera)     TWIN_MODE="camera" ;;
        --fusion)     TWIN_MODE="fusion" ;;
        --camera-obstacles) TWIN_CAMERA_OBSTACLES=1 ;;
        --ai|--survey) ;;  # the defaults now (kept for old command lines)
        --no-ai)      AI="false" ;;
        --manual)     SURVEY_AUTO="0" ;;
        --no-build)   TWIN_AUTO_BUILD="0" ;;
        --no-explore|--plan|--nav)
                      SURVEY_AUTO="0"; PASSTHRU+=("$1") ;;   # no explorer: nothing for the survey to drive
        --wrist-camera) TWIN_WRIST_CAMERA=1 ;;
        --no-yolo|--no-vlm|--no-battery|--no-inspect|--no-periscope)
                      NO_AI+=("${1#--no-}") ;;
        --resume)     RESUME="true" ;;
        --no-localize) NO_LOCALIZE="true" ;;
        --save-location) ACTION="save-location" ;;
        --sim)        SIM="true"; PASSTHRU+=("$1") ;;
        --foxglove)   TWIN_FOXGLOVE="1" ;;
        --rosbridge)  ;;   # rosbridge is always on now (kept for old command lines)
        --no-web)     TWIN_WEB="0" ;;
        --no-bridge)  TWIN_ROSBRIDGE="0"; TWIN_WEB="0"; TWIN_FOXGLOVE="0" ;;
        --build)      ACTION="build" ;;
        --stop)       ACTION="stop" ;;
        --export)     ACTION="export"
                      if [ -n "${2:-}" ] && [ "${2#-}" = "$2" ]; then EXPORT_DB="$2"; shift; fi ;;
        --mesh)       MESH="true" ;;
        --view)       ACTION="view" ;;
        --check|--clean|--setup-ssh)
                      ACTION="passthru"; PASSTHRU+=("$1") ;;
        --host-ip|--robot-ip|--iface|--drive)
                      PASSTHRU+=("$1")
                      if [ $# -ge 2 ]; then PASSTHRU+=("$2"); shift; fi ;;
        -h|--help)    awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; exit 0 ;;
        *)            PASSTHRU+=("$1") ;;
    esac
    shift
done

# --ai turns the AI nodes on; --no-yolo / --no-vlm / --no-battery / --no-inspect win.
# inspect: inspection_mission.py (dashboard Inspect: walk to a 3D point, IK-aim the arm at it).
ai_default() { [ "${AI}" = "true" ] && [[ " ${NO_AI[*]} " != *" $1 "* ]] && echo 1 || echo 0; }
TWIN_YOLO="${TWIN_YOLO:-$(ai_default yolo)}"
TWIN_VLM="${TWIN_VLM:-$(ai_default vlm)}"
TWIN_BATTERY="${TWIN_BATTERY:-$(ai_default battery)}"
TWIN_INSPECT="${TWIN_INSPECT:-$(ai_default inspect)}"
TWIN_PERISCOPE="${TWIN_PERISCOPE:-$(ai_default periscope)}"
TWIN_WRIST_CAMERA="${TWIN_WRIST_CAMERA:-0}"
TWIN_ARM="${TWIN_ARM:-1}"
TWIN_BATTERY_LOW="${TWIN_BATTERY_LOW:-15}"
TWIN_BATTERY_RESUME="${TWIN_BATTERY_RESUME:-20}"
TWIN_DOCK="${TWIN_DOCK:-0 0 0}"
TWIN_VLM_ACT="${TWIN_VLM_ACT:-1}"
TWIN_VLM_CONFIRM="${TWIN_VLM_CONFIRM:-2}"
# Zero-touch localization at the dock whenever --ai or --resume is used.
if { [ "${AI}" = "true" ] || [ "${RESUME}" = "true" ]; } && [ "${NO_LOCALIZE:-false}" != "true" ]; then
    TWIN_AUTOLOCALIZE="${TWIN_AUTOLOCALIZE:-1}"
fi
TWIN_AUTOLOCALIZE="${TWIN_AUTOLOCALIZE:-0}"
TWIN_SURVEY_AUTO="${TWIN_SURVEY_AUTO:-${SURVEY_AUTO}}"
TWIN_AUTO_BUILD="${TWIN_AUTO_BUILD:-1}"

# No camera relay: nothing to wait for in auto mode.
[[ " ${PASSTHRU[*]} " == *" --no-camera "* ]] && [ "${TWIN_MODE}" = auto ] && TWIN_MODE="lidar"
if [ "${TWIN_MODE}" = rgbd ]; then
    TWIN_RGB_TOPIC="${TWIN_RGB_TOPIC:-/camera/color/image_raw}"
else
    TWIN_RGB_TOPIC="${TWIN_RGB_TOPIC:-/camera/image_raw}"
fi
# Gazebo camera: horizontal_fov 1.2 rad (gazebo/sim/build_model.py).
[ "${SIM}" = "true" ] && TWIN_CAMERA_HFOV_DEG="${TWIN_CAMERA_HFOV_DEG:-68.75}"
TWIN_CAMERA_HFOV_DEG="${TWIN_CAMERA_HFOV_DEG:-120}"

TWIN_WEB_PORT="${TWIN_WEB_PORT:-3000}"
TWIN_MESH_TRIANGLES="${TWIN_MESH_TRIANGLES:-300000}"   # browser-friendly (60 fps)
[ "${MESH:-false}" = "true" ] && [ "${ACTION}" = "run" ] && ACTION="export"
TWIN_ROSBRIDGE_PORT="${TWIN_ROSBRIDGE_PORT:-9090}"
TWIN_FOXGLOVE_PORT="${TWIN_FOXGLOVE_PORT:-8765}"

# ===========================================================================
# Helpers
# ===========================================================================
die() { echo "ERROR: $*" >&2; exit 1; }

twin_running()  { docker ps    --format '{{.Names}}' 2>/dev/null | grep -x "${TWIN_CONTAINER}" >/dev/null; }
twin_exists()   { docker ps -a --format '{{.Names}}' 2>/dev/null | grep -x "${TWIN_CONTAINER}" >/dev/null; }
image_present() { docker image inspect "${TWIN_IMAGE}" >/dev/null 2>&1; }

check_docker() {
    command -v docker >/dev/null 2>&1 || die "docker not found."
    docker ps >/dev/null 2>&1 || die "cannot talk to Docker daemon. Try: newgrp docker"
}

# Graceful stop: SIGTERM, RTAB-Map writes its database, then the container ends.
stop_twin() {
    if twin_running; then
        echo "[3d] Stopping ${TWIN_CONTAINER} (RTAB-Map saving its database, up to ${TWIN_STOP_TIMEOUT} s)..."
        docker stop -t "${TWIN_STOP_TIMEOUT}" "${TWIN_CONTAINER}" >/dev/null || true
    fi
}

newest_db() {
    ls -1t "${TWIN_DB_DIR}"/*.db 2>/dev/null | head -n1 || true
}

do_build() {
    [ -f "${TWIN_DIR}/docker/Dockerfile" ] || die "${TWIN_DIR}/docker/Dockerfile not found."
    echo "[3d] Building ${TWIN_IMAGE} (RTAB-Map, rosbridge, CPU PyTorch + YOLO11s, web dashboard; ~3-4 GB download)..."
    # Context is twin/: the image also builds the React dashboard (twin/webapp).
    docker build -t "${TWIN_IMAGE}" -f "${TWIN_DIR}/docker/Dockerfile" "${TWIN_DIR}"
    echo "[3d] Built ${TWIN_IMAGE}."
}

# The image bakes in only the Dockerfile and the dashboard (twin/webapp); the
# Python nodes are mounted. A source file newer than the image -> rebuild
# (Docker's layer cache keeps that to the changed stages).
image_stale() {
    local created
    created="$(docker image inspect -f '{{.Created}}' "${TWIN_IMAGE}" 2>/dev/null)" || return 1
    [ -n "$(find "${TWIN_DIR}/docker" "${TWIN_DIR}/webapp" \
              \( -name node_modules -o -name dist \) -prune -o \
              -type f -newermt "${created}" -print -quit 2>/dev/null)" ]
}

# Missing or outdated image: build it now (needs internet), unless --no-build.
ensure_image() {
    if ! image_present; then
        [ "${TWIN_AUTO_BUILD}" = 1 ] || die "image ${TWIN_IMAGE} missing: run $0 --build (one-time, needs internet)"
        echo "[3d] Image ${TWIN_IMAGE} missing: building it now (first run only; needs internet)."
        do_build
    elif image_stale; then
        if [ "${TWIN_AUTO_BUILD}" = 1 ]; then
            echo "[3d] Dashboard / Dockerfile changed since ${TWIN_IMAGE} was built: rebuilding (cached, usually < 1 min)..."
            do_build || echo "[3d] WARN: rebuild failed (offline?); using the existing image"
        else
            echo "[3d] NOTE: ${TWIN_IMAGE} is older than twin/webapp or twin/docker (--no-build: not rebuilt)"
        fi
    fi
}

# The VLM needs Ollama up with the model pulled. Start `ollama serve` if it is
# installed but not running, pull a missing model; never block the session.
ensure_ollama() {
    local url="${TWIN_VLM_URL:-http://localhost:11434/api/generate}" model="${TWIN_VLM_MODEL:-llava:7b}" base tags
    base="${url%/api/*}"
    command -v curl >/dev/null 2>&1 || return 0
    if ! tags="$(curl -s -m 3 "${base}/api/tags")"; then
        if [[ "${base}" == *://localhost* || "${base}" == *://127.0.0.1* ]] && command -v ollama >/dev/null 2>&1; then
            echo "[checks] Ollama not running: starting 'ollama serve' (log ${TWIN_LOG_DIR}/ollama.log)"
            nohup ollama serve > "${TWIN_LOG_DIR}/ollama.log" 2>&1 &
            local serve_pid=$!
            for _ in $(seq 1 15); do
                tags="$(curl -s -m 2 "${base}/api/tags")" && break
                kill -0 "${serve_pid}" 2>/dev/null || break   # serve exited (see ollama.log)
                sleep 1
            done
        fi
        if [ -z "${tags:-}" ]; then
            echo "[checks] WARN: Ollama not reachable at ${base}$(command -v ollama >/dev/null 2>&1 || echo " (not installed: curl -fsSL https://ollama.com/install.sh | sh)"); rooms are named from YOLO objects only"
            return 0
        fi
    fi
    if ! grep -q "\"${model}\"" <<< "${tags}"; then
        if command -v ollama >/dev/null 2>&1; then
            echo "[checks] Ollama has no ${model}: pulling it now (~4.7 GB, once)..."
            OLLAMA_HOST="${base}" ollama pull "${model}" \
                || { echo "[checks] WARN: 'ollama pull ${model}' failed (offline?); the VLM stays idle"; return 0; }
        else
            echo "[checks] WARN: Ollama at ${base} has no ${model}: run 'ollama pull ${model}' there"
            return 0
        fi
    fi
    echo "[checks] Ollama ${model} at ${base} ............ OK"
}

do_export() {
    local db="${EXPORT_DB:-$(newest_db)}" dir name out
    [ -n "${db}" ] || die "no database in ${TWIN_DB_DIR} (pass one: --export <file.db>)"
    [ -f "${db}" ] || die "${db} not found."
    ensure_image
    dir="$(cd "$(dirname "${db}")" && pwd)"
    name="$(basename "${db}" .db)"
    out="${dir}/${name}-export"
    mkdir -p "${out}"
    echo "[3d] Exporting ${db} -> ${out}/ (laser-scan point cloud, PLY)..."
    # shellcheck disable=SC2086
    docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp \
        -v "${dir}:/data" "${TWIN_IMAGE}" bash -c \
        "source /opt/ros/humble/setup.bash && rtabmap-export --scan ${TWIN_EXPORT_ARGS:-} --output_dir '/data/${name}-export' '/data/${name}.db'"
    ls -1 "${out}"
    if [ "${MESH}" != "true" ]; then
        echo "[3d] Solid mesh for the dashboard:  $0 --export --mesh"
        return 0
    fi

    # Poisson mesh (Open3D, in the image) -> <db>_mesh.ply next to the database,
    # where the dashboard's "Mesh" view finds it (served as maps/).
    local cloud="" f
    for f in "${out}"/*.ply; do   # the largest exported cloud
        [ -f "${f}" ] || continue
        if [ -z "${cloud}" ] || [ "$(stat -c %s "${f}")" -gt "$(stat -c %s "${cloud}")" ]; then cloud="${f}"; fi
    done
    [ -n "${cloud}" ] || die "rtabmap-export wrote no .ply in ${out}"
    echo "[3d] Meshing $(basename "${cloud}") -> ${dir}/${name}_mesh.ply (Poisson, <= ${TWIN_MESH_TRIANGLES} triangles; minutes for a big map)..."
    # shellcheck disable=SC2086
    docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp \
        -v "${dir}:/data" -v "${REPO_DIR}/scripts/tools:/tools:ro" "${TWIN_IMAGE}" \
        python3 /tools/export_tour.py "/data/${name}-export/$(basename "${cloud}")" --from-ply \
            -o "/data/${name}_mesh.ply" --max-triangles "${TWIN_MESH_TRIANGLES}" ${TWIN_MESH_ARGS:-}
    echo "[3d] Mesh ready: ${dir}/${name}_mesh.ply"
    echo "[3d] View it: dashboard -> 3D Scan (http://localhost:${TWIN_WEB_PORT}) during a session,"
    echo "     or without the robot: $0 --view   (docs/ai_and_vision/3d_nav.md#presentation-mesh)"
}

# Dashboard + saved meshes without a robot or session (presentations).
# Same served layout as start_inside.sh: app, maps/, config/. Ctrl+C stops.
do_view() {
    ensure_image
    twin_running && die "${TWIN_CONTAINER} is running: its dashboard already serves the meshes (port ${TWIN_WEB_PORT})."
    mkdir -p "${TWIN_DB_DIR}" "${REPO_DIR}/config"
    local meshes
    meshes="$(ls -1t "${TWIN_DB_DIR}"/*_mesh.ply 2>/dev/null | head -3 || true)"
    [ -n "${meshes}" ] || echo "[3d] NOTE: no *_mesh.ply in ${TWIN_DB_DIR} yet: run $0 --export --mesh first"
    echo "[3d] Dashboard (view only) on http://localhost:${TWIN_WEB_PORT}  ->  View: Mesh. Ctrl+C to stop."
    docker run --rm -it --name go2-3d-view --network host --user "$(id -u):$(id -g)" \
        -v "${TWIN_DB_DIR}:/data:ro" -v "${REPO_DIR}/config:/config:ro" -v "${TWIN_DIR}:/twin:ro" \
        "${TWIN_IMAGE}" bash -c '
            root=/opt/webapp
            [ /twin/webapp/dist/index.html -nt /opt/webapp/index.html ] && root=/twin/webapp/dist
            mkdir -p /tmp/view && for f in "$root"/*; do ln -s "$f" /tmp/view/; done
            ln -s /data /tmp/view/maps && ln -s /config /tmp/view/config
            exec python3 -m http.server '"${TWIN_WEB_PORT}"' --bind 0.0.0.0 --directory /tmp/view'
}

# Float ROS parameters reject "15"; make it "15.0".
float() { case "$1" in *.*|*[eE]*) echo "$1" ;; *) echo "$1.0" ;; esac; }

kill_battery_manager() {
    docker exec -u root "${CONTAINER_NAME}" pkill -f battery_manager.py >/dev/null 2>&1 || true
}

# battery_manager.py needs the unitree_go messages, so it runs in go2-lidar3d
# (like the camera relay). $@ = the session's DDS env as docker -e options.
start_battery_manager() {
    local dock_x dock_y dock_yaw
    read -r dock_x dock_y dock_yaw <<< "${TWIN_DOCK}"
    kill_battery_manager
    docker cp "${NODES_DIR}/battery_manager.py" "${CONTAINER_NAME}:/tmp/battery_manager.py" >/dev/null 2>&1 || return 1
    docker exec -u root "$@" "${CONTAINER_NAME}" bash -c \
        "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash 2>/dev/null; source /opt/go2-unitree-msgs/local_setup.bash 2>/dev/null; exec python3 -u /tmp/battery_manager.py --ros-args -p auto_dock:=$( [ "${TWIN_BATTERY}" = 1 ] && echo true || echo false ) -p low_soc:=$(float "${TWIN_BATTERY_LOW}") -p resume_soc:=$(float "${TWIN_BATTERY_RESUME}") -p dock_x:=$(float "${dock_x:-0}") -p dock_y:=$(float "${dock_y:-0}") -p dock_yaw:=$(float "${dock_yaw:-0}") -p map_frame:=${TWIN_MAP_FRAME} -p robot_base_frame:=${TWIN_BASE_FRAME}" \
        > "${TWIN_LOG_DIR}/battery.log" 2>&1 &
    sleep 4
    docker exec "${CONTAINER_NAME}" pgrep -f battery_manager.py >/dev/null 2>&1
}

# "PID UID" of the session's go2_odom_tf node (lidar3d_map -> base_link, the TF
# RTAB-Map needs), started after this script so a node left from an earlier
# session is never picked, and up for TWIN_NODE_SETTLE s because the session
# restarts it once while starting. go2-lidar3d shares the host PID namespace,
# so any --ros-args process would also match the simulator's and helpers.
session_ros_pid() {
    local age=$(( $(date +%s) - T0 ))
    docker exec -u root "${CONTAINER_NAME}" ps -eo pid=,uid=,etimes=,args= 2>/dev/null \
        | awk -v max="${age}" -v min="${TWIN_NODE_SETTLE}" '$3 <= max && $3 >= min && index($0, "--ros-args") \
              && index($0, "__node:=go2_odom_tf") {print $1, $2; exit}' || true
}

# DDS settings (one KEY=value per line) of a node, read as the node's own user:
# root in the container has no CAP_SYS_PTRACE, so it cannot read
# /proc/<pid>/environ of the go2 user's nodes.
node_dds_env() {   # $1 = pid, $2 = uid
    docker exec -u "$2" "${CONTAINER_NAME}" cat "/proc/$1/environ" 2>/dev/null | tr '\0' '\n' \
        | grep -E '^(ROS_DOMAIN_ID|RMW_IMPLEMENTATION|CYCLONEDDS_URI|ROS_LOCALHOST_ONLY)=' || true
}

# Start go2-3d on the same DDS settings (domain, RMW, CycloneDDS config) as the
# session's own ROS nodes. Runs in the background while wifi_nav.sh runs.
start_twin_when_ready() {
    local deadline pid uid node env_lines line key val uri path netmode host_ip mode yolo vlm battery
    local env_args=() mounts=() net=()

    deadline=$(( $(date +%s) + TWIN_START_TIMEOUT ))
    echo "[3d] Waiting for the wifi_nav.sh session's go2_odom_tf node (up to ${TWIN_START_TIMEOUT} s)..."
    env_lines=""
    while [ "$(date +%s)" -lt "${deadline}" ]; do
        node="$(session_ros_pid)"
        if [ -n "${node}" ]; then
            read -r pid uid <<< "${node}"
            env_lines="$(node_dds_env "${pid}" "${uid}")"
            grep -q '^ROS_DOMAIN_ID=' <<< "${env_lines}" && break
        fi
        sleep 3
    done
    # Without the session's domain go2-3d would sit alone on domain 0: the
    # bridge would list only go2-3d's own topics and never receive data.
    if ! grep -q '^ROS_DOMAIN_ID=' <<< "${env_lines}"; then
        echo "[3d] ERROR: could not read the session's DDS settings after ${TWIN_START_TIMEOUT} s — 3D stack NOT started."
        return 1
    fi

    while IFS= read -r line; do
        [ -n "${line}" ] || continue
        key="${line%%=*}"; val="${line#*=}"
        env_args+=(-e "${key}=${val}")
        [ "${key}" = CYCLONEDDS_URI ] && uri="${val}"
    done <<< "${env_lines}"

    # Safety first: the battery manager does not depend on go2-3d coming up.
    # It always runs (monitor: /battery_state for the dashboard); --ai adds docking.
    if start_battery_manager "${env_args[@]}"; then
        if [ "${TWIN_BATTERY}" = 1 ]; then
            battery="dock (< ${TWIN_BATTERY_LOW} %: to ${TWIN_DOCK}; log battery.log)"
        else
            battery="monitor only (/battery_state; docking with --ai)"
        fi
        echo "[3d] Battery manager running in ${CONTAINER_NAME}: ${battery}"
    else
        battery="FAILED (see ${TWIN_LOG_DIR}/battery.log)"
        echo "[3d] WARN: battery manager did not start; see ${TWIN_LOG_DIR}/battery.log"
    fi

    # A file-based CycloneDDS config is copied out and mounted at the same path.
    if [ -n "${uri:-}" ] && [[ "${uri}" != *"<"* ]] && [[ "${uri}" != *","* ]]; then
        path="${uri#file://}"
        if docker cp "${CONTAINER_NAME}:${path}" "${TWIN_LOG_DIR}/cyclonedds.xml" >/dev/null 2>&1; then
            mounts+=(-v "${TWIN_LOG_DIR}/cyclonedds.xml:${path}:ro")
        else
            echo "[3d] WARN: could not copy the DDS config ${path} from ${CONTAINER_NAME}"
        fi
    fi

    netmode="$(docker inspect -f '{{.HostConfig.NetworkMode}}' "${CONTAINER_NAME}" 2>/dev/null || true)"
    if [ "${netmode}" = host ]; then
        net=(--network host)
    else
        net=(--network "container:${CONTAINER_NAME}")
        echo "[3d] NOTE: ${CONTAINER_NAME} is not host-networked (${netmode}); sharing its network namespace."
    fi

    twin_exists && docker rm -f "${TWIN_CONTAINER}" >/dev/null 2>&1
    local camera_args=()
    if [ "${TWIN_MODE}" = fusion ] || [ "${TWIN_CAMERA_OBSTACLES}" = 1 ]; then
        camera_args=(-v "${TWIN_CAMERA_INFO_FILE}:/camera_cfg/camera.yaml:ro"
                     -e TWIN_CAMERA_INFO=/camera_cfg/camera.yaml
                     -e TWIN_CAMERA_OBSTACLES="${TWIN_CAMERA_OBSTACLES}")
    fi
    echo "[3d] Starting ${TWIN_CONTAINER} on DDS $(sed -n 's/^ROS_DOMAIN_ID=/domain /p' <<< "${env_lines}"): RTAB-Map (${TWIN_MODE}), rosbridge, dashboard, click-to-goal..."
    if ! docker run -d --name "${TWIN_CONTAINER}" "${net[@]}" \
            --stop-timeout "${TWIN_STOP_TIMEOUT}" \
            --user "$(id -u):$(id -g)" -e HOME=/tmp/go2-3d-home -e ROS_LOG_DIR=/logs/ros \
            "${env_args[@]}" \
            -e TWIN_DB="/data/${DB_NAME}" -e TWIN_MODE="${TWIN_MODE}" \
            -e TWIN_ROSBRIDGE="${TWIN_ROSBRIDGE}" -e TWIN_ROSBRIDGE_PORT="${TWIN_ROSBRIDGE_PORT}" \
            -e TWIN_WEB="${TWIN_WEB}" -e TWIN_WEB_PORT="${TWIN_WEB_PORT}" \
            -e TWIN_FOXGLOVE="${TWIN_FOXGLOVE}" -e TWIN_FOXGLOVE_PORT="${TWIN_FOXGLOVE_PORT}" \
            -e TWIN_CLOUD_TOPIC="${TWIN_CLOUD_TOPIC}" -e TWIN_RGB_TOPIC="${TWIN_RGB_TOPIC}" \
            -e TWIN_MAP_FRAME="${TWIN_MAP_FRAME}" -e TWIN_BASE_FRAME="${TWIN_BASE_FRAME}" \
            -e TWIN_CAMERA_WAIT="${TWIN_CAMERA_WAIT}" -e TWIN_CAMERA_HFOV_DEG="${TWIN_CAMERA_HFOV_DEG}" \
            -e TWIN_YOLO="${TWIN_YOLO}" -e TWIN_VLM="${TWIN_VLM}" \
            -e TWIN_DEPTH_TOPIC -e TWIN_CAMINFO_TOPIC -e TWIN_CAMERA_TF -e TWIN_CAMERA_FRAME \
            -e TWIN_YOLO_MODEL -e TWIN_YOLO_RATE -e TWIN_YOLO_CLASSES -e TWIN_RTAB_ARGS \
            -e TWIN_VLM_URL -e TWIN_VLM_MODEL -e TWIN_VLM_PERIOD \
            -e TWIN_VLM_ACT="${TWIN_VLM_ACT}" -e TWIN_VLM_CONFIRM="${TWIN_VLM_CONFIRM}" \
            -e TWIN_AUTOLOCALIZE="${TWIN_AUTOLOCALIZE}" -e TWIN_DOCK="${TWIN_DOCK}" \
            -e TWIN_ROOMS -e TWIN_ROOM_DOOR_WIDTH -e TWIN_ROOM_MIN_AREA \
            -e TWIN_SURVEY -e TWIN_SURVEY_AUTO="${TWIN_SURVEY_AUTO:-0}" -e TWIN_INSPECT="${TWIN_INSPECT}" \
            -e TWIN_PERISCOPE="${TWIN_PERISCOPE}" -e TWIN_WRIST_CAMERA_TF \
            -v "${TWIN_DB_DIR}:/data" -v "${TWIN_LOG_DIR}:/logs" \
            -v "${REPO_DIR}/config:/config" \
            -v "${TWIN_DIR}:/twin:ro" -v "${NODES_DIR}:/nodes:ro" "${camera_args[@]}" \
            "${mounts[@]}" \
            "${TWIN_IMAGE}" bash /twin/start_inside.sh >/dev/null; then
        echo "[3d] ERROR: docker run failed — 3D stack NOT started."
        return 1
    fi

    # Up to TWIN_CAMERA_WAIT s of this go to camera auto-detection in the container.
    for _ in $(seq 1 $(( TWIN_CAMERA_WAIT + 60 ))); do
        if docker logs "${TWIN_CONTAINER}" 2>&1 | grep -q '\[go2-3d\] READY'; then
            mode="$(docker logs "${TWIN_CONTAINER}" 2>&1 | sed -n 's/^\[go2-3d\] MODE //p' | tail -n1)"
            yolo="off"
            docker logs "${TWIN_CONTAINER}" 2>&1 | grep -q '^\[go2-3d\] yolo started' \
                && yolo="on (/yolo/detections, /yolo/obstacles, /yolo/markers, /yolo/image)"
            vlm="off"
            docker logs "${TWIN_CONTAINER}" 2>&1 | grep -q '^\[go2-3d\] vlm started' \
                && vlm="on (${TWIN_VLM_MODEL:-llava:7b} -> /vlm_decision; log vlm.log)"
            host_ip="$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "${CONTAINER_NAME}" 2>/dev/null \
                | sed -n 's/^GO2_HOST_IP=//p' | head -n1)"
            host_ip="${host_ip:-$(hostname -I 2>/dev/null | awk '{print $1}')}"
            echo "================================================================"
            echo " 3D digital twin READY"
            echo "   RTAB-Map    : ${mode:-?} -> ${TWIN_DB_DIR}/${DB_NAME}"
            echo "   YOLO        : ${yolo}"
            echo "   VLM         : ${vlm}"
            echo "   Battery     : ${battery}"
            if [ "${TWIN_SURVEY_AUTO}" = 1 ]; then
                echo "   Survey      : AUTO - starts once the explorer is up; hands off until the dog is back at the dock"
                echo "                 (stop: dashboard Auto Survey; report: config/survey_report.json)"
            else
                echo "   Survey      : on demand - dashboard 'Auto Survey'"
            fi
            if [ "${TWIN_WEB}" = 1 ]; then
                echo "   Dashboard   : http://${host_ip}:${TWIN_WEB_PORT}   (on this computer: http://localhost:${TWIN_WEB_PORT})"
                echo "   Click-to-go : dashboard toolbar 'Click-to-go', then click the floor"
            fi
            [ "${TWIN_ROSBRIDGE}" = 1 ] && echo "   rosbridge   : ws://${host_ip}:${TWIN_ROSBRIDGE_PORT}"
            [ "${TWIN_FOXGLOVE}" = 1 ] && echo "   Foxglove    : Open connection -> Foxglove WebSocket -> ws://${host_ip}:${TWIN_FOXGLOVE_PORT}"
            if [ "${TWIN_CAMERA_OBSTACLES}" = 1 ]; then
                echo "   Obstacles   : /camera/obstacle_points -> Nav2 (check: /camera/obstacle_depth image)"
                echo "                 Nav2 waits for the first camera obstacles before driving"
            fi
            if [ "${TWIN_MODE}" = fusion ]; then
                echo "   Fusion      : /rtabmap/cloud_map coloured from the calibrated camera"
                echo "                 /camera/lidar_overlay = LiDAR on the camera image (RViz/Foxglove Image)"
            fi
            echo "   Logs        : ${TWIN_LOG_DIR}/"
            echo "================================================================"
            return 0
        fi
        twin_running || break
        sleep 1
    done
    echo "[3d] ERROR: ${TWIN_CONTAINER} did not come up. Last log lines:"
    docker logs --tail 20 "${TWIN_CONTAINER}" 2>&1 | sed 's/^/    /'
    return 1
}

# ===========================================================================
# Actions
# ===========================================================================
[ -f "${WIFI_NAV}" ] || die "${WIFI_NAV} not found."
check_docker

case "${ACTION}" in
    build)  do_build; exit 0 ;;
    export) do_export; exit 0 ;;
    view)   do_view; exit 0 ;;
    stop)   stop_twin; echo "[3d] Done."; exit 0 ;;
    save-location)
        twin_running || die "${TWIN_CONTAINER} is not running: start a session first (wifi_3d_nav.sh), then run this in a second terminal."
        mkdir -p "${REPO_DIR}/config"
        exec docker exec -it "${TWIN_CONTAINER}" bash -c \
            "source /opt/ros/humble/setup.bash && exec python3 -u /nodes/save_location.py --ros-args -p map_frame:=${TWIN_MAP_FRAME}" ;;
    passthru)
        if [[ " ${PASSTHRU[*]} " == *" --check "* ]]; then
            if ! image_present; then
                echo "[checks] 3D image ${TWIN_IMAGE} ................. MISSING (built on the next run, or $0 --build)"
            elif image_stale; then
                echo "[checks] 3D image ${TWIN_IMAGE} ................. OUTDATED (rebuilt on the next run)"
            else
                echo "[checks] 3D image ${TWIN_IMAGE} ................. OK"
            fi
        fi
        [[ " ${PASSTHRU[*]} " == *" --clean "* ]] && stop_twin
        exec bash "${WIFI_NAV}" "${PASSTHRU[@]}" ;;
esac

mkdir -p "${TWIN_DB_DIR}" "${TWIN_LOG_DIR}" "${REPO_DIR}/config"
ensure_image
for f in "${TWIN_DIR}/start_inside.sh" "${TWIN_DIR}/yolo_node.py" "${TWIN_DIR}/image_compressor.py" \
         "${TWIN_DIR}/room_segmentation_node.py" "${TWIN_DIR}/survey_manager.py" \
         "${NODES_DIR}/go2_click_to_goal.py" "${NODES_DIR}/go2_camera_info.py" \
         "${NODES_DIR}/battery_manager.py" "${NODES_DIR}/vlm_reasoning_node.py" \
         "${NODES_DIR}/save_location.py" "${NODES_DIR}/auto_localize.py"; do
    [ -f "${f}" ] || die "${f} not found."
done

[ "${TWIN_VLM}" = 1 ] && ensure_ollama

if [ "${TWIN_MODE}" = fusion ] || [ "${TWIN_CAMERA_OBSTACLES}" = 1 ]; then
    [[ " ${PASSTHRU[*]} " == *" --no-camera "* ]] && die "--fusion / --camera-obstacles need the camera: drop --no-camera"
    if [[ " ${PASSTHRU[*]} " == *" --sim "* ]]; then
        TWIN_CAMERA_INFO_FILE="${TWIN_CAMERA_INFO_FILE:-${REPO_DIR}/config/camera/sim_front_camera.yaml}"
    else
        TWIN_CAMERA_INFO_FILE="${TWIN_CAMERA_INFO_FILE:-${REPO_DIR}/config/camera/front_camera.yaml}"
    fi
    [ -f "${TWIN_CAMERA_INFO_FILE}" ] || die "--fusion / --camera-obstacles need a camera calibration: ${TWIN_CAMERA_INFO_FILE} not found.
       Run ./scripts/tools/calibrate_camera.sh with a session up (docs/ai_and_vision/camera_lidar_fusion.md)."
    for f in go2_fusion_camera.py go2_lidar_overlay.py go2_camera_obstacles.py; do
        [ -f "${NODES_DIR}/${f}" ] || die "${NODES_DIR}/${f} not found."
    done
fi

# A go2-3d left over from an interrupted run still holds that run's map.
stop_twin

# --ai with YOLO: make sure Nav2's costmap reads /yolo/obstacles. Idempotent
# (a no-op once applied); a failure is reported but never blocks the session.
YOLO_COSTMAP="off"
if [ "${TWIN_YOLO}" = 1 ]; then
    echo "[3d] Adding /yolo/obstacles to the Nav2 costmap config (add-yolo-costmap-source.sh)..."
    if NAV2_WIFI_DIR="${NAV2_WIFI_DIR}" GO2_CONTAINER="${CONTAINER_NAME}" \
            bash "${REPO_DIR}/scripts/tools/add-yolo-costmap-source.sh" 2>&1 | sed 's/^/    /'; then
        YOLO_COSTMAP="on (Nav2 avoids YOLO obstacles; undo: add-yolo-costmap-source.sh --undo)"
    else
        YOLO_COSTMAP="FAILED (YOLO runs, Nav2 ignores it; see the messages above)"
        echo "[3d] WARN: costmap patch failed; continuing without YOLO obstacles in Nav2"
    fi
fi

# --wrist-camera: the D1 wrist camera's depth cloud also marks Nav2's costmap
# (add-wrist-costmap-source.sh, idempotent; --undo there removes it).
if [ "${TWIN_WRIST_CAMERA}" = 1 ]; then
    echo "[3d] Adding /wrist_camera/depth/points to the Nav2 costmap config (add-wrist-costmap-source.sh)..."
    NAV2_WIFI_DIR="${NAV2_WIFI_DIR}" GO2_CONTAINER="${CONTAINER_NAME}" \
        bash "${REPO_DIR}/scripts/tools/add-wrist-costmap-source.sh" 2>&1 | sed 's/^/    /' \
        || echo "[3d] WARN: wrist costmap patch failed; continuing without it"
fi

T0="$(date +%s)"
if [ "${RESUME}" = "true" ]; then
    # Continue the newest map: RTAB-Map opens the existing database, adds this
    # run as a new session and links it to the old map when it recognises a
    # place. The original is copied first (.bak-*, not matched by *.db).
    PREV_DB="$(newest_db)"
    [ -n "${PREV_DB}" ] || die "--resume: no database in ${TWIN_DB_DIR} to resume."
    DB_NAME="$(basename "${PREV_DB}")"
    cp -p "${PREV_DB}" "${PREV_DB}.bak-$(date +%Y%m%d-%H%M%S)"
    DB_NOTE="resumed (backup ${DB_NAME}.bak-*)"
else
    DB_NAME="rtabmap-$(date +%Y%m%d-%H%M%S).db"
    DB_NOTE="new"
fi

echo "================================================================"
echo " Go2 3D digital twin (wifi_nav.sh + RTAB-Map + web dashboard)"
echo "================================================================"
echo "  RTAB-Map    : ${TWIN_MODE}, cloud ${TWIN_CLOUD_TOPIC}$( [ "${TWIN_MODE}" != lidar ] && echo ", camera ${TWIN_RGB_TOPIC}" )"
[ -n "${TWIN_CAMERA_INFO_FILE:-}" ] && echo "  Calibration : ${TWIN_CAMERA_INFO_FILE}"
[ "${TWIN_CAMERA_OBSTACLES}" = 1 ] && echo "  Obstacles   : camera depth (LiDAR-scaled) -> Nav2 costmap source 'camera'"
# wifi_nav.sh adds (or, without this flag, removes) the costmap camera source.
export CAMERA_OBSTACLES="${TWIN_CAMERA_OBSTACLES}"
on_if_camera() { [ "$1" = 1 ] && { [ "${TWIN_MODE}" = auto ] || [ "${TWIN_MODE}" = camera ] || [ "${TWIN_MODE}" = fusion ]; } && echo "on if the camera publishes" || echo off; }
echo "  YOLO        : $(on_if_camera "${TWIN_YOLO}")"
echo "  YOLO → Nav2 : ${YOLO_COSTMAP}"
echo "  VLM         : $(on_if_camera "${TWIN_VLM}")$( [ "${TWIN_VLM}" = 1 ] && echo " (${TWIN_VLM_MODEL:-llava:7b} at ${TWIN_VLM_URL:-http://localhost:11434/api/generate}; $( [ "${TWIN_VLM_ACT}" = 1 ] && echo "${TWIN_VLM_CONFIRM}x BLOCKED -> pause + dock" || echo "report only" ))" )"
echo "  Localize    : $( [ "${TWIN_AUTOLOCALIZE}" = 1 ] && echo "auto: /initialpose at the dock (0 0 0) on start" || echo "manual" )"
echo "  Survey      : $( [ "${TWIN_SURVEY_AUTO:-0}" = 1 ] && echo "AUTO: explore, rooms, categorize each, report, dock (no user input)" || echo "on demand (dashboard Auto Survey)" )"
echo "  Battery     : $( [ "${TWIN_BATTERY}" = 1 ] && echo "dock below ${TWIN_BATTERY_LOW} % (to ${TWIN_DOCK}), explorer blocked until ${TWIN_BATTERY_RESUME} %" || echo "monitor only" )"
echo "  Inspect     : $( [ "${TWIN_INSPECT}" = 1 ] && echo "on: dashboard Inspect -> walk to the point -> IK arm scan (inspection.log)" || echo "off (--ai)" )"
echo "  Periscope   : $( [ "${TWIN_PERISCOPE}" = 1 ] && echo "on: dashboard Periscope Scan -> table edge -> arm periscope pose (table_search.log)" || echo "off (--ai)" ); wrist camera $( [ "${TWIN_WRIST_CAMERA}" = 1 ] && echo "in the costmap" || echo "not in the costmap (--wrist-camera)" )"
echo "  Database    : ${TWIN_DB_DIR}/${DB_NAME}  [${DB_NOTE}]"
echo "  Dashboard   : $( [ "${TWIN_WEB}" = 1 ] && echo "port ${TWIN_WEB_PORT}" || echo off ), rosbridge $( [ "${TWIN_ROSBRIDGE}" = 1 ] && echo "port ${TWIN_ROSBRIDGE_PORT}" || echo off )$( [ "${TWIN_FOXGLOVE}" = 1 ] && echo ", foxglove_bridge port ${TWIN_FOXGLOVE_PORT}" )"
echo "  wifi_nav.sh : ${PASSTHRU[*]:-(defaults)}"
echo "  The 3D stack starts once the session's ROS nodes are up."
echo

# D1 arm -> /d1_arm/joint_states and the dashboard's Orbit Scan button, on the
# host (arm_state_bridge.py; it waits for rosbridge and the arm by itself).
# Wired DDS when the arm port has a cable link, else through the Go2's bridge.
ARM_STATE_PID=""
start_arm_state() {
    [ "${TWIN_ARM}" = 1 ] || return 0
    local robot args=()
    # shellcheck disable=SC2016
    robot="$(bash -c 'source "$1"; [ "$(cat "/sys/class/net/${ARM_IFACE}/carrier" 2>/dev/null)" = 1 ] \
        || head -n1 "${GO2_IP_CACHE}" 2>/dev/null' _ "${SCRIPT_DIR}/go2_net.sh" 2>/dev/null || true)"
    [ -n "${robot}" ] && args=(--robot "${robot}")
    python3 -u "${NODES_DIR}/arm_state_bridge.py" "${args[@]}" --rosbridge "localhost:${TWIN_ROSBRIDGE_PORT}" \
        --orbit-log "${TWIN_LOG_DIR}/arm_orbit.log" > "${TWIN_LOG_DIR}/arm_state.log" 2>&1 &
    ARM_STATE_PID=$!
    echo "[3d] D1 arm state: ${robot:+through the Go2 at ${robot}}${robot:-wired DDS} (log ${TWIN_LOG_DIR}/arm_state.log)"
}

WATCH_PID=""
CLEANED_UP="false"
cleanup() {
    [ "${CLEANED_UP}" = "true" ] && return 0
    CLEANED_UP="true"
    if [ -n "${WATCH_PID}" ]; then
        kill "${WATCH_PID}" 2>/dev/null || true
        wait "${WATCH_PID}" 2>/dev/null || true
    fi
    kill_battery_manager
    if [ -n "${ARM_STATE_PID}" ]; then
        kill "${ARM_STATE_PID}" 2>/dev/null || true   # stops a running orbit scan first
        wait "${ARM_STATE_PID}" 2>/dev/null || true
    fi
    stop_twin
    if [ -f "${TWIN_DB_DIR}/${DB_NAME}" ]; then
        echo "[3d] Map saved: ${TWIN_DB_DIR}/${DB_NAME} ($(du -h "${TWIN_DB_DIR}/${DB_NAME}" | cut -f1))"
        echo "[3d] Export to PLY:  $0 --export"
    fi
}
trap cleanup EXIT
# Runs after wifi_nav.sh (which gets the same Ctrl+C) has finished its own teardown.
trap 'exit 130' INT
trap 'exit 143' TERM

mkdir -p "${TWIN_LOG_DIR}"
start_arm_state
start_twin_when_ready &
WATCH_PID=$!

status=0
bash "${WIFI_NAV}" "${PASSTHRU[@]}" || status=$?
exit "${status}"
