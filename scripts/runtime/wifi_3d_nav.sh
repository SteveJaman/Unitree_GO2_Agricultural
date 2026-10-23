#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# wifi_3d_nav.sh
#
# 3D "digital twin" session: the normal wifi_nav.sh session (relay,
# Point-LIO, Nav2, RViz, explorer, camera), plus a separate go2-3d container:
#   * RTAB-Map            dense 3D map of the run, saved to a .db file;
#                         coloured from the Go2 camera when it publishes
#   * Foxglove bridge     ws://<VM IP>:8765, live 3D view in the Foxglove app
#   * go2_click_to_goal   a point clicked in Foxglove becomes a Nav2 goal
# and with --ai:
#   * YOLO11s             camera detections -> /yolo/detections, /yolo/obstacles
#   * VLM (Ollama llava)  camera view -> /vlm_decision (HYDROPONIC/BLOCKED/CLEAR)
#   * battery_manager     (in go2-lidar3d) < 15 %: pause the explorer, go to the
#                         dock (map origin); explorer blocked until 20 %
#
# wifi_nav.sh runs unchanged and every option not listed below is passed to
# it (--no-explore, --robot-ip, --sim, ...). RTAB-Map lives in its own
# container so the wifi_nav.sh teardown never kills it before it saves.
#
# One-time:  ./scripts/runtime/wifi_3d_nav.sh --build    (needs internet)
#
# Usage:
#   ./scripts/runtime/wifi_3d_nav.sh                 # explore + 3D map
#   ./scripts/runtime/wifi_3d_nav.sh --no-explore    # click-to-go only
#   ./scripts/runtime/wifi_3d_nav.sh --ai            # + YOLO, VLM, battery manager
#   ./scripts/runtime/wifi_3d_nav.sh --ai --no-yolo  # --ai without YOLO (also --no-vlm, --no-battery)
#   ./scripts/runtime/wifi_3d_nav.sh --resume        # continue the newest map (after a battery swap)
#   ./scripts/runtime/wifi_3d_nav.sh --save-location # (2nd terminal) label Foxglove clicks
#   ./scripts/runtime/wifi_3d_nav.sh --lidar-only    # no camera fusion (no YOLO/VLM)
#   ./scripts/runtime/wifi_3d_nav.sh --camera        # require the Go2 camera
#   ./scripts/runtime/wifi_3d_nav.sh --rgbd          # RealSense colour/depth instead
#   ./scripts/runtime/wifi_3d_nav.sh --rosbridge     # rosbridge :9090 instead
#   ./scripts/runtime/wifi_3d_nav.sh --no-bridge     # RTAB-Map only
#   ./scripts/runtime/wifi_3d_nav.sh --export [DB]   # newest (or DB) -> PLY
#   ./scripts/runtime/wifi_3d_nav.sh --stop          # stop go2-3d, save DB
#
# Environment:
#   TWIN_CLOUD_TOPIC=/lidar3d/registered  TWIN_BRIDGE_PORT=8765 (9090 rosbridge)
#   TWIN_RGB_TOPIC=/camera/image_raw   TWIN_CAMERA_WAIT=60 (s, camera auto-detect)
#   TWIN_CAMERA_TF="0.327 0 0.043 -1.5708 0 -1.5708"  base_link -> camera optical
#                                      frame (x y z yaw pitch roll; "none" = skip)
#   TWIN_CAMERA_HFOV_DEG=120 (68.75 with --sim)        approximate intrinsics
#   TWIN_YOLO_MODEL=/opt/yolo/yolo11s.pt  TWIN_YOLO_RATE=2  TWIN_YOLO_CLASSES=chair,...|all
#   TWIN_VLM_URL=http://localhost:11434/api/generate  TWIN_VLM_MODEL=llava:7b  TWIN_VLM_PERIOD=5
#   TWIN_BATTERY_LOW=15  TWIN_BATTERY_RESUME=20  TWIN_DOCK="0 0 0" (x y yaw in lidar3d_map)
#   TWIN_DEPTH_TOPIC / TWIN_CAMINFO_TOPIC                       (--rgbd)
#   TWIN_MAP_FRAME=lidar3d_map   TWIN_BASE_FRAME=base_link
#   TWIN_RTAB_ARGS="--Grid/CellSize 0.1"   extra RTAB-Map parameters
#   TWIN_EXPORT_ARGS="--voxel 0.02"        extra rtabmap-export options (--export)
#   TWIN_DB_DIR=$NAV2_WIFI_DIR/ws/maps/rtabmap   TWIN_LOG_DIR=$NAV2_WIFI_DIR/ws/log/3d
#   TWIN_START_TIMEOUT=300   TWIN_STOP_TIMEOUT=60 (seconds RTAB-Map gets to save)
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
TWIN_CLOUD_TOPIC="${TWIN_CLOUD_TOPIC:-/lidar3d/registered}"
TWIN_MAP_FRAME="${TWIN_MAP_FRAME:-lidar3d_map}"
TWIN_BASE_FRAME="${TWIN_BASE_FRAME:-base_link}"
TWIN_CAMERA_WAIT="${TWIN_CAMERA_WAIT:-60}"

# --- Parse args ------------------------------------------------------------
ACTION="run"
TWIN_MODE="auto"
TWIN_BRIDGE="foxglove"
AI="false"
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
        --ai)         AI="true" ;;
        --no-yolo|--no-vlm|--no-battery)
                      NO_AI+=("${1#--no-}") ;;
        --resume)     RESUME="true" ;;
        --save-location) ACTION="save-location" ;;
        --sim)        SIM="true"; PASSTHRU+=("$1") ;;
        --rosbridge)  TWIN_BRIDGE="rosbridge" ;;
        --no-bridge)  TWIN_BRIDGE="none" ;;
        --build)      ACTION="build" ;;
        --stop)       ACTION="stop" ;;
        --export)     ACTION="export"
                      if [ -n "${2:-}" ] && [ "${2#-}" = "$2" ]; then EXPORT_DB="$2"; shift; fi ;;
        --check|--clean|--setup-ssh)
                      ACTION="passthru"; PASSTHRU+=("$1") ;;
        --host-ip|--robot-ip|--iface)
                      PASSTHRU+=("$1")
                      if [ $# -ge 2 ]; then PASSTHRU+=("$2"); shift; fi ;;
        -h|--help)    awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; exit 0 ;;
        *)            PASSTHRU+=("$1") ;;
    esac
    shift
done

# --ai turns the three AI nodes on; --no-yolo / --no-vlm / --no-battery win.
ai_default() { [ "${AI}" = "true" ] && [[ " ${NO_AI[*]} " != *" $1 "* ]] && echo 1 || echo 0; }
TWIN_YOLO="${TWIN_YOLO:-$(ai_default yolo)}"
TWIN_VLM="${TWIN_VLM:-$(ai_default vlm)}"
TWIN_BATTERY="${TWIN_BATTERY:-$(ai_default battery)}"
TWIN_BATTERY_LOW="${TWIN_BATTERY_LOW:-15}"
TWIN_BATTERY_RESUME="${TWIN_BATTERY_RESUME:-20}"
TWIN_DOCK="${TWIN_DOCK:-0 0 0}"

if [ "${TWIN_MODE}" = rgbd ]; then
    TWIN_RGB_TOPIC="${TWIN_RGB_TOPIC:-/camera/color/image_raw}"
else
    TWIN_RGB_TOPIC="${TWIN_RGB_TOPIC:-/camera/image_raw}"
fi
# Gazebo camera: horizontal_fov 1.2 rad (gazebo/sim/build_model.py).
[ "${SIM}" = "true" ] && TWIN_CAMERA_HFOV_DEG="${TWIN_CAMERA_HFOV_DEG:-68.75}"
TWIN_CAMERA_HFOV_DEG="${TWIN_CAMERA_HFOV_DEG:-120}"

case "${TWIN_BRIDGE}" in
    foxglove)  TWIN_BRIDGE_PORT="${TWIN_BRIDGE_PORT:-8765}" ;;
    rosbridge) TWIN_BRIDGE_PORT="${TWIN_BRIDGE_PORT:-9090}" ;;
esac

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
    echo "[3d] Building ${TWIN_IMAGE} (RTAB-Map, bridges, CPU PyTorch + YOLO11s; ~3-4 GB download)..."
    docker build -t "${TWIN_IMAGE}" "${TWIN_DIR}/docker"
    echo "[3d] Built ${TWIN_IMAGE}."
}

do_export() {
    local db="${EXPORT_DB:-$(newest_db)}" dir name out
    [ -n "${db}" ] || die "no database in ${TWIN_DB_DIR} (pass one: --export <file.db>)"
    [ -f "${db}" ] || die "${db} not found."
    image_present || die "image ${TWIN_IMAGE} missing: run $0 --build"
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
    echo "[3d] Mesh it with:  python3 scripts/tools/export_tour.py <file.ply> --from-ply -o ${name}_mesh.ply"
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
        "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash 2>/dev/null; exec python3 -u /tmp/battery_manager.py --ros-args -p low_soc:=$(float "${TWIN_BATTERY_LOW}") -p resume_soc:=$(float "${TWIN_BATTERY_RESUME}") -p dock_x:=$(float "${dock_x:-0}") -p dock_y:=$(float "${dock_y:-0}") -p dock_yaw:=$(float "${dock_yaw:-0}") -p map_frame:=${TWIN_MAP_FRAME} -p robot_base_frame:=${TWIN_BASE_FRAME}" \
        > "${TWIN_LOG_DIR}/battery.log" 2>&1 &
    sleep 4
    docker exec "${CONTAINER_NAME}" pgrep -f battery_manager.py >/dev/null 2>&1
}

# PID of a ROS node of the session started after this script (so a node left
# from an earlier session in a surviving container is never picked).
session_ros_pid() {
    local age=$(( $(date +%s) - T0 ))
    docker exec -u root "${CONTAINER_NAME}" ps -eo pid=,etimes=,args= 2>/dev/null \
        | awk -v max="${age}" '$2 <= max && index($0, "--ros-args") {print $1; exit}' || true
}

# Start go2-3d on the same DDS settings (domain, RMW, CycloneDDS config) as the
# session's own ROS nodes. Runs in the background while wifi_nav.sh runs.
start_twin_when_ready() {
    local deadline pid env_lines line key val uri path netmode host_ip mode yolo vlm battery
    local env_args=() mounts=() net=()

    deadline=$(( $(date +%s) + TWIN_START_TIMEOUT ))
    echo "[3d] Waiting for the wifi_nav.sh session's ROS nodes (up to ${TWIN_START_TIMEOUT} s)..."
    pid=""
    while [ "$(date +%s)" -lt "${deadline}" ]; do
        pid="$(session_ros_pid)"
        [ -n "${pid}" ] && break
        sleep 3
    done
    if [ -z "${pid}" ]; then
        echo "[3d] ERROR: no ROS nodes in ${CONTAINER_NAME} after ${TWIN_START_TIMEOUT} s — 3D stack NOT started."
        return 1
    fi
    sleep 5   # let the session finish writing its DDS config

    env_lines="$(docker exec -u root "${CONTAINER_NAME}" cat "/proc/${pid}/environ" 2>/dev/null | tr '\0' '\n' \
        | grep -E '^(ROS_DOMAIN_ID|RMW_IMPLEMENTATION|CYCLONEDDS_URI|ROS_LOCALHOST_ONLY)=' || true)"
    while IFS= read -r line; do
        [ -n "${line}" ] || continue
        key="${line%%=*}"; val="${line#*=}"
        env_args+=(-e "${key}=${val}")
        [ "${key}" = CYCLONEDDS_URI ] && uri="${val}"
    done <<< "${env_lines}"

    # Safety first: the battery manager does not depend on go2-3d coming up.
    battery="off"
    if [ "${TWIN_BATTERY}" = 1 ]; then
        if start_battery_manager "${env_args[@]}"; then
            battery="on (< ${TWIN_BATTERY_LOW} %: dock at ${TWIN_DOCK}; log battery.log)"
            echo "[3d] Battery manager running in ${CONTAINER_NAME}"
        else
            battery="FAILED (see ${TWIN_LOG_DIR}/battery.log)"
            echo "[3d] WARN: battery manager did not start; see ${TWIN_LOG_DIR}/battery.log"
        fi
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
    echo "[3d] Starting ${TWIN_CONTAINER}: RTAB-Map (${TWIN_MODE}), bridge ${TWIN_BRIDGE}, click-to-goal..."
    if ! docker run -d --name "${TWIN_CONTAINER}" "${net[@]}" \
            --stop-timeout "${TWIN_STOP_TIMEOUT}" \
            --user "$(id -u):$(id -g)" -e HOME=/tmp/go2-3d-home -e ROS_LOG_DIR=/logs/ros \
            "${env_args[@]}" \
            -e TWIN_DB="/data/${DB_NAME}" -e TWIN_MODE="${TWIN_MODE}" \
            -e TWIN_BRIDGE="${TWIN_BRIDGE}" -e TWIN_BRIDGE_PORT="${TWIN_BRIDGE_PORT:-}" \
            -e TWIN_CLOUD_TOPIC="${TWIN_CLOUD_TOPIC}" -e TWIN_RGB_TOPIC="${TWIN_RGB_TOPIC}" \
            -e TWIN_MAP_FRAME="${TWIN_MAP_FRAME}" -e TWIN_BASE_FRAME="${TWIN_BASE_FRAME}" \
            -e TWIN_CAMERA_WAIT="${TWIN_CAMERA_WAIT}" -e TWIN_CAMERA_HFOV_DEG="${TWIN_CAMERA_HFOV_DEG}" \
            -e TWIN_YOLO="${TWIN_YOLO}" -e TWIN_VLM="${TWIN_VLM}" \
            -e TWIN_DEPTH_TOPIC -e TWIN_CAMINFO_TOPIC -e TWIN_CAMERA_TF -e TWIN_CAMERA_FRAME \
            -e TWIN_YOLO_MODEL -e TWIN_YOLO_RATE -e TWIN_YOLO_CLASSES -e TWIN_RTAB_ARGS \
            -e TWIN_VLM_URL -e TWIN_VLM_MODEL -e TWIN_VLM_PERIOD \
            -v "${TWIN_DB_DIR}:/data" -v "${TWIN_LOG_DIR}:/logs" \
            -v "${REPO_DIR}/config:/config" \
            -v "${TWIN_DIR}:/twin:ro" -v "${NODES_DIR}:/nodes:ro" \
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
            if [ "${TWIN_BRIDGE}" != none ]; then
                echo "   Foxglove    : Open connection -> $( [ "${TWIN_BRIDGE}" = foxglove ] && echo "Foxglove WebSocket" || echo "Rosbridge" ) -> ws://${host_ip}:${TWIN_BRIDGE_PORT}"
            fi
            echo "   3D panel    : display frame lidar3d_map; topics /rtabmap/cloud_map, /tf, /plan, /yolo/markers"
            echo "   Click-to-go : Publish -> Point on /clicked_point (top-down view for accuracy)"
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
    stop)   stop_twin; echo "[3d] Done."; exit 0 ;;
    save-location)
        twin_running || die "${TWIN_CONTAINER} is not running: start a session first (wifi_3d_nav.sh), then run this in a second terminal."
        mkdir -p "${REPO_DIR}/config"
        exec docker exec -it "${TWIN_CONTAINER}" bash -c \
            "source /opt/ros/humble/setup.bash && exec python3 -u /nodes/save_location.py --ros-args -p map_frame:=${TWIN_MAP_FRAME}" ;;
    passthru)
        if [[ " ${PASSTHRU[*]} " == *" --check "* ]]; then
            if image_present; then
                echo "[checks] 3D image ${TWIN_IMAGE} ................. OK"
            else
                echo "[checks] 3D image ${TWIN_IMAGE} ................. MISSING (run $0 --build)"
            fi
        fi
        [[ " ${PASSTHRU[*]} " == *" --clean "* ]] && stop_twin
        exec bash "${WIFI_NAV}" "${PASSTHRU[@]}" ;;
esac

image_present || die "image ${TWIN_IMAGE} missing: run $0 --build (one-time, needs internet)"
for f in "${TWIN_DIR}/start_inside.sh" "${TWIN_DIR}/yolo_node.py" \
         "${NODES_DIR}/go2_click_to_goal.py" "${NODES_DIR}/go2_camera_info.py" \
         "${NODES_DIR}/battery_manager.py" "${NODES_DIR}/vlm_reasoning_node.py" \
         "${NODES_DIR}/save_location.py"; do
    [ -f "${f}" ] || die "${f} not found."
done
mkdir -p "${TWIN_DB_DIR}" "${TWIN_LOG_DIR}" "${REPO_DIR}/config"

# A go2-3d left over from an interrupted run still holds that run's map.
stop_twin

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
echo " Go2 3D digital twin (wifi_nav.sh + RTAB-Map + ${TWIN_BRIDGE} bridge)"
echo "================================================================"
echo "  RTAB-Map    : ${TWIN_MODE}, cloud ${TWIN_CLOUD_TOPIC}$( [ "${TWIN_MODE}" != lidar ] && echo ", camera ${TWIN_RGB_TOPIC}" )"
on_if_camera() { [ "$1" = 1 ] && { [ "${TWIN_MODE}" = auto ] || [ "${TWIN_MODE}" = camera ]; } && echo "on if the camera publishes" || echo off; }
echo "  YOLO        : $(on_if_camera "${TWIN_YOLO}")"
echo "  VLM         : $(on_if_camera "${TWIN_VLM}")$( [ "${TWIN_VLM}" = 1 ] && echo " (${TWIN_VLM_MODEL:-llava:7b} at ${TWIN_VLM_URL:-http://localhost:11434/api/generate})" )"
echo "  Battery     : $( [ "${TWIN_BATTERY}" = 1 ] && echo "dock below ${TWIN_BATTERY_LOW} % (to ${TWIN_DOCK}), explorer blocked until ${TWIN_BATTERY_RESUME} %" || echo off )"
echo "  Database    : ${TWIN_DB_DIR}/${DB_NAME}  [${DB_NOTE}]"
echo "  Bridge      : ${TWIN_BRIDGE}$( [ "${TWIN_BRIDGE}" != none ] && echo " on port ${TWIN_BRIDGE_PORT}" )"
echo "  wifi_nav.sh : ${PASSTHRU[*]:-(defaults)}"
echo "  The 3D stack starts once the session's ROS nodes are up."
echo

WATCH_PID=""
CLEANED_UP="false"
cleanup() {
    [ "${CLEANED_UP}" = "true" ] && return 0
    CLEANED_UP="true"
    if [ -n "${WATCH_PID}" ]; then
        kill "${WATCH_PID}" 2>/dev/null || true
        wait "${WATCH_PID}" 2>/dev/null || true
    fi
    [ "${TWIN_BATTERY}" = 1 ] && kill_battery_manager
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

start_twin_when_ready &
WATCH_PID=$!

status=0
bash "${WIFI_NAV}" "${PASSTHRU[@]}" || status=$?
exit "${status}"
