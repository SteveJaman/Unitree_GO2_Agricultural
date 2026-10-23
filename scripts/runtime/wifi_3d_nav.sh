#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# wifi_3d_nav.sh
#
# 3D "digital twin" session: the normal wifi_nav.sh session (relay,
# Point-LIO, Nav2, RViz, explorer, camera), plus a separate go2-3d container:
#   * RTAB-Map            dense 3D map of the run, saved to a .db file;
#                         coloured from the Go2 camera when it publishes
#   * YOLO11s             camera detections -> /yolo/detections, /yolo/obstacles
#   * Foxglove bridge     ws://<VM IP>:8765, live 3D view in the Foxglove app
#   * go2_click_to_goal   a point clicked in Foxglove becomes a Nav2 goal
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
#   ./scripts/runtime/wifi_3d_nav.sh --lidar-only    # no camera fusion, no YOLO
#   ./scripts/runtime/wifi_3d_nav.sh --camera        # require the Go2 camera
#   ./scripts/runtime/wifi_3d_nav.sh --no-yolo       # camera colour, no YOLO
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
TWIN_YOLO="${TWIN_YOLO:-1}"
SIM="false"
EXPORT_DB=""
PASSTHRU=()

while [ $# -gt 0 ]; do
    case "$1" in
        --rgbd)       TWIN_MODE="rgbd" ;;
        --lidar-only) TWIN_MODE="lidar" ;;
        --camera)     TWIN_MODE="camera" ;;
        --no-yolo)    TWIN_YOLO="0" ;;
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
    local deadline pid env_lines line key val uri path netmode host_ip mode yolo
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
            -e TWIN_YOLO="${TWIN_YOLO}" \
            -e TWIN_DEPTH_TOPIC -e TWIN_CAMINFO_TOPIC -e TWIN_CAMERA_TF -e TWIN_CAMERA_FRAME \
            -e TWIN_YOLO_MODEL -e TWIN_YOLO_RATE -e TWIN_YOLO_CLASSES -e TWIN_RTAB_ARGS \
            -v "${TWIN_DB_DIR}:/data" -v "${TWIN_LOG_DIR}:/logs" \
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
            host_ip="$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "${CONTAINER_NAME}" 2>/dev/null \
                | sed -n 's/^GO2_HOST_IP=//p' | head -n1)"
            host_ip="${host_ip:-$(hostname -I 2>/dev/null | awk '{print $1}')}"
            echo "================================================================"
            echo " 3D digital twin READY"
            echo "   RTAB-Map    : ${mode:-?} -> ${TWIN_DB_DIR}/${DB_NAME}"
            echo "   YOLO        : ${yolo}"
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
         "${NODES_DIR}/go2_click_to_goal.py" "${NODES_DIR}/go2_camera_info.py"; do
    [ -f "${f}" ] || die "${f} not found."
done
mkdir -p "${TWIN_DB_DIR}" "${TWIN_LOG_DIR}"

# A go2-3d left over from an interrupted run still holds that run's map.
stop_twin

T0="$(date +%s)"
DB_NAME="rtabmap-$(date +%Y%m%d-%H%M%S).db"

echo "================================================================"
echo " Go2 3D digital twin (wifi_nav.sh + RTAB-Map + ${TWIN_BRIDGE} bridge)"
echo "================================================================"
echo "  RTAB-Map    : ${TWIN_MODE}, cloud ${TWIN_CLOUD_TOPIC}$( [ "${TWIN_MODE}" != lidar ] && echo ", camera ${TWIN_RGB_TOPIC}" )"
echo "  YOLO        : $( [ "${TWIN_YOLO}" = 1 ] && { [ "${TWIN_MODE}" = auto ] || [ "${TWIN_MODE}" = camera ]; } && echo "on if the camera publishes" || echo off )"
echo "  Database    : ${TWIN_DB_DIR}/${DB_NAME}"
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
