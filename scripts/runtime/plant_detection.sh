#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# plant_detection.sh: YOLO plant detection with 3D boxes on the arm camera,
# inside go2-lidar3d. Works the same for the real Go2 (RealSense D435i boot
# service, docs/ai_and_vision/realsense.md) and the Gazebo simulation
# (SIM_MOVEIT=1 SIM_WRIST_CAMERA=d435i): both publish /camera/color/image_raw,
# /camera/aligned_depth_to_color/image_raw and /camera/color/camera_info.
#
# Uses nothing from the host but Docker: no Python environment, no ROS, no
# other workspace. Everything it needs is installed into go2-lidar3d by
# scripts/tools/detection_setup.sh (first run: a few minutes, needs internet).
#
#   ./scripts/runtime/plant_detection.sh                 run the detector (Ctrl-C stops it)
#   ./scripts/runtime/plant_detection.sh --rviz          + RViz with the annotated image and 3D boxes
#   ./scripts/runtime/plant_detection.sh --compressed    real robot over Wi-Fi: subscribe to the JPEG /
#                                                        PNG copies and decompress them here
#   ./scripts/runtime/plant_detection.sh --setup-only    install the dependencies and exit
#   ./scripts/runtime/plant_detection.sh --stop          stop a detector left running
#   ./scripts/runtime/plant_detection.sh -- -p confidence_threshold:=0.3
#                                                        extra ROS arguments for the node
#
# Publishes: /annotated_image, /detected_objects (2D), /detections_3d
# (vision_msgs/Detection3DArray, camera optical frame), /detected_objects_3d
# (RViz markers), /detected_objects_pointcloud.
# Parameters: config/detection/object_detection.yaml (DETECT_CONFIG=... for another file).
#
# Environment: CONTAINER_NAME=go2-lidar3d  DETECT_CONFIG=config/detection/object_detection.yaml
# The host Python environment variant is plant_detection_host.sh.
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
CONTAINER_NAME="${CONTAINER_NAME:-go2-lidar3d}"
SIM_CONTAINER="${SIM_CONTAINER:-go2-sim}"
DETECT_CONFIG="${DETECT_CONFIG:-${REPO_DIR}/config/detection/object_detection.yaml}"
WORK=/tmp/plant_detection          # inside the container
PYDEPS=/ws/.detect_pydeps
TORCH_DIR=/ws/.torch

RVIZ=0; COMPRESSED=0; SETUP_ONLY=0; STOP=0; NODE_ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --rviz) RVIZ=1 ;;
        --compressed) COMPRESSED=1 ;;
        --setup-only) SETUP_ONLY=1 ;;
        --stop) STOP=1 ;;
        --) shift; NODE_ARGS=("$@"); break ;;
        -h|--help) awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; exit 0 ;;
        *) echo "Unknown option: $1 (see --help)" >&2; exit 2 ;;
    esac
    shift
done

log() { echo "[detect] $*" >&2; }
die() { log "ERROR: $*"; exit 1; }

running() { docker ps --format '{{.Names}}' | grep -qx "$1"; }

stop_detector() {
    docker exec "${CONTAINER_NAME}" bash -c \
        'pkill -INT -f "object_detection_rgbd.nodes.object_detection_node" ; pkill -INT -f "plant_detection[.]rviz" ; pkill -INT -f "__node:=detect_republish_"' \
        >/dev/null 2>&1 || true
}

running "${CONTAINER_NAME}" \
    || die "container ${CONTAINER_NAME} is not running. Start a session first: ./scripts/runtime/wifi_nav.sh (robot) or ./scripts/runtime/wifi_nav.sh --sim"

if [ "${STOP}" = 1 ]; then
    stop_detector
    log "stopped"
    exit 0
fi
[ -f "${DETECT_CONFIG}" ] || die "no parameter file ${DETECT_CONFIG}"

"${REPO_DIR}/scripts/tools/detection_setup.sh"
if [ "${SETUP_ONLY}" = 1 ]; then
    log "setup complete"
    exit 0
fi

# The node's source and parameters, copied in fresh at every start.
docker exec -u root "${CONTAINER_NAME}" bash -c "rm -rf ${WORK} && mkdir -p ${WORK} && chmod 1777 ${WORK}"
docker cp -q "${REPO_DIR}/src/object_detection_rgbd/object_detection_rgbd" "${CONTAINER_NAME}:${WORK}/object_detection_rgbd"
docker cp -q "${DETECT_CONFIG}" "${CONTAINER_NAME}:${WORK}/object_detection.yaml"
docker cp -q "${REPO_DIR}/config/plant_detection.rviz" "${CONTAINER_NAME}:${WORK}/plant_detection.rviz"
docker exec -u root "${CONTAINER_NAME}" chmod -R a+rX "${WORK}"

# DDS settings. While the simulation runs, its own file: the camera topics come
# from it, and it is the same as the session file when wifi_nav.sh --sim started
# both. Otherwise the session file setup-robot-wifi.sh wrote in the container
# (/tmp/go2-wifi-cyclonedds.xml: this machine's address, the robot as peer).
# Not the environment of a running process, as ros_shell.sh does: the container
# shares the host's process list, and the simulation's MoveIt processes are on
# another ROS domain.
dds_env() {
    DDS=(-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e ROS_DOMAIN_ID="${GO2_RELAY_DOMAIN_ID:-64}")
    if running "${SIM_CONTAINER}" && docker exec "${SIM_CONTAINER}" test -f /tmp/sim-gen/cyclonedds.xml; then
        docker exec "${SIM_CONTAINER}" cat /tmp/sim-gen/cyclonedds.xml \
            | docker exec -i "${CONTAINER_NAME}" bash -c "cat > ${WORK}/cyclonedds.xml"
        DDS+=(-e CYCLONEDDS_URI="file://${WORK}/cyclonedds.xml")
        log "simulation running: DDS settings taken from ${SIM_CONTAINER}"
    elif docker exec "${CONTAINER_NAME}" test -f /tmp/go2-wifi-cyclonedds.xml; then
        DDS+=(-e CYCLONEDDS_URI=file:///tmp/go2-wifi-cyclonedds.xml)
    else
        log "WARN: no DDS settings found in ${CONTAINER_NAME}; using CycloneDDS defaults on domain ${GO2_RELAY_DOMAIN_ID:-64}"
    fi
}
dds_env

stop_detector
trap 'stop_detector' EXIT

ROS_ENV="source /opt/ros/humble/setup.bash; [ -f /ws/install/setup.bash ] && source /ws/install/setup.bash"

if [ "${COMPRESSED}" = 1 ]; then
    # Over Wi-Fi the raw images do not fit (docs/ai_and_vision/realsense.md): take the
    # driver's JPEG colour and PNG depth copies and decompress them in the container.
    log "decompressing /camera/color/image_raw/compressed and /camera/aligned_depth_to_color/image_raw/compressedDepth"
    docker exec -d "${DDS[@]}" "${CONTAINER_NAME}" bash -c "${ROS_ENV}
exec ros2 run image_transport republish compressed raw --ros-args -r __node:=detect_republish_color \
    -r in/compressed:=/camera/color/image_raw/compressed -r out:=/detect/color/image_raw"
    docker exec -d "${DDS[@]}" "${CONTAINER_NAME}" bash -c "${ROS_ENV}
exec ros2 run image_transport republish compressedDepth raw --ros-args -r __node:=detect_republish_depth \
    -r in/compressedDepth:=/camera/aligned_depth_to_color/image_raw/compressedDepth -r out:=/detect/aligned_depth/image_raw"
    NODE_ARGS=(-p image_topic:=/detect/color/image_raw -p depth_image_topic:=/detect/aligned_depth/image_raw "${NODE_ARGS[@]}")
fi

if [ "${RVIZ}" = 1 ]; then
    if [ -n "${DISPLAY:-}" ]; then
        xhost +local:docker >/dev/null 2>&1 || true
        docker exec -d "${DDS[@]}" -e DISPLAY="${DISPLAY}" -e QT_QPA_PLATFORM=xcb -e QT_X11_NO_MITSHM=1 \
            -e LIBGL_ALWAYS_SOFTWARE=1 "${CONTAINER_NAME}" bash -c "${ROS_ENV}
exec rviz2 -d ${WORK}/plant_detection.rviz"
        log "RViz started (fixed frame camera_color_optical_frame)"
    else
        log "WARN: no DISPLAY; --rviz skipped"
    fi
fi

EXTRA=""
[ "${#NODE_ARGS[@]}" -gt 0 ] && EXTRA="$(printf '%q ' "${NODE_ARGS[@]}")"

log "starting the detector in ${CONTAINER_NAME} (the model takes a few seconds to load; Ctrl-C stops it)"
TTY=(-i); [ -t 0 ] && TTY=(-it)
# PYTHONPATH: the pip packages of detection_setup.sh first (their numpy and cv2 go
# with PyTorch), then the node's source. Run from the hub folder, where yolov5s.pt is.
docker exec "${TTY[@]}" "${DDS[@]}" -e TORCH_HOME="${TORCH_DIR}" -e YOLOV5_CONFIG_DIR=/tmp/yolov5-config \
    -e PYTHONUNBUFFERED=1 -w "${TORCH_DIR}/hub" "${CONTAINER_NAME}" bash -c "${ROS_ENV}
export PYTHONPATH=${PYDEPS}:${WORK}:\${PYTHONPATH}
exec python3 -m object_detection_rgbd.nodes.object_detection_node --ros-args \
    --params-file ${WORK}/object_detection.yaml ${EXTRA}"
