#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# plant_detection_host.sh: the plant detector in a Python environment on THIS
# machine instead of go2-lidar3d. Optional: for a workstation that already has a
# virtual environment with PyTorch (the one the detector was first tested in).
# The standard way, which needs nothing on the host, is plant_detection.sh.
#
# This script installs nothing and creates no environment. If the environment
# or the CycloneDDS RMW is missing it says so and points to plant_detection.sh.
#
# Needs on the host:
#   * ROS 2 Humble with vision_msgs and cv_bridge, and CycloneDDS:
#       sudo apt install ros-humble-rmw-cyclonedds-cpp
#   * a virtual environment created with --system-site-packages that has torch
#     and the YOLOv5 Python packages (DETECT_VENV, default ~/venvs/odrgbd)
#
#   ./scripts/runtime/plant_detection_host.sh               run the detector (Ctrl-C stops it)
#   ./scripts/runtime/plant_detection_host.sh --rviz        + RViz on the host
#   ./scripts/runtime/plant_detection_host.sh --compressed  real robot over Wi-Fi: decompress the
#                                                           JPEG / PNG copies here
#   ./scripts/runtime/plant_detection_host.sh -- -p confidence_threshold:=0.3
#
# Same node, parameters (config/detection/object_detection.yaml) and topics as
# plant_detection.sh. DDS settings (domain 64, CycloneDDS) are copied from the
# running go2-lidar3d or go2-sim container.
#
# Environment: DETECT_VENV=~/venvs/odrgbd  DETECT_CONFIG=...  CONTAINER_NAME=go2-lidar3d
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DETECT_VENV="${DETECT_VENV:-$HOME/venvs/odrgbd}"
DETECT_CONFIG="${DETECT_CONFIG:-${REPO_DIR}/config/detection/object_detection.yaml}"
CONTAINER_NAME="${CONTAINER_NAME:-go2-lidar3d}"
SIM_CONTAINER="${SIM_CONTAINER:-go2-sim}"
DDS_XML=/tmp/plant-detection-host-cyclonedds.xml

RVIZ=0; COMPRESSED=0; NODE_ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --rviz) RVIZ=1 ;;
        --compressed) COMPRESSED=1 ;;
        --) shift; NODE_ARGS=("$@"); break ;;
        -h|--help) awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; exit 0 ;;
        *) echo "Unknown option: $1 (see --help)" >&2; exit 2 ;;
    esac
    shift
done

log() { echo "[detect-host] $*" >&2; }
# Missing prerequisites are not installed here: the container script needs none of them.
need() { log "ERROR: $*"; log "Use ./scripts/runtime/plant_detection.sh instead: it runs in go2-lidar3d and needs nothing on the host."; exit 1; }

[ -f /opt/ros/humble/setup.bash ] || need "ROS 2 Humble is not installed on this machine (/opt/ros/humble)"
[ -x "${DETECT_VENV}/bin/python" ] || need "no Python environment at ${DETECT_VENV} (DETECT_VENV)"
[ -f "${DETECT_CONFIG}" ] || { log "ERROR: no parameter file ${DETECT_CONFIG}"; exit 1; }

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1 \
    || need "CycloneDDS is not installed for ROS on the host (sudo apt install ros-humble-rmw-cyclonedds-cpp)"
for pkg in vision_msgs cv_bridge; do
    ros2 pkg prefix "${pkg}" >/dev/null 2>&1 || need "ROS package ${pkg} is not installed on the host"
done
"${DETECT_VENV}/bin/python" -c 'import torch' 2>/dev/null || need "${DETECT_VENV} has no torch"

running() { command -v docker >/dev/null 2>&1 && docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$1"; }

# The session's CycloneDDS file lives inside a container; copy its text to the host.
# The simulation's file first while it runs (the camera topics come from it).
# Both containers use the host network, so the addresses in it are this machine's.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID="${GO2_RELAY_DOMAIN_ID:-64}"
unset CYCLONEDDS_URI
if running "${SIM_CONTAINER}" && docker exec "${SIM_CONTAINER}" cat /tmp/sim-gen/cyclonedds.xml > "${DDS_XML}" 2>/dev/null && [ -s "${DDS_XML}" ]; then
    export CYCLONEDDS_URI="file://${DDS_XML}"
    log "simulation running: DDS settings from ${SIM_CONTAINER} (domain ${ROS_DOMAIN_ID})"
elif running "${CONTAINER_NAME}" && docker exec "${CONTAINER_NAME}" cat /tmp/go2-wifi-cyclonedds.xml > "${DDS_XML}" 2>/dev/null && [ -s "${DDS_XML}" ]; then
    export CYCLONEDDS_URI="file://${DDS_XML}"
    log "DDS settings from ${CONTAINER_NAME} (domain ${ROS_DOMAIN_ID})"
else
    rm -f "${DDS_XML}"
    log "WARN: neither ${CONTAINER_NAME} nor ${SIM_CONTAINER} is running; CycloneDDS defaults on domain ${ROS_DOMAIN_ID}"
fi

PIDS=()
cleanup() {
    trap '' INT TERM
    for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null || true; done
}
trap cleanup EXIT

if [ "${COMPRESSED}" = 1 ]; then
    ros2 pkg prefix compressed_depth_image_transport >/dev/null 2>&1 \
        || need "--compressed needs ros-humble-image-transport-plugins on the host"
    log "decompressing /camera/color/image_raw/compressed and /camera/aligned_depth_to_color/image_raw/compressedDepth"
    ros2 run image_transport republish compressed raw --ros-args -r __node:=detect_republish_color \
        -r in/compressed:=/camera/color/image_raw/compressed -r out:=/detect/color/image_raw &
    PIDS+=("$!")
    ros2 run image_transport republish compressedDepth raw --ros-args -r __node:=detect_republish_depth \
        -r in/compressedDepth:=/camera/aligned_depth_to_color/image_raw/compressedDepth -r out:=/detect/aligned_depth/image_raw &
    PIDS+=("$!")
    NODE_ARGS=(-p image_topic:=/detect/color/image_raw -p depth_image_topic:=/detect/aligned_depth/image_raw "${NODE_ARGS[@]}")
fi

if [ "${RVIZ}" = 1 ]; then
    if [ -n "${DISPLAY:-}" ] && command -v rviz2 >/dev/null 2>&1; then
        rviz2 -d "${REPO_DIR}/config/plant_detection.rviz" >/dev/null 2>&1 &
        PIDS+=("$!")
        log "RViz started (fixed frame camera_color_optical_frame)"
    else
        log "WARN: no DISPLAY or no rviz2 on the host; --rviz skipped"
    fi
fi

# shellcheck disable=SC1091
source "${DETECT_VENV}/bin/activate"
# Run from the torch hub folder: yolov5s.pt is looked for (and, the first time,
# downloaded) there. The node's source comes straight from this repository.
HUB_DIR="$(python -c 'import torch; print(torch.hub.get_dir())')"
mkdir -p "${HUB_DIR}"
cd "${HUB_DIR}"
export PYTHONPATH="${REPO_DIR}/src/object_detection_rgbd:${PYTHONPATH}" PYTHONUNBUFFERED=1
log "starting the detector with ${DETECT_VENV} (Ctrl-C stops it)"
python -m object_detection_rgbd.nodes.object_detection_node --ros-args \
    --params-file "${DETECT_CONFIG}" "${NODE_ARGS[@]}"
