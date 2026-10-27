#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# lidar_overlay.sh: camera-LiDAR fusion check.
#
# Runs go2_lidar_overlay.py inside go2-lidar3d next to a running wifi_nav.sh
# (or wifi_3d_nav.sh) session. It projects /lidar3d/registered onto the front
# camera and publishes /camera/lidar_overlay. View it with an RViz Image
# display (Reliability: Reliable) or Foxglove's Image panel. Points should sit
# on the objects they hit; edges should line up.
#
# Usage:
#   ./scripts/tools/lidar_overlay.sh            # real camera: /camera/camera_info from the relay
#   ./scripts/tools/lidar_overlay.sh --sim      # Gazebo camera, exact sim intrinsics
#   ./scripts/tools/lidar_overlay.sh -p offset_rpy:="[0.0, 0.05, 0.0]"   # extra node params
# Tune the camera mount live from another shell (go2sh):
#   ros2 param set /go2_lidar_overlay offset_rpy "[0.0, 0.05, 0.0]"
# Ctrl+C stops it.
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_DIR="$( cd "${SCRIPT_DIR}/../.." &> /dev/null && pwd )"
CONTAINER_NAME="${CONTAINER_NAME:-go2-lidar3d}"
NODE="${REPO_DIR}/scripts/nodes/go2_lidar_overlay.py"

PARAMS=()
INFO_FILE=""
while [ $# -gt 0 ]; do
    case "$1" in
        --sim) INFO_FILE="${REPO_DIR}/config/camera/sim_front_camera.yaml" ;;
        -p)    PARAMS+=(-p "$2"); shift ;;
        -h|--help) awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; exit 0 ;;
        *)     echo "unknown option $1 (see --help)" >&2; exit 1 ;;
    esac
    shift
done

docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}" \
    || { echo "ERROR: ${CONTAINER_NAME} is not running: start wifi_nav.sh first" >&2; exit 1; }

docker cp "${NODE}" "${CONTAINER_NAME}:/tmp/go2_lidar_overlay.py"
if [ -n "${INFO_FILE}" ]; then
    docker cp "${INFO_FILE}" "${CONTAINER_NAME}:/tmp/overlay_camera.yaml"
    PARAMS+=(-p camera_info_file:=/tmp/overlay_camera.yaml)
fi

args=""
for a in "${PARAMS[@]}"; do args+=" $(printf '%q' "$a")"; done
exec docker exec -it "${CONTAINER_NAME}" bash -c \
    "source /ws/scripts/go2-env.sh >/dev/null 2>&1; exec python3 /tmp/go2_lidar_overlay.py --ros-args${args}"
