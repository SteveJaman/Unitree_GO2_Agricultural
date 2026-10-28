#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# calibrate_camera.sh: intrinsic calibration of the Go2 front camera.
#
# Runs calibrate_camera.py inside go2-lidar3d against the live
# /camera/image_raw of a running wifi_nav.sh session, then copies the result
# to config/camera/front_camera.yaml, which wifi_nav.sh hands to the camera
# relay so it publishes /camera/camera_info.
#
# Usage (session running, checkerboard in hand):
#   ./scripts/tools/calibrate_camera.sh                       # 9x6 inner corners, 25 mm
#   ./scripts/tools/calibrate_camera.sh --cols 8 --rows 5 --square 0.030
#   ./scripts/tools/calibrate_camera.sh --samples 30
# Ctrl+C early calibrates with the views taken so far (needs at least 8).
# Any calibrate_camera.py option is passed through (--help lists them).
#
# Environment: CONTAINER_NAME=go2-lidar3d  CAMERA_CALIB=config/camera/front_camera.yaml
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_DIR="$( cd "${SCRIPT_DIR}/../.." &> /dev/null && pwd )"
CONTAINER_NAME="${CONTAINER_NAME:-go2-lidar3d}"
CAMERA_CALIB="${CAMERA_CALIB:-${REPO_DIR}/config/camera/front_camera.yaml}"
OUT_IN=/tmp/go2-camera-calib
STAMP="$(date +%Y%m%d-%H%M%S)"
RUN_DIR="${REPO_DIR}/config/camera/runs/${STAMP}"

die() { echo "ERROR: $*" >&2; exit 1; }

docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}" \
    || die "${CONTAINER_NAME} is not running: start ./scripts/runtime/wifi_nav.sh first"

docker exec -u root "${CONTAINER_NAME}" rm -rf "${OUT_IN}"
docker cp "${SCRIPT_DIR}/calibrate_camera.py" "${CONTAINER_NAME}:/tmp/calibrate_camera.py"

echo "Move the board slowly through the whole image: centre, edges, corners,"
echo "near and far, tilted left/right/up/down. Views are captured automatically."
echo
status=0
docker exec -it "${CONTAINER_NAME}" bash -c \
    "source /ws/scripts/go2-env.sh >/dev/null 2>&1; exec python3 /tmp/calibrate_camera.py --out ${OUT_IN} $*" \
    || status=$?

mkdir -p "${RUN_DIR}"
docker cp "${CONTAINER_NAME}:${OUT_IN}/." "${RUN_DIR}/" 2>/dev/null || true
[ -f "${RUN_DIR}/front_camera.yaml" ] || die "no calibration written (exit ${status}); views kept in ${RUN_DIR}"

if [ "${status}" -eq 0 ]; then
    cp "${RUN_DIR}/front_camera.yaml" "${CAMERA_CALIB}"
    echo "Installed ${CAMERA_CALIB}. Restart wifi_nav.sh to publish /camera/camera_info."
else
    echo "RMS error too high: kept in ${RUN_DIR}/front_camera.yaml, NOT installed."
    echo "Redo with more varied views, or install it by hand: cp ${RUN_DIR}/front_camera.yaml ${CAMERA_CALIB}"
fi
echo "Views and preview: ${RUN_DIR}/ (undistorted_preview.png: straight edges should look straight)"
