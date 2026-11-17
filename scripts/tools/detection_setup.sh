#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# detection_setup.sh: what the plant detector (src/object_detection_rgbd) needs
# inside go2-lidar3d. The image has no PyTorch, vision_msgs, cv_bridge or pip.
# scripts/runtime/plant_detection.sh runs this at every start; it is quick once
# everything is in place.
#
#   apt (in the container; a recreated container loses it, ~1 min to redo):
#       ros-humble-vision-msgs ros-humble-cv-bridge python3-pip
#       ros-humble-image-transport-plugins          (for --compressed)
#   pip, into /ws/.detect_pydeps (the /ws bind mount, so it survives a recreated
#   container; ~1 GB, a few minutes the first time):
#       CPU PyTorch (the go2-3d image's version) + the Python packages the YOLOv5
#       code imports. ultralytics is not pinned: the YOLOv5 code below is its
#       master branch and imports from the current ultralytics package.
#   /ws/.torch/hub: the YOLOv5 code (ultralytics/yolov5) and yolov5s.pt
#
# Only this process sets PYTHONPATH to /ws/.detect_pydeps: the newer numpy in
# there is not seen by Nav2 or anything else in the container.
#
# Usage:
#   ./scripts/tools/detection_setup.sh           install what is missing
#   ./scripts/tools/detection_setup.sh --check   only report (exit 1 if incomplete)
#   ./scripts/tools/detection_setup.sh --force   reinstall the pip packages
#
# Environment: CONTAINER_NAME=go2-lidar3d
# ---------------------------------------------------------------------------
set -eo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-go2-lidar3d}"
PYDEPS=/ws/.detect_pydeps
TORCH_DIR=/ws/.torch
# Change this when the package list below changes: the marker file carries it.
DEPS_VERSION=2
MODE=install
case "${1:-}" in
    --check) MODE=check ;;
    --force) MODE=force ;;
    -h|--help) awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; exit 0 ;;
    "") ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
esac

log() { echo "[detect-setup] $*" >&2; }
die() { log "ERROR: $*"; exit 1; }

docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}" \
    || die "container ${CONTAINER_NAME} is not running (./scripts/runtime/wifi_nav.sh, or wifi_nav.sh --sim, starts it)"

apt_present() {
    docker exec "${CONTAINER_NAME}" bash -c 'source /opt/ros/humble/setup.bash
ros2 pkg prefix vision_msgs && ros2 pkg prefix cv_bridge && ros2 pkg prefix compressed_depth_image_transport \
    && python3 -m pip --version' >/dev/null 2>&1
}
pip_present()   { docker exec "${CONTAINER_NAME}" test -f "${PYDEPS}/.installed-${DEPS_VERSION}"; }
model_present() { docker exec "${CONTAINER_NAME}" test -f "${TORCH_DIR}/hub/yolov5s.pt" -a -d "${TORCH_DIR}/hub/ultralytics_yolov5_master"; }

if [ "${MODE}" = check ]; then
    ok=0
    apt_present   && log "apt packages ............ OK" || { log "apt packages ............ missing"; ok=1; }
    pip_present   && log "PyTorch + YOLOv5 deps ... OK (${PYDEPS})" || { log "PyTorch + YOLOv5 deps ... missing"; ok=1; }
    model_present && log "YOLOv5 code + weights ... OK (${TORCH_DIR}/hub)" || { log "YOLOv5 code + weights ... missing"; ok=1; }
    exit "${ok}"
fi

if apt_present; then
    log "apt packages ............ OK"
else
    log "installing vision_msgs, cv_bridge, image transport plugins and pip in ${CONTAINER_NAME} (apt, ~1 min)..."
    # Same apt-key repair as realsense_host.sh: the ROS key in older images expired on 2025-06-01.
    docker exec -u root -e DEBIAN_FRONTEND=noninteractive "${CONTAINER_NAME}" bash -c '
set -o pipefail
upd() { apt-get update 2>&1 | tee /tmp/detect-apt-update.log >/dev/null; ! grep -qE "EXPKEYSIG|NO_PUBKEY|is not signed|invalid signature" /tmp/detect-apt-update.log; }
if ! upd; then
    echo "[detect-setup] ROS apt key expired: re-adding packages.ros.org with the renewed key"
    rm -f /etc/apt/sources.list.d/ros2*.list /etc/apt/sources.list.d/ros2*.sources
    k=/usr/share/keyrings/ros-archive-keyring.gpg u=https://raw.githubusercontent.com/ros/rosdistro/master/ros.key
    curl -fsSL "$u" -o "$k" 2>/dev/null || python3 -c "import urllib.request,sys; urllib.request.urlretrieve(sys.argv[1], sys.argv[2])" "$u" "$k" || exit 1
    echo "deb [arch=$(dpkg --print-architecture) signed-by=$k] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release; echo $VERSION_CODENAME) main" > /etc/apt/sources.list.d/ros2.list
    upd || { tail -5 /tmp/detect-apt-update.log; exit 1; }
fi
apt-get install -y -qq --no-install-recommends ros-humble-vision-msgs ros-humble-cv-bridge \
    ros-humble-image-transport-plugins python3-pip >/tmp/detect-apt-install.log 2>&1 \
    || { tail -5 /tmp/detect-apt-install.log; exit 1; }
' || die "apt install failed in ${CONTAINER_NAME} (needs internet)"
    apt_present || die "apt packages still missing after the install"
    log "apt packages installed"
fi

# /ws is owned by the host user; the container user must be able to write there.
docker exec -u root "${CONTAINER_NAME}" bash -c "mkdir -p ${PYDEPS} ${TORCH_DIR}/hub && chown -R \$(stat -c %u:%g /ws) ${PYDEPS} ${TORCH_DIR}" \
    || die "cannot create ${PYDEPS} in ${CONTAINER_NAME}"
OWNER="$(docker exec "${CONTAINER_NAME}" stat -c %u:%g /ws)"

if pip_present && [ "${MODE}" != force ]; then
    log "PyTorch + YOLOv5 deps ... OK (${PYDEPS})"
else
    log "installing CPU PyTorch and the YOLOv5 Python packages into ${PYDEPS} (~1 GB, a few minutes)..."
    # numpy < 2: ROS Humble's Python packages (cv_bridge) are built against numpy 1.x;
    # opencv-python >= 4.12 requires numpy 2.
    docker exec -u "${OWNER}" -e HOME=/tmp -e PYDEPS="${PYDEPS}" -e DEPS_VERSION="${DEPS_VERSION}" "${CONTAINER_NAME}" bash -c '
set -e
rm -rf "${PYDEPS:?}"/* "${PYDEPS}"/.installed-*
python3 -m pip install --no-cache-dir --quiet --target "$PYDEPS" \
    --extra-index-url https://download.pytorch.org/whl/cpu \
    "torch==2.5.1+cpu" "torchvision==0.20.1+cpu" \
    "numpy>=1.23,<2" "opencv-python<4.12" ultralytics \
    pandas seaborn requests tqdm pyyaml pillow psutil gitpython "setuptools<81"
PYTHONPATH="$PYDEPS" python3 -c "import numpy, cv2, torch, torchvision, pandas, ultralytics
print(\"[detect-setup] numpy\", numpy.__version__, \"cv2\", cv2.__version__, \"torch\", torch.__version__)"
touch "$PYDEPS/.installed-$DEPS_VERSION"
' || die "pip install failed in ${CONTAINER_NAME} (needs internet; rerun, or --force)"
    log "PyTorch + YOLOv5 deps installed"
fi

if model_present; then
    log "YOLOv5 code + weights ... OK (${TORCH_DIR}/hub)"
else
    log "fetching the YOLOv5 code and yolov5s.pt into ${TORCH_DIR}/hub..."
    # Run from the hub folder: the YOLOv5 code saves the weights in the working directory,
    # which is where the node looks for them (so later starts need no internet).
    docker exec -u "${OWNER}" -e HOME=/tmp -e TORCH_HOME="${TORCH_DIR}" -e PYTHONPATH="${PYDEPS}" \
        -e YOLOV5_CONFIG_DIR=/tmp/yolov5-config -w "${TORCH_DIR}/hub" "${CONTAINER_NAME}" python3 -c '
import torch
m = torch.hub.load("ultralytics/yolov5", "yolov5s", pretrained=True, trust_repo=True)
print("[detect-setup] yolov5s:", len(m.names), "classes")
' || die "could not fetch YOLOv5 (needs internet)"
    model_present || die "YOLOv5 code or yolov5s.pt missing in ${TORCH_DIR}/hub after the download"
    log "YOLOv5 code + weights ready"
fi
