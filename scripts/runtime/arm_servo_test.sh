#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# arm_servo_test.sh: hold the D1 arm's tool at a set distance from an object the
# RealSense sees (yolo_depth_tracker.py -> /object_distance -> distance_servo.py).
#
# Three terminals:
#   1)  ARM_JOINT_SIGNS="-1 1 1 1 1 -1" ./scripts/runtime/arm_servo_test.sh
#         MoveIt 2 + RViz + moveit_d1_bridge on the REAL arm (arm_moveit.sh --real),
#         NO inspection mission: the arm moves to the start pose and holds it.
#         Prints the commands for 2) and 3).
#   2)  ./scripts/runtime/arm_servo_test.sh tracker [-p target_class:=bottle] [-p show:=true]
#         YOLO + depth on the camera (domain 64) -> /object_distance on MoveIt's domain 78.
#   2') or, instead of 2): the test branch's RealSense 3D detector (YOLO11s, 3D boxes):
#       ./scripts/runtime/arm_servo_test.sh detector [--class bottle] [--rviz]
#         plant_detection.sh in go2-lidar3d -> /detections_3d -> detections_to_distance.py
#         -> /object_distance on domain 78.
#   3)  ./scripts/runtime/arm_servo_test.sh servo [-p dry_run:=true]
#         distance_servo.py on domain 78: steps the tool along X to hold 0.50 m.
#
#   ./scripts/runtime/arm_servo_test.sh --mock      # 1) on the mock arm: nothing moves
#
# The D435i plugged into THIS computer instead of the dog (testing the logic):
#   0)  ./scripts/runtime/arm_servo_test.sh camera                 # the driver, on loopback
#   2)  ./scripts/runtime/arm_servo_test.sh detector --local --class bottle --rviz
#       (or: tracker --local ...)   then 1) and 3) as usual
#   ... --no-rviz / --robot IP                       # passed on to arm_moveit.sh
#
# Start pose (SERVO_START_JOINTS, rad, Joint1..Joint6; default "0 0.25 0.30 0 -0.55 0"):
# the flange 0.34 m ahead of the arm base and 0.32 m up, level, so the servo's travel
# (0.10 m back, 0.15 m forward) is reachable both ways. Joint2 at 90 deg is not usable:
# it puts the upper arm flat forward and the flange 0.15 m BELOW the arm base, pointing
# down into the dog's back (and 1.57 is Joint2's limit).
#
# Where things run (why the tracker is in a container):
#   * MoveIt, RViz, the bridge, distance_servo.py: this computer, ROS_DOMAIN_ID=78,
#     ROS_LOCALHOST_ONLY=1, the default RMW (FastDDS), as arm_moveit.sh.
#   * The camera: the robot's go2-realsense service, ROS_DOMAIN_ID=64, CycloneDDS, sent
#     to this computer's Wi-Fi address (realsense_host.sh).
#   * yolo_depth_tracker.py: in a go2-3d:local container (YOLO + CycloneDDS), with one
#     Cyclone config for both domains: 64 on the Wi-Fi with the robot as a peer, 78 on
#     loopback, which the host's localhost-only FastDDS on 78 talks to.
#
# Geometry: the arm is mounted facing the dog's rear and the camera is on the dog's body,
# so moving the arm does not change the measured distance. The servo walks to its travel
# limit and stops there unless the object (or the dog) moves: move the object by hand to
# see it follow. distance_servo.py's limits: docs/hardware/arm_servo_test.md.
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ARM_DOMAIN="${ARM_ROS_DOMAIN_ID:-78}"
CAM_DOMAIN="${GO2_RELAY_DOMAIN_ID:-64}"
START_JOINTS="${SERVO_START_JOINTS:-0 0.25 0.30 0 -0.55 0}"
TRACKER_IMAGE="${SERVO_TRACKER_IMAGE:-go2-3d:local}"
# The camera service's compressed colour topic carried nothing (2026-11-18); raw ~10 Hz.
MODEL_DIR="${SERVO_MODEL_DIR:-$HOME/.cache/go2-yolo}"
ME="./scripts/runtime/arm_servo_test.sh"
# --local (or SERVO_LOCAL_CAMERA=1): the D435i plugged into THIS computer (arm_servo_test.sh
# camera); the camera's domain then runs on loopback too, and the dog is not involved.
LOCAL="${SERVO_LOCAL_CAMERA:-0}"
CAMERA_IMAGE="${SERVO_CAMERA_IMAGE:-go2-realsense-host:local}"

log() { echo "[servo-test] $*"; }
die() { echo "[servo-test] ERROR: $*" >&2; exit 1; }

instructions() {
    cat <<EOF

================================================================
 Arm servo test: MoveIt is up, the arm holds the start pose.
 Start these in two more terminals, in ${REPO_DIR}:

 2) The tracker (camera on ROS_DOMAIN_ID=${CAM_DOMAIN} -> /object_distance on ROS_DOMAIN_ID=${ARM_DOMAIN}):

    ${ME} tracker -p target_class:=bottle -p show:=true

 2') Or the 3D detector instead of 2) (YOLO11s, 3D boxes; RViz with --rviz):

    ${ME} detector --class bottle --rviz

 3) The servo, first planning only, then for real (ROS_DOMAIN_ID=${ARM_DOMAIN}, as MoveIt):

    ${ME} servo -p dry_run:=true
    ${ME} servo

 What they run, if you want them by hand:
    ROS_DOMAIN_ID=${ARM_DOMAIN} ROS_LOCALHOST_ONLY=1 python3 scripts/nodes/distance_servo.py --ros-args -p dry_run:=true
    ROS_DOMAIN_ID=${CAM_DOMAIN} (CycloneDDS, in ${TRACKER_IMAGE}) python3 yolo_depth_tracker.py --ros-args -p out_domain_id:=${ARM_DOMAIN}
 Watch the distance:  ROS_DOMAIN_ID=${ARM_DOMAIN} ROS_LOCALHOST_ONLY=1 ros2 topic echo /object_distance

 Stop: Ctrl+C in 3), then 2), then here. The arm stays enabled and holding:
 fold it with ./scripts/runtime/start_arm.sh fold
================================================================
EOF
}

# --- 1) MoveIt + bridge + start pose ---------------------------------------
run_arm() {
    local mode=(--real)
    if [ "${1:-}" = "--mock" ]; then
        shift
        mode=()
        log "mock arm: MoveIt and RViz only, nothing moves"
    else
        [ -n "${ARM_JOINT_SIGNS:-}" ] || die "set ARM_JOINT_SIGNS (the joint sign test, docs/hardware/d1_arm.md), e.g.
  ARM_JOINT_SIGNS=\"-1 1 1 1 1 -1\" ${ME}"
        log "REAL arm: it moves to the start pose (${START_JOINTS} rad) at 20 % speed"
    fi
    local ready
    ready="$(mktemp -u /tmp/arm_servo_ready.XXXXXX)"
    # The instructions once the arm is at the start pose (or gave up on it).
    ( for _ in $(seq 1 600); do
          [ -e "${ready}" ] && { rm -f "${ready}"; instructions; exit 0; }
          sleep 0.5
      done ) &
    # Anything else (--no-rviz, --robot IP) goes to arm_moveit.sh.
    HOLD_READY_FILE="${ready}" exec "${SCRIPT_DIR}/arm_moveit.sh" "${mode[@]}" "$@" --hold "${START_JOINTS}"
}

# --- shared: the network and one Cyclone config for both domains -----------
resolve_net() {
    if [ "${LOCAL}" = 1 ]; then
        log "local camera: domain ${CAM_DOMAIN} and ${ARM_DOMAIN} on loopback (no dog)"
        return 0
    fi
    # shellcheck source=scripts/runtime/go2_net.sh disable=SC1091
    source "${SCRIPT_DIR}/go2_net.sh"
    if ! { [ -n "${GO2_ROBOT_IP:-}" ] && [ -n "${GO2_HOST_IP:-}" ]; }; then
        go2_resolve_network "" "" true >/dev/null || die "could not find the Go2 on the Wi-Fi"
    fi
    log "camera: robot ${GO2_ROBOT_IP} -> this computer ${GO2_HOST_IP}, domain ${CAM_DOMAIN}; output on domain ${ARM_DOMAIN} (loopback)"
}

loopback_domain() {   # $1 = domain id
    cat <<XML
  <Domain id="$1">
    <General>
      <Interfaces><NetworkInterface address="127.0.0.1" multicast="false"/></Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>30</MaxAutoParticipantIndex>
      <Peers><Peer address="127.0.0.1"/></Peers>
    </Discovery>
  </Domain>
XML
}

cam_domain_cfg() {
    if [ "${LOCAL}" = 1 ]; then
        loopback_domain "${CAM_DOMAIN}"
        return
    fi
    cat <<XML
  <Domain id="${CAM_DOMAIN}">
    <General>
      <Interfaces><NetworkInterface address="${GO2_HOST_IP}" priority="default" multicast="default"/></Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
      <Peers><Peer address="${GO2_ROBOT_IP}"/></Peers>
    </Discovery>
  </Domain>
XML
}

# Domain CAM_DOMAIN on the Wi-Fi with the robot as a peer (--local: loopback), ARM_DOMAIN on loopback, where
# the host's localhost-only FastDDS (MoveIt) talks to it. Prints the file.
dual_cfg() {
    cat <<XML
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
$(cam_domain_cfg)
  <Domain id="${ARM_DOMAIN}">
    <General>
      <Interfaces><NetworkInterface address="127.0.0.1" multicast="false"/></Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>30</MaxAutoParticipantIndex>
      <Peers><Peer address="127.0.0.1"/></Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
XML
}

# --- 2) the tracker ----------------------------------------------------------
run_tracker() {
    command -v docker >/dev/null || die "docker not found"
    docker image inspect "${TRACKER_IMAGE}" >/dev/null 2>&1 \
        || die "${TRACKER_IMAGE} not found: build it with ./scripts/runtime/wifi_3d_nav.sh --build"
    resolve_net
    mkdir -p "${MODEL_DIR}"
    local cfg
    cfg="$(mktemp /tmp/servo-cyclonedds.XXXXXX.xml)"
    dual_cfg > "${cfg}"
    chmod 644 "${cfg}"
    local x11=()
    if [ -n "${DISPLAY:-}" ]; then
        xhost "+SI:localuser:$(id -un)" >/dev/null 2>&1 || true
        x11=(-e DISPLAY="${DISPLAY}" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
    fi
    local params=("$@")
    # The model is downloaded once into MODEL_DIR (the container's working directory).
    log "YOLO model in ${MODEL_DIR} (downloaded on the first run). Ctrl+C to stop."
    docker rm -f go2-servo-tracker >/dev/null 2>&1 || true
    trap 'rm -f "${cfg}"' EXIT
    local tty=()
    [ -t 0 ] && tty=(-it)
    docker run --rm "${tty[@]}" --name go2-servo-tracker --network host --ipc host \
        --user "$(id -u):$(id -g)" -e HOME=/tmp "${x11[@]}" \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///servo-cyclonedds.xml \
        -e ROS_DOMAIN_ID="${CAM_DOMAIN}" \
        -v "${cfg}:/servo-cyclonedds.xml:ro" \
        -v "${REPO_DIR}/scripts/nodes:/nodes:ro" -v "${MODEL_DIR}:/models" -w /models \
        --entrypoint bash "${TRACKER_IMAGE}" -c \
        'source /opt/ros/humble/setup.bash && exec python3 -u /nodes/yolo_depth_tracker.py --ros-args -p out_domain_id:='"${ARM_DOMAIN}"' -p color_topic:='"${SERVO_COLOR_TOPIC:-/camera/color/image_raw}"' "$@"' \
        tracker "${params[@]}"
}

# --- 2') the test branch's RealSense 3D detector instead of the tracker -----------
# plant_detection.sh (src/object_detection_rgbd, YOLO11s) in go2-lidar3d publishes
# /detections_3d on domain CAM_DOMAIN; detections_to_distance.py turns the nearest wanted
# box into /object_distance on ARM_DOMAIN. Options: --raw (raw images instead of the
# JPEG/PNG copies; too much for the Wi-Fi), --rviz, --class NAME (also the detector's
# only class), then -- and more detector parameters.
run_detector() {
    local container="${CONTAINER_NAME:-go2-lidar3d}" compressed=(--compressed) rviz=() cls="${SERVO_TARGET_CLASS:-}" node_args=()
    while [ $# -gt 0 ]; do
        case "$1" in
            --raw) compressed=() ;;
            --rviz) rviz=(--rviz) ;;
            --class) cls="${2:?--class needs a COCO class name, e.g. bottle}"; shift ;;
            --) shift; node_args=("$@"); break ;;
            *) die "detector: unknown option $1 (--raw | --rviz | --class NAME | -- detector parameters)" ;;
        esac
        shift
    done
    resolve_net
    docker ps -a --format '{{.Names}}' | grep -qx "${container}" \
        || die "${container} does not exist: ./scripts/tools/realsense_host.sh status creates it"
    docker ps --format '{{.Names}}' | grep -qx "${container}" || docker start "${container}" >/dev/null
    local det_cfg=""
    if [ "${LOCAL}" = 1 ]; then
        # The detector on loopback like the camera; raw images are fine there.
        det_cfg="$(mktemp /tmp/servo-detect-cyclonedds.XXXXXX.xml)"
        dual_cfg > "${det_cfg}"
        export DETECT_CYCLONEDDS_FILE="${det_cfg}"
        compressed=()
    else
        # The session's DDS file for the current IPs (what plant_detection.sh uses).
        docker exec -e GO2_HOST_IP="${GO2_HOST_IP}" -e GO2_ROBOT_IP="${GO2_ROBOT_IP}" -e GO2_RELAY_DOMAIN_ID="${CAM_DOMAIN}" \
            "${container}" bash -c 'source /ws/scripts/setup-robot-wifi.sh >/dev/null 2>&1' \
            || die "setup-robot-wifi.sh failed in ${container}"
    fi
    "${SCRIPT_DIR}/plant_detection.sh" --setup-only
    # YOLO11s: the cached copy, so the container does not download it again.
    if [ -f "${MODEL_DIR}/yolo11s.pt" ] && ! docker exec "${container}" test -f /ws/.torch/hub/yolo11s.pt; then
        docker exec "${container}" mkdir -p /ws/.torch/hub
        docker cp -q "${MODEL_DIR}/yolo11s.pt" "${container}:/ws/.torch/hub/yolo11s.pt"
    fi
    local relay=/tmp/servo_relay
    docker exec -u root "${container}" bash -c "mkdir -p ${relay} && chmod 1777 ${relay}"
    dual_cfg | docker exec -i "${container}" bash -c "cat > ${relay}/cyclonedds.xml"
    docker cp -q "${REPO_DIR}/scripts/nodes/detections_to_distance.py" "${container}:${relay}/"
    docker exec "${container}" pkill -INT -f "${relay}/detections_to_distance.py" >/dev/null 2>&1 || true
    # No --class: no target_class parameter at all (ROS rejects an empty "-p name:=").
    local cls_arg=""
    [ -n "${cls}" ] && cls_arg="-p target_class:='${cls}'"
    docker exec -d -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI="file://${relay}/cyclonedds.xml" \
        -e ROS_DOMAIN_ID="${CAM_DOMAIN}" -e PYTHONUNBUFFERED=1 "${container}" bash -c \
        "source /opt/ros/humble/setup.bash; exec python3 ${relay}/detections_to_distance.py --ros-args -p out_domain_id:=${ARM_DOMAIN} ${cls_arg} > ${relay}/relay.log 2>&1"
    log "relay /detections_3d -> /object_distance started (log: docker exec ${container} cat ${relay}/relay.log)"
    # shellcheck disable=SC2064  # expand now: the container and path are fixed
    trap "docker exec ${container} pkill -INT -f ${relay}/detections_to_distance.py >/dev/null 2>&1 || true; rm -f ${det_cfg}" EXIT
    local extra=()
    [ -n "${cls}" ] && extra=(-p "filter_classes:=['${cls}']")
    "${SCRIPT_DIR}/plant_detection.sh" "${compressed[@]}" "${rviz[@]}" -- "${extra[@]}" "${node_args[@]}"
}

# --- 0) the D435i plugged into this computer (--local) -----------------------------
# The realsense2_camera driver in CAMERA_IMAGE (realsense/host/Dockerfile, built on the
# first run), on loopback, with the robot driver's topic names: /camera/color/image_raw,
# /camera/aligned_depth_to_color/image_raw (+ /compressed, /compressedDepth),
# /camera/color/camera_info. SERVO_CAMERA_PROFILE=640,480,15 (USB 2: keep 15 fps or lower).
run_camera() {
    LOCAL=1
    if ! docker image inspect "${CAMERA_IMAGE}" >/dev/null 2>&1; then
        log "building ${CAMERA_IMAGE} (ROS Humble realsense2_camera, a few minutes, needs internet)..."
        docker build -t "${CAMERA_IMAGE}" "${REPO_DIR}/realsense/host" || die "image build failed"
    fi
    command -v lsusb >/dev/null && { lsusb | grep -q '8086:' \
        || die "no Intel RealSense on this computer's USB (lsusb): plug the D435i in (a USB 3 port, blue)"; }
    local cfg profile="${SERVO_CAMERA_PROFILE:-640,480,15}"
    cfg="$(mktemp /tmp/servo-camera-cyclonedds.XXXXXX.xml)"
    printf '<?xml version="1.0" encoding="UTF-8" ?>\n<CycloneDDS xmlns="https://cdds.io/config">\n%s\n</CycloneDDS>\n' \
        "$(loopback_domain "${CAM_DOMAIN}")" > "${cfg}"
    chmod 644 "${cfg}"
    trap 'rm -f "${cfg}"' EXIT
    log "D435i on this computer: ${profile} colour + aligned depth on domain ${CAM_DOMAIN} (loopback). Ctrl+C to stop."
    docker rm -f go2-realsense-local >/dev/null 2>&1 || true
    local tty=()
    [ -t 0 ] && tty=(-it)
    # --privileged + /dev: librealsense reaches the camera's USB and video nodes, also after
    # it re-enumerates (initial_reset).
    docker run --rm "${tty[@]}" --name go2-realsense-local --privileged -v /dev:/dev \
        --network host --ipc host -e ROS_DOMAIN_ID="${CAM_DOMAIN}" \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///camera-cyclonedds.xml \
        -v "${cfg}:/camera-cyclonedds.xml:ro" "${CAMERA_IMAGE}" bash -c \
        "source /opt/ros/humble/setup.bash && exec ros2 launch realsense2_camera rs_launch.py \
            camera_namespace:=/ camera_name:=camera align_depth.enable:=true \
            rgb_camera.color_profile:=${profile} depth_module.depth_profile:=${profile} \
            enable_infra1:=false enable_infra2:=false enable_gyro:=false enable_accel:=false"
}

# --- 3) the servo ------------------------------------------------------------
run_servo() {
    [ -f /opt/ros/humble/setup.bash ] || die "ROS 2 Humble not found"
    unset CYCLONEDDS_URI RMW_IMPLEMENTATION
    export ROS_DOMAIN_ID="${ARM_DOMAIN}" ROS_LOCALHOST_ONLY=1
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    local args=("$@")
    log "distance_servo.py on domain ${ARM_DOMAIN} (localhost) ${args[*]}"
    if [ "${#args[@]}" -gt 0 ]; then
        exec python3 -u "${REPO_DIR}/scripts/nodes/distance_servo.py" --ros-args "${args[@]}"
    fi
    exec python3 -u "${REPO_DIR}/scripts/nodes/distance_servo.py"
}

case "${1:-}" in
    ""|arm)    shift || true; run_arm "$@" ;;
    --mock)    shift; run_arm --mock "$@" ;;
    -h|--help) sed -n "2,$(grep -n '^# ---' "$0" | sed -n 2p | cut -d: -f1)p" "$0" ;;
    tracker)   shift; [ "${1:-}" = --local ] && { LOCAL=1; shift; }; run_tracker "$@" ;;
    detector)  shift; [ "${1:-}" = --local ] && { LOCAL=1; shift; }; run_detector "$@" ;;
    camera)    shift; run_camera "$@" ;;
    servo)     shift; run_servo "$@" ;;
    --*)       run_arm "$@" ;;
    *)         die "unknown: $1 (arm | --mock | camera | tracker | detector | servo | --help)" ;;
esac
