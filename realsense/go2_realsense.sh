#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# go2_realsense.sh: ON THE ROBOT (Go2 Jetson). RealSense D435i driver in Docker.
#
# The robot's ROS 2 Foxy, Unitree packages and factory DDS config are never
# touched: everything runs in the go2-realsense:foxy image, with only
# /dev/bus/usb passed in and the host network shared (needed for DDS).
#
#   ./go2_realsense.sh build                      # once, ~25-40 min on the Jetson
#   GO2_HOST_IP=<workstation IP> ./go2_realsense.sh start
#   ./go2_realsense.sh topics | hz | logs | status | stop
#   ./go2_realsense.sh check                      # SDK-only device check (no ROS)
#   ./go2_realsense.sh shell                      # shell in the running container
#
# Environment:
#   GO2_HOST_IP       workstation IP(s), space/comma separated: unicast DDS peers (required for start)
#   RS_DDS_IFACE      robot interface to bind (default: the one that routes to GO2_HOST_IP)
#   ROS_DOMAIN_ID     default 64, the Wi-Fi domain of this repo (0 is refused: Unitree's domain)
#   RS_COLOR_PROFILE / RS_DEPTH_PROFILE   "w,h,fps", default 640,480,15
#   RS_ALIGN=true  RS_IMU=false  RS_POINTCLOUD=false  RS_INITIAL_RESET=true  RS_QOS=SYSTEM_DEFAULT
#   RS_DDS_MULTICAST=spdp  RS_DDS_TRACE=0  RS_LOG_LEVEL=info
#   RS_PRIVILEGED=1   0 = device-cgroup rule instead of --privileged
#   RS_RESTART=no     Docker restart policy (unless-stopped to start with the robot)
#   DOCKER=docker     set DOCKER="sudo docker" if the user is not in the docker group
#   RS_IMAGE=go2-realsense:foxy   RS_CONTAINER=go2-realsense   RS_BUILD_JOBS=4
# ---------------------------------------------------------------------------
set -eo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
read -r -a DOCKER_CMD <<< "${DOCKER:-docker}"
IMAGE="${RS_IMAGE:-go2-realsense:foxy}"
NAME="${RS_CONTAINER:-go2-realsense}"
DOMAIN="${ROS_DOMAIN_ID:-64}"
D435I_ID="8086:0b3a"

dk() { "${DOCKER_CMD[@]}" "$@"; }
die() { echo "[rs] ERROR: $*" >&2; exit 1; }
running() { dk ps --format '{{.Names}}' | grep -qx "${NAME}"; }

# Run a command inside the running container with its DDS settings.
in_container() {
    running || die "container ${NAME} is not running (./go2_realsense.sh start)"
    dk exec "${NAME}" bash -c "unset LD_LIBRARY_PATH; source /opt/ros/foxy/setup.bash; source /opt/rs_ws/install/setup.bash; \
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///tmp/rs-cyclonedds.xml ROS_DOMAIN_ID=\$(cat /tmp/rs-domain); $*"
}

usb_report() {
    if command -v lsusb >/dev/null; then
        if lsusb -d "${D435I_ID}" >/dev/null 2>&1; then
            echo "[rs] D435i on USB: $(lsusb -d "${D435I_ID}")"
        elif lsusb | grep -qi '8086:'; then
            echo "[rs] Intel USB device (not 0b3a): $(lsusb | grep -i '8086:')"
        else
            echo "[rs] WARNING: no Intel RealSense on USB. Use the rear USB-C port of the expansion dock." >&2
        fi
        # 5000M = USB 3; 480M = USB 2 (fewer, slower profiles; often a cable / C-orientation issue)
        lsusb -t 2>/dev/null | grep -B1 -i 'uvcvideo\|Video' | grep -o '[0-9]*M$' | sort -u \
            | sed 's/^/[rs] video device link speed: /' || true
    fi
}

cmd_build() {
    [ -f "${DIR}/Dockerfile" ] || die "no Dockerfile next to this script"
    command -v df >/dev/null && df -h /var/lib/docker 2>/dev/null | tail -1 | awk '{print "[rs] free space for Docker: " $4}'
    dk build --build-arg BUILD_JOBS="${RS_BUILD_JOBS:-4}" -t "${IMAGE}" "${DIR}"
}

cmd_start() {
    : "${GO2_HOST_IP:?Set GO2_HOST_IP to the workstation IP (the DDS unicast peer)}"
    dk image inspect "${IMAGE}" >/dev/null 2>&1 || die "image ${IMAGE} not built (./go2_realsense.sh build)"
    local first iface
    first="${GO2_HOST_IP//,/ }"; first="${first%% *}"
    iface="${RS_DDS_IFACE:-$(ip route get "${first}" 2>/dev/null | grep -oP 'dev \K\S+' | head -1)}"
    [ -n "${iface}" ] || die "no route to ${first}; set RS_DDS_IFACE"
    case "${iface}" in docker*|br-*|veth*|lo) die "route to ${first} goes via ${iface}; set RS_DDS_IFACE (wlan0 / eth0)";; esac
    [ "${DOMAIN}" != 0 ] || die "ROS_DOMAIN_ID=0 is Unitree's onboard domain; use another (default 64)"

    usb_report
    if running; then echo "[rs] ${NAME} already running; restarting"; fi
    dk rm -f "${NAME}" >/dev/null 2>&1 || true

    local dev_args=(-v /dev/bus/usb:/dev/bus/usb -v /run/udev:/run/udev:ro)
    if [ "${RS_PRIVILEGED:-1}" = 1 ]; then
        dev_args+=(--privileged)
    else
        # 189 = USB device nodes (libusb / RSUSB, including the D435i IMU HID).
        dev_args+=(--device-cgroup-rule='c 189:* rmw')
    fi

    local env_args=(-e ROS_DOMAIN_ID="${DOMAIN}" -e RS_DDS_IFACE="${iface}" -e RS_DDS_PEERS="${GO2_HOST_IP}")
    local v
    for v in RS_COLOR_PROFILE RS_DEPTH_PROFILE RS_ALIGN RS_IMU RS_POINTCLOUD RS_INITIAL_RESET \
             RS_QOS RS_DDS_MULTICAST RS_DDS_TRACE RS_LOG_LEVEL; do
        [ -n "${!v:-}" ] && env_args+=(-e "${v}=${!v}")
    done

    echo "[rs] starting ${NAME}: domain ${DOMAIN}, bound to ${iface}, peer(s) ${GO2_HOST_IP}"
    dk run -d --name "${NAME}" \
        --network host --ipc host \
        --restart "${RS_RESTART:-no}" \
        --log-opt max-size=10m --log-opt max-file=2 \
        "${dev_args[@]}" "${env_args[@]}" \
        "${IMAGE}" camera >/dev/null
    sleep 6
    if ! running; then
        echo "[rs] ${NAME} exited. Last log lines:" >&2
        dk logs --tail 30 "${NAME}" 2>&1 | sed 's/^/    /' >&2
        exit 1
    fi
    dk logs --tail 15 "${NAME}" 2>&1 | sed 's/^/    /'
    echo "[rs] running. Next: ./go2_realsense.sh topics   then on the workstation: realsense_host.sh check"
}

cmd_check() {
    usb_report
    running && die "${NAME} is running and holds the camera: ./go2_realsense.sh stop first"
    dk run --rm --network host --privileged -v /dev/bus/usb:/dev/bus/usb "${IMAGE}" check
}

case "${1:-}" in
    build)   cmd_build ;;
    start)   cmd_start ;;
    restart) cmd_start ;;
    stop)    dk rm -f "${NAME}" >/dev/null 2>&1 && echo "[rs] stopped" || echo "[rs] not running" ;;
    logs)    dk logs -f --tail 100 "${NAME}" ;;
    status)  usb_report; dk ps -a --filter "name=^${NAME}$" --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}' ;;
    check)   cmd_check ;;
    shell)   running || die "not running"; dk exec -it "${NAME}" bash ;;
    topics)  in_container "ros2 topic list -t | grep -E '^/camera|^/tf' || echo 'no /camera topics yet'" ;;
    hz)      in_container "for t in /camera/color/image_raw /camera/depth/image_rect_raw /camera/aligned_depth_to_color/image_raw; do echo \"== \$t\"; timeout 6 ros2 topic hz \$t | tail -2; done" ;;
    *)       sed -n '2,30p' "$0"; exit 2 ;;
esac
