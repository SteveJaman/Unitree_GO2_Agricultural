#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# realsense_host.sh: ON THE WORKSTATION (native Ubuntu 22.04, ROS 2 Humble).
# Receive the Go2's RealSense D435i topics (realsense/go2_realsense.sh on the
# robot) over CycloneDDS, on the same Wi-Fi domain as the rest of this repo.
#
#   ./scripts/tools/realsense_host.sh deploy        # copy realsense/ to the robot
#   ./scripts/tools/realsense_host.sh robot-build   # build the image on the robot (once)
#   ./scripts/tools/realsense_host.sh robot-start   # start the camera container on the robot
#   eval "$(./scripts/tools/realsense_host.sh env)" # this shell: domain, RMW, CycloneDDS
#   ./scripts/tools/realsense_host.sh check         # topics, rates, QoS from this machine
#   ./scripts/tools/realsense_host.sh republish     # compressed/compressedDepth -> raw, local
#   ./scripts/tools/realsense_host.sh view          # rqt_image_view
#   ./scripts/tools/realsense_host.sh robot-stop
#
# Inside a running wifi_nav.sh / wifi_3d_nav.sh session nothing is needed here:
# go2-lidar3d already runs Cyclone on domain 64 with the robot as a peer, so
# the /camera/... topics appear there, and wifi_3d_nav.sh --rgbd uses them.
#
# Environment:
#   GO2_ROBOT_IP / GO2_HOST_IP   auto (go2_net.sh: mDNS, cache, scan) unless set
#   RS_HOST_IFACE                interface to bind (default: the route to the robot);
#                                set it for a wired link (e.g. enp0s31f6 + 192.168.123.x)
#   ROS_DOMAIN_ID=64             must match the robot side
#   GO2_ROBOT_USER=unitree       RS_ROBOT_DIR=~/go2-realsense (on the robot)
#   RS_* (RS_COLOR_PROFILE, RS_DEPTH_PROFILE, RS_ALIGN, ...) forwarded by robot-start
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DOMAIN="${ROS_DOMAIN_ID:-64}"
ROBOT_USER="${GO2_ROBOT_USER:-unitree}"
ROBOT_DIR="${RS_ROBOT_DIR:-go2-realsense}"   # relative to the robot user's home
XML="${HOME}/.ros/go2-realsense-cyclonedds.xml"

log() { echo "$@" >&2; }
die() { log "[rs-host] ERROR: $*"; exit 1; }

resolve_net() {
    if [ -n "${GO2_ROBOT_IP:-}" ] && [ -n "${RS_HOST_IFACE:-}" ]; then
        :   # fully pinned (e.g. wired link): skip discovery
    else
        # shellcheck source=../runtime/go2_net.sh
        source "${REPO_DIR}/scripts/runtime/go2_net.sh"
        go2_resolve_network "" "" true >/dev/null || die "could not find the Go2 (set GO2_ROBOT_IP)"
    fi
    IFACE="${RS_HOST_IFACE:-$(ip route get "${GO2_ROBOT_IP}" 2>/dev/null | grep -oP 'dev \K\S+' | head -1)}"
    [ -n "${IFACE}" ] || die "no route to ${GO2_ROBOT_IP}"
    HOST_IP="${GO2_HOST_IP:-$(ip -4 -o addr show dev "${IFACE}" | awk '{print $4}' | cut -d/ -f1 | head -1)}"
    [ -n "${HOST_IP}" ] || die "${IFACE} has no IPv4 address"
    case "${IFACE}" in docker*|br-*|veth*|lo) die "route to the robot goes via ${IFACE}: set RS_HOST_IFACE";; esac
}

robot_ssh()   { ssh    -o HostKeyAlias=go2-robot -o ConnectTimeout=8 "${ROBOT_USER}@${GO2_ROBOT_IP}" "$@"; }
robot_ssh_t() { ssh -t -o HostKeyAlias=go2-robot -o ConnectTimeout=8 "${ROBOT_USER}@${GO2_ROBOT_IP}" "$@"; }

write_xml() {
    mkdir -p "$(dirname "${XML}")"
    # Humble ships Cyclone 0.10: <Interfaces> is the current syntax.
    cat > "${XML}" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces><NetworkInterface name="${IFACE}" /></Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
      <Peers><Peer address="${GO2_ROBOT_IP}"/></Peers>
    </Discovery>
    <Internal>
      <SocketReceiveBufferSize min="8MB"/>
    </Internal>
  </Domain>
</CycloneDDS>
EOF
}

ros_env() {
    # shellcheck disable=SC1091
    [ -n "${ROS_DISTRO:-}" ] || { [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash; } \
        || die "ROS 2 not sourced and /opt/ros/humble missing"
    export ROS_DOMAIN_ID="${DOMAIN}" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="file://${XML}"
    unset ROS_LOCALHOST_ONLY
    ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1 \
        || die "rmw_cyclonedds_cpp missing: sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp"
}

rmem_check() {
    local r; r="$(cat /proc/sys/net/core/rmem_max 2>/dev/null || echo 0)"
    [ "${r}" -ge 8388608 ] || log "[rs-host] WARNING: net.core.rmem_max=${r} is small; large images will drop. See docs/setup/setup.md (60-go2-dds.conf)."
}

forward_rs_env() {
    local out="" v
    for v in RS_COLOR_PROFILE RS_DEPTH_PROFILE RS_ALIGN RS_IMU RS_POINTCLOUD RS_INITIAL_RESET RS_QOS RS_DDS_TRACE RS_LOG_LEVEL RS_DDS_IFACE; do
        [ -n "${!v:-}" ] && out+="${v}=$(printf '%q' "${!v}") "
    done
    echo "${out}"
}

case "${1:-}" in
    env)
        resolve_net; write_xml
        echo "export ROS_DOMAIN_ID=${DOMAIN} RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://${XML}; unset ROS_LOCALHOST_ONLY"
        log "[rs-host] bound to ${IFACE} (${HOST_IP}), robot peer ${GO2_ROBOT_IP}, domain ${DOMAIN}"
        ;;
    deploy)
        resolve_net
        robot_ssh "mkdir -p ~/${ROBOT_DIR}"
        scp -o HostKeyAlias=go2-robot -q "${REPO_DIR}"/realsense/{Dockerfile,entrypoint.sh,go2_realsense.sh} \
            "${ROBOT_USER}@${GO2_ROBOT_IP}:${ROBOT_DIR}/"
        robot_ssh "chmod +x ~/${ROBOT_DIR}/*.sh"
        log "[rs-host] copied realsense/ to ${ROBOT_USER}@${GO2_ROBOT_IP}:~/${ROBOT_DIR}"
        ;;
    robot-build)
        resolve_net
        log "[rs-host] building on the robot (long the first time; Ctrl-C does not stop the remote build cleanly)"
        robot_ssh_t "cd ~/${ROBOT_DIR} && ./go2_realsense.sh build"
        ;;
    robot-start)
        resolve_net
        # shellcheck disable=SC2029  # expanded here on purpose
        robot_ssh "cd ~/${ROBOT_DIR} && $(forward_rs_env) GO2_HOST_IP=${HOST_IP} ROS_DOMAIN_ID=${DOMAIN} ./go2_realsense.sh start"
        ;;
    robot-stop)
        resolve_net; robot_ssh "cd ~/${ROBOT_DIR} && ./go2_realsense.sh stop" ;;
    robot-logs)
        resolve_net; robot_ssh_t "cd ~/${ROBOT_DIR} && ./go2_realsense.sh logs" ;;
    check)
        resolve_net; write_xml; ros_env; rmem_check
        log "[rs-host] ${IFACE} (${HOST_IP}) -> robot ${GO2_ROBOT_IP}, domain ${DOMAIN}"
        ros2 daemon stop >/dev/null 2>&1 || true   # a daemon started with other settings caches a stale graph
        log "== /camera topics (discovery)"
        timeout 15 ros2 topic list -t --no-daemon | grep -E '^/camera' || log "   none: see docs/ai_and_vision/realsense.md#dds-discovery-failures"
        for t in /camera/color/image_raw/compressed /camera/aligned_depth_to_color/image_raw/compressedDepth \
                 /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /camera/depth/image_rect_raw; do
            log "== rate ${t}"
            timeout 8 ros2 topic hz "${t}" 2>/dev/null | grep -m2 'average rate\|min:' || log "   no messages in 8 s"
        done
        log "== QoS of the colour publisher"
        timeout 10 ros2 topic info -v /camera/color/image_raw 2>/dev/null | grep -E 'Node name|Reliability|Durability' || true
        log "== camera_info"
        timeout 8 ros2 topic echo --once /camera/color/camera_info 2>/dev/null | grep -E 'height|width|frame_id' || true
        ;;
    republish)
        resolve_net; write_xml; ros_env
        ros2 pkg prefix image_transport >/dev/null 2>&1 \
            || die "sudo apt install ros-${ROS_DISTRO}-image-transport-plugins"
        log "[rs-host] /camera/color/image_raw/compressed -> /rs_local/color/image_raw"
        log "[rs-host] /camera/aligned_depth_to_color/image_raw/compressedDepth -> /rs_local/depth/image_raw  (Ctrl-C stops both)"
        ros2 run image_transport republish compressed raw --ros-args \
            -r in/compressed:=/camera/color/image_raw/compressed -r out:=/rs_local/color/image_raw &
        p1=$!
        ros2 run image_transport republish compressedDepth raw --ros-args \
            -r in/compressedDepth:=/camera/aligned_depth_to_color/image_raw/compressedDepth -r out:=/rs_local/depth/image_raw &
        p2=$!
        trap 'kill ${p1} ${p2} 2>/dev/null' INT TERM EXIT
        wait
        ;;
    view)
        resolve_net; write_xml; ros_env
        ros2 run rqt_image_view rqt_image_view /camera/color/image_raw/compressed
        ;;
    *)
        sed -n '2,30p' "$0"; exit 2 ;;
esac
