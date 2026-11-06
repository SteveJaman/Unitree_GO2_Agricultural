#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# realsense_host.sh: ON THE WORKSTATION. The Go2's RealSense D435i in the
# repo's own go2-lidar3d container (no separate test container), with both
# ends found by MAC so DHCP changes don't matter.
#
# How the topics get here: the camera runs on the robot as a boot service
# (realsense/boot, domain 64, CycloneDDS). It finds this machine by its Wi-Fi
# MAC and sends to its current IP; go2-lidar3d already runs Cyclone on domain
# 64 with the robot as a peer (setup-robot-wifi.sh), so any ROS process in it
# sees /camera/color/*, /camera/depth/*, /camera/aligned_depth_to_color/*.
# wifi_nav.sh also pushes this machine's IP to the robot at every session start.
#
# Once per robot / workstation:
#   ./scripts/tools/realsense_host.sh macs          # learn + save the robot's MAC; show this machine's
#   ./scripts/tools/realsense_host.sh robot-build   # build go2-realsense:foxy on the robot (if not built)
#   ./scripts/tools/realsense_host.sh setup-robot [--hotplug]
#                                                   # install the boot service with this machine's MAC
# Any time:
#   ./scripts/tools/realsense_host.sh status        # robot service, peers, container
#   ./scripts/tools/realsense_host.sh check         # /camera topics + rates, inside go2-lidar3d
#   ./scripts/tools/realsense_host.sh view          # rqt_image_view inside go2-lidar3d
#   ./scripts/tools/realsense_host.sh rviz          # rviz2 inside go2-lidar3d
#   ./scripts/tools/realsense_host.sh shell         # ROS shell inside go2-lidar3d on the camera's DDS setup
#   ./scripts/tools/realsense_host.sh push-peer     # tell the robot this machine's current IP
#   ./scripts/tools/realsense_host.sh plugins       # install image transport plugins + rqt in go2-lidar3d
#   ./scripts/tools/realsense_host.sh robot-logs | robot-restart | robot-stop
#
# Environment:
#   GO2_ROBOT_IP / GO2_HOST_IP   skip discovery when both are set (wifi_nav.sh passes them)
#   GO2_IFACE                    Wi-Fi interface (auto)        CONTAINER_NAME=go2-lidar3d
#   GO2_RELAY_DOMAIN_ID=64       RS_ROBOT_DIR=files/rs-test (on the robot, under ~)
#   NAV2_WIFI_DIR=~/files/go2-nav2-wifi   (creates go2-lidar3d if it does not exist)
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
CONTAINER_NAME="${CONTAINER_NAME:-go2-lidar3d}"
DOMAIN="${GO2_RELAY_DOMAIN_ID:-64}"
ROBOT_DIR="${RS_ROBOT_DIR:-files/rs-test}"
NAV2_WIFI_DIR="${NAV2_WIFI_DIR:-$HOME/files/go2-nav2-wifi}"
PEER_FILE=/tmp/go2-realsense-peer

# shellcheck source=../runtime/go2_net.sh
source "${REPO_DIR}/scripts/runtime/go2_net.sh"

log() { echo "[realsense] $*" >&2; }
die() { log "ERROR: $*"; exit 1; }

# --- network ---------------------------------------------------------------
resolve_net() {
    if [ -n "${GO2_ROBOT_IP:-}" ] && [ -n "${GO2_HOST_IP:-}" ] && go2_valid_ip "${GO2_ROBOT_IP}" && go2_valid_ip "${GO2_HOST_IP}"; then
        :
    else
        go2_resolve_network "" "" true >/dev/null || die "could not find the Go2 on ${GO2_IFACE}"
    fi
    export GO2_ROBOT_IP GO2_HOST_IP GO2_IFACE
}

host_mac() { tr '[:upper:]' '[:lower:]' < "/sys/class/net/${GO2_IFACE}/address" 2>/dev/null; }

# Bit 1 of the first octet set = locally administered: often a randomised MAC
# (NetworkManager), which changes and would break the robot's lookup.
mac_is_random() { local oct="${1%%:*}"; [ $(( 0x${oct:-0} & 2 )) -ne 0 ]; }

robot_ssh()   { ssh    "${GO2_SSH_OPTS[@]}" -o ConnectTimeout=8 "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" "$@"; }
robot_ssh_t() { ssh -t "${GO2_SSH_OPTS[@]}" -o ConnectTimeout=8 "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" "$@"; }

push_peer() {
    # /tmp on the robot: no sudo needed; the boot service re-reads it every RS_PEER_CHECK s.
    robot_ssh "echo ${GO2_HOST_IP} > ${PEER_FILE}" \
        && log "robot told: workstation is ${GO2_HOST_IP} (picked up within ~20 s if it changed)"
}

# --- go2-lidar3d -------------------------------------------------------------
ensure_container() {
    if ! docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
        [ -x "${NAV2_WIFI_DIR}/setup-lidar3d.sh" ] || die "${CONTAINER_NAME} does not exist and ${NAV2_WIFI_DIR}/setup-lidar3d.sh is missing (run ./scripts/setup.sh)"
        log "creating ${CONTAINER_NAME} for ${GO2_HOST_IP} / robot ${GO2_ROBOT_IP} (setup-lidar3d.sh --laptop)"
        (cd "${NAV2_WIFI_DIR}" && ./setup-lidar3d.sh --laptop) || die "could not create ${CONTAINER_NAME}"
    fi
    if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
        docker start "${CONTAINER_NAME}" >/dev/null || die "could not start ${CONTAINER_NAME}"
        sleep 2
    fi
    # ROS_LOG_DIR must stay writable for the container user (see wifi_nav.sh).
    docker exec -u root "${CONTAINER_NAME}" bash -c \
        'd="${ROS_LOG_DIR:-/tmp/go2-lidar3d-ros-log}"; mkdir -p "$d" && chmod 1777 "$d"' >/dev/null 2>&1 || true
}

# Run a command in go2-lidar3d on the session's DDS setup, rebuilt for the
# CURRENT IPs (setup-robot-wifi.sh writes /tmp/go2-wifi-cyclonedds.xml:
# bound to this machine's IP, the robot as peer, ROS_DOMAIN_ID=64).
in_container() {   # $1 = -it or -i, rest = command string
    local tty="$1"; shift
    xhost +local:docker >/dev/null 2>&1 || true
    docker exec "${tty}" \
        -e GO2_HOST_IP="${GO2_HOST_IP}" -e GO2_ROBOT_IP="${GO2_ROBOT_IP}" -e GO2_RELAY_DOMAIN_ID="${DOMAIN}" \
        -e DISPLAY="${DISPLAY:-:0}" -e QT_QPA_PLATFORM=xcb -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 \
        "${CONTAINER_NAME}" bash -c "source /ws/scripts/setup-robot-wifi.sh >/dev/null 2>&1 || { echo 'setup-robot-wifi.sh failed' >&2; exit 1; }
ros2 daemon stop >/dev/null 2>&1
$*"
}

plugins_present() {
    docker exec "${CONTAINER_NAME}" bash -c 'source /opt/ros/humble/setup.bash; ros2 pkg prefix compressed_depth_image_transport && ros2 pkg prefix rqt_image_view' >/dev/null 2>&1
}

# The go2-lidar3d image ships only image_transport's raw plugin: RViz then
# fails with "Unable to load plugin for transport 'image_transport/compressed_sub'".
# Installed into the container (a recreated container loses it; wifi_nav.sh
# re-runs this). Fixes the ROS apt key that expired on 2025-06-01 if needed.
ensure_plugins() {
    if plugins_present; then
        log "go2-lidar3d image transport plugins + rqt ...... OK"
        return 0
    fi
    log "installing image transport plugins + rqt_image_view in ${CONTAINER_NAME} (apt, once per container, ~1 min)..."
    docker exec -u root -e DEBIAN_FRONTEND=noninteractive "${CONTAINER_NAME}" bash -c '
set -o pipefail
upd() { apt-get update 2>&1 | tee /tmp/rs-apt-update.log >/dev/null; ! grep -qE "EXPKEYSIG|NO_PUBKEY|is not signed|invalid signature" /tmp/rs-apt-update.log; }
if ! upd; then
    echo "[realsense] ROS apt key expired: re-adding packages.ros.org with the renewed key"
    rm -f /etc/apt/sources.list.d/ros2*.list /etc/apt/sources.list.d/ros2*.sources
    k=/usr/share/keyrings/ros-archive-keyring.gpg u=https://raw.githubusercontent.com/ros/rosdistro/master/ros.key
    curl -fsSL "$u" -o "$k" 2>/dev/null || python3 -c "import urllib.request,sys; urllib.request.urlretrieve(sys.argv[1], sys.argv[2])" "$u" "$k" || exit 1
    echo "deb [arch=$(dpkg --print-architecture) signed-by=$k] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release; echo $VERSION_CODENAME) main" > /etc/apt/sources.list.d/ros2.list
    upd || { tail -5 /tmp/rs-apt-update.log; exit 1; }
fi
apt-get install -y -qq --no-install-recommends ros-humble-image-transport-plugins ros-humble-rqt-image-view >/tmp/rs-apt-install.log 2>&1 \
    || { tail -5 /tmp/rs-apt-install.log; exit 1; }
' || { log "WARN: install failed (see above): RViz/rqt can only show raw images."; return 1; }
    plugins_present && log "image transport plugins + rqt installed in ${CONTAINER_NAME}"
}

copy_to_robot() {
    robot_ssh "mkdir -p ~/${ROBOT_DIR}/boot"
    scp -q "${GO2_SSH_OPTS[@]}" "${REPO_DIR}"/realsense/{Dockerfile,entrypoint.sh,go2_realsense.sh} \
        "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}:${ROBOT_DIR}/"
    scp -q "${GO2_SSH_OPTS[@]}" "${REPO_DIR}"/realsense/boot/* \
        "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}:${ROBOT_DIR}/boot/"
    robot_ssh "chmod +x ~/${ROBOT_DIR}/*.sh ~/${ROBOT_DIR}/boot/*.sh"
    log "copied realsense/ to ${GO2_ROBOT_USER}@${GO2_ROBOT_IP}:~/${ROBOT_DIR}"
}

save_conf() {   # $1 = key, $2 = value
    mkdir -p "$(dirname "${GO2_NET_CONF}")"
    touch "${GO2_NET_CONF}"
    if grep -q "^$1=" "${GO2_NET_CONF}"; then
        sed -i "s|^$1=.*|$1=$2|" "${GO2_NET_CONF}"
    else
        echo "$1=$2" >> "${GO2_NET_CONF}"
    fi
}

case "${1:-}" in
    macs)
        resolve_net
        hm="$(host_mac)"
        echo "This machine: ${GO2_IFACE} ${GO2_HOST_IP}  MAC ${hm:-?}"
        if [ -n "${hm}" ] && mac_is_random "${hm}"; then
            echo "  WARNING: ${hm} is a locally administered (often randomised) MAC. If it changes,"
            echo "  the robot loses this machine. Pin the real one for this Wi-Fi network:"
            echo "    nmcli connection modify \"\$(nmcli -g GENERAL.CONNECTION device show ${GO2_IFACE})\" 802-11-wireless.cloned-mac-address permanent"
        fi
        rmac="$(go2_mac_of "${GO2_ROBOT_IP}")"
        rsys="$(robot_ssh 'cat /sys/class/net/wlan0/address' 2>/dev/null | tr '[:upper:]' '[:lower:]' || true)"
        echo "Robot:        ${GO2_ROBOT_IP}  MAC ${rmac:-?} (robot reports wlan0 ${rsys:-?})"
        [ -n "${rmac}" ] || rmac="${rsys}"
        [ -n "${rmac}" ] || die "could not read the robot MAC"
        save_conf GO2_ROBOT_MAC "${rmac}"
        echo "Saved GO2_ROBOT_MAC=${rmac} in ${GO2_NET_CONF}: every repo script now finds the robot by it first."
        echo "Check it stays the same after a robot reboot (run 'macs' again); if it changes each boot, delete that line"
        echo "(the 02:00:00 prefix scan and the SSH-key probe still find it)."
        ;;
    setup-robot)
        resolve_net
        hm="$(host_mac)"; [ -n "${hm}" ] || die "cannot read the MAC of ${GO2_IFACE}"
        mac_is_random "${hm}" && log "WARNING: ${hm} looks randomised (see: $0 macs)"
        copy_to_robot
        robot_ssh "docker image inspect go2-realsense:foxy >/dev/null 2>&1" \
            || log "NOTE: go2-realsense:foxy is not built on the robot yet: $0 robot-build"
        log "installing the boot service (the robot's sudo password may be asked)..."
        robot_ssh_t "cd ~/${ROBOT_DIR}/boot && sudo ./install_boot_service.sh --host-mac ${hm} ${2:-}"
        push_peer
        ;;
    robot-build)
        resolve_net; copy_to_robot
        robot_ssh_t "cd ~/${ROBOT_DIR} && docker build --network host --build-arg BUILD_JOBS=\${RS_BUILD_JOBS:-4} -t go2-realsense:foxy . 2>&1 | tee build.log"
        ;;
    push-peer)
        resolve_net; push_peer ;;
    status)
        resolve_net
        echo "workstation ${GO2_HOST_IP} ($(host_mac)), robot ${GO2_ROBOT_IP} ($(go2_mac_of "${GO2_ROBOT_IP}"))"
        robot_ssh "systemctl is-enabled go2-realsense 2>/dev/null | sed 's/^/service enabled: /';
                   systemctl is-active go2-realsense 2>/dev/null | sed 's/^/service active:  /';
                   [ -x /opt/go2-realsense/go2_realsense_boot.sh ] && echo \"peers it finds:  \$(RS_ENV_FILE=/etc/go2-realsense.env /opt/go2-realsense/go2_realsense_boot.sh --peers 2>/dev/null)\";
                   docker ps --filter name=^go2-realsense$ --format 'container:       {{.Status}}';
                   journalctl -u go2-realsense -n 8 --no-pager 2>/dev/null | sed 's/^/  /'" || true
        ;;
    robot-logs)    resolve_net; robot_ssh_t "journalctl -u go2-realsense -f" ;;
    robot-restart) resolve_net; push_peer; robot_ssh_t "sudo systemctl restart go2-realsense" ;;
    robot-stop)    resolve_net; robot_ssh_t "sudo systemctl stop go2-realsense" ;;
    plugins)
        resolve_net; ensure_container; ensure_plugins ;;
    check)
        resolve_net; ensure_container
        log "${CONTAINER_NAME}: ${GO2_HOST_IP} -> robot ${GO2_ROBOT_IP}, domain ${DOMAIN}"
        in_container -i '
echo "== /camera topics"
timeout 15 ros2 topic list --no-daemon | grep -E "^/camera/(color|depth|aligned)" || echo "   none: ./scripts/tools/realsense_host.sh status"
for t in /camera/color/image_raw/compressed /camera/aligned_depth_to_color/image_raw/compressedDepth /camera/color/image_raw; do
    echo "== rate $t"
    timeout 8 ros2 topic hz "$t" 2>/dev/null | grep -m2 "average rate\|min:" || echo "   no messages in 8 s"
done
echo "== camera_info"
timeout 8 ros2 topic echo --once /camera/color/camera_info 2>/dev/null | grep -E "^(height|width)|frame_id" || true'
        ;;
    view)
        resolve_net; ensure_container; ensure_plugins || true
        in_container -i 'ros2 run rqt_image_view rqt_image_view /camera/color/image_raw/compressed' ;;
    rviz)
        resolve_net; ensure_container; ensure_plugins || true
        log "RViz: Add > By topic > /camera/color/image_raw > Image; Transport Hint compressed, Reliability Best Effort"
        in_container -i 'rviz2' ;;
    shell)
        resolve_net; ensure_container
        in_container -it 'echo "ROS 2 shell in '"${CONTAINER_NAME}"' (domain ${ROS_DOMAIN_ID}, ${CYCLONEDDS_URI})"; exec bash -i' ;;
    *)
        sed -n '2,37p' "$0"; exit 2 ;;
esac
