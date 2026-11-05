#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# go2_realsense_boot.sh: started by go2-realsense.service when the Go2 boots.
#
#   1. waits up to CAMERA_WAIT s for the D435i (8086:0b3a) on USB;
#      not there -> logs it and exits 0: nothing is started, nothing retries;
#   2. waits up to NET_WAIT s for RS_DDS_IFACE (wlan0) to get an IPv4 address;
#   3. runs the go2-realsense container in the foreground (systemd tracks it):
#      the image's entrypoint writes the CycloneDDS config and starts
#      realsense2_camera_node as /camera/camera.
#
# Settings: /etc/go2-realsense.env (see go2-realsense.env).
# Logs:     journalctl -u go2-realsense -f
#
#   go2_realsense_boot.sh            # what the service runs
#   go2_realsense_boot.sh --dry-run  # checks + prints the docker command, starts nothing
# ---------------------------------------------------------------------------
set -uo pipefail

ENV_FILE="${RS_ENV_FILE:-/etc/go2-realsense.env}"
# shellcheck disable=SC1090
[ -f "${ENV_FILE}" ] && . "${ENV_FILE}"

NAME="${RS_CONTAINER:-go2-realsense}"
IMAGE="${RS_IMAGE:-go2-realsense:foxy}"
IFACE="${RS_DDS_IFACE:-wlan0}"
DOMAIN="${ROS_DOMAIN_ID:-64}"
CAMERA_WAIT="${CAMERA_WAIT:-60}"
NET_WAIT="${NET_WAIT:-120}"
SYSFS_USB="${RS_SYSFS_USB:-/sys/bus/usb/devices}"   # overridable for testing
DRY_RUN=0
[ "${1:-}" = "--dry-run" ] && DRY_RUN=1

log() { echo "[go2-realsense] $*"; }

camera_present() {
    local d
    for d in "${SYSFS_USB}"/*; do
        [ -f "${d}/idVendor" ] && [ -f "${d}/idProduct" ] || continue
        if [ "$(cat "${d}/idVendor")" = "8086" ] && [ "$(cat "${d}/idProduct")" = "0b3a" ]; then
            CAMERA_PATH="${d##*/}"
            CAMERA_SPEED="$(cat "${d}/speed" 2>/dev/null || echo '?')"
            return 0
        fi
    done
    return 1
}

iface_ip() { ip -4 -o addr show dev "${IFACE}" 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -1; }

# --- 1. camera -------------------------------------------------------------
waited=0
until camera_present; do
    if [ "${waited}" -ge "${CAMERA_WAIT}" ]; then
        log "no RealSense D435i (8086:0b3a) on USB after ${CAMERA_WAIT} s: not starting the camera node."
        log "plug it into the rear USB-C port of the expansion dock, then: sudo systemctl start go2-realsense"
        exit 0
    fi
    sleep 2; waited=$((waited + 2))
done
log "D435i found at USB ${CAMERA_PATH}, ${CAMERA_SPEED} Mb/s ($([ "${CAMERA_SPEED}" = 480 ] && echo 'USB 2: keep 15 fps or lower' || echo 'USB 3'))."

# --- 2. settings that would break the node ------------------------------------
if [ "${DOMAIN}" = 0 ]; then
    log "ROS_DOMAIN_ID=0 is Unitree's onboard domain: refusing. Set 64 in ${ENV_FILE}."
    exit 0
fi
if [ -z "${RS_DDS_PEERS:-}" ]; then
    log "WARNING: RS_DDS_PEERS empty in ${ENV_FILE}: the workstation may not discover the topics over Wi-Fi."
fi

# --- 3. network --------------------------------------------------------------
waited=0
until [ -n "$(iface_ip)" ]; do
    if [ "${waited}" -ge "${NET_WAIT}" ]; then
        log "${IFACE} has no IPv4 address after ${NET_WAIT} s: failing so systemd retries later."
        exit 1
    fi
    sleep 3; waited=$((waited + 3))
done
log "${IFACE} is $(iface_ip); DDS domain ${DOMAIN}, peer(s): ${RS_DDS_PEERS:-none}."

# --- 4. docker ---------------------------------------------------------------
if [ "${DRY_RUN}" = 0 ]; then
    command -v docker >/dev/null || { log "docker not installed"; exit 0; }
    docker image inspect "${IMAGE}" >/dev/null 2>&1 \
        || { log "image ${IMAGE} not found: build it in ~/files/rs-test first."; exit 0; }
    docker rm -f "${NAME}" >/dev/null 2>&1 || true   # a container left from an unclean shutdown
fi

cmd=(docker run --rm --name "${NAME}"
     --network host --ipc host --privileged
     -v /dev/bus/usb:/dev/bus/usb -v /run/udev:/run/udev:ro
     --log-driver none
     -e ROS_DOMAIN_ID="${DOMAIN}"
     -e RS_DDS_IFACE="${IFACE}"
     -e RS_DDS_PEERS="${RS_DDS_PEERS:-}")
for v in RS_COLOR_PROFILE RS_DEPTH_PROFILE RS_ALIGN RS_IMU RS_POINTCLOUD RS_INITIAL_RESET \
         RS_QOS RS_DDS_MULTICAST RS_DDS_TRACE RS_LOG_LEVEL; do
    [ -n "${!v:-}" ] && cmd+=(-e "${v}=${!v}")
done
cmd+=("${IMAGE}" camera)

if [ "${DRY_RUN}" = 1 ]; then
    log "dry run, would execute:"; printf '  %q' "${cmd[@]}"; echo
    exit 0
fi
log "starting ${NAME} (${IMAGE})"
exec "${cmd[@]}"
