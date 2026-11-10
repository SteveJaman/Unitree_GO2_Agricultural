#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# go2_realsense_boot.sh: started by go2-realsense.service when the Go2 boots.
#
#   1. waits up to CAMERA_WAIT s for the D435i (8086:0b3a) on USB;
#      not there -> logs it and exits 0: nothing is started, nothing retries;
#   2. waits up to NET_WAIT s for RS_DDS_IFACE (wlan0) to get an IPv4 address;
#   3. finds the workstation(s) the topics go to (the CycloneDDS unicast peers):
#        - by MAC (RS_HOST_MAC): kernel neighbour table, then a ping sweep of
#          the /24 when the MAC is not in it; follows DHCP changes;
#        - the IP a host session pushed (/tmp/go2-realsense-peer, written by
#          wifi_nav.sh / realsense_host.sh over SSH);
#        - fixed addresses (RS_DDS_PEERS), if any;
#   4. runs the go2-realsense container (the image's entrypoint writes the DDS
#      config and starts realsense2_camera_node as /camera/camera), re-checks
#      the peers every RS_PEER_CHECK s, and restarts the container when the
#      workstation's IP changes. A crashed container = exit 1, systemd restarts.
#
# Settings: /etc/go2-realsense.env (see go2-realsense.env).
# Logs:     journalctl -u go2-realsense -f
#
#   go2_realsense_boot.sh            # what the service runs
#   go2_realsense_boot.sh --dry-run  # checks + peers + the docker command, starts nothing
#   go2_realsense_boot.sh --peers    # only print the peers it would use
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
PEER_WAIT="${PEER_WAIT:-45}"            # at boot: how long to look for the workstation before starting anyway
PEER_CHECK="${RS_PEER_CHECK:-20}"       # seconds between peer re-checks
SWEEP_EVERY="${RS_SWEEP_EVERY:-60}"     # at most one ping sweep per this many seconds
PUSHED_FILE="${RS_PUSHED_PEER_FILE:-/tmp/go2-realsense-peer}"
PUSHED_MAX_AGE="${RS_PUSHED_MAX_AGE:-86400}"
SYSFS_USB="${RS_SYSFS_USB:-/sys/bus/usb/devices}"   # overridable for testing
MODE="run"
case "${1:-}" in --dry-run) MODE=dry ;; --peers) MODE=peers ;; esac

log() { echo "[go2-realsense] $*" >&2; }

valid_ip() { [[ "$1" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; }
norm_mac() { tr '[:upper:]-' '[:lower:]:' <<< "$1"; }

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

iface_cidr() { ip -4 -o addr show dev "${IFACE}" scope global 2>/dev/null | awk '{print $4; exit}'; }
iface_ip()   { local c; c="$(iface_cidr)"; echo "${c%/*}"; }

# IP of a MAC in the neighbour table of IFACE (entries that are not FAILED).
ip_of_mac() {
    local mac; mac="$(norm_mac "$1")"
    ip neigh show dev "${IFACE}" 2>/dev/null | awk -v m="${mac}" '
        $0 ~ /FAILED|INCOMPLETE/ {next}
        {for (i = 1; i < NF; i++) if ($i == "lladdr" && tolower($(i + 1)) == m) {print $1; exit}}'
}

# Fill the neighbour table: one ping to every address of our /24, in parallel (~2 s).
LAST_SWEEP=0
sweep() {
    local now base i
    now="$(date +%s)"
    [ $((now - LAST_SWEEP)) -ge "${SWEEP_EVERY}" ] || return 0
    LAST_SWEEP="${now}"
    base="$(iface_ip)"; base="${base%.*}"
    [ -n "${base}" ] || return 0
    for i in $(seq 1 254); do
        ping -c1 -W1 -I "${IFACE}" "${base}.${i}" >/dev/null 2>&1 &
    done
    wait
}

# Peers, space-separated, sorted, de-duplicated.
resolve_peers() {
    local out=() mac ip missing=0 age
    for mac in ${RS_HOST_MAC:-}; do
        ip="$(ip_of_mac "${mac}")"
        if [ -n "${ip}" ]; then
            ping -c1 -W1 -I "${IFACE}" "${ip}" >/dev/null 2>&1 || true   # keep the entry fresh
            out+=("${ip}")
        else
            missing=1
        fi
    done
    if [ "${missing}" = 1 ]; then
        sweep
        for mac in ${RS_HOST_MAC:-}; do
            ip="$(ip_of_mac "${mac}")"
            [ -n "${ip}" ] && out+=("${ip}")
        done
    fi
    if [ -f "${PUSHED_FILE}" ]; then
        age=$(( $(date +%s) - $(stat -c %Y "${PUSHED_FILE}" 2>/dev/null || echo 0) ))
        if [ "${age}" -le "${PUSHED_MAX_AGE}" ]; then
            for ip in $(tr ',' ' ' < "${PUSHED_FILE}"); do valid_ip "${ip}" && out+=("${ip}"); done
        fi
    fi
    for ip in ${RS_DDS_PEERS:-}; do valid_ip "${ip}" && out+=("${ip}"); done
    [ ${#out[@]} -gt 0 ] || return 0
    printf '%s\n' "${out[@]}" | grep -vx "$(iface_ip)" | sort -u | tr '\n' ' ' | sed 's/ $//'
}

if [ "${MODE}" = peers ]; then
    resolve_peers; echo; exit 0
fi

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

if [ "${DOMAIN}" = 0 ]; then
    log "ROS_DOMAIN_ID=0 is Unitree's onboard domain: refusing. Set 64 in ${ENV_FILE}."
    exit 0
fi

# --- 2. network --------------------------------------------------------------
waited=0
until [ -n "$(iface_ip)" ]; do
    if [ "${waited}" -ge "${NET_WAIT}" ]; then
        log "${IFACE} has no IPv4 address after ${NET_WAIT} s: failing so systemd retries later."
        exit 1
    fi
    sleep 3; waited=$((waited + 3))
done
log "${IFACE} is $(iface_cidr); DDS domain ${DOMAIN}."

# --- 3. peers ----------------------------------------------------------------
[ -n "${RS_HOST_MAC:-}${RS_DDS_PEERS:-}" ] || [ -f "${PUSHED_FILE}" ] \
    || log "WARNING: no RS_HOST_MAC or RS_DDS_PEERS in ${ENV_FILE}: relying on a host session to push its IP."
waited=0
PEERS="$(resolve_peers)"
while [ -z "${PEERS}" ] && [ "${waited}" -lt "${PEER_WAIT}" ] && [ "${MODE}" = run ]; do
    sleep 5; waited=$((waited + 5)); PEERS="$(resolve_peers)"
done
if [ -n "${PEERS}" ]; then
    log "workstation peer(s): ${PEERS} (MAC ${RS_HOST_MAC:-none})"
else
    log "workstation not found yet (MAC ${RS_HOST_MAC:-none}): starting anyway (multicast discovery only), re-checking every ${PEER_CHECK} s."
fi

# --- 4. container ------------------------------------------------------------
build_cmd() {   # $1 = peers
    CMD=(docker run --rm --name "${NAME}"
         --network host --ipc host --privileged
         -v /dev/bus/usb:/dev/bus/usb -v /run/udev:/run/udev:ro
         --log-driver none
         -e ROS_DOMAIN_ID="${DOMAIN}"
         -e RS_DDS_IFACE="${IFACE}"
         -e RS_DDS_PEERS="$1")
    local v
    for v in RS_COLOR_PROFILE RS_DEPTH_PROFILE RS_ALIGN RS_IMU RS_POINTCLOUD RS_INITIAL_RESET \
             RS_QOS RS_DDS_MULTICAST RS_DDS_TRACE RS_LOG_LEVEL; do
        [ -n "${!v:-}" ] && CMD+=(-e "${v}=${!v}")
    done
    CMD+=("${IMAGE}" camera)
}

if [ "${MODE}" = dry ]; then
    build_cmd "${PEERS}"
    log "dry run, would execute:"; printf '  %q' "${CMD[@]}" >&2; echo >&2
    exit 0
fi

command -v docker >/dev/null || { log "docker not installed"; exit 0; }
docker image inspect "${IMAGE}" >/dev/null 2>&1 \
    || { log "image ${IMAGE} not found: build it first (go2_realsense.sh build)."; exit 0; }
docker rm -f "${NAME}" >/dev/null 2>&1 || true   # left over from an unclean shutdown

CPID=""
start_container() {
    build_cmd "$1"
    log "starting ${NAME} (${IMAGE}), peers: ${1:-none}"
    "${CMD[@]}" </dev/null &
    CPID=$!
    CUR_PEERS="$1"
}
stop_container() {
    docker stop -t 10 "${NAME}" >/dev/null 2>&1 || true
    [ -n "${CPID}" ] && wait "${CPID}" 2>/dev/null
    CPID=""
}
trap 'log "stopping"; stop_container; exit 0' TERM INT

start_container "${PEERS}"
while :; do
    sleep "${PEER_CHECK}" & wait $!
    if ! kill -0 "${CPID}" 2>/dev/null; then
        wait "${CPID}"; rc=$?
        log "${NAME} exited (code ${rc})."
        exit 1
    fi
    NEW="$(resolve_peers)"
    if [ -n "${NEW}" ] && [ "${NEW}" != "${CUR_PEERS}" ]; then
        log "workstation peer(s) changed: '${CUR_PEERS:-none}' -> '${NEW}': restarting ${NAME}"
        stop_container
        start_container "${NEW}"
    fi
done
