#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# wifi_nav.sh
#
# Live waypoint navigation + autonomous frontier exploration over Wi-Fi
# using the go2-nav2-wifi stack, with a CUSTOM frontier explorer.
#
# One-command workflow:
#   1. Power on the dog and stand it
#   2. Run:  ./wifi_nav.sh
#   3. In RViz: wait ~12 s, set 2D Pose Estimate, click "Auto Explore: START"
#
# Bare invocation turns ON explore + camera. Use flags to turn them off:
#   --no-explore    manual waypoint mode (2D Goal Pose only)
#   --no-camera     skip the camera relay
#   --plan          preview only, no motion, no explore, no camera
#
# Network (Wi-Fi, DHCP — nothing is hardcoded):
#   Host IP  : auto-detected on GO2_IFACE (default enp0s3, VirtualBox bridge)
#   Robot IP : discovered on the same interface (cache, hostname, MAC scan)
#   The D1 arm interface (ARM_IFACE, default enp0s8) is never used.
#
# Usage:
#   ./wifi_nav.sh --arm                      # START EVERYTHING: nav + fold the D1 arm
#   ./wifi_nav.sh                            # explore + camera (default)
#   ./wifi_nav.sh --no-camera                # explore only
#   ./wifi_nav.sh --no-explore               # camera only, manual goals
#   ./wifi_nav.sh --explore --floor-filter   # + floor plane filter
#   ./wifi_nav.sh --nav                      # manual waypoint navigation
#   ./wifi_nav.sh --plan                     # preview only (no motion)
#   ./wifi_nav.sh --record                   # also save input data for replay
#   ./wifi_nav.sh --clean                    # clean both ends and exit
#   ./wifi_nav.sh --check                    # verify prerequisites only
#   ./wifi_nav.sh --setup-ssh                # one-time passwordless SSH setup
#   ./wifi_nav.sh --no-relay                 # skip auto relay startup
#   ./wifi_nav.sh --host-ip <IP>             # override host IP detection
#   ./wifi_nav.sh --robot-ip <IP>            # override robot discovery
#   ./wifi_nav.sh --iface <IFACE>            # Wi-Fi interface (default enp0s3)
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
NODES_DIR="$( cd "${SCRIPT_DIR}/../nodes" &> /dev/null && pwd )"

# shellcheck source=go2_net.sh
source "${SCRIPT_DIR}/go2_net.sh"

# --- Configuration ---------------------------------------------------------
NAV2_WIFI_DIR="${NAV2_WIFI_DIR:-$HOME/files/go2-nav2-wifi}"

GO2_RELAY_DOMAIN_ID="${GO2_RELAY_DOMAIN_ID:-64}"
MAPS_DIR="${NAV2_WIFI_DIR}/ws/maps/lidar3d"
ROBOT_MAPS_DIR="${ROBOT_MAPS_DIR:-go2-nav2-lidar3d-onboard/ws/maps/lidar3d}"
CONTAINER_NAME="${CONTAINER_NAME:-go2-lidar3d}"
FLOOR_FILTER_SCRIPT="${NODES_DIR}/go2_floor_filter.py"
EXPLORER_SCRIPT="${NODES_DIR}/frontier_explorer.py"
CAMERA_RELAY_SCRIPT="${NODES_DIR}/go2_camera_relay.py"
ARM_START_SCRIPT="${SCRIPT_DIR}/start_arm.sh"
ARM_LOG="${NAV2_WIFI_DIR}/ws/log/arm.log"

# apt scipy in the container is built against numpy 1.21; a newer pip numpy
# crashes it with "ValueError: numpy.dtype size changed".
NUMPY_PIN="${NUMPY_PIN:-numpy<1.25}"

# --- SAFETY thresholds -----------------------------------------------------
SAFETY_STOP_DISTANCE="${SAFETY_STOP_DISTANCE:-0.45}"
SAFETY_WARN_DISTANCE="${SAFETY_WARN_DISTANCE:-0.70}"
SAFETY_MAX_VEL_X="${SAFETY_MAX_VEL_X:-0.20}"
SAFETY_MAX_VEL_THETA="${SAFETY_MAX_VEL_THETA:-0.50}"
SAFETY_INFLATION_RADIUS="${SAFETY_INFLATION_RADIUS:-0.70}"

# --- Explore runtime overrides --------------------------------------------
EXPLORE_MAP_FRAME="${EXPLORE_MAP_FRAME:-lidar3d_map}"
EXPLORE_MAP_TOPIC="${EXPLORE_MAP_TOPIC:-/global_costmap/costmap}"
EXPLORE_STOP_TOPIC="${EXPLORE_STOP_TOPIC:-/cmd_vel}"
EXPLORE_MIN_FRONTIER_SIZE="${EXPLORE_MIN_FRONTIER_SIZE:-4}"
EXPLORE_MIN_KNOWN_CELLS="${EXPLORE_MIN_KNOWN_CELLS:-200}"
EXPLORE_BLIND_ENABLE="${EXPLORE_BLIND_ENABLE:-false}"
EXPLORE_COVERAGE_ENABLE="${EXPLORE_COVERAGE_ENABLE:-false}"
EXPLORE_RETURN_TO_INIT="${EXPLORE_RETURN_TO_INIT:-false}"

# --- Parse args ------------------------------------------------------------
# Default: explore + camera ON.
MODE="--nav"
ACTION="run"
EXPLORE="true"
FLOOR_FILTER="false"
CAMERA="true"
START_RELAY="true"
ARM="false"
HOST_IP_FLAG=""
ROBOT_IP_FLAG=""
export GO2_RECORD="${GO2_RECORD:-0}"

need_arg() { [ -n "${2:-}" ] || { echo "ERROR: $1 needs a value" >&2; exit 1; }; }

while [ $# -gt 0 ]; do
    case "$1" in
        --explore)      EXPLORE="true" ;;
        --no-explore)   EXPLORE="false" ;;
        --camera)       CAMERA="true" ;;
        --no-camera)    CAMERA="false" ;;
        --floor-filter) FLOOR_FILTER="true" ;;
        --plan)         MODE="--plan"; EXPLORE="false"; CAMERA="false" ;;
        --nav)          MODE="--nav";  EXPLORE="false" ;;
        --record)       export GO2_RECORD=1 ;;
        --clean)        ACTION="clean" ;;
        --check)        ACTION="check" ;;
        --setup-ssh)    ACTION="setup-ssh" ;;
        --no-relay)     START_RELAY="false" ;;
        --arm)          ARM="true" ;;
        --host-ip)      need_arg "$1" "${2:-}"; HOST_IP_FLAG="$2"; shift ;;
        --robot-ip)     need_arg "$1" "${2:-}"; ROBOT_IP_FLAG="$2"; shift ;;
        --iface)        need_arg "$1" "${2:-}"; GO2_IFACE="$2"; shift ;;
        -h|--help)      awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; exit 0 ;;
        *)              echo "Unknown option: $1" >&2; exit 1 ;;
    esac
    shift
done

# ===========================================================================
# Helpers
# ===========================================================================
robot_ssh() {
    ssh "${GO2_SSH_OPTS[@]}" "$@"
}

kill_explore_inside_container() {
    docker exec -u root "${CONTAINER_NAME}" bash -c '
        pids=$(ps -eo pid,cmd | grep -E "frontier_explorer|explore_lite|ros2 run explore" \
               | grep -v grep | awk "{print \$1}")
        if [ -n "$pids" ]; then
            for pid in $pids; do kill -9 "$pid" 2>/dev/null || true; done
        fi
        sleep 1
    ' 2>/dev/null || true
}

kill_floor_filter_inside_container() {
    docker exec -u root "${CONTAINER_NAME}" bash -c '
        pids=$(ps -eo pid,cmd | grep -E "go2_floor_filter" \
               | grep -v grep | awk "{print \$1}")
        if [ -n "$pids" ]; then
            for pid in $pids; do kill -9 "$pid" 2>/dev/null || true; done
        fi
    ' 2>/dev/null || true
}

kill_camera_relay_inside_container() {
    docker exec -u root "${CONTAINER_NAME}" bash -c '
        pids=$(ps -eo pid,cmd | grep -E "go2_camera_relay" \
               | grep -v grep | awk "{print \$1}")
        if [ -n "$pids" ]; then
            for pid in $pids; do kill -9 "$pid" 2>/dev/null || true; done
        fi
    ' 2>/dev/null || true
}

stop_exploration() {
    docker exec -u root "${CONTAINER_NAME}" bash -c '
        source /opt/ros/humble/setup.bash
        source /ws/install/setup.bash 2>/dev/null
        timeout 3 ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}" 2>/dev/null || true
    ' 2>/dev/null || true
}

# Remote scripts go over stdin (bash -s + quoted heredoc), never on the
# command line: a `pkill -f robot-relay-wifi` inside a `bash -c "..."` string
# matches its own shell and kills the script halfway through.
remote_kill_relays() {
    robot_ssh -o ConnectTimeout="${1:-5}" "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" bash -s <<'REMOTE_KILL'
pkill -9 -f 'robot[-_]relay[-_]wifi|robot-motion-wifi|go2_cmd_vel_tcp|robot_sport_bridge' 2>/dev/null || true
rm -f /tmp/robot-relay.pid /tmp/go2-relay-wifi.sock*
REMOTE_KILL
}

start_robot_relay() {
    if [ "${START_RELAY}" != "true" ]; then
        echo "[relay] Skipping (--no-relay)"
        return 0
    fi

    echo "[relay] Reaching robot at ${GO2_ROBOT_USER}@${GO2_ROBOT_IP}..."

    if ! robot_ssh -o ConnectTimeout=5 \
            "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" "true" >/dev/null 2>&1; then
        echo "[relay] ERROR: cannot SSH to robot."
        echo "         Run:  ./wifi_nav.sh --setup-ssh"
        return 1
    fi

    echo "[relay] Killing stale relay processes and starting FULL mode (data -> ${GO2_HOST_IP})..."

    # IPs are validated by go2_resolve_network, so they are safe to inline.
    local rc=0
    robot_ssh -o ConnectTimeout=10 "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" \
        "GO2_HOST_IP=${GO2_HOST_IP} GO2_ROBOT_IP=${GO2_ROBOT_IP} GO2_RELAY_DOMAIN_ID=${GO2_RELAY_DOMAIN_ID} bash -l -s" \
        <<'REMOTE_RELAY' || rc=$?
set -e
export GO2_HOST_IP GO2_ROBOT_IP GO2_RELAY_DOMAIN_ID

pkill -9 -f 'robot[-_]relay[-_]wifi|robot-motion-wifi|go2_cmd_vel_tcp|robot_sport_bridge' 2>/dev/null || true
rm -f /tmp/go2-relay-wifi.sock*
sleep 3

if ss -tln 2>/dev/null | grep -q ':17999 '; then
    echo '[relay] ERROR: port 17999 still held after pkill'
    ss -tlnp 2>/dev/null | grep ':17999 ' || true
    exit 1
fi

echo "[relay] Robot side: GO2_HOST_IP=${GO2_HOST_IP}"
# </dev/null: this script is being read from stdin; without it the relay
# would swallow the rest of the script.
setsid nohup bash "$HOME/robot-relay-wifi.sh" </dev/null >/tmp/robot-relay.log 2>&1 &
echo $! > /tmp/robot-relay.pid

for _ in $(seq 1 20); do
    if ss -tln 2>/dev/null | grep -q ':17999 '; then
        echo '[relay] FULL mode confirmed on port 17999'
        exit 0
    fi
    sleep 1
done

echo '[relay] ERROR: port 17999 never bound. Last 30 lines:'
tail -30 /tmp/robot-relay.log 2>/dev/null || true
exit 1
REMOTE_RELAY

    if [ "${rc}" -ne 0 ]; then
        echo "[relay] ERROR: relay start failed."
        echo "[relay] Debug: ssh ${GO2_ROBOT_USER}@${GO2_ROBOT_IP} 'cat /tmp/robot-relay.log'"
        return 1
    fi
    return 0
}

stop_robot_relay() {
    if [ "${START_RELAY}" != "true" ] || [ -z "${GO2_ROBOT_IP}" ]; then
        return 0
    fi
    remote_kill_relays 3 >/dev/null 2>&1 || true
}

do_clean() {
    echo "[clean] Killing local stale processes..."
    pkill -9 -f "rviz2"             2>/dev/null || true
    pkill -9 -f "mapping.sh"        2>/dev/null || true
    pkill -9 -f "pointlio_mapping"  2>/dev/null || true
    pkill -9 -f "go2_cloud_stamp"   2>/dev/null || true
    pkill -9 -f "frontier_explorer" 2>/dev/null || true
    pkill -9 -f "go2_camera_relay"  2>/dev/null || true
    pkill -9 -f "safety_watchdog"   2>/dev/null || true

    if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${CONTAINER_NAME}"; then
        echo "[clean] Killing stale helpers inside container..."
        kill_explore_inside_container
        kill_floor_filter_inside_container
        kill_camera_relay_inside_container
    fi

    echo "[clean] Stopping local Docker containers..."
    if [ -f "${NAV2_WIFI_DIR}/docker/docker-compose.yml" ]; then
        (cd "${NAV2_WIFI_DIR}/docker" && docker compose down 2>/dev/null) || true
    fi

    if [ -z "${GO2_ROBOT_IP}" ]; then
        echo "[clean] WARN: robot IP unknown — skipping remote cleanup (pass --robot-ip)"
        return 0
    fi
    if ! robot_ssh -o ConnectTimeout=5 \
            "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" "true" >/dev/null 2>&1; then
        echo "[clean] WARN: cannot reach robot at ${GO2_ROBOT_IP} — skipping remote cleanup"
        return 0
    fi

    echo "[clean] Killing remote stale relays..."
    remote_kill_relays 5 >/dev/null 2>&1 || true

    echo "[clean] Removing incomplete runs on the robot..."
    robot_ssh "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" \
        "ROBOT_MAPS_DIR=$(printf '%q' "${ROBOT_MAPS_DIR}") bash -s" <<'REMOTE_CLEAN' 2>/dev/null || true
BASE="$HOME/${ROBOT_MAPS_DIR}"
if [ -d "$BASE" ]; then
    removed=0
    for d in "$BASE"/run-*/; do
        [ -d "$d" ] || continue
        if [ ! -f "$d/result.json" ]; then
            echo "  removing incomplete: $(basename "$d")"
            rm -rf "$d"
            removed=$((removed + 1))
        fi
    done
    echo "[clean] removed $removed incomplete run(s)"
    ls -1 "$BASE" 2>/dev/null | head -10 || true
fi
REMOTE_CLEAN

    echo "[clean] Done."
}

do_setup_ssh() {
    echo "[ssh] Setting up passwordless SSH to ${GO2_ROBOT_USER}@${GO2_ROBOT_IP}"
    if [ ! -f "$HOME/.ssh/id_ed25519" ] && [ ! -f "$HOME/.ssh/id_rsa" ]; then
        echo "[ssh] No key found — generating ~/.ssh/id_ed25519"
        mkdir -p "$HOME/.ssh" && chmod 700 "$HOME/.ssh"
        ssh-keygen -t ed25519 -N "" -f "$HOME/.ssh/id_ed25519"
    fi
    echo "[ssh] You will be asked for the '${GO2_ROBOT_USER}' password once."
    ssh-copy-id -o HostKeyAlias=go2-robot -o StrictHostKeyChecking=accept-new \
        "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}"
    if robot_ssh -o ConnectTimeout=5 "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" true; then
        go2_save_robot_ip "${GO2_ROBOT_IP}"
        echo "[ssh] OK — passwordless SSH works. Robot IP cached for discovery."
    else
        echo "[ssh] ERROR: key copied but BatchMode login still fails." >&2
        return 1
    fi
}

# --- Container: DDS bind follows the current DHCP address -----------------
container_exists()  { docker ps -a --format '{{.Names}}' 2>/dev/null | grep -qx "${CONTAINER_NAME}"; }
container_running() { docker ps    --format '{{.Names}}' 2>/dev/null | grep -qx "${CONTAINER_NAME}"; }

container_host_ip() {
    docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "${CONTAINER_NAME}" 2>/dev/null \
        | sed -n 's/^GO2_HOST_IP=//p' | head -n1
}

container_dds_bind() {
    docker exec "${CONTAINER_NAME}" grep -oP 'NetworkInterface address="\K[^"]+' \
        /tmp/go2-wifi-cyclonedds.xml 2>/dev/null | head -n1 || true
}

# The CycloneDDS bind address is baked in when the container is created, so
# a new DHCP lease means the container must be recreated.
ensure_container() {
    local cur
    if container_exists; then
        cur="$(container_host_ip)"
        if [ -n "${cur}" ] && [ "${cur}" != "${GO2_HOST_IP}" ]; then
            echo "[container] ${CONTAINER_NAME} was created for ${cur}; host is now ${GO2_HOST_IP} — recreating"
            docker rm -f "${CONTAINER_NAME}" >/dev/null
        fi
    fi
    if ! container_exists; then
        echo "[container] Creating ${CONTAINER_NAME} for ${GO2_HOST_IP} (setup-lidar3d.sh --laptop)..."
        (cd "${NAV2_WIFI_DIR}" && ./setup-lidar3d.sh --laptop) || return 1
    fi
    if ! container_running; then
        echo "[container] Starting ${CONTAINER_NAME}..."
        docker start "${CONTAINER_NAME}" >/dev/null || return 1
        sleep 2
    fi
    if ! container_running; then
        echo "[container] ERROR: ${CONTAINER_NAME} exits right after start. Last log lines:"
        docker logs --tail 20 "${CONTAINER_NAME}" 2>&1 | sed 's/^/    /'
        echo "  If this is 'numpy.dtype size changed', rebuild the image (the Dockerfile is now pinned):"
        echo "    docker rm -f ${CONTAINER_NAME} && (cd ${NAV2_WIFI_DIR} && ./setup-lidar3d.sh --laptop)"
        return 1
    fi
}

# Persistent fix: pin numpy in the upstream Dockerfile(s) so rebuilds keep it.
patch_upstream_dockerfiles() {
    local f
    while IFS= read -r f; do
        grep -q 'numpy<1.25' "$f" && continue
        if grep -qiE '^[[:space:]]*USER[[:space:]]+' "$f"; then
            echo "[numpy] WARN: ${f} sets USER — add by hand: RUN pip3 install '${NUMPY_PIN}'"
            continue
        fi
        cat >> "$f" <<EOF

# Added by Unitree_GO2_Agricultural/scripts/runtime/wifi_nav.sh:
# apt scipy is built against numpy 1.21; newer numpy crashes it with
# "ValueError: numpy.dtype size changed".
RUN (python3 -m pip --version >/dev/null 2>&1 || (apt-get update && apt-get install -y --no-install-recommends python3-pip)) \\
 && python3 -m pip install --no-cache-dir '${NUMPY_PIN}'
EOF
        echo "[numpy] Pinned ${NUMPY_PIN} in ${f} (applies on next image build)"
    done < <(find "${NAV2_WIFI_DIR}/docker" -maxdepth 2 -type f \
                \( -name 'Dockerfile' -o -name 'Dockerfile.*' -o -name '*.Dockerfile' \) 2>/dev/null)
}

container_numpy_version() {
    docker exec "${CONTAINER_NAME}" python3 -c 'import numpy; print(numpy.__version__)' 2>/dev/null || true
}

numpy_ok() {   # $1 = version; true if < 1.25
    [ -n "$1" ] && [ "$1" != "1.25" ] && \
        [ "$(printf '%s\n1.25\n' "$1" | sort -V | head -n1)" = "$1" ]
}

# Immediate fix: pin numpy in the running container before the stack launches.
pin_container_numpy() {
    local ver
    ver="$(container_numpy_version)"
    if numpy_ok "${ver}"; then
        echo "[numpy] container numpy ${ver} ............. OK"
        return 0
    fi
    echo "[numpy] container numpy ${ver:-missing} — installing '${NUMPY_PIN}'..."
    docker exec -u root "${CONTAINER_NAME}" python3 -m pip install --no-cache-dir "${NUMPY_PIN}" \
        || { echo "[numpy] ERROR: pip install failed inside ${CONTAINER_NAME}"; return 1; }
    if ! docker exec "${CONTAINER_NAME}" python3 -c 'import scipy.spatial' >/dev/null 2>&1; then
        echo "[numpy] WARN: scipy still fails to import with numpy $(container_numpy_version)."
        echo "        A pip-installed package may be built against a newer numpy:"
        echo "        docker exec ${CONTAINER_NAME} python3 -m pip list 2>/dev/null | grep -iE 'numpy|scipy|pandas'"
    fi
}

# DDS must bind to the Wi-Fi address only — never the arm's interface.
check_dds_bind() {
    local bind xml
    bind="$(container_dds_bind)"
    xml="$(docker exec "${CONTAINER_NAME}" cat /tmp/go2-wifi-cyclonedds.xml 2>/dev/null || true)"
    if [ -n "${xml}" ] && grep -q "${ARM_IFACE}" <<< "${xml}"; then
        echo "[checks] ERROR: container DDS config references the arm interface ${ARM_IFACE}."
        return 1
    fi
    if [ -n "${xml}" ] && [ -n "${ARM_HOST_CIDR:-}" ] && grep -qF "${ARM_HOST_CIDR%/*}" <<< "${xml}"; then
        echo "[checks] ERROR: container DDS config references the arm address ${ARM_HOST_CIDR%/*}."
        return 1
    fi
    if [ -z "${bind}" ]; then
        echo "[checks] DDS bind address ...................... (config not generated yet)"
    elif [ "${bind}" = "${GO2_HOST_IP}" ]; then
        echo "[checks] DDS bind ${bind} (${GO2_IFACE}) ......... OK"
    else
        echo "[checks] ERROR: DDS bound to ${bind}, but ${GO2_IFACE} is ${GO2_HOST_IP}."
        echo "         Recreate: docker rm -f ${CONTAINER_NAME} && ./wifi_nav.sh"
        return 1
    fi
}

# ===========================================================================
# Network: resolve host + robot IPs (DHCP — nothing hardcoded)
# ===========================================================================
ROBOT_REQUIRED="true"
[ "${ACTION}" = "clean" ] && ROBOT_REQUIRED="false"

echo "[net] Resolving network on ${GO2_IFACE} (arm interface ${ARM_IFACE} is ignored)..."
if ! go2_resolve_network "${HOST_IP_FLAG}" "${ROBOT_IP_FLAG}" "${ROBOT_REQUIRED}"; then
    echo "ERROR: network resolution failed." >&2
    exit 1
fi

# The arm is optional: a bad arm setup disables it, it never blocks navigation.
ARM_STATUS="off"
if [ "${ARM}" = "true" ] && [ "${ACTION}" != "clean" ]; then
    if [ ! -f "${ARM_START_SCRIPT}" ]; then
        echo "[arm] WARN: ${ARM_START_SCRIPT} missing — arm disabled"
        ARM="false"; ARM_STATUS="disabled (start_arm.sh missing)"
    elif go2_check_arm_iface; then
        ARM_STATUS="on (${ARM_IFACE} ${ARM_HOST_CIDR}, DDS domain 0; Go2 on ${GO2_IFACE}, domain ${GO2_RELAY_DOMAIN_ID})"
    else
        echo "[arm] WARN: arm interface check failed — continuing with navigation only"
        ARM="false"; ARM_STATUS="disabled (${ARM_IFACE} not ready)"
    fi
fi

# ===========================================================================
# Dispatch: setup-ssh / clean
# ===========================================================================
if [ "${ACTION}" = "setup-ssh" ]; then
    do_setup_ssh
    exit $?
fi

if [ "${ACTION}" = "clean" ]; then
    echo "================================================================"
    echo " Cleaning stale processes and partial runs"
    echo "================================================================"
    do_clean
    exit 0
fi

# ===========================================================================
# Banner
# ===========================================================================
echo "================================================================"
echo " Go2 Nav2 Wi-Fi navigation"
echo "================================================================"
echo "  Action      : ${ACTION}"
echo "  Mode        : ${MODE}"
echo "  Explore     : ${EXPLORE}"
echo "  Floor filter: ${FLOOR_FILTER}"
echo "  Camera      : ${CAMERA}"
echo "  Auto relay  : ${START_RELAY}"
echo "  D1 arm      : ${ARM_STATUS}"
echo "  GO2_RECORD  : ${GO2_RECORD}"
echo "  Interface   : ${GO2_IFACE}  (arm ${ARM_IFACE}: never used for Go2 traffic)"
echo "  Host IP     : ${GO2_HOST_IP}  [${GO2_HOST_IP_SRC}]"
echo "  Robot IP    : ${GO2_ROBOT_IP}  [${GO2_ROBOT_IP_SRC}]"
echo "  Robot user  : ${GO2_ROBOT_USER}"
echo "  Container   : ${CONTAINER_NAME}"
echo "  Repo        : ${NAV2_WIFI_DIR}"
echo

# ===========================================================================
# Prerequisite checks
# ===========================================================================
if ! command -v docker >/dev/null 2>&1; then
    echo "ERROR: docker not found." >&2; exit 1
fi
if ! docker ps >/dev/null 2>&1; then
    echo "ERROR: cannot talk to Docker daemon. Try: newgrp docker" >&2; exit 1
fi
[ -d "${NAV2_WIFI_DIR}" ] || { echo "ERROR: ${NAV2_WIFI_DIR} not found." >&2; exit 1; }
[ -f "${NAV2_WIFI_DIR}/mapping.sh" ] || { echo "ERROR: mapping.sh not found." >&2; exit 1; }
echo "[checks] docker + repo present .................... OK"

if robot_ssh -o ConnectTimeout=3 \
        "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" "echo OK" >/dev/null 2>&1; then
    echo "[checks] passwordless SSH to robot ............... OK"
    go2_save_robot_ip "${GO2_ROBOT_IP}"
else
    echo "[checks] passwordless SSH to robot ............... NOT SET UP"
    echo "         Run:  ./wifi_nav.sh --setup-ssh"
    if [ "${START_RELAY}" = "true" ] && [ "${MODE}" != "--plan" ]; then
        echo "[checks] ERROR: auto relay requires SSH. Use --no-relay to skip."
        exit 1
    fi
fi

if [ "${EXPLORE}" = "true" ]; then
    if [ -f "${EXPLORER_SCRIPT}" ]; then
        echo "[checks] frontier_explorer.py present ............ OK"
    else
        echo "[checks] frontier_explorer.py present ............ MISSING (${EXPLORER_SCRIPT})"
        EXPLORE="false"
    fi

    if docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}" && \
       docker exec -u root "${CONTAINER_NAME}" bash -c \
           "source /opt/ros/humble/setup.bash; \
            source /ws/install/setup.bash 2>/dev/null; \
            ros2 pkg list 2>/dev/null | grep -q go2_explore_panel" \
       2>/dev/null; then
        echo "[checks] go2_explore_panel built .................. OK"
    else
        echo "[checks] go2_explore_panel built .................. MISSING"
    fi
fi

if [ "${FLOOR_FILTER}" = "true" ]; then
    if [ -f "${FLOOR_FILTER_SCRIPT}" ]; then
        echo "[checks] floor filter script present .............. OK"
    else
        echo "[checks] floor filter script present .............. MISSING (${FLOOR_FILTER_SCRIPT})"
        FLOOR_FILTER="false"
    fi
fi

if [ "${CAMERA}" = "true" ]; then
    if [ -f "${CAMERA_RELAY_SCRIPT}" ]; then
        echo "[checks] camera relay script present .............. OK"
    else
        echo "[checks] camera relay script present .............. MISSING (${CAMERA_RELAY_SCRIPT})"
        CAMERA="false"
    fi
fi

if [ "${ACTION}" = "check" ]; then
    if container_exists; then
        cur_ip="$(container_host_ip)"
        if [ "${cur_ip}" = "${GO2_HOST_IP}" ]; then
            echo "[checks] container created for ${cur_ip} ........ OK"
        else
            echo "[checks] container created for ${cur_ip:-?} ........ STALE (will be recreated on run)"
        fi
        if container_running; then
            check_dds_bind || true
            ver="$(container_numpy_version)"
            if numpy_ok "${ver}"; then
                echo "[checks] container numpy ${ver} ............. OK"
            else
                echo "[checks] container numpy ${ver:-missing} ............. NEEDS PIN (fixed on run)"
            fi
        fi
    else
        echo "[checks] container ${CONTAINER_NAME} ................. MISSING (created on run)"
    fi
    if [ "${ARM}" = "true" ]; then
        echo "[checks] arm interface ${ARM_IFACE} ${ARM_HOST_CIDR} ....... OK"
        if grep -qxF 'FOLDED_ANGLES = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0, 0.0]' \
               <(tr -d '\r' < "${NODES_DIR}/d1_arm_calibration.py"); then
            echo "[checks] arm calibration ...................... NOT DONE (./start_arm.sh --read)"
        else
            echo "[checks] arm calibration ...................... OK"
        fi
    fi
fi

# ===========================================================================
# Pre-flight cleanup
# ===========================================================================
if [ "${ACTION}" != "check" ]; then
    echo
    echo "[pre-flight] Cleaning stale processes + incomplete runs..."
    do_clean
    echo
fi

if [ "${ACTION}" = "check" ]; then
    echo "All checks done."
    exit 0
fi

# ===========================================================================
# Container: current host IP + numpy pin
# ===========================================================================
patch_upstream_dockerfiles
if ! ensure_container; then
    echo "ERROR: could not bring up ${CONTAINER_NAME}." >&2
    exit 1
fi
pin_container_numpy || exit 1
check_dds_bind || exit 1
echo

# ===========================================================================
# Start robot relay (automated)
# ===========================================================================
if [ "${MODE}" != "--plan" ] && [ "${START_RELAY}" = "true" ]; then
    echo "================================================================"
    echo " Starting robot relay (FULL mode)"
    echo "================================================================"
    if ! start_robot_relay; then
        echo
        echo "ERROR: could not start robot relay."
        echo "  Check: ssh ${GO2_ROBOT_USER}@${GO2_ROBOT_IP} 'cat /tmp/robot-relay.log'"
        exit 1
    fi
    echo
fi

# ===========================================================================
# Explore-mode banner
# ===========================================================================
if [ "${EXPLORE}" = "true" ]; then
    echo "================================================================"
    echo " EXPLORE MODE — the robot will move autonomously"
    echo "================================================================"
    echo
    echo " In RViz:"
    echo "   1. Wait ~12 s for Nav2 lifecycle"
    echo "   2. 2D Pose Estimate — set robot's current position"
    echo "   3. Click 'Auto Explore: START' in the Go2 Explore panel"
    echo "   4. Robot explores autonomously; click STOP to end"
    echo
    echo " Emergency stop: physical remote or Ctrl+C in this terminal."
    echo "================================================================"
    echo
fi

# ===========================================================================
# Launch
# ===========================================================================
cd "${NAV2_WIFI_DIR}"
export GO2_HOST_IP GO2_ROBOT_IP GO2_RELAY_DOMAIN_ID

EXPLORE_HOST_PID=""
ARM_PID=""

cleanup() {
    echo
    echo "[wifi_nav] Shutting down..."
    # Don't pull the container out from under a fold in progress.
    if [ -n "${ARM_PID}" ] && kill -0 "${ARM_PID}" 2>/dev/null; then
        echo "[wifi_nav] Waiting up to 20 s for the arm fold to finish..."
        for _ in $(seq 1 20); do
            kill -0 "${ARM_PID}" 2>/dev/null || break
            sleep 1
        done
    fi
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${CONTAINER_NAME}"; then
        echo "[wifi_nav] SAFETY: publishing stop to /explore/resume"
        stop_exploration
    fi
    if [ -n "${EXPLORE_HOST_PID}" ]; then
        kill "${EXPLORE_HOST_PID}" 2>/dev/null || true
    fi
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${CONTAINER_NAME}"; then
        kill_explore_inside_container
        kill_floor_filter_inside_container
        kill_camera_relay_inside_container
    fi
    (cd "${NAV2_WIFI_DIR}/docker" && docker compose down 2>/dev/null) || true
    stop_robot_relay
    echo "[wifi_nav] Done."
}
trap cleanup EXIT INT TERM

echo "Launching: ./mapping.sh --3d ${MODE}"
echo

if [ "${EXPLORE}" = "true" ] || [ "${FLOOR_FILTER}" = "true" ] || [ "${CAMERA}" = "true" ]; then
    (
        sleep 20

        # ---------- Floor filter ----------
        if [ "${FLOOR_FILTER}" = "true" ]; then
            docker exec -u root "${CONTAINER_NAME}" bash -c '
                pids=$(ps -eo pid,cmd | grep -E "go2_floor_filter" \
                       | grep -v grep | awk "{print \$1}")
                if [ -n "$pids" ]; then
                    for pid in $pids; do kill -9 "$pid" 2>/dev/null || true; done
                fi
            ' 2>/dev/null || true

            docker cp "${FLOOR_FILTER_SCRIPT}" \
                "${CONTAINER_NAME}:/tmp/go2_floor_filter.py" 2>/dev/null || true

            docker exec -u root "${CONTAINER_NAME}" bash -c \
                "source /opt/ros/humble/setup.bash && \
                 source /ws/install/setup.bash 2>/dev/null; \
                 nohup python3 /tmp/go2_floor_filter.py \
                     --ros-args \
                     -p input_topic:=/lidar3d/registered \
                     -p output_topic:=/lidar3d/obstacle_points \
                     -p floor_margin:=0.20 \
                     > /tmp/floor_filter.log 2>&1 &" \
                2>/dev/null || true
        fi

        # ---------- Camera relay ----------
        if [ "${CAMERA}" = "true" ]; then
            echo "[camera] Killing stale camera relay instances..."
            kill_camera_relay_inside_container
            echo "[camera] Copying go2_camera_relay.py into container..."
            docker cp "${CAMERA_RELAY_SCRIPT}" \
                "${CONTAINER_NAME}:/tmp/go2_camera_relay.py" 2>/dev/null || true
            echo "[camera] Starting camera relay..."
            docker exec -u root "${CONTAINER_NAME}" bash -c \
                "source /opt/ros/humble/setup.bash && \
                 source /ws/install/setup.bash 2>/dev/null; \
                 nohup python3 /tmp/go2_camera_relay.py \
                     --ros-args \
                     -p source_topic:=/frontvideostream \
                     -p output_topic:=/camera/image_raw \
                     -p frame_id:=camera_link \
                     -p resolution:=720p \
                     > /tmp/camera_relay.log 2>&1 &" \
                2>/dev/null || true
        fi

        # ---------- Custom frontier explorer ----------
        if [ "${EXPLORE}" = "true" ]; then
            echo "[explore] Killing stale explorer instances..."
            docker exec -u root "${CONTAINER_NAME}" bash -c '
                pids=$(ps -eo pid,cmd | grep -E "frontier_explorer|explore_lite|ros2 run explore" \
                       | grep -v grep | awk "{print \$1}")
                if [ -n "$pids" ]; then
                    for pid in $pids; do kill -9 "$pid" 2>/dev/null || true; done
                fi
                sleep 1
            ' 2>/dev/null || true

            echo "[explore] Copying frontier_explorer.py into container..."
            docker cp "${EXPLORER_SCRIPT}" \
                "${CONTAINER_NAME}:/tmp/frontier_explorer.py" 2>/dev/null || true

            echo "[explore] Starting custom frontier explorer..."
            docker exec -u root "${CONTAINER_NAME}" bash -c \
                "source /opt/ros/humble/setup.bash && \
                 source /ws/install/setup.bash 2>/dev/null; \
                 exec python3 /tmp/frontier_explorer.py \
                     --ros-args \
                     -p robot_base_frame:=base_link \
                     -p map_frame:=${EXPLORE_MAP_FRAME} \
                     -p map_topic:=${EXPLORE_MAP_TOPIC} \
                     -p goal_topic:=/goal_pose \
                     -p cmd_vel_topic:=/cmd_vel_nav \
                     -p stop_topic:=${EXPLORE_STOP_TOPIC} \
                     -p min_frontier_size:=${EXPLORE_MIN_FRONTIER_SIZE} \
                     -p min_known_cells:=${EXPLORE_MIN_KNOWN_CELLS} \
                     -p blind_enable:=${EXPLORE_BLIND_ENABLE} \
                     -p blind_trigger_cells:=100 \
                     -p blind_step_distance:=0.6 \
                     -p blind_step_speed:=0.12 \
                     -p coverage_enable:=${EXPLORE_COVERAGE_ENABLE} \
                     -p return_to_init:=${EXPLORE_RETURN_TO_INIT} \
                     -p tick_hz:=1.0" \
                > "${NAV2_WIFI_DIR}/ws/log/frontier.log" 2>&1 &
        fi
    ) &
    EXPLORE_HOST_PID=$!
    echo "[wifi_nav] background helpers will start after ~20 s (host PID ${EXPLORE_HOST_PID})"
    echo
fi

# ---------- D1 arm: fold + release in the background ----------
# start_arm.sh runs the arm with CYCLONEDDS_URI unset and its own DDS config
# bound to ARM_IFACE (domain 0), so it never joins the Go2's Wi-Fi DDS.
if [ "${ARM}" = "true" ]; then
    mkdir -p "$(dirname "${ARM_LOG}")"
    echo "[arm] Folding the D1 arm in the background (log: ${ARM_LOG})"
    (
        if ARM_IFACE="${ARM_IFACE}" CONTAINER_NAME="${CONTAINER_NAME}" NODES_DIR="${NODES_DIR}" \
                bash "${ARM_START_SCRIPT}" > "${ARM_LOG}" 2>&1; then
            echo "[arm] Arm folded and released."
        else
            echo "[arm] WARNING: arm start failed — navigation continues. Last lines of ${ARM_LOG}:"
            tail -5 "${ARM_LOG}" 2>/dev/null | sed 's/^/    /'
        fi
    ) &
    ARM_PID=$!
fi

./mapping.sh --3d ${MODE} || true

# ===========================================================================
# Post-run: slam log tail
# ===========================================================================
echo
echo "================================================================"
echo " Last 20 lines of the newest slam log:"
echo "================================================================"
LATEST_LOG=$(ls -1t "${NAV2_WIFI_DIR}/ws/log/" 2>/dev/null \
    | grep "^mapping" | head -1 || true)
if [ -n "${LATEST_LOG}" ] && \
   [ -f "${NAV2_WIFI_DIR}/ws/log/${LATEST_LOG}/slam.log" ]; then
    tail -20 "${NAV2_WIFI_DIR}/ws/log/${LATEST_LOG}/slam.log"
else
    echo "  (no slam.log found)"
fi
echo

# ===========================================================================
# Post-run: frontier explorer log tail
# ===========================================================================
if [ "${EXPLORE}" = "true" ] && \
   [ -f "${NAV2_WIFI_DIR}/ws/log/frontier.log" ]; then
    echo "================================================================"
    echo " Last 20 lines of frontier.log:"
    echo "================================================================"
    tail -20 "${NAV2_WIFI_DIR}/ws/log/frontier.log"
    echo
fi

# ===========================================================================
# Post-run: camera relay log tail
# ===========================================================================
if [ "${CAMERA}" = "true" ] && \
   docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${CONTAINER_NAME}"; then
    echo "================================================================"
    echo " Last 10 lines of camera_relay.log:"
    echo "================================================================"
    docker exec -u root "${CONTAINER_NAME}" tail -10 /tmp/camera_relay.log 2>/dev/null || echo "  (no log)"
    echo
fi

# ===========================================================================
# Post-run: archived runs summary
# ===========================================================================
echo "================================================================"
echo " Archived runs in ${MAPS_DIR}:"
echo "================================================================"
if [ -d "${MAPS_DIR}" ]; then
    ls -1t "${MAPS_DIR}" 2>/dev/null | head -5 | while read -r d; do
        [ -d "${MAPS_DIR}/${d}" ] || continue
        size=$(du -sh "${MAPS_DIR}/${d}" 2>/dev/null | cut -f1)
        if [ -f "${MAPS_DIR}/${d}/result.json" ]; then
            printf "  %-45s  (%s)  ✓ complete\n" "$d" "$size"
        else
            printf "  %-45s  (%s)  ✗ partial\n"  "$d" "$size"
        fi
    done
else
    echo "  (no runs yet)"
fi
echo
echo "Run './wifi_mapping.sh --list' for full details."
