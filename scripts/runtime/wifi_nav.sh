#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# wifi_nav.sh
#
# Live waypoint navigation + autonomous frontier exploration over Wi-Fi
# using the go2-nav2-wifi stack, with a CUSTOM frontier explorer.
#
# --drive diff|hybrid|omni (default hybrid, or env GO2_DRIVE): how Nav2 drives
# the dog.
#   diff    forward-only arcs, Regulated Pure Pursuit + rotation shim, no
#           sideways speed: the go2-nav2-wifi setup the real dog has always
#           used. Stuck recovery can only step back.
#   hybrid  the same forward-facing driving (the lidar looks where the dog
#           walks); only stuck recovery uses the omni moves: step back or
#           sidestep 0.3 m into known free space, then retry.
#   omni    MPPI (Omni) walks backwards and sidesteps during normal driving
#           too, and the planner uses omnidirectional lattice moves:
#           config/nav2/go2_omni.yaml, scripts/tools/nav2_omni.py.
#   --diff / --hybrid / --omni are short for these.
# Comparison and how to test both: docs/navigation/locomotion_comparison.md
# Every session runs go2_stuck_recovery.py: when a leg is aborted (dog facing a
# wall) it steps back (and aside with --omni) and retries; --no-recovery: off.
#
# One-command workflow:
#   1. Power on the dog and stand it
#   2. Run:  ./wifi_nav.sh
#   3. In RViz: wait ~12 s, set 2D Pose Estimate, click "Auto Explore: START"
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
NODES_DIR="$( cd "${SCRIPT_DIR}/../nodes" &> /dev/null && pwd )"

# shellcheck source-path=SCRIPTDIR source=go2_net.sh
source "${SCRIPT_DIR}/go2_net.sh"

# --- Configuration ---------------------------------------------------------
NAV2_WIFI_DIR="${NAV2_WIFI_DIR:-$HOME/files/go2-nav2-wifi}"

GO2_RELAY_DOMAIN_ID="${GO2_RELAY_DOMAIN_ID:-64}"
MAPS_DIR="${NAV2_WIFI_DIR}/ws/maps/lidar3d"
ROBOT_MAPS_DIR="${ROBOT_MAPS_DIR:-go2-nav2-lidar3d-onboard/ws/maps/lidar3d}"
CONTAINER_NAME="${CONTAINER_NAME:-go2-lidar3d}"
FLOOR_FILTER_SCRIPT="${NODES_DIR}/go2_floor_filter.py"
RECOVERY_SCRIPT="${NODES_DIR}/go2_stuck_recovery.py"
EXPLORER_SCRIPT="${NODES_DIR}/frontier_explorer.py"
CAMERA_RELAY_SCRIPT="${NODES_DIR}/go2_camera_relay.py"
ROBOT_CAMERA_DIR="$( cd "${SCRIPT_DIR}/../robot" &> /dev/null && pwd )"   # scripts/robot: go2_camera_wifi.*, arm_bridge_wifi.* (run on the robot)
CAMERA_CALIB="${CAMERA_CALIB:-$( cd "${SCRIPT_DIR}/../.." &> /dev/null && pwd )/config/camera/front_camera.yaml}"
CAMERA_WIDTH="${CAMERA_WIDTH:-960}"   # robot-side JPEG width (1920 native); the calibration must match
CAMERA_FPS="${CAMERA_FPS:-5}"
ARM_START_SCRIPT="${SCRIPT_DIR}/start_arm.sh"
SIM_RUN="$( cd "${SCRIPT_DIR}/../../gazebo" &> /dev/null && pwd )/run_sim.sh"
ARM_LOG="${NAV2_WIFI_DIR}/ws/log/arm.log"
ARM_BRIDGE_PORT="${ARM_BRIDGE_PORT:-18100}"

NUMPY_PIN="${NUMPY_PIN:-numpy<1.25}"

# --- SAFETY thresholds -----------------------------------------------------
SAFETY_STOP_DISTANCE="${SAFETY_STOP_DISTANCE:-0.45}"
SAFETY_WARN_DISTANCE="${SAFETY_WARN_DISTANCE:-0.70}"
SAFETY_MAX_VEL_X="${SAFETY_MAX_VEL_X:-0.20}"
SAFETY_MAX_VEL_THETA="${SAFETY_MAX_VEL_THETA:-0.50}"
SAFETY_INFLATION_RADIUS="${SAFETY_INFLATION_RADIUS:-0.70}"

# --- Explore runtime overrides --------------------------------------------
# Explorer parameters live in config/frontier_explorer.yaml (shared by the
# robot and --sim). EXPLORE_<NAME>=value in the environment overrides one
# parameter for this run, e.g. EXPLORE_MIN_GOAL_DIST=1.0.
EXPLORER_PARAMS="${EXPLORER_PARAMS:-$( cd "${SCRIPT_DIR}/../../config" &> /dev/null && pwd )/frontier_explorer.yaml}"
EXPLORE_OVERRIDES=""
while IFS='=' read -r _name _value; do
    _param="$(echo "${_name#EXPLORE_}" | tr '[:upper:]' '[:lower:]')"
    [[ "${_value}" =~ ^[A-Za-z0-9_./:+-]+$ ]] || { echo "ERROR: ${_name} has an unsupported value" >&2; exit 2; }
    EXPLORE_OVERRIDES+=" -p ${_param}:=${_value}"
done < <(env | grep -E '^EXPLORE_[A-Z0-9_]+=' | grep -vE '^EXPLORE_HOST_PID=' || true)

# --- Parse args ------------------------------------------------------------
MODE="--nav"
ACTION="run"
EXPLORE="true"
FLOOR_FILTER="false"
CAMERA="true"
DRIVE="${GO2_DRIVE:-hybrid}"   # diff | hybrid | omni (--drive); OMNI below follows it
RECOVERY="true"
START_RELAY="true"
ARM="false"
ARM_LINK="wired"      # --arm-wifi: the arm plugged into the Go2, through its Wi-Fi bridge
SIM="false"
SIM_STARTED="false"
SIM_OWNER="wifi-nav-sim-$$"
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
        --sync-clock)   ACTION="sync-clock" ;;
        --no-relay)     START_RELAY="false" ;;
        --arm)          ARM="true" ;;
        --arm-wifi)     ARM="true"; ARM_LINK="wifi" ;;
        --drive)        need_arg "$1" "${2:-}"; DRIVE="$2"; shift ;;
        --omni)         DRIVE="omni" ;;
        --diff)         DRIVE="diff" ;;
        --hybrid)       DRIVE="hybrid" ;;
        --no-recovery)  RECOVERY="false" ;;
        --sim)          SIM="true" ;;
        --host-ip)      need_arg "$1" "${2:-}"; HOST_IP_FLAG="$2"; shift ;;
        --robot-ip)     need_arg "$1" "${2:-}"; ROBOT_IP_FLAG="$2"; shift ;;
        --iface)        need_arg "$1" "${2:-}"; GO2_IFACE="$2"; shift ;;
        -h|--help)      awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; exit 0 ;;
        *)              echo "Unknown option: $1" >&2; exit 1 ;;
    esac
    shift
done
# OMNI: Nav2 itself drives omnidirectionally. SIDESTEP: stuck recovery may
# step aside (hybrid sends it straight to /cmd_vel, past the vy-clamping smoother).
case "${DRIVE}" in
    diff)   OMNI="false"; SIDESTEP="false" ;;
    hybrid) OMNI="false"; SIDESTEP="true" ;;
    omni)   OMNI="true";  SIDESTEP="true" ;;
    *) echo "Unknown --drive '${DRIVE}': use diff, hybrid or omni" >&2; exit 1 ;;
esac

# ===========================================================================
# Helpers
# ===========================================================================
robot_ssh() {
    ssh "${GO2_SSH_OPTS[@]}" "$@"
}

# --- Robot/laptop clock offset ---------------------------------------------
# Point-LIO stamps TF with the Jetson clock; Nav2 runs on the laptop clock.
# With skew, the rotation controller looks up a stale robot orientation and
# over-turns, then crawls/shudders (go2-nav2-wifi
# docs/NAVIGATION-INCIDENT-2026-09-12.md: 5 s skew). Without internet the
# Jetson has no time source (it has shown 1970). Four-timestamp estimate,
# best of 5 by round trip: offset = robot - laptop midpoint, in seconds.
CLOCK_MAX_SKEW="${CLOCK_MAX_SKEW:-0.1}"
ROBOT_CLOCK_OFFSET=""
ROBOT_CLOCK_RTT=""
measure_robot_clock_offset() {
    local best_rtt="" best_off="" t0 r t1 rtt off
    # Reuse one SSH connection: a fresh connection spends most of its round
    # trip on the handshake before `date` runs, which skews the midpoint
    # estimate by up to a few hundred ms over Wi-Fi.
    local mux=(-o ControlMaster=auto -o "ControlPath=/tmp/go2-clock-%C" -o ControlPersist=10)
    robot_ssh -T -o ConnectTimeout=5 "${mux[@]}" "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" true 2>/dev/null || return 1
    for _ in 1 2 3 4 5 6 7 8; do
        t0="$(date +%s.%N)"
        r="$(robot_ssh -T -o ConnectTimeout=5 "${mux[@]}" "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" 'date +%s.%N' 2>/dev/null)" || return 1
        t1="$(date +%s.%N)"
        [[ "${r}" =~ ^[0-9]+\.[0-9]+$ ]] || return 1
        read -r rtt off < <(awk -v a="$t0" -v b="$r" -v c="$t1" 'BEGIN {printf "%.6f %.6f\n", c - a, b - (a + c) / 2}')
        if [ -z "${best_rtt}" ] || awk -v x="$rtt" -v y="$best_rtt" 'BEGIN {exit !(x < y)}'; then
            best_rtt="$rtt"; best_off="$off"
        fi
    done
    ROBOT_CLOCK_OFFSET="${best_off}"
    # Uncertainty is half the round trip (SSH connection setup dominates it).
    ROBOT_CLOCK_RTT="${best_rtt}"
}

clock_skew_too_large() {
    awk -v o="${ROBOT_CLOCK_OFFSET}" -v m="${CLOCK_MAX_SKEW}" 'BEGIN {if (o < 0) o = -o; exit !(o > m)}'
}

# Set the Jetson clock from this laptop (asks for the robot's sudo password).
# The robot computes "now - offset" itself after sudo authenticates, so the
# time spent typing the password does not add error.
do_sync_clock() {
    measure_robot_clock_offset || { echo "[clock] ERROR: cannot read the robot clock over SSH." >&2; return 1; }
    echo "[clock] robot - laptop = ${ROBOT_CLOCK_OFFSET} s; setting the robot clock (enter the robot sudo password)"
    robot_ssh -t "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" \
        "sudo sh -c 'date -s \"@\$(python3 -c \"import time; print(f\\\"{time.time() - (${ROBOT_CLOCK_OFFSET}):.6f}\\\")\")\" >/dev/null'" || return 1
    measure_robot_clock_offset || return 1
    echo "[clock] now robot - laptop = ${ROBOT_CLOCK_OFFSET} s (±$(awk -v r="${ROBOT_CLOCK_RTT}" 'BEGIN {printf "%.3f", r / 2}') s)"
    if clock_skew_too_large; then
        echo "[clock] ERROR: still above ${CLOCK_MAX_SKEW} s." >&2
        return 1
    fi
    echo "[clock] OK. The robot keeps this until it reboots; NTP/chrony on the robot makes it permanent."
}

# Nodes we start with `docker exec` must use the same DDS setup as the
# go2-session stack (Cyclone + its generated config). A bare root exec gets
# the image's default RMW and no CYCLONEDDS_URI, so it cannot see Nav2/RViz
# (in --sim the stack is loopback-only) and the explorer never receives
# /explore/resume (nor the stop on exit).
DDS_EXEC_ENV=(-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
              -e CYCLONEDDS_URI=file:///tmp/go2-wifi-cyclonedds.xml)

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

kill_recovery_inside_container() {
    docker exec -u root "${CONTAINER_NAME}" bash -c \
        'pkill -f "^python3 /tmp/go2_stuck_recovery.py" 2>/dev/null || true' 2>/dev/null || true
}

# MPPI is not in the upstream go2-lidar3d image; install it on first use.
ensure_mppi() {
    if docker exec "${CONTAINER_NAME}" test -d /opt/ros/humble/share/nav2_mppi_controller; then
        return 0
    fi
    echo "[omni] Installing ros-humble-nav2-mppi-controller in ${CONTAINER_NAME} (needs internet)..."
    docker exec -u root "${CONTAINER_NAME}" bash -c \
        'apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq ros-humble-nav2-mppi-controller' \
        >/dev/null 2>&1
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
    docker exec -u root "${DDS_EXEC_ENV[@]}" "${CONTAINER_NAME}" bash -c '
        source /opt/ros/humble/setup.bash
        source /ws/install/setup.bash 2>/dev/null
        timeout -k 2 3 ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}" 2>/dev/null || true
    ' 2>/dev/null || true
}

remote_kill_relays() {
    robot_ssh -T -o ConnectTimeout="${1:-5}" "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" bash -s <<'REMOTE_KILL'
pkill -9 -f 'robot[-_]relay[-_]wifi|robot-motion-wifi|go2_cmd_vel_tcp|robot_sport_bridge|go2_camera_wifi|go2_camera_jpeg_cli' 2>/dev/null || true
rm -f /tmp/robot-relay.pid /tmp/go2-relay-wifi.sock*
REMOTE_KILL
}

start_robot_relay() {
    if [ "${START_RELAY}" != "true" ]; then
        echo "[relay] Skipping (--no-relay)"
        return 0
    fi

    echo "[relay] Reaching robot at ${GO2_ROBOT_USER}@${GO2_ROBOT_IP}..."

    if ! robot_ssh -T -o ConnectTimeout=5 \
            "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" "true" >/dev/null 2>&1; then
        echo "[relay] ERROR: cannot SSH to robot."
        echo "         Run:  ./wifi_nav.sh --setup-ssh"
        return 1
    fi

    echo "[relay] Killing stale relay processes and starting FULL mode (data -> ${GO2_HOST_IP})..."

    local out=""
    # SSH returns 255 when backgrounding a process, so we capture output and check for success
    out=$(robot_ssh -T -o ConnectTimeout=10 "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" \
        "WN_HOST_IP=${GO2_HOST_IP} WN_ROBOT_IP=${GO2_ROBOT_IP} WN_DOMAIN_ID=${GO2_RELAY_DOMAIN_ID} bash -s" <<'REMOTE_RELAY' 2>&1 || true
set -e
export GO2_HOST_IP="${WN_HOST_IP}" GO2_ROBOT_IP="${WN_ROBOT_IP}" GO2_RELAY_DOMAIN_ID="${WN_DOMAIN_ID}"

pkill -9 -f 'robot[-_]relay[-_]wifi|robot-motion-wifi|go2_cmd_vel_tcp|robot_sport_bridge|go2_camera_wifi|go2_camera_jpeg_cli' 2>/dev/null || true
rm -f /tmp/go2-relay-wifi.sock*
sleep 3

if ss -tln 2>/dev/null | grep -q ':17999 '; then
    echo '[relay] ERROR: port 17999 still held after pkill'
    ss -tlnp 2>/dev/null | grep ':17999 ' || true
    exit 1
fi

echo "[relay] Robot side: GO2_HOST_IP=${GO2_HOST_IP}"
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
    )

    echo "$out"

    if [[ "$out" == *"FULL mode confirmed on port 17999"* ]]; then
        return 0
    fi

    echo "[relay] ERROR: relay start failed."
    echo "[relay] Debug: ssh ${GO2_ROBOT_USER}@${GO2_ROBOT_IP} 'cat /tmp/robot-relay.log'"
    return 1
}

stop_robot_relay() {
    if [ "${START_RELAY}" != "true" ] || [ -z "${GO2_ROBOT_IP}" ]; then
        return 0
    fi
    remote_kill_relays 3 >/dev/null 2>&1 || true
}

# --arm-wifi: the D1 arm plugged into the Go2's Ethernet port. Copy
# scripts/robot/arm_bridge_wifi.* to ~/go2-agri-arm and start it detached
# (setsid: it outlives this SSH session); it serves the laptop over TCP.
# The one-message package it needs is built once by hand (--build), never here.
ARM_ROBOT_DIR="go2-agri-arm"
start_arm_bridge() {
    local robot="${GO2_ROBOT_USER}@${GO2_ROBOT_IP}"
    robot_ssh -T -n -o ConnectTimeout=5 "${robot}" \
        "pkill -TERM -f 'arm_bridge_[w]ifi' 2>/dev/null; mkdir -p ~/${ARM_ROBOT_DIR}" 2>/dev/null \
        || { echo "[arm] cannot reach ${robot} over SSH"; return 1; }
    scp -q -r "${GO2_SSH_OPTS[@]}" "${ROBOT_CAMERA_DIR}/arm_bridge_wifi.sh" "${ROBOT_CAMERA_DIR}/arm_bridge_wifi.py" \
        "${ROBOT_CAMERA_DIR}/unitree_arm" "${NAV2_WIFI_DIR}/ws/scripts/robot-source-unitree-ros.sh" \
        "${robot}:${ARM_ROBOT_DIR}/" 2>/dev/null \
        || { echo "[arm] could not copy the arm bridge to the robot"; return 1; }
    if ! robot_ssh -T -n -o ConnectTimeout=5 "${robot}" "test -f ~/${ARM_ROBOT_DIR}/ws/install/setup.bash" 2>/dev/null; then
        echo "[arm] The arm bridge is not built on the robot yet. Once, on the robot:"
        echo "        bash ~/${ARM_ROBOT_DIR}/arm_bridge_wifi.sh --build"
        return 1
    fi
    robot_ssh -T -n -o ConnectTimeout=5 "${robot}" \
        "GO2_HOST_IP=${GO2_HOST_IP} ARM_BRIDGE_PORT=${ARM_BRIDGE_PORT} setsid nohup bash ~/${ARM_ROBOT_DIR}/arm_bridge_wifi.sh > /tmp/arm_bridge_wifi.log 2>&1 < /dev/null &" \
        2>/dev/null || { echo "[arm] could not start the arm bridge"; return 1; }
    for _ in $(seq 1 20); do
        if (exec 3<>"/dev/tcp/${GO2_ROBOT_IP}/${ARM_BRIDGE_PORT}") 2>/dev/null; then
            echo "[arm] Bridge up: ${GO2_ROBOT_IP}:${ARM_BRIDGE_PORT} (robot log /tmp/arm_bridge_wifi.log)"
            return 0
        fi
        sleep 1
    done
    echo "[arm] Bridge did not open port ${ARM_BRIDGE_PORT}; on the robot: tail /tmp/arm_bridge_wifi.log"
    return 1
}

# SIGTERM, not -9: the bridge locks a held arm before it exits.
stop_arm_bridge() {
    [ "${ARM}" = "true" ] && [ "${ARM_LINK}" = "wifi" ] && [ -n "${GO2_ROBOT_IP}" ] || return 0
    robot_ssh -T -n -o ConnectTimeout=3 "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" \
        "pkill -TERM -f 'arm_bridge_[w]ifi'" >/dev/null 2>&1 || true
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

    if docker ps --format '{{.Names}}' 2>/dev/null | grep -x "${CONTAINER_NAME}" >/dev/null; then
        echo "[clean] Killing stale helpers inside container..."
        kill_explore_inside_container
        kill_recovery_inside_container
        kill_floor_filter_inside_container
        kill_camera_relay_inside_container
    fi

    echo "[clean] Stopping local Docker containers..."
    if [ -f "${NAV2_WIFI_DIR}/docker/docker-compose.yml" ]; then
        (cd "${NAV2_WIFI_DIR}/docker" && docker compose down 2>/dev/null) || true
    fi

    if [ "${SIM}" = "true" ]; then
        echo "[clean] Simulation: no remote robot to clean."
        return 0
    fi
    if [ -z "${GO2_ROBOT_IP}" ]; then
        echo "[clean] WARN: robot IP unknown — skipping remote cleanup (pass --robot-ip)"
        return 0
    fi
    if ! robot_ssh -T -o ConnectTimeout=5 \
            "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" "true" >/dev/null 2>&1; then
        echo "[clean] WARN: cannot reach robot at ${GO2_ROBOT_IP} — skipping remote cleanup"
        return 0
    fi

    echo "[clean] Killing remote stale relays..."
    remote_kill_relays 5 >/dev/null 2>&1 || true

    echo "[clean] Removing incomplete runs on the robot..."
    robot_ssh -T "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" \
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
    if robot_ssh -T -o ConnectTimeout=5 "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" true; then
        go2_save_robot_ip "${GO2_ROBOT_IP}"
        echo "[ssh] OK — passwordless SSH works. Robot IP cached for discovery."
    else
        echo "[ssh] ERROR: key copied but BatchMode login still fails." >&2
        return 1
    fi
}

# --- Container: DDS bind follows the current DHCP address -----------------
container_exists()  { docker ps -a --format '{{.Names}}' 2>/dev/null | grep -x "${CONTAINER_NAME}" >/dev/null; }
container_running() { docker ps    --format '{{.Names}}' 2>/dev/null | grep -x "${CONTAINER_NAME}" >/dev/null; }

container_env() {   # $1 = variable name
    docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "${CONTAINER_NAME}" 2>/dev/null \
        | sed -n "s/^${1}=//p" | head -n1
}
container_host_ip() { container_env GO2_HOST_IP; }

container_dds_bind() {
    docker exec "${CONTAINER_NAME}" grep -oP 'NetworkInterface address="\K[^"]+' \
        /tmp/go2-wifi-cyclonedds.xml 2>/dev/null | head -n1 || true
}

ensure_container() {
    local cur_host cur_robot
    if container_exists && [ "${SIM}" != "true" ]; then
        cur_host="$(container_env GO2_HOST_IP)"
        cur_robot="$(container_env GO2_ROBOT_IP)"
        if { [ -n "${cur_host}" ] && [ "${cur_host}" != "${GO2_HOST_IP}" ]; } ||
           { [ -n "${cur_robot}" ] && [ "${cur_robot}" != "${GO2_ROBOT_IP}" ]; }; then
            echo "[container] ${CONTAINER_NAME} was created for host ${cur_host:-?} / robot ${cur_robot:-?};"
            echo "            now host ${GO2_HOST_IP} / robot ${GO2_ROBOT_IP} — recreating"
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
        return 1
    fi
    ensure_ros_log_dir
}

# ROS_LOG_DIR (/tmp/go2-lidar3d-ros-log in the container) must stay writable for
# the container user: the root `docker exec`s here (ros2 topic pub, ...) create
# it owned by root otherwise, and RViz / the motion bridge, which run as the
# user, then abort with "Permission denied" and the whole session stops.
ensure_ros_log_dir() {
    docker exec -u root "${CONTAINER_NAME}" bash -c \
        'd="${ROS_LOG_DIR:-/tmp/go2-lidar3d-ros-log}"; mkdir -p "$d" && chmod 1777 "$d"' \
        >/dev/null 2>&1 || true
}

patch_upstream_dockerfiles() {
    local f
    local f last_user
    while IFS= read -r f; do
        grep -q 'numpy<1.25' "$f" && continue
        last_user="$(awk 'toupper($1) == "USER" {u = $2} END {print u}' "$f")"
        {
            echo
            echo "# Added by Unitree_GO2_Agricultural/scripts/runtime/wifi_nav.sh:"
            echo "USER root"
            echo "RUN (python3 -m pip --version >/dev/null 2>&1 || (apt-get update && apt-get install -y --no-install-recommends python3-pip)) \\"
            echo " && python3 -m pip install --no-cache-dir '${NUMPY_PIN}'"
            if [ -n "${last_user}" ] && [ "${last_user}" != root ] && [ "${last_user}" != 0 ]; then
                echo "USER ${last_user}"
            fi
        } >> "$f"
        echo "[numpy] Pinned ${NUMPY_PIN} in ${f} (applies on next image build)"
    done < <(find "${NAV2_WIFI_DIR}/docker" -maxdepth 2 -type f \
                \( -name 'Dockerfile' -o -name 'Dockerfile.*' -o -name '*.Dockerfile' \) \
                ! -name '*.dockerignore' 2>/dev/null)
}

container_numpy_version() {
    docker exec "${CONTAINER_NAME}" python3 -c 'import numpy; print(numpy.__version__)' 2>/dev/null || true
}

numpy_ok() {   # $1 = version; true if < 1.25
    [ -n "$1" ] && [ "$1" != "1.25" ] && \
        [ "$(printf '%s\n1.25\n' "$1" | sort -V | head -n1)" = "$1" ]
}

pin_container_numpy() {
    local ver
    ver="$(container_numpy_version)"
    if numpy_ok "${ver}"; then
        echo "[numpy] container numpy ${ver} ............. OK"
        return 0
    fi
    echo "[numpy] container numpy ${ver:-missing} — installing '${NUMPY_PIN}'..."
    if ! docker exec -u root "${CONTAINER_NAME}" python3 -m pip install --no-cache-dir "${NUMPY_PIN}"; then
        echo "[numpy] WARN: pip install failed inside ${CONTAINER_NAME} (no internet?)."
        return 0
    fi
    if ! docker exec "${CONTAINER_NAME}" python3 -c 'import scipy.spatial' >/dev/null 2>&1; then
        echo "[numpy] WARN: scipy still fails to import with numpy $(container_numpy_version)."
    fi
}

# The camera relay (go2_camera_relay.py) needs OpenCV; the go2-lidar3d image
# has none, and a recreated container (new host / robot IP) loses anything
# installed by hand. Headless wheel, no deps: it must not replace the pinned numpy
# (4.8+ needs numpy >= 1.22 at import).
OPENCV_PIN="${OPENCV_PIN:-opencv-python-headless==4.7.0.72}"
ensure_container_opencv() {
    if docker exec "${CONTAINER_NAME}" python3 -c 'import cv2' >/dev/null 2>&1; then
        echo "[camera] container OpenCV ................. OK"
        return 0
    fi
    echo "[camera] OpenCV missing in ${CONTAINER_NAME} — installing '${OPENCV_PIN}'..."
    if ! docker exec -u root "${CONTAINER_NAME}" python3 -m pip install -q --no-cache-dir --no-deps "${OPENCV_PIN}" ||
       ! docker exec "${CONTAINER_NAME}" python3 -c 'import cv2' >/dev/null 2>&1; then
        echo "[camera] WARN: OpenCV install failed (no internet?): no camera relay, YOLO/VLM get no images."
    fi
}

check_dds_bind() {
    local bind xml
    bind="$(container_dds_bind)"
    xml="$(docker exec "${CONTAINER_NAME}" cat /tmp/go2-wifi-cyclonedds.xml 2>/dev/null || true)"
    if [ -n "${xml}" ] && grep -q "${ARM_IFACE}" <<< "${xml}"; then
        echo "[checks] ERROR: container DDS config references the arm interface ${ARM_IFACE}."
        return 1
    fi
    if [ -z "${bind}" ]; then
        echo "[checks] DDS bind address ...................... (config not generated yet)"
    elif [ "${bind}" = "${GO2_HOST_IP}" ]; then
        echo "[checks] DDS bind ${bind} (${GO2_IFACE}) ......... OK"
    else
        echo "[checks] DDS config from the last session bound ${bind}; this session rebinds to ${GO2_HOST_IP}"
    fi
}

# ===========================================================================
# Network: resolve host + robot IPs (DHCP — nothing hardcoded)
# ===========================================================================
ROBOT_REQUIRED="true"
[ "${ACTION}" = "clean" ] && ROBOT_REQUIRED="false"

if [ "${SIM}" = "true" ]; then
    echo "[net] Simulation: robot = this machine (Gazebo); no discovery, SSH or relay"
    if [ -n "${HOST_IP_FLAG}" ]; then
        go2_valid_ip "${HOST_IP_FLAG}" || { echo "ERROR: --host-ip '${HOST_IP_FLAG}' is not an IPv4 address." >&2; exit 1; }
        GO2_HOST_IP="${HOST_IP_FLAG}"; GO2_HOST_IP_SRC="--host-ip"
    elif go2_detect_host_ip 2>/dev/null; then
        GO2_HOST_IP_SRC="auto-detected on ${GO2_IFACE}"
    else
        GO2_HOST_IP=127.0.0.1; GO2_HOST_IP_SRC="loopback (no Wi-Fi address)"
    fi
    GO2_ROBOT_IP="${GO2_HOST_IP}"; GO2_ROBOT_IP_SRC="simulator on this machine"
    START_RELAY="false"
    ARM_IFACE="${SIM_ARM_IFACE:-simarm0}"
    export GO2_HOST_IP GO2_ROBOT_IP ARM_IFACE
    [ -x "${SIM_RUN}" ] || [ -f "${SIM_RUN}" ] || { echo "ERROR: ${SIM_RUN} not found." >&2; exit 1; }
else
    echo "[net] Resolving network on ${GO2_IFACE} (arm interface ${ARM_IFACE} is ignored)..."
    if ! go2_resolve_network "${HOST_IP_FLAG}" "${ROBOT_IP_FLAG}" "${ROBOT_REQUIRED}"; then
        echo "ERROR: network resolution failed." >&2
        exit 1
    fi
fi

# The arm is optional: a bad arm setup disables it, it never blocks navigation.
ARM_STATUS="off"
if [ "${ARM}" = "true" ] && [ "${ACTION}" != "clean" ] && [ "${SIM}" = "true" ]; then
    ARM_STATUS="on (simulated D1 on ${ARM_IFACE}; checked after the simulator starts)"
elif [ "${ARM}" = "true" ] && [ "${ACTION}" != "clean" ] && [ "${ARM_LINK}" = "wifi" ]; then
    ARM_STATUS="on (plugged into the Go2, over Wi-Fi: arm_bridge_wifi on the robot, TCP ${ARM_BRIDGE_PORT})"
elif [ "${ARM}" = "true" ] && [ "${ACTION}" != "clean" ]; then
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

if [ "${ACTION}" = "setup-ssh" ]; then
    do_setup_ssh
    exit $?
fi

if [ "${ACTION}" = "sync-clock" ]; then
    [ "${SIM}" = "true" ] && { echo "--sync-clock is for the real robot (not --sim)." >&2; exit 2; }
    do_sync_clock
    exit $?
fi

if [ "${ACTION}" = "clean" ]; then
    echo "================================================================"
    echo " Cleaning stale processes and partial runs"
    echo "================================================================"
    do_clean
    exit 0
fi

echo "================================================================"
echo " Go2 Nav2 Wi-Fi navigation"
echo "================================================================"
echo "  Action      : ${ACTION}"
echo "  Mode        : ${MODE}"
echo "  Explore     : ${EXPLORE}"
echo "  Floor filter: ${FLOOR_FILTER}"
echo "  Camera      : ${CAMERA}"
case "${DRIVE}" in
    diff)   drive_note="RPP + rotation shim: forward arcs, turn in place" ;;
    hybrid) drive_note="forward arcs like diff; step back / sidestep only when stuck" ;;
    omni)   drive_note="MPPI Omni: forward, reverse, sidestep" ;;
esac
echo "  Drive       : ${DRIVE}  (${drive_note})"
echo "  Recovery    : ${RECOVERY}$( [ "${RECOVERY}" = "true" ] && echo "  (aborted leg -> step back$( [ "${SIDESTEP}" = "true" ] && echo "/aside" ), retry)" )"
echo "  Auto relay  : ${START_RELAY}"
echo "  D1 arm      : ${ARM_STATUS}"
echo "  Simulation  : ${SIM}$( [ "${SIM}" = "true" ] && echo "  (Gazebo: ${SIM_RUN})" )"
echo "  GO2_RECORD  : ${GO2_RECORD}"
echo "  Interface   : ${GO2_IFACE}  (arm ${ARM_IFACE}: never used for Go2 traffic)"
echo "  Host IP     : ${GO2_HOST_IP}  [${GO2_HOST_IP_SRC}]"
echo "  Robot IP    : ${GO2_ROBOT_IP}  [${GO2_ROBOT_IP_SRC}]"
echo "  Robot user  : ${GO2_ROBOT_USER}"
echo "  Container   : ${CONTAINER_NAME}"
echo "  Repo        : ${NAV2_WIFI_DIR}"
echo

# ===========================================================================
# Single instance: a second run's pre-flight cleanup pkill -9's the first
# run's robot relay mid-start ("Killed", "port 17999 never bound").
# ===========================================================================
if [ "${ACTION}" = "run" ]; then
    exec 8>"/tmp/wifi_nav.${USER}.lock"
    if ! flock -n 8; then
        echo "ERROR: wifi_nav.sh is already running (another terminal?). Stop it first." >&2
        exit 1
    fi
fi

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

# docker cp below is best-effort (|| true), so a missing node script would
# otherwise run a stale /tmp copy in the container. Fail early instead.
for f in "${EXPLORER_SCRIPT}" "${CAMERA_RELAY_SCRIPT}" "${FLOOR_FILTER_SCRIPT}"; do
    [ -f "$f" ] || { echo "ERROR: $f not found (set NODES_DIR)." >&2; exit 1; }
done
echo "[checks] node scripts present ($(basename "${NODES_DIR}")/) ........ OK"

if [ "${SIM}" = "true" ]; then
    if docker image inspect "${SIM_IMAGE:-go2-sim:local}" >/dev/null 2>&1; then
        echo "[checks] simulator image go2-sim:local ............ OK"
    else
        echo "[checks] simulator image go2-sim:local ............ MISSING (run ./gazebo/setup_gazebo.sh)"
        [ "${ACTION}" = "check" ] || exit 1
    fi
elif robot_ssh -T -o ConnectTimeout=3 \
        "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" "echo OK" >/dev/null 2>&1; then
    echo "[checks] passwordless SSH to robot ............... OK"
    go2_save_robot_ip "${GO2_ROBOT_IP}"
    if measure_robot_clock_offset; then
        if clock_skew_too_large; then
            echo "[checks] robot clock offset ${ROBOT_CLOCK_OFFSET} s ......... TOO LARGE (max ${CLOCK_MAX_SKEW} s)"
            echo "         Skew makes Nav2 act on stale robot poses: over-turning, shudder, wall hits."
            echo "         Fix:  ./wifi_nav.sh --sync-clock     (override: GO2_ALLOW_CLOCK_SKEW=1)"
            if [ "${ACTION}" = "run" ] && [ "${GO2_ALLOW_CLOCK_SKEW:-0}" != 1 ]; then
                exit 1
            fi
        else
            echo "[checks] robot clock offset ${ROBOT_CLOCK_OFFSET} s ......... OK"
        fi
    else
        echo "[checks] robot clock offset ......................... UNKNOWN (could not read robot clock)"
    fi
else
    echo "[checks] passwordless SSH to robot ............... NOT SET UP"
    echo "         Run:  ./wifi_nav.sh --setup-ssh"
    if [ "${ACTION}" = "run" ]; then
        echo "[checks] ERROR: mapping.sh starts SLAM and motion on the robot over SSH."
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

    if docker ps --format '{{.Names}}' | grep -x "${CONTAINER_NAME}" >/dev/null && \
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

# DDS over Wi-Fi: a 73 KB camera frame or a LiDAR cloud is ~50 UDP packets,
# and losing one loses the whole sample. With the Ubuntu default (208 KB)
# the sockets overflow and the camera arrives at ~0.2 of its 5 fps.
RMEM_MIN=8388608
rmem_max="$(cat /proc/sys/net/core/rmem_max 2>/dev/null || echo 0)"
rmem_default="$(cat /proc/sys/net/core/rmem_default 2>/dev/null || echo 0)"
if [ "${rmem_max}" -ge "${RMEM_MIN}" ] && [ "${rmem_default}" -ge "${RMEM_MIN}" ]; then
    echo "[checks] UDP receive buffers ($(( rmem_max / 1048576 )) MB) ............. OK"
else
    echo "[checks] UDP receive buffers ................... TOO SMALL (rmem_max ${rmem_max}, rmem_default ${rmem_default})"
    echo "         Camera frames and LiDAR clouds get dropped. Fix once (then restart the session):"
    echo "           printf 'net.core.rmem_max=16777216\\nnet.core.rmem_default=8388608\\n' | sudo tee /etc/sysctl.d/60-go2-dds.conf"
    echo "           sudo sysctl --system"
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
    if [ "${ARM}" = "true" ] && [ "${ARM_LINK}" = "wifi" ]; then
        echo "[checks] arm link ......................... Go2 bridge over Wi-Fi (TCP ${ARM_BRIDGE_PORT})"
    elif [ "${ARM}" = "true" ]; then
        echo "[checks] arm interface ${ARM_IFACE} ${ARM_HOST_CIDR} ....... OK"
        if grep -qxF 'FOLDED_ANGLES = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0, 0.0]' \
               <(tr -d '\r' < "${NODES_DIR}/d1_arm_calibration.py"); then
            echo "[checks] arm calibration ...................... NOT DONE (./start_arm.sh --read)"
        else
            echo "[checks] arm calibration ...................... OK"
        fi
    fi
fi

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

patch_upstream_dockerfiles
if ! ensure_container; then
    echo "ERROR: could not bring up ${CONTAINER_NAME}." >&2
    exit 1
fi
pin_container_numpy
[ "${CAMERA}" = "true" ] && ensure_container_opencv
check_dds_bind || exit 1
echo

if [ "${SIM}" = "true" ]; then
    echo "================================================================"
    echo " Starting the Gazebo simulator"
    echo "================================================================"
    docker ps --format '{{.Names}}' | grep -x go2-sim >/dev/null || SIM_STARTED="true"
    if ! GO2_HOST_IP="${GO2_HOST_IP}" GO2_ROBOT_IP="${GO2_ROBOT_IP}" GO2_RELAY_DOMAIN_ID="${GO2_RELAY_DOMAIN_ID}" \
            SIM_ARM="$( [ "${ARM}" = "true" ] && echo 1 || echo 0 )" SIM_ARM_IFACE="${ARM_IFACE}" \
            SIM_CAMERA="${SIM_CAMERA:-$( [ "${CAMERA}" = "true" ] && echo 1 || echo 0 )}" \
            bash "${SIM_RUN}" up; then
        echo "ERROR: simulator did not start (see: ./gazebo/run_sim.sh logs)." >&2
        exit 1
    fi
    if [ "${ARM}" = "true" ]; then
        if go2_check_arm_iface; then
            ARM_STATUS="on (simulated D1 on ${ARM_IFACE} ${ARM_HOST_CIDR})"
        else
            echo "[arm] WARN: simulated arm interface not ready — continuing with navigation only"
            ARM="false"
        fi
    fi
    if [ "${CAMERA}" = "true" ]; then
        echo "[camera] Simulation publishes /camera/image_raw directly; camera relay not needed."
        CAMERA="false"
    fi
    echo
fi

# With Jetson compute (mapping.sh's default), `mapping.sh --3d --nav` starts its
# own sensor relay and motion bridge (robot-motion-wifi.sh, port 17999). A FULL
# relay started here would hold 17999 first, the motion bridge would die with
# "Address already in use", and mapping.sh would tear the whole session down.
# The pre-flight clean has already freed the port, so leave the robot side to it.
if [ "${SIM}" != "true" ] && [ "${START_RELAY}" = "true" ] && \
   [ "${GO2_LIDAR3D_COMPUTE:-jetson}" = "jetson" ]; then
    echo "[relay] mapping.sh starts the robot relay + motion bridge itself (Jetson compute); not starting a FULL relay"
    RELAY_BY_MAPPING="true"
fi

if [ "${MODE}" != "--plan" ] && [ "${START_RELAY}" = "true" ] && [ "${RELAY_BY_MAPPING:-false}" != "true" ]; then
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

cd "${NAV2_WIFI_DIR}"
export GO2_HOST_IP GO2_ROBOT_IP GO2_RELAY_DOMAIN_ID

EXPLORE_HOST_PID=""
ARM_PID=""

CLEANED_UP="false"
cleanup() {
    [ "${CLEANED_UP}" = "true" ] && return 0
    CLEANED_UP="true"
    echo
    echo "[wifi_nav] Shutting down..."
    if [ -n "${ARM_PID}" ] && kill -0 "${ARM_PID}" 2>/dev/null; then
        echo "[wifi_nav] Waiting up to 20 s for the arm fold to finish..."
        for _ in $(seq 1 20); do
            kill -0 "${ARM_PID}" 2>/dev/null || break
            sleep 1
        done
    fi
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -x "${CONTAINER_NAME}" >/dev/null; then
        echo "[wifi_nav] SAFETY: publishing stop to /explore/resume"
        stop_exploration
    fi
    if [ -n "${EXPLORE_HOST_PID}" ]; then
        kill "${EXPLORE_HOST_PID}" 2>/dev/null || true
    fi
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -x "${CONTAINER_NAME}" >/dev/null; then
        kill_explore_inside_container
        kill_floor_filter_inside_container
        kill_camera_relay_inside_container
        kill_recovery_inside_container
    fi
    (cd "${NAV2_WIFI_DIR}/docker" && docker compose down 2>/dev/null) || true
    stop_arm_bridge
    stop_robot_relay
    if [ "${OMNI}" = "true" ]; then
        python3 "${SCRIPT_DIR}/../tools/nav2_omni.py" --disable "${NAV2_WIFI_DIR}" || true
    fi
    # Leave go2-nav2-wifi's planner config without the camera source, so a
    # later mapping.sh run never waits on camera obstacles that never come.
    if [ "${CAMERA_OBSTACLES:-0}" = 1 ]; then
        python3 "${SCRIPT_DIR}/../tools/nav2_camera_source.py" --disable \
            "${NAV2_WIFI_DIR}/go2_nav2/config/lidar3d_planner.yaml" || true
    fi
    if [ "${SIM}" = "true" ]; then
        docker exec "${CONTAINER_NAME}" bash /ws/scripts/go2-session.sh stop lidar3d-viz \
            --owner "${SIM_OWNER}" >/dev/null 2>&1 || true
        if [ "${SIM_STARTED}" = "true" ] && [ "${SIM_KEEP:-0}" != 1 ]; then
            bash "${SIM_RUN}" down || true
        elif [ "${SIM_STARTED}" = "true" ]; then
            echo "[wifi_nav] SIM_KEEP=1: simulator left running (./gazebo/run_sim.sh down)"
        fi
    fi
    echo "[wifi_nav] Done."
}
trap cleanup EXIT INT TERM

ensure_ros_log_dir   # again: the pre-flight root execs may have created it
if [ "${SIM}" = "true" ]; then
    echo "Launching: laptop session (go2-session.sh lidar3d-viz) against the simulator"
else
    echo "Launching: ./mapping.sh --3d ${MODE}"
fi
echo

if [ "${EXPLORE}" = "true" ] || [ "${FLOOR_FILTER}" = "true" ] || [ "${CAMERA}" = "true" ] || [ "${RECOVERY}" = "true" ]; then
    (
        sleep 20

        if [ "${RECOVERY}" = "true" ]; then
            kill_recovery_inside_container
            docker cp "${RECOVERY_SCRIPT}" "${CONTAINER_NAME}:/tmp/go2_stuck_recovery.py" 2>/dev/null || true
            # Sidesteps need vy: the omni smoother allows it; with hybrid the
            # escape bypasses the forward-only smoother (which clamps vy to 0)
            # and waits until it has stopped publishing.
            recovery_args="-p allow_sidestep:=${SIDESTEP}"
            if [ "${DRIVE}" = "hybrid" ]; then
                recovery_args+=" -p cmd_topic:=/cmd_vel -p escape_delay_s:=1.5"
            fi
            echo "[recovery] Starting stuck recovery (drive ${DRIVE}, sidestep: ${SIDESTEP})..."
            docker exec -u root "${DDS_EXEC_ENV[@]}" "${CONTAINER_NAME}" bash -c \
                "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash 2>/dev/null; nohup python3 /tmp/go2_stuck_recovery.py --ros-args ${recovery_args} > /tmp/stuck_recovery.log 2>&1 &" 2>/dev/null || true
        fi

        if [ "${FLOOR_FILTER}" = "true" ]; then
            docker exec -u root "${CONTAINER_NAME}" bash -c '
                pids=$(ps -eo pid,cmd | grep -E "go2_floor_filter" \
                       | grep -v grep | awk "{print \$1}")
                if [ -n "$pids" ]; then
                    for pid in $pids; do kill -9 "$pid" 2>/dev/null || true; done
                fi
            ' 2>/dev/null || true
            docker cp "${FLOOR_FILTER_SCRIPT}" "${CONTAINER_NAME}:/tmp/go2_floor_filter.py" 2>/dev/null || true
            docker exec -u root "${DDS_EXEC_ENV[@]}" "${CONTAINER_NAME}" bash -c \
                "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash 2>/dev/null; nohup python3 /tmp/go2_floor_filter.py --ros-args -p input_topic:=/lidar3d/registered -p output_topic:=/lidar3d/obstacle_points -p floor_margin:=0.20 > /tmp/floor_filter.log 2>&1 &" 2>/dev/null || true
        fi

        if [ "${CAMERA}" = "true" ]; then
            echo "[camera] Killing stale camera relay instances..."
            kill_camera_relay_inside_container
            echo "[camera] Copying go2_camera_relay.py into container..."
            docker cp "${CAMERA_RELAY_SCRIPT}" "${CONTAINER_NAME}:/tmp/go2_camera_relay.py" 2>/dev/null || true
            # Lens calibration (scripts/tools/calibrate_camera.py) -> /camera/camera_info
            camera_info_arg=""
            if [ -f "${CAMERA_CALIB}" ]; then
                docker cp "${CAMERA_CALIB}" "${CONTAINER_NAME}:/tmp/front_camera.yaml" 2>/dev/null \
                    && camera_info_arg="-p camera_info_file:=/tmp/front_camera.yaml"
                echo "[camera] Calibration ${CAMERA_CALIB} -> /camera/camera_info"
            else
                echo "[camera] No ${CAMERA_CALIB}: no /camera/camera_info (run scripts/tools/calibrate_camera.sh)"
            fi
            # Images are stamped front_camera_optical (z forward, x right, y
            # down), the pinhole convention of camera_info, YOLO and RTAB-Map;
            # front_camera is the URDF link (x forward). go2-3d adds the TF.
            # setsid: nohup alone died with the SSH session about a minute later.
            # In 3D mode mapping.sh starts the robot relay without the camera
            # (GO2_RELAY_CAMERA=0), so the robot sends JPEG frames itself:
            # scripts/robot/go2_camera_wifi.sh -> /go2/front_camera/compressed.
            echo "[camera] Starting the robot camera publisher (${GO2_ROBOT_USER}@${GO2_ROBOT_IP}:~/go2-agri-camera)..."
            robot_ssh -T -o ConnectTimeout=5 "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" \
                "pkill -f 'go2_camera_[w]ifi|go2_camera_jpeg_[c]li' 2>/dev/null; mkdir -p ~/go2-agri-camera" 2>/dev/null || true
            if scp -q "${GO2_SSH_OPTS[@]}" "${ROBOT_CAMERA_DIR}/go2_camera_wifi.sh" "${ROBOT_CAMERA_DIR}/go2_camera_wifi.py" \
                    "${NAV2_WIFI_DIR}/ws/scripts/robot-source-unitree-ros.sh" \
                    "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}:go2-agri-camera/" 2>/dev/null; then
                robot_ssh -T -o ConnectTimeout=5 "${GO2_ROBOT_USER}@${GO2_ROBOT_IP}" \
                    "GO2_HOST_IP=${GO2_HOST_IP} GO2_RELAY_DOMAIN_ID=${GO2_RELAY_DOMAIN_ID} GO2_CAM_WIDTH=${CAMERA_WIDTH} GO2_CAM_FPS=${CAMERA_FPS} setsid nohup bash ~/go2-agri-camera/go2_camera_wifi.sh > /tmp/go2_camera_wifi.log 2>&1 < /dev/null &" \
                    2>/dev/null || echo "[camera] WARN: could not start the robot camera publisher"
            else
                echo "[camera] WARN: could not copy the camera publisher to the robot"
            fi
            # The relay joins the session's CycloneDDS config (the robot as a
            # unicast peer). With the container default (FastDDS, multicast
            # discovery only) it found the robot camera rarely or never over Wi-Fi.
            echo "[camera] Starting camera relay..."
            docker exec -u root "${DDS_EXEC_ENV[@]}" "${CONTAINER_NAME}" bash -c \
                "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash 2>/dev/null; source /opt/go2-unitree-msgs/local_setup.bash 2>/dev/null; [ -f /tmp/go2-wifi-cyclonedds.xml ] && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///tmp/go2-wifi-cyclonedds.xml; nohup python3 /tmp/go2_camera_relay.py --ros-args -p source_type:=compressed -p source_topic:=/go2/front_camera/compressed -p output_topic:=/camera/image_raw -p frame_id:=front_camera_optical ${camera_info_arg} > /tmp/camera_relay.log 2>&1 &" 2>/dev/null || true
        fi

        if [ "${EXPLORE}" = "true" ]; then
            echo "[explore] Killing stale explorer instances..."
            docker exec -u root "${CONTAINER_NAME}" bash -c '
                pids=$(ps -eo pid,cmd | grep -E "frontier_explorer|explore_lite|ros2 run explore" | grep -v grep | awk "{print \$1}")
                if [ -n "$pids" ]; then
                    for pid in $pids; do kill -9 "$pid" 2>/dev/null || true; done
                fi
                sleep 1
            ' 2>/dev/null || true
            echo "[explore] Copying frontier_explorer.py + $(basename "${EXPLORER_PARAMS}") into container..."
            docker cp "${EXPLORER_SCRIPT}" "${CONTAINER_NAME}:/tmp/frontier_explorer.py" 2>/dev/null || true
            docker cp "${EXPLORER_PARAMS}" "${CONTAINER_NAME}:/tmp/frontier_explorer.yaml" 2>/dev/null || true
            # With stuck recovery running, an aborted leg gets a step back and
            # a retry; the explorer waits for it before dropping the frontier.
            if [ "${RECOVERY}" = "true" ] && [[ "${EXPLORE_OVERRIDES}" != *recovery_wait_s* ]]; then
                EXPLORE_OVERRIDES+=" -p recovery_wait_s:=8.0"
            fi
            [ -n "${EXPLORE_OVERRIDES}" ] && echo "[explore] overrides:${EXPLORE_OVERRIDES}"
            echo "[explore] Starting custom frontier explorer..."
            docker exec -u root "${DDS_EXEC_ENV[@]}" "${CONTAINER_NAME}" bash -c \
                "source /opt/ros/humble/setup.bash && \
                 source /ws/install/setup.bash 2>/dev/null; \
                 exec python3 /tmp/frontier_explorer.py \
                     --ros-args \
                     --params-file /tmp/frontier_explorer.yaml${EXPLORE_OVERRIDES}" \
                > "${NAV2_WIFI_DIR}/ws/log/frontier.log" 2>&1 &
        fi
    ) &
    EXPLORE_HOST_PID=$!
    echo "[wifi_nav] background helpers will start after ~20 s (host PID ${EXPLORE_HOST_PID})"
    echo
fi

if [ "${ARM}" = "true" ] && [ "${ARM_LINK}" = "wifi" ] && [ "${SIM}" = "true" ]; then
    echo "[arm] WARN: --arm-wifi needs the real robot; the simulator uses --arm"
    ARM="false"
elif [ "${ARM}" = "true" ] && [ "${ARM_LINK}" = "wifi" ]; then
    mkdir -p "$(dirname "${ARM_LOG}")"
    echo "[arm] Arm over Wi-Fi: bridge on the robot, then fold in the background (log: ${ARM_LOG})"
    (
        if start_arm_bridge && env ARM_ROBOT="${GO2_ROBOT_IP}" ARM_BRIDGE_PORT="${ARM_BRIDGE_PORT}" \
                bash "${ARM_START_SCRIPT}" fold > "${ARM_LOG}" 2>&1; then
            echo "[arm] Arm folded and released (over Wi-Fi)."
        else
            echo "[arm] WARNING: arm over Wi-Fi failed — navigation continues. Last lines of ${ARM_LOG}:"
            tail -5 "${ARM_LOG}" 2>/dev/null | sed 's/^/    /'
        fi
    ) &
    ARM_PID=$!
elif [ "${ARM}" = "true" ]; then
    mkdir -p "$(dirname "${ARM_LOG}")"
    echo "[arm] Folding the D1 arm in the background (log: ${ARM_LOG})"
    (
        sim_arm_env=()
        if [ "${SIM}" = "true" ]; then
            sim_arm_env=(ARM_IP="${SIM_ARM_SUBNET:-192.168.123}.100" ARM_HOST_IP="${SIM_ARM_SUBNET:-192.168.123}.51")
        fi
        if env ARM_IFACE="${ARM_IFACE}" "${sim_arm_env[@]}" bash "${ARM_START_SCRIPT}" fold > "${ARM_LOG}" 2>&1; then
            echo "[arm] Arm folded and released."
        else
            echo "[arm] WARNING: arm start failed — navigation continues. Last lines of ${ARM_LOG}:"
            tail -5 "${ARM_LOG}" 2>/dev/null | sed 's/^/    /'
        fi
    ) &
    ARM_PID=$!
fi

# mapping.sh's Ctrl+C cleanup waits on the robot over one shared SSH link; give
# that link keep-alives and every cleanup step a time limit (idempotent).
if [ "${SIM}" != "true" ]; then
    python3 "${SCRIPT_DIR}/../tools/mapping_stop_timeouts.py" "${NAV2_WIFI_DIR}" || true
fi

# Full mobility (--omni): MPPI Omni + omni lattice, else the forward-only default.
if [ "${OMNI}" = "true" ] && ! ensure_mppi; then
    echo "[omni] WARN: MPPI install failed; continuing with --drive hybrid"
    OMNI="false"; DRIVE="hybrid"
fi
if [ "${OMNI}" = "true" ]; then omni_arg=--enable; else omni_arg=--disable; fi
python3 "${SCRIPT_DIR}/../tools/nav2_omni.py" "${omni_arg}" "${NAV2_WIFI_DIR}" || true

# Camera obstacles in the Nav2 costmaps only for wifi_3d_nav.sh --camera-obstacles:
# the source makes Nav2 stop when camera data stops, so every other session removes it.
if [ "${CAMERA_OBSTACLES:-0}" = 1 ]; then cam_src=--enable; else cam_src=--disable; fi
python3 "${SCRIPT_DIR}/../tools/nav2_camera_source.py" "${cam_src}" \
    "${NAV2_WIFI_DIR}/go2_nav2/config/lidar3d_planner.yaml" || true

# Live camera, LiDAR-on-camera overlay and the RTAB-Map 3D map in RViz (once per file).
python3 "${SCRIPT_DIR}/../tools/rviz_add_camera.py" \
    "${NAV2_WIFI_DIR}/go2_nav2/rviz/lidar3d_planning.rviz" \
    "${NAV2_WIFI_DIR}/go2_nav2/rviz/lidar3d_navigation.rviz" || true

if [ "${SIM}" = "true" ]; then
    : "${DISPLAY:?RViz needs a desktop session: run this in a terminal on the desktop of this computer}"
    sim_nav=0; sim_plan=1
    [ "${MODE}" = "--nav" ] && sim_nav=1
    if [ "${sim_nav}" = 1 ]; then
        docker exec "${CONTAINER_NAME}" test -f /ws/install/go2_rviz_controls/lib/libgo2_rviz_controls.so || docker exec "${CONTAINER_NAME}" bash /ws/scripts/build-rviz-controls.sh
    fi
    xuser="$(getent passwd "$(docker exec "${CONTAINER_NAME}" id -u)" | cut -d: -f1)"
    [ -n "${xuser}" ] && xhost "+SI:localuser:${xuser}" >/dev/null 2>&1 || true
    docker exec -e GO2_HOST_IP="${GO2_HOST_IP}" -e GO2_ROBOT_IP="${GO2_ROBOT_IP}" -e GO2_RELAY_DOMAIN_ID="${GO2_RELAY_DOMAIN_ID}" -e GO2_LIDAR3D_NAV="${sim_nav}" -e GO2_LIDAR3D_PLAN="${sim_plan}" -e GO2_LIDAR3D_BACKEND=pointlio -e DISPLAY="${DISPLAY}" -e QT_QPA_PLATFORM=xcb -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 -e XDG_CONFIG_HOME=/tmp/go2-mapping-config "${CONTAINER_NAME}" bash /ws/scripts/go2-session.sh start lidar3d-viz --owner "${SIM_OWNER}" || true
else
    # X11 FIX: Explicitly grant Docker access to the display
    echo "[rviz] Granting X11 access to Docker..."
    xhost +local:root >/dev/null 2>&1 || true
    xhost +local:docker >/dev/null 2>&1 || true

    # Pass the DISPLAY variable explicitly
    if [ -z "${DISPLAY}" ]; then
        export DISPLAY=:0
    fi

    ./mapping.sh --3d ${MODE} || true
fi

echo
echo "================================================================"
echo " Last 20 lines of the newest slam log:"
echo "================================================================"
LATEST_LOG=""
for d in "${NAV2_WIFI_DIR}"/ws/log/mapping*/; do
    [ -d "$d" ] || continue
    if [ -z "${LATEST_LOG}" ] || [ "$d" -nt "${NAV2_WIFI_DIR}/ws/log/${LATEST_LOG}" ]; then
        LATEST_LOG=$(basename "$d")
    fi
done
if [ -n "${LATEST_LOG}" ] && [ -f "${NAV2_WIFI_DIR}/ws/log/${LATEST_LOG}/slam.log" ]; then
    tail -20 "${NAV2_WIFI_DIR}/ws/log/${LATEST_LOG}/slam.log"
else
    echo "  (no slam.log found)"
fi
echo

if [ "${EXPLORE}" = "true" ] && [ -f "${NAV2_WIFI_DIR}/ws/log/frontier.log" ]; then
    echo "================================================================"
    echo " Last 20 lines of frontier.log:"
    echo "================================================================"
    tail -20 "${NAV2_WIFI_DIR}/ws/log/frontier.log"
    echo
fi

if [ "${CAMERA}" = "true" ] && docker ps --format '{{.Names}}' 2>/dev/null | grep -x "${CONTAINER_NAME}" >/dev/null; then
    echo "================================================================"
    echo " Last 10 lines of camera_relay.log:"
    echo "================================================================"
    docker exec -u root "${CONTAINER_NAME}" tail -10 /tmp/camera_relay.log 2>/dev/null || echo "  (no log)"
    echo
fi

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
