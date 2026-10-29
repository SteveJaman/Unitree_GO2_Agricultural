#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# run_sim.sh: start/stop the Go2 + D1 Gazebo simulation (on the host).
#
#   ./gazebo/run_sim.sh up       start (no-op if running); waits until ready
#   ./gazebo/run_sim.sh down     stop and remove the sim arm interface
#   ./gazebo/run_sim.sh status   container state + last log lines
#   ./gazebo/run_sim.sh logs     follow the simulation log
#   ./gazebo/run_sim.sh shell    shell inside the sim container
#
# What "up" gives you (what the real Jetson + arm would):
#   * DDS domain 64 on GO2_HOST_IP: /lidar3d/odom, /lidar3d/registered,
#     /lidar3d/path, /joint_states, /camera/image_raw
#   * TCP 17999: the cmd_vel server go2_cmd_vel_tcp.py clients connect to
#   * D1 arm: DDS domain 0 on interface simarm0 (192.168.123.51, arm "at" .100)
#
# Environment:
#   GO2_HOST_IP=...       DDS bind address (default: auto on the Wi-Fi interface, else 127.0.0.1)
#   SIM_GUI=0             1 = Gazebo window. Off by default: on the old 4-core VM
#                         the window ran the sim at ~14% real time, so the
#                         robot crawled while Nav2/explorer timed out
#                         (they run on wall-clock time). RViz shows the same.
#   SIM_ARM=1             create simarm0 + run the D1 emulator (0 = no arm)
#   SIM_ARM_IFACE=simarm0 SIM_ARM_SUBNET=192.168.123
#   SIM_RENDER_ENGINE=ogre2  (try ogre if the window stays black)
#   SIM_CAMERA=1          0 = no front camera (0.51x -> 0.85x real time on the old 4-core VM)
#   SIM_LIDAR_HZ=5        lidar rate; 10 Hz halved the sim speed on the old 4-core VM
#   SIM_WORLD=agri_rooms  field + building with rooms (agri_field = field only)
#   SIM_SPAWN="-4.5 0 0"  SIM_D1_MOUNT="0 0 0.057 0 0 0"  SIM_TIMEOUT=240
#   SIM_SENSOR_FROM_BASE="x y z qx qy qz qw"  (default: read from go2-nav2-wifi lidar3d_robot_viz.yaml)
#   SIM_DRIFT_YAW=0.02 SIM_DRIFT_SCALE=0.03  odometry drift for testing (default 0 = ground truth)
#   SIM_SENSOR_YAML=$NAV2_WIFI_DIR/go2_nav2/config/lidar3d_robot_viz.yaml
#                         lidar mount (sensor_from_base) the laptop stack uses;
#                         the sim places its lidar to match
# ---------------------------------------------------------------------------
set -eo pipefail

GZ_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${GZ_DIR}/.." && pwd)"
NAME="${SIM_CONTAINER:-go2-sim}"
IMAGE="${SIM_IMAGE:-go2-sim:local}"
SIM_GUI="${SIM_GUI:-0}"
SIM_ARM="${SIM_ARM:-1}"
SIM_ARM_IFACE="${SIM_ARM_IFACE:-simarm0}"
SIM_ARM_SUBNET="${SIM_ARM_SUBNET:-192.168.123}"
SIM_TIMEOUT="${SIM_TIMEOUT:-240}"
IFACE_MARKER="/tmp/go2-sim-created-${SIM_ARM_IFACE}"
NAV2_WIFI_DIR="${NAV2_WIFI_DIR:-$HOME/files/go2-nav2-wifi}"
SIM_SENSOR_YAML="${SIM_SENSOR_YAML:-${NAV2_WIFI_DIR}/go2_nav2/config/lidar3d_robot_viz.yaml}"

die() { echo "ERROR: $*" >&2; exit 1; }

resolve_host_ip() {
    if [ -n "${GO2_HOST_IP:-}" ]; then return 0; fi
    # shellcheck source=../scripts/runtime/go2_net.sh
    source "${REPO_DIR}/scripts/runtime/go2_net.sh"
    if go2_detect_host_ip 2>/dev/null; then
        echo "[sim] host IP ${GO2_HOST_IP} (${GO2_IFACE})"
    else
        GO2_HOST_IP=127.0.0.1
        echo "[sim] no Wi-Fi address; using ${GO2_HOST_IP} (sim only needs this machine)"
    fi
}

# The laptop's go2_odom_tf turns /lidar3d/odom into base_link with
# sensor_from_base from go2-nav2-wifi's lidar3d_robot_viz.yaml (bind-mounted
# into go2-lidar3d). The sim must place its virtual sensor with the same
# transform, or RViz/Nav2 see a shifted, tilted robot. Read it from that file.
resolve_sensor_from_base() {
    if [ -n "${SIM_SENSOR_FROM_BASE:-}" ]; then
        echo "[sim] lidar mount from SIM_SENSOR_FROM_BASE"
        return 0
    fi
    local yaml="${NAV2_WIFI_DIR:-$HOME/files/go2-nav2-wifi}/go2_nav2/config/lidar3d_robot_viz.yaml"
    if [ -f "$yaml" ]; then
        # First uncommented "sensor_from_base: [..]" line.
        SIM_SENSOR_FROM_BASE="$(sed -n 's/^[[:space:]]*sensor_from_base:[[:space:]]*\[\(.*\)\].*/\1/p' "$yaml" \
            | head -1 | tr ',' ' ' | tr -s ' ')"
    fi
    if [ -n "${SIM_SENSOR_FROM_BASE:-}" ]; then
        echo "[sim] lidar mount from ${yaml}: ${SIM_SENSOR_FROM_BASE}"
    else
        echo "[sim] WARN: ${yaml} not found; using build_model.py's built-in lidar mount." >&2
        echo "[sim]       If the laptop uses a different mount, the robot pose in RViz will be wrong." >&2
    fi
}

sim_running() { docker ps --format '{{.Names}}' | grep -x "$NAME" >/dev/null; }
sim_exists()  { docker ps -a --format '{{.Names}}' | grep -x "$NAME" >/dev/null; }

sudo_cmd() { if [ "$(id -u)" -eq 0 ]; then "$@"; else sudo "$@"; fi; }

setup_arm_iface() {
    [ "$SIM_ARM" = 1 ] || return 0
    if ip link show dev "$SIM_ARM_IFACE" >/dev/null 2>&1; then
        echo "[sim] ${SIM_ARM_IFACE} present"
        return 0
    fi
    # Another interface on the same subnet (e.g. the real arm on the wired port) would
    # make routing to the arm ambiguous.
    local other
    other="$(ip -4 -o addr show | awk -v p="^${SIM_ARM_SUBNET//./\\.}\\." '$4 ~ p {print $2; exit}')"
    if [ -n "$other" ]; then
        die "${other} already uses ${SIM_ARM_SUBNET}.x (real arm?). Unplug it, or use SIM_ARM_SUBNET=192.168.124, or SIM_ARM=0."
    fi
    echo "[sim] creating ${SIM_ARM_IFACE} (sim arm network, needs sudo)..."
    sudo_cmd ip link add "$SIM_ARM_IFACE" type dummy
    sudo_cmd ip link set "$SIM_ARM_IFACE" multicast on
    sudo_cmd ip addr add "${SIM_ARM_SUBNET}.51/24" dev "$SIM_ARM_IFACE"
    sudo_cmd ip addr add "${SIM_ARM_SUBNET}.100/32" dev "$SIM_ARM_IFACE"
    sudo_cmd ip link set "$SIM_ARM_IFACE" up
    touch "$IFACE_MARKER"
}

remove_arm_iface() {
    if [ -f "$IFACE_MARKER" ] && ip link show dev "$SIM_ARM_IFACE" >/dev/null 2>&1; then
        echo "[sim] removing ${SIM_ARM_IFACE}..."
        sudo_cmd ip link del "$SIM_ARM_IFACE" || true
    fi
    rm -f "$IFACE_MARKER"
}

# Sim seconds per wall second, measured over 5 s. The laptop stack runs on
# wall-clock time, so below ~0.8 the robot moves slower than Nav2 commands
# and goals time out.
sim_seconds() {
    docker exec "$NAME" bash -c 'GZ_PARTITION=go2sim GZ_IP=127.0.0.1 timeout 3 gz topic -e -n 1 -t /stats' 2>/dev/null \
        | awk '/^sim_time/{f=1} f&&/ sec:/{s=$2} f&&/nsec:/{print s+$2/1e9; exit}'
}

report_rtf() {
    local a b w0 w1 rtf
    a="$(sim_seconds)"; w0="$(date +%s.%N)"
    sleep 5
    b="$(sim_seconds)"; w1="$(date +%s.%N)"
    if [ -z "$a" ] || [ -z "$b" ]; then
        echo "[sim] (could not measure the real-time factor)"
        return 0
    fi
    rtf="$(awk -v a="$a" -v b="$b" -v w0="$w0" -v w1="$w1" 'BEGIN{printf "%.2f", (b-a)/(w1-w0)}')"
    if awk -v r="$rtf" 'BEGIN{exit !(r < 0.8)}'; then
        echo "[sim] WARN: sim runs at ${rtf}x real time; the robot will crawl and goals time out."
        echo "[sim]       Try SIM_CAMERA=0 (wifi_nav --no-camera does this), keep SIM_GUI=0 and"
        echo "[sim]       SIM_LIDAR_HZ=5, or close other heavy apps."
    else
        echo "[sim] real-time factor ${rtf}"
    fi
}

cmd_up() {
    docker image inspect "$IMAGE" >/dev/null 2>&1 || die "image ${IMAGE} missing: run ./gazebo/setup_gazebo.sh"
    [ -d "$GZ_DIR/models/go2_description" ] && [ -d "$GZ_DIR/models/d1_description" ] \
        || die "models missing: run ./gazebo/setup_gazebo.sh"
    if sim_running; then
        echo "[sim] ${NAME} already running"
        return 0
    fi
    sim_exists && docker rm -f "$NAME" >/dev/null
    resolve_host_ip
    resolve_sensor_from_base
    setup_arm_iface

    local gui="$SIM_GUI"
    if [ "$gui" = 1 ] && [ -z "${DISPLAY:-}" ]; then
        echo "[sim] no DISPLAY: running headless"
        gui=0
    fi
    if [ "$gui" = 1 ] && command -v xhost >/dev/null 2>&1; then
        xhost "+SI:localuser:$(id -un)" >/dev/null 2>&1 || true
    fi
    local dev_args=()
    if [ -d /dev/dri ]; then dev_args=(--device /dev/dri --group-add video); fi
    local sensor_args=()
    if [ -f "$SIM_SENSOR_YAML" ]; then
        sensor_args=(-v "$SIM_SENSOR_YAML:/nav2_cfg/lidar3d_robot_viz.yaml:ro"
                     -e SIM_SENSOR_YAML=/nav2_cfg/lidar3d_robot_viz.yaml)
    else
        echo "[sim] WARN: ${SIM_SENSOR_YAML} not found; sim uses the Go2 URDF lidar mount"
    fi

    echo "[sim] starting ${NAME} (GUI=${gui}, arm=${SIM_ARM}, DDS ${GO2_HOST_IP} domain ${GO2_RELAY_DOMAIN_ID:-64})"
    docker run -d --name "$NAME" --network host --ipc host \
        --user "$(id -u):$(id -g)" "${dev_args[@]}" \
        -e GO2_HOST_IP="$GO2_HOST_IP" -e GO2_ROBOT_IP="${GO2_ROBOT_IP:-$GO2_HOST_IP}" \
        -e GO2_RELAY_DOMAIN_ID="${GO2_RELAY_DOMAIN_ID:-64}" \
        -e SIM_GUI="$gui" -e SIM_ARM="$SIM_ARM" -e SIM_ARM_IFACE="$SIM_ARM_IFACE" \
        -e SIM_RENDER_ENGINE="${SIM_RENDER_ENGINE:-ogre2}" \
        -e SIM_CAMERA="${SIM_CAMERA:-1}" -e SIM_LIDAR_HZ="${SIM_LIDAR_HZ:-5}" -e SIM_WORLD="${SIM_WORLD:-agri_rooms}" -e SIM_SPAWN="${SIM_SPAWN:--4.5 0 0}" -e SIM_D1_MOUNT="${SIM_D1_MOUNT:-0.0 0 0.057 0 0 0}" \
        -e SIM_SENSOR_FROM_BASE="${SIM_SENSOR_FROM_BASE:-}" \
        -e SIM_DRIFT_YAW="${SIM_DRIFT_YAW:-0}" -e SIM_DRIFT_SCALE="${SIM_DRIFT_SCALE:-0}" \
        -e DISPLAY="${DISPLAY:-}" -e QT_X11_NO_MITSHM=1 -e HOME=/tmp/simhome \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "$GZ_DIR:/sim:ro" "${sensor_args[@]}" \
        "$IMAGE" >/dev/null

    echo "[sim] waiting for the first lidar frame (up to ${SIM_TIMEOUT} s; software rendering is slow)..."
    local t=0
    while [ "$t" -lt "$SIM_TIMEOUT" ]; do
        if ! sim_running; then
            echo "[sim] ERROR: simulation exited. Last log lines:" >&2
            docker logs --tail 40 "$NAME" 2>&1 | sed 's/^/    /' >&2
            exit 1
        fi
        if docker logs "$NAME" 2>&1 | grep -q "SIM READY"; then
            echo "[sim] READY: robot publishing on domain ${GO2_RELAY_DOMAIN_ID:-64}, cmd_vel TCP on :17999"
            if [ "$SIM_ARM" = 1 ]; then
                if docker logs "$NAME" 2>&1 | grep -q "D1 ARM EMULATOR READY"; then
                    echo "[sim] READY: D1 arm on ${SIM_ARM_IFACE} (use ARM_IFACE=${SIM_ARM_IFACE})"
                else
                    echo "[sim] WARN: D1 emulator not ready yet; see ./gazebo/run_sim.sh logs"
                fi
            fi
            report_rtf
            return 0
        fi
        sleep 2; t=$((t + 2))
    done
    echo "[sim] ERROR: no lidar frame after ${SIM_TIMEOUT} s. Last log lines:" >&2
    docker logs --tail 40 "$NAME" 2>&1 | sed 's/^/    /' >&2
    echo "  Try SIM_RENDER_ENGINE=ogre, or SIM_GUI=0, or a longer SIM_TIMEOUT." >&2
    exit 1
}

cmd_down() {
    if sim_exists; then
        echo "[sim] stopping ${NAME}..."
        docker rm -f "$NAME" >/dev/null
    fi
    remove_arm_iface
    echo "[sim] down"
}

case "${1:-}" in
    up)     cmd_up ;;
    down)   cmd_down ;;
    status) if sim_running; then echo "[sim] ${NAME} running"; docker logs --tail 15 "$NAME" 2>&1; else echo "[sim] not running"; fi ;;
    logs)   docker logs -f "$NAME" ;;
    shell)  docker exec -it "$NAME" bash ;;
    -h|--help|"") awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0" ;;
    *) die "unknown command: $1 (up|down|status|logs|shell)" ;;
esac
