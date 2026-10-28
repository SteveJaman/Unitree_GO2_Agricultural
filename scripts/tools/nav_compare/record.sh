#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# record.sh: record one navigation trial of a running wifi_nav.sh /
# wifi_3d_nav.sh session (Gazebo sim or the real dog) for analyze.py.
#
#   record.sh start NAME --drive diff|hybrid|omni [--note "text"]
#   ... let the dog explore (dashboard / RViz Auto Explore) ...
#   record.sh stop NAME
#
# start: copies pose_logger.py into go2-lidar3d and logs the robot pose, every
#        /cmd_vel and (on stop) the final costmap.
# stop:  stops the logger and collects into $NAV_COMPARE_DIR/NAME/:
#        pose.csv cmd.csv costmap.npz frontier.log recovery.log meta.env
#
# NAV_COMPARE_DIR defaults to ~/files/go2-nav2-wifi/ws/log/nav_compare.
# sim_trial.sh wraps this for unattended Gazebo runs. See
# docs/navigation/locomotion_comparison.md for the test protocol.
# ---------------------------------------------------------------------------
set -eo pipefail

HERE="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
NAV2_WIFI_DIR="${NAV2_WIFI_DIR:-$HOME/files/go2-nav2-wifi}"
OUT_ROOT="${NAV_COMPARE_DIR:-${NAV2_WIFI_DIR}/ws/log/nav_compare}"
CONTAINER="${CONTAINER_NAME:-go2-lidar3d}"
DDS=(-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///tmp/go2-wifi-cyclonedds.xml)

die() { echo "record.sh: $*" >&2; exit 1; }

ACTION="${1:-}"; NAME="${2:-}"
[ -n "${ACTION}" ] && [ -n "${NAME}" ] || die "usage: record.sh start|stop NAME [--drive diff|hybrid|omni] [--note TEXT]"
shift 2
DRIVE=""; NOTE=""
while [ $# -gt 0 ]; do
    case "$1" in
        --drive) DRIVE="${2:-}"; shift ;;
        --note)  NOTE="${2:-}"; shift ;;
        *) die "unknown option $1" ;;
    esac
    shift
done
OUT="${OUT_ROOT}/${NAME}"
CTR_OUT="/tmp/nav_trial_${NAME}"

case "${ACTION}" in
start)
    case "${DRIVE}" in diff|hybrid|omni) ;; *) die "--drive diff|hybrid|omni is required" ;; esac
    docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}" || die "${CONTAINER} is not running: start the session first"
    mkdir -p "${OUT}"
    {
        echo "NAME=${NAME}"
        echo "DRIVE=${DRIVE}"
        echo "NOTE=\"${NOTE}\""
        echo "START=$(date +%s)"
    } > "${OUT}/meta.env"
    docker cp "${HERE}/pose_logger.py" "${CONTAINER}:/tmp/nav_compare_pose_logger.py"
    docker exec -u root "${DDS[@]}" "${CONTAINER}" bash -c \
        "source /opt/ros/humble/setup.bash; rm -rf ${CTR_OUT}; nohup python3 /tmp/nav_compare_pose_logger.py --out ${CTR_OUT} > /tmp/nav_compare_${NAME}.log 2>&1 & echo \$! > ${CTR_OUT}.pid"
    echo "[record] ${NAME} (${DRIVE}) logging; stop with: $0 stop ${NAME}"
    ;;
stop)
    [ -f "${OUT}/meta.env" ] || die "no trial ${NAME} in ${OUT_ROOT} (start it first)"
    # By PID: a pkill -f pattern would also match (and kill) this bash -c.
    docker exec -u root "${CONTAINER}" bash -c \
        "p=\$(cat ${CTR_OUT}.pid 2>/dev/null); [ -n \"\$p\" ] || exit 0; kill -INT \$p 2>/dev/null; for i in \$(seq 1 40); do kill -0 \$p 2>/dev/null || break; sleep 0.5; done" || true
    for f in pose.csv cmd.csv costmap.npz; do
        docker cp "${CONTAINER}:${CTR_OUT}/${f}" "${OUT}/${f}" 2>/dev/null || echo "[record] WARN: no ${f}"
    done
    docker exec "${CONTAINER}" cat /tmp/stuck_recovery.log > "${OUT}/recovery.log" 2>/dev/null || true
    cp "${NAV2_WIFI_DIR}/ws/log/frontier.log" "${OUT}/frontier.log" 2>/dev/null || echo "[record] WARN: no frontier.log"
    echo "END=$(date +%s)" >> "${OUT}/meta.env"
    echo "[record] ${NAME} saved to ${OUT}"
    ;;
*) die "unknown action ${ACTION} (start|stop)" ;;
esac
