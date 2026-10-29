#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# sim_trial.sh: one unattended exploration trial in the Gazebo sim
# (agri_rooms), recorded with record.sh for analyze.py.
#
#   sim_trial.sh NAME --drive diff|hybrid|omni [other wifi_nav.sh options]
#   e.g. for i in 1 2 3; do for d in diff hybrid omni; do sim_trial.sh $d$i --drive $d; done; done
#
# Starts wifi_nav.sh --sim --host-ip 127.0.0.1 --no-camera, waits for the
# explorer, presses Auto Explore, stops at "Exploration complete" or after
# LIMIT seconds (default 600), saves the trial and shuts everything down.
# Run it from a terminal on the desktop of this computer (RViz needs $DISPLAY). Nothing else
# heavy should run meanwhile: the result depends on the real-time factor.
# ---------------------------------------------------------------------------
set -o pipefail

HERE="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO="$( cd "${HERE}/../../.." &> /dev/null && pwd )"
NAV2_WIFI_DIR="${NAV2_WIFI_DIR:-$HOME/files/go2-nav2-wifi}"
FLOG="${NAV2_WIFI_DIR}/ws/log/frontier.log"
OUT_ROOT="${NAV_COMPARE_DIR:-${NAV2_WIFI_DIR}/ws/log/nav_compare}"
LIMIT="${LIMIT:-600}"

NAME="${1:?usage: sim_trial.sh NAME --drive diff|hybrid|omni [wifi_nav.sh options]}"; shift
DRIVE=""
for ((i = 1; i <= $#; i++)); do
    [ "${!i}" = "--drive" ] && { j=$((i + 1)); DRIVE="${!j}"; }
done
case "${DRIVE}" in diff|hybrid|omni) ;; *) echo "sim_trial.sh: pass --drive diff|hybrid|omni" >&2; exit 1 ;; esac
mkdir -p "${OUT_ROOT}/${NAME}"
SESSION_LOG="${OUT_ROOT}/${NAME}/session.log"

resume() {
    docker exec go2-lidar3d bash -c "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///tmp/go2-wifi-cyclonedds.xml; source /opt/ros/humble/setup.bash; ros2 topic pub --once -w 1 /explore/resume std_msgs/msg/Bool '{data: true}'" >/dev/null 2>&1
}

cd "${REPO}" || exit 1
rm -f "${FLOG}"
echo "[trial] ${NAME}: starting the sim session (log ${SESSION_LOG})"
setsid ./scripts/runtime/wifi_nav.sh --sim --host-ip 127.0.0.1 --no-camera "$@" > "${SESSION_LOG}" 2>&1 &
PID=$!
for _ in $(seq 1 300); do
    grep -q "frontier_explorer ready" "${FLOG}" 2>/dev/null && break
    kill -0 "${PID}" 2>/dev/null || { echo "[trial] session exited early; see ${SESSION_LOG}"; exit 1; }
    sleep 2
done
sleep 30   # costmap settles
"${HERE}/record.sh" start "${NAME}" --drive "${DRIVE}" --note "sim agri_rooms $(grep -oE 'real-time factor [0-9.]+' "${SESSION_LOG}" | tail -1)"
resume
START=$(date +%s)
until grep -q "Exploration complete" "${FLOG}" 2>/dev/null || [ $(( $(date +%s) - START )) -ge "${LIMIT}" ]; do
    grep -q "Exploration ENABLED" "${FLOG}" 2>/dev/null || resume
    sleep 5
done
echo "[trial] ${NAME}: done after $(( $(date +%s) - START )) s"
"${HERE}/record.sh" stop "${NAME}"

PG=$(ps -o pgid= -p "${PID}" | tr -d ' ')
kill -INT -- "-${PG:-${PID}}" 2>/dev/null
for _ in $(seq 1 60); do kill -0 "${PID}" 2>/dev/null || break; sleep 2; done
kill -9 -- "-${PG:-${PID}}" 2>/dev/null
"${REPO}/gazebo/run_sim.sh" down >/dev/null 2>&1
echo "[trial] ${NAME}: session stopped"
