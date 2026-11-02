#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# sim_trial.sh: one unattended exploration trial in the Gazebo sim
# (SIM_WORLD, default agri_rooms), recorded with record.sh for analyze.py.
#
#   sim_trial.sh NAME --drive diff|hybrid|omni [other wifi_nav.sh options]
#   e.g. for i in 1 2 3; do for d in diff hybrid omni; do sim_trial.sh $d$i --drive $d; done; done
#   SIM_WORLD=lab_office LIMIT=900 sim_trial.sh lab_hybrid1 --drive hybrid
#
# Starts wifi_nav.sh --sim --host-ip 127.0.0.1 --no-camera, waits for the
# explorer, presses Auto Explore, stops at "Exploration complete" or after
# LIMIT seconds (default 600). Then, if gazebo/worlds/$SIM_WORLD.json has a
# "tour" (lab_office does; TOUR=0 skips it), drives it with nav_tour.py: one
# goal per room on the map just built. Saves the trial and shuts everything down.
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
WORLD="${SIM_WORLD:-agri_rooms}"
WORLD_JSON="${REPO}/gazebo/worlds/${WORLD}.json"
TOUR="${TOUR:-1}"
TOUR_LIMIT="${TOUR_LIMIT:-1500}"

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
# A background job of a script starts with SIGINT ignored, and bash cannot
# undo that: the Ctrl+C below would never reach wifi_nav.sh's cleanup. So
# restore SIGINT and start a new session (process group) before exec.
python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.setsid(); os.execvp(sys.argv[1], sys.argv[1:])'     ./scripts/runtime/wifi_nav.sh --sim --host-ip 127.0.0.1 --no-camera "$@" > "${SESSION_LOG}" 2>&1 < /dev/null &
PID=$!
for _ in $(seq 1 300); do
    grep -q "frontier_explorer ready" "${FLOG}" 2>/dev/null && break
    kill -0 "${PID}" 2>/dev/null || { echo "[trial] session exited early; see ${SESSION_LOG}"; exit 1; }
    sleep 2
done
sleep 30   # costmap settles
"${HERE}/record.sh" start "${NAME}" --drive "${DRIVE}" --note "sim ${WORLD} $(grep -oE 'real-time factor [0-9.]+' "${SESSION_LOG}" | tail -1)"
resume
START=$(date +%s)
until grep -q "Exploration complete" "${FLOG}" 2>/dev/null || [ $(( $(date +%s) - START )) -ge "${LIMIT}" ]; do
    grep -q "Exploration ENABLED" "${FLOG}" 2>/dev/null || resume
    sleep 5
done
echo "[trial] ${NAME}: exploration ended after $(( $(date +%s) - START )) s"
echo "EXPLORE_END=$(date +%s)" >> "${OUT_ROOT}/${NAME}/meta.env"
"${HERE}/record.sh" snapshot "${NAME}" || true

if [ "${TOUR}" = 1 ] && [ -f "${WORLD_JSON}" ] &&
   goals="$(python3 -c 'import json, sys; t = json.load(open(sys.argv[1])).get("tour"); print(json.dumps(t)) if t else sys.exit(1)' "${WORLD_JSON}")"; then
    echo "[trial] ${NAME}: navigation tour ($(python3 -c 'import json, sys; print(len(json.loads(sys.argv[1])))' "${goals}") goals)"
    docker cp "${HERE}/nav_tour.py" go2-lidar3d:/tmp/nav_compare_tour.py
    docker exec go2-lidar3d bash -c "rm -rf /tmp/nav_tour_${NAME}"
    timeout "${TOUR_LIMIT}" docker exec -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///tmp/go2-wifi-cyclonedds.xml         go2-lidar3d bash -c "source /opt/ros/humble/setup.bash; python3 -u /tmp/nav_compare_tour.py --out /tmp/nav_tour_${NAME} --goals '${goals}'"         > "${OUT_ROOT}/${NAME}/tour.log" 2>&1
    docker cp "go2-lidar3d:/tmp/nav_tour_${NAME}/tour.csv" "${OUT_ROOT}/${NAME}/tour.csv" 2>/dev/null         || echo "[trial] WARN: no tour.csv (see ${OUT_ROOT}/${NAME}/tour.log)"
    grep -o "tour [0-9]*/[0-9]* .*" "${OUT_ROOT}/${NAME}/tour.log" | sed 's/^/[trial]   /'
fi
"${HERE}/record.sh" stop "${NAME}"

PG=$(ps -o pgid= -p "${PID}" | tr -d ' ')
kill -INT -- "-${PG:-${PID}}" 2>/dev/null
for _ in $(seq 1 60); do kill -0 "${PID}" 2>/dev/null || break; sleep 2; done
kill -9 -- "-${PG:-${PID}}" 2>/dev/null
"${REPO}/gazebo/run_sim.sh" down >/dev/null 2>&1
echo "[trial] ${NAME}: session stopped"
