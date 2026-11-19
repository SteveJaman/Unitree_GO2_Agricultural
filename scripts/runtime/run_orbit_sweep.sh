#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# run_orbit_sweep.sh: the three-shell orbit frontier over standoff d x orbit radius r.
#
#   ./scripts/runtime/run_orbit_sweep.sh                     # frontier + orbits, all cells
#   ./scripts/runtime/run_orbit_sweep.sh --frontier-only     # IK frontier only (minutes)
#   ./scripts/runtime/run_orbit_sweep.sh --repeats 3 --out DIR
#   ./scripts/runtime/run_orbit_sweep.sh --orbit-only --out DIR   # orbits on DIR's frontier
#   --keep-sim: leave the simulation up at the end
#   ./scripts/runtime/run_orbit_sweep.sh --world cropking_nft46_basil --plants "basil_0_1 basil_0_2" \
#       --cell "0.50 0.25" --model ~/.cache/go2-yolo/basil_yolo11s.pt --out DIR   # species orbits
#
# Matrix: d in {0.35, 0.40, 0.45, 0.50} x r in {0.25, 0.30, 0.35, 0.40} (m), shells = all.
# 1. Analytic: R(90) = sqrt(d^2 + r^2) against the D1-550 reach 0.605 m. Cells with
#    R(90) > 0.625 are SKIPPED (logged), 0.605 < R(90) <= 0.625 are "marginal" (run).
# 2. IK frontier (orbit_waypoints.py --mode ik-frontier) for every cell not skipped:
#    the limiting azimuth per shell at 1 deg, collision-aware.
# 3. Orbit execution: 1 trial per runnable cell (at least the az 0 waypoint of every shell
#    reachable collision-free, the dog clear of the rack), then --repeats more on the 5 of them
#    with the widest total azimuth reach.
# 4. coverage_eval.py on every run, then everything into OUT/orbit_trials.csv (and
#    results/orbit_trials.csv in the repo).
# Guards: before every trial the GPU tracker must report >= 4 colour+depth pairs/s
# (else the trial is logged TRACKER_STARVED); a trial that hangs (no Gazebo clock for
# 10 s, or one waypoint past 180 s: exit 42; or the whole 17-waypoint orbit past 1200 s) is
# killed, logged ABORTED_SIM_HANG, the simulation
# restarted and the sweep continues; more than 5 restarts stops the sweep.
# Needs the stack of run_sim_demo.sh (this starts it with --health --keep).
# ---------------------------------------------------------------------------
set -o pipefail   # no -u: ROS's setup.bash reads unset variables

REPO="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)"
NAV2_WIFI_DIR="${NAV2_WIFI_DIR:-$HOME/files/go2-nav2-wifi}"
OUT="" REPEATS=3 FRONTIER_ONLY=0 MAX_RESTARTS=5 KEEP_SIM=0 ORBIT_ONLY=0 WORLD=cropking_nft46 PLANTS="" CELL="" MODEL="" SKIP_FIRST=0
DS="0.35 0.40 0.45 0.50" RS="0.25 0.30 0.35 0.40"
while [ $# -gt 0 ]; do
    case "$1" in
        --out) OUT="$2"; shift ;;
        --repeats) REPEATS="$2"; shift ;;
        --frontier-only) FRONTIER_ONLY=1 ;;
        --orbit-only) ORBIT_ONLY=1 ;;          # reuse OUT's frontier
        --keep-sim) KEEP_SIM=1 ;;
        --world) WORLD="$2"; shift ;;           # gazebo/worlds/<world> (species: cropking_nft46_<species>)
        --plants) PLANTS="$2"; shift ;;         # orbit these front_plants of the world at --cell (Phase 3)
        --cell) CELL="$2"; shift ;;             # "d r" for --plants
        --model) MODEL="$2"; shift ;;           # detector for coverage_eval (default: the lettuce one)
        --skip-first) SKIP_FIRST=1 ;;           # with --orbit-only: only the repeats (the 1-trial pass is done)
        --ds) DS="$2"; shift ;;
        --rs) RS="$2"; shift ;;
        -h|--help) awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; exit 0 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
OUT="${OUT:-$NAV2_WIFI_DIR/ws/log/orbit/sweep_$(date +%Y%m%d-%H%M%S)}"
mkdir -p "$OUT"
SIM=go2-sim
RESTARTS=0
log() { echo "[sweep $(date +%H:%M:%S)] $*" | tee -a "$OUT/sweep.log"; }

start_stack() {
    if docker ps -q --filter "name=^${SIM}$" | grep -q . && [ "$RESTARTS" = 0 ]; then
        log "reusing the running simulation"
    else
        log "starting the simulation stack (run_sim_demo.sh --health --keep)"
        DEMO_WORLD="$WORLD" "$REPO/scripts/runtime/run_sim_demo.sh" --health --keep --out "$OUT/stack_$RESTARTS" > "$OUT/stack_$RESTARTS.log" 2>&1 \
            || { log "stack health check failed: $OUT/stack_$RESTARTS.log"; return 1; }
    fi
    docker cp "$REPO/scripts/nodes/orbit_waypoints.py" "$SIM:/tmp/orbit_orbit_waypoints.py" >/dev/null
    docker exec "$SIM" mkdir -p /tmp/orbit_src /tmp/orbit_out
    docker cp "$REPO/src/d1_moveit_config/scripts/servo_demo_trials.py" "$SIM:/tmp/orbit_src/" >/dev/null
    docker cp "$REPO/src/d1_moveit_config/scripts/full_mission_test.py" "$SIM:/tmp/orbit_src/" >/dev/null
    # the GPU tracker (as run_sim_demo.sh starts it), kept running for the whole sweep
    cp "$HOME/.cache/go2-yolo/lettuce_yolo11s.pt" "$OUT/lettuce_yolo11s.pt"
    (source /opt/ros/humble/setup.bash; unset RMW_IMPLEMENTATION CYCLONEDDS_URI; \
        ROS_DOMAIN_ID=64 ROS_LOCALHOST_ONLY=1 exec python3 -u "$REPO/scripts/nodes/yolo_depth_tracker.py" --ros-args \
        -p model:="$OUT/lettuce_yolo11s.pt" -p device:=cuda:0 -p target_class:=lettuce -p depth_mode:=box \
        -p color_topic:=/camera/color/image_raw -p depth_topic:=/camera/aligned_depth_to_color/image_raw) \
        > "$OUT/tracker_$RESTARTS.log" 2>&1 < /dev/null &
    TRACKER_PID=$!
    sleep 25
}

stop_stack() {
    [ -n "${TRACKER_PID:-}" ] && kill -INT "$TRACKER_PID" 2>/dev/null
    local p g
    p=$(ps -eo pid,cmd | grep -E "^ *[0-9]+ bash ./scripts/runtime/wifi_nav.sh --sim" | awk '{print $1}' | head -1)
    if [ -n "$p" ]; then
        g=$(ps -o pgid= -p "$p" | tr -d ' ')
        kill -INT -- "-$g" 2>/dev/null
        for _ in $(seq 1 60); do kill -0 "$p" 2>/dev/null || break; sleep 2; done
    fi
    "$REPO/gazebo/run_sim.sh" down >/dev/null 2>&1 || true
}
[ "$KEEP_SIM" = 1 ] || trap stop_stack EXIT

tracker_ok() {
    local r
    r=$(grep -o "[0-9.]* colour+depth pairs/s" "$OUT/tracker_$RESTARTS.log" 2>/dev/null | tail -1 | awk '{print $1+0}')
    awk -v r="${r:-0}" 'BEGIN { exit !(r >= 4) }'
}

in_sim() {  # in_sim TIMEOUT ARGS... -> exit code of orbit_waypoints.py
    local t="$1"; shift
    docker exec "$SIM" bash -c "source /opt/ros/humble/setup.bash; source /tmp/moveit_ws/install/setup.bash; \
        export ROS_DOMAIN_ID=66 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///tmp/sim-gen/cyclonedds.xml \
        GZ_PARTITION=go2sim GZ_IP=127.0.0.1 MISSION_DIR=/tmp/orbit_src; \
        timeout $t python3 -u /tmp/orbit_orbit_waypoints.py --world-json /sim/worlds/$WORLD.json $*"
}

restart() {
    RESTARTS=$((RESTARTS + 1))
    log "ABORTED_SIM_HANG: restarting the simulation (restart $RESTARTS)"
    if [ "$RESTARTS" -gt "$MAX_RESTARTS" ]; then
        log "more than $MAX_RESTARTS restarts: the simulation is unstable, stopping the sweep"
        exit 3
    fi
    stop_stack
    start_stack || exit 4
}

# ---------------------------------------------------------------- 1. analytic classes
CELLS=()
: > "$OUT/cells.tsv"
for d in $DS; do
    for r in $RS; do
        cls=$(python3 -c "import math; R=math.hypot($d,$r); print('skip' if R > 0.625 else 'marginal' if R > 0.605 else 'feasible', round(R,3))")
        echo -e "$d\t$r\t$cls" >> "$OUT/cells.tsv"
        if [ "${cls%% *}" = skip ]; then
            log "SKIP d=$d r=$r: analytic R(90) = ${cls#* } m > 0.625 (reach 0.605)"
        else
            CELLS+=("$d $r")
        fi
    done
done

start_stack || exit 4

if [ -n "$PLANTS" ]; then
    # Phase 3: the orbit at one cell around each listed plant, then coverage with --model
    set -- $CELL
    D="$1" R="$2"
    mkdir -p "$OUT/runs"
    for p in $PLANTS; do
        rid="${WORLD}_${p}_d${D}_r${R}"
        if ! tracker_ok; then log "TRACKER_STARVED before $rid"; echo "$rid TRACKER_STARVED" >> "$OUT/aborted.txt"; continue; fi
        in_sim 1200 --mode orbit --d "$D" --r "$R" --shells all --plant "$p" --out /tmp/orbit_out --run-id "$rid" 2>&1 \
            | grep -E "^\[orbit\]" | tee -a "$OUT/sweep.log"
        code=${PIPESTATUS[0]}
        if [ "$code" = 42 ] || [ "$code" = 124 ]; then echo "$rid ABORTED_SIM_HANG" >> "$OUT/aborted.txt"; restart; continue; fi
        docker cp "$SIM:/tmp/orbit_out/$rid" "$OUT/runs/" >/dev/null 2>&1
    done
    python3 "$REPO/scripts/nodes/coverage_eval.py" --runs "$OUT/runs" --world-json "$REPO/gazebo/worlds/$WORLD.json" \
        --model "${MODEL:-$OUT/lettuce_yolo11s.pt}" --per-shell --csv "$OUT/orbit_trials.csv" | tee -a "$OUT/sweep.log"
    log "done: $OUT/orbit_trials.csv"
    exit 0
fi

# ---------------------------------------------------------------- 2. IK frontier
[ "$ORBIT_ONLY" = 1 ] && CELLS_FRONTIER=() || CELLS_FRONTIER=("${CELLS[@]}")
for c in "${CELLS_FRONTIER[@]}"; do
    set -- $c
    # no motion: up to 1500 s per cell (Python IK); only a stopped Gazebo clock (42) is a hang
    in_sim 1500 --mode ik-frontier --d "$1" --r "$2" --out "/tmp/orbit_out/frontier_d$1_r$2" 2>&1 | grep -E "^\[orbit\]" | tee -a "$OUT/sweep.log"
    code=${PIPESTATUS[0]}
    [ "$code" = 124 ] && log "frontier d=$1 r=$2: over 1500 s, cell incomplete"
    [ "$code" = 42 ] && restart
    mkdir -p "$OUT/frontier"
    docker cp "$SIM:/tmp/orbit_out/frontier_d$1_r$2/frontier.json" "$OUT/frontier/d$1_r$2.json" >/dev/null 2>&1
done
python3 "$REPO/scripts/tools/orbit_frontier_table.py" "$OUT" | tee "$OUT/frontier.md"
[ "$FRONTIER_ONLY" = 1 ] && exit 0

# ---------------------------------------------------------------- 3. orbits
mapfile -t RUN_CELLS < <(python3 "$REPO/scripts/tools/orbit_frontier_table.py" "$OUT" --list runnable)
# repeats: on the 5 runnable cells with the widest total azimuth reach (no cell reaches every
# waypoint since the first frontier, so "the feasible cells" of the plan are these)
mapfile -t FULL_CELLS < <(python3 "$REPO/scripts/tools/orbit_frontier_table.py" "$OUT" --list best)
run_cell() {  # run_cell d r rep
    local rid="d$1_r$2_t$3"
    if ! tracker_ok; then
        log "TRACKER_STARVED before $rid: tracker below 4 pairs/s"; echo "$rid TRACKER_STARVED" >> "$OUT/aborted.txt"; return
    fi
    in_sim 1200 --mode orbit --d "$1" --r "$2" --shells all --out /tmp/orbit_out --run-id "$rid" 2>&1 \
        | grep -E "^\[orbit\]" | tee -a "$OUT/sweep.log"
    local code=${PIPESTATUS[0]}
    if [ "$code" = 42 ] || [ "$code" = 124 ]; then
        echo "$rid ABORTED_SIM_HANG" >> "$OUT/aborted.txt"
        restart
        return
    fi
    docker cp "$SIM:/tmp/orbit_out/$rid" "$OUT/runs/" >/dev/null 2>&1 || mkdir -p "$OUT/runs" && docker cp "$SIM:/tmp/orbit_out/$rid" "$OUT/runs/" >/dev/null 2>&1
}
mkdir -p "$OUT/runs"
[ "$SKIP_FIRST" = 1 ] || for c in "${RUN_CELLS[@]}"; do set -- $c; run_cell "$1" "$2" 1; done
for c in "${FULL_CELLS[@]}"; do set -- $c; for t in $(seq 2 $((REPEATS + 1))); do run_cell "$1" "$2" "$t"; done; done

# ---------------------------------------------------------------- 4. coverage + CSV
python3 "$REPO/scripts/nodes/coverage_eval.py" --runs "$OUT/runs" --world-json "$REPO/gazebo/worlds/cropking_nft46.json" \
    --model "$OUT/lettuce_yolo11s.pt" --per-shell --csv "$OUT/orbit_trials.csv" | tee -a "$OUT/sweep.log"
mkdir -p "$REPO/results"
cp "$OUT/orbit_trials.csv" "$REPO/results/orbit_trials.csv"
cp "$OUT/frontier.md" "$REPO/results/orbit_frontier.md"
log "done: $OUT/orbit_trials.csv"
