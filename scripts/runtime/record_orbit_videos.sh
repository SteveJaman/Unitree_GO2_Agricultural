#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# record_orbit_videos.sh: the orbit study's video proof (Phase 6), screen captures of the
# simulation desktop. MP4s go to ~/files/go2-nav2-wifi/ws/log/orbit_video/<run_id>/ (never git).
#
#   ./scripts/runtime/record_orbit_videos.sh [--early-k "2 3 9 14"] [--only "v1 v3a v3b v3c"]
#
# V1   showcase/showcase.mp4: one full servo trial (Nav2 drive to the rack, park tail-first,
#      distance servo to 0.50 m) with Gazebo (camera following the dog), Nav2 RViz, MoveIt RViz
#      and the camera RViz tiled 2x2 (desktop_capture.py), plus the composed demo.mp4 of
#      record_demo_video.py (wrist camera with the YOLO box, live distance).
# V3a  v3_lettuce_full/: a full 3-shell orbit of the control lettuce at (0.40, 0.25).
# V3c  v3_lettuce_early_stop/: the orbit restricted to the early-stop policy's waypoints
#      (--early-k: autonomy_policy.py's stops for the same plant; orbit_waypoints.py --only-k).
# V3b  v3_strawberry_under/: the under shell of a strawberry plant (world cropking_nft46_strawberry).
# V2 (wrist_<species>_<shell>.mp4) is made from the logged frames by scripts/tools/wrist_video.py.
#
# Needs a desktop session on $DISPLAY (Gazebo GUI, RViz), python-xlib (PYTHONPATH), ffmpeg
# ($FFMPEG, PATH or imageio-ffmpeg). No `set -u`: the ROS setup scripts it sources trip on it.
# ---------------------------------------------------------------------------
set -eo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$REPO"
V="$HOME/files/go2-nav2-wifi/ws/log/orbit_video"
EARLY_K="" ONLY=""
while [ $# -gt 0 ]; do
    case "$1" in
        --early-k) EARLY_K="$2"; shift ;;
        --only) ONLY="$2"; shift ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
export DISPLAY="${DISPLAY:-:0}"
SIM=go2-sim
log() { echo "[video $(date +%H:%M:%S)] $*"; }
want() { [ -z "$ONLY" ] || [[ " $ONLY " == *" $1 "* ]]; }

REC_PID=""
rec_start() {  # rec_start OUTDIR -> records OUTDIR/screen.mkv
    mkdir -p "$1"
    python3 scripts/tools/desktop_capture.py tile --wait 60 >/dev/null 2>&1 || true
    python3 scripts/tools/desktop_capture.py record "$1/screen.mkv" > "$1/record.log" 2>&1 &
    REC_PID=$!
    sleep 3
}
rec_stop() {  # rec_stop OUTDIR NAME -> OUTDIR/NAME.mp4
    [ -n "$REC_PID" ] && kill -INT "$REC_PID" 2>/dev/null && wait "$REC_PID" 2>/dev/null || true
    REC_PID=""
    local ff
    ff="${FFMPEG:-$(command -v ffmpeg || python3 -c 'import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())')}"
    "$ff" -y -loglevel error -i "$1/screen.mkv" -c copy -movflags +faststart "$1/$2.mp4" && rm -f "$1/screen.mkv"
    log "$1/$2.mp4: $("$ff" -i "$1/$2.mp4" 2>&1 | grep -o 'Duration: [0-9:.]*')"
}
trap '[ -n "$REC_PID" ] && kill -INT "$REC_PID" 2>/dev/null; true' EXIT

orbit() {  # orbit WORLD RUN_ID ARGS... (in the running simulation)
    local world="$1" rid="$2"; shift 2
    docker cp scripts/nodes/orbit_waypoints.py "$SIM:/tmp/orbit_orbit_waypoints.py" >/dev/null
    docker exec "$SIM" mkdir -p /tmp/orbit_src /tmp/orbit_out
    docker cp src/d1_moveit_config/scripts/servo_demo_trials.py "$SIM:/tmp/orbit_src/" >/dev/null
    docker cp src/d1_moveit_config/scripts/full_mission_test.py "$SIM:/tmp/orbit_src/" >/dev/null
    docker exec "$SIM" bash -c "source /opt/ros/humble/setup.bash; source /tmp/moveit_ws/install/setup.bash; \
        export ROS_DOMAIN_ID=66 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///tmp/sim-gen/cyclonedds.xml \
        GZ_PARTITION=go2sim GZ_IP=127.0.0.1 MISSION_DIR=/tmp/orbit_src; \
        timeout 1200 python3 -u /tmp/orbit_orbit_waypoints.py --world-json /sim/worlds/$world.json --mode orbit \
        --d 0.40 --r 0.25 --out /tmp/orbit_out --run-id $rid $*" 2>&1 | grep -E "^\[orbit\]" || true
}

down() {  # the session (wifi_nav.sh --sim, left up by --keep), the simulation, the camera RViz
    local p g
    p=$(pgrep -f "bash ./scripts/runtime/wifi_nav.sh --sim" | head -1)
    if [ -n "$p" ]; then
        g=$(ps -o pgid= -p "$p" | tr -d ' ')
        kill -INT -- "-$g" 2>/dev/null
        for _ in $(seq 1 30); do kill -0 "$p" 2>/dev/null || break; sleep 2; done
        kill -0 "$p" 2>/dev/null && kill -KILL -- "-$g" 2>/dev/null
    fi
    ./gazebo/run_sim.sh down >/dev/null 2>&1 || true
    ./scripts/runtime/plant_detection.sh --stop >/dev/null 2>&1 || true
}

# ---- V1 + V3a + V3c: the control world
if want v1 || want v3a || want v3c; then
    down
    if want v1; then
        D="$V/showcase"
        mkdir -p "$D"
        log "V1: full servo trial, GUI + RViz, recording the desktop"
        # the demo brings the windows up itself: start recording once they exist
        ( for _ in $(seq 1 90); do python3 scripts/tools/desktop_capture.py list 2>/dev/null | grep -qi "plant_detection" && break; sleep 5; done
          python3 scripts/tools/desktop_capture.py tile --wait 60 >/dev/null 2>&1 || true
          exec python3 scripts/tools/desktop_capture.py record "$D/screen.mkv" ) > "$D/record.log" 2>&1 &
        REC_PID=$!
        ./scripts/runtime/run_sim_demo.sh --gui --rviz --video --trials 1 --policies proportional --keep --no-train \
            --out "$D/demo" > "$D/demo.log" 2>&1 || log "V1: run_sim_demo.sh exited non-zero (see $D/demo.log)"
        sleep 5
        rec_stop "$D" showcase
        [ -f "$D/demo/demo.mp4" ] && cp "$D/demo/demo.mp4" "$D/composed_demo.mp4"
    else
        ./scripts/runtime/run_sim_demo.sh --health --gui --rviz --keep --out "$V/stack_control" > "$V/stack_control.log" 2>&1
    fi
    if want v3a; then
        D="$V/v3_lettuce_full"
        log "V3a: full orbit, lettuce"
        rec_start "$D"
        orbit cropking_nft46 v3_lettuce_full --shells all | tee "$D/orbit.log"
        rec_stop "$D" v3_lettuce_full
        docker cp "$SIM:/tmp/orbit_out/v3_lettuce_full" "$D/run" >/dev/null 2>&1 || true
    fi
    if want v3c && [ -n "$EARLY_K" ]; then
        D="$V/v3_lettuce_early_stop"
        log "V3c: early-stop orbit, waypoints $EARLY_K"
        rec_start "$D"
        orbit cropking_nft46 v3_lettuce_early_stop --shells all --only-k $EARLY_K | tee "$D/orbit.log"
        rec_stop "$D" v3_lettuce_early_stop
        docker cp "$SIM:/tmp/orbit_out/v3_lettuce_early_stop" "$D/run" >/dev/null 2>&1 || true
    fi
    down
fi

# ---- V3b: strawberry, under shell
if want v3b; then
    D="$V/v3_strawberry_under"
    mkdir -p "$D"
    log "V3b: strawberry world, under shell"
    DEMO_WORLD=cropking_nft46_strawberry ./scripts/runtime/run_sim_demo.sh --health --gui --rviz --keep \
        --out "$D/stack" > "$D/stack.log" 2>&1 || log "V3b: stack health failed (see $D/stack.log)"
    rec_start "$D"
    orbit cropking_nft46_strawberry v3_strawberry_under --shells under --plant strawberry_0_3 | tee "$D/orbit.log"
    rec_stop "$D" v3_strawberry_under
    docker cp "$SIM:/tmp/orbit_out/v3_strawberry_under" "$D/run" >/dev/null 2>&1 || true
    down
fi
log "done: $V"
