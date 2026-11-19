#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# run_sim_demo.sh: the final demo, end to end in Gazebo (headless), with data and graphs.
#
#   ./run_sim_demo.sh                       3 policies x 5 trials (about 30 min)
#   ./run_sim_demo.sh --trials 3 --policies proportional
#   ./run_sim_demo.sh --skip-nav --keep     drive straight to the rack; leave the sim running
#
# 1. World: gazebo/worlds/cropking_nft46.sdf, our NFT 4-6 with lettuce (rebuilt if its
#    builder changed); the lettuce detector ~/.cache/go2-yolo/lettuce_yolo11s.pt (made
#    by scripts/tools/lettuce_dataset.py --train if missing: COCO has no lettuce).
# 2. Session: wifi_nav.sh --sim (Gazebo server only, no GUI; Nav2 in go2-lidar3d on domain
#    64), MoveIt 2 + gz_ros2_control for the D1 (domain 66) with a RealSense D435i on the
#    claw (SIM_ARM_EE=claw; the real driver's /camera/* topics). Realistic by default:
#    SIM_REALISM=1 (arm on effort control under gravity, body lag and bob), SIM_GAIT=1
#    (the legs trot); set either to 0 for the plain kinematic sim.
# 3. Tracker: yolo_depth_tracker.py, lettuce -> /object_distance on 66: on the host GPU if it
#    has CUDA PyTorch + ultralytics, else in go2-3d:local on the CPU (SIM_TRACKER=host|container).
# 4. Trials (src/d1_moveit_config/scripts/servo_demo_trials.py in go2-sim): stow, Nav2 to
#    the rack, park tail-first; then per trial: start pose (J2 0.25, J3 0.30, J5 -0.55),
#    a randomised parking offset, distance_servo.py until it settles at 0.50 m, logging
#    the servo's CSVs, Gazebo ground truth and wrist RGB-D before/after every move.
# 5. Report: scripts/tools/servo_trials_report.py -> distance_over_time.png and friends.
#
# Output: OUT (default ~/files/go2-nav2-wifi/ws/log/servo_demo/<date-time>), see the end.
# Options: --trials N (per policy, 5) --policies "a b" --skip-nav --keep --out DIR
#          --timeout S (per trial, 60) --no-train
#          --reuse: trials in a simulation left up by --keep (implies --keep --skip-nav)
#          --health: bring everything up, measure the wrist camera's rate (>= 10 Hz), the tracker's
#                  colour+depth pairs (>= 4/s), list the RViz windows, screenshot the desktop
#                  (OUT/health.png; tiled with --gui --rviz), write OUT/health.json, stop. No trials.
#          --disturb M: 4 s into each trial the dog slides M metres (alternately away from and
#                  towards the rack): the servo has to recover, open loop cannot
#          --gui: the Gazebo window on the desktop (the demo itself stays the same)
#          --video: record OUT/demo.mp4 (record_demo_video.py: overview / side view, the
#                   wrist camera with the YOLO box, live distance); H.264 with ffmpeg
#                   (system, FFMPEG=path, or pip install imageio-ffmpeg), else MPEG-4
#          --rviz: two RViz windows: the arm (MoveIt MotionPlanning: planning scene, planned and
#                  real arm) and the wrist camera through the test branch's RGB-D plant detector
#                  (plant_detection.sh: YOLO view, 3D lettuce boxes and points), lettuce model
#   Showcase:  ./run_sim_demo.sh --gui --rviz --video --trials 1 --policies proportional
# Needs: go2-sim-moveit:local (./gazebo/setup_gazebo.sh --moveit), go2-lidar3d with the
# detector's packages (./scripts/tools/detection_setup.sh), ~/files/unitree-D1-550-Robot-ARM,
# a desktop session (Gazebo's sensors render through $DISPLAY), the host's Python with
# ultralytics + matplotlib (training and the report).
# ---------------------------------------------------------------------------
set -uo pipefail

REPO="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)"
NAV2_WIFI_DIR="${NAV2_WIFI_DIR:-$HOME/files/go2-nav2-wifi}"
TRIALS=5 POLICIES="proportional fixed_step open_loop" SKIP_NAV=0 KEEP=0 TIMEOUT=60 TRAIN=1 REUSE=0 GUI=0 VIDEO=0 RVIZ=0 DISTURB=0 HEALTH=0
OUT=""
while [ $# -gt 0 ]; do
    case "$1" in
        --trials) TRIALS="$2"; shift ;;
        --policies) POLICIES="$2"; shift ;;
        --skip-nav) SKIP_NAV=1 ;;
        --keep) KEEP=1 ;;
        --timeout) TIMEOUT="$2"; shift ;;
        --out) OUT="$2"; shift ;;
        --no-train) TRAIN=0 ;;
        --reuse) REUSE=1; KEEP=1; SKIP_NAV=1 ;;
        --gui) GUI=1 ;;
        --video) VIDEO=1 ;;
        --rviz) RVIZ=1 ;;
        --disturb) DISTURB="$2"; shift ;;
        --health) HEALTH=1 ;;
        -h|--help) awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; exit 0 ;;
        *) echo "unknown option $1 (--help)" >&2; exit 2 ;;
    esac
    shift
done
OUT="${OUT:-$NAV2_WIFI_DIR/ws/log/servo_demo/$(date +%Y%m%d-%H%M%S)}"
mkdir -p "$OUT"
SIM=go2-sim LIDAR=go2-lidar3d
WEIGHTS="$HOME/.cache/go2-yolo/lettuce_yolo11s.pt"
SESSION_LOG="$OUT/session.log"
log() { echo "[demo $(date +%H:%M:%S)] $*" | tee -a "$OUT/demo.log"; }
die() { log "ERROR: $*"; exit 1; }

# ---------------------------------------------------------------- 1. world + detector
cd "$REPO" || exit 1
W=gazebo/worlds
[ -f "$W/models/lettuce/meshes/lettuce.obj" ] && [ "$W/models/lettuce/meshes/lettuce.obj" -nt "$W/build_lettuce_model.py" ] \
    || python3 "$W/build_lettuce_model.py" | tee -a "$OUT/demo.log"
[ "$W/cropking_nft46.sdf" -nt "$W/build_cropking_nft46_world.py" ] || python3 "$W/build_cropking_nft46_world.py" | tee -a "$OUT/demo.log"
if [ ! -f "$WEIGHTS" ]; then
    [ "$TRAIN" = 1 ] || die "no $WEIGHTS (run without --no-train, or scripts/tools/lettuce_dataset.py --train)"
    log "no lettuce detector yet: generating the Gazebo dataset and fine-tuning yolo11s (~15 min)"
    python3 scripts/tools/lettuce_dataset.py --frames 700 --train 2>&1 | grep -E "^\[dataset\]|Epoch|all " | tee -a "$OUT/demo.log"
    [ -f "$WEIGHTS" ] || die "training did not produce $WEIGHTS"
fi
docker image inspect go2-sim-moveit:local >/dev/null 2>&1 || die "no go2-sim-moveit:local: ./gazebo/setup_gazebo.sh --moveit"
if docker ps -q --filter "name=^${SIM}$" | grep -q .; then
    [ "$REUSE" = 1 ] || die "${SIM} is already running: stop it first (./gazebo/run_sim.sh down), or --reuse it"
elif [ "$REUSE" = 1 ]; then
    die "--reuse: no ${SIM} running (start one with --keep)"
fi

# ---------------------------------------------------------------- 2. session
SESSION_PID=""
cleanup() {
    trap '' INT TERM
    log "cleaning up"
    docker rm -f go2-demo-tracker >/dev/null 2>&1 || true
    [ -n "${TRACKER_PID:-}" ] && kill -INT "$TRACKER_PID" 2>/dev/null
    if [ "$KEEP" = 0 ] && [ "$RVIZ" = 1 ]; then
        ./scripts/runtime/plant_detection.sh --stop >/dev/null 2>&1 || true
        docker exec "$LIDAR" pkill -f "child-frame-id camera_color_optical_frame" >/dev/null 2>&1 || true
    fi
    docker exec "$SIM" pkill -INT -f "servo_demo_trials.py|distance_servo.py|record_demo_video.py" >/dev/null 2>&1 || true
    if [ "$KEEP" = 0 ]; then
        [ -n "$SESSION_PID" ] && kill -INT -- "-$SESSION_PID" 2>/dev/null
        for _ in $(seq 1 60); do [ -n "$SESSION_PID" ] && kill -0 "$SESSION_PID" 2>/dev/null || break; sleep 1; done
        [ -n "$SESSION_PID" ] && kill -KILL -- "-$SESSION_PID" 2>/dev/null
        ./gazebo/run_sim.sh down >/dev/null 2>&1 || true
    else
        log "--keep: the simulation stays up (./gazebo/run_sim.sh down to stop it)"
    fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

if [ "$REUSE" = 0 ]; then
log "starting the simulation session (headless Gazebo, Nav2, MoveIt; log $SESSION_LOG)"
SIM_WORLD=cropking_nft46 SIM_SPAWN="0 0 0" SIM_MOVEIT=1 SIM_ARM=0 SIM_WRIST_CAMERA=d435i SIM_GUI="$GUI" \
SIM_ARM_EE="${SIM_ARM_EE:-claw}" SIM_REALISM="${SIM_REALISM:-1}" SIM_GAIT="${SIM_GAIT:-1}" \
SIM_CAMERA=0 SIM_TIMEOUT=400 ARM_IFACE=simarm0 \
    setsid ./scripts/runtime/wifi_nav.sh --sim --host-ip 127.0.0.1 --no-camera --no-explore \
    > "$SESSION_LOG" 2>&1 < /dev/null &
SESSION_PID=$!

wait_for() {  # wait_for WHAT TIMEOUT CHECK...
    local what="$1" timeout="$2" t0=$SECONDS; shift 2
    until "$@" >/dev/null 2>&1; do
        kill -0 "$SESSION_PID" 2>/dev/null || die "the session exited while waiting for ${what}: tail $SESSION_LOG"
        [ $((SECONDS - t0)) -gt "$timeout" ] && die "${what} not up after ${timeout} s: tail $SESSION_LOG"
        sleep 3
    done
    log "${what} up after $((SECONDS - t0)) s"
}
nav2_up() { [ "$(grep -c 'Managed nodes are active' "$SESSION_LOG")" -ge 2 ]; }
moveit_up() { docker exec "$SIM" grep -q "Configured and activated.*arm_controller\|arm_controller.*Configured and activated" /tmp/sim-gen/moveit.log; }
wait_for "Nav2" 900 nav2_up
wait_for "MoveIt" 300 moveit_up
else
    log "reusing the running simulation"
    SESSION_PID=$$   # nothing of ours to watch: wait_for's liveness check passes
fi

ROS_LIDAR='source /opt/ros/humble/setup.bash; source /ws/install/setup.bash 2>/dev/null; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///tmp/go2-wifi-cyclonedds.xml ROS_DOMAIN_ID=64'
camera_up() { docker exec "$LIDAR" bash -c "$ROS_LIDAR; timeout 10 ros2 topic echo --once /camera/color/camera_info --field width"; }
wait_for "wrist camera (/camera/color/image_raw)" 120 camera_up
docker exec "$LIDAR" bash -c "$ROS_LIDAR; timeout 8 ros2 topic hz /camera/color/image_raw 2>&1 | grep -m2 'average rate'" \
    | sed 's/^/  colour /' | tee -a "$OUT/demo.log"
docker exec "$LIDAR" bash -c "$ROS_LIDAR; timeout 8 ros2 topic hz /camera/aligned_depth_to_color/image_raw 2>&1 | grep -m2 'average rate'" \
    | sed 's/^/  depth  /' | tee -a "$OUT/demo.log"

if [ "$GUI" = 1 ]; then
    # the Gazebo window's camera follows the dog, from the rack side
    for _ in $(seq 1 20); do
        docker exec "$SIM" bash -c 'export GZ_PARTITION=go2sim GZ_IP=127.0.0.1; \
            gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 2000 --req "data: \"go2\"" \
            && gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 2000 \
               --req "x: -0.6, y: -1.0, z: 0.55"' 2>/dev/null | grep -q true && break
        sleep 3
    done
fi

# ---------------------------------------------------------------- 3. tracker
# go2-3d:local has YOLO (ultralytics), OpenCV and message_filters; the sim's Cyclone config
# (loopback, any domain) serves the camera's domain 64 and MoveIt's 66 alike.
docker cp "$SIM:/tmp/sim-gen/cyclonedds.xml" "$OUT/cyclonedds.xml" >/dev/null || die "no Cyclone config in $SIM"
chmod 644 "$OUT/cyclonedds.xml"
cp "$WEIGHTS" "$OUT/lettuce_yolo11s.pt"
docker rm -f go2-demo-tracker >/dev/null 2>&1 || true
ANNOTATED=""   # an empty -p NAME:= is a fatal ROS argument error: only when recording
{ [ "$VIDEO" = 1 ] || [ "$RVIZ" = 1 ]; } && ANNOTATED="-p annotated_topic:=/tracker/annotated"
TRACKER_ARGS="-p target_class:=lettuce -p min_confidence:=0.4 -p depth_mode:=box \
 -p color_topic:=/camera/color/image_raw -p depth_topic:=/camera/aligned_depth_to_color/image_raw -p out_domain_id:=66 $ANNOTATED"
TRACKER_PID=""
# On the host's NVIDIA GPU when it has CUDA PyTorch and ultralytics (SIM_TRACKER=host|container|auto):
# with the Gazebo window, RViz and the video, YOLO on the CPU starved (~1 frame/s). The host
# talks to the sim's Cyclone DDS over loopback with localhost-only FastDDS.
if [ "${SIM_TRACKER:-auto}" != container ] && (set +u; source /opt/ros/humble/setup.bash 2>/dev/null; \
        python3 -c 'import torch, ultralytics, message_filters; assert torch.cuda.is_available()') >/dev/null 2>&1; then
    # shellcheck disable=SC2086  # TRACKER_ARGS is a list of ROS arguments
    (set +u; source /opt/ros/humble/setup.bash; unset RMW_IMPLEMENTATION CYCLONEDDS_URI; \
        ROS_DOMAIN_ID=64 ROS_LOCALHOST_ONLY=1 exec python3 -u "$REPO/scripts/nodes/yolo_depth_tracker.py" --ros-args \
        -p model:="$OUT/lettuce_yolo11s.pt" -p device:=cuda:0 $TRACKER_ARGS) > "$OUT/tracker.log" 2>&1 < /dev/null &
    TRACKER_PID=$!
    log "tracker on the host GPU (pid $TRACKER_PID)"
else
    [ "${SIM_TRACKER:-auto}" = host ] && die "SIM_TRACKER=host: no CUDA PyTorch + ultralytics on the host"
    docker run -d --name go2-demo-tracker --network host --ipc host --user "$(id -u):$(id -g)" -e HOME=/tmp \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///out/cyclonedds.xml -e ROS_DOMAIN_ID=64 \
        -v "$REPO/scripts/nodes:/nodes:ro" -v "$OUT:/out" -w /out --entrypoint bash go2-3d:local -c \
        'source /opt/ros/humble/setup.bash && exec python3 -u /nodes/yolo_depth_tracker.py --ros-args \
         -p model:=/out/lettuce_yolo11s.pt '"$TRACKER_ARGS"' > /out/tracker.log 2>&1' \
        >/dev/null || die "could not start the tracker container"
    log "tracker in go2-3d:local (CPU)"
fi
tracker_up() { grep -q "colour+depth pairs/s" "$OUT/tracker.log"; }
wait_for "YOLO tracker" 180 tracker_up
grep -m1 "pairs/s" "$OUT/tracker.log" | tee -a "$OUT/demo.log"

if [ "$RVIZ" = 1 ]; then
    xhost "+SI:localuser:$(id -un)" >/dev/null 2>&1 || true
    if [ "${SIM_ARM_EE:-claw}" = claw ]; then EE="end_effector:=gripper claw_camera:=true"
    else EE="end_effector:=camera camera_model:=d435i"; fi
    docker exec -d "$SIM" bash -c 'source /opt/ros/humble/setup.bash; source /tmp/moveit_ws/install/setup.bash; \
        export ROS_DOMAIN_ID=${SIM_ARM_DOMAIN:-66} RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///tmp/sim-gen/cyclonedds.xml; \
        ros2 launch d1_moveit_config rviz.launch.py hardware:=gazebo '"$EE"' > /tmp/sim-gen/rviz_arm.log 2>&1'
    # The camera: the RGB-D plant detector from the test branch (src/object_detection_rgbd,
    # plant_detection.sh) on the wrist camera, with its RViz (config/plant_detection.rviz:
    # YOLO view, 3D boxes, the lettuce's points), on the lettuce model.
    docker exec -u root "$LIDAR" mkdir -p /ws/.torch/hub
    docker cp "$WEIGHTS" "$LIDAR:/ws/.torch/hub/lettuce_yolo11s.pt" >/dev/null
    docker exec -u root "$LIDAR" chmod a+r /ws/.torch/hub/lettuce_yolo11s.pt
    DETECT_CONFIG="$REPO/config/detection/object_detection_sim.yaml" setsid ./scripts/runtime/plant_detection.sh --rviz \
        > "$OUT/plant_detection.log" 2>&1 < /dev/null &
    # the real driver publishes the camera's frames on TF; the simulation only stamps its images
    # with camera_color_optical_frame, which plant_detection.rviz uses as its fixed frame
    docker exec -d "$LIDAR" bash -c "$ROS_LIDAR; ros2 run tf2_ros static_transform_publisher \
        --frame-id camera_link --child-frame-id camera_color_optical_frame --roll -1.5708 --yaw -1.5708 > /tmp/camera_tf.log 2>&1"
    log "RViz: the arm (MoveIt) and the plant detector on the wrist camera (log $OUT/plant_detection.log)"
fi

if [ "$HEALTH" = 1 ]; then
    log "health check: camera rate, tracker rate, windows, screenshot"
    hz() {  # hz TOPIC -> average rate over ~10 s
        docker exec "$LIDAR" bash -c "$ROS_LIDAR; timeout 12 ros2 topic hz $1 2>&1" | awk '/average rate/ {r=$3} END {print r+0}'
    }
    COLOR_HZ=$(hz /camera/color/image_raw)
    DEPTH_HZ=$(hz /camera/aligned_depth_to_color/image_raw)
    for _ in $(seq 1 6); do sleep 5; done   # two more tracker reports
    PAIRS=$(grep -o "[0-9.]* colour+depth pairs/s" "$OUT/tracker.log" | tail -1 | awk '{print $1+0}')
    if [ "$GUI" = 1 ] && [ "$RVIZ" = 1 ]; then
        python3 "$REPO/scripts/tools/desktop_capture.py" tile --wait 30 2>&1 | tee -a "$OUT/demo.log" || true
        sleep 3
    fi
    WINDOWS=$(python3 "$REPO/scripts/tools/desktop_capture.py" list 2>/dev/null | grep -iE "rviz|gazebo" || true)
    python3 "$REPO/scripts/tools/desktop_capture.py" shot "$OUT/health.png" >/dev/null 2>&1 || log "WARN: no screenshot"
    python3 - "$OUT/health.json" "$COLOR_HZ" "$DEPTH_HZ" "${PAIRS:-0}" "$WINDOWS" "$GUI" "$RVIZ" <<'PY'
import json, sys
out, color, depth, pairs, windows, gui, rviz = sys.argv[1:]
w = [x for x in windows.splitlines() if x.strip()]
need = (["gazebo"] if gui == "1" else []) + (["lidar3d_navigation", "moveit", "plant_detection"] if rviz == "1" else ["lidar3d_navigation"])
found = {k: any(k in x.lower() and (k == "gazebo" or "rviz" in x.lower()) for x in w) for k in need}
checks = {"color_hz>=10": float(color) >= 10, "depth_hz>=10": float(depth) >= 10, "tracker_pairs>=4": float(pairs) >= 4,
          **{f"window:{k}": v for k, v in found.items()}}
rep = {"color_hz": float(color), "depth_hz": float(depth), "tracker_pairs_per_s": float(pairs), "windows": w,
       "checks": checks, "pass": all(checks.values())}
json.dump(rep, open(out, "w"), indent=1)
print(json.dumps(rep, indent=1))
PY
    if python3 -c "import json,sys; sys.exit(0 if json.load(open('$OUT/health.json'))['pass'] else 1)"; then
        log "HEALTH PASS: colour ${COLOR_HZ} Hz, depth ${DEPTH_HZ} Hz, tracker ${PAIRS} pairs/s"
        exit 0
    fi
    log "HEALTH FAIL: see $OUT/health.json"
    exit 1
fi

# ---------------------------------------------------------------- 4. trials
docker exec "$SIM" mkdir -p /tmp/servo
docker cp scripts/nodes/distance_servo.py "$SIM:/tmp/servo/" >/dev/null
args="--out /tmp/servo_demo --trials $TRIALS --policies $POLICIES --trial-timeout $TIMEOUT"
[ "$SKIP_NAV" = 1 ] && args="$args --skip-nav"
[ "$DISTURB" != 0 ] && args="$args --disturb $DISTURB"
log "trials: $args"
if [ "$VIDEO" = 1 ]; then
    docker exec "$SIM" mkdir -p /tmp/servo_demo_video
    docker exec -d "$SIM" bash -c 'source /opt/ros/humble/setup.bash; \
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///tmp/sim-gen/cyclonedds.xml GZ_PARTITION=go2sim GZ_IP=127.0.0.1; \
        python3 -u /moveit_src/d1_moveit_config/scripts/record_demo_video.py --out /tmp/servo_demo_video/demo.mp4 \
        > /tmp/servo_demo_video/video.log 2>&1'
    log "recording the demo video"
fi
docker exec "$SIM" bash -c 'source /opt/ros/humble/setup.bash; source /tmp/moveit_ws/install/setup.bash; \
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///tmp/sim-gen/cyclonedds.xml GZ_PARTITION=go2sim GZ_IP=127.0.0.1; \
    rm -rf /tmp/servo_demo; python3 -u /moveit_src/d1_moveit_config/scripts/servo_demo_trials.py '"$args" 2>&1 \
    | tee "$OUT/trials.log" | grep --line-buffered -E '^\[trials\]|Error|Traceback'
docker cp "$SIM:/tmp/servo_demo/." "$OUT/" >/dev/null 2>&1 || log "WARN: no trial data to copy"
if [ "$VIDEO" = 1 ]; then
    sleep 3   # a few seconds of the settled arm at the end
    docker exec "$SIM" pkill -INT -f record_demo_video.py >/dev/null 2>&1 || true
    for _ in $(seq 1 30); do docker exec "$SIM" grep -q "frames (" /tmp/servo_demo_video/video.log 2>/dev/null && break; sleep 1; done
    docker cp "$SIM:/tmp/servo_demo_video/demo.mp4" "$OUT/demo_mp4v.mp4" >/dev/null 2>&1 || log "WARN: no video recorded"
    docker exec "$SIM" cat /tmp/servo_demo_video/video.log 2>/dev/null | tail -2 | tee -a "$OUT/demo.log"
    FF="${FFMPEG:-$(command -v ffmpeg || python3 -c 'import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())' 2>/dev/null)}"
    if [ -f "$OUT/demo_mp4v.mp4" ] && [ -n "$FF" ]; then
        "$FF" -loglevel error -y -i "$OUT/demo_mp4v.mp4" -c:v libx264 -pix_fmt yuv420p -crf 23 -movflags +faststart "$OUT/demo.mp4" \
            && rm -f "$OUT/demo_mp4v.mp4" && log "video: $OUT/demo.mp4"
    elif [ -f "$OUT/demo_mp4v.mp4" ]; then
        mv "$OUT/demo_mp4v.mp4" "$OUT/demo.mp4"
        log "video (MPEG-4 part 2: no ffmpeg for H.264; pip install imageio-ffmpeg, or FFMPEG=...): $OUT/demo.mp4"
    fi
fi
docker cp "$SIM:/tmp/sim-gen/moveit.log" "$OUT/moveit.log" >/dev/null 2>&1 || true

# ---------------------------------------------------------------- 5. report
if [ -f "$OUT/trials.json" ]; then
    python3 scripts/tools/servo_trials_report.py "$OUT" 2>&1 | tee -a "$OUT/demo.log"
    log "done: $OUT (distance_over_time.png, results.md, error_vs_step.png, ...)"
else
    die "no trials.json: see $OUT/trials.log"
fi
