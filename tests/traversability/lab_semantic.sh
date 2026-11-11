#!/bin/bash
# Semantic traversability stack in the lab_office world, driven from RViz.
# Run in WSL, in a terminal on the desktop (RViz needs DISPLAY):
#   tests/traversability/lab_semantic.sh up      # sim (if not up) + stack + YOLO + RViz
#   tests/traversability/lab_semantic.sh watch   # follow gait and VLM labels
#   tests/traversability/lab_semantic.sh down    # stop stack, YOLO and RViz (the sim keeps running)
# The stack is run_trial.sh's traversability_semantic mode (the configs of the
# n = 10 trials) with INTERACTIVE=1: no scripted goals. Set goals with RViz's
# "2D Goal Pose" (lidar3d_map = the Gazebo world frame; spawn is (-4.5, 0)).
# Needs Ollama with qwen2.5vl on the Windows host (scripts/config/ollama.yaml).
# Logs and recordings: go2-lidar3d:/tmp/trav_out/lab_semantic, copied to $OUT on down.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
WORLD=lab_office
MODE=traversability_semantic
OUT="${OUT:-$HOME/files/lab_semantic}"
C=go2-lidar3d
ROS='source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///ws/log/3d/cyclonedds.xml ROS_DOMAIN_ID=64'
die() { echo "[lab_semantic] ERROR: $*" >&2; exit 1; }
say() { echo "[lab_semantic] $*"; }

running() { [ "$(docker inspect -f '{{.State.Running}}' "$1" 2>/dev/null)" = true ]; }

sim_up() {
    if running go2-sim; then
        local w
        w="$(docker exec go2-sim printenv SIM_WORLD 2>/dev/null)"
        [ "$w" = "$WORLD" ] || die "go2-sim is running world '$w', not $WORLD: ./gazebo/run_sim.sh down first"
        say "sim already up ($WORLD)"
        return
    fi
    say "starting the sim ($WORLD, camera + gait bridge)"
    (cd "$REPO" && GO2_HOST_IP=127.0.0.1 SIM_ARM=0 SIM_WORLD=$WORLD SIM_CAMERA=1 SIM_GAIT=1 \
        SIM_GUI="${SIM_GUI:-0}" ./gazebo/run_sim.sh up) || die "run_sim.sh up failed (see above)"
}

copy_files() {
    docker exec $C mkdir -p /tmp/trav
    for f in planner.yaml planner_$MODE.yaml controller_$MODE.yaml run_trial.sh start_extra_traversability.sh \
             start_extra_$MODE.sh save_frames.py record_semantic.py save_costmap.py record_json_topic.py lab_semantic.rviz; do
        docker cp "$HERE/$f" $C:/tmp/trav/$f || die "copy $f"
    done
    docker cp "$REPO/scripts/config/ollama.yaml" $C:/tmp/trav/ollama.yaml
    for f in object_box_fusion.py sim_box_detector.py gait_supervisor.py vlm_heuristic_node.py semantic_labeler.py \
             low_object_clusterer.py; do
        docker cp "$REPO/scripts/nodes/$f" $C:/tmp/trav/$f || die "copy $f"
    done
    local tmp
    tmp="$(mktemp)"
    python3 "$REPO/scripts/tools/world_objects.py" "$REPO/gazebo/worlds/$WORLD.json" "$tmp" >/dev/null || die "world_objects.py"
    docker cp "$tmp" $C:/tmp/trav/${WORLD}_objects.json
    rm -f "$tmp"
    docker exec $C bash -c "sed -i 's/\r$//' /tmp/trav/*.sh"
}

yolo_up() {
    docker rm -f go2-yolo >/dev/null 2>&1
    docker run -d --name go2-yolo --network host --entrypoint bash \
        -v "$REPO/scripts/nodes:/nodes:ro" \
        -v "$HOME/files/go2-nav2-wifi/ws/log/3d/cyclonedds.xml:/cyclonedds.xml:ro" \
        go2-3d:local -c 'source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///cyclonedds.xml ROS_DOMAIN_ID=64; exec python3 /nodes/yolo_json_node.py' >/dev/null \
        || die "YOLO container failed to start"
}

# The sim bridge keeps the last /gait/command: start from NORMAL.
gait_reset() {
    docker exec $C bash -c "$ROS; timeout 30 ros2 topic pub -w 1 --once /gait/command std_msgs/msg/String '{data: NORMAL}'" \
        >/dev/null 2>&1 || say "WARNING: gait reset to NORMAL failed"
}

stack_down() {
    docker exec $C bash -c '
        f=/tmp/trav/stack.pids
        [ -f "$f" ] || exit 0
        while read -r p; do kill -TERM -- "-$p" 2>/dev/null; done < "$f"
        sleep 2
        while read -r p; do kill -KILL -- "-$p" 2>/dev/null; done < "$f"
        rm -f "$f"' 2>/dev/null
    docker rm -f go2-yolo go2-rviz >/dev/null 2>&1
}

cmd_up() {
    : "${DISPLAY:?RViz needs a desktop session: run this in a WSL terminal on the desktop}"
    sim_up
    running $C || docker start $C >/dev/null || die "docker start $C failed"
    stack_down
    copy_files
    gait_reset
    yolo_up
    docker exec $C rm -rf /tmp/trav_out/lab_semantic
    local gw
    gw="$(ip route | awk '/default/ {print $3; exit}')"
    curl -s -m 5 -o /dev/null "http://$gw:11434/api/tags" \
        || say "WARNING: Ollama not reachable at $gw:11434; the VLM labeller will have no labels"
    say "starting Nav2 + traversability + semantic stack"
    docker exec -d -e INTERACTIVE=1 -e WORLD_JSON=/tmp/trav/${WORLD}_objects.json \
        -e ONLY_CLASSES="$(python3 -c "import sys; sys.path.insert(0, '$REPO/scripts/tools'); import world_objects; print(world_objects.STAND_IN)")" \
        $C bash /tmp/trav/run_trial.sh $MODE /tmp/trav_out/lab_semantic
    for _ in $(seq 1 60); do
        docker exec $C test -f /tmp/trav_out/lab_semantic/ready && break
        sleep 2
    done
    docker exec $C test -f /tmp/trav_out/lab_semantic/ready || die "stack not ready after 120 s: docker exec $C ls /tmp/trav_out/lab_semantic"
    grep -q . <(docker exec $C bash -c "$ROS; timeout 10 ros2 node list" | grep -E "controller_server|planner_server") \
        || die "Nav2 nodes not found: docker exec $C tail /tmp/trav_out/lab_semantic/controller.log"

    # RViz in its own container: go2-lidar3d has rviz2 but no X socket mounted.
    say "starting RViz"
    xhost +SI:localuser:root >/dev/null 2>&1 || true
    docker rm -f go2-rviz >/dev/null 2>&1
    docker run -d --name go2-rviz --network host --ipc host --volumes-from $C \
        -v /tmp/.X11-unix:/tmp/.X11-unix -v "$HERE/lab_semantic.rviz:/lab_semantic.rviz:ro" \
        -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 \
        --entrypoint bash go2-lidar3d:local -c "$ROS; source /ws/install/setup.bash; exec rviz2 -d /lab_semantic.rviz" >/dev/null \
        || die "RViz container failed to start"
    sleep 5
    running go2-rviz || die "RViz exited: docker logs go2-rviz"
    cat <<EOF

[lab_semantic] up. In RViz: "2D Goal Pose" (top toolbar), click and drag on the map.
  Spawn (-4.5, 0) in the corridor. Rooms: north y > 1 (office, lab, conference),
  south y < -1 (break room, storage, workshop, office); doors are 1.0-1.2 m.
  Gait and labels:  tests/traversability/lab_semantic.sh watch
  Stop:             tests/traversability/lab_semantic.sh down
  Note: lab_office furniture is all >= 0.30 m (no step-over items), so expect
  AVOID labels and CAREFUL/STOP gait near furniture, not HIGH_STEP.
EOF
}

cmd_watch() {
    docker exec $C bash -c "$ROS
        ros2 topic echo --field data /gait/command std_msgs/msg/String | sed -u 's/^/[gait] /' &
        ros2 topic echo --field data /semantic/labels_json std_msgs/msg/String | sed -u 's/^/[vlm]  /' &
        wait"
}

cmd_down() {
    stack_down
    gait_reset
    mkdir -p "$OUT"
    local d
    d="$OUT/$(date +%Y%m%d_%H%M%S)"
    docker cp $C:/tmp/trav_out/lab_semantic "$d" >/dev/null 2>&1 && say "logs and recordings: $d"
    say "stack stopped (sim still running: ./gazebo/run_sim.sh down to stop it)"
}

case "${1:-up}" in
    up) cmd_up ;;
    watch) cmd_watch ;;
    down) cmd_down ;;
    *) echo "usage: $0 up|watch|down" >&2; exit 2 ;;
esac
