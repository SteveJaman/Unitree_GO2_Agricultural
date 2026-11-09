#!/bin/bash
# Alternating trials against a running sim. Run in WSL:
#   GO2_HOST_IP=127.0.0.1 SIM_ARM=0 SIM_WORLD=cluttered_items SIM_SPAWN="0 0 0" \
#     SIM_CAMERA=1 SIM_CONTACTS=1 SIM_GAIT=1 ./gazebo/run_sim.sh up
#   tests/traversability/run_all.sh RESULTS_DIR "1 2 3 4 5"
# Environment:
#   MODES="baseline traversability"  modes, alternated within each run number;
#                 traversability_semantic adds the VLM labeller and SemanticCostmapLayer;
#                 *_mppi* modes run controller_server in the go2-3d image (MPPI and
#                 legged_mppi_critic, built in $MPPI_WS) instead of go2-lidar3d
#   MPPI_WS=$HOME/files/ws_mppi   colcon workspace built in go2-3d:local
#   SEED=1234     random spawn and goals (route.py); run i uses seed SEED + i - 1.
#                 Unset: the fixed (0,0) -> (5,0) -> (10,0) -> (0,0) sequence
#   TIMEOUT=120   seconds per leg
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
RES="${1:-$HERE/results}"
RUNS="${2:-1 2 3 4 5}"
MODES="${MODES:-baseline traversability}"
SEED="${SEED:-}"
TIMEOUT="${TIMEOUT:-120}"
WORLD=cluttered_items
GZ='export GZ_PARTITION=go2sim GZ_IP=127.0.0.1'
mkdir -p "$RES"

docker exec go2-lidar3d mkdir -p /tmp/trav
for f in planner.yaml controller_baseline.yaml controller_traversability.yaml run_trial.sh route.py \
         start_extra_traversability.sh save_frames.py record_semantic.py save_costmap.py record_json_topic.py; do
    docker cp "$HERE/$f" go2-lidar3d:/tmp/trav/$f
done
for f in "$HERE"/controller_traversability_*.yaml "$HERE"/planner_*.yaml "$HERE"/start_extra_traversability_*.sh; do
    [ -f "$f" ] && docker cp "$f" go2-lidar3d:/tmp/trav/
done
docker cp "$REPO/tests/nav_trial_runner.py" go2-lidar3d:/tmp/trav/nav_trial_runner.py
docker cp "$REPO/scripts/config/ollama.yaml" go2-lidar3d:/tmp/trav/ollama.yaml
for f in object_box_fusion.py sim_box_detector.py gait_supervisor.py vlm_heuristic_node.py semantic_labeler.py \
         low_object_clusterer.py; do
    docker cp "$REPO/scripts/nodes/$f" go2-lidar3d:/tmp/trav/$f
done
docker cp "$REPO/gazebo/worlds/$WORLD.json" go2-lidar3d:/tmp/trav/$WORLD.json
# traversability_nobackstop: same local costmap as traversability, gait backstop off.
docker exec go2-lidar3d cp /tmp/trav/controller_traversability.yaml /tmp/trav/controller_traversability_nobackstop.yaml
# traversability_histfloor: same, with the height band layer's floor from a histogram.
docker exec go2-lidar3d bash -c "sed 's/^        ground_tolerance: 0.02$/        ground_tolerance: 0.02\n        floor_estimation: histogram/' \
    /tmp/trav/controller_traversability.yaml > /tmp/trav/controller_traversability_histfloor.yaml"
docker exec go2-lidar3d bash -c "sed -i 's/\r$//' /tmp/trav/*.sh"

yolo_up() {
    docker rm -f go2-yolo >/dev/null 2>&1
    docker run -d --name go2-yolo --network host --entrypoint bash \
        -v "$REPO/scripts/nodes:/nodes:ro" \
        -v "$HOME/files/go2-nav2-wifi/ws/log/3d/cyclonedds.xml:/cyclonedds.xml:ro" \
        go2-3d:local -c 'source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///cyclonedds.xml ROS_DOMAIN_ID=64; exec python3 /nodes/yolo_json_node.py' >/dev/null
}

# MPPI controller_server in go2-3d (MPPI is not installed in go2-lidar3d).
mppi_up() {
    docker rm -f go2-mppi >/dev/null 2>&1
    docker run -d --name go2-mppi --network host --entrypoint bash \
        -v "${MPPI_WS:-$HOME/files/ws_mppi}:/ws_mppi:ro" -v "$HERE:/trav:ro" \
        -v "$HOME/files/go2-nav2-wifi/ws/log/3d/cyclonedds.xml:/cyclonedds.xml:ro" \
        go2-3d:local -c "source /opt/ros/humble/setup.bash; source /ws_mppi/install/setup.bash; \
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///cyclonedds.xml ROS_DOMAIN_ID=64; \
            exec ros2 run nav2_controller controller_server --ros-args \
            --params-file /trav/controller_$1.yaml -r cmd_vel:=cmd_vel_nav" >/dev/null
}

# The sim bridge keeps the last /gait/command it got. Without this, a trial
# inherits the previous trial's gait: baseline runs no gait supervisor and walked
# whole trials with the 0.12 m lift left by traversability_semantic.
gait_reset() {
    docker exec go2-lidar3d bash -c 'source /opt/ros/humble/setup.bash
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///ws/log/3d/cyclonedds.xml ROS_DOMAIN_ID=64
        timeout 30 ros2 topic pub -w 1 --once /gait/command std_msgs/msg/String "{data: NORMAL}"' >/dev/null 2>&1 \
        || echo "WARNING: gait reset to NORMAL failed" >&2
}

sim_time() {
    docker exec go2-sim bash -c "$GZ; timeout 3 gz topic -e -n 1 -t /stats" 2>/dev/null \
        | awk '/^sim_time/{f=1} f&&/ sec:/{s=$2} f&&/nsec:/{print s+$2/1e9; exit}'
}

trial() {
    local mode=$1 name=$2 run=$3 dir="$RES/$2" extra=() x=0 y=0 yaw=0
    [ -f "$dir/legs.json" ] && { echo "skip $name"; return; }
    rm -rf "$dir"; mkdir -p "$dir"
    if [ -n "$SEED" ]; then
        local seed=$((SEED + run - 1))
        read -r x y yaw < <(python3 "$HERE/route.py" "$REPO/gazebo/worlds/$WORLD.json" "$seed" \
            | python3 -c 'import json,sys; s=json.load(sys.stdin)["spawn"]; print(*s)')
        extra=(--randomize --seed "$seed")
    fi
    local qz qw
    qz=$(python3 -c "import math; print(math.sin($yaw / 2))"); qw=$(python3 -c "import math; print(math.cos($yaw / 2))")
    docker exec go2-sim bash -c "$GZ; gz service -s /world/$WORLD/set_pose --reqtype gz.msgs.Pose \
        --reptype gz.msgs.Boolean --timeout 3000 \
        --req 'name: \"go2\", position: {x: $x, y: $y, z: 0.2984}, orientation: {z: $qz, w: $qw}'" >/dev/null
    sleep 2
    gait_reset
    case "$mode" in traversability*) yolo_up; sleep 12 ;; *) docker rm -f go2-yolo >/dev/null 2>&1 ;; esac
    local ext=0
    case "$mode" in *mppi*) mppi_up "$mode"; ext=1 ;; esac
    docker exec go2-sim rm -f /tmp/contacts.jsonl /tmp/contact_monitor.pid
    docker exec -d go2-sim bash -c "echo \$\$ > /tmp/contact_monitor.pid; $GZ; exec python3 /sim/sim/contact_monitor.py --out /tmp/contacts.jsonl --topics \$(cat /tmp/sim-gen/contact_topics.txt)"
    docker exec go2-lidar3d rm -rf /tmp/trav_out/$name
    local s0 w0 s1 w1
    s0="$(sim_time)"; w0="$(date +%s.%N)"
    timeout 700 docker exec -e CONTROLLER_EXTERNAL=$ext go2-lidar3d bash /tmp/trav/run_trial.sh "$mode" /tmp/trav_out/$name \
        --timeout "$TIMEOUT" "${extra[@]}" 2>&1 | tail -1 | cut -c1-200
    s1="$(sim_time)"; w1="$(date +%s.%N)"
    docker exec go2-sim bash -c 'kill $(cat /tmp/contact_monitor.pid) 2>/dev/null'
    docker rm -f go2-yolo >/dev/null 2>&1
    if [ "$ext" = 1 ]; then
        docker logs go2-mppi > "$dir/controller_mppi.log" 2>&1
        docker rm -f go2-mppi >/dev/null 2>&1
    fi
    docker cp go2-lidar3d:/tmp/trav_out/$name/. "$dir/" >/dev/null
    docker cp go2-sim:/tmp/contacts.jsonl "$dir/contacts.jsonl" 2>/dev/null || : > "$dir/contacts.jsonl"
    awk -v a="$s0" -v b="$s1" -v c="$w0" -v d="$w1" 'BEGIN{printf "%.3f\n",(b-a)/(d-c)}' > "$dir/rtf.txt"
    echo "$name rtf=$(cat "$dir/rtf.txt") contacts=$(wc -l < "$dir/contacts.jsonl")"
}

for i in $RUNS; do
    for m in $MODES; do
        trial "$m" "$m$i" "$i"
    done
done
