#!/bin/bash
# Alternating baseline / traversability trials against a running sim. Run in WSL:
#   GO2_HOST_IP=127.0.0.1 SIM_ARM=0 SIM_WORLD=cluttered_items SIM_SPAWN="0 0 0" \
#     SIM_CAMERA=1 SIM_CONTACTS=1 SIM_GAIT=1 ./gazebo/run_sim.sh up
#   tests/traversability/run_all.sh RESULTS_DIR "1 2 3 4 5"
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
RES="${1:-$HERE/results}"
RUNS="${2:-1 2 3 4 5}"
WORLD=cluttered_items
GZ='export GZ_PARTITION=go2sim GZ_IP=127.0.0.1'
mkdir -p "$RES"

docker exec go2-lidar3d mkdir -p /tmp/trav
for f in planner.yaml controller_baseline.yaml controller_traversability.yaml run_trial.sh \
         start_extra_traversability.sh save_frames.py; do
    docker cp "$HERE/$f" go2-lidar3d:/tmp/trav/$f
done
docker cp "$REPO/tests/nav_trial_runner.py" go2-lidar3d:/tmp/trav/nav_trial_runner.py
for f in object_box_fusion.py sim_box_detector.py gait_supervisor.py; do
    docker cp "$REPO/scripts/nodes/$f" go2-lidar3d:/tmp/trav/$f
done
docker cp "$REPO/gazebo/worlds/$WORLD.json" go2-lidar3d:/tmp/trav/$WORLD.json
docker exec go2-lidar3d bash -c "sed -i 's/\r$//' /tmp/trav/*.sh"

yolo_up() {
    docker rm -f go2-yolo >/dev/null 2>&1
    docker run -d --name go2-yolo --network host --entrypoint bash \
        -v "$REPO/scripts/nodes:/nodes:ro" \
        -v "$HOME/files/go2-nav2-wifi/ws/log/3d/cyclonedds.xml:/cyclonedds.xml:ro" \
        go2-3d:local -c 'source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///cyclonedds.xml ROS_DOMAIN_ID=64; exec python3 /nodes/yolo_json_node.py' >/dev/null
}

sim_time() {
    docker exec go2-sim bash -c "$GZ; timeout 3 gz topic -e -n 1 -t /stats" 2>/dev/null \
        | awk '/^sim_time/{f=1} f&&/ sec:/{s=$2} f&&/nsec:/{print s+$2/1e9; exit}'
}

trial() {
    local mode=$1 name=$2 dir="$RES/$2"
    [ -f "$dir/legs.json" ] && { echo "skip $name"; return; }
    rm -rf "$dir"; mkdir -p "$dir"
    docker exec go2-sim bash -c "$GZ; gz service -s /world/$WORLD/set_pose --reqtype gz.msgs.Pose \
        --reptype gz.msgs.Boolean --timeout 3000 \
        --req 'name: \"go2\", position: {x: 0, y: 0, z: 0.2984}, orientation: {w: 1}'" >/dev/null
    sleep 2
    if [ "$mode" = traversability ]; then yolo_up; sleep 12; else docker rm -f go2-yolo >/dev/null 2>&1; fi
    docker exec go2-sim rm -f /tmp/contacts.jsonl
    docker exec -d go2-sim bash -c "$GZ; exec python3 /sim/sim/contact_monitor.py --out /tmp/contacts.jsonl --topics \$(cat /tmp/sim-gen/contact_topics.txt)"
    docker exec go2-lidar3d rm -rf /tmp/trav_out/$name
    local s0 w0 s1 w1
    s0="$(sim_time)"; w0="$(date +%s.%N)"
    timeout 560 docker exec go2-lidar3d bash /tmp/trav/run_trial.sh "$mode" /tmp/trav_out/$name 2>&1 | tail -1 | cut -c1-200
    s1="$(sim_time)"; w1="$(date +%s.%N)"
    docker exec go2-sim pkill -f contact_monitor
    docker rm -f go2-yolo >/dev/null 2>&1
    docker cp go2-lidar3d:/tmp/trav_out/$name/. "$dir/" >/dev/null
    docker cp go2-sim:/tmp/contacts.jsonl "$dir/contacts.jsonl" 2>/dev/null || : > "$dir/contacts.jsonl"
    awk -v a="$s0" -v b="$s1" -v c="$w0" -v d="$w1" 'BEGIN{printf "%.3f\n",(b-a)/(d-c)}' > "$dir/rtf.txt"
    echo "$name rtf=$(cat "$dir/rtf.txt") contacts=$(wc -l < "$dir/contacts.jsonl")"
}

for i in $RUNS; do
    trial baseline "baseline$i"
    trial traversability "traversability$i"
done
