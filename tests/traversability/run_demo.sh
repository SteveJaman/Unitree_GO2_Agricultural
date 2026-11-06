#!/bin/bash
# Film the Go2 walking across items, once with the low baseline step and once with
# the full pipeline. Run in WSL, with the sim up with SIM_CHASE=1 and SIM_GAIT=1:
#   tests/traversability/run_demo.sh OUT_DIR
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
OUT="${1:-/tmp/walk_demo}"
WORLD=cluttered_items
GZ='export GZ_PARTITION=go2sim GZ_IP=127.0.0.1'
ROUTE="1.0,-0.75 4.8,-0.72 5.2,0.15 6.2,0.15 6.4,0.55 7.7,0.55"   # south lane: the camera rides on the robot's right
mkdir -p "$OUT"

docker exec go2-lidar3d mkdir -p /tmp/trav
docker cp "$HERE/start_extra_traversability.sh" go2-lidar3d:/tmp/trav/
docker cp "$HERE/save_frames.py" go2-lidar3d:/tmp/trav/
for f in object_box_fusion.py sim_box_detector.py gait_supervisor.py; do
    docker cp "$REPO/scripts/nodes/$f" go2-lidar3d:/tmp/trav/$f
done
docker cp "$REPO/gazebo/worlds/$WORLD.json" go2-lidar3d:/tmp/trav/$WORLD.json
docker cp "$HERE/demo_stack.sh" go2-lidar3d:/tmp/trav/demo_stack.sh
docker exec go2-lidar3d bash -c "sed -i 's/\r$//' /tmp/trav/*.sh"

film() {
    local name=$1 mode=$2
    docker exec go2-sim bash -c "$GZ; gz service -s /world/$WORLD/set_pose --reqtype gz.msgs.Pose \
        --reptype gz.msgs.Boolean --timeout 3000 \
        --req 'name: \"go2\", position: {x: 0, y: 0, z: 0.2984}, orientation: {w: 1}'" >/dev/null
    sleep 2
    docker exec go2-sim rm -f /tmp/contacts.jsonl
    docker exec go2-sim bash -c 'rm -rf /tmp/chase; mkdir -p /tmp/chase'
    docker exec -d go2-sim bash -c "$GZ; exec python3 /sim/sim/contact_monitor.py --out /tmp/contacts.jsonl --topics \$(cat /tmp/sim-gen/contact_topics.txt)"
    docker exec -d go2-lidar3d bash /tmp/trav/demo_stack.sh "$mode" /tmp/trav_out/demo_$name
    if [ "$mode" = traversability ]; then
        docker rm -f go2-yolo >/dev/null 2>&1
        docker run -d --name go2-yolo --network host --entrypoint bash \
            -v "$REPO/scripts/nodes:/nodes:ro" \
            -v "$HOME/files/go2-nav2-wifi/ws/log/3d/cyclonedds.xml:/cyclonedds.xml:ro" \
            go2-3d:local -c 'source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///cyclonedds.xml ROS_DOMAIN_ID=64; exec python3 /nodes/yolo_json_node.py' >/dev/null
    fi
    sleep 22
    docker exec -d go2-sim bash -c "$GZ; exec timeout 400 gz topic -e -t /chase/image > /dev/null"
    sleep 2
    timeout 380 python3 "$HERE/drive_line.py" $ROUTE --speed 0.25 | tail -1
    sleep 2
    docker exec go2-sim pkill -f "gz topic" ; docker exec go2-sim pkill -f contact_monitor
    docker exec go2-lidar3d pkill -f demo_stack
    docker exec go2-lidar3d bash -c 'for n in lidar3d_robot_viz robot_state_publisher go2_joint_state_bridge go2_urdf_self_filter go2_odom_tf object_box_fusion gait_supervisor sim_box_detector save_frames feet_log; do pkill -f $n; done'
    docker rm -f go2-yolo >/dev/null 2>&1
    rm -rf "$OUT/$name"; mkdir -p "$OUT/$name"
    docker cp go2-sim:/tmp/chase/. "$OUT/$name/frames" >/dev/null
    docker cp go2-sim:/tmp/contacts.jsonl "$OUT/$name/contacts.jsonl" 2>/dev/null
    echo "$name frames=$(ls "$OUT/$name/frames" | wc -l) contacts=$(wc -l < "$OUT/$name/contacts.jsonl")"
}

film baseline baseline
film traversability traversability
