#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# traversability_nodes.sh: the traversability nodes of wifi_3d_nav.sh --semantic.
# Runs INSIDE go2-lidar3d (wifi_3d_nav.sh copies it and the nodes to /tmp/go2-trav):
#   traversability_nodes.sh start   start the nodes (each in its own process group)
#   traversability_nodes.sh stop    gait to NORMAL, then stop exactly those groups
#   traversability_nodes.sh status  which nodes are running
#
# Nodes (docs/simulation/traversability_sim.md, HARDWARE_HANDOFF.md):
#   low_object_clusterer.py  LiDAR-only low clusters -> /objects/low_clusters (gait backstop)
#   gait_supervisor.py       gait from boxes, costmap, clusters, labels -> /gait/command
#   with the camera path (TRAV_CAMERA=1; YOLO runs in its own container, see wifi_3d_nav.sh):
#   object_box_fusion.py     /objects/detections_json + LiDAR -> /objects/boxes_json
#   vlm_heuristic_node.py    VLM labels -> /semantic/obstacles (SemanticCostmapLayer)
#   sim_box_detector.py      --sim only: ground-truth stand-in for classes YOLO lacks
#
# Environment (set by wifi_3d_nav.sh, plus the session's DDS settings):
#   TRAV_SIM=0|1  TRAV_CAMERA=0|1  TRAV_VLM=0|1  TRAV_LOG=/ws/log/3d/traversability
#   TRAV_CAMERA_INFO=camera yaml (intrinsics)  TRAV_WORLD_JSON=sim object list  TRAV_STAND_IN=classes
#   TRAV_OLLAMA_URL=http://host:11434  TRAV_HW_CONFIG=hardware.yaml (--hardware)
#   GAIT_ENABLE_HARDWARE=true|false  GAIT_DRY_RUN=true|false  TRAV_SDK_IFACE=network interface
#   TRAV_HOST_UID / TRAV_HOST_GID: logs are handed to this user on stop
# The legs are commanded only with backend sport_client, enable_hardware_backend true
# and dry_run false (wifi_3d_nav.sh --semantic --hardware --send-gait).
# ---------------------------------------------------------------------------
set -u
D=/tmp/go2-trav
PIDFILE="${D}/nodes.pids"
ENVFILE="${D}/env.sh"
LOG="${TRAV_LOG:-/ws/log/3d/traversability}"

ros_env() {
    set +u   # the ROS setup scripts read unset variables
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    # shellcheck disable=SC1091
    source /ws/install/setup.bash 2>/dev/null
    set -u
    export PYTHONPATH="${D}:${PYTHONPATH:-}"
}

alive() { [ -f "${PIDFILE}" ] && while read -r p _; do kill -0 "$p" 2>/dev/null && return 0; done < "${PIDFILE}"; return 1; }

# bg NAME CMD...: own process group (setsid), "PID NAME" to PIDFILE, log LOG/NAME.log.
bg() {
    local name=$1; shift
    setsid "$@" > "${LOG}/${name}.log" 2>&1 &
    echo "$! ${name}" >> "${PIDFILE}"
}

# fx fy cx cy from a ROS camera_info yaml (camera_matrix.data).
intrinsics() {
    python3 - "$1" <<'EOF'
import sys, yaml
k = yaml.safe_load(open(sys.argv[1]))["camera_matrix"]["data"]
print(*(float(k[i]) for i in (0, 4, 2, 5)))   # float: the nodes declare these as doubles
EOF
}

# -p options for gait_supervisor from hardware.yaml's gait_supervisor section; the three
# gates (backend, enable_hardware_backend, dry_run) come from the flags, never the file.
hw_gait_args() {
    python3 - "$1" <<'EOF'
import sys, yaml
sec = (yaml.safe_load(open(sys.argv[1])) or {}).get("gait_supervisor", {}) or {}
if "ros__parameters" in sec:
    sec = sec["ros__parameters"] or {}
for k, v in sec.items():
    if k in ("backend", "enable_hardware_backend", "dry_run"):
        continue
    v = str(v).lower() if isinstance(v, bool) else v
    print(f"-p {k}:={v}")
EOF
}

do_stop() {
    [ -f "${ENVFILE}" ] && source "${ENVFILE}"
    if alive; then
        ros_env
        # NORMAL through the backend before the supervisor goes (sport_client: FootRaiseHeight
        # back to NORMAL when it is active; gazebo: a no-op).
        timeout 10 ros2 service call /gait_supervisor/override_normal std_srvs/srv/Trigger '{}' \
            > /dev/null 2>&1 || echo "[trav] WARN: /gait_supervisor/override_normal did not answer"
    fi
    if [ -f "${PIDFILE}" ]; then
        while read -r p _; do kill -TERM -- "-$p" 2>/dev/null; done < "${PIDFILE}"
        sleep 2
        while read -r p _; do kill -KILL -- "-$p" 2>/dev/null; done < "${PIDFILE}"
        rm -f "${PIDFILE}"
    fi
    # The sim's gait bridge keeps the last /gait/command: leave it at NORMAL.
    if [ "${TRAV_SIM:-0}" = 1 ]; then
        ros_env
        timeout 15 ros2 topic pub -w 1 --once /gait/command std_msgs/msg/String '{data: NORMAL}' \
            > /dev/null 2>&1 || echo "[trav] WARN: could not reset the sim gait to NORMAL"
    fi
    [ -n "${TRAV_HOST_UID:-}" ] && [ -d "${LOG}" ] && chown -R "${TRAV_HOST_UID}:${TRAV_HOST_GID:-${TRAV_HOST_UID}}" "${LOG}" 2>/dev/null
    echo "[trav] stopped"
}

do_start() {
    do_stop > /dev/null
    mkdir -p "${LOG}"
    # Saved for stop (it runs from a fresh docker exec).
    {
        for k in ROS_DOMAIN_ID RMW_IMPLEMENTATION CYCLONEDDS_URI ROS_LOCALHOST_ONLY TRAV_SIM TRAV_LOG \
                 TRAV_HOST_UID TRAV_HOST_GID; do
            [ -n "${!k:-}" ] && printf 'export %s=%q\n' "$k" "${!k}"
        done
    } > "${ENVFILE}"
    ros_env
    : > "${PIDFILE}"
    local backend="gazebo" fx fy cx cy gait_args=() cam_args=()
    [ "${TRAV_SIM:-0}" = 1 ] || backend="robot"

    # Step-over height: TRAV_STEP_OVER from wifi_3d_nav.sh (robot 0.03 m, measured; sim 0.12 m).
    local step="${TRAV_STEP_OVER:-0.12}"
    # Robot: ignore floor below the step-over height (the dog walks over it anyway; at
    # 1.2 cm the real LiDAR's floor read as a 0.67 m-wide object ahead) and tile long
    # clusters so a cabinet line beside the dog is not a circle reaching into the corridor.
    local cluster_robot=()
    [ "${TRAV_SIM:-0}" = 1 ] || cluster_robot=(-p h_min_m:="${step}" -p tile_m:=0.24)
    bg clusterer python3 "${D}/low_object_clusterer.py" --ros-args -p dry_run:=false -p backend:="${backend}" \
        -p step_limit_m:="${step}" "${cluster_robot[@]}"

    if [ "${TRAV_CAMERA:-0}" = 1 ]; then
        read -r fx fy cx cy < <(intrinsics "${TRAV_CAMERA_INFO}") \
            || { echo "[trav] ERROR: no camera_matrix in ${TRAV_CAMERA_INFO}"; return 1; }
        cam_args=(-p "fx:=${fx}" -p "fy:=${fy}" -p "cx:=${cx}" -p "cy:=${cy}")
        if [ "${TRAV_SIM:-0}" = 1 ] && [ -n "${TRAV_WORLD_JSON:-}" ]; then
            bg detector python3 "${D}/sim_box_detector.py" --ros-args -p world:="${TRAV_WORLD_JSON}" \
                -p only_classes:="${TRAV_STAND_IN:-}"
        fi
        # Sim: the fixed floor of the n = 10 trials. Robot: the floor from a height
        # histogram (the body pitches and rolls; HARDWARE_HANDOFF.md).
        # Robot: forget objects that moved (in view, unmatched 3 s; any, unseen 60 s).
        local fusion_robot=()
        [ "${TRAV_SIM:-0}" = 1 ] || fusion_robot=(-p track_view_timeout_s:=3.0 -p track_max_age_s:=60.0)
        bg fusion python3 "${D}/object_box_fusion.py" --ros-args "${cam_args[@]}" \
            -p floor_estimation:="$([ "${TRAV_SIM:-0}" = 1 ] && echo fixed || echo histogram)" \
            -p step_over_height_max:="${step}" "${fusion_robot[@]}"
        if [ "${TRAV_VLM:-1}" = 1 ]; then
            bg vlm python3 "${D}/vlm_heuristic_node.py" --ros-args --params-file "${D}/ollama.yaml" "${cam_args[@]}" \
                -p ollama_url:="${TRAV_OLLAMA_URL}" -p dry_run:=false -p backend:="${backend}" \
                -p step_over_height_max:="${step}" ${TRAV_VLM_MODEL:+-p model:="${TRAV_VLM_MODEL}"}
            gait_args+=(-p use_semantic:=true)
        fi
    fi

    if [ "${GAIT_ENABLE_HARDWARE:-false}" = true ]; then
        # shellcheck disable=SC2207
        gait_args+=(-p backend:=sport_client -p enable_hardware_backend:=true -p dry_run:="${GAIT_DRY_RUN:-true}"
                    -p sdk_network_interface:="${TRAV_SDK_IFACE:-wlan0}" $(hw_gait_args "${TRAV_HW_CONFIG}"))
        python3 -c "from unitree_sdk2py.go2.sport.sport_client import SportClient as S; assert hasattr(S, 'FootRaiseHeight')" \
            2>/dev/null || echo "[trav] WARN: unitree_sdk2py with SportClient.FootRaiseHeight is not importable here;" \
                                "the sport_client backend will log [FATAL] and stay inactive (HARDWARE_HANDOFF.md check 1)"
    else
        # Sim: /gait/command drives the Gazebo legs. Robot without --hardware: the same
        # topic, which nothing on the dog listens to (gait decisions logged only).
        gait_args+=(-p backend:=gazebo -p dry_run:=true)
    fi
    # HIGH_STEP when an object + foot_margin (0.02) fits under the foot: clearance = step + margin.
    gait_args+=(-p foot_clearance:="$(python3 -c "print(float('${step}') + 0.02)")")
    bg gait python3 "${D}/gait_supervisor.py" --ros-args "${gait_args[@]}"

    sleep 6
    local dead=0 p name
    while read -r p name; do
        if ! kill -0 "$p" 2>/dev/null; then
            dead=1
            echo "[trav] ERROR: ${name} exited; last lines of ${LOG}/${name}.log:"
            tail -n 5 "${LOG}/${name}.log" | sed 's/^/    /'
        fi
    done < "${PIDFILE}"
    echo "[trav] running: $(awk '{print $2}' "${PIDFILE}" | tr '\n' ' ')(logs ${LOG})"
    return "${dead}"
}

case "${1:-}" in
    start)  do_start ;;
    stop)   do_stop ;;
    status) if [ -f "${PIDFILE}" ]; then
                while read -r p name; do
                    printf '%-10s %s\n' "${name}" "$(kill -0 "$p" 2>/dev/null && echo running || echo exited)"
                done < "${PIDFILE}"
            else
                echo "not started"
            fi ;;
    *)      echo "usage: $0 start|stop|status" >&2; exit 2 ;;
esac
