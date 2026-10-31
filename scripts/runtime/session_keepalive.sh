# shellcheck shell=bash
# shellcheck disable=SC2154  # mapping_pid, viz_pid, target, log_dir: set by mapping.sh
# ---------------------------------------------------------------------------
# session_keepalive.sh: sourced by go2-nav2-wifi's mapping.sh (hook added by
# scripts/tools/mapping_stop_timeouts.py; wifi_nav.sh sets GO2_KEEPALIVE).
#
# Upstream mapping.sh ends the whole run when the FIRST of its three parts
# ends (`wait -n` on SLAM, RViz/Nav2 and the motion bridge), so a Wi-Fi drop
# closed RViz, Nav2 and the dashboard with no explanation. Instead:
#
#   * RViz / Nav2 ends (you closed RViz): the session ends, as before.
#   * SLAM on the Jetson ends: the dog's position is gone, so the motion
#     bridge is stopped first (the dog cannot move) and the explorer paused;
#     then the cause and the fix are printed. RViz and the dashboard stay.
#   * The motion bridge ends: the dog cannot move; the explorer is paused,
#     the cause and fix printed.
#
# Either way the session waits for you: press Ctrl+C to end it, fix, restart.
# Uses mapping.sh's variables: mapping_pid viz_pid motion_pid log_dir target
# GO2_CONTAINER. go2_keepalive_wait's status becomes mapping.sh's result.
# ---------------------------------------------------------------------------

go2_ka_say() { printf '%s\n' "$*" >&2; }

# Pause the explorer and stop any velocity, on the session's DDS setup.
go2_ka_halt_nav() {
    timeout -k 2 8 docker exec -u root \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///tmp/go2-wifi-cyclonedds.xml \
        "$GO2_CONTAINER" bash -c '
            source /opt/ros/humble/setup.bash; source /ws/install/setup.bash 2>/dev/null
            ros2 topic pub --once -w 0 /explore/resume std_msgs/msg/Bool "{data: false}" >/dev/null 2>&1 &
            ros2 topic pub --once -w 0 /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 &
            wait' >/dev/null 2>&1 || true
}

# $1 = what ended (slam | motion), $2 = its exit status, $3 = its log.
go2_ka_explain() {
    local what="$1" rc="$2" log="$3" tail_txt cause fix robot
    tail_txt="$(tail -n 40 "$log" 2>/dev/null)"
    robot="${target#*@}"
    if grep -q "SENSOR CLOCK FAULT" <<< "$tail_txt"; then
        cause="The Jetson's clock jumped after the dog booted (it syncs to the internet ~1-2 min after power-on), so SLAM stopped forwarding data."
        fix="Wait ~2 minutes after powering on the dog, then start again. Check on the robot: timedatectl (System clock synchronized: yes)."
    elif grep -qiE "Address already in use|port 17999" <<< "$tail_txt"; then
        cause="A motion bridge from an earlier run still holds port 17999 on the robot."
        fix="./scripts/runtime/wifi_nav.sh --clean, then start again."
    elif grep -qiE "session (is )?locked|already running|owned by" <<< "$tail_txt"; then
        cause="Another session (an earlier run, or another laptop) still runs on the Jetson."
        fix="./scripts/runtime/wifi_nav.sh --clean, then start again."
    elif [ "$rc" = 255 ] || grep -qiE "Timeout, server not responding|Connection (to .* )?closed|client_loop|Broken pipe|Connection reset|No route to host" <<< "$tail_txt"; then
        cause="The Wi-Fi link to the dog dropped (SSH to ${robot} lost)."
        fix="Check the dog is still on Wi-Fi (ping ${robot}); keep it in range; if the Wi-Fi dongle sleeps: ./scripts/tools/fix_wifi_powersave.sh. Then start again."
    else
        cause="It exited with status ${rc}."
        fix="Read the log above (full log: ${log}), fix, then start again."
    fi
    go2_ka_say ""
    go2_ka_say "=================================================================="
    if [ "$what" = slam ]; then
        go2_ka_say " PROBLEM: SLAM on the dog stopped. The robot is HALTED (motion bridge"
        go2_ka_say "          stopped, explorer paused): its position is no longer known."
    else
        go2_ka_say " PROBLEM: the motion bridge on the dog stopped. The robot is HALTED"
        go2_ka_say "          (it cannot be driven); the explorer is paused."
    fi
    go2_ka_say " Cause : ${cause}"
    go2_ka_say " Fix   : ${fix}"
    go2_ka_say " Last lines of ${log}:"
    tail -n 8 "$log" 2>/dev/null | sed 's/^/     /' >&2
    go2_ka_say " RViz and the dashboard stay open so you can look around."
    go2_ka_say " Press Ctrl+C to end the session (the map is still saved)."
    go2_ka_say "=================================================================="
}

go2_keepalive_wait() {
    local slam_up=1 motion_up=0 rc
    [ -n "${motion_pid:-}" ] && motion_up=1
    while :; do
        if ! kill -0 "$viz_pid" 2>/dev/null; then
            wait "$viz_pid" 2>/dev/null; return $?   # RViz closed: end the run, as upstream
        fi
        if [ "$slam_up" = 1 ] && ! kill -0 "$mapping_pid" 2>/dev/null; then
            slam_up=0; wait "$mapping_pid" 2>/dev/null; rc=$?
            if [ "$motion_up" = 1 ]; then       # no position: the dog must not move
                kill -TERM "$motion_pid" 2>/dev/null || true
                wait "$motion_pid" 2>/dev/null || true
                motion_up=0
            fi
            go2_ka_halt_nav
            go2_ka_explain slam "$rc" "$log_dir/slam.log"
        fi
        if [ "$motion_up" = 1 ] && ! kill -0 "$motion_pid" 2>/dev/null; then
            motion_up=0; wait "$motion_pid" 2>/dev/null; rc=$?
            go2_ka_halt_nav
            go2_ka_explain motion "$rc" "$log_dir/motion.log"
        fi
        sleep 1
    done
}
