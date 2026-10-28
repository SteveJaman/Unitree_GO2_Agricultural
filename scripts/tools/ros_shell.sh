#!/usr/bin/env bash
# ros_shell.sh: open a ROS 2 shell in a container on the same DDS setup as
# the nav stack, so `ros2 topic list` shows its topics.
#
# A plain `docker exec -it go2-lidar3d bash` misses RMW_IMPLEMENTATION and
# CYCLONEDDS_URI and only sees /parameter_events and /rosout.
#
# Usage:
#   ros_shell.sh                      interactive shell in go2-lidar3d
#   ros_shell.sh go2-sim              interactive shell in go2-sim
#   ros_shell.sh go2-lidar3d ros2 topic hz /scan    run one command
#
# Environment overrides:
#   CYCLONEDDS_XML=/tmp/go2-wifi-cyclonedds.xml   fallback when no ROS process is running
set -uo pipefail

CONTAINER="${1:-go2-lidar3d}"
shift || true
XML="${CYCLONEDDS_XML:-/tmp/go2-wifi-cyclonedds.xml}"

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "Container '$CONTAINER' is not running. Running containers:" >&2
  docker ps --format '  {{.Names}}' >&2
  exit 1
fi

# Copy the DDS settings of a ROS process already running in the container
# (go2-sim uses /tmp/sim-gen/cyclonedds.xml and domain 64; the nav stack
# uses $XML), falling back to $XML when nothing is running yet.
RUNNING_ENV=$(docker exec "$CONTAINER" bash -c '
  for p in /proc/[0-9]*; do
    e=$(tr "\0" "\n" < "$p/environ" 2>/dev/null) || continue
    if grep -q "^CYCLONEDDS_URI=" <<<"$e"; then
      grep -E "^(CYCLONEDDS_URI|ROS_DOMAIN_ID)=" <<<"$e"; break
    fi
  done')
ENV=(-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp)
if [ -n "$RUNNING_ENV" ]; then
  while read -r kv; do ENV+=(-e "$kv"); done <<<"$RUNNING_ENV"
elif docker exec "$CONTAINER" test -f "$XML"; then
  ENV+=(-e CYCLONEDDS_URI="file://$XML")
fi

# Source ROS and the workspace, and restart the ros2 daemon so it does not
# serve a topic list cached under other DDS settings.
SETUP='source /opt/ros/humble/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
ros2 daemon stop >/dev/null 2>&1'

if [ $# -gt 0 ]; then
  exec docker exec "${ENV[@]}" "$CONTAINER" bash -c "$SETUP; $(printf '%q ' "$@")"
fi
# The sourced ROS environment is exported, so the interactive child keeps it.
exec docker exec -it "${ENV[@]}" "$CONTAINER" bash -c "$SETUP
echo \"ROS 2 shell in $CONTAINER (\${CYCLONEDDS_URI:-cyclone defaults})\"
exec bash -i"
