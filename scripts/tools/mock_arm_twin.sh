#!/usr/bin/env bash
# mock_arm_twin.sh: the web dashboard with a fake D1 arm, no robot needed.
#
# Runs the go2-3d image with rosbridge (:9090), the dashboard (:3000) and
# cmu-legacy/mock_sim.py --arm --tf: the Go2 stands at the map origin, the
# arm sits on its back, and the dashboard's Orbit Scan button plays the same
# sweep as scripts/nodes/orbit_scan.py. ROS stays on its own domain, localhost
# only, so it never meets a real session.
#
# Usage:
#   ./scripts/tools/mock_arm_twin.sh        # then open http://localhost:3000
#
# Environment overrides:
#   TWIN_IMAGE=go2-3d:local  TWIN_WEB_PORT=3000  TWIN_ROSBRIDGE_PORT=9090
#   MOCK_ROS_DOMAIN_ID=88
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
IMAGE="${TWIN_IMAGE:-go2-3d:local}"
WEB_PORT="${TWIN_WEB_PORT:-3000}"
WS_PORT="${TWIN_ROSBRIDGE_PORT:-9090}"
DOMAIN="${MOCK_ROS_DOMAIN_ID:-88}"

docker image inspect "${IMAGE}" >/dev/null 2>&1 \
    || { echo "${IMAGE} not found: build it with ./scripts/runtime/wifi_3d_nav.sh --build" >&2; exit 1; }
for p in "${WEB_PORT}" "${WS_PORT}"; do
    if (exec 3<>"/dev/tcp/127.0.0.1/${p}") 2>/dev/null; then
        echo "Port ${p} is in use (a wifi_3d_nav.sh session?). Stop it first." >&2
        exit 1
    fi
done

echo "[mock] Dashboard with a fake D1 arm: http://localhost:${WEB_PORT}  (Orbit Scan button). Ctrl+C to stop."
exec docker run --rm -it --name go2-3d-mock-arm --network host --user "$(id -u):$(id -g)" \
    -e HOME=/tmp -e ROS_DOMAIN_ID="${DOMAIN}" -e ROS_LOCALHOST_ONLY=1 \
    -v "${REPO_DIR}/twin:/twin:ro" -v "${REPO_DIR}/cmu-legacy:/cmu-legacy:ro" \
    "${IMAGE}" bash -c '
        source /opt/ros/humble/setup.bash
        root=/opt/webapp
        [ /twin/webapp/dist/index.html -nt /opt/webapp/index.html ] && root=/twin/webapp/dist
        mkdir -p /tmp/view/maps /tmp/view/config && for f in "$root"/*; do ln -s "$f" /tmp/view/; done
        python3 -m http.server '"${WEB_PORT}"' --bind 0.0.0.0 --directory /tmp/view >/tmp/web.log 2>&1 &
        ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:='"${WS_PORT}"' >/tmp/rosbridge.log 2>&1 &
        trap "kill 0" EXIT
        exec python3 -u /cmu-legacy/mock_sim.py --arm --tf'
