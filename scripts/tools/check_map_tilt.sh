#!/usr/bin/env bash
# check_map_tilt.sh: did Point-LIO keep the floor level during a run?
#
# Gyro-only Point-LIO has no gravity reference: a fast turn can tip its
# attitude estimate, and every scan after that lands in the map tilted
# (2026-10-29 18:19: 17.7 deg after a 230 deg walking turn at 25-28 deg/s).
# This fetches a run's map and trajectory from the robot, fits the floor in
# each tenth of the run (points are stored in scan order), and reports the
# angle against the first tenth, plus the fastest turn in the trajectory.
#
# Usage:
#   ./scripts/tools/check_map_tilt.sh                 # latest run on the robot
#   ./scripts/tools/check_map_tilt.sh run-20261029-181915-fnbeq78j
#   ./scripts/tools/check_map_tilt.sh ~/files/go2-nav2-wifi/ws/maps/lidar3d/run-...   # a local copy
#
# Environment overrides:
#   GO2_ROBOT_IP=...        robot address (default: ubuntu.local, else the cached one)
#   ROBOT_MAPS=/home/unitree/go2-nav2-lidar3d-onboard/ws/maps/lidar3d
#   TILT_MAX_DEG=3          PASS threshold (a level run stays within ~2 deg)
#   CONTAINER_NAME=go2-lidar3d  (provides numpy)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../runtime/go2_net.sh
source "${SCRIPT_DIR}/../runtime/go2_net.sh"

CONTAINER="${CONTAINER_NAME:-go2-lidar3d}"
ROBOT_MAPS="${ROBOT_MAPS:-/home/unitree/go2-nav2-lidar3d-onboard/ws/maps/lidar3d}"
TILT_MAX_DEG="${TILT_MAX_DEG:-3}"
TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"; docker exec "${CONTAINER}" rm -rf /tmp/map_tilt >/dev/null 2>&1 || true' EXIT

if [ -n "${1:-}" ] && [ -d "$1" ]; then
    # A run already copied to this computer.
    echo "Run: $1 (local)"
    cp "$1/PCD/scans.pcd" "$1/trajectory.tum" "${TMP}/" \
        || { echo "No PCD/scans.pcd or trajectory.tum in $1" >&2; exit 1; }
else
    ROBOT_IP="$(go2_known_robot_ip || true)"
    go2_valid_ip "${ROBOT_IP}" || { echo "No robot IP: set GO2_ROBOT_IP or run wifi_nav.sh once." >&2; exit 1; }
    ROBOT="${GO2_ROBOT_USER}@${ROBOT_IP}"
    RUN="${1:-$(ssh "${GO2_SSH_OPTS[@]}" -T -n "${ROBOT}" "ls -1t ${ROBOT_MAPS} | grep '^run-' | head -n1")}"
    [ -n "${RUN}" ] || { echo "No runs found in ${ROBOT}:${ROBOT_MAPS}" >&2; exit 1; }
    echo "Run: ${RUN} (${ROBOT})"
    scp -q "${GO2_SSH_OPTS[@]}" "${ROBOT}:${ROBOT_MAPS}/${RUN}/PCD/scans.pcd" \
        "${ROBOT}:${ROBOT_MAPS}/${RUN}/trajectory.tum" "${TMP}/" \
        || { echo "Could not copy scans.pcd / trajectory.tum (run still in progress, or only on this computer?)" >&2; exit 1; }
fi
docker exec "${CONTAINER}" mkdir -p /tmp/map_tilt
docker cp -q "${TMP}/scans.pcd" "${CONTAINER}:/tmp/map_tilt/scans.pcd"
docker cp -q "${TMP}/trajectory.tum" "${CONTAINER}:/tmp/map_tilt/trajectory.tum"

docker exec -i "${CONTAINER}" python3 - "${TILT_MAX_DEG}" <<'EOF'
import math, sys
import numpy as np

limit = float(sys.argv[1])
raw = open('/tmp/map_tilt/scans.pcd', 'rb').read()
head, _, body = raw.partition(b'DATA binary\n')
fields = next(l for l in head.decode().splitlines() if l.startswith('FIELDS')).split()[1:]
pts = np.frombuffer(body, dtype=np.float32).reshape(-1, len(fields))[:, :3]
rng = np.random.default_rng(0)

def floor_normal(p):
    """Largest plane by RANSAC, refined by SVD on its inliers."""
    best = None
    for _ in range(300):
        a, b, c = p[rng.choice(len(p), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-6:
            continue
        n /= np.linalg.norm(n)
        inl = np.abs((p - a) @ n) < 0.05
        if best is None or inl.sum() > best.sum():
            best = inl
    q = p[best] - p[best].mean(0)
    n = np.linalg.svd(q, full_matrices=False)[2][-1]
    return n if n[2] >= 0 else -n

parts = np.array_split(pts, 10)
ref = None
worst = 0.0
print('Floor angle against the first tenth of the run (points in scan order):')
for i, p in enumerate(parts):
    p = p[rng.choice(len(p), min(len(p), 30000), replace=False)]
    n = floor_normal(p)
    ref = n if ref is None else ref
    ang = math.degrees(math.acos(min(1.0, abs(float(n @ ref)))))
    worst = max(worst, ang)
    print(f'  {10 * i:3d}-{10 * i + 10:3d}%  {ang:5.1f} deg')

rows = [list(map(float, l.split())) for l in open('/tmp/map_tilt/trajectory.tum')
        if l.strip() and not l.startswith('#')]
def yaw(r):
    qx, qy, qz, qw = r[4:8]
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
fastest, prev = 0.0, None
for r in rows:
    if prev is None or r[0] - prev[0] >= 0.5:
        if prev is not None:
            d = (yaw(r) - yaw(prev) + math.pi) % (2 * math.pi) - math.pi
            fastest = max(fastest, abs(math.degrees(d)) / (r[0] - prev[0]))
        prev = r
print(f'Duration {rows[-1][0] - rows[0][0]:.0f} s, fastest turn {fastest:.0f} deg/s')
print(f'{"PASS" if worst <= limit else "FAIL"}: max floor tilt {worst:.1f} deg (limit {limit:g} deg)')
sys.exit(0 if worst <= limit else 1)
EOF
