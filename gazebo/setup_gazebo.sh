#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# setup_gazebo.sh: one-time setup of the Go2 + D1 Gazebo simulation (VM).
#
#   1. Downloads the official Unitree models into gazebo/models/ (not in git):
#        go2_description  unitreerobotics/unitree_ros (BSD-3-Clause), pinned commit
#        d1_description   Unitree D1 gripper-version package, sha256-checked
#   2. Builds the go2-sim:local image (ROS 2 Humble + Gazebo Harmonic).
#   3. Validates the combined model with Gazebo's own URDF->SDF converter.
#
# Usage:
#   ./gazebo/setup_gazebo.sh              # everything
#   ./gazebo/setup_gazebo.sh --models     # downloads only
#   ./gazebo/setup_gazebo.sh --image      # image build + validation only
#   ./gazebo/setup_gazebo.sh --force      # re-download models
# ---------------------------------------------------------------------------
set -euo pipefail

GZ_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODELS="${GZ_DIR}/models"
IMAGE="${SIM_IMAGE:-go2-sim:local}"

UNITREE_ROS_URL="https://github.com/unitreerobotics/unitree_ros.git"
UNITREE_ROS_COMMIT="5994d4faef0a9cadd3287f8de0199a67eeb2a259"   # 2026-09-30
D1_URL="https://oss-global-cdn.unitree.com/static/90b2525be5d84531ab9814f48b2f86a7.zip"
D1_SHA256="76cbe7725b1640cf214a6aa97e27d995c33ad5e4a4e7f29d57ec7ff7346a7164"

DO_MODELS=1; DO_IMAGE=1; FORCE=0
for a in "$@"; do
    case "$a" in
        --models) DO_IMAGE=0 ;;
        --image)  DO_MODELS=0 ;;
        --force)  FORCE=1 ;;
        -h|--help) awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; exit 0 ;;
        *) echo "Unknown option: $a" >&2; exit 1 ;;
    esac
done

die() { echo "ERROR: $*" >&2; exit 1; }
step() { echo; echo "=== $* ==="; }

if [ "$(. /etc/os-release 2>/dev/null; echo "${VERSION_CODENAME:-}")" != jammy ]; then
    echo "WARN: built and documented for Ubuntu 22.04 (jammy); continuing anyway."
fi
for c in docker git curl python3 sha256sum; do
    command -v "$c" >/dev/null 2>&1 || die "$c not found"
done
docker ps >/dev/null 2>&1 || die "cannot talk to Docker (try: newgrp docker)"

if [ "$DO_MODELS" = 1 ]; then
    mkdir -p "$MODELS"

    step "Go2 description (unitree_ros @ ${UNITREE_ROS_COMMIT:0:7})"
    if [ -f "$MODELS/go2_description/urdf/go2_description.urdf" ] && [ "$FORCE" = 0 ]; then
        echo "present: $MODELS/go2_description"
    else
        tmp="$(mktemp -d)"
        git init -q "$tmp"
        git -C "$tmp" remote add origin "$UNITREE_ROS_URL"
        # Sparse rules written directly: `git sparse-checkout set` needs a commit on older git.
        git -C "$tmp" config core.sparseCheckout true
        printf '/robots/go2_description/\n/LICENSE\n' > "$tmp/.git/info/sparse-checkout"
        git -C "$tmp" fetch -q --depth 1 --filter=blob:none origin "$UNITREE_ROS_COMMIT"
        git -C "$tmp" checkout -q FETCH_HEAD
        [ -f "$tmp/robots/go2_description/urdf/go2_description.urdf" ] || die "go2_description missing in download"
        rm -rf "$MODELS/go2_description"
        cp -r "$tmp/robots/go2_description" "$MODELS/"
        cp "$tmp/LICENSE" "$MODELS/go2_description/LICENSE.unitree_ros"
        rm -rf "$tmp"
        echo "installed: $MODELS/go2_description"
    fi

    step "D1 description (Unitree, gripper version)"
    if [ -f "$MODELS/d1_description/urdf/d1_description.urdf" ] && [ "$FORCE" = 0 ]; then
        echo "present: $MODELS/d1_description"
    else
        tmp="$(mktemp -d)"
        curl -fsSL --retry 3 -o "$tmp/d1.zip" "$D1_URL"
        got="$(sha256sum "$tmp/d1.zip" | cut -d' ' -f1)"
        [ "$got" = "$D1_SHA256" ] || die "D1 download checksum mismatch (got $got). Unitree may have changed the file; check it, then update D1_SHA256."
        rm -rf "$MODELS/d1_description"
        python3 -c 'import sys, zipfile; zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])' "$tmp/d1.zip" "$MODELS"
        rm -rf "$tmp"
        [ -f "$MODELS/d1_description/urdf/d1_description.urdf" ] || die "d1_description missing in archive"
        echo "installed: $MODELS/d1_description"
    fi

    step "Model build check (host Python)"
    python3 "$GZ_DIR/sim/build_model.py" --out "$(mktemp -d)"
fi

if [ "$DO_IMAGE" = 1 ]; then
    [ -d "$MODELS/go2_description" ] && [ -d "$MODELS/d1_description" ] || die "models missing; run without --image first"

    step "Building ${IMAGE} (ROS 2 Humble + Gazebo Harmonic; first build takes a while)"
    docker build -t "$IMAGE" "$GZ_DIR/docker"

    step "Validating the model with Gazebo's URDF->SDF converter"
    docker run --rm -v "$GZ_DIR:/sim:ro" "$IMAGE" bash -c '
        set -e
        python3 /sim/sim/build_model.py --go2 /sim/models/go2_description --d1 /sim/models/d1_description \
            --mesh-root /sim/models --out /tmp/gen
        gz sdf -p /tmp/gen/go2_d1.urdf > /tmp/gen/go2_d1.sdf
        gz sdf -k /tmp/gen/go2_d1.sdf
        echo "sensors: $(grep -c "<sensor " /tmp/gen/go2_d1.sdf)  plugins: $(grep -c "<plugin " /tmp/gen/go2_d1.sdf)  revolute+prismatic joints: $(grep -cE "<joint name=\"[^\"]*\" type=\"(revolute|prismatic)\"" /tmp/gen/go2_d1.sdf)"
        gz sdf -k /sim/worlds/agri_field.sdf
        python3 -c "import gz.transport13, gz.msgs10, cyclonedds; print(\"python bindings: OK\")"
    '
fi

echo
echo "Setup complete. Next:"
echo "  ./gazebo/run_sim.sh up            # start Gazebo + the simulated robot and arm"
echo "  ./gazebo/test_sim.sh              # automated smoke test"
echo "  ./scripts/runtime/wifi_nav.sh --sim [--arm]   # the full stack against the simulator"
