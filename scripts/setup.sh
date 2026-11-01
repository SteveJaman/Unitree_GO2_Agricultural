#!/usr/bin/env bash
# setup.sh: set up a fresh workstation, or check an existing one (safe to rerun).
#
# Checks each piece and installs only what is missing:
#   1. Docker (docker.io + docker-compose-v2, you in the docker group)
#   2. The upstream stack, cloned to ~/files/go2-nav2-wifi
#   3. The lab fixes on it (scripts/tools/apply-lab-nav-fixes.sh)
#   4. Host Python packages: cyclonedds, ruckig, ikpy
#      (--with-extras: also ultralytics and pyrealsense2; YOLO runs inside the
#      go2-3d image and the RealSense on the Jetson, so the host rarely needs them)
#   5. Ollama and the llava:7b model (~4.7 GB)
#   6. The Docker images go2-lidar3d:local and go2-3d:local (first build: long)
#
# Not covered (see docs/setup/setup.md): passwordless SSH to the robot, the arm
# calibration, the UDP buffer sysctl, the NVIDIA driver.
#
# Usage:
#   ./scripts/setup.sh                 # check, install what is missing
#   ./scripts/setup.sh --with-extras   # also ultralytics and pyrealsense2
#   NAV2_WIFI_DIR=/other/path ./scripts/setup.sh
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
NAV2_WIFI_DIR="${NAV2_WIFI_DIR:-$HOME/files/go2-nav2-wifi}"
UPSTREAM_URL="https://github.com/dancher00/go2-nav2-wifi"
MODEL="${TWIN_VLM_MODEL:-llava:7b}"
PY_PACKAGES=(cyclonedds ruckig ikpy)
EXTRAS=0

for arg in "$@"; do
    case "${arg}" in
        --with-extras) EXTRAS=1 ;;
        -h|--help) awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; exit 0 ;;
        *) echo "Unknown option: ${arg} (see --help)" >&2; exit 2 ;;
    esac
done
[ "${EXTRAS}" = 1 ] && PY_PACKAGES+=(ultralytics pyrealsense2)

if [ "${EUID}" -eq 0 ]; then
    echo "Run this as your normal user, not with sudo: it asks for sudo itself where needed." >&2
    exit 1
fi

PROBLEMS=()
problem() { echo "[!] $1"; PROBLEMS+=("$1"); }

apt_install() {
    sudo apt-get update -qq && sudo apt-get install -y "$@"
}

# git and curl are needed for the clone and the Ollama installer.
missing=()
for tool in git curl; do command -v "${tool}" >/dev/null 2>&1 || missing+=("${tool}"); done
if [ "${#missing[@]}" -gt 0 ]; then
    echo "[...] Installing ${missing[*]}..."
    apt_install "${missing[@]}" || problem "could not install ${missing[*]}"
fi

# 1. Docker -------------------------------------------------------------------
DOCKER_OK=0
if command -v docker >/dev/null 2>&1; then
    echo "[✓] Docker installed"
    if ! docker compose version >/dev/null 2>&1; then
        echo "[...] Installing docker-compose-v2 (the compose plugin)..."
        apt_install docker-compose-v2 || problem "could not install docker-compose-v2"
    fi
else
    echo "[...] Installing Docker..."
    if apt_install docker.io docker-compose-v2; then
        sudo usermod -aG docker "${USER}"
        echo "    You were added to the docker group: log out and back in (or run 'newgrp docker'),"
        echo "    then run ./scripts/setup.sh again to finish the Docker images."
    else
        problem "could not install Docker"
    fi
fi
if command -v docker >/dev/null 2>&1; then
    if docker info >/dev/null 2>&1; then
        DOCKER_OK=1
    elif id -nG "${USER}" | grep -qw docker; then
        problem "Docker is installed but not usable yet: log out and back in (or 'newgrp docker'), or start it: sudo systemctl start docker"
    else
        echo "[...] Adding ${USER} to the docker group..."
        sudo usermod -aG docker "${USER}" || true
        problem "Docker needs a new login for the docker group: log out and back in, then rerun this script"
    fi
fi

# 2. Upstream repo ------------------------------------------------------------
if [ -f "${NAV2_WIFI_DIR}/mapping.sh" ]; then
    echo "[✓] Upstream repo found"
elif [ -e "${NAV2_WIFI_DIR}" ]; then
    problem "${NAV2_WIFI_DIR} exists but is not a go2-nav2-wifi clone (no mapping.sh): move it away and rerun"
else
    echo "[...] Installing upstream repo (cloning ${UPSTREAM_URL})..."
    mkdir -p "$(dirname "${NAV2_WIFI_DIR}")"
    git clone "${UPSTREAM_URL}" "${NAV2_WIFI_DIR}" || problem "could not clone ${UPSTREAM_URL}"
fi

# 3. Lab patches --------------------------------------------------------------
# apply-lab-nav-fixes.sh marks the lidar mount it fixes with "Lab fix:" and adds
# the velocity smoother to the controller launch file.
if [ -f "${NAV2_WIFI_DIR}/mapping.sh" ]; then
    if grep -qs "Lab fix:" "${NAV2_WIFI_DIR}/go2_nav2/config/lidar3d_robot_viz.yaml" &&
       grep -qs "velocity_smoother" "${NAV2_WIFI_DIR}/go2_nav2/launch/lidar3d_controller.launch.py"; then
        echo "[✓] Lab patches applied"
    else
        echo "[...] Installing lab patches (apply-lab-nav-fixes.sh)..."
        (cd "${NAV2_WIFI_DIR}" && bash "${REPO_DIR}/scripts/tools/apply-lab-nav-fixes.sh") \
            || problem "the lab patches did not apply (see the message above)"
    fi
else
    problem "lab patches skipped: no upstream repo"
fi

# 4. Python packages ----------------------------------------------------------
missing=()
for pkg in "${PY_PACKAGES[@]}"; do
    pip3 show "${pkg}" >/dev/null 2>&1 || missing+=("${pkg}")
done
if [ "${#missing[@]}" -eq 0 ]; then
    echo "[✓] Python packages present (${PY_PACKAGES[*]})"
else
    if ! command -v pip3 >/dev/null 2>&1; then
        echo "[...] Installing python3-pip..."
        apt_install python3-pip || problem "could not install python3-pip"
    fi
    echo "[...] Installing Python packages: ${missing[*]}..."
    python3 -m pip install --user "${missing[@]}" || problem "pip could not install: ${missing[*]}"
fi
if [ "${EXTRAS}" = 0 ]; then
    echo "    (ultralytics, pyrealsense2: not needed on this computer; --with-extras installs them)"
fi

# 5. Ollama and the model -----------------------------------------------------
if ! command -v ollama >/dev/null 2>&1; then
    echo "[...] Installing Ollama..."
    curl -fsSL https://ollama.com/install.sh | sh || problem "could not install Ollama"
fi
if command -v ollama >/dev/null 2>&1; then
    if ! ollama list >/dev/null 2>&1; then   # server not running (no systemd, e.g. WSL)
        nohup ollama serve >/tmp/ollama-setup.log 2>&1 &
        for _ in $(seq 1 15); do ollama list >/dev/null 2>&1 && break; sleep 1; done
    fi
    if ollama list 2>/dev/null | awk 'NR > 1 { print $1 }' | grep -qx "${MODEL}"; then
        echo "[✓] Ollama and ${MODEL} ready"
    elif ollama list >/dev/null 2>&1; then
        echo "[...] Installing ${MODEL} (ollama pull, ~4.7 GB)..."
        ollama pull "${MODEL}" || problem "'ollama pull ${MODEL}' failed (offline?)"
    else
        problem "Ollama does not answer (log: /tmp/ollama-setup.log)"
    fi
fi

# 6. Docker images ------------------------------------------------------------
if [ "${DOCKER_OK}" = 1 ]; then
    if docker image inspect go2-lidar3d:local >/dev/null 2>&1; then
        lidar_ok=1
    elif [ -f "${NAV2_WIFI_DIR}/mapping.sh" ]; then
        # Build only: wifi_nav.sh creates the container on its first run, for
        # this computer's and the robot's current IPs.
        echo "[...] Installing go2-lidar3d:local (Docker build, ~15 min the first time)..."
        GO2_UID="$(id -u)"; GO2_GID="$(id -g)"; export GO2_UID GO2_GID
        if ! docker image inspect go2-humble:local >/dev/null 2>&1; then
            docker build -t go2-humble:local -f "${NAV2_WIFI_DIR}/docker/Dockerfile" "${NAV2_WIFI_DIR}/docker" \
                || problem "could not build go2-humble:local"
        fi
        if docker compose -p go2-lidar3d -f "${NAV2_WIFI_DIR}/docker/docker-compose.yml" \
                -f "${NAV2_WIFI_DIR}/docker/docker-compose.lidar3d.yml" build; then
            lidar_ok=1
        else
            problem "could not build go2-lidar3d:local"
        fi
    else
        problem "go2-lidar3d:local skipped: no upstream repo"
    fi
    if docker image inspect go2-3d:local >/dev/null 2>&1; then
        threed_ok=1
    else
        echo "[...] Installing go2-3d:local (wifi_3d_nav.sh --build, ~20 min the first time)..."
        if "${REPO_DIR}/scripts/runtime/wifi_3d_nav.sh" --build; then
            threed_ok=1
        else
            problem "could not build go2-3d:local"
        fi
    fi
    if [ "${lidar_ok:-0}" = 1 ] && [ "${threed_ok:-0}" = 1 ]; then
        echo "[✓] Docker images built"
    fi
else
    problem "Docker images skipped: Docker is not usable yet"
fi

# Summary ---------------------------------------------------------------------
echo
if [ "${#PROBLEMS[@]}" -gt 0 ]; then
    echo "Setup check complete, with ${#PROBLEMS[@]} item(s) to fix:"
    printf '  - %s\n' "${PROBLEMS[@]}"
    echo "Fix them and run ./scripts/setup.sh again. Details: docs/setup/setup.md"
    exit 1
fi
echo "Setup check complete. To run the robot: ./scripts/runtime/wifi_3d_nav.sh"
echo "Still by hand, once (docs/setup/setup.md): passwordless SSH to the robot, arm calibration, UDP buffers."
