# Setup and Everyday Use

[← Back to README](../README.md) · [Architecture](architecture.md) · [3D twin](3d_nav.md) · [D1 arm](d1arm.md) · [Troubleshooting](troubleshooting.md)

Everything needed on a new workstation, then the everyday commands for
navigation and the arm. The 3D digital twin has its own one-time steps
(image build, Ollama): [3d_nav.md → One-time setup](3d_nav.md#one-time-setup).

---

## Navigation — one-time setup (per workstation)

```bash
# 1. Clone the upstream stack and this repo
git clone https://github.com/dancher00/go2-nav2-wifi ~/files/go2-nav2-wifi
git clone https://github.com/SteveJaman/Unitree_GO2_Agricultural ~/files/Unitree_GO2_Agricultural

# 2. Apply lab fixes (RPP + velocity smoother + correct lidar mount),
#    see docs/LAB-NAV-FIXES.md. Run it from the go2-nav2-wifi clone:
cd ~/files/go2-nav2-wifi
bash ~/files/Unitree_GO2_Agricultural/scripts/tools/apply-lab-nav-fixes.sh

# 3. Set up passwordless SSH to the robot (discovers its IP; add
#    --robot-ip <IP> if discovery fails)
cd ~/files/Unitree_GO2_Agricultural
./scripts/runtime/wifi_nav.sh --setup-ssh

# The Docker container is created automatically on the first run of
# wifi_nav.sh, bound to the VM's current Wi-Fi IP.

# 4. (Optional) Install pre-commit hooks for linting
pip install pre-commit
pre-commit install
```

## 3D digital twin and AI — one-time setup

After the navigation setup above (needs internet):

```bash
curl -fsSL https://ollama.com/install.sh | sh   # Ollama, for the VLM
```

Raise the VM's UDP receive buffers once. The Ubuntu default (208 KB) is too
small for DDS over Wi-Fi: a camera frame is ~50 UDP packets and one dropped
packet loses the frame, so the camera arrives at a fraction of its 5 fps and
RTAB-Map's camera mode gets no data. `wifi_nav.sh` warns when they are too
small.

```bash
printf 'net.core.rmem_max=16777216\nnet.core.rmem_default=8388608\n' | sudo tee /etc/sysctl.d/60-go2-dds.conf
sudo sysctl --system
```

That is all. The first `./scripts/runtime/wifi_3d_nav.sh` builds the
`go2-3d` image (RTAB-Map, YOLO, web dashboard; ~3–4 GB) and pulls
`llava:7b` (~4.7 GB). It also starts `ollama serve` when it isn't running.

Every session: dog standing at the dock, then
`./scripts/runtime/wifi_3d_nav.sh` and open `http://<VM IP>:3000`.
Details: [3d_nav.md](3d_nav.md).

## Navigation — every session

```bash
# 1. Power on the dog and stand it up

# 2. Run:
cd ~/files/Unitree_GO2_Agricultural
./scripts/runtime/wifi_nav.sh

# 3. In RViz (opens automatically):
#    a. Wait ~12 s for the map to appear
#    b. "2D Pose Estimate" — click the dog's position
#       (not needed with wifi_3d_nav.sh: auto-localized at the dock)
#    c. Click "Auto Explore: START" in the Go2 Explore panel
```

That's it. The dog explores autonomously. Click **STOP** or Ctrl+C in the
terminal to end. The physical remote is the emergency stop.

## Navigation — what the script does automatically

1. Detects the VM's Wi-Fi IP on `enp0s3` and discovers the robot's IP (see [Network](architecture.md#network))
2. Kills stale processes on the VM **and** on the Jetson
3. Cleans up partial runs on the robot
4. Recreates the container if it was created for an old host or robot IP.
   Pins `numpy<1.25` inside it (prevents `ValueError: numpy.dtype size changed`).
   With no internet it warns and carries on.
5. Checks that the container's DDS config never references the arm interface
   (a config left over from an older session is just noted; the new session rewrites it)
6. Relay: with Jetson compute (the default, `GO2_LIDAR3D_COMPUTE=jetson`),
   `mapping.sh` starts the robot relay and motion bridge itself, so
   `wifi_nav.sh` doesn't start one. Otherwise it SSHes into the Jetson, kills
   any relay holding port 17999, starts `robot-relay-wifi.sh` in FULL mode
   with the current `GO2_HOST_IP`, and waits for "FULL mode confirmed on
   port 17999"
7. Launches Nav2 + RViz via `mapping.sh --3d --nav`
8. After 20 s: starts the frontier explorer and camera relay
9. On Ctrl+C: stops exploration, kills helpers, stops the Jetson relay, brings down Docker

## Arm — one-time setup

1. In VirtualBox, attach the arm's Ethernet to the VM as Adapter 2 (Bridged,
   adapter type **Intel PRO/1000**). It shows up as `enp0s8`. The Go2 Wi-Fi
   stays on `enp0s3`.
2. Give `enp0s8` the address `192.168.123.99/24`. This doesn't persist across
   reboots, so re-run it after a VM restart:

   ```bash
   sudo ip addr add 192.168.123.99/24 dev enp0s8
   sudo ip link set enp0s8 up
   ```

3. Install the Python DDS bindings on the VM (the arm scripts run on the host,
   not in the container): `pip3 install cyclonedds`
4. Check everything:

   ```bash
   ./scripts/runtime/start_arm.sh --check         # address + ping
   ARM_IFACE=enp0s8 ./scripts/tools/diag_arm.sh   # all PASS expected
   ```

Discovery is multicast. Don't use `--peer` (unicast): it breaks on this arm's
firmware. If the arm doesn't reply to ping, cold-boot the VM.

## Arm — calibration (once)

The folded pose lives in `FOLDED_ANGLES` in
[`scripts/nodes/d1_arm_calibration.py`](../scripts/nodes/d1_arm_calibration.py)
(degrees: joints 0–5, then the gripper).

```bash
# 1. With the motors released, fold the arm by hand onto the dog's back
# 2. Read the angles:
./scripts/runtime/start_arm.sh read
#    -> FOLDED_ANGLES = [ ... ]
# 3. Paste that line over FOLDED_ANGLES in scripts/nodes/d1_arm_calibration.py
```

Redo this whenever the mount changes.

To measure how far each joint's reported angle drifts from the commanded one,
run the offset calibration (the arm moves ±10° per joint, clear the workspace):

```bash
./scripts/runtime/start_arm.sh --calibrate --test-deg 10 --output cal.json --yes
```

## Arm — every session

The usual way is `./scripts/runtime/wifi_nav.sh --arm`. It checks that
`enp0s8` exists, has an IPv4 address, and doesn't share a subnet with
`enp0s3`. It then runs `start_arm.sh fold` in the background right before Nav2
launches, logging to `go2-nav2-wifi/ws/log/arm.log`. If the arm isn't ready,
navigation still starts and the arm is skipped with a warning.

The fold: power on and enable the motors (skipped when they already are), send
one funcode=2 multi-joint move to `FOLDED_ANGLES`, wait until the pose is
reached (±3°, up to 30 s), then **release the motors and power off**. After
that the arm is limp and rests on its mount; if the pose is not reached it
stays holding instead.

With the arm plugged into the Go2 instead of `enp0s8`, use
`./scripts/runtime/wifi_nav.sh --arm-wifi`: same fold, over Wi-Fi through the
robot's bridge ([d1arm.md](d1arm.md#arm-over-wi-fi-plugged-into-the-go2)).

To run the arm on its own:

```bash
# 1. Power the arm (24V DC) and confirm the RJ45 is connected

# 2. Fold into the calibrated resting pose (enable -> move -> release):
cd ~/files/Unitree_GO2_Agricultural
./scripts/runtime/start_arm.sh fold

# 3. Move one joint (joint index, degrees), then release:
./scripts/runtime/start_arm.sh enable sleep 2 joint 0 20 sleep 3 release
```

See [D1-T Arm Control](d1arm.md) for every command.

## Navigation — flags

| Flag | Effect |
|---|---|
| *(none)* | Explore + camera (default) |
| `--arm` | **Start everything:** also fold the D1 arm in the background (over `enp0s8`) |
| `--arm-wifi` | Same, with the arm plugged into the Go2: bridge on the robot, arm over Wi-Fi (see [d1arm.md](d1arm.md#arm-over-wi-fi-plugged-into-the-go2)) |
| `--omni` | **Full Go2 mobility:** MPPI walks backwards and sidesteps, omnidirectional planning. See [omni_nav.md](omni_nav.md) |
| `--no-recovery` | Turn off the stuck recovery (on by default: an aborted leg backs the dog up 0.3 m and retries) |
| `--sim` | Run against the Gazebo simulator instead of the robot (no discovery, SSH or relay); combine with `--arm` |
| `--no-explore` | Manual waypoint mode (2D Goal Pose only) — see [docs/tour.md](tour.md) |
| `--no-camera` | Skip the camera relay |
| `--nav` | Manual waypoint mode, camera on |
| `--plan` | Preview only — no motion, no explore, no camera |
| `--floor-filter` | Add the floor-plane filter |
| `--record` | Save input data for replay |
| `--no-relay` | Skip the auto relay startup (manage Jetson yourself) |
| `--clean` | Clean both ends and exit |
| `--check` | Verify prerequisites only, no launch |
| `--setup-ssh` | One-time passwordless SSH setup |
| `--host-ip <IP>` | Use this host IP instead of auto-detecting it on `enp0s3` |
| `--robot-ip <IP>` | Use this robot IP instead of discovering it (required if discovery fails) |
| `--iface <IFACE>` | Wi-Fi interface for Go2 traffic (default `enp0s3`; `enp0s8` is refused) |

## Navigation — runtime overrides (no file edits)

Every explorer parameter can be overridden with an env var:

```bash
EXPLORE_BLIND_ENABLE=true        ./scripts/runtime/wifi_nav.sh
EXPLORE_MIN_FRONTIER_SIZE=6      ./scripts/runtime/wifi_nav.sh
EXPLORE_MAP_TOPIC=/global_costmap/costmap_raw ./scripts/runtime/wifi_nav.sh
./scripts/runtime/wifi_nav.sh --robot-ip 192.168.1.42
```
