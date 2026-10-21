# Unitree_GO2_Agricultural

Autonomous exploration and navigation layer for the **Unitree Go2 EDU** quadruped,
with support for the **Unitree D1-T** arm for crop-scanning and data collection.
This repo sits on top of the [`dancher00/go2-nav2-wifi`](https://github.com/dancher00/go2-nav2-wifi)
stack — Point-LIO SLAM running onboard the Jetson, Nav2 running on a laptop VM,
bridged over Wi-Fi — and adds a custom frontier-based explorer, an RViz control
panel, a live camera relay, an arm controller, and a full set of orchestration
scripts.

**One command to start everything (dog + arm):**

```bash
./scripts/runtime/wifi_nav.sh --arm
```

Power on the dog and the arm, stand the dog, run that. The script finds both
devices on the network, launches navigation over Wi-Fi (`enp0s3`), and folds
the D1 arm into its calibrated resting pose over Ethernet (`enp0s8`) in the
background. Without `--arm` it runs navigation only.

> First time with the arm? Calibrate the folded pose once. See
> [Arm — calibration](#arm--calibration-once). Until you do, `--arm` refuses to
> move the arm and navigation runs on its own.

**No robot handy?** Run the same stack against a Gazebo simulation of the Go2
with the D1 on its back (see [Simulation](#simulation)):

```bash
./gazebo/setup_gazebo.sh                     # once
./scripts/runtime/wifi_nav.sh --sim --arm
```

---

## Table of Contents

- [Project Scope](#project-scope)
- [Documentation](#documentation)
- [Architecture](#architecture)
- [Network](#network)
- [Quick Start](#quick-start)
- [Scripts](#scripts)
- [Explorer](#explorer)
- [D1-T Arm Control](#d1-t-arm-control)
- [Live Camera](#live-camera)
- [Simulation](#simulation)
- [Repository Structure](#repository-structure)
- [Collision Threshold Tuning](#collision-threshold-tuning)
- [Known Blockers and Fixes](#known-blockers-and-fixes)
- [Troubleshooting](#troubleshooting)
- [Development](#development)
- [What This Repo Adds Over Upstream](#what-this-repo-adds-over-upstream)
- [Legacy](#legacy)
- [External Dependencies](#external-dependencies)
- [License](#license)

---

## Project Scope

Two integrated capabilities:

1. **Autonomous navigation and mapping** — the Go2 explores and maps an
   environment on its own, driven by a custom frontier explorer.
2. **Crop scanning with the D1-T arm** — the arm holds a hyperspectral
   camera on a custom 3D-printed mount and moves it through a scanning
   pattern over plants, for later plant-analysis data collection.

The two systems are independent today. Integration — an AI model commanding
both the dog and the arm — is the long-term goal.

---

## Documentation

| Doc | Description |
|---|---|
| [docs/STATUS.md](docs/STATUS.md) | Session notes and current status |
| [docs/camera.md](docs/camera.md) | Live camera setup and troubleshooting |
| [docs/d1arm.md](docs/d1arm.md) | D1 arm runbook: protocol, network, commands, calibration, troubleshooting |
| [gazebo/README.md](gazebo/README.md) | Gazebo simulation of the Go2 + D1: setup, what is simulated, limits |
| [docs/roadmap.md](docs/roadmap.md) | Phase 1 roadmap |
| [docs/tour.md](docs/tour.md) | Waypoint mode setup |
| [docs/troubleshooting.md](docs/troubleshooting.md) | Standalone troubleshooting guide |
| [config/README.md](config/README.md) | Config folder notes (RViz layout) |
| [cmu-legacy/README.md](cmu-legacy/README.md) | Archived CMU-stack code |

---

## Architecture

### Navigation

```
Robot (Jetson, DHCP)                     Laptop VM (Ubuntu 22.04, DHCP on enp0s3)
─────────────────────────────            ───────────────────────────────────
LiDAR + IMU ─► Point-LIO                 go2-lidar3d container (Docker)
      │                                        │
      └─► /lidar3d/registered ──Wi-Fi relay──► /lidar3d/registered
                                               │
                                        Nav2 (planner/controller)
                                               │
                                        Regulated Pure Pursuit
                                               │
                                           /cmd_vel_nav
                                               │
                                        velocity_smoother
                                               │
                                             /cmd_vel
                                               │
                                        go2_cmd_vel_tcp_client
                                               │
                                     TCP :17999 (Wi-Fi)
                                               │
      go2_cmd_vel_tcp_server ◄─────────────────┘
             │
             ▼
      robot_sport_bridge
             │
             ▼
      Unitree Sport API → motors
```

### Arm

```
D1-T arm (6 DOF + gripper, 192.168.123.100)
      │
      │ RJ45 → VM enp0s8 (192.168.123.99), DDS domain 0, multicast
      ▼
rt/arm_Command      ◄── JSON commands ──  d1_arm_control.py / d1_arm_calibration.py
rt/arm_Feedback     ── joint angles, acks ──►   (host python3 + pip cyclonedds,
                                                 ROS not sourced, bound to enp0s8)
                                                            ▲
                                  start_arm.sh fold ◄── wifi_nav.sh --arm
```

Key facts:
- **SLAM runs on the Jetson.** The VM never runs `slam_toolbox`. The global
  frame is `lidar3d_map`, produced by Point-LIO onboard.
- **All ROS runs inside Docker** on the VM. The host shell sees no topics.
  Use `go2sh` or `docker exec -u root go2-lidar3d bash -lc '…'`.
- **DDS domain 64** on the Wi-Fi side, domain 0 onboard. The relay bridges them.
- **The explorer reads from `/global_costmap/costmap`**, not `/map`. The
  upstream stack does not publish a `/map` topic — the Nav2 costmap is the
  occupancy grid source.
- **The D1-T arm uses DDS + JSON** on `rt/arm_Command`, not ROS 2 messages.
  The arm control script wraps the raw interface.
- **Arm and dog DDS never mix.** The Go2 runs on `enp0s3` (domain 64), and the
  arm on `enp0s8` (domain 0) with its own CycloneDDS config. The scripts check
  that the two interfaces are different and on separate subnets.

---

## Network

The router assigns addresses by DHCP, so **no IPs are hardcoded**. They are
resolved on every run by [`scripts/runtime/go2_net.sh`](scripts/runtime/go2_net.sh).

| Node | IP | How it is found |
|---|---|---|
| Laptop VM | DHCP | `GO2_HOST_IP`: read from the Wi-Fi interface `enp0s3` (VirtualBox bridge). Override: `--host-ip` |
| Go2 Jetson | DHCP | `GO2_ROBOT_IP`: last known IP, then hostname `unitree`, then a MAC-prefix scan (`02:00:00`) on `enp0s3`, then an SSH-key probe. Override: `--robot-ip` |
| D1-T arm | `192.168.123.100` | Wired, on `enp0s8`. The nav stack never uses this interface |
| DDS domain | `64` | Wi-Fi side |
| Relay port | `17999` | TCP, `/cmd_vel` |

The container is created with `GO2_HOST_IP` and `GO2_ROBOT_IP` baked into its
environment. `mapping.sh` reads both from there and corrects only a stale host
IP, so a stale robot IP would send it to the wrong robot. When either address
changes, `wifi_nav.sh` recreates the container. Each session then writes its
CycloneDDS config afresh, bound to the current address. If the bind address were
wrong, the container would announce the wrong address and no topics would cross
the wire.

Interface safeguards: `enp0s8` (the arm, `ARM_IFACE`) is refused as the nav
interface. It is never scanned, and if the route to the robot goes through
it, the script stops with an error. The script also checks that the container's
DDS config binds only the Wi-Fi address.

Tunables: `GO2_IFACE` (default `enp0s3`), `ARM_IFACE` (default `enp0s8`),
`GO2_MAC_PREFIX`, `GO2_ROBOT_HOSTNAME`. Discovery is faster and more reliable
with `arp-scan` (`sudo apt install arp-scan`) or `nmap` installed; without
them it falls back to a ping sweep.

---

## Quick Start

### Navigation — one-time setup (per workstation)

```bash
# 1. Clone the upstream stack and this repo
git clone https://github.com/dancher00/go2-nav2-wifi ~/files/go2-nav2-wifi
git clone https://github.com/SteveJaman/Unitree_GO2_Agricultural ~/files/Unitree_GO2_Agricultural

# 2. Apply lab fixes (RPP + velocity smoother + correct lidar mount)
cd ~/files/go2-nav2-wifi
bash apply-lab-nav-fixes.sh

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

### Navigation — every session

```bash
# 1. Power on the dog and stand it up

# 2. Run:
cd ~/files/Unitree_GO2_Agricultural
./scripts/runtime/wifi_nav.sh

# 3. In RViz (opens automatically):
#    a. Wait ~12 s for the map to appear
#    b. "2D Pose Estimate" — click the dog's position
#    c. Click "Auto Explore: START" in the Go2 Explore panel
```

That's it. The dog explores autonomously. Click **STOP** or Ctrl+C in the
terminal to end. The physical remote is the emergency stop.

### Navigation — what the script does automatically

1. Detects the VM's Wi-Fi IP on `enp0s3` and discovers the robot's IP (see [Network](#network))
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

### Arm — one-time setup

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

### Arm — calibration (once)

The folded pose lives in `FOLDED_ANGLES` in
[`scripts/nodes/d1_arm_calibration.py`](scripts/nodes/d1_arm_calibration.py)
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

### Arm — every session

The usual way is `./scripts/runtime/wifi_nav.sh --arm`. It checks that
`enp0s8` exists, has an IPv4 address, and doesn't share a subnet with
`enp0s3`. It then runs `start_arm.sh fold` in the background right before Nav2
launches, logging to `go2-nav2-wifi/ws/log/arm.log`. If the arm isn't ready,
navigation still starts and the arm is skipped with a warning.

The fold: power on and enable the motors, send one funcode=2 multi-joint move
to `FOLDED_ANGLES`, wait 5 s, then **release the motors and power off**. After
that the arm is limp and rests on its mount.

To run the arm on its own:

```bash
# 1. Power the arm (24V DC) and confirm the RJ45 is connected

# 2. Fold into the calibrated resting pose (enable -> move -> release):
cd ~/files/Unitree_GO2_Agricultural
./scripts/runtime/start_arm.sh fold

# 3. Move one joint (joint index, degrees), then release:
./scripts/runtime/start_arm.sh enable sleep 2 joint 0 20 sleep 3 release
```

See [D1-T Arm Control](#d1-t-arm-control) for every command.

### Navigation — flags

| Flag | Effect |
|---|---|
| *(none)* | Explore + camera (default) |
| `--arm` | **Start everything:** also fold the D1 arm in the background (over `enp0s8`) |
| `--sim` | Run against the Gazebo simulator instead of the robot (no discovery, SSH or relay); combine with `--arm` |
| `--no-explore` | Manual waypoint mode (2D Goal Pose only) — see [docs/tour.md](docs/tour.md) |
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

### Navigation — runtime overrides (no file edits)

Every explorer parameter can be overridden with an env var:

```bash
EXPLORE_BLIND_ENABLE=true        ./scripts/runtime/wifi_nav.sh
EXPLORE_MIN_FRONTIER_SIZE=6      ./scripts/runtime/wifi_nav.sh
EXPLORE_MAP_TOPIC=/global_costmap/costmap_raw ./scripts/runtime/wifi_nav.sh
./scripts/runtime/wifi_nav.sh --robot-ip 192.168.1.42
```

---

## Scripts

### Runtime ([`scripts/runtime/`](scripts/runtime/))

| Script | Purpose |
|---|---|
| [`wifi_nav.sh`](scripts/runtime/wifi_nav.sh) | **Main entry point.** Relay + Nav2 + RViz + explorer + camera; `--arm` also folds the D1 arm |
| [`wifi_mapping.sh`](scripts/runtime/wifi_mapping.sh) | Mapping-only wrapper (no explorer) |
| [`start_rtabmap.sh`](scripts/runtime/start_rtabmap.sh) | Launch RTAB-Map for dense 3D reconstruction alongside the session |
| [`start_arm.sh`](scripts/runtime/start_arm.sh) | D1-T arm entry point: preflight (address + ping), then `fold`, `read`, `--calibrate`, or any `d1_arm_control.py` command chain |
| [`go2_net.sh`](scripts/runtime/go2_net.sh) | Sourced library: host-IP detection, `find_go2_ip()` robot discovery, interface safeguards |

### Nodes ([`scripts/nodes/`](scripts/nodes/))

| File | Purpose |
|---|---|
| [`frontier_explorer.py`](scripts/nodes/frontier_explorer.py) | Three-phase explorer: blind lead-in → Yamauchi frontier → visited-cell coverage → return. Optional spin-360 capture for tours |
| [`go2_camera_relay.py`](scripts/nodes/go2_camera_relay.py) | Bridge `/frontvideostream` (custom Unitree msg) → `/camera/image_raw` (standard `sensor_msgs/Image`) |
| [`go2_floor_filter.py`](scripts/nodes/go2_floor_filter.py) | Floor-plane filter, prevents floor being treated as an obstacle |
| [`d1_arm_control.py`](scripts/nodes/d1_arm_control.py) | D1-T arm chain-style CLI + `D1Arm` class (JSON over CycloneDDS): state, enable, release, home, power, joint(s), feedback, `lift`, `sweep` |
| [`d1_arm_calibration.py`](scripts/nodes/d1_arm_calibration.py) | `--read` current angles / `--move` to `FOLDED_ANGLES`, then release and power off |
| [`direct_test_movement.py`](scripts/nodes/direct_test_movement.py) | Small, safe movement test for the arm (uses `d1_arm_control.py`) |

### Tools ([`scripts/tools/`](scripts/tools/))

| Script | Purpose |
|---|---|
| [`auto_walk.sh`](scripts/tools/auto_walk.sh) | Scripted 1 m square patrol via `/goal_pose` — proof the motion path works |
| [`test_goal_cli.sh`](scripts/tools/test_goal_cli.sh) | Publish a single `PoseStamped` goal from the CLI |
| [`test_explore.sh`](scripts/tools/test_explore.sh) | Diagnose the explorer without the RViz button |
| [`preflight.sh`](scripts/tools/preflight.sh) | Layered verification: container, DDS, packages, nodes, topics, TF |
| [`campus_session.sh`](scripts/tools/campus_session.sh) | All-in-one verify → tune → patch → launch → monitor |
| [`tune_costmap.sh`](scripts/tools/tune_costmap.sh) | Change inflation radius and footprint padding at runtime (`--safe`, `--wide`, `--tight`, `--show`) |
| [`verify_costmap.sh`](scripts/tools/verify_costmap.sh) | Confirm the costmap topic exists and publishes |
| [`save_rviz_config.sh`](scripts/tools/save_rviz_config.sh) | Interactive guide to save the RViz layout to [`config/tour.rviz`](config/tour.rviz) |
| [`export_tour.py`](scripts/tools/export_tour.py) | Open3D Poisson mesh reconstruction from an RTAB-Map PLY |
| [`diag_arm.sh`](scripts/tools/diag_arm.sh) | D1 arm diagnostic: interface, ping, container, cyclonedds, DDS match |
| [`calibrate_arm.py`](scripts/tools/calibrate_arm.py) | D1 arm per-joint offset measurement (commanded vs reported angle); run via `start_arm.sh --calibrate` |

---

## Explorer

The custom explorer ([`scripts/nodes/frontier_explorer.py`](scripts/nodes/frontier_explorer.py)) replaces `explore_lite`.
Four phases:

1. **Blind lead-in** — when the map has < `blind_trigger_cells` known cells,
   publish direct `Twist` on `/cmd_vel_nav` to force new sensor data.
   *(Disabled by default — flip `EXPLORE_BLIND_ENABLE=true` to test.)*
2. **Frontier detection** (Yamauchi 1997) — find free cells adjacent to unknown,
   cluster them, score by `size / distance`, publish the best as a `PoseStamped`
   on `/goal_pose`.
3. **Visited-cell coverage** — when no frontiers remain, drive to the nearest
   unvisited free cell. *(Disabled by default.)*
4. **Return to origin** — drives back to the starting pose.
   *(Disabled by default.)*
5. **Spin-on-goal** — rotate 360° after each successful goal for tour capture.
   *(Disabled by default — flip `-p spin_on_goal:=true`.)*

### Parameters

| Param | Default | Notes |
|---|---|---|
| `robot_base_frame` | `base_link` | Must match the TF tree |
| `map_frame` | `lidar3d_map` | Frame from Point-LIO |
| `map_topic` | `/global_costmap/costmap` | The stack has no `/map`; the costmap is the grid source |
| `goal_topic` | `/goal_pose` | Consumed by `goal_pose_nav` |
| `cmd_vel_topic` | `/cmd_vel_nav` | Goes through the smoother — do NOT use `/cmd_vel` |
| `stop_topic` | `/cmd_vel` | Emergency stop bypasses the smoother so it overrides Nav2 |
| `min_frontier_size` | `4` | Cells per cluster — frontiers are thin |
| `min_known_cells` | `200` | Map must have this many known cells before frontier logic runs |
| `blind_trigger_cells` | `100` | Below this, do a blind step |
| `blind_enable` | `false` | Off by default — fights Nav2 during first tests |
| `coverage_enable` | `false` | Off by default |
| `return_to_init` | `false` | Off by default |
| `spin_on_goal` | `false` | Off by default — enable for tour capture |
| `spin_rate_rad_s` | `0.4` | Rotation rate during spin_360 |
| `tick_hz` | `1.0` | Main loop rate |

### Running the explorer manually

```bash
docker cp ~/files/Unitree_GO2_Agricultural/scripts/nodes/frontier_explorer.py \
    go2-lidar3d:/tmp/frontier_explorer.py

docker exec -u root -it go2-lidar3d bash -lc '
    source /opt/ros/humble/setup.bash
    source /ws/install/setup.bash 2>/dev/null
    python3 /tmp/frontier_explorer.py --ros-args \
        -p map_frame:=lidar3d_map \
        -p map_topic:=/global_costmap/costmap \
        -p goal_topic:=/goal_pose \
        -p cmd_vel_topic:=/cmd_vel_nav \
        -p stop_topic:=/cmd_vel \
        -p blind_enable:=false \
        -p coverage_enable:=false \
        -p return_to_init:=false \
        -p spin_on_goal:=false \
        -p tick_hz:=1.0
'
```

Enable in another terminal:

```bash
docker exec -u root go2-lidar3d bash -lc '
    source /opt/ros/humble/setup.bash && source /ws/install/setup.bash 2>/dev/null
    ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"
'
```

Watch status:

```bash
docker exec -u root go2-lidar3d bash -lc '
    source /opt/ros/humble/setup.bash && source /ws/install/setup.bash 2>/dev/null
    ros2 topic echo /explore/status
'
```

Expected states: `IDLE` → `EXPLORING` → goals on `/goal_pose` → `RETURNING` → `DONE`.

---

## D1-T Arm Control

The D1-T is a 6-DOF collaborative arm with a gripper, controlled via **DDS with
JSON strings** (type `unitree_arm::msg::dds_::ArmString_`, DDS domain 0).
Commands go to `rt/arm_Command`. Joint angles and acks come back on
`rt/arm_Feedback`, and `current_servo_angle` serves as a backup source of joint
angles. **All angles are degrees.** Joints 0–5 are the arm; joint 6 is the
gripper (servo degrees, 0 = closed). Full protocol notes:
[docs/d1arm.md](docs/d1arm.md).

### Physical setup

- **Power:** 24V DC via the DC 5.5-2.1 barrel jack (~1 min boot)
- **Communication:** RJ45 to the VM's `enp0s8`. Arm at `192.168.123.100`, VM at
  `192.168.123.99` (`sudo ip addr add 192.168.123.99/24 dev enp0s8`). DDS
  discovery is multicast. It is kept separate from the Go2's Wi-Fi DDS
- **Debug:** USB-C (serial debug only)

### Commands

Every command is `{"seq":N,"address":1,"funcode":F,"data":{...}}`, where
`data` is a nested JSON object, not a string.

| Action | funcode | `data` | Purpose |
|---|---|---|---|
| Single joint | `1` | `{"id":J,"angle":DEG,"delay_ms":0}` | Move joint J (6 = gripper) |
| All joints | `2` | `{"mode":1,"angle0":…,"angle6":…}` | Move all 7 at once |
| Enable all | `5` | `{"mode":80000}` | Motors on, fully locked |
| Release | `5` | `{"mode":0}` | Motors off (limp, drag-teach) |
| Home | `7` | *(none)* | Return all joints to zero |

Joint limits (degrees, match the D1-550 URDF): ±135, ±90, ±90, ±135, ±90, ±135.
The scripts reject anything beyond them.

### Usage

All arm commands go through `start_arm.sh`, which runs a preflight (host
address, ping) first. Commands chain in one process, so the arm is only
discovered once.

```bash
A=./scripts/runtime/start_arm.sh

$A --check                                 # preflight only
$A fold                                    # fold to FOLDED_ANGLES, release, power off
$A read                                    # print FOLDED_ANGLES (calibration)
$A state                                   # status
$A feedback                                # joint angles
$A motors                                  # motor info

# Move one joint (joint index, degrees), then release
$A enable sleep 2 joint 0 20 sleep 3 release

# Several joints (7 values: 6 joints + gripper; non-zero joints sent one by one)
$A enable joints 0 -30 30 0 20 0 0 sleep 3 release

# Home all joints to zero, then release
$A home sleep 6 release

# Shoulder lift and joint sweep (ask for confirmation without --yes)
$A lift --deg 45 --yes
$A sweep --pct 90 --yes                    # each joint to ±90% of its limit and back
$A sweep --joints 0,1 --pct 50 --yes

# Per-joint offset calibration
$A --calibrate --test-deg 10 --output cal.json --yes
```

Run the arm scripts without sourcing ROS: ROS's `libddsc` can clash with the
pip `cyclonedds`.

### Parameters

| Flag / env | Default | Notes |
|---|---|---|
| `--iface` / `ARM_IFACE` | `enp0s8` | Interface that reaches the arm |
| `--ip` / `ARM_IP` | `192.168.123.100` | Arm address (preflight ping and route check) |
| `ARM_HOST_IP` | `192.168.123.99` | VM address `start_arm.sh` expects on the interface |
| `--domain` / `ARM_DOMAIN` | `0` | DDS domain |
| `--enable-mode` | `80000` | `enable` value (80000 = fully locked) |
| `--match-timeout` | `20` | Seconds to wait for DDS discovery |
| `--deg` | `45` | `lift` target for the shoulder |
| `--pct` / `--joints` | `90` / `all` | `sweep` range (% of limit) and joints |
| `--yes` | off | Skip the confirmation prompt for `lift` / `sweep` |
| `--peer` | *(none)* | Unicast discovery. **Don't use**: this firmware needs multicast |

### Verifying the arm is reachable

```bash
ARM_IFACE=enp0s8 ./scripts/tools/diag_arm.sh         # interface, ping, cyclonedds, DDS match
./scripts/runtime/start_arm.sh feedback              # joint angles
```

### Roadmap for the arm

| Step | Purpose | Status |
|---|---|---|
| 1. Direct control | Prove the arm responds to JSON commands | Written: chain control, fold, lift/sweep, offset calibration |
| 2. ROS 2 bridge node | Subscribe to `/arm/command` instead of the raw script | Pending |
| 3. RViz panel | Slider per joint, alongside the explorer panel | Pending |
| 4. Camera mount | 3D-printed attachment for the hyperspectral camera | Hardware |
| 5. Scan routine | Move the camera through a plant-scanning path | Pending (`run` sequence files are the building block) |
| 6. AI autonomy | Model commands both the dog and the arm | Long-term |

See [docs/roadmap.md](docs/roadmap.md) for the overall project roadmap.

---

## Simulation

[`gazebo/`](gazebo/) is a Gazebo Harmonic simulation of the Go2 with the D1 on
its back, in a small field with planted beds. It replaces the **robot side**:
lidar odometry and cloud, the cmd_vel TCP server, the camera, and the D1
controller. The **laptop side** stays exactly as it is: Nav2, RViz, the explorer
and the arm scripts.

```bash
./gazebo/setup_gazebo.sh                     # once: official Unitree models + image
./scripts/runtime/wifi_nav.sh --sim --arm    # full stack, simulated
./gazebo/test_sim.sh                         # automated smoke test
```

The simulated Go2 glides rather than walks (legs held in a standing pose), and
its localization is perfect. It tests navigation, exploration and arm logic, not
gait or SLAM. Details and limits: [gazebo/README.md](gazebo/README.md).

---

## Live Camera

The Go2's front camera publishes on `/frontvideostream` as a custom
`unitree_go/Go2FrontVideoData` message that RViz can't render. The camera relay
([`scripts/nodes/go2_camera_relay.py`](scripts/nodes/go2_camera_relay.py)) subscribes to that topic, decodes the
H.264/JPEG payload, and republishes as standard `sensor_msgs/Image` on
`/camera/image_raw`.

The relay launches automatically when `wifi_nav.sh` runs (unless `--no-camera`).

**In RViz:**

1. Click **Add** at the bottom of the Displays panel
2. Select **Image** → OK
3. Set **Topic** to `/camera/image_raw`
4. Set **Reliability** to `Best Effort` (critical — default Reliable will drop frames)
5. **File → Save Config As** to [`config/tour.rviz`](config/tour.rviz) so it loads next time

Or view outside RViz:

```bash
go2sh
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

See [docs/camera.md](docs/camera.md) for full setup and troubleshooting.

> **Note:** the hyperspectral camera on the D1-T arm is a separate device from
> the Go2's front camera. Its ROS 2 / DDS interface will be documented in a
> dedicated doc once the physical mount is in place.

---

## Repository Structure

> Code blocks can't contain clickable links. Use the [Documentation](#documentation)
> section above, or browse the folders directly on GitHub.

```
Unitree_GO2_Agricultural/
├── README.md
├── LICENSE
├── .gitignore
├── .gitattributes
├── .clang-format                     # C++ style for the RViz plugin
├── .pre-commit-config.yaml           # ruff, shellcheck, clang-format, markdownlint
├── config/
│   ├── README.md
│   └── tour.rviz                     # RViz layout (camera, path, costmap, waypoint panel)
├── docs/
│   ├── STATUS.md                     # session notes
│   ├── camera.md                     # live camera setup
│   ├── d1arm.md                      # D1 arm runbook
│   ├── roadmap.md                    # Phase 1 roadmap
│   ├── tour.md                       # waypoint mode setup
│   └── troubleshooting.md            # standalone troubleshooting
├── scripts/
│   ├── runtime/                      # shell entry points
│   │   ├── wifi_nav.sh               # main orchestrator
│   │   ├── wifi_mapping.sh           # mapping-only wrapper
│   │   ├── start_rtabmap.sh          # RTAB-Map launcher
│   │   ├── start_arm.sh              # D1-T arm entry point (preflight, fold, control)
│   │   └── go2_net.sh                # shared IP detection / robot discovery
│   ├── nodes/                        # ROS 2 Python nodes
│   │   ├── frontier_explorer.py      # custom explorer
│   │   ├── go2_camera_relay.py       # camera bridge
│   │   ├── go2_floor_filter.py       # floor-plane filter
│   │   ├── d1_arm_control.py         # D1-T arm control (chain CLI, lift, sweep)
│   │   ├── d1_arm_calibration.py     # D1-T folded-pose calibration
│   │   └── direct_test_movement.py   # small safe arm movement test
│   └── tools/                        # diagnostics and tests
│       ├── auto_walk.sh
│       ├── test_goal_cli.sh
│       ├── test_explore.sh
│       ├── preflight.sh
│       ├── campus_session.sh
│       ├── tune_costmap.sh
│       ├── verify_costmap.sh
│       ├── save_rviz_config.sh
│       ├── export_tour.py
│       ├── diag_arm.sh               # D1 arm diagnostic
│       └── calibrate_arm.py          # D1 arm per-joint offset calibration
├── gazebo/                           # Gazebo simulation (Go2 + D1), see gazebo/README.md
│   ├── setup_gazebo.sh / run_sim.sh / test_sim.sh
│   ├── docker/Dockerfile             # go2-sim:local (Humble + Gazebo Harmonic)
│   ├── sim/                          # model builder, robot bridge, D1 emulator
│   ├── worlds/agri_field.sdf
│   └── models/                       # downloaded Unitree models (not in git)
├── src/
│   └── go2_explore_panel/            # RViz panel with Auto Explore toggle
└── cmu-legacy/                       # archived CMU-stack code, not used
```

Folders: [`config/`](config/) · [`docs/`](docs/) · [`scripts/`](scripts/) · [`src/go2_explore_panel/`](src/go2_explore_panel/) · [`cmu-legacy/`](cmu-legacy/)

---

## Collision Threshold Tuning

Nav2's collision avoidance is controlled by two dynamic parameters on the
costmaps: `inflation_radius` and `footprint_padding`. Recommended values:

| Parameter | Safe | Wide | Tight | Notes |
|---|---|---|---|---|
| `inflation_radius` | `0.70` | `0.85` | `0.45` | Must be ≥ the Go2's inscribed radius (~0.20 m) |
| `footprint_padding` | `0.05` | `0.05` | `0.02` | Extra buffer around the robot outline |

Change at runtime without restarting:

```bash
./scripts/tools/tune_costmap.sh --safe              # restore 0.70 / 0.05
./scripts/tools/tune_costmap.sh --wide              # 0.85 / 0.05, crowded spaces
./scripts/tools/tune_costmap.sh --tight             # 0.45 / 0.02, small spaces
./scripts/tools/tune_costmap.sh 0.65 0.05           # custom
./scripts/tools/tune_costmap.sh --show              # current values
```

Every change also sets `cost_scaling_factor=3.0` for a smoother cost gradient.
`wifi_nav.sh` starts with `SAFETY_INFLATION_RADIUS=0.70`.

```bash
```

Or use `rqt_reconfigure` for sliders:

```bash
go2sh
ros2 run rqt_reconfigure rqt_reconfigure
```

To persist values across sessions, edit `go2_nav2/config/lidar3d_planner.yaml`
and the matching override in `go2_nav2/launch/lidar3d_controller.launch.py`.

---

## Known Blockers and Fixes

### Robot IP not found / DHCP changed

`wifi_nav.sh` stops with `could not find the Go2 on enp0s3` when discovery fails.
Find the robot's address in the router's DHCP lease list, or run `hostname -I`
on the Jetson, then:

```bash
./scripts/runtime/wifi_nav.sh --robot-ip <IP>
```

The address is cached, so the next run checks it first. If the Jetson's Wi-Fi
MAC doesn't start with `02:00:00`, set `GO2_MAC_PREFIX` to the real prefix.

### Container crashes with `numpy.dtype size changed`

`wifi_nav.sh` pins `numpy<1.25` inside the running container before launch. It
also appends the pin to the upstream Dockerfiles in `go2-nav2-wifi/docker/`,
including `Dockerfile.lidar3d`, so rebuilds keep it. It installs as root, then
restores the file's user; `.dockerignore` files are left alone. If the pip
install fails (often: no internet on the robot's Wi-Fi), it warns and continues.
Run the pin once with internet. If the container exits before the pin can be
applied, rebuild it:

```bash
docker rm -f go2-lidar3d
cd ~/files/go2-nav2-wifi && ./setup-lidar3d.sh --laptop
```

### Jetson relay port 17999 conflict

If the motion relay fails with `OSError: [Errno 98] Address already in use`,
a stale process is holding port 17999. **`wifi_nav.sh` handles this
automatically.** It kills stale relays over SSH before starting the new one.

> With Jetson compute (the default), upstream `mapping.sh` starts its own
> motion bridge on 17999 (`robot-motion-wifi.sh`), so `wifi_nav.sh` no longer
> starts a second FULL relay on the same port. If `motion.log` still shows
> "Address already in use", run with `--no-relay`. Details:
> [docs/STATUS.md](docs/STATUS.md).

If you need to do it manually:

```bash
ssh unitree@<robot-ip>       # shown in the wifi_nav.sh banner
pkill -9 -f 'robot-relay-wifi|robot-motion-wifi|go2_cmd_vel_tcp|robot_sport_bridge'
sleep 3
ss -tlnp | grep 17999        # must be empty
```

### Explorer produces no goals (`goals_sent=0`)

Check in order:

1. **Costmap is publishing:** `ros2 topic hz /global_costmap/costmap` — expect ~1 Hz
2. **Explorer is subscribed:** `ros2 topic info /global_costmap/costmap --verbose` — Subscription count should be 1
3. **Frontier detection is running:** explorer logs `detect_frontiers: known=N frontiers=K` every 5 s
4. If `frontiers=0`, lower `EXPLORE_MIN_FRONTIER_SIZE` or drive the dog manually for 10 s to open up the map

### Dog doesn't move

If `/cmd_vel` publishes at 20 Hz with real values but the dog stands still:

1. Check the Jetson relay: `ssh unitree@<robot-ip> 'ss -tlnp | grep 17999'`
2. Check the relay log: `ssh unitree@<robot-ip> 'tail -20 /tmp/robot-relay.log'`
3. Confirm FULL mode: the log should say `cmd_vel TCP server started on port 17999`

### Arm does not respond

1. **Run the diagnostic:** `ARM_IFACE=enp0s8 ./scripts/tools/diag_arm.sh`.
   It checks the interface, ping, container, `cyclonedds`, and DDS match.
2. **Check the address.** `enp0s8` needs `192.168.123.99`:
   `sudo ip addr add 192.168.123.99/24 dev enp0s8`, then
   `./scripts/runtime/start_arm.sh --check`.
3. **Check the arm is powered.** 24V DC via the barrel jack; allow ~1 min to boot.
4. **Check `cyclonedds`** on the VM: `pip3 show cyclonedds`. Discovery is
   multicast; don't pass `--peer`. Don't source ROS in the same shell.
5. **Read the joint feedback:**
   ```bash
   ./scripts/runtime/start_arm.sh feedback
   ```
   If feedback arrives but commands do nothing, the motors may not be enabled.
   Run `enable` first.
6. **`wifi_nav.sh --arm` skipped the arm?** Read `go2-nav2-wifi/ws/log/arm.log`.
   A preflight failure there means the address or ping check failed; run
   `./scripts/runtime/start_arm.sh --check`. See also
   [Arm — calibration](#arm--calibration-once).

---

## Troubleshooting

See [`docs/troubleshooting.md`](docs/troubleshooting.md) for the full guide.
Common items:

| Symptom | Cause | Fix |
|---|---|---|
| `GO2_CONTAINER: Start the project Docker container first` | Container missing | `cd ~/files/go2-nav2-wifi && ./setup-lidar3d.sh --laptop` |
| `ros2: command not found` on host | ROS runs inside Docker | Use `go2sh` or `docker exec -u root go2-lidar3d bash -lc '…'` |
| `Invalid frame ID "lidar3d_map"` | Session not up, or Point-LIO not forwarding | Check `mapping.sh --3d --nav` log; check Jetson relay |
| `/cmd_vel` has 20 Hz but dog doesn't move | Jetson port 17999 conflict | `wifi_nav.sh` handles it; if `motion.log` says "Address already in use", try `--no-relay` |
| Simulator won't start or RViz is empty in `--sim` | Image, rendering, or `simarm0` | [gazebo/README.md](gazebo/README.md#troubleshooting), `./gazebo/test_sim.sh` |
| Explorer stays `IDLE` | Button not wired, or costmap silent | [`./scripts/tools/test_explore.sh`](scripts/tools/test_explore.sh) |
| Camera not showing in RViz | QoS mismatch | Set Image display **Reliability** to `Best Effort` — see [docs/camera.md](docs/camera.md) |
| Arm does not respond to commands | Wrong `--iface`, or motors not enabled | See [Arm does not respond](#arm-does-not-respond) above |
| `could not find the Go2 on enp0s3` | DHCP changed / discovery failed | `wifi_nav.sh --robot-ip <IP>` — see [Robot IP not found](#robot-ip-not-found--dhcp-changed) |
| `DDS discovery failed` (arm) | No DDS match within 20 s | Arm powered and booted? No `--peer`, no ROS sourced; check `diag_arm.sh` |
| RViz renders slowly | CPU rendering | Close the point cloud display while tuning |

---

## Development

Lint and format checks run automatically on `git commit` once pre-commit is
installed:

```bash
pip install pre-commit
pre-commit install
pre-commit run --all-files    # first-time baseline
```

Hooks cover: Python (ruff), shell (shellcheck), C++ (clang-format), Markdown
(markdownlint), and general hygiene (trailing whitespace, EOF newline, large
files). To skip once: `git commit --no-verify`.

Session notes live in [docs/STATUS.md](docs/STATUS.md).

---

## What This Repo Adds Over Upstream

The upstream `go2-nav2-wifi` provides SLAM (Point-LIO), Nav2, the Wi-Fi relay,
and manual waypoint navigation via RViz. This repo adds:

- **Autonomous frontier exploration** — the explorer node, not in upstream
- **Blind lead-in** for cold-start mapping
- **Visited-cell coverage** pass to fill gaps after frontiers are exhausted
- **Spin-on-goal capture** for tour reconstruction
- **RViz control panel** with START/STOP buttons
- **Live camera relay** — `/frontvideostream` → `/camera/image_raw`
- **Floor-plane filter** to prevent the ground being treated as an obstacle
- **D1-T arm control**: direct DDS/JSON commands chained in one process,
  shoulder lift and joint sweep tests, folded-pose and per-joint offset calibration
- **One-command orchestration**: `wifi_nav.sh --arm` starts dog and arm,
  with auto relay startup, cleanup, and monitoring
- **DHCP-proof networking**: host IP auto-detection, robot discovery,
  container recreated when the host or robot IP changes, arm/dog DDS isolation checks
- **Gazebo simulation**: Go2 + D1 from the official Unitree models; the
  unchanged stack and arm scripts run against it (`wifi_nav.sh --sim`)
- **Diagnostic and tuning scripts** — preflight, test_explore, campus_session, tune_costmap
- **RTAB-Map integration** — dense 3D reconstruction and mesh export

---

## Legacy

Earlier work based on the CMU `autonomy_stack_go2` running over Ethernet or
the Unity simulation is archived under [`cmu-legacy/`](cmu-legacy/) for reference. It is not
part of the active pipeline. See [`cmu-legacy/README.md`](cmu-legacy/README.md).

---

## External Dependencies

| Project | Purpose |
|---|---|
| [`dancher00/go2-nav2-wifi`](https://github.com/dancher00/go2-nav2-wifi) | SLAM (Point-LIO) + Nav2 + Wi-Fi relay |
| [`cyclonedds`](https://pypi.org/project/cyclonedds/) | Python DDS bindings used by the D1-T arm scripts (`pip3 install cyclonedds` on the VM) |
| [`JeewanthaSadaruwan/unitree-D1-550-Robot-ARM`](https://github.com/JeewanthaSadaruwan/unitree-D1-550-Robot-ARM) | Reference only: ideas behind an earlier smooth-motion version of the arm control (reimplemented, no code copied) |

---

## License

MIT — see [LICENSE](LICENSE).
