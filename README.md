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
| [docs/d1arm.md](docs/d1arm.md) | D1 arm runbook: protocol, network, glide, sequences, troubleshooting |
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
      │ RJ45 → VM enp0s8 (192.168.123.x), DDS domain 0
      ▼
rt/arm_Command      ◄── JSON commands ──  d1_arm_control.py / d1_arm_calibration.py
rt/arm_Feedback     ── joint angles, acks ──►        (run in go2-lidar3d, ROS not
current_servo_angle ── joint angles (backup) ─►       sourced, DDS bound to enp0s8)
                                                            ▲
                                       start_arm.sh ◄── wifi_nav.sh --arm
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

The container's CycloneDDS config is baked in from `GO2_HOST_IP` when the
container is created. When the DHCP lease changes, `wifi_nav.sh` detects the
mismatch and recreates the container. If the bind address were wrong, the
container would announce the wrong address and no topics would cross the wire.

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
4. Recreates the container if it was created for an old IP, and pins
   `numpy<1.25` inside it (prevents `ValueError: numpy.dtype size changed`)
5. Checks that container DDS binds the Wi-Fi address, not the arm interface
6. SSHes into the Jetson, kills any relay holding port 17999, and starts
   `robot-relay-wifi.sh` in FULL mode with the current `GO2_HOST_IP`
7. Waits for port 17999 to bind before proceeding
8. Launches Nav2 + RViz via `mapping.sh --3d --nav`
9. After 20 s: starts the frontier explorer and camera relay
10. On Ctrl+C: stops exploration, kills helpers, stops the Jetson relay, brings down Docker

### Arm — one-time setup

1. In VirtualBox, attach the arm's Ethernet (USB adapter or a second bridged
   adapter) to the VM as `enp0s8`. The Go2 Wi-Fi stays on `enp0s3`.
2. Give `enp0s8` an address on the arm's subnet. This doesn't persist across
   reboots, so re-run it after a VM restart:

   ```bash
   ./scripts/runtime/start_arm.sh --setup-net    # adds 192.168.123.51/24, pings the arm
   ```

3. Check everything:

   ```bash
   ARM_IFACE=enp0s8 ./scripts/tools/diag_arm.sh   # all PASS expected
   ```

The arm scripts use the `cyclonedds` Python package (pinned to `0.10.2`, the
version Unitree's SDK is built against). `start_arm.sh` installs it in the
container on first run, so there is no separate SDK install.

### Arm — calibration (once)

The folded pose lives in `FOLDED_ANGLES` in
[`scripts/nodes/d1_arm_calibration.py`](scripts/nodes/d1_arm_calibration.py)
(degrees: joints 0–5, then the gripper). The shipped value is a placeholder
that drives joints 1 and 2 to their ±90° limits, so `start_arm.sh` refuses to
fold until you calibrate.

```bash
# 1. With the motors released, fold the arm by hand onto the dog's back
# 2. Read the angles:
./scripts/runtime/start_arm.sh --read
#    -> FOLDED_ANGLES = [ ... ]
# 3. Paste that line over FOLDED_ANGLES in scripts/nodes/d1_arm_calibration.py
```

Redo this whenever the mount changes.

### Arm — every session

The usual way is `./scripts/runtime/wifi_nav.sh --arm`. It checks that
`enp0s8` exists, has an IPv4 address, and doesn't share a subnet with
`enp0s3`. It then runs `start_arm.sh` in the background right before Nav2
launches, logging to `go2-nav2-wifi/ws/log/arm.log`. If the arm isn't ready,
navigation still starts and the arm is skipped with a warning.

The fold: enable motors, then one funcode=2 multi-joint glide to
`FOLDED_ANGLES`, then wait 5 s and **release the motors**. After that the arm
is limp and rests on its mount.

To run the arm on its own:

```bash
# 1. Power the arm (24V DC) and confirm the RJ45 is connected

# 2. Fold into the calibrated resting pose (enable -> glide -> release):
cd ~/files/Unitree_GO2_Agricultural
./scripts/runtime/start_arm.sh

# 3. Smooth move from the current pose (degrees: 6 joints + gripper):
docker exec -u root go2-lidar3d env -u CYCLONEDDS_URI \
    python3 /tmp/d1_arm_control.py --iface enp0s8 enable glide 0 -30 30 0 20 0 0
```

See [D1-T Arm Control](#d1-t-arm-control) for every command.

### Navigation — flags

| Flag | Effect |
|---|---|
| *(none)* | Explore + camera (default) |
| `--arm` | **Start everything:** also fold the D1 arm in the background (over `enp0s8`) |
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
| [`start_arm.sh`](scripts/runtime/start_arm.sh) | D1-T arm: verify interface, fold to the calibrated pose and release (`--read` to calibrate, `--no-home` to enable only, `--setup-net` to address `enp0s8`) |
| [`go2_net.sh`](scripts/runtime/go2_net.sh) | Sourced library: host-IP detection, `find_go2_ip()` robot discovery, interface safeguards |

### Nodes ([`scripts/nodes/`](scripts/nodes/))

| File | Purpose |
|---|---|
| [`frontier_explorer.py`](scripts/nodes/frontier_explorer.py) | Three-phase explorer: blind lead-in → Yamauchi frontier → visited-cell coverage → return. Optional spin-360 capture for tours |
| [`go2_camera_relay.py`](scripts/nodes/go2_camera_relay.py) | Bridge `/frontvideostream` (custom Unitree msg) → `/camera/image_raw` (standard `sensor_msgs/Image`) |
| [`go2_floor_filter.py`](scripts/nodes/go2_floor_filter.py) | Floor-plane filter, prevents floor being treated as an obstacle |
| [`d1_arm_control.py`](scripts/nodes/d1_arm_control.py) | D1-T arm CLI + `D1Arm` class (JSON over CycloneDDS): enable, joints, smooth `glide`, gripper, averaged angles, sequence files |
| [`d1_arm_calibration.py`](scripts/nodes/d1_arm_calibration.py) | `--read` current angles / `--move` glide to `FOLDED_ANGLES`, then release |
| [`direct_test_movement.py`](scripts/nodes/direct_test_movement.py) | Small, safe movement test for the arm (uses `d1_arm_control.py`) |

### Tools ([`scripts/tools/`](scripts/tools/))

| Script | Purpose |
|---|---|
| [`auto_walk.sh`](scripts/tools/auto_walk.sh) | Scripted 1 m square patrol via `/goal_pose` — proof the motion path works |
| [`test_goal_cli.sh`](scripts/tools/test_goal_cli.sh) | Publish a single `PoseStamped` goal from the CLI |
| [`test_explore.sh`](scripts/tools/test_explore.sh) | Diagnose the explorer without the RViz button |
| [`preflight.sh`](scripts/tools/preflight.sh) | Layered verification: container, DDS, packages, nodes, topics, TF |
| [`campus_session.sh`](scripts/tools/campus_session.sh) | All-in-one verify → tune → patch → launch → monitor |
| [`tune_costmap.sh`](scripts/tools/tune_costmap.sh) | Change inflation radius and footprint padding at runtime |
| [`verify_costmap.sh`](scripts/tools/verify_costmap.sh) | Confirm the costmap topic exists and publishes |
| [`save_rviz_config.sh`](scripts/tools/save_rviz_config.sh) | Interactive guide to save the RViz layout to [`config/tour.rviz`](config/tour.rviz) |
| [`export_tour.py`](scripts/tools/export_tour.py) | Open3D Poisson mesh reconstruction from an RTAB-Map PLY |
| [`diag_arm.sh`](scripts/tools/diag_arm.sh) | D1 arm diagnostic: interface, ping, container, cyclonedds, DDS match |

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
  `192.168.123.51` (`start_arm.sh --setup-net`). It is kept separate from the
  Go2's Wi-Fi DDS
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

```bash
# Fold to the calibrated pose and release (typical session start)
./scripts/runtime/start_arm.sh

# Print current angles for calibration
./scripts/runtime/start_arm.sh --read

# Enable only, no motion
./scripts/runtime/start_arm.sh --no-home

# Give enp0s8 its 192.168.123.x address (after a VM reboot)
./scripts/runtime/start_arm.sh --setup-net

# Smooth, speed-limited move from the measured pose (degrees; peak 25 deg/s)
docker exec -u root go2-lidar3d env -u CYCLONEDDS_URI \
    python3 /tmp/d1_arm_control.py --iface enp0s8 enable glide 0 -30 30 0 20 0 0

# Gripper, averaged angles, sequence file
docker exec -u root go2-lidar3d env -u CYCLONEDDS_URI \
    python3 /tmp/d1_arm_control.py --iface enp0s8 gripper 60 2000 angles
docker exec -u root go2-lidar3d env -u CYCLONEDDS_URI \
    python3 /tmp/d1_arm_control.py --iface enp0s8 run /tmp/scan.txt

# Move one joint (joint index, degrees)
docker exec -u root go2-lidar3d env -u CYCLONEDDS_URI \
    python3 /tmp/d1_arm_control.py --iface enp0s8 joint 0 20

# Move all joints in one command (7 values: 6 joints + gripper)
docker exec -u root go2-lidar3d env -u CYCLONEDDS_URI \
    python3 /tmp/d1_arm_control.py --iface enp0s8 joints 0 -30 30 0 20 0 0

# Raw feedback / status
docker exec -u root go2-lidar3d env -u CYCLONEDDS_URI \
    python3 /tmp/d1_arm_control.py --iface enp0s8 feedback status

# Chain commands in ONE process (each new process must rediscover the arm)
docker exec -u root go2-lidar3d env -u CYCLONEDDS_URI \
    python3 /tmp/d1_arm_control.py --iface enp0s8 \
        enable sleep 2 glide 0 -30 30 0 20 0 0 gripper 60 sleep 1 release
```

Run the arm scripts with `env -u CYCLONEDDS_URI` and without sourcing ROS.
The container's `CYCLONEDDS_URI` belongs to the Go2 Wi-Fi stack, and ROS's
`libddsc` can clash with the pip `cyclonedds`.

`joints` sends one target and lets the arm's controller choose the motion.
`glide` starts from the measured pose (a 10-sample average). It streams a
smooth quintic trajectory with a capped peak speed, then waits until every
joint is within tolerance. **It refuses to move without joint feedback.**

### Sequence files

`run <file>` executes one CLI command per line; `#` starts a comment. This
is the building block for the scan routine:

```text
# /tmp/scan.txt  (docker cp it into the container first)
enable
glide 0 -30 30 0 20 0 0
sleep 2
glide 20 -30 30 0 20 0 0
release
```

### Parameters

| Flag | Default | Notes |
|---|---|---|
| `--iface` | `enp0s8` (or `$ARM_IFACE`) | Interface that reaches the arm |
| `--peer` | *(multicast)* | Unicast discovery peer, e.g. `192.168.123.100`, if multicast fails |
| `--vmax` | `25` | `glide` peak joint speed, deg/s |
| `--rate` | `50` | `glide` setpoint stream rate, Hz |
| `--tolerance` | `5` | `glide` "reached" tolerance, degrees |
| `--enable-mode` | `80000` | `enable` value (80000 = fully locked) |

### Verifying the arm is reachable

```bash
ARM_IFACE=enp0s8 ./scripts/tools/diag_arm.sh         # interface, ping, cyclonedds, DDS match
docker exec -u root go2-lidar3d env -u CYCLONEDDS_URI \
    python3 /tmp/d1_arm_control.py --iface enp0s8 angles
```

### Roadmap for the arm

| Step | Purpose | Status |
|---|---|---|
| 1. Direct control | Prove the arm responds to JSON commands | Written: control, fold calibration, smooth `glide`, sequences |
| 2. ROS 2 bridge node | Subscribe to `/arm/command` instead of the raw script | Pending |
| 3. RViz panel | Slider per joint, alongside the explorer panel | Pending |
| 4. Camera mount | 3D-printed attachment for the hyperspectral camera | Hardware |
| 5. Scan routine | Move the camera through a plant-scanning path | Pending (`run` sequence files are the building block) |
| 6. AI autonomy | Model commands both the dog and the arm | Long-term |

See [docs/roadmap.md](docs/roadmap.md) for the overall project roadmap.

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
│   │   ├── start_arm.sh              # D1-T arm fold / calibrate / network setup
│   │   └── go2_net.sh                # shared IP detection / robot discovery
│   ├── nodes/                        # ROS 2 Python nodes
│   │   ├── frontier_explorer.py      # custom explorer
│   │   ├── go2_camera_relay.py       # camera bridge
│   │   ├── go2_floor_filter.py       # floor-plane filter
│   │   ├── d1_arm_control.py         # D1-T arm control (glide, gripper, sequences)
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
│       └── diag_arm.sh               # D1 arm diagnostic
├── src/
│   └── go2_explore_panel/            # RViz panel with Auto Explore toggle
└── cmu-legacy/                       # archived CMU-stack code, not used
```

Folders: [`config/`](config/) · [`docs/`](docs/) · [`scripts/`](scripts/) · [`src/go2_explore_panel/`](src/go2_explore_panel/) · [`cmu-legacy/`](cmu-legacy/)

---

## Collision Threshold Tuning

Nav2's collision avoidance is controlled by two dynamic parameters on the
costmaps: `inflation_radius` and `footprint_padding`. Recommended values:

| Parameter | Safe | Tight | Notes |
|---|---|---|---|
| `inflation_radius` | `0.35` | `0.28` | Must be ≥ the Go2's inscribed radius (~0.20 m) |
| `footprint_padding` | `0.02` | `0.00` | Extra buffer around the robot outline |

Change at runtime without restarting:

```bash
./scripts/tools/tune_costmap.sh --safe              # restore 0.35 / 0.02
./scripts/tools/tune_costmap.sh 0.30 0.01           # custom
./scripts/tools/tune_costmap.sh --show              # current values
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
also appends the pin to the upstream Dockerfile(s) in `go2-nav2-wifi/docker/`
so rebuilds keep it. If the container exits before the pin can be applied,
rebuild it:

```bash
docker rm -f go2-lidar3d
cd ~/files/go2-nav2-wifi && ./setup-lidar3d.sh --laptop
```

### Jetson relay port 17999 conflict

If the motion relay fails with `OSError: [Errno 98] Address already in use`,
a stale process is holding port 17999. **`wifi_nav.sh` handles this
automatically** — it kills stale relays over SSH before starting the new one.

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
2. **Check the address.** `enp0s8` needs a `192.168.123.x` address:
   `./scripts/runtime/start_arm.sh --setup-net`.
3. **Check the arm is powered.** 24V DC via the barrel jack; allow ~1 min to boot.
4. **Check `cyclonedds`:** `docker exec go2-lidar3d pip3 show cyclonedds` should
   show `0.10.2`. If discovery fails, try `--peer 192.168.123.100`.
5. **Read the joint feedback:**
   ```bash
   docker exec -u root go2-lidar3d env -u CYCLONEDDS_URI \
       python3 /tmp/d1_arm_control.py --iface enp0s8 angles
   ```
   If feedback arrives but commands do nothing, the motors may not be enabled.
   Run `enable` first.
6. **`wifi_nav.sh --arm` skipped the arm?** Read `go2-nav2-wifi/ws/log/arm.log`.
   A `FOLDED_ANGLES is still the uncalibrated placeholder` error means the fold
   isn't calibrated yet. See [Arm — calibration](#arm--calibration-once).

---

## Troubleshooting

See [`docs/troubleshooting.md`](docs/troubleshooting.md) for the full guide.
Common items:

| Symptom | Cause | Fix |
|---|---|---|
| `GO2_CONTAINER: Start the project Docker container first` | Container missing | `cd ~/files/go2-nav2-wifi && ./setup-lidar3d.sh --laptop` |
| `ros2: command not found` on host | ROS runs inside Docker | Use `go2sh` or `docker exec -u root go2-lidar3d bash -lc '…'` |
| `Invalid frame ID "lidar3d_map"` | Session not up, or Point-LIO not forwarding | Check `mapping.sh --3d --nav` log; check Jetson relay |
| `/cmd_vel` has 20 Hz but dog doesn't move | Jetson port 17999 conflict | `wifi_nav.sh` handles it |
| Explorer stays `IDLE` | Button not wired, or costmap silent | [`./scripts/tools/test_explore.sh`](scripts/tools/test_explore.sh) |
| Camera not showing in RViz | QoS mismatch | Set Image display **Reliability** to `Best Effort` — see [docs/camera.md](docs/camera.md) |
| Arm does not respond to commands | Wrong `--iface`, or motors not enabled | See [Arm does not respond](#arm-does-not-respond) above |
| `could not find the Go2 on enp0s3` | DHCP changed / discovery failed | `wifi_nav.sh --robot-ip <IP>` — see [Robot IP not found](#robot-ip-not-found--dhcp-changed) |
| `glide REFUSED: no joint feedback` | Arm telemetry not arriving | Check `diag_arm.sh`; never plan a move from a guessed pose |
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
- **D1-T arm control**: direct DDS/JSON commands, smooth speed-limited
  `glide`, gripper control, sequence files, folded-pose calibration
- **One-command orchestration**: `wifi_nav.sh --arm` starts dog and arm,
  with auto relay startup, cleanup, and monitoring
- **DHCP-proof networking**: host IP auto-detection, robot discovery,
  container recreated when the IP changes, arm/dog DDS isolation checks
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
| [`cyclonedds`](https://pypi.org/project/cyclonedds/) `0.10.2` | Python DDS bindings used by the D1-T arm scripts (auto-installed by `start_arm.sh`) |
| [`JeewanthaSadaruwan/unitree-D1-550-Robot-ARM`](https://github.com/JeewanthaSadaruwan/unitree-D1-550-Robot-ARM) | Reference only: ideas behind `glide` and the servo-angle feedback (reimplemented, no code copied) |

---

## License

MIT — see [LICENSE](LICENSE).