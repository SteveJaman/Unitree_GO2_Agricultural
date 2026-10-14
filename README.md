# Unitree_GO2_Agricultural

Autonomous exploration and navigation layer for the **Unitree Go2 EDU** quadruped,
with support for the **Unitree D1-T** arm for crop-scanning and data collection.
This repo sits on top of the [`dancher00/go2-nav2-wifi`](https://github.com/dancher00/go2-nav2-wifi)
stack — Point-LIO SLAM running onboard the Jetson, Nav2 running on a laptop VM,
bridged over Wi-Fi — and adds a custom frontier-based explorer, an RViz control
panel, a live camera relay, an arm controller, and a full set of orchestration
scripts.

**One command to run everything:**

```bash
./scripts/runtime/wifi_nav.sh
```

Power on the dog, stand it, run that. The script does the rest.

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

## Architecture

### Navigation

```
Robot (Jetson, 192.168.1.9)              Laptop VM (Ubuntu 22.04, 192.168.1.4)
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
D1-T arm (6 DOF + gripper)
      │
      │ RJ45, DDS
      ▼
rt/arm_Command  ──── JSON payloads ────  d1_arm_control.py
      │                                        │
      │                                        ▼
      ▼                                 start_arm.sh
current_servo_angle  ◄── joint feedback
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

---

## Network

| Node | IP | Notes |
|---|---|---|
| Laptop VM | `192.168.1.4` | `GO2_HOST_IP` |
| Go2 Jetson | `192.168.1.9` | `GO2_ROBOT_IP` |
| D1-T arm | via Go2 onboard switch | DDS on `rt/arm_Command` |
| DDS domain | `64` | Wi-Fi side |
| Relay port | `17999` | TCP, `/cmd_vel` |

The container's CycloneDDS config is generated at startup from `GO2_HOST_IP`.
If that IP is wrong the container announces the wrong address to the robot and
no topics cross the wire.

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

# 3. Create the Docker container
export GO2_HOST_IP=192.168.1.4 GO2_ROBOT_IP=192.168.1.9
./setup-lidar3d.sh --laptop

# 4. Set up passwordless SSH to the robot
cd ~/files/Unitree_GO2_Agricultural
./scripts/runtime/wifi_nav.sh --setup-ssh

# 5. (Optional) Install pre-commit hooks for linting
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

1. Kills stale processes on the VM **and** on the Jetson
2. SSHes into the Jetson, kills any relay holding port 17999
3. Starts `robot-relay-wifi.sh` in FULL mode (sensor + motion)
4. Waits for port 17999 to bind before proceeding
5. Cleans up partial runs on the robot
6. Launches Nav2 + RViz via `mapping.sh --3d --nav`
7. After 20 s: starts the frontier explorer and camera relay
8. On Ctrl+C: stops exploration, kills helpers, stops the Jetson relay, brings down Docker

### Arm — one-time setup

Install the Unitree SDK2 in the container:

```bash
docker exec -u root go2-lidar3d bash -lc '
    pip3 install unitree_sdk2_python
'
```

If pip fails, build from source:

```bash
docker exec -u root go2-lidar3d bash -lc '
    cd /tmp
    git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
    cd unitree_sdk2_python
    export CYCLONEDDS_HOME=/opt/ros/humble
    pip3 install -e .
'
```

### Arm — every session

```bash
# 1. Power the arm (24V DC) and confirm the RJ45 is connected

# 2. Enable motors and home:
cd ~/files/Unitree_GO2_Agricultural
./scripts/runtime/start_arm.sh

# 3. Move a joint directly:
docker exec -u root go2-lidar3d bash -lc '
    python3 /tmp/d1_arm_control.py --iface eth0 joint 0 0.5
'

# 4. Or move all six joints at once:
docker exec -u root go2-lidar3d bash -lc '
    python3 /tmp/d1_arm_control.py --iface eth0 joints 0.0 0.3 0.6 0.0 0.5 0.0
'
```

### Navigation — flags

| Flag | Effect |
|---|---|
| *(none)* | Explore + camera (default) |
| `--no-explore` | Manual waypoint mode (2D Goal Pose only) |
| `--no-camera` | Skip the camera relay |
| `--nav` | Manual waypoint mode, camera on |
| `--plan` | Preview only — no motion, no explore, no camera |
| `--floor-filter` | Add the floor-plane filter |
| `--record` | Save input data for replay |
| `--no-relay` | Skip the auto relay startup (manage Jetson yourself) |
| `--clean` | Clean both ends and exit |
| `--check` | Verify prerequisites only, no launch |
| `--setup-ssh` | One-time passwordless SSH setup |

### Navigation — runtime overrides (no file edits)

Every explorer parameter can be overridden with an env var:

```bash
EXPLORE_BLIND_ENABLE=true        ./scripts/runtime/wifi_nav.sh
EXPLORE_MIN_FRONTIER_SIZE=6      ./scripts/runtime/wifi_nav.sh
EXPLORE_MAP_TOPIC=/global_costmap/costmap_raw ./scripts/runtime/wifi_nav.sh
GO2_ROBOT_IP=192.168.1.42        ./scripts/runtime/wifi_nav.sh
```

---

## Scripts

### Runtime (`scripts/runtime/`)

| Script | Purpose |
|---|---|
| `wifi_nav.sh` | **Main entry point.** Relay + Nav2 + RViz + explorer + camera |
| `wifi_mapping.sh` | Mapping-only wrapper (no explorer) |
| `start_rtabmap.sh` | Launch RTAB-Map for dense 3D reconstruction alongside the session |
| `start_arm.sh` | Initialize the D1-T arm: enable motors, home, verify interface |

### Nodes (`scripts/nodes/`)

| File | Purpose |
|---|---|
| `frontier_explorer.py` | Three-phase explorer: blind lead-in → Yamauchi frontier → visited-cell coverage → return. Optional spin-360 capture for tours |
| `go2_camera_relay.py` | Bridge `/frontvideostream` (custom Unitree msg) → `/camera/image_raw` (standard `sensor_msgs/Image`) |
| `go2_floor_filter.py` | Floor-plane filter, prevents floor being treated as an obstacle |
| `d1_arm_control.py` | Direct control for the D1-T arm via Unitree SDK2 (JSON over DDS) |

### Tools (`scripts/tools/`)

| Script | Purpose |
|---|---|
| `auto_walk.sh` | Scripted 1 m square patrol via `/goal_pose` — proof the motion path works |
| `test_goal_cli.sh` | Publish a single `PoseStamped` goal from the CLI |
| `test_explore.sh` | Diagnose the explorer without the RViz button |
| `preflight.sh` | Layered verification: container, DDS, packages, nodes, topics, TF |
| `campus_session.sh` | All-in-one verify → tune → patch → launch → monitor |
| `tune_costmap.sh` | Change inflation radius and footprint padding at runtime |
| `verify_costmap.sh` | Confirm the costmap topic exists and publishes |
| `save_rviz_config.sh` | Interactive guide to save the RViz layout to `config/tour.rviz` |
| `export_tour.py` | Open3D Poisson mesh reconstruction from an RTAB-Map PLY |

---

## Explorer

The custom explorer (`scripts/nodes/frontier_explorer.py`) replaces `explore_lite`.
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
JSON strings** on the `rt/arm_Command` topic. Joint feedback arrives on
`current_servo_angle`.

### Physical setup

- **Power:** 24V DC via the DC 5.5-2.1 barrel jack
- **Communication:** RJ45, typically connected to the Go2's onboard switch, so
  the arm and robot share a DDS domain
- **Debug:** USB-C (serial debug only)

### Commands

| Action | JSON payload | Purpose |
|---|---|---|
| Home | `{"funcode":7,"address":0,"seq":0,"data":""}` | Return all joints to zero |
| Enable | `{"funcode":5,"address":0,"seq":0,"data":""}` | Power on motors (torque enabled) |
| Release | `{"funcode":5,"address":0,"seq":0,"data":"release"}` | Power off (drag-teach mode) |
| Single joint | `{"funcode":1,"address":X,"seq":0,"data":"Y"}` | Move joint X to angle Y radians |
| All joints | `{"funcode":2,"address":0,"seq":0,"data":"[a0,...]"}` | Move all six joints at once |

### Usage

```bash
# Enable and home (typical session start)
./scripts/runtime/start_arm.sh

# Enable only, skip homing
./scripts/runtime/start_arm.sh --no-home

# Move one joint (joint index, angle in radians)
docker exec -u root go2-lidar3d bash -lc '
    python3 /tmp/d1_arm_control.py --iface eth0 joint 0 0.5
'

# Move all six joints
docker exec -u root go2-lidar3d bash -lc '
    python3 /tmp/d1_arm_control.py --iface eth0 joints 0.0 0.3 0.6 0.0 0.5 0.0
'

# Read current joint angles
docker exec -u root go2-lidar3d bash -lc '
    python3 /tmp/d1_arm_control.py --iface eth0 feedback
'

# Chain commands
docker exec -u root go2-lidar3d bash -lc '
    python3 /tmp/d1_arm_control.py --iface eth0 \
        home sleep 1 enable joints 0 0.3 0.6 0 0.5 0
'
```

### Parameters

| Flag | Default | Notes |
|---|---|---|
| `--iface` | `eth0` | Container interface that reaches the arm. Find with `ip -brief link` inside the container |

### Verifying the arm is reachable

From the container:

```bash
go2sh
ip -brief link              # see which interface the arm is on
python3 /tmp/d1_arm_control.py --iface <iface> feedback
```

From the Jetson:

```bash
ssh unitree@192.168.1.9
ip -brief link
ros2 topic list 2>/dev/null | grep arm
```

If the arm's topics appear in the container, the existing Wi-Fi relay already
carries the DDS traffic and no extra relay is needed. If not, the relay must be
extended to forward the arm's DDS domain, or a second relay added.

### Roadmap for the arm

| Step | Purpose | Status |
|---|---|---|
| 1. Direct control | Prove the arm responds to JSON commands | Script written |
| 2. ROS 2 bridge node | Subscribe to `/arm/command` instead of the raw script | Pending |
| 3. RViz panel | Slider per joint, alongside the explorer panel | Pending |
| 4. Camera mount | 3D-printed attachment for the hyperspectral camera | Hardware |
| 5. Scan routine | Move the camera through a plant-scanning path | Pending |
| 6. AI autonomy | Model commands both the dog and the arm | Long-term |

---

## Live Camera

The Go2's front camera publishes on `/frontvideostream` as a custom
`unitree_go/Go2FrontVideoData` message that RViz can't render. The camera relay
(`scripts/nodes/go2_camera_relay.py`) subscribes to that topic, decodes the
H.264/JPEG payload, and republishes as standard `sensor_msgs/Image` on
`/camera/image_raw`.

The relay launches automatically when `wifi_nav.sh` runs (unless `--no-camera`).

**In RViz:**

1. Click **Add** at the bottom of the Displays panel
2. Select **Image** → OK
3. Set **Topic** to `/camera/image_raw`
4. Set **Reliability** to `Best Effort` (critical — default Reliable will drop frames)
5. **File → Save Config As** to `config/tour.rviz` so it loads next time

Or view outside RViz:

```bash
go2sh
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

See `docs/camera.md` for full setup and troubleshooting.

> **Note:** the hyperspectral camera on the D1-T arm is a separate device from
> the Go2's front camera. Its ROS 2 / DDS interface will be documented in a
> dedicated doc once the physical mount is in place.

---

## Repository Structure

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
│   ├── roadmap.md                    # Phase 1 roadmap
│   ├── tour.md                       # waypoint mode setup
│   └── troubleshooting.md            # standalone troubleshooting
├── scripts/
│   ├── runtime/                      # shell entry points
│   │   ├── wifi_nav.sh               # main orchestrator
│   │   ├── wifi_mapping.sh           # mapping-only wrapper
│   │   ├── start_rtabmap.sh          # RTAB-Map launcher
│   │   └── start_arm.sh              # D1-T arm init (enable + home)
│   ├── nodes/                        # ROS 2 Python nodes
│   │   ├── frontier_explorer.py      # custom explorer
│   │   ├── go2_camera_relay.py       # camera bridge
│   │   ├── go2_floor_filter.py       # floor-plane filter
│   │   └── d1_arm_control.py         # D1-T arm control
│   └── tools/                        # diagnostics and tests
│       ├── auto_walk.sh
│       ├── test_goal_cli.sh
│       ├── test_explore.sh
│       ├── preflight.sh
│       ├── campus_session.sh
│       ├── tune_costmap.sh
│       ├── verify_costmap.sh
│       ├── save_rviz_config.sh
│       └── export_tour.py
├── src/
│   └── go2_explore_panel/            # RViz panel with Auto Explore toggle
└── cmu-legacy/                       # archived CMU-stack code, not used
```

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

### Jetson relay port 17999 conflict

If the motion relay fails with `OSError: [Errno 98] Address already in use`,
a stale process is holding port 17999. **`wifi_nav.sh` handles this
automatically** — it kills stale relays over SSH before starting the new one.

If you need to do it manually:

```bash
ssh unitree@192.168.1.9
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

1. Check the Jetson relay: `ssh unitree@192.168.1.9 'ss -tlnp | grep 17999'`
2. Check the relay log: `ssh unitree@192.168.1.9 'tail -20 /tmp/robot-relay.log'`
3. Confirm FULL mode: the log should say `cmd_vel TCP server started on port 17999`

### Arm does not respond

1. **Check the interface name.** Inside the container: `ip -brief link`. The
   `--iface` flag must match the interface the arm is on.
2. **Check the arm is powered.** 24V DC via the barrel jack. The arm's
   controller should show activity on power.
3. **Check the RJ45.** Arm should be on the same network segment as the Go2.
4. **Verify the SDK2 is installed:**
   ```bash
   docker exec -u root go2-lidar3d bash -lc 'python3 -c "import unitree_sdk2py; print(\"ok\")"'
   ```
5. **Read the joint feedback:**
   ```bash
   docker exec -u root go2-lidar3d bash -lc '
       python3 /tmp/d1_arm_control.py --iface <iface> feedback
   '
   ```
   If feedback arrives but commands do nothing, the motors may not be enabled —
   run `enable` first.

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
| Explorer stays `IDLE` | Button not wired, or costmap silent | `./scripts/tools/test_explore.sh` |
| Camera not showing in RViz | QoS mismatch | Set Image display **Reliability** to `Best Effort` |
| Arm does not respond to commands | Wrong `--iface`, or motors not enabled | See **Arm does not respond** above |
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
- **D1-T arm control** — direct DDS/JSON commands and a launcher
- **One-command orchestration** — auto relay startup, cleanup, monitoring
- **Diagnostic and tuning scripts** — preflight, test_explore, campus_session, tune_costmap
- **RTAB-Map integration** — dense 3D reconstruction and mesh export

---

## Legacy

Earlier work based on the CMU `autonomy_stack_go2` running over Ethernet or
the Unity simulation is archived under `cmu-legacy/` for reference. It is not
part of the active pipeline. See `cmu-legacy/README.md`.

---

## External Dependencies

| Project | Purpose |
|---|---|
| [`dancher00/go2-nav2-wifi`](https://github.com/dancher00/go2-nav2-wifi) | SLAM (Point-LIO) + Nav2 + Wi-Fi relay |
| [`unitree_sdk2_python`](https://github.com/unitreerobotics/unitree_sdk2_python) | DDS interface for the D1-T arm and Go2 Sport API |

---

## License

MIT