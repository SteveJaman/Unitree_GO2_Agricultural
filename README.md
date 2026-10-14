# Unitree_GO2_Agricultural

Autonomous exploration and navigation layer for the **Unitree Go2 EDU** quadruped.
This repo sits on top of the [`dancher00/go2-nav2-wifi`](https://github.com/dancher00/go2-nav2-wifi)
stack — Point-LIO SLAM running onboard the Jetson, Nav2 running on a laptop VM,
bridged over Wi-Fi — and adds a custom frontier-based explorer, an RViz control
panel, a live camera relay, and a full set of orchestration scripts.

**One command to run everything:**

```bash
./scripts/runtime/wifi_nav.sh
```

Power on the dog, stand it, run that. The script does the rest.

---

## Architecture

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

Key facts:
- **SLAM runs on the Jetson.** The VM never runs `slam_toolbox`. The global
  frame is `lidar3d_map`, produced by Point-LIO onboard.
- **All ROS runs inside Docker** on the VM. The host shell sees no topics.
  Use `go2sh` or `docker exec -u root go2-lidar3d bash -lc '…'`.
- **DDS domain 64** on the Wi-Fi side, domain 0 onboard. The relay bridges them.
- **The explorer reads from `/global_costmap/costmap`**, not `/map`. The
  upstream stack does not publish a `/map` topic — the Nav2 costmap is the
  occupancy grid source.

---

## Network

| Node | IP | Notes |
|---|---|---|
| Laptop VM | `192.168.1.4` | `GO2_HOST_IP` |
| Go2 Jetson | `192.168.1.9` | `GO2_ROBOT_IP` |
| DDS domain | `64` | Wi-Fi side |
| Relay port | `17999` | TCP, `/cmd_vel` |

The container's CycloneDDS config is generated at startup from `GO2_HOST_IP`.
If that IP is wrong the container announces the wrong address to the robot and
no topics cross the wire.

---

## Quick Start

### One-time setup (per workstation)

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

# 4. Set up passwordless SSH to the robot (once)
cd ~/files/Unitree_GO2_Agricultural
./scripts/runtime/wifi_nav.sh --setup-ssh
```

### Every session

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

### What the script does automatically

1. Kills stale processes on the VM **and** on the Jetson
2. SSHes into the Jetson, kills any relay holding port 17999
3. Starts `robot-relay-wifi.sh` in FULL mode (sensor + motion)
4. Waits for port 17999 to bind before proceeding
5. Cleans up partial runs on the robot
6. Launches Nav2 + RViz via `mapping.sh --3d --nav`
7. After 20 s: starts the frontier explorer and camera relay
8. On Ctrl+C: stops exploration, kills helpers, stops the Jetson relay, brings down Docker

### Flags

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

### Runtime overrides (no file edits)

Every explorer parameter can be overridden with an env var:

```bash
EXPLORE_BLIND_ENABLE=true        ./scripts/runtime/wifi_nav.sh   # turn blind lead-in on
EXPLORE_MIN_FRONTIER_SIZE=6      ./scripts/runtime/wifi_nav.sh   # different frontier size
EXPLORE_MAP_TOPIC=/global_costmap/costmap_raw ./scripts/runtime/wifi_nav.sh
GO2_ROBOT_IP=192.168.1.42        ./scripts/runtime/wifi_nav.sh   # different robot IP
```

---

## Scripts

### Runtime (`scripts/runtime/`)

| Script | Purpose |
|---|---|
| `wifi_nav.sh` | **Main entry point.** Relay + Nav2 + RViz + explorer + camera |
| `wifi_mapping.sh` | Mapping-only wrapper (no explorer) |

### Nodes (`scripts/nodes/`)

| File | Purpose |
|---|---|
| `frontier_explorer.py` | Three-phase explorer: blind lead-in → Yamauchi frontier → visited-cell coverage → return |
| `go2_camera_relay.py` | Bridge `/frontvideostream` (custom Unitree msg) → `/camera/image_raw` (standard `sensor_msgs/Image`) |
| `go2_floor_filter.py` | Floor-plane filter, prevents floor being treated as an obstacle |

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
        -p blind_enable:=false \
        -p coverage_enable:=false \
        -p return_to_init:=false \
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

---

## Repository Structure

```
Unitree_GO2_Agricultural/
├── README.md
├── LICENSE
├── .gitignore
├── config/
│   └── README.md                     # will hold tour.rviz when saved
├── docs/
│   ├── STATUS.md                     # session notes
│   ├── tour.md                       # waypoint mode setup guide
│   └── roadmap.md                    # Phase 1 roadmap
├── scripts/
│   ├── runtime/                      # Shell entry points
│   │   ├── wifi_nav.sh               # main orchestrator
│   │   └── wifi_mapping.sh           # mapping-only wrapper
│   ├── nodes/                        # ROS 2 Python nodes
│   │   ├── frontier_explorer.py      # custom explorer
│   │   ├── go2_camera_relay.py       # camera bridge
│   │   └── go2_floor_filter.py       # floor-plane filter
│   └── tools/                        # Diagnostics and tests
│       ├── auto_walk.sh
│       ├── test_goal_cli.sh
│       ├── test_explore.sh
│       ├── preflight.sh
│       ├── campus_session.sh
│       ├── tune_costmap.sh
│       └── verify_costmap.sh
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
a stale process is holding port 17999. **`wifi_nav.sh` now handles this
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

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `GO2_CONTAINER: Start the project Docker container first` | Container missing | `cd ~/files/go2-nav2-wifi && ./setup-lidar3d.sh --laptop` |
| `ros2: command not found` on host | ROS runs inside Docker | Use `go2sh` or `docker exec -u root go2-lidar3d bash -lc '…'` |
| `Invalid frame ID "lidar3d_map"` | Session not up, or Point-LIO not forwarding | Check `mapping.sh --3d --nav` log; check Jetson relay |
| `/cmd_vel` has 20 Hz but dog doesn't move | Jetson port 17999 conflict | `wifi_nav.sh` handles it; verify with `ssh unitree@192.168.1.9 'ss -tlnp \| grep 17999'` |
| Explorer stays `IDLE` | Button not wired, or costmap silent | `./scripts/tools/test_explore.sh` |
| Explorer `state=EXPLORING, goals_sent=0` | Costmap too sparse, or frame mismatch | `ros2 topic hz /global_costmap/costmap`; check explorer log |
| `SENSOR-ONLY: no cmd_vel TCP server…` in `slam.log` | **Normal** — sensor relay's own banner | Ignore; motion relay is a separate process |
| Camera not showing in RViz | QoS mismatch | Set Image display **Reliability** to `Best Effort` |
| Camera relay crashes with "frame not JPEG-decodable" | Firmware ships raw H.264 | Try `-p resolution:=360p` in the relay launch |
| RViz renders slowly | CPU rendering | Close the point cloud display while tuning; RViz is CPU-bound |

---

## What This Repo Adds Over Upstream

The upstream `go2-nav2-wifi` provides SLAM (Point-LIO), Nav2, the Wi-Fi relay,
and manual waypoint navigation via RViz. This repo adds:

- **Autonomous frontier exploration** — the explorer node, not in upstream
- **Blind lead-in** for cold-start mapping
- **Visited-cell coverage** pass to fill gaps after frontiers are exhausted
- **RViz control panel** with START/STOP buttons
- **Live camera relay** — `/frontvideostream` → `/camera/image_raw`
- **Floor-plane filter** to prevent the ground being treated as an obstacle
- **One-command orchestration** — auto relay startup, cleanup, and monitoring
- **Diagnostic scripts** — preflight, test_explore, campus_session

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

---

## License

MIT