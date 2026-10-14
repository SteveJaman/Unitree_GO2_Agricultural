# Unitree_GO2_Agricultural

Autonomous exploration and navigation layer for the **Unitree Go2 EDU** quadruped.
This repo sits on top of the [`dancher00/go2-nav2-wifi`](https://github.com/dancher00/go2-nav2-wifi)
stack (Point-LIO SLAM on the Jetson + Nav2 on a laptop VM over Wi-Fi) and adds a
custom frontier-based explorer, an RViz control panel, and orchestration scripts.

---

## Current Status

| Layer | Status |
|---|---|
| Jetson relay (sensor + motion) | ⚠️ Blocked by port 17999 conflict — see [Known Blocker](#known-blocker) |
| Jetson Point-LIO | ✅ Publishing `/lidar3d/registered` @ ~14 Hz |
| VM container `go2-lidar3d` | ✅ Running, DDS bound to `192.168.1.4` |
| Nav2 (planner + controller + smoother) | ✅ `/cmd_vel` @ 20 Hz with real values (0.3 m/s observed) |
| TF `lidar3d_map → base_link` | ✅ Resolves |
| Waypoint navigation (RViz 2D Goal Pose) | ✅ Chain works end-to-end |
| Custom frontier explorer | ⏳ Code deployed, waiting on motion path |
| Floor filter | ⏳ Deployed, not exercised |

**Bottom line:** the VM side (SLAM subscriber, Nav2, smoother, planner, controller,
cmd_vel publishing) is fully working. The only remaining blocker is a stale process
on the Jetson holding port 17999, which prevents the motion relay from starting.

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

Key points:
- **All SLAM runs on the Jetson.** The VM never runs `slam_toolbox` or
  `pointcloud_to_laserscan` — the global frame `lidar3d_map` is produced by
  Point-LIO onboard and forwarded over Wi-Fi.
- **All ROS runs inside Docker** on the VM. Nothing on the host shell sees
  topics. Use `go2sh` or `docker exec`.
- **DDS domain 64** on the Wi-Fi side, domain 0 onboard. The relay bridges them.

---

## Network

| Node | IP | Notes |
|---|---|---|
| Laptop VM | `192.168.1.4` | `GO2_HOST_IP` |
| Go2 Jetson | `192.168.1.9` | `GO2_ROBOT_IP` |
| DDS domain | `64` | Wi-Fi side |
| Relay port | `17999` | TCP, `/cmd_vel` |

---

## Quick Start

### Prerequisites (one time)

```bash
git clone https://github.com/dancher00/go2-nav2-wifi ~/files/go2-nav2-wifi
git clone https://github.com/SteveJaman/Unitree_GO2_Agricultural ~/files/Unitree_GO2_Agricultural

# Apply lab fixes (RPP + velocity smoother + correct lidar mount)
cd ~/files/go2-nav2-wifi
bash ~/files/Unitree_GO2_Agricultural/apply-lab-nav-fixes.sh

# Create the Docker container
export GO2_HOST_IP=192.168.1.4 GO2_ROBOT_IP=192.168.1.9
./setup-lidar3d.sh --laptop
```

### Every session

**Terminal 1 — Jetson: clear stale relays**

```bash
ssh unitree@192.168.1.9
pkill -9 -f 'robot-relay-wifi|robot-motion-wifi|go2_cmd_vel_tcp|robot_sport_bridge'
sleep 3
ss -tlnp | grep 17999        # MUST print nothing
exit
```

**Terminal 2 — VM: launch the stack**

```bash
cd ~/files/go2-nav2-wifi
export GO2_HOST_IP=192.168.1.4 GO2_ROBOT_IP=192.168.1.9
./mapping.sh --3d --nav
```

Stand the dog **before** launching. Wait for RViz to open and the point cloud to
appear. Do **not** interrupt — first 3D map takes ~15 s.

**Terminal 3 — VM: send goals**

```bash
cd ~/files/Unitree_GO2_Agricultural/scripts

./auto_walk.sh                 # scripted 1 m square patrol
# or
./test_goal_cli.sh 1.0 0.0     # single 1 m forward goal
# or click "2D Goal Pose" in RViz
```

---

## Scripts

All scripts live in `scripts/`.

| Script | Purpose |
|---|---|
| `auto_walk.sh` | Drive a scripted square loop via `/goal_pose` (proof of motion path) |
| `test_goal_cli.sh` | Publish a single `PoseStamped` goal from CLI |
| `test_explore.sh` | Bypass the RViz panel and enable the explorer directly; report failure mode |
| `preflight.sh` | Verify every layer (container, env, DDS, plugins, nodes, topic rates) |
| `wifi_nav.sh` | Full orchestrator: pre-flight + `mapping.sh --3d --nav` + explorer/floor filter |
| `wifi_mapping.sh` | Mapping wrapper |
| `wifi_automap.sh` | Roomba-style autonomous mapping |
| `frontier_explorer.py` | Three-phase explorer: blind lead-in → Yamauchi frontier → visited-cell coverage → return |
| `go2_floor_filter.py` | Floor-plane filter, prevents floor being treated as an obstacle |
| `live_mapping.sh` / `live_mapping_sim.sh` | CMU-stack based mapping (legacy path) |
| `system_*.sh` | Legacy CMU autonomy-stack launchers (Ethernet + simulation) |
| `verify_topics.sh` | Check the ROS 2 topics used by the legacy stack |

---

## Known Blocker

**Jetson motion relay fails to start:**

```
OSError: [Errno 98] Address already in use
```

Cause: a stale relay process from a previous session is still holding port
`17999`. The `mapping.sh --3d --nav` session then logs:

```
[go2_cmd_vel_tcp_server]: TCP cmd server 0.0.0.0:17999 -> /cmd_vel ...   ← never reaches this
Traceback ... OSError: [Errno 98] Address already in use
```

Symptoms:
- VM `/cmd_vel` publishes 20 Hz with real values (0.3 m/s)
- VM `/cmd_vel_nav` publishes 10 Hz
- TF `lidar3d_map → base_link` resolves
- **Dog does not move** — commands die at the Wi-Fi boundary

Fix (before every session, on the Jetson):

```bash
ssh unitree@192.168.1.9
pkill -9 -f 'robot-relay-wifi|robot-motion-wifi|go2_cmd_vel_tcp|robot_sport_bridge'
sleep 3
ss -tlnp | grep 17999        # must be empty
```

Then relaunch `./mapping.sh --3d --nav`. The script starts **both** relays
(sensor + motion) itself. **Never** start a relay manually while a session is
running — that is what causes the conflict.

---

## Explorer

The custom explorer (`scripts/frontier_explorer.py`) replaces `explore_lite`.
Four phases:

1. **Blind lead-in** — when the map has < `blind_trigger_cells` known cells,
   publish direct `Twist` on `/cmd_vel_nav` to force new sensor data.
2. **Frontier detection** (Yamauchi 1997) — find free cells adjacent to unknown,
   cluster them, score by `size / distance`, publish the best as a `PoseStamped`
   on `/goal_pose`.
3. **Visited-cell coverage** — when no frontiers remain, drive to the nearest
   unvisited free cell.
4. **Return to origin** — optional, drives back to the starting pose.

Parameters (`--ros-args -p NAME:=VALUE`):

| Param | Default | Notes |
|---|---|---|
| `robot_base_frame` | `base_link` | Must match the TF tree |
| `map_frame` | `map` | **Set to `lidar3d_map` for this stack** |
| `goal_topic` | `/goal_pose` | Consumed by `goal_pose_nav` |
| `cmd_vel_topic` | `/cmd_vel_nav` | Goes through the smoother — do NOT use `/cmd_vel` |
| `blind_enable` | `true` | Disable for first tests — fights Nav2 |
| `coverage_enable` | `true` | Disable for first tests |
| `return_to_init` | `true` | Disable for first tests |
| `min_frontier_size` | `8` | Cells per cluster |
| `min_known_cells` | `200` | Map must have this many known cells before frontier logic runs |
| `blind_trigger_cells` | `100` | Below this, do a blind step |
| `tick_hz` | `1.0` | Main loop rate |

### Running the explorer manually

```bash
docker cp ~/files/Unitree_GO2_Agricultural/scripts/frontier_explorer.py \
    go2-lidar3d:/tmp/frontier_explorer.py

docker exec -u root -it go2-lidar3d bash -lc '
    source /opt/ros/humble/setup.bash
    source /ws/install/setup.bash 2>/dev/null
    python3 /tmp/frontier_explorer.py --ros-args \
        -p robot_base_frame:=base_link \
        -p map_frame:=lidar3d_map \
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

## Repository Structure

```
Unitree_GO2_Agricultural/
├── docs/
│   ├── STATUS.md                 # Session-by-session status notes
│   ├── architecture.md
│   ├── troubleshooting.md
│   └── ...
├── scripts/
│   ├── auto_walk.sh              # Scripted square patrol
│   ├── test_goal_cli.sh          # Single CLI goal
│   ├── test_explore.sh           # Explorer diagnostic
│   ├── preflight.sh              # Full stack check
│   ├── wifi_nav.sh               # Orchestrator
│   ├── frontier_explorer.py      # Custom explorer
│   ├── go2_floor_filter.py       # Floor-plane filter
│   └── ...
├── config/
│   ├── cyclonedds_ethernet.xml
│   └── cyclonedds_wireless.xml
├── src/go2_integration_pkg/      # Legacy CMU-stack integration
└── README.md
```

---

## What This Repo Adds Over Upstream

The upstream `go2-nav2-wifi` provides SLAM (Point-LIO), Nav2, the Wi-Fi relay,
and manual waypoint navigation via RViz. This repo adds:

- **Autonomous frontier exploration** — the explorer node, not in upstream
- **Blind lead-in** for cold-start mapping
- **Visited-cell coverage** pass to fill gaps after frontiers are exhausted
- **RViz control panel** with START/STOP buttons
- **Floor-plane filter** to prevent the ground being treated as an obstacle
- **Orchestration scripts** (`wifi_nav.sh`, `auto_walk.sh`, diagnostics)

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `GO2_CONTAINER: Start the project Docker container first` | Container missing | `cd ~/files/go2-nav2-wifi && ./setup-lidar3d.sh --laptop` |
| `ros2: command not found` on host | ROS runs in the container | Use `go2sh` or `docker exec -u root go2-lidar3d bash -lc '...'` |
| `Invalid frame ID "lidar3d_map"` | Session not up, or Point-LIO not forwarding | Check `mapping.sh --3d --nav` log; check Jetson relay |
| `/cmd_vel` has 20 Hz but dog doesn't move | Jetson port 17999 conflict | Kill stale relay — see [Known Blocker](#known-blocker) |
| Explorer status stays `IDLE` | Button not wired, or `/map` silent | `./test_explore.sh` — it reports which failure mode |
| Explorer `state=EXPLORING, goals_sent=0` | Map too sparse (`< min_known_cells`) | Let map grow; check `ros2 topic hz /map` |
| `SENSOR-ONLY: no cmd_vel TCP server…` in `slam.log` | **Normal** — sensor relay's own banner | Ignore; motion relay is a separate process |
| `Address already in use` in `motion.log` | Stale relay on Jetson | Kill it, restart the session |

---

## External Dependencies

| Project | Purpose |
|---|---|
| [`dancher00/go2-nav2-wifi`](https://github.com/dancher00/go2-nav2-wifi) | SLAM (Point-LIO) + Nav2 + Wi-Fi relay |
| `autonomy_stack_go2` | Legacy CMU stack (Ethernet mode only) |
| Unitree SDK2 / `unitree_api` msgs | Required on the Jetson for the Sport bridge |

---

## Legacy Paths

The `system_*.sh` and `live_mapping*.sh` scripts and the
`src/go2_integration_pkg/` package are from an earlier Ethernet-based workflow
that uses the CMU autonomy stack instead of `go2-nav2-wifi`. They still work
over a direct Ethernet cable but are not part of the autonomous exploration
pipeline described above.

---

## License

MIT
