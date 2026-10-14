# Session Status

Last updated: 2026-10-14

---

## What's complete (code)

Everything below is written, committed, and wired together. None of it
has been verified on the robot yet except where noted.

### Repository structure

```
scripts/
├── runtime/     wifi_nav.sh, wifi_mapping.sh, start_rtabmap.sh, start_arm.sh
├── nodes/       frontier_explorer.py, go2_camera_relay.py,
│                go2_floor_filter.py, d1_arm_control.py
└── tools/       auto_walk.sh, campus_session.sh, export_tour.py,
                 preflight.sh, save_rviz_config.sh, test_explore.sh,
                 test_goal_cli.sh, tune_costmap.sh, verify_costmap.sh
```

- CMU-era code archived under `cmu-legacy/`
- Pre-commit hooks configured (ruff, shellcheck, clang-format, markdownlint)
- README rewritten for the current stack
- Docs: `camera.md`, `roadmap.md`, `tour.md`, `troubleshooting.md`

### Navigation — verified working

- VM container `go2-lidar3d` running, DDS bound to `192.168.1.4`
- CycloneDDS peers: VM `.4` ↔ robot `.9`
- Lab-nav-fixes applied (RPP controller, velocity smoother, Go2 URDF lidar mount)
- Nav2 planner + controller + smoother reach `active`
- `/goal_pose` → `/cmd_vel_nav` → `/cmd_vel` chain publishing
- `/cmd_vel` produces real non-zero velocities (0.3 m/s observed)
- TF `lidar3d_map → base_link` resolves continuously
- `auto_walk.sh` completes a square — **motion path proven**

### Explorer — code complete, untested

Six fixes applied this session:

| # | Fix |
|---|---|
| 1 | `map_frame` default `map` → `lidar3d_map` |
| 2 | `map_topic` default `/map` → `/global_costmap/costmap` |
| 3 | `min_frontier_size` 8 → 4 |
| 4 | `stop_topic` added → `/cmd_vel` (bypasses smoother, overrides Nav2) |
| 5 | Blacklist tag now uses map origin offset |
| 6 | Debug log: `detect_frontiers: known=N frontiers=K` |

Spin-on-goal integrated, gated by `-p spin_on_goal:=true` (default off).

### Camera — code complete, untested

`go2_camera_relay.py` bridges `/frontvideostream` (custom Unitree msg) →
`/camera/image_raw` (standard `sensor_msgs/Image`). Launches automatically
via `wifi_nav.sh` unless `--no-camera`.

### D1-T arm — code complete, untested

- `d1_arm_control.py` — direct DDS/JSON control
- `start_arm.sh` — enable + home from VM
- Commands: home, enable, release, joint, joints, feedback
- Requires `unitree_sdk2_python` installed in the container

### One-command orchestration

`./scripts/runtime/wifi_nav.sh` now handles the entire session:

1. Kills stale processes on VM **and** Jetson
2. SSHes into Jetson, clears port 17999
3. Starts relay in FULL mode
4. Waits for port 17999 to bind
5. Cleans up partial runs
6. Launches Nav2 + RViz via `mapping.sh --3d --nav`
7. Starts explorer + camera relay after 20 s
8. On Ctrl+C: stops everything cleanly

---

## Pending verification (needs robot)

| # | Check | Command | Expected |
|---|---|---|---|
| 1 | Costmap exists and publishes | `ros2 topic hz /global_costmap/costmap` | ~1 Hz |
| 2 | Explorer subscribes | `ros2 topic info /global_costmap/costmap --verbose` | Sub count = 1 |
| 3 | Frontier detection runs | explorer log | `detect_frontiers: known=N frontiers=K` |
| 4 | Explorer sends goals | `ros2 topic echo /explore/status` | `goals_sent > 0` |
| 5 | Dog moves to frontier | visual | dog walks toward unexplored area |
| 6 | RViz START button publishes | `ros2 topic echo /explore/resume` while clicking | `data: true` |
| 7 | STOP button halts dog | click | stop within 1 s |
| 8 | Full autonomous run | watch | `IDLE → EXPLORING → RETURNING → DONE` |
| 9 | Camera reaches container | `ros2 topic hz /camera/image_raw` | ~10–30 Hz |
| 10 | Arm reachable | `python3 /tmp/d1_arm_control.py --iface <iface> feedback` | joint angles |
| 11 | Arm responds to `joint` | `... joint 0 0.5` | arm moves joint 0 |
| 12 | Save RViz layout | File → Save Config As | `config/tour.rviz` exists |

---

## Blocked by hardware

| Item | Blocker |
|---|---|
| `config/tour.rviz` | Must be saved from a running RViz on the VM |
| Camera backend on Jetson | `WARN: camera unavailable` in relay log |
| Hyperspectral camera mount | 3D-printed attachment + sensor |
| RTAB-Map + RealSense | Depth camera not yet purchased |

---

## Known issues

### Port 17999 conflict — now auto-handled

`wifi_nav.sh` SSHes into the Jetson and clears the port before launching.
Manual fallback:

```bash
ssh unitree@192.168.1.9
pkill -9 -f 'robot-relay-wifi|robot-motion-wifi|go2_cmd_vel_tcp|robot_sport_bridge'
sleep 3
ss -tlnp | grep 17999        # must be empty
```

### `SENSOR-ONLY: no cmd_vel TCP server…` in `slam.log`

**Normal.** That is the sensor relay's own banner. The motion relay is a
separate process started by `mapping.sh --3d --nav`. Ignore this line.

### Explorer `goals_sent=0` in early tests

Root cause was `map_topic=/map` (topic doesn't exist) combined with
`map_frame=map` (frame doesn't exist). Both fixed in the current code.
Verification pending on the robot.

### Collision threshold runtime-only

`tune_costmap.sh` sets `inflation_radius` and `footprint_padding` at
runtime. To persist, edit `go2_nav2/config/lidar3d_planner.yaml` and the
matching override in `go2_nav2/launch/lidar3d_controller.launch.py`.

---

## Do NOT do

- Do not run `./setup-lidar3d.sh` without `--laptop` — hangs on the legacy builder
- Do not start a relay manually while a session is up — `wifi_nav.sh` does it
- Do not Ctrl+C `mapping.sh --3d --nav` before RViz finishes loading — leaves a partial session
- Do not run `start_slam.sh` (if it still exists) — inserts a duplicate SLAM layer that conflicts with Point-LIO

---

## Next session plan

```bash
# 1. Power on the dog, stand it up
# 2. Power on the D1 arm if using it (24V DC, RJ45 connected)

# 3. Launch the full stack
cd ~/files/Unitree_GO2_Agricultural
./scripts/runtime/wifi_nav.sh

# 4. In another terminal: verify costmap
go2sh
ros2 topic hz /global_costmap/costmap      # expect ~1 Hz
ros2 topic echo /explore/status            # watch goals_sent

# 5. Click START in RViz
#    Watch explorer log for FRONTIER: lines
#    Watch /explore/status for goals_sent > 0

# 6. If explorer works, test the arm
docker exec -u root go2-lidar3d bash -lc '
    python3 /tmp/d1_arm_control.py --iface <iface> feedback
'

# 7. Save the RViz layout
./scripts/tools/save_rviz_config.sh
#    Then follow the printed steps in RViz

# 8. Commit any config/tour.rviz that gets saved
```

---

## Success criteria

**Phase 1 (navigation):**
> A person who has never seen the robot runs one command, waits 10 minutes,
> and receives a `.ply` file they can open and recognize as the room the
> robot just explored.

**Phase 2 (arm):**
> A person who has never seen the arm runs one command, and the arm moves
> the hyperspectral camera through a repeatable scan path over a plant,
> storing the captured data for later analysis.

See `docs/roadmap.md` for the full milestone list.