# Session Status

[← Back to README](../README.md) · [Roadmap](roadmap.md) · [Troubleshooting](troubleshooting.md)

Last updated: 2026-10-23

---

## What's complete (code)

Everything below is written, committed, and wired together. None of it
has been verified on the robot yet except where noted.

### Repository structure

```
scripts/
├── runtime/     wifi_nav.sh, wifi_3d_nav.sh, wifi_mapping.sh, start_rtabmap.sh, start_arm.sh,
│                go2_net.sh (sourced: IP detection / robot discovery)
├── nodes/       frontier_explorer.py, go2_camera_relay.py, go2_floor_filter.py, go2_click_to_goal.py,
│                d1_arm_control.py, d1_arm_calibration.py, direct_test_movement.py
└── tools/       auto_walk.sh, calibrate_arm.py, campus_session.sh, diag_arm.sh, export_tour.py,
                 preflight.sh, save_rviz_config.sh, test_explore.sh,
                 test_goal_cli.sh, tune_costmap.sh, verify_costmap.sh
```

Browse: [`scripts/runtime/`](../scripts/runtime/) · [`scripts/nodes/`](../scripts/nodes/) · [`scripts/tools/`](../scripts/tools/)

- CMU-era code archived under [`cmu-legacy/`](../cmu-legacy/)
- Pre-commit hooks configured (ruff, shellcheck, clang-format, markdownlint)
- [README](../README.md) rewritten for the current stack
- Docs: [`camera.md`](camera.md), [`d1arm.md`](d1arm.md), [`roadmap.md`](roadmap.md), [`tour.md`](tour.md), [`3d_nav.md`](3d_nav.md), [`troubleshooting.md`](troubleshooting.md)

### Navigation — verified working

- VM container `go2-lidar3d` running, DDS bound to the VM's Wi-Fi address
  (verified when the lab network used `192.168.1.4` ↔ robot `192.168.1.9`;
  addresses are now resolved at runtime, see below)
- Lab-nav-fixes applied (RPP controller, velocity smoother, Go2 URDF lidar mount)
- Nav2 planner + controller + smoother reach `active`
- `/goal_pose` → `/cmd_vel_nav` → `/cmd_vel` chain publishing
- `/cmd_vel` produces real non-zero velocities (0.3 m/s observed)
- TF `lidar3d_map → base_link` resolves continuously
- [`auto_walk.sh`](../scripts/tools/auto_walk.sh) completes a square — **motion path proven**

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
Code: [`frontier_explorer.py`](../scripts/nodes/frontier_explorer.py).

### Camera — code complete, untested

[`go2_camera_relay.py`](../scripts/nodes/go2_camera_relay.py) bridges `/frontvideostream` (custom Unitree msg) →
`/camera/image_raw` (standard `sensor_msgs/Image`). Launches automatically
via `wifi_nav.sh` unless `--no-camera`. See [camera.md](camera.md).

### DHCP networking — code complete, untested on robot (2026-10-18)

- [`go2_net.sh`](../scripts/runtime/go2_net.sh): host IP auto-detected on `enp0s3`;
  robot found by cache → hostname → MAC prefix scan → SSH-key probe;
  `--host-ip` / `--robot-ip` / `--iface` overrides
- Container recreated when its baked-in host **or robot** IP is stale. `mapping.sh`
  reads both from the container; it fixes a stale host IP itself, not a stale robot IP
- `numpy<1.25` pinned in the container (warns and continues without internet) and in
  the upstream Dockerfiles, including `Dockerfile.lidar3d` (`.dockerignore` files untouched)
- SSH relay start fixed (heredoc; the old `bash -lc` string killed itself via `pkill -f`)
- Relay IP passed under private names, so a stale `GO2_HOST_IP` in the Jetson's login
  profile can no longer redirect the relay
- Discovery `ping` falls back when binding to `enp0s3` isn't permitted
- Container checks no longer use `grep -q` in pipelines (SIGPIPE + `pipefail` misreport)
- `--setup-ssh` implemented directly in `wifi_nav.sh`
- Discovery logic unit-tested against mocked `ip`/`arp-scan` output; `wifi_nav.sh`
  end-to-end tested against fake `docker`/`ssh`/`ip`/Jetson (56 checks)

### D1-T arm — reworked for host-side control (2026-10-20)

Verified config: `enp0s8`, DDS domain 0, **multicast** discovery (unicast
`--peer` breaks on this firmware). The scripts now run on the VM with the pip
`cyclonedds` package, not inside the container.

- [`d1_arm_control.py`](../scripts/nodes/d1_arm_control.py): chain-style DDS/JSON CLI. Commands:
  state, enable, release, home, power, joint, joints, sleep, feedback, status,
  motors, **lift** (shoulder), **sweep** (each joint to ±pct% of its limit).
  Built-in preflight (interface address, route, ping). Removed: `glide`,
  `gripper`, `angles`, `run`
- [`d1_arm_calibration.py`](../scripts/nodes/d1_arm_calibration.py): `--read` folded angles, `--move` fold
  (power on → enable → move → release → power off). `FOLDED_ANGLES` calibrated
- [`calibrate_arm.py`](../scripts/tools/calibrate_arm.py) (new): per-joint offset, commanded vs reported angle
- [`start_arm.sh`](../scripts/runtime/start_arm.sh): preflight (`192.168.123.99` on `enp0s8`, ping the arm),
  then `fold`, `read`, `--calibrate`, or any `d1_arm_control.py` chain. Removed:
  `--read`/`--no-home`/`--setup-net` flags and the no-argument fold
- `wifi_nav.sh --arm`: checks `enp0s8`, runs `start_arm.sh fold` in the background
- [`diag_arm.sh`](../scripts/tools/diag_arm.sh): DDS check updated for the new control output

### Gazebo simulation — code complete, untested in Gazebo (2026-10-18)

[`gazebo/`](../gazebo/README.md): Go2 + D1 from the official Unitree models, Gazebo Harmonic,
`wifi_nav.sh --sim [--arm]`. The robot side (lidar odom/cloud, cmd_vel TCP :17999,
camera, D1 controller) is simulated; the laptop side is the real one.

Verified off-robot: model builder on the real Unitree URDFs (tree, meshes, spawn
height); pose maths against upstream `compose_pose`; cmd_vel TCP server; the D1
emulator against the **unmodified** `d1_arm_control.py` and `d1_arm_calibration.py`
over real CycloneDDS; `wifi_nav.sh --sim` / `run_sim.sh` with a fake harness.
Not yet run: Gazebo itself (needs the Ubuntu VM): run `./gazebo/test_sim.sh`.

### 3D digital twin — code complete, untested (2026-10-21)

[`wifi_3d_nav.sh`](../scripts/runtime/wifi_3d_nav.sh) runs `wifi_nav.sh`
unchanged, plus a separate `go2-3d` container ([`twin/`](../twin/)). The
container runs RTAB-Map (LiDAR, TF odometry from Point-LIO), foxglove_bridge
`:8765` (or rosbridge `:9090`) and
[`go2_click_to_goal.py`](../scripts/nodes/go2_click_to_goal.py)
(`/clicked_point` → nearest safe floor → `/goal_pose`). It copies the session's DDS settings, and
the RTAB-Map DB is saved on every exit. Runbook: [3d_nav.md](3d_nav.md).

Checked off the VM: bash syntax and shellcheck, Python syntax. Not yet run:
the `go2-3d:local` image build, RTAB-Map on live data, the Foxglove
connection, click-to-go, `--export`.

**Camera fusion + YOLO11s (2026-10-23).**
- **Camera fusion:** when `/camera/image_raw` publishes (auto-detected, else
  LiDAR-only), `go2-3d` adds:
  - the `base_link → camera_link` optical TF (Go2 URDF mount);
  - [`go2_camera_info.py`](../scripts/nodes/go2_camera_info.py), approximate
    intrinsics;
  - LiDAR projected into the image as depth, so RTAB-Map runs RGB-D.
- **YOLO11s:** [`twin/yolo_node.py`](../twin/yolo_node.py) runs on the CPU and
  publishes `/yolo/detections`, `/yolo/image`, `/yolo/markers` and
  `/yolo/obstacles`. Obstacles are placed by LiDAR depth, or by the floor plane
  when no LiDAR points fall inside the box.
- **Nav2:** [`add-yolo-costmap-source.sh`](../scripts/tools/add-yolo-costmap-source.sh)
  is an opt-in patch that adds `/yolo/obstacles` to the costmap config.
- **Verified off-robot:**
  - optical TF axes;
  - LiDAR and floor depth for a synthetic object (exact match);
  - box placement;
  - the costmap patch on a sample config (global only, idempotent, undo).
- **Not yet run:** the image build (no Docker on the dev machine), YOLO on
  real frames, the costmap patch on the real upstream file.

### One-command orchestration

[`./scripts/runtime/wifi_nav.sh --arm`](../scripts/runtime/wifi_nav.sh) now handles the entire session:

1. Detects the VM's Wi-Fi IP and discovers the robot's IP
2. Kills stale processes on VM **and** Jetson, cleans up partial runs
3. Recreates the container if its host or robot IP is stale, pins numpy, checks DDS
4. Jetson compute (default): leaves the relay + motion bridge to `mapping.sh`.
   Otherwise SSHes into Jetson, clears port 17999, starts the relay in FULL mode
   and waits for it to confirm
5. (`--arm`) folds the D1 arm in the background over `enp0s8`
6. Launches Nav2 + RViz via `mapping.sh --3d --nav`
7. Starts explorer + camera relay after 20 s
8. On Ctrl+C: waits for an arm fold in progress, then stops everything cleanly

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
| 10 | Arm reachable | `ARM_IFACE=enp0s8 ./scripts/tools/diag_arm.sh` | all PASS |
| 11 | Arm responds to `joint` | `start_arm.sh enable sleep 2 joint 0 20 sleep 3 release` | joint 0 moves 20° |
| 12 | Joint offsets small | `start_arm.sh --calibrate --test-deg 10 --yes` | offsets within a degree or two |
| 13 | Fold calibration | `start_arm.sh read`, paste, `start_arm.sh fold` | arm folds onto mount, goes limp |
| 14 | Robot discovery | `./scripts/runtime/wifi_nav.sh --check` | banner shows `[discovered]` robot IP |
| 15 | Go2 Jetson MAC prefix | `ip link` on the Jetson | starts `02:00:00` (else set `GO2_MAC_PREFIX`) |
| 16 | Save RViz layout | File → Save Config As | `config/tour.rviz` exists |
| 17 | 3D image builds | `./scripts/runtime/wifi_3d_nav.sh --build` | `go2-3d:local` built |
| 18 | 3D stack starts | `./scripts/runtime/wifi_3d_nav.sh --no-explore` | `3D digital twin READY` printed |
| 19 | Dense map grows | Foxglove 3D panel, `/rtabmap/cloud_map` | map fills in as the dog walks |
| 20 | Click-to-go | Foxglove Publish → Point on the floor, then on a desk | `goal (...)` in `click_to_goal.log`; desk click logs `snapped … to free floor`, green arrow beside it |
| 21 | DB saved + export | Ctrl+C, then `wifi_3d_nav.sh --export` | `[3d] Map saved`, PLY in `<db>-export/` |
| 22 | Camera fusion | camera relay up, `wifi_3d_nav.sh` | READY shows `RTAB-Map : camera`; `/rtabmap/cloud_map` coloured |
| 23 | Camera TF / intrinsics | Foxglove: `/lidar3d/registered` over `/yolo/image` region | colours on the right surfaces (else calibrate, roadmap 3.4) |
| 24 | YOLO | stand in front of the dog | `/yolo/image` boxes; `/yolo/markers` `person ~X m` matches a tape measure |
| 25 | YOLO → Nav2 | `add-yolo-costmap-source.sh`, chair in the path | chair shows in `/global_costmap/costmap`, plan goes around it |

---

## Blocked by hardware

| Item | Blocker |
|---|---|
| [`config/tour.rviz`](../config/tour.rviz) | Must be saved from a running RViz on the VM (see [tour.md](tour.md)) |
| Camera backend on Jetson | `WARN: camera unavailable` in relay log |
| Hyperspectral camera mount | 3D-printed attachment + sensor |
| RTAB-Map + RealSense (`wifi_3d_nav.sh --rgbd`) | Depth camera not yet purchased; LiDAR-only and Go2-camera fusion work without it |

---

## Known issues

Full list of fixes: [troubleshooting.md](troubleshooting.md).

### Port 17999 conflict — now auto-handled

`wifi_nav.sh` SSHes into the Jetson and clears the port before launching.
Manual fallback:

```bash
ssh unitree@<robot-ip>       # shown in the wifi_nav.sh banner
pkill -9 -f 'robot-relay-wifi|robot-motion-wifi|go2_cmd_vel_tcp|robot_sport_bridge'
sleep 3
ss -tlnp | grep 17999        # must be empty
```

### Double motion bridge on port 17999 — fixed (2026-10-20)

In `--3d --nav` mode upstream `mapping.sh` starts its own motion bridge on the
Jetson (`robot-motion-wifi.sh`, cmd_vel TCP :17999 + sport bridge). With Jetson
compute (`GO2_LIDAR3D_COMPUTE=jetson`, the default) `wifi_nav.sh` now skips its
own FULL relay, so only one process binds :17999. Other compute modes still get
the FULL relay from `wifi_nav.sh`. If `motion.log` still shows "Address already
in use", run with `--no-relay`.

### `SENSOR-ONLY: no cmd_vel TCP server…` in `slam.log`

**Normal.** That is the sensor relay's own banner. The motion relay is a
separate process started by `mapping.sh --3d --nav`. Ignore this line.

### Explorer `goals_sent=0` in early tests

Root cause was `map_topic=/map` (topic doesn't exist) combined with
`map_frame=map` (frame doesn't exist). Both fixed in the current code.
Verification pending on the robot.

### Collision threshold runtime-only

[`tune_costmap.sh`](../scripts/tools/tune_costmap.sh) sets `inflation_radius` and `footprint_padding` at
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
# 2. Power on the D1 arm if using it (24V DC, RJ45 to the VM's enp0s8)
sudo ip addr add 192.168.123.99/24 dev enp0s8  # after each VM reboot
./scripts/runtime/start_arm.sh --check

# 3. Launch the full stack (drop --arm for navigation only)
cd ~/files/Unitree_GO2_Agricultural
./scripts/runtime/wifi_nav.sh --check          # confirm discovered IPs first
./scripts/runtime/wifi_nav.sh --arm

# 4. In another terminal: verify costmap
go2sh
ros2 topic hz /global_costmap/costmap      # expect ~1 Hz
ros2 topic echo /explore/status            # watch goals_sent

# 5. Click START in RViz
#    Watch explorer log for FRONTIER: lines
#    Watch /explore/status for goals_sent > 0

# 6. Check the arm fold calibration and joint offsets
./scripts/runtime/start_arm.sh read            # paste FOLDED_ANGLES into d1_arm_calibration.py if the mount changed
./scripts/runtime/start_arm.sh --calibrate --test-deg 10 --yes

# 7. Save the RViz layout
./scripts/tools/save_rviz_config.sh
#    Then follow the printed steps in RViz

# 8. Commit any config/tour.rviz that gets saved
```

3D twin (instead of step 3, once `--build` has run):
`./scripts/runtime/wifi_3d_nav.sh --no-explore`, then Foxglove → `ws://<VM IP>:8765`
(see [3d_nav.md](3d_nav.md)).

No robot? Run the same plan against the simulator (`./gazebo/setup_gazebo.sh`
once, then `./gazebo/test_sim.sh` and `./scripts/runtime/wifi_nav.sh --sim --arm`).

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

See [roadmap.md](roadmap.md) for the full milestone list.
