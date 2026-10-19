# Roadmap

[← Back to README](../README.md) · [Status](STATUS.md) · [Tour setup](tour.md) · [Camera](camera.md) · [Troubleshooting](troubleshooting.md)

Two phases. Phase 1 is autonomous navigation and 3D tour capture. Phase 2
is the D1-T arm and crop-scanning pipeline. Each phase has its own success
metric. Do not start Phase 2 work until Phase 1 milestones 0–2 are green.

---

# Phase 1 — Autonomous 3D Tour System

**Goal:** Go2 autonomously explores an unknown space, captures a dense
colored 3D map, and produces a viewable "home tour" that can be replayed
in a standalone viewer.

**Definition of done:**
1. Robot explores a 3×3 m room with no human input after START
2. A dense colored PLY is exported
3. The PLY opens in Open3D and looks like a room
4. Every waypoint click in RViz shows a numbered marker and a planned path line
5. All of the above runs from one launch command

## Milestone 0 — Stabilize the foundation

Everything else depends on this. Do not skip.

| # | Task | Status |
|---|---|---|
| 0.1 | Jetson relay starts in FULL mode (no `--sensors-only`) | ✅ auto-handled by [`wifi_nav.sh`](../scripts/runtime/wifi_nav.sh) |
| 0.2 | Port 17999 conflict eliminated before every session | ✅ auto-handled |
| 0.3 | `mapping.sh --3d --nav` launches a clean session | ✅ verified |
| 0.4 | [`auto_walk.sh`](../scripts/tools/auto_walk.sh) completes one square | ✅ verified |
| 0.5 | DHCP-proof networking: host IP auto-detect, robot discovery, stale-container recreate | ✅ code ([`go2_net.sh`](../scripts/runtime/go2_net.sh)), ⏳ verify on robot |
| 0.6 | Container `numpy<1.25` pin (fixes `numpy.dtype size changed`) | ✅ auto-handled by `wifi_nav.sh` |
| 0.7 | Gazebo simulation of Go2 + D1 for off-robot testing | ✅ code ([`gazebo/`](../gazebo/README.md)), ⏳ first run on the VM (`gazebo/test_sim.sh`) |

**Blocker:** if 0.1–0.2 fail, nothing downstream matters. `wifi_nav.sh`
now SSHes into the Jetson and clears the port before every launch. Manual
fallback documented in [troubleshooting.md](troubleshooting.md).

## Milestone 1 — Visual waypoint system (RViz)

No new code. Configuration only. Code-complete; needs a running RViz to
save the layout. Step-by-step guide: [tour.md](tour.md).

| # | Task | Status |
|---|---|---|
| 1.1 | Add `nav2_rviz_plugins/Navigation2` panel | ⏳ save on campus |
| 1.2 | Enable "Waypoint Mode" checkbox | ⏳ |
| 1.3 | Add Path display on `/plan` | ⏳ |
| 1.4 | Add Path display on `/local_plan` | ⏳ |
| 1.5 | Test 3-waypoint route | ⏳ |

**Helper:** [`scripts/tools/save_rviz_config.sh`](../scripts/tools/save_rviz_config.sh) prints the exact steps to
follow while RViz is open. Target file: [`config/tour.rviz`](../config/tour.rviz).

**Output:** drop waypoints, see the route, watch the robot follow it — no
CLI, no `auto_walk.sh`.

## Milestone 2 — Explorer verification

The explorer code ([`frontier_explorer.py`](../scripts/nodes/frontier_explorer.py)) is written and fixed. It has never produced a goal on a
live map. This is the single most important verification in Phase 1.

| # | Task | Status |
|---|---|---|
| 2.1 | Run `frontier_explorer.py` with `blind_enable:=false` | ⏳ verify on robot |
| 2.2 | Confirm RViz START button publishes `/explore/resume` | ⏳ |
| 2.3 | Explorer drives to first frontier | ⏳ |
| 2.4 | Full exploration of 3×3 m room | ⏳ |
| 2.5 | No re-sending of blacklisted goals | ⏳ |

**Fix applied this session:**
- `map_frame` default → `lidar3d_map` (was `map`)
- `map_topic` default → `/global_costmap/costmap` (was `/map`, doesn't exist)
- `min_frontier_size` → `4` (was 8, too strict)
- `stop_topic` → `/cmd_vel` (bypasses smoother so it overrides Nav2)
- Blacklist tag uses map origin offset
- Debug log added: `detect_frontiers: known=N frontiers=K`

**Risk:** if 2.1 shows `goals_sent=0` after enabling, the frontier detector
is finding nothing. The debug log tells you which guard is firing. See
[troubleshooting → Explorer](troubleshooting.md#explorer).

## Milestone 3 — Camera pipeline

Required for reconstruction. Currently blocked by hardware.

| # | Task | Status |
|---|---|---|
| 3.1 | Fix the camera backend on Jetson | ⏳ hardware |
| 3.2 | Verify image reaches VM | ⏳ |
| 3.3 | Mount Intel RealSense D435i (if available) | ⏳ hardware |
| 3.4 | Calibrate extrinsics camera ↔ `base_link` | ⏳ |

**Note:** [`go2_camera_relay.py`](../scripts/nodes/go2_camera_relay.py) is already written and bridges
`/frontvideostream` → `/camera/image_raw`. What is missing is the *camera
backend* on the Jetson — right now the relay logs
`WARN: camera unavailable`. See [camera.md](camera.md).

**Blocker if no RealSense:** built-in camera + LiDAR gives sparse color
only. Enough to prototype, not enough for a real-estate quality tour.

**Fallback:** LiDAR + Point-LIO only. You get a dense *uncolored* point
cloud. Usable for geometry, not for a tour.

## Milestone 4 — RTAB-Map reconstruction

The pipeline that produces the dense 3D tour.

| # | Task | Status |
|---|---|---|
| 4.1 | Install `ros-humble-rtabmap-ros` in the container | ⏳ |
| 4.2 | Launch RTAB-Map alongside the Nav2 session | ✅ [`scripts/runtime/start_rtabmap.sh`](../scripts/runtime/start_rtabmap.sh) written |
| 4.3 | Remap RTAB-Map output to `/tour/*` to avoid `/map` conflict | ✅ in the launch script |
| 4.4 | Manual walk in one room, export PLY | ⏳ hardware |
| 4.5 | Open PLY in Open3D | ✅ [`scripts/tools/export_tour.py`](../scripts/tools/export_tour.py) written |
| 4.6 | Poisson mesh from point cloud | ✅ `export_tour.py` includes this |

**Compute risk:** RTAB-Map is heavier than Point-LIO. If the VM struggles,
run RTAB-Map on a separate machine or throttle the LiDAR input rate.

## Milestone 5 — Capture automation

Make the explorer pause and capture at each frontier.

| # | Task | Status |
|---|---|---|
| 5.1 | Add `spin_360()` to `frontier_explorer.py` | ✅ implemented |
| 5.2 | Call `spin_360()` after each `goals_succeeded` | ✅ gated by `spin_on_goal` param |
| 5.3 | Suppress goal publishing during spin | ✅ (spin blocks the tick loop) |
| 5.4 | Wait for RTAB-Map to process spin frames | ⏳ verify on robot |

**Enable:** `-p spin_on_goal:=true`. Default is `false` so normal
exploration runs are unaffected.

## Milestone 6 — End-to-end autonomous capture

The full pipeline, no human input.

| # | Task | Status |
|---|---|---|
| 6.1 | One launch command runs everything | ⏳ `wifi_nav.sh --capture` flag not yet added |
| 6.2 | Explorer autonomously maps the room | ⏳ |
| 6.3 | RTAB-Map produces dense map | ⏳ |
| 6.4 | Post-processing script runs automatically | ⏳ |
| 6.5 | Web viewer loads the mesh | ⏳ |

**Deliverable for Phase 1 demo:** run `./scripts/runtime/wifi_nav.sh`,
walk away, come back in 5 minutes, open `tour.html`, see the room.

## Milestone 7 — Hardening

Polish for repeated use.

| # | Task | Status |
|---|---|---|
| 7.1 | Pre-flight script checks all layers | ✅ [`scripts/tools/preflight.sh`](../scripts/tools/preflight.sh) written |
| 7.2 | Session logs archived per run | ⏳ |
| 7.3 | Explorer failure recovery | ⏳ |
| 7.4 | Config file for tour settings | ⏳ |
| 7.5 | README section on running a tour | ✅ [`docs/tour.md`](tour.md) written |

---

# Phase 2 — D1-T Arm and Crop Scanning

**Goal:** The D1-T arm holds a hyperspectral camera on a 3D-printed mount
and moves it through a repeatable scanning pattern over crop plants. Data
is captured for later plant-analysis research.

**Definition of done:**
1. Arm responds to commands from the VM container
2. Hyperspectral camera is mounted on the arm's end effector
3. Arm moves the camera through a defined scan path over a plant
4. Captured data is stored in a structured format for analysis
5. The dog and arm can be commanded independently from the same terminal

## Milestone 8 — Arm foundation

| # | Task | Status |
|---|---|---|
| 8.1 | [`d1_arm_control.py`](../scripts/nodes/d1_arm_control.py) written — DDS/JSON control | ✅ written |
| 8.2 | [`start_arm.sh`](../scripts/runtime/start_arm.sh) written — fold / `--read` / `--setup-net` from the VM | ✅ written |
| 8.3 | Python DDS bindings in the container | ✅ `cyclonedds==0.10.2` auto-installed by `start_arm.sh` (no SDK2 needed) |
| 8.4 | Arm reachable from VM container (`enp0s8`, `192.168.123.x`) | ⏳ verify with [`diag_arm.sh`](../scripts/tools/diag_arm.sh) |
| 8.5 | Arm responds to `joint` command | ⏳ hardware |
| 8.6 | Arm responds to `joints` command | ⏳ hardware |
| 8.7 | Joint feedback readable (`angles`) | ⏳ hardware |
| 8.8 | Smooth `glide` (quintic, peak-speed capped, measured start) | ✅ written, simulated; ⏳ hardware |
| 8.9 | Folded-pose calibration ([`d1_arm_calibration.py`](../scripts/nodes/d1_arm_calibration.py)) | ✅ written; ⏳ calibrate on the mount |
| 8.10 | `wifi_nav.sh --arm`: dog + arm from one command, DDS isolated | ✅ written; ⏳ hardware |

**Protocol:** `{"seq":N,"address":1,"funcode":F,"data":{...}}` on
`rt/arm_Command`. `data` is nested JSON, and angles are degrees: funcode 1
single joint, 2 all joints, 5 enable (`mode` 80000) / release (`mode` 0),
7 home. Full table: [d1arm.md](d1arm.md).

**Network:** the arm is wired to the VM's `enp0s8` (`192.168.123.x`, DDS
domain 0). It is kept separate from the Go2's Wi-Fi DDS on `enp0s3`
(domain 64). First test on hardware:

```bash
./scripts/runtime/start_arm.sh --setup-net
ARM_IFACE=enp0s8 ./scripts/tools/diag_arm.sh
docker exec -u root go2-lidar3d env -u CYCLONEDDS_URI \
    python3 /tmp/d1_arm_control.py --iface enp0s8 angles
```

If it fails, see [troubleshooting → D1-T Arm](troubleshooting.md#d1-t-arm).

## Milestone 9 — ROS 2 bridge node

Wrap the raw DDS interface in ROS 2 topics so the arm can be controlled
with the same tooling as the rest of the stack.

| # | Task | Status |
|---|---|---|
| 9.1 | `d1_arm_node.py` subscribes to `/arm/command` (String) | ⏳ |
| 9.2 | Publishes `/arm/joint_states` (JointState) | ⏳ |
| 9.3 | Service interface for enable / home / release | ⏳ |
| 9.4 | Joint trajectory action | ⏳ |

**Why:** the current CLI script works for scripted commands, but a ROS 2
node lets RViz, action clients, and the eventual AI controller drive the
arm through standard interfaces.

## Milestone 10 — RViz panel for the arm

Slider per joint, alongside the existing explorer panel
([`src/go2_explore_panel/`](../src/go2_explore_panel/)).

| # | Task | Status |
|---|---|---|
| 10.1 | C++ panel `go2_arm_panel` with 6 joint sliders | ⏳ |
| 10.2 | Live joint state feedback | ⏳ |
| 10.3 | Home / Enable / Release buttons | ⏳ |
| 10.4 | Load in RViz alongside the explorer panel | ⏳ |

**Alternative (no C++):** use `rqt_reconfigure` and `rqt_joint_trajectory_controller`
if the arm's ROS 2 bridge exposes standard interfaces. This is a much
smaller effort.

## Milestone 11 — Hyperspectral camera mount

Hardware work.

| # | Task | Status |
|---|---|---|
| 11.1 | 3D-print the camera mount for the D1-T end effector | ⏳ design |
| 11.2 | Mount the hyperspectral camera | ⏳ hardware |
| 11.3 | Power and data cable routing along the arm | ⏳ hardware |
| 11.4 | Camera ROS 2 / DDS driver | ⏳ depends on sensor |
| 11.5 | Calibrate camera extrinsics to arm end effector | ⏳ |

**Open question:** what sensor model is the hyperspectral camera? The ROS 2
interface depends entirely on the answer. Document in [camera.md](camera.md)
once known.

## Milestone 12 — Scan routine

| # | Task | Status |
|---|---|---|
| 12.1 | Define a scan path (e.g. linear sweep over a plant) | ⏳ (write it as a `run` sequence file) |
| 12.2 | Play the path with waypoint interpolation | ✅ `glide` + `run <file>` in `d1_arm_control.py`; ⏳ hardware |
| 12.3 | Trigger camera capture at each step | ⏳ |
| 12.4 | Store captures with pose metadata | ⏳ |
| 12.5 | Repeat for multiple plants | ⏳ |

**Expected shape:** the arm moves to N points along a path, dwells at each
for a capture, moves to the next. The routine should be parameterizable
(start pose, end pose, number of steps, dwell time).

## Milestone 13 — Data collection and storage

| # | Task | Status |
|---|---|---|
| 13.1 | Define a per-scan directory layout | ⏳ |
| 13.2 | Save hyperspectral frames + arm pose | ⏳ |
| 13.3 | Metadata file (JSON) per scan | ⏳ |
| 13.4 | Optional: upload to a shared drive | ⏳ |

## Milestone 14 — Dog + arm coordination

The long-term goal. An AI model commands both the dog and the arm.

| # | Task | Status |
|---|---|---|
| 14.1 | Dog navigates to a plant | ⏳ |
| 14.2 | Dog stops and stabilizes | ⏳ |
| 14.3 | Arm executes the scan routine | ⏳ |
| 14.4 | Dog moves to the next plant | ⏳ |
| 14.5 | AI model issues both navigation and arm commands | ⏳ long-term |

---

## Order of work

```
Phase 1
    Milestone 0  ← foundation, must be green
         │
         v
    Milestone 1  ← RViz waypoint setup
         │
         v
    Milestone 2  ← verify explorer produces goals
         │
         v
    Milestone 3  ← camera backend
         │
         v
    Milestone 4  ← RTAB-Map reconstruction
         │
         v
    Milestone 5  ← spin-on-goal capture
         │
         v
    Milestone 6  ← end-to-end tour
         │
         v
    Milestone 7  ← polish

Phase 2 (starts after Phase 1 M2 is green)
    Milestone 8  ← arm responds to commands
         │
         v
    Milestone 9  ← ROS 2 bridge node
         │
         v
    Milestone 10 ← RViz panel
         │
         v
    Milestone 11 ← camera mount (hardware)
         │
         v
    Milestone 12 ← scan routine
         │
         v
    Milestone 13 ← data storage
         │
         v
    Milestone 14 ← dog + arm autonomy
```

## What this roadmap does NOT include

Deferred beyond both phases:

- Multi-floor tours
- Live streaming of the tour to a remote client
- Cloud upload / Matterport-style hosted viewer
- Automatic room segmentation and labeling
- Semantic classification of plants (YOLO on hyperspectral)
- Voice or VLM interface
- Gait modulation (`footRaiseHeight`)

These layer on top of Phase 1 and 2 once both are stable.

---

## Success Metrics

**Phase 1:** a person who has never seen the robot runs one command, waits
10 minutes, and receives a `.ply` file they can open and recognize as the
room the robot just explored.

**Phase 2:** a person who has never seen the arm runs one command, and the
arm moves the hyperspectral camera through a repeatable scan path over a
plant, storing the captured data for later analysis.

Everything in this roadmap serves those two sentences.