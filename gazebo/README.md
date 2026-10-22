# Gazebo simulation: Go2 + D1

A Gazebo Harmonic simulation of the Unitree Go2 with the D1 arm on its back, in a
small field with raised beds and plants. The simulator stands in for the real
robot: the **unchanged** project scripts (`wifi_nav.sh`, `start_arm.sh`,
`d1_arm_control.py`, `d1_arm_calibration.py`, the explorer, `diag_arm.sh` …)
run against it.

```bash
./gazebo/setup_gazebo.sh                    # once: models + image + validation
./scripts/runtime/wifi_nav.sh --sim --arm   # the whole stack, simulated
```

Runs on the Ubuntu 22.04 VM, next to the existing `go2-lidar3d` container. Needs
Docker and a desktop session (RViz and the Gazebo window). No GPU required
(software rendering), but it is slow on a VM.

---

## What is simulated

| Real system | Simulation |
|---|---|
| Jetson: Point-LIO → `/lidar3d/odom`, `/lidar3d/registered`, `/lidar3d/path` on DDS domain 64 | `sim_robot_bridge.py` publishes the same topics, same frames (`lidar3d_map`), from Gazebo |
| Leg joints from `/lf/lowstate` | `/joint_states` with a fixed standing pose |
| cmd_vel TCP server on Jetson :17999 (`go2_cmd_vel_tcp.py`) | Same 12-byte protocol on :17999 of this machine → Gazebo |
| Sport API walking | **Gliding base**: the body follows `/cmd_vel` (VelocityControl); legs hold a standing pose, no gait |
| Front camera | Gazebo camera → `/camera/image_raw` (no relay needed) |
| D1 arm over `enp0s8`, DDS domain 0, JSON on `rt/arm_Command` | `d1_arm_emulator.py` speaks the same protocol on interface `simarm0` and drives the Gazebo joints |

The laptop side is **the real one**: `wifi_nav.sh --sim` launches the same
`go2-session.sh start lidar3d-viz` (Nav2 + RViz) that `mapping.sh --3d --nav`
launches, pointed at this machine instead of the robot.

```
 go2-lidar3d container (unchanged)            go2-sim container (this folder)
 ───────────────────────────────              ─────────────────────────────────
 Nav2 + RViz + explorer  ── DDS domain 64 ──► sim_robot_bridge.py ◄─ ros_gz_bridge ◄─ Gazebo
   go2_cmd_vel_tcp client ── TCP :17999 ────►   (cmd_vel server)                     │
 start_arm.sh / d1 scripts ─ DDS domain 0 ──► d1_arm_emulator.py ── gz-transport ────┘
                              on simarm0
```

## Models

`setup_gazebo.sh` downloads the official models into `gazebo/models/` (not in git):

| Model | Source | Notes |
|---|---|---|
| `go2_description` | [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros) `robots/go2_description`, commit `5994d4f`, BSD-3-Clause | URDF + DAE meshes |
| `d1_description` | Unitree D1 gripper-version package (`oss-global-cdn.unitree.com/…/90b2525b….zip`), SHA-256 checked | URDF + STL meshes, 6 joints + 2 fingers |

`sim/build_model.py` combines them (run automatically at start):

- Go2 legs become fixed joints at a standing pose (trunk at ~0.29 m).
- The D1 is mounted on the trunk top (`SIM_D1_MOUNT`, default `0 0 0.057 0 0 0`).
- **The D1 URDF ships `effort=0 velocity=0` on every joint** (the controllers would
  apply no force); real limits are patched in.
- A virtual lidar is placed where the stack's lidar mount puts the sensor, so the
  laptop's `go2_odom_tf` recovers the exact body pose. The mount (`sensor_from_base`)
  is read at every start from go2-nav2-wifi's
  `go2_nav2/config/lidar3d_robot_viz.yaml` (`SIM_SENSOR_YAML`), the file
  `go2_odom_tf` itself uses. With the [lab fix](../docs/LAB-NAV-FIXES.md) applied, the
  lidar sits at the Go2 URDF mount (0.289, 0, −0.047 m). If the file is missing, the
  sim falls back to that same URDF mount and prints a warning.

## Commands

```bash
./gazebo/setup_gazebo.sh            # models + image + gz sdf validation (re-run: --force)
./gazebo/run_sim.sh up              # start; waits for the first lidar frame
./gazebo/run_sim.sh status | logs | shell
./gazebo/run_sim.sh down            # stop; removes simarm0
./gazebo/test_sim.sh [--down]       # automated smoke test (topics, motion, arm)
./scripts/runtime/wifi_nav.sh --sim [--arm] [--no-explore]   # full stack
```

With `wifi_nav.sh --sim`, the simulator is started if needed and stopped on exit
(`SIM_KEEP=1` leaves it running). In RViz: set **2D Pose Estimate** at the robot,
then **Auto Explore: START** or a **2D Goal Pose**, as on the real robot.

Arm against the simulator (the arm "network" is `simarm0`, `192.168.123.51`, arm "at" `.100`):

```bash
export ARM_IFACE=simarm0 ARM_HOST_IP=192.168.123.51
./scripts/runtime/start_arm.sh read
./scripts/runtime/start_arm.sh fold
./scripts/runtime/start_arm.sh enable sleep 1 joint 0 20 sleep 3 feedback
```

`wifi_nav.sh --sim --arm` sets these variables for you. The arm scripts run on
the VM, so the VM needs `pip3 install cyclonedds`.

## Settings

| Variable | Default | |
|---|---|---|
| `SIM_GUI` | `1` | Gazebo window (`0` = headless) |
| `SIM_RENDER_ENGINE` | `ogre2` | Use `ogre` if the window is black on the VM |
| `SIM_ARM` | `1` | Create `simarm0` and run the D1 emulator |
| `SIM_ARM_SUBNET` | `192.168.123` | Change if a real arm adapter already uses it |
| `SIM_SPAWN` | `-4.5 0 0` | Start pose `x y yaw` (headland west of the beds) |
| `SIM_D1_MOUNT` | `0 0 0.057 0 0 0` | D1 pose on the trunk, `x y z roll pitch yaw` |
| `SIM_SENSOR_YAML` | `$NAV2_WIFI_DIR/go2_nav2/config/lidar3d_robot_viz.yaml` | File the lidar mount (`sensor_from_base`) is read from; must match the one the session's `go2_odom_tf` uses |
| `SIM_TIMEOUT` | `240` | Seconds to wait for the first lidar frame |
| `GO2_HOST_IP` | auto (`enp0s3`, else `127.0.0.1`) | DDS address shared with `go2-lidar3d` |

## Simulation differences

These are deliberate. They're listed so test results aren't over-read:

- **No gait.** Body sway, foot slip, stairs, and gait-induced lidar motion aren't simulated.
- **Perfect localization.** `/lidar3d/odom` is ground truth. Point-LIO drift and initialization aren't tested.
- **Lidar.** It's a level 360°×65° scanner (180×16 rays at 10 Hz), not the L1's hemispherical pattern.
- **D1 motion.**
  - Motion is rate-limited to 60°/s, or paced by `delay_ms`. The real controller's own profile is unknown.
  - **Release** holds the pose, because Gazebo joints can't go limp.
  - Motion commands sent before `enable` are acknowledged and ignored.
- **Gripper.** Protocol 0–65° maps to finger travel of 0–30 mm. That's an assumption.
- **Not simulated:** `wifi_mapping.sh` / `mapping.sh` themselves (they drive a real Jetson over SSH), RTAB-Map, and the hyperspectral camera.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `image go2-sim:local missing` | `./gazebo/setup_gazebo.sh` |
| `no lidar frame after 240 s` | Software rendering is slow: `SIM_TIMEOUT=600`, or `SIM_GUI=0`, or `SIM_RENDER_ENGINE=ogre` |
| Gazebo window black or crashes | `SIM_RENDER_ENGINE=ogre ./gazebo/run_sim.sh up` |
| `enp0s8 already uses 192.168.123.x` | Real arm adapter is configured: unplug it, or `SIM_ARM_SUBNET=192.168.124` |
| Arm scripts: `DDS discovery failed` on `simarm0` | Check `ip link show simarm0` shows `MULTICAST,UP`. Then check `./gazebo/run_sim.sh logs` for `D1 ARM EMULATOR READY` |
| RViz dog tilted (~20°) and heading ~120° off in `--sim` | The sim and `go2_odom_tf` use different lidar mounts. Check that `./gazebo/run_sim.sh logs` says `sensor_from_base from /nav2_cfg/lidar3d_robot_viz.yaml`, then restart the sim (`./gazebo/run_sim.sh down`). `tf2_echo lidar3d_map base_link` should show roll and pitch at 0 |
| Headless (`SIM_GUI=0`): camera image black, LiDAR points all at the sensor | `ogre2` headless rendering fails on the VM: `SIM_RENDER_ENGINE=ogre` |
| RViz shows no robot or cloud | `./gazebo/test_sim.sh` shows which topic is missing. The sim and the session must use the same `GO2_HOST_IP` |
| `D1 download checksum mismatch` | Unitree changed the file. Inspect it, then update `D1_SHA256` in `setup_gazebo.sh` |

## Files

| File | Role |
|---|---|
| `setup_gazebo.sh` | One-time: download models, build image, validate |
| `run_sim.sh` | Start/stop the sim container and the `simarm0` interface |
| `test_sim.sh` | Smoke test using the real `go2-lidar3d` container and arm scripts |
| `docker/Dockerfile` | `go2-sim:local`: ROS 2 Humble + Gazebo Harmonic + `ros-humble-ros-gzharmonic` |
| `sim/start_inside.sh` | Container entry: model, Gazebo, spawn, bridges, emulator |
| `sim/build_model.py` | Go2 + D1 → `go2_d1.urdf` + `sim_params.json` (stdlib only) |
| `sim/sim_robot_bridge.py` | The "Jetson": robot topics + cmd_vel TCP server |
| `sim/d1_arm_emulator.py` | The "D1 controller": JSON-over-DDS protocol → Gazebo joints |
| `sim/bridge.yaml` | `ros_gz_bridge` topic map |
| `worlds/agri_field.sdf` | 12 × 8 m field, three planted beds, walls, crates |
