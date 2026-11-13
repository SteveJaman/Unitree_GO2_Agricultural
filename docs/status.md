# Project Status

[← README](../README.md) · [Docs index](README.md) · [Changelog](changelog.md) · [Verification checklist](verification_checklist.md) · [Roadmap](roadmap.md)

Last updated: 2026-11-13. Where each part of the system stands: verified on the robot,
verified in simulation or offline only, or open. Details and evidence are in the
[changelog](changelog.md); robot test steps are in the
[verification checklist](verification_checklist.md).

---

## Overview

| Area | Robot | Simulation / offline | Docs |
|---|---|---|---|
| Nav2 on the Go2 (`wifi_nav.sh`) | Motion path proven (`auto_walk.sh` square); Nav2 active; TF and `/cmd_vel` chain live | Gazebo and Unity sims | [setup](setup/setup.md), [architecture](architecture.md) |
| Frontier explorer | Not yet verified on the dog (checklist 1–8) | Lab world benchmark 3/3 runs 100/100 (2026-11-01) | [explorer](navigation/explorer.md) |
| Drive modes (`hybrid` default, `diff`, `omni`) | MPPI loads; no movement test yet | 9-trial comparison; `hybrid` best in the lab world | [drive modes](navigation/drive_modes.md), [comparison](navigation/locomotion_comparison.md) |
| 3D localization level (Point-LIO tilt and height) | Levelling node on by default since 2026-11-12; **not yet run on the dog** | Replay of the failing run: tilt ≤ 9° (was 91°); unit tests | [troubleshooting](troubleshooting.md#the-3d-map-tilts-after-a-turn) |
| 3D digital twin and web dashboard | Dashboard live view, camera and battery seen on the robot (2026-10-27) | Hub, settings and survey tested in the sim | [3D twin](ai_and_vision/3d_nav.md) |
| Camera relay | ~4.5 fps to the workstation; relay crash fixed | — | [camera](ai_and_vision/camera.md) |
| Camera calibration | Not done; a 110° lens estimate is used with a warning | Exact intrinsics in the sim | [fusion](ai_and_vision/camera_lidar_fusion.md) |
| Traversability (`--semantic`) | Measured: 4–6 cm real step clearance, no foot-lift API; 3 cm step-over applied; YOLO → 3D box → VLM → semantic obstacles and gait STOP verified standing (2026-11-11). Not yet: driving through objects | Validated at n = 10 per mode | [handoff](hardware/traversability_handoff.md), [sim record](simulation/traversability_sim.md) |
| Voxel costmap (`--voxel`) | Layer and voxel grids verified on the dog with `--semantic` (standing) | Works, 0 collisions, but explores worse than the 2D layer; STVL option not yet run live | [voxel costmap](navigation/voxel_costmap.md) |
| Free Avoid (`--avoid`) and fall recovery | Free Avoid stopped the dog at a box and held distance; not yet watched with Nav2 driving | — | [3D twin](ai_and_vision/3d_nav.md) |
| D1 arm | Signs measured; five-view `inspect_plant` sweep run on the real arm (2026-11-12); arm over Wi-Fi through the Go2 | MoveIt 2, orbit sweep and hydroponic mission in Gazebo | [D1 arm](hardware/d1_arm.md), [hydroponic trial](simulation/hydroponic_trial.md) |
| RealSense D435i | Boot service on the Go2; topics in `go2-lidar3d` | — | [RealSense](ai_and_vision/realsense.md) |

## Open items

- Levelled localization and `--avoid` with Nav2 driving, on the dog.
- Traversability: driving through real objects; the 3 cm step-over limit with real objects.
- STVL (`VOXEL_PLUGIN=stvl`) live test: a person walking past should clear.
- Voxel layer exploration gap: rooms seen only through a doorway stay unknown.
- A real camera calibration (`./scripts/tools/calibrate_camera.sh`).
- Unattended survey does not start the explorer; start it with Auto Survey / Auto Explore.
- Gazebo still mounts the arm backward (`SIM_D1_MOUNT` yaw 180°).
- The gripper roll the operator calls flat is not settled.

## Blocked by hardware

| Item | Blocker |
|---|---|
| [`config/tour.rviz`](../config/tour.rviz) | Must be saved from a running RViz on the workstation (see [tour.md](navigation/tour.md)) |
| Camera backend on Jetson | `WARN: camera unavailable` in relay log |
| Hyperspectral camera mount | 3D-printed attachment + sensor |
| D435i on the arm's end | USB-C extension cable not yet arrived |
| RTAB-Map + RealSense (`wifi_3d_nav.sh --rgbd`) | D435i in hand and streaming tools written; not yet mounted or run with `--rgbd`. LiDAR-only and Go2-camera fusion work without it |

---

## Do not

- Do not run `./setup-lidar3d.sh` without `--laptop` — hangs on the legacy builder
- Do not start a relay manually while a session is up — `wifi_nav.sh` does it
- Do not Ctrl+C `mapping.sh --3d --nav` before RViz finishes loading — leaves a partial session
- Do not run `start_slam.sh` (if it still exists) — inserts a duplicate SLAM layer that conflicts with Point-LIO

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
