# Documentation

[← README](../README.md) · [Status](status.md) · [Changelog](changelog.md) · [Troubleshooting](troubleshooting.md)

Guides for the Unitree Go2 EDU agricultural platform: navigation, 3D mapping and AI, the D1-T
arm, and the simulations used to test them. New to the project? Read [setup](setup/setup.md),
then [architecture](architecture.md), then [status](status.md).

## Project

| Document | Contents |
|---|---|
| [Status](status.md) | Where each part stands: verified on the robot, in simulation only, or open |
| [Changelog](changelog.md) | Session-by-session notes, newest first |
| [Verification checklist](verification_checklist.md) | Robot test checks with commands and expected results; session plan |
| [Roadmap](roadmap.md) | Milestones and what is next |
| [Architecture](architecture.md) | Scope, data flow, network and DDS domains, DHCP discovery |
| [Troubleshooting](troubleshooting.md) | Known problems with their fixes, collision tuning, diagnostics |

## Setup

| Document | Contents |
|---|---|
| [Setup and everyday use](setup/setup.md) | One-time workstation setup, everyday commands, flags and overrides |
| [Scripts reference](setup/scripts.md) | Every runtime script, ROS node and tool |
| [Development](setup/development.md) | Repository layout, lint hooks, archived code, external projects |

## Navigation

| Document | Contents |
|---|---|
| [Frontier explorer](navigation/explorer.md) | Autonomous exploration: running it, goal choice, parameters, results |
| [Drive modes](navigation/drive_modes.md) | `--drive hybrid` (default) / `diff` / `omni`, stuck recovery |
| [Locomotion comparison](navigation/locomotion_comparison.md) | The three drive modes compared in simulation; how to test them on the dog |
| [Voxel costmap](navigation/voxel_costmap.md) | `--voxel`: 3D `VoxelLayer`, front camera + LiDAR fusion, sim results |
| [Drift and shudder](navigation/drift_and_shudder.md) | Why the dog drifted or stepped in place, and the fixes |
| [Waypoint tour](navigation/tour.md) | RViz waypoint mode and tour setup |
| [Lab navigation fixes](navigation/lab_nav_fixes.md) | Fixes applied to upstream go2-nav2-wifi |

## AI and Vision

| Document | Contents |
|---|---|
| [3D digital twin](ai_and_vision/3d_nav.md) | `wifi_3d_nav.sh`: RTAB-Map, web dashboard, YOLO, VLM, rooms, autonomous survey |
| [Live camera](ai_and_vision/camera.md) | Go2 camera relay and who uses the feed |
| [Camera–LiDAR fusion](ai_and_vision/camera_lidar_fusion.md) | Camera calibration, LiDAR overlay check, `--fusion`, `--camera-obstacles` |
| [RealSense D435i](ai_and_vision/realsense.md) | The D435i on the Go2: boot service and topics |
| [Plant detection (3D boxes)](ai_and_vision/plant_detection.md) | YOLO11s + aligned depth on the D435i: `/detections_3d`, `plant_detection.sh` (from the test branch) |

## Hardware

| Document | Contents |
|---|---|
| [D1 arm](hardware/d1_arm.md) | Arm protocol, joint control, calibration, Teach & Playback, Wi-Fi, MoveIt 2 |
| [Traversability handoff](hardware/traversability_handoff.md) | Traversability on the real Go2: measurements, pre-session checks, safety gates |
| [Learned locomotion plan](hardware/learned_locomotion_plan.md) | Plan for per-leg step-over on the real Go2 with a learned walking controller (foot-swing height input) |
| [Step over by crawling](hardware/step_over_crawl.md) | `step_over_crawl.py`: low-level per-leg step-over on the real Go2 (`--plan`, `--stand`; not yet run on the dog) |
| [Arm distance servo test](hardware/arm_servo_test.md) | `arm_servo_test.sh`: RealSense YOLO depth holds the D1 tool 0.5 m from an object |
| [Height drift](hardware/z_drift.md) | Why Point-LIO sinks, the levelling fix, the walk test and `z_drift_monitor.py` |
| [Workstation freezes](hardware/workstation_freeze.md) | The VLM's RAM growth, RViz and Gazebo on the CPU: causes and guards |
| [Real-dog decision test](hardware/real_decision_test.md) | Plan: measure detection, height, label and gait decisions on the real Go2 against ruler-measured objects |

## Simulation

| Document | Contents |
|---|---|
| [Gazebo simulation](simulation/gazebo.md) | Go2 + D1 in Gazebo Harmonic: setup, what is simulated, worlds |
| [GPU rendering](simulation/gpu_rendering.md) | `SIM_GL`: Gazebo on the Intel or NVIDIA graphics instead of the CPU |
| [Indoor lab world](simulation/lab_office.md) | The furnished 7-room lab world and the drive-mode trials |
| [Traversability in simulation](simulation/traversability_sim.md) | Height bands, gait, backstop and semantic layer: the full sim record |
| [Hydroponic station trial](simulation/hydroponic_trial.md) | End-to-end MoveIt 2 + Nav2 mission at a hydroponic rack |
| [Final demo: NFT 4-6 servo](simulation/servo_demo.md) | `run_sim_demo.sh`: our NFT 4-6 with lettuce, RealSense on the arm, distance servo trials, data and graphs |
| [Unity simulation](simulation/unity.md) | The explorer and Nav2 in the CMU Unity sim |

Supporting data: [`simulation/results/`](simulation/results/) (traversability trial tables),
[`simulation/semantic/`](simulation/semantic/) (VLM prompt and model evaluations) and
[`images/`](images/) (figures and their data).

## Conventions

- File names are `lower_snake_case`; each page starts with a title and a navigation line.
- Results state where they were measured: on the robot, in Gazebo, or offline.
- Dated session notes go in the [changelog](changelog.md); [status](status.md) holds only the
  current state.
