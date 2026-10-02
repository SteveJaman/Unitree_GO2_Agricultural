# Unitree_GO2_Agricultural

Integration layer between the Unitree Go2 EDU robot and the CMU autonomy_stack_go2 SLAM and planner stack.

## Overview

This repo provides orchestration scripts, DDS configuration, a mapping node, and per-mode launch files.

It does not contain the autonomy stack or the WebRTC SDK. Clone those separately. See External Dependencies.

## Supported Connection Modes

| Mode | Command | LiDAR rate | Status |
|------|---------|------------|--------|
| Ethernet | `./scripts/system_real_robot_ethernet.sh` | 14.7 Hz | Working |
| Unity simulation | `./scripts/system_simulation.sh` | ~14 Hz | Primary dev workflow |
| Unity + mapping | `./scripts/system_simulation_with_mapping.sh` | ~14 Hz | Full pipeline |
| WebRTC (legacy) | `go2_robot_sdk` + relay nodes | ~1 Hz | Degraded, last resort |
| Dog direct (no autonomy) | `ros2 run go2_integration_pkg map_node.py` | ~14 Hz | Mapping only |

## Quick Start

Prerequisites:

- Ubuntu 22.04 with ROS 2 Humble
- Unitree Go2 EDU (SDK-enabled)
- Ethernet cable (or USB Wi-Fi dongle on Jetson for wireless)
- CMU Unity model downloaded for simulation

Run these three steps. Source ROS 2 first so the build finds its environment.

```bash
# 1. Clone the repo
git clone https://github.com/SteveJaman/Unitree_GO2_Agricultural.git && cd Unitree_GO2_Agricultural

# 2. Build the integration package
source /opt/ros/humble/setup.bash && colcon build --packages-select go2_integration_pkg && source install/setup.bash

# 3. Run simulation with mapping
./scripts/system_simulation_with_mapping.sh
```

The simulation needs autonomy_stack_go2 in place. See External Dependencies.

## Repository Structure

```
Unitree_GO2_Agricultural/
|-- config/
|   |-- cyclonedds_ethernet.xml
|   +-- cyclonedds_wireless.xml
|-- docs/
|   |-- ETHERNET.md
|   |-- ROADMAP.md
|   |-- SETUP.md
|   |-- TROUBLESHOOTING.md
|   +-- WIRELESS.md
|-- scripts/
|   |-- move_forward.sh
|   |-- system_real_robot_ethernet.sh
|   |-- system_simulation.sh
|   |-- system_simulation_with_mapping.sh
|   +-- verify_topics.sh
|-- simulation/
|   |-- README.md
|   +-- custom_scenes/
+-- src/go2_integration_pkg/
    |-- CMakeLists.txt
    |-- package.xml
    |-- go2_integration_pkg/
    |   |-- __init__.py
    |   |-- cloud_relay_node.py
    |   |-- cmd_vel_bridge.py
    |   |-- map_node.py
    |   |-- move_forward.py
    |   +-- core/
    |       |-- __init__.py
    |       +-- mapping.py
    +-- launch/
        |-- integrated_robot.launch.py
        +-- mapping.launch.py
```

## External Dependencies

These projects are cloned separately. They are not part of this repo.

| Project | Location | Purpose |
|---------|----------|---------|
| `autonomy_stack_go2` | `~/files/autonomy_stack_go2` | CMU SLAM + planner |
| `go2_robot_sdk` | `~/files/ros2_ws/src/go2_robot_sdk` | WebRTC fallback |

Clone and build each one. Replace the angle-bracket placeholders with the repository URLs for your copies.

```bash
# autonomy_stack_go2
git clone <autonomy-stack-go2-url> ~/files/autonomy_stack_go2
cd ~/files/autonomy_stack_go2
# Build with the steps in that repository's own documentation

# go2_robot_sdk
git clone <go2-robot-sdk-url> ~/files/ros2_ws/src/go2_robot_sdk
cd ~/files/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_robot_sdk
```

## Scripts Reference

| Script | Purpose |
|--------|---------|
| `scripts/system_real_robot_ethernet.sh` | Run the autonomy stack on the real robot over Ethernet |
| `scripts/system_simulation.sh` | Run the autonomy stack against the Unity simulation |
| `scripts/system_simulation_with_mapping.sh` | Run the Unity simulation with the mapping node |
| `scripts/move_forward.sh` | Send a forward motion command to test the motion path |
| `scripts/verify_topics.sh` | Check the ROS 2 topics used by the system |

## Mapping

The mapping node builds maps from the LiDAR cloud and odometry. It auto-detects the cloud and odom topics across all modes.

It produces:

- A 2D occupancy grid using log-odds Bayesian updates
- A 3D point cloud accumulated with voxel downsampling
- A Poisson mesh reconstructed with Open3D
- Optional RGB colour fusion from the camera

Press Ctrl+C to save the map. Files land in `~/go2_maps/<timestamp>/` as PNG, PLY, OBJ, and NPY.

## Topic Contract

| Topic | Type | Direction |
|-------|------|-----------|
| `/utlidar/cloud` | `sensor_msgs/PointCloud2` | in (dog) |
| `/utlidar/cloud_deskewed` | `sensor_msgs/PointCloud2` | in (dog, preferred) |
| `/utlidar/imu` | `sensor_msgs/Imu` | in (dog) |
| `/utlidar/robot_odom` | `nav_msgs/Odometry` | in (dog) |
| `/registered_scan` | `sensor_msgs/PointCloud2` | internal (CMU) |
| `/state_estimation` | `nav_msgs/Odometry` | internal (CMU) |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | out |
| `/map/occupancy` | `nav_msgs/OccupancyGrid` | out (TRANSIENT_LOCAL) |
| `/map/points` | `sensor_msgs/PointCloud2` | out |
| `/api/sport/request` | `unitree_api/Request` | out (motion) |

## Documentation

| File | Description |
|------|-------------|
| `docs/SETUP.md` | First-time clone, dependency, and build setup |
| `docs/ETHERNET.md` | Run the stack over a direct Ethernet cable |
| `docs/WIRELESS.md` | Run the stack over a USB Wi-Fi dongle with native DDS |
| `docs/TROUBLESHOOTING.md` | Fixes for environment, DDS, SLAM, motion, and network errors |
| `docs/ROADMAP.md` | Development phases and planned work |

## Roadmap

See `docs/ROADMAP.md`. It lays out the development phases for the project.

## Status

Phase 0 is mostly complete. Phase 1 code is written but not yet verified on the workstation.

## License

MIT