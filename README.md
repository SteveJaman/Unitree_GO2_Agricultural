# Unitree_GO2_Agricultural

Integration layer for the Unitree Go2 EDU robot. It wires the CMU autonomy_stack_go2 SLAM and planner stack into a full pipeline with a mapping node, RGB camera fusion, SLAM, localisation, and Nav2 navigation.

## Overview

This repo provides orchestration scripts, DDS configuration, ROS 2 nodes, and launch files for mapping and navigation on the Go2 EDU.

It does not contain the autonomy stack or the WebRTC SDK. Clone those separately. See [External Dependencies](#external-dependencies).

## Supported Connection Modes

| Mode | Command | LiDAR rate | Status |
|------|---------|------------|--------|
| Ethernet | `./scripts/system_real_robot_ethernet.sh` | 14.7 Hz | Working |
| Unity simulation | `./scripts/system_simulation.sh` | ~14 Hz | Primary dev workflow |
| Unity + mapping | `./scripts/system_simulation_with_mapping.sh` | ~14 Hz | Full pipeline |
| Unity + camera + mapping | `./scripts/system_simulation_with_camera.sh` | ~14 Hz | Full pipeline with live camera |
| Navigation with localization | `./scripts/system_navigation.sh <map.yaml>` | ~14 Hz | Nav2 + AMCL |
| WebRTC (legacy) | `go2_robot_sdk` + relay nodes | ~1 Hz | Degraded, last resort |
| Dog direct (no autonomy) | `ros2 run go2_integration_pkg map_node.py` | ~14 Hz | Mapping only |

## Quick Start

Run these three steps. Source ROS 2 before the build so it finds its environment.

```bash
# 1. Clone the repo
git clone https://github.com/SteveJaman/Unitree_GO2_Agricultural.git && cd Unitree_GO2_Agricultural

# 2. Build the integration package
source /opt/ros/humble/setup.bash && colcon build --packages-select go2_integration_pkg && source install/setup.bash

# 3. Run simulation with camera
./scripts/system_simulation_with_camera.sh
```

Start `./scripts/system_simulation.sh` in another terminal first. See [Workflows](#workflows).

Full setup steps are in [docs/setup.md](docs/setup.md).

## Repository Structure

```
Unitree_GO2_Agricultural/
|-- config/
|   |-- cyclonedds_ethernet.xml
|   +-- cyclonedds_wireless.xml
|-- docs/
|   |-- architecture.md
|   |-- ethernet.md
|   |-- roadmap.md
|   |-- setup.md
|   |-- troubleshooting.md
|   +-- wireless.md
|-- scripts/
|   |-- move_forward.sh
|   |-- system_navigation.sh
|   |-- system_real_robot_ethernet.sh
|   |-- system_simulation.sh
|   |-- system_simulation_with_camera.sh
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
    |   |-- camera_relay.py
    |   |-- cloud_relay_node.py
    |   |-- cmd_vel_bridge.py
    |   |-- map_node.py
    |   |-- move_forward.py
    |   |-- pointcloud_to_scan.py
    |   +-- core/
    |       |-- __init__.py
    |       +-- mapping.py
    +-- launch/
        |-- integrated_robot.launch.py
        |-- localization.launch.py
        |-- mapping.launch.py
        |-- nav2_bringup.launch.py
        +-- slam_mapping.launch.py
```

## External Dependencies

These projects are cloned separately. They are not part of this repo.

| Project | Location | Purpose |
|---------|----------|---------|
| `autonomy_stack_go2` | `~/files/autonomy_stack_go2` | CMU SLAM + planner |
| `go2_robot_sdk` | `~/files/ros2_ws/src/go2_robot_sdk` | WebRTC fallback (optional) |

Clone and build each one. Replace the angle-bracket placeholders with the repository URLs for your copies.

```bash
# autonomy_stack_go2
git clone <autonomy-stack-go2-url> ~/files/autonomy_stack_go2
cd ~/files/autonomy_stack_go2
# Build with the steps in that repository's own documentation

# go2_robot_sdk (optional, WebRTC fallback only)
git clone <go2-robot-sdk-url> ~/files/ros2_ws/src/go2_robot_sdk
cd ~/files/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_robot_sdk
```

## Nodes Provided

Nodes live in `src/go2_integration_pkg/go2_integration_pkg/`.

| Node | Purpose |
|------|---------|
| `move_forward.py` | Send one forward velocity command via the Unitree Sport API for motion testing |
| `cloud_relay_node.py` | WebRTC fallback: `/point_cloud2` to `/utlidar/cloud` |
| `cmd_vel_bridge.py` | WebRTC fallback: `TwistStamped` to `Twist` conversion |
| `map_node.py` | Log-odds 2D occupancy grid, 3D point cloud accumulation, Poisson mesh reconstruction, optional RGB camera fusion, auto-detection of cloud and odom topics, saves to `~/go2_maps/<timestamp>/` on Ctrl+C |
| `pointcloud_to_scan.py` | Convert 3D `PointCloud2` to 2D `LaserScan` for `slam_toolbox` and `AMCL` |
| `camera_relay.py` | Relay camera image and camera info with QoS matching for RViz |

## Launch Files Provided

Launch files live in `src/go2_integration_pkg/launch/`.

| Launch file | Purpose |
|-------------|---------|
| `integrated_robot.launch.py` | WebRTC fallback: starts relay nodes |
| `mapping.launch.py` | Start `map_node.py` with optional RGB fusion |
| `slam_mapping.launch.py` | Online 2D SLAM with `slam_toolbox` |
| `localization.launch.py` | `map_server` + `AMCL` against a saved map |
| `nav2_bringup.launch.py` | Full Nav2 navigation stack (planner, controller, behaviors, BT navigator) |

## Scripts Reference

Scripts live in `scripts/`.

| Script | Purpose |
|--------|---------|
| `system_real_robot_ethernet.sh` | Run the autonomy stack on the real robot over Ethernet |
| `system_simulation.sh` | Run the autonomy stack against the Unity simulation |
| `system_simulation_with_mapping.sh` | Run the Unity simulation with the mapping node |
| `system_simulation_with_camera.sh` | Run the Unity simulation with the camera and mapping |
| `system_navigation.sh <map.yaml>` | Start Nav2 with AMCL localisation against a saved map |
| `move_forward.sh` | Send a forward motion command to test the motion path |
| `verify_topics.sh` | Check the ROS 2 topics used by the system |

## Mapping and Navigation Capabilities

- **Live camera streaming** via the WebRTC SDK, forwarded to RViz
- **2D occupancy grid** with log-odds Bayesian updates, saved as PNG
- **3D point cloud** accumulated with voxel downsampling, saved as binary PLY
- **Poisson mesh reconstruction** with Open3D, saved as OBJ
- **RGB colour fusion** from the camera, projecting LiDAR points into the image plane
- **Online 2D SLAM** via `slam_toolbox` to build a map live
- **Localisation** via `AMCL` against a saved map
- **Autonomous navigation** via Nav2 (planner, controller, costmaps, recovery behaviors)
- **Auto-detection** of cloud and odom topics across Ethernet, simulation, and dog-direct modes

## Workflows

### Mapping Workflow (Build a New Map)

1. Start the robot. Use the simulation or the real robot over Ethernet.

```bash
./scripts/system_simulation.sh          # or system_real_robot_ethernet.sh
```

2. In a second terminal, start mapping and the camera.

```bash
./scripts/system_simulation_with_camera.sh
```

3. In RViz, add these displays. Drive the robot around to build the map.

```
Map        (/map)
LaserScan  (/scan)
Image      (/camera/image_raw_relayed)
```

4. Save the map.

```bash
ros2 run nav2_map_server map_saver_cli -f ~/go2_maps/my_map
```

### Navigation Workflow (Use a Saved Map)

1. Start the robot.

```bash
./scripts/system_real_robot_ethernet.sh
```

2. In a second terminal, start navigation with the saved map.

```bash
./scripts/system_navigation.sh ~/go2_maps/my_map.yaml
```

3. In RViz, set the initial position with "2D Pose Estimate".
4. In RViz, send the robot to a target with "2D Goal Pose".

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
| `/scan` | `sensor_msgs/LaserScan` | internal (for SLAM) |
| `/map` | `nav_msgs/OccupancyGrid` | out (slam_toolbox) |
| `/map/occupancy` | `nav_msgs/OccupancyGrid` | out (TRANSIENT_LOCAL) |
| `/map/points` | `sensor_msgs/PointCloud2` | out |
| `/camera/image_raw` | `sensor_msgs/Image` | in |
| `/camera/image_raw_relayed` | `sensor_msgs/Image` | internal |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | in |
| `/api/sport/request` | `unitree_api/Request` | out (motion) |

## Prerequisites

- Ubuntu 22.04 with ROS 2 Humble
- Unitree Go2 EDU (SDK-enabled)
- Ethernet cable (or USB Wi-Fi dongle on the Jetson for wireless)
- CMU Unity model downloaded for simulation
- ROS 2 packages: `slam_toolbox`, `nav2_bringup`, `nav2_amcl`, `nav2_map_server`, `nav2_lifecycle_manager`, `nav2_controller`, `nav2_planner`, `nav2_bt_navigator`, `nav2_behaviors`, `cv_bridge`, `tf2_ros`, `image_transport`
- Python: `numpy`, `open3d` (for mesh reconstruction)

## Documentation

All guides live in the [docs](docs/) folder.

| File | Description |
|------|-------------|
| [docs/setup.md](docs/setup.md) | First-time clone, dependency, and build setup |
| [docs/architecture.md](docs/architecture.md) | Topic contract, QoS policies, Sport API command IDs, data flow |
| [docs/ethernet.md](docs/ethernet.md) | Run the stack over a direct Ethernet cable |
| [docs/wireless.md](docs/wireless.md) | Run the stack over a USB Wi-Fi dongle with native DDS |
| [docs/troubleshooting.md](docs/troubleshooting.md) | Fixes for environment, DDS, SLAM, motion, and network errors |
| [docs/roadmap.md](docs/roadmap.md) | Development phases and planned work |

## Roadmap

See [docs/roadmap.md](docs/roadmap.md) for the development phases and planned work.

## Status

Phase 0 is mostly complete. Phase 1 code is written but not yet verified on the workstation. Phase 2 is in progress.

## License

MIT