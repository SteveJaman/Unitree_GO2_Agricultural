# Setup

This page takes a developer from a fresh clone to a working build. Finish it before any connection-mode page.

## What This Repo Is

Unitree_GO2_Agricultural is the integration layer between the Unitree Go2 EDU robot and the CMU autonomy_stack_go2 SLAM and planner stack.

The repo holds configs, launch scripts, and the go2_integration_pkg ROS 2 package. It does not hold the autonomy stack or the robot SDK.

## Connection Modes

| Priority | Mode | Use when |
|----------|------|----------|
| 1 | Ethernet | A direct cable is possible. 14.7 Hz LiDAR, most reliable. |
| 2 | Wireless | A USB Wi-Fi dongle is on the Jetson. Native DDS. |
| 3 | Zenoh | Wi-Fi is flaky. TCP-based bridge. |
| 4 | WebRTC | Legacy fallback. ~1 Hz LiDAR, degraded. |

Start with Ethernet. See docs/ethernet.md.

## Prerequisites

| Item | Where | Version |
|------|-------|---------|
| Ubuntu | VM | 22.04 |
| ROS 2 | VM | Humble |
| ROS 2 | Go2 Jetson | Foxy |
| L1 LiDAR firmware | Go2 Jetson | v1.4.0 or newer |
| autonomy_stack_go2 | ~/files/autonomy_stack_go2 | External, must be built |
| go2_robot_sdk | ~/files/ros2_ws/src/go2_robot_sdk | External, must be built |

The two external repositories are not part of this repo. Clone and build them before you continue.

## Install System Dependencies

The package uses CycloneDDS as its middleware and Nav2 for navigation. Install both.

```bash
sudo apt update

# Core ROS 2 middleware
sudo apt install -y \
    ros-humble-rmw-cyclonedds-cpp

# Navigation and mapping stack
sudo apt install -y \
    ros-humble-slam-toolbox \
    ros-humble-nav2-bringup \
    ros-humble-nav2-amcl \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-controller \
    ros-humble-nav2-planner \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-behaviors

# Camera and vision
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf2-ros \
    ros-humble-tf2-py

# Python runtime
sudo apt install -y python3-numpy python3-pip
```

Install the Python-only mesh reconstruction library. This is optional. The mapping node skips mesh export if open3d is missing.

```bash
pip install --user open3d
```

## L1 LiDAR IMU Calibration (One-Time Per Robot)

The IMU inside the L1 LiDAR must be calibrated before SLAM runs. This is a hardware setup step, not part of this software package. Do it once per robot.

### Procedure

SSH into the Go2 Jetson. Place the robot on a flat, level surface and keep it completely still.

```bash
ssh unitree@192.168.123.18
rosrun unilidar_sdk2 imu_calibrator
```

Let it run for about 3 minutes. It collects gyroscope and accelerometer biases and writes the result to `~/.unilidar/imu_bias.yaml`. The L1 driver loads this file automatically on every boot.

### Firmware Check

Verify the L1 firmware version. Firmware below v1.4.0 has a timestamp misalignment bug that causes periodic SLAM drift every 12.7 seconds.

If the version is below v1.4.0, upgrade using Unitree's official `unilidar_firmware_updater` tool. The upgrade takes about 90 seconds.

### Why This Is Not in the Code

Calibration happens upstream of the L1 driver. By the time Point-LIO subscribes to `/utlidar/imu`, the bias corrections are already applied. Adding calibration logic to this repo would be redundant and would conflict with the driver.

## Install CycloneDDS

The configs in config/ target CycloneDDS. Install the ROS 2 middleware package.

```bash
sudo apt update
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

## Clone the Repo

Replace `<repo-url>` with the URL of your copy of the repo.

```bash
git clone <repo-url> Unitree_GO2_Agricultural
cd Unitree_GO2_Agricultural
```

## Repo Layout

```
Unitree_GO2_Agricultural
|-- config
|   |-- cyclonedds_ethernet.xml
|   +-- cyclonedds_wireless.xml
|-- docs
|   |-- ethernet.md
|   |-- roadmap.md
|   |-- setup.md
|   |-- troubleshooting.md
|   +-- wireless.md
|-- scripts
|   |-- move_forward.sh
|   |-- system_navigation.sh
|   |-- system_real_robot_ethernet.sh
|   |-- system_simulation.sh
|   |-- system_simulation_with_camera.sh
|   |-- system_simulation_with_mapping.sh
|   +-- verify_topics.sh
|-- simulation
|   |-- README.md
|   +-- custom_scenes
+-- src
    +-- go2_integration_pkg
        |-- CMakeLists.txt
        |-- package.xml
        |-- go2_integration_pkg
        |   |-- __init__.py
        |   |-- camera_relay.py
        |   |-- cloud_relay_node.py
        |   |-- cmd_vel_bridge.py
        |   |-- map_node.py
        |   |-- move_forward.py
        |   |-- pointcloud_to_scan.py
        |   +-- core
        |       |-- __init__.py
        |       +-- mapping.py
        +-- launch
            |-- integrated_robot.launch.py
            |-- localization.launch.py
            |-- mapping.launch.py
            |-- nav2_bringup.launch.py
            +-- slam_mapping.launch.py
```

## Make the Scripts Executable

Git on some systems drops the executable bit. Set it again.

```bash
chmod +x scripts/*.sh
```

Strip Windows line endings if the files were edited on Windows. A CRLF ending makes bash fail on the first line.

```bash
sed -i 's/\r$//' scripts/*.sh
```

## Build the Integration Package

Source ROS 2 first. The build needs its environment.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_integration_pkg
source install/setup.bash
```

Run the build from the repo root.

## Verify the Install

Confirm ROS 2 finds the package.

```bash
ros2 pkg list | grep go2_integration_pkg
```

Confirm the executables are installed. There must be six.

```bash
ros2 pkg executables go2_integration_pkg
```

Expected output:

```
go2_integration_pkg camera_relay.py
go2_integration_pkg cloud_relay_node.py
go2_integration_pkg cmd_vel_bridge.py
go2_integration_pkg map_node.py
go2_integration_pkg move_forward.py
go2_integration_pkg pointcloud_to_scan.py
```

Confirm the external dependencies exist.

```bash
ls ~/files/autonomy_stack_go2
ls ~/files/ros2_ws/src/go2_robot_sdk
```

Both commands must list files. An error means a dependency is missing.

## What Each Node Does

| Node | Subscribes to | Publishes to | Purpose |
|------|---------------|--------------|---------|
| move_forward.py | none | /api/sport/request | Send one forward command to test motion |
| cloud_relay_node.py | /point_cloud2 | /utlidar/cloud | WebRTC fallback topic rename |
| cmd_vel_bridge.py | /cmd_vel_stamped | /cmd_vel | WebRTC fallback message conversion |
| map_node.py | cloud and odom topics | /map/occupancy, /map/points | 2D grid and 3D cloud mapping |
| pointcloud_to_scan.py | PointCloud2 | /scan | Convert 3D cloud to 2D scan for SLAM |
| camera_relay.py | /camera/image_raw | /camera/image_raw_relayed | QoS-safe camera image relay |

## What Each Launch File Does

| Launch file | Purpose |
|-------------|---------|
| integrated_robot.launch.py | WebRTC fallback relays |
| mapping.launch.py | Start map_node with optional RGB fusion |
| slam_mapping.launch.py | Online 2D SLAM with slam_toolbox |
| localization.launch.py | map_server and AMCL against a saved map |
| nav2_bringup.launch.py | Full Nav2 navigation stack |

## What Working Looks Like

| Check | Expected result |
|-------|-----------------|
| ros2 pkg list | Includes go2_integration_pkg |
| ros2 pkg executables | Lists all six nodes |
| ls on both external paths | Lists files, no error |
| ls -l scripts | Shell scripts show the x permission |

## Cross-References

- docs/ethernet.md - run the stack over a direct cable
- docs/wireless.md - run the stack over Wi-Fi with a USB dongle
- docs/troubleshooting.md - fixes for build, environment, and connection errors
- docs/roadmap.md - development phases and planned work