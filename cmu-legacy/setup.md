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
| autonomy_stack_go2 | ~/files/autonomy_stack_go2 | External, must be built |
| go2_robot_sdk | ~/files/ros2_ws/src/go2_robot_sdk | External, must be built |

The two external repositories are not part of this repo. Clone and build them before you continue.

## Install CycloneDDS

The configs in config/ target CycloneDDS. Install the ROS 2 middleware package.

```bash
sudo apt update
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

## Clone the Repo

Replace <repo-url> with the URL of your copy of the repo.

```bash
git clone <repo-url> Unitree_GO2_Agricultural
cd Unitree_GO2_Agricultural
```

## Repo Layout

```
Unitree_GO2_Agricultural
|-- config
|   |-- cyclonedds_ethernet.xml
|   |-- cyclonedds_wireless.xml
|   |-- zenoh_robot_config.json5
|   +-- zenoh_vm_config.json5
|-- docs
|-- scripts
|   |-- move_forward.sh
|   |-- system_real_robot_ethernet.sh
|   |-- system_real_robot_wireless.sh
|   |-- system_real_robot_webrtc.sh
|   |-- zenoh_bridge_robot.sh
|   +-- zenoh_bridge_vm.sh
+-- src
    +-- go2_integration_pkg
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

Confirm the external dependencies exist.

```bash
ls ~/files/autonomy_stack_go2
ls ~/files/ros2_ws/src/go2_robot_sdk
```

Both commands must list files. An error means a dependency is missing.

## What Working Looks Like

| Check | Expected result |
|-------|-----------------|
| ros2 pkg list | Includes go2_integration_pkg |
| ls on both external paths | Lists files, no error |
| ls -l scripts | Shell scripts show the x permission |

## Cross-References

- docs/ethernet.md - run the stack over a direct cable
- docs/troubleshooting.md - fixes for build, environment, and connection errors
- [architecture.md](architecture.md) - topic contract, QoS, command IDs, and data flow
---

## Executable Bits on Python Nodes

If you see `No executable found` when running `ros2 run go2_integration_pkg <node>.py`, the source file is missing its `+x` bit.

**Fix:**

```bash
chmod +x src/go2_integration_pkg/go2_integration_pkg/*.py
colcon build --symlink-install --packages-select go2_integration_pkg
source install/setup.bash
```

Verify with `ros2 pkg executables go2_integration_pkg` — you should see six executables. The bits are committed to the repo.

---

## Local Simulation Test — No Robot, No Gazebo

```bash
./scripts/live_mapping_sim.sh
```

Runs `scripts/mock_sim.py`, which publishes synthetic `/registered_scan`, `/state_estimation`, and `/tf` on a circular trajectory. `live_mapping.sh` then runs its normal detection logic against those topics.

Press Ctrl+C to stop. Diagnostics: `log/diagnostics_sim.txt`.
