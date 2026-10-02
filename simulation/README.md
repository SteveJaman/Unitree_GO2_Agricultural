# Simulation

Simulation support for the Unitree GO2 Agricultural integration.

## What lives here

This folder is a **placeholder** for simulation-related resources in
this repo. The actual Unity simulation environment is maintained by
the CMU autonomy stack and lives outside this repo. See "Where the
Unity environment lives" below.

| Path | Purpose |
|---|---|
| `custom_scenes/` | Reserved for future custom Unity scenes, if needed |
| `README.md` | This file |

## Where the Unity environment lives

The CMU autonomy stack ships a pre-built Unity environment model:

    ~/files/autonomy_stack_go2/src/base_autonomy/vehicle_simulator/
        mesh/unity/
        ├── environment/
        │   ├── Model_Data/
        │   ├── Model.x86_64
        │   ├── UnityPlayer.so
        │   └── ...
        ├── map.ply
        └── ...

Download it from the CMU autonomy stack README:
https://github.com/jizhang-cmu/autonomy_stack_go2

## How to run the simulation

    cd ~/files/Unitree_GO2_Agricultural
    chmod +x scripts/*.sh
    ./scripts/system_simulation.sh

The script does three things:

1. Sources ROS 2 Humble and the built workspaces.
2. Launches the Unity environment (`Model.x86_64`) in the background.
3. Waits for the ROS-TCP-Endpoint to come up, then launches the
   CMU autonomy stack (RViz + point_lio + planner + terrain analysis).

The result is two windows:

| Window | Content |
|---|---|
| Unity | Photorealistic 3D environment with the Go2 walking |
| RViz | Top-down grayscale LiDAR map, robot pose, trajectory, path |

## Topics in simulation mode

Unity publishes:

| Topic | Type | Notes |
|---|---|---|
| `/utlidar/cloud` | `sensor_msgs/PointCloud2` | LiDAR sweep |
| `/utlidar/imu` | `sensor_msgs/Imu` | IMU samples |
| `/camera/image/raw` | `sensor_msgs/Image` | RGB camera |
| `/camera/semantic_image/raw` | `sensor_msgs/Image` | Semantic segmentation |

Unity subscribes:

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Velocity command |

The names match what the CMU autonomy stack expects, so no relay
nodes are needed for sensor topics. The `go2_integration_pkg` relay
nodes exist for the WebRTC fallback path only.

## Requirements

- Ubuntu 22.04 with ROS 2 Humble (x86_64)
- CMU autonomy stack cloned and built at `~/files/autonomy_stack_go2`
- This repo built (`colcon build --symlink-install`)
- Unity environment model extracted at the path shown above
- `Model.x86_64` marked executable (`chmod +x`)

## Why not MuJoCo or Gazebo

The CMU autonomy stack was designed against Unity. Its `system_simulation.launch`
file, its RViz config, and its sensor topic conventions all assume Unity
as the simulator. Using Unity avoids any topic remapping or physics tuning.

MuJoCo and Gazebo would both work as physics backends but require rewriting
the integration layer. Unity is the path of least resistance and matches
what the CMU team supports.