# 3D Voxel Costmap and Camera + LiDAR Fusion

[← Back to README](../../README.md) · [Drive modes](drive_modes.md) · [Camera–LiDAR fusion](../ai_and_vision/camera_lidar_fusion.md)

`wifi_3d_nav.sh --voxel` replaces the 3D stack's 2D `ObstacleLayer` with Nav2's
3D `VoxelLayer`; adding `--camera-obstacles` feeds the Go2 front camera's
LiDAR-scaled depth into the same voxel grid. Without `--voxel` the costmap is the
standard one. Added 2026-11-09; checked on a standalone costmap, not yet in a
full Gazebo or robot session.

## What changes

[`scripts/tools/nav2_voxel_layer.py`](../../scripts/tools/nav2_voxel_layer.py)
edits `go2-nav2-wifi`'s `go2_nav2/config/lidar3d_planner.yaml` at session start
(`wifi_nav.sh`, from `VOXEL_COSTMAP=1`); the controller's local costmap is a copy
of that global costmap, so both change. Every edited line is tagged
`# go2-voxel-layer`, and a session without `--voxel` restores the file byte for byte.

| Setting | Value | Why |
|---|---|---|
| `plugin` | `nav2_costmap_2d::VoxelLayer` | Obstacles kept in 3D and cleared by 3D raytracing |
| `z_resolution` × `z_voxels` | 0.1 m × 16 = 1.6 m | `VoxelLayer` allows at most 16 voxels per column; 0.05 m would stop at 0.8 m |
| `max_obstacle_height` | 1.5 m | Tables, shelves and doorframes count; the ceiling does not |
| `mark_threshold` | 0 | A column is marked as soon as one voxel is occupied (the layer marks when the count exceeds the threshold) |
| `publish_voxel_map` | true | `nav2_msgs/VoxelGrid` on `/global_costmap/voxel_grid` and `/local_costmap/voxel_grid` |
| LiDAR source topic | `/lidar3d/obstacle_points` (was `/lidar3d/registered` in the global costmap) | `/lidar3d/registered` still contains the robot's own body (about 640 points per scan in sim), a likely source of phantom obstacles; the local costmap already used the self-filtered cloud |
| LiDAR `min_obstacle_height` | 0.15 m (unchanged) | Keeps the floor out |

The other sources keep their own settings: `yolo` (YOLO boxes) and, with
`--camera-obstacles`, `camera`:
[`go2_camera_obstacles.py`](../../scripts/nodes/go2_camera_obstacles.py) runs Depth
Anything V2 on the front camera, scales it with the LiDAR, and publishes
`/camera/obstacle_points`. The camera source marks but does not clear, so LiDAR
rays through glass never erase what the camera sees. The Go2 front camera has no
depth sensor of its own; the RealSense D435i point cloud is not used here.

`nav2_camera_source.py` was fixed along the way: it only accepted a source list of
exactly `lidar`, so with the YOLO source present (`lidar yolo`) the camera source
could not be added.

## Checked

- [`tests/tools/test_costmap_patches.py`](../../tests/tools/test_costmap_patches.py):
  the voxel, camera and traversability patches on a copy of the real file, in every
  order and across 64 two-session flag combinations; each ends byte for byte where it
  should, and the local costmap the launch file builds has the `VoxelLayer` with the
  LiDAR, YOLO and camera sources.
- [`tests/tools/voxel_costmap_check.py`](../../tests/tools/voxel_costmap_check.py),
  a standalone costmap with that local configuration and synthetic points:

  | Object | VoxelLayer | ObstacleLayer (before) |
  |---|---|---|
  | Table top at 0.75 m, no legs under it | 254 | 254 |
  | 1.4 m tall object | 254 | 254 |
  | One point at 0.5 m | 254 | 254 |
  | 0.10 m item (below the 0.15 m floor cut) | 0 | 0 |
  | Points at 2.0 m (above the grid) | 0 | 0 |

  173 voxels marked in `/costmap/voxel_grid`. Marking is the same as before; what the
  voxel layer adds is clearing per height, which this static check does not exercise.

## Use it

```bash
git pull
./scripts/runtime/wifi_3d_nav.sh --sim --voxel                      # Gazebo, LiDAR voxels
./scripts/runtime/wifi_3d_nav.sh --sim --voxel --camera-obstacles   # + front camera fusion
./scripts/runtime/wifi_3d_nav.sh --voxel --camera-obstacles         # the robot
```

No rebuild is needed: the config is edited before Nav2 starts. `--camera-obstacles`
needs the go2-3d image with the depth model (`wifi_3d_nav.sh --build`) and the camera
calibration (`config/camera/sim_front_camera.yaml` in the sim).

In RViz, show the voxels as a point cloud by converting the grid (in a shell with the
session's DDS settings, `./scripts/tools/ros_shell.sh`):

```bash
ros2 run nav2_costmap_2d nav2_costmap_2d_cloud --ros-args -r voxel_grid:=/local_costmap/voxel_grid
```

then add a PointCloud2 display for `/voxel_marked_cloud`. Back to the 2D layer: run a
session without `--voxel`.

Not done: temporal decay. The spatio-temporal voxel layer (STVL, `decay_time`) is not
installed in the go2-lidar3d image; adding it needs a change to `go2-nav2-wifi`'s
Dockerfile.
