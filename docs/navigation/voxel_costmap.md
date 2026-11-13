# 3D Voxel Costmap and Camera + LiDAR Fusion

[← README](../../README.md) · [Docs index](../README.md) · [Drive modes](drive_modes.md) · [Camera–LiDAR fusion](../ai_and_vision/camera_lidar_fusion.md)

`wifi_3d_nav.sh --voxel` replaces the 3D stack's 2D `ObstacleLayer` with Nav2's
3D `VoxelLayer`; adding `--camera-obstacles` feeds the Go2 front camera's
LiDAR-scaled depth into the same voxel grid. Without `--voxel` the costmap is the
standard one. Added 2026-11-09; run in full Gazebo sessions on 2026-11-11
([Sim sessions](#sim-sessions-2026-11-11)): it works, but **in the sim it explores worse
than the 2D layer** (rooms seen only through a doorway stay unknown), so the 2D layer stays
the default. Not yet run on the robot.

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
| LiDAR source topic | `/lidar3d/obstacle_points` in the file (the YAML said `/lidar3d/registered`) | Makes the file say what runs: `lidar3d_planning.launch.py` and `lidar3d_controller.launch.py` already override the global and local costmaps' LiDAR topic to the self-filtered `/lidar3d/obstacle_points` at launch, so this changes nothing at runtime. `/lidar3d/registered` still contains the robot's own body (about 640 points per scan in sim) |
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

**Camera floor cut with `--voxel`** (fixed 2026-11-11): the camera source marks from
the camera node's 0.08 m floor cut up, and never clears. With the 2D layer every LiDAR
ray clears the whole column, so stray low marks disappear. The `VoxelLayer` clears per
height, and the LiDAR source drops points below 0.15 m before ray-tracing, so camera
marks between 0.08 and 0.15 m were never cleared: in the first sim session they piled
up into a phantom wall across the corridor ("RegulatedPurePursuitController detected
collision ahead!", then no valid path). `wifi_nav.sh` now passes
`nav2_camera_source.py --enable --min-height 0.15` with `--voxel`, the LiDAR's floor cut;
the 2D mode keeps `-10.0`. Covered by the patch tests; **not yet re-run in a sim
session.**

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

## Sim sessions (2026-11-11)

`wifi_3d_nav.sh --sim --world lab_office` with RViz, Gazebo at about 0.55x real time.

**Configuration, live** (`ros2 param get` on the running costmaps, `--voxel --camera-obstacles`):
both the global and the local costmap run `nav2_costmap_2d::VoxelLayer` with
`z_resolution` 0.1, `z_voxels` 16, `mark_threshold` 0, `publish_voxel_map` true,
sources `lidar yolo camera`, LiDAR on `/lidar3d/obstacle_points`, camera marking only.
`/global_costmap/voxel_grid` 4 Hz, `/local_costmap/voxel_grid` 5 Hz,
`/camera/obstacle_points` 3.2 Hz. The RViz converter command above works: 1203 marked
voxels from 0.05 to 1.45 m. After the sessions, a session without the flags restored
`lidar3d_planner.yaml` byte for byte (same SHA-256 as before the tests).

**Driving and exploring** (`tests/sim_benchmark.py`: unattended exploration of the lab,
-10 per collision, -5 per room under 90 % mapped; LiDAR only, no camera):

| Layer | Runs | Scores | Collisions | Floor known | Rooms under 90 % |
|---|---|---|---|---|---|
| 2D `ObstacleLayer` | 1 (2026-11-11) + 3 (2026-11-01) | 100; 100, 100, 100 | 0 | 99.8 % | none |
| `VoxelLayer` (`VOXEL_COSTMAP=1`) | 3 | 90, 90, 90 | 0 | 87.5-88.6 % | N3 conference (44-48 %), S4 office (28-39 %) in every run |

With the voxel layer the robot never collided and drove goals down the corridor (4.3 m
in 30 s), but the two rooms the explorer sees only through their doorways stay
unknown in the global costmap (51 % and 72 % unknown, against 0 % and 2 % with the 2D
layer from the same doorways), so the explorer finds no frontier there and reports
"complete" early. The cause is not established yet: a standalone clearing experiment
(rolling global costmap, sensor at 0.4 m, a room behind a 1 m doorway) was started and
not finished. Likely suspects are how `VoxelLayer` ray-traces from the sensor into
unknown space (per-height clearing, the 0.15 m floor cut of the clearing rays) compared
to the 2D layer's column clearing.

**Conclusion so far:** `--voxel` runs, marks in 3D and does not collide, but it maps
less of the lab than the 2D layer, so do not use it for exploration until the clearing
difference is understood. `--voxel --camera-obstacles` needs a re-run with the camera
floor cut fix.

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
