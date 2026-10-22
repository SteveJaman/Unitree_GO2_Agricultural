# config/

| File | Purpose |
|---|---|
| `tour.rviz` | RViz layout with Nav2 waypoint panel, global/local path displays, and costmap. Load with `rviz2 -d config/tour.rviz`. |
| `camera/front_camera.yaml` | Front camera calibration written by `scripts/tools/calibrate_camera.sh`; `wifi_nav.sh` passes it to the relay → `/camera/camera_info`. Not present until you calibrate. |
| `camera/sim_front_camera.yaml` | Exact Gazebo camera intrinsics, for `scripts/tools/lidar_overlay.sh --sim` |
| `foxglove_layout.json` | Foxglove layout for `wifi_3d_nav.sh`: 3D panel (frame `lidar3d_map`, `/rtabmap/cloud_map`, `/tf`, `/plan`, click-to-go on `/clicked_point`) + Image panel (`/camera/image_raw`). Foxglove → Layouts → Import from file. See [docs/3d_nav.md](../docs/3d_nav.md#import-the-layout). |

Regenerate by opening RViz, adjusting the layout, and File → Save Config As.
The layout can be built without the robot against the Gazebo simulation
(`./scripts/runtime/wifi_nav.sh --sim`, see [gazebo/README.md](../gazebo/README.md)).
