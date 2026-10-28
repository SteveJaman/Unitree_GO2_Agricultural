# config/

| File | Purpose |
|---|---|
| `tour.rviz` | RViz layout with Nav2 waypoint panel, global/local path displays, and costmap. Load with `rviz2 -d config/tour.rviz`. |
| `camera/front_camera.yaml` | Front camera calibration written by `scripts/tools/calibrate_camera.sh`; `wifi_nav.sh` passes it to the relay → `/camera/camera_info`. Not present until you calibrate. |
| `camera/sim_front_camera.yaml` | Exact Gazebo camera intrinsics, for `scripts/tools/lidar_overlay.sh --sim` |
| `foxglove_layout.json` | Foxglove layout for `wifi_3d_nav.sh --foxglove`: 3D panel (frame `lidar3d_map`, `/rtabmap/cloud_map`, `/tf`, `/plan`, click-to-go on `/clicked_point`) + Image panel (`/camera/image_raw`). Foxglove → Layouts → Import from file. |
| `survey_report.json` | Written at the end of an [autonomous survey](../docs/3d_nav.md#autonomous-survey---survey): per room the name, the reason (objects, VLM or no evidence), the type score, the objects found inside it, the VLM answers and whether it was reached; plus the survey duration. |
| `arm_waypoints.json` | D1 arm scan poses saved by `scripts/tools/teach_arm.py`, replayed by `play_arm.py` ([docs/d1arm.md](../docs/d1arm.md#plant-scanning-teach--playback)). User data, git-ignored. |
| `home_map.json` | Points at the RTAB-Map database of the last completed survey (`database`, `surveyed` time, `rooms` count). |
| `semantic_rooms.json` | The room map written by the room segmentation node: per room `id`, `label`, `polygon`, `center` (goal), `area`, plus `merges`. Names and merges come from the dashboard. Created on the first segmentation; see [docs/3d_nav.md](../docs/3d_nav.md#rooms-semantic-map). |
| `semantic_locations.json` | Named points (`Dock`, `Hydroponic_Station`, …) saved with `./scripts/runtime/wifi_3d_nav.sh --save-location`. Each entry has `x, y, z` in `lidar3d_map` (plus `rtabmap_map` when available) and a timestamp. Created on the first save. Coordinates are relative to where the dog started that session; see [docs/3d_nav.md](../docs/3d_nav.md#saving-named-locations---save-location). |

Regenerate by opening RViz, adjusting the layout, and File → Save Config As.
The layout can be built without the robot against the Gazebo simulation
(`./scripts/runtime/wifi_nav.sh --sim`, see [gazebo/README.md](../gazebo/README.md)).
