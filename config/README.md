# config/

| File | Purpose |
|---|---|
| `tour.rviz` | RViz layout with Nav2 waypoint panel, global/local path displays, and costmap. Load with `rviz2 -d config/tour.rviz`. |
| `semantic_rooms.json` | The room map written by the room segmentation node: per room `id`, `label`, `polygon`, `center` (goal), `area`, plus `merges`. Names and merges come from the dashboard. Created on the first segmentation; see [docs/3d_nav.md](../docs/3d_nav.md#rooms-semantic-map). |
| `semantic_locations.json` | Named points (`Dock`, `Hydroponic_Station`, …) saved with `./scripts/runtime/wifi_3d_nav.sh --save-location`. Each entry has `x, y, z` in `lidar3d_map` (plus `rtabmap_map` when available) and a timestamp. Created on the first save. Coordinates are relative to where the dog started that session; see [docs/3d_nav.md](../docs/3d_nav.md#saving-named-locations---save-location). |

Regenerate by opening RViz, adjusting the layout, and File → Save Config As.
The layout can be built without the robot against the Gazebo simulation
(`./scripts/runtime/wifi_nav.sh --sim`, see [gazebo/README.md](../gazebo/README.md)).
