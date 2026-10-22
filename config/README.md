# config/

| File | Purpose |
|---|---|
| `tour.rviz` | RViz layout with Nav2 waypoint panel, global/local path displays, and costmap. Load with `rviz2 -d config/tour.rviz`. |

Regenerate by opening RViz, adjusting the layout, and File → Save Config As.
The layout can be built without the robot against the Gazebo simulation
(`./scripts/runtime/wifi_nav.sh --sim`, see [gazebo/README.md](../gazebo/README.md)).