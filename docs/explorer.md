# Frontier Explorer

[← Back to README](../README.md) · [Scripts](scripts.md) · [Troubleshooting → Explorer](troubleshooting.md#explorer) · [Roadmap](roadmap.md)

The custom explorer ([`scripts/nodes/frontier_explorer.py`](../scripts/nodes/frontier_explorer.py)) replaces `explore_lite`.
Four phases:

1. **Blind lead-in** — when the map has < `blind_trigger_cells` known cells,
   publish direct `Twist` on `/cmd_vel_nav` to force new sensor data.
   *(Disabled by default — flip `EXPLORE_BLIND_ENABLE=true` to test.)*
2. **Frontier detection** (Yamauchi 1997) — find free cells adjacent to unknown,
   cluster them, score by `size / path distance`, publish the best as a
   `PoseStamped` on `/goal_pose` (see [Frontier choice](#frontier-choice)).
3. **Visited-cell coverage** — when no frontiers remain, drive to the nearest
   unvisited free cell. *(Disabled by default.)*
4. **Return to origin** — drives back to the starting pose.
   *(Disabled by default.)*
5. **Spin-on-goal** — rotate 360° after each successful goal for tour capture.
   *(Disabled by default — flip `-p spin_on_goal:=true`.)*

## Parameters

| Param | Default | Notes |
|---|---|---|
| `robot_base_frame` | `base_link` | Must match the TF tree |
| `map_frame` | `lidar3d_map` | Frame from Point-LIO |
| `map_topic` | `/global_costmap/costmap` | The stack has no `/map`; the costmap is the grid source |
| `goal_topic` | `/goal_pose` | Consumed by `goal_pose_nav` |
| `cmd_vel_topic` | `/cmd_vel_nav` | Goes through the smoother — do NOT use `/cmd_vel` |
| `stop_topic` | `/cmd_vel` | Emergency stop bypasses the smoother so it overrides Nav2 |
| `min_frontier_size` | `4` | Cells per cluster — frontiers are thin |
| `min_known_cells` | `200` | Map must have this many known cells before frontier logic runs |
| `blind_trigger_cells` | `100` | Below this, do a blind step |
| `blind_enable` | `false` | Off by default — fights Nav2 during first tests |
| `coverage_enable` | `false` | Off by default |
| `return_to_init` | `false` | Off by default |
| `spin_on_goal` | `false` | Off by default — enable for tour capture |
| `spin_rate_rad_s` | `0.4` | Rotation rate during the spin-on-goal |
| `frontier_blacklist_s` / `frontier_blacklist_radius` | `30` / `0.6` | An abandoned frontier is skipped for this long, within this radius (m) |
| `traversable_max_cost` | `90` | Costmap value below which a cell counts for path distance (99+ inscribed / lethal) |
| `goal_snap_radius` | `1.0` | Search radius (m) around the frontier centroid for a clear goal |
| `frontier_gone_radius` / `replan_on_explored` | `0.6` / `true` | Drop the goal once no frontier cell is left this close to it |
| `tick_hz` | `1.0` | Main loop rate |

## Frontier choice

- **Path distance:** one BFS over traversable cells from the robot, each time
  a new frontier is chosen.
  - A frontier just behind a wall scores by the way round. In the test map:
    1.9 m straight line, 9.1 m path, so a farther open frontier wins.
  - Frontiers the robot cannot reach are dropped.
  - Cost: about 1.4 s on a 50 × 50 m map at 5 cm. It runs only when a goal is
    chosen.
- **Goal:** not the cluster centroid, which on curved frontiers often lies in
  unknown space. It is the reachable cell near the centroid with the lowest
  cost (the most clearance), kept 2 cells back from unknown space. It faces
  the frontier, so the robot arrives looking into the unknown area instead of
  turning to +x.
- **Re-plan:** while driving, the goal is dropped as soon as its frontier has
  been seen, and the next one is chosen. The LiDAR usually clears a frontier
  long before the robot reaches it. The count is in `/explore/status` as
  `replans=`.
- **Blacklist:** abandoned goals (no progress, timeout) are blacklisted by
  position, so the frontier is skipped even if its cluster shape changes.
  (Before, the check missed almost every time: it compared the centroid
  pixel with the first 10 cluster cells.)
- **Nothing blocks:** the 10-message stop burst and the spin-on-goal run
  from a 20 Hz timer. TF, map updates and STOP keep working during them; the
  spin used to freeze the node for ~16 s.

## Running the explorer manually

```bash
docker cp ~/files/Unitree_GO2_Agricultural/scripts/nodes/frontier_explorer.py \
    go2-lidar3d:/tmp/frontier_explorer.py

docker exec -u root -it go2-lidar3d bash -lc '
    source /opt/ros/humble/setup.bash
    source /ws/install/setup.bash 2>/dev/null
    python3 /tmp/frontier_explorer.py --ros-args \
        -p map_frame:=lidar3d_map \
        -p map_topic:=/global_costmap/costmap \
        -p goal_topic:=/goal_pose \
        -p cmd_vel_topic:=/cmd_vel_nav \
        -p stop_topic:=/cmd_vel \
        -p blind_enable:=false \
        -p coverage_enable:=false \
        -p return_to_init:=false \
        -p spin_on_goal:=false \
        -p tick_hz:=1.0
'
```

Enable in another terminal:

```bash
docker exec -u root go2-lidar3d bash -lc '
    source /opt/ros/humble/setup.bash && source /ws/install/setup.bash 2>/dev/null
    ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"
'
```

Watch status:

```bash
docker exec -u root go2-lidar3d bash -lc '
    source /opt/ros/humble/setup.bash && source /ws/install/setup.bash 2>/dev/null
    ros2 topic echo /explore/status
'
```

Expected states: `IDLE` → `EXPLORING` → goals on `/goal_pose` → `RETURNING` → `DONE`.
