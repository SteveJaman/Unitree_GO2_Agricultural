# Frontier Explorer

[← Back to README](../../README.md) · [Scripts](../setup/scripts.md) · [Drive modes](drive_modes.md) · [Troubleshooting → Explorer](../troubleshooting.md#explorer) · [Roadmap](../roadmap.md)

The Go2 maps an area on its own: [`frontier_explorer.py`](../../scripts/nodes/frontier_explorer.py)
picks the edge between known and unknown space and sends it to Nav2 as a goal.
It repeats until no reachable edge is left. It replaces `explore_lite`.

This page covers running it, how it chooses goals, its parameters, and the
fixes and test results so far.

## Run it

```bash
# Real robot
./scripts/runtime/wifi_nav.sh
# Gazebo sim (no robot): field + building with rooms
./scripts/runtime/wifi_nav.sh --sim --host-ip 127.0.0.1 --no-camera
```

Then click **Auto Explore: START** in RViz (or `ros2 topic pub --once
/explore/resume std_msgs/Bool "{data: true}"`). Click **STOP**, or press Ctrl+C
in the terminal, to stop. Ctrl+C also stops the robot and the helpers.

To get a shell that sees the stack's topics (same DDS settings as the stack):

```bash
./scripts/tools/ros_shell.sh                                   # go2-lidar3d
./scripts/tools/ros_shell.sh go2-lidar3d ros2 topic echo /explore/status
```

A plain `docker exec -it go2-lidar3d bash` only shows `/rosout` and
`/parameter_events`.

## How it works

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

## Frontier choice

- **Path distance:** one BFS over traversable cells from the robot, each time
  a new frontier is chosen.
  - A frontier just behind a wall scores by the way round. In the test map:
    1.9 m straight line, 9.1 m path, so a farther open frontier wins.
  - Frontiers the robot cannot reach are dropped.
  - Cost: about 1.4 s on a 50 × 50 m map at 5 cm. It runs only when a goal is
    chosen.
- **Goal:** not the cluster centroid, which on curved frontiers often lies in
  unknown space. It is the nearest free cell to the centroid that is at least
  `goal_clearance_m` from walls and `goal_unknown_clearance_m` from unknown
  space. It faces the frontier, so the robot arrives looking into the unknown
  area instead of turning to +x.
- **Re-plan:** while driving, the goal is dropped as soon as its frontier has
  been seen, and the next one is chosen. The LiDAR usually clears a frontier
  long before the robot reaches it. The count is in `/explore/status` as
  `replans=`.
- **Blacklist:** abandoned goals (no progress, timeout, Nav2 abort) are
  blacklisted by position, so the frontier is skipped even if its cluster
  shape changes. After `frontier_max_attempts` it is skipped for good; until
  then the explorer waits instead of reporting "complete".
  (Before, the check missed almost every time: it compared the centroid
  pixel with the first 10 cluster cells.)
- **Nothing blocks:** the 10-message stop burst and the spin-on-goal run
  from a 20 Hz timer. TF, map updates and STOP keep working during them; the
  spin used to freeze the node for ~16 s.

## Parameters

There is one file for the real robot and the sim:
[`config/frontier_explorer.yaml`](../../config/frontier_explorer.yaml).
`wifi_nav.sh` copies it into the container. To override one value without
editing the file:

```bash
EXPLORE_GOAL_CLEARANCE_M=0.5 ./scripts/runtime/wifi_nav.sh --sim ...
```

| Param | Default | Notes |
|---|---|---|
| `robot_base_frame` | `base_link` | Must match the TF tree |
| `map_frame` | `lidar3d_map` | Frame from Point-LIO |
| `map_topic` | `/global_costmap/costmap` | The stack has no `/map`; the costmap is the grid source |
| `goal_topic` | `/goal_pose` | Consumed by `goal_pose_nav` |
| `cmd_vel_topic` | `/cmd_vel_nav` | Goes through the smoother — do NOT use `/cmd_vel` |
| `stop_topic` | `/cmd_vel` | Emergency stop bypasses the smoother so it overrides Nav2 |
| `min_frontier_size` | `4` | Cells per cluster — frontiers are thin |
| `frontier_max_cost` | `98` | Cells up to this cost count as free on a frontier edge (doorways) |
| `min_goal_dist` | `0.8` | Skip frontiers closer than this (must exceed goal tolerance; the lidar blind ring makes fake frontiers under the robot) |
| `unknown_hole_max_cells` | `16` | Unknown pockets this small are lidar gaps, not frontiers |
| `min_known_cells` | `200` | Map must have this many known cells before frontier logic runs |
| `blind_trigger_cells` | `100` | Below this, do a blind step |
| `blind_enable` | `false` | Off by default — fights Nav2 during first tests |
| `coverage_enable` | `false` | Off by default |
| `return_to_init` | `false` | Off by default |
| `spin_on_goal` | `false` | Off by default — enable for tour capture |
| `spin_rate_rad_s` | `0.4` | Rotation rate during the spin-on-goal |
| `frontier_blacklist_s` / `frontier_blacklist_radius_m` | `30` / `0.5` | An abandoned frontier is skipped for this long, within this radius (m) |
| `frontier_max_attempts` | `3` | Then skipped for good; until then a blacklisted frontier blocks "complete" |
| `traversable_max_cost` | `98` | Costmap value up to which a cell counts for path distance (99+ inscribed / lethal) |
| `goal_clearance_m` / `goal_unknown_clearance_m` | `0.6` / `0.3` | A goal is snapped to a cell this far from walls / unknown space |
| `goal_search_radius_m` | `1.5` | Search radius (m) around the frontier centroid for a clear goal |
| `goal_timeout_s` + `goal_timeout_per_m` | `60` + `4` | Time allowed per goal grows with its distance |
| `unstick_enable` | `false` | Back off when boxed in or in lethal space |
| `frontier_gone_radius` / `replan_on_explored` | `0.6` / `true` | Drop the goal once no frontier cell is left this close to it |
| `turn_clearance_m` | `0.6` | Room needed to turn in place before a goal behind the robot; closer to a wall, frontiers straight ahead win |
| `recovery_wait_s` | `0` (8 from `wifi_nav.sh` while stuck recovery runs) | After a controller abort, keep the goal this long for [stuck recovery](drive_modes.md)'s step back and retry before dropping the frontier |
| `goal_xy_tolerance` | `0.35` | Distance at which a goal counts as reached |
| `progress_timeout_s` | `20` | No progress toward the goal for this long: abandon it |
| `tick_hz` | `1.0` | Main loop rate |

## Running the explorer by hand

Without `wifi_nav.sh` (it normally starts the explorer with the shared config):

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

## Improvements so far (branch `wifi-3d-nav`)

| Problem | Fix |
|---|---|
| START in RViz did nothing | The explorer was started without the stack's `RMW_IMPLEMENTATION` / `CYCLONEDDS_URI`, so it was on a different DDS setup from RViz. `wifi_nav.sh` now starts the explorer, the floor filter, the camera relay and the stop with the same settings (`DDS_EXEC_ENV`) |
| Explorer stalled on a fresh map | Speckles in the lidar ring were treated as frontiers, leaving one valid goal cell. Small unknown pockets are now filled as free (`unknown_hole_max_cells`) |
| Explorer said "complete" in the corridor | Three causes: doorway frontiers were too small (`frontier_max_cost`), long goals timed out (`goal_timeout_per_m`), and blacklisted frontiers were hidden instead of retried (`frontier_max_attempts`) |
| Two configs drifted apart | One shared `config/frontier_explorer.yaml` with `EXPLORE_*` overrides |
| Robot clock off from the laptop | `wifi_nav.sh` checks the clock skew at startup |

Merged from upstream (`f29ab6d`) on top of these:

- **Path distance.** Frontiers score by size over BFS path length from the
  robot (`traversable_max_cost` 98), and unreachable ones are dropped.
- **Goal faces the frontier,** so the robot arrives looking into the unknown.
- **Re-plan** (`replan_on_explored`). The goal is dropped once its frontier
  has been seen, but only when no wall is within `turn_clearance_m`.
  In the first merged sim run, a re-plan stopped the robot in room 1's
  doorway, where every next goal needed a turn that Nav2 refused.
- **Nothing blocks the node:** the stop burst and the spin-on-goal run on a 20 Hz timer.

Merged run in the sim (2026-10-26, `agri_rooms`, `--no-camera`, real-time factor 0.85):

- "Exploration complete" after about 4 minutes, with goals in all three rooms.
- 14 goals: 1 reached, 4 dropped early because their frontier was already
  seen, and 9 abandoned.
- Of the 9 abandoned, 7 were Nav2 "collision ahead" aborts (see the known
  issue below), and 2 were goals that turned out too close to a wall once
  the map filled in.

Sim fixes (see [`docs/simulation/gazebo.md`](../simulation/gazebo.md)):

- The robot no longer tilts or floats: an upright hold.
- The robot no longer crawls: headless by default, 5 Hz lidar, `--no-camera`, and a real-time-factor check.
- Lidar scans are no longer empty without the window: rendering goes through `$DISPLAY`.
- Slow turns now move the robot: frictionless feet.
- New `agri_rooms` world with a corridor and three rooms for exploration tests.

Results in the sim, `agri_rooms`:

- One run mapped the doorway, the corridor and all three rooms in about 2 minutes.
- Other runs got stuck as described in the next section.

## Stuck recovery and `--omni` with this explorer (2026-10-27)

Upstream added [stuck recovery and `--omni`](drive_modes.md). Stuck recovery runs
in every session: after an abort it steps back 0.3 m and re-sends the goal.
`--omni` switches Nav2 to MPPI, which can reverse and sidestep.

Merging them needed three fixes:

- **The explorer waits for recovery** (`recovery_wait_s`). Before, it dropped
  the goal at once and sent the next one while recovery was still backing up.
- **Recovery runs on the stack's DDS setup** (`DDS_EXEC_ENV`). Without it,
  it never saw an abort in the sim.
- **Recovery counts inflation as free** (`free_threshold` 99, was 50). In a
  doorway every cell near the dog is inflation (cost 50–98), so it reported
  `no free space` every time. It still avoids inscribed and lethal cells and
  unknown space.

Sim runs, `agri_rooms`, `--no-camera`, one run each:

| Run | Setup | Time | Goals | Reached | Dropped early | Nav2 aborts | Rooms seen |
|---|---|---|---|---|---|---|---|
| A | `--no-recovery` | 186 s | 24 | 0 | 2 | 21 | none: stuck in room 1's doorway, then a false "complete" |
| B | recovery as merged | 232 s | 12 | 0 | 4 | 7 | 1: recovery never saw the aborts (DDS) |
| B2 | + DDS fix | 336 s | 12 | 2 | 3 | 6 | 1, 2: recovery said `no free space` 6 times |
| B3 | + inflation counts as free | 226 s | 17 | 2 | 8 | 2 | all 3. Both aborts had a wall right behind the dog |
| C | B3 + `--omni` | 231 s | 12 | 1 | 8 | 0 | all 3. Two goals dropped for no progress |

Results vary a lot from run to run (run 2 on 2026-10-26 had 7 aborts, A had 21).
`--omni` is the only setup with no aborts. A 9-trial comparison on
2026-10-28 found it slower overall, and it walked backwards and sideways
half the time. The default is now `--drive hybrid`, which drives like `diff`
and sidesteps only during stuck recovery. See
[locomotion_comparison.md](locomotion_comparison.md).

## Known issue: stuck next to an obstacle

**Symptom.** The robot stops in the purple (inflated) part of the costmap,
facing a crate or wall about 0.6 m away. The explorer log shows several
`Nav2 controller ABORTED` within a second each, then repeated
`No frontier has a goal clear of walls`.

**Cause.**

1. Nav2's Regulated Pure Pursuit (RPP) controller drives forward along an arc.
2. Its collision check sees the obstacle in that arc and aborts ("collision ahead").
3. The rotation shim turns in place only when the path is more than 0.8 rad off the robot's heading, so at smaller angles nothing turns the robot first.
4. The explorer treats each abort as a bad frontier and blacklists it.

**What helps now.**

- Stuck recovery (on by default) steps back and retries. With the default `--drive hybrid`, it sidesteps if a wall is right behind the dog.
- `--drive omni` lets MPPI reverse and sidestep all the time: no aborts in the sim, but slower exploration ([comparison](locomotion_comparison.md)).
- Drive the robot a little away with 2D Goal Pose, then START again.

**Planned.**

- **Explorer:** when Nav2 aborts before the robot has moved, back off and retry instead of blacklisting the frontier.
- **Nav2 (go2-nav2-wifi):**
  - Trial `use_rotate_to_heading: true`. It got the robot through the room 2/3 doorway once in the sim.
  - Trial a lower shim `angular_dist_threshold`.

  Neither change is made yet; both need repeat trials first.

## Troubleshooting

See [troubleshooting.md → Explorer](../troubleshooting.md#explorer).
