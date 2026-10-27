# Frontier auto-explore: how to run it and what changed

The Go2 maps an area on its own: [`frontier_explorer.py`](../scripts/nodes/frontier_explorer.py)
picks the edge between known and unknown space and sends it to Nav2 as a goal.
It repeats until no reachable edge is left.

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

## Parameters

There is one file for the real robot and the sim:
[`config/frontier_explorer.yaml`](../config/frontier_explorer.yaml).
`wifi_nav.sh` copies it into the container. To override one value without
editing the file:

```bash
EXPLORE_GOAL_CLEARANCE_M=0.5 ./scripts/runtime/wifi_nav.sh --sim ...
```

| Parameter | Value | What it does |
|---|---|---|
| `frontier_max_cost` | 98 | A frontier cell can have cost up to this (just below inscribed). With only cost 0, the edges in 1.2 m doorways were too small to count |
| `min_frontier_size` | 4 | Smallest frontier, in cells |
| `unknown_hole_max_cells` | 16 | Unknown pockets this small are lidar gaps (the blind ring around the robot, speckles), not frontiers |
| `goal_clearance_m` | 0.6 | A goal must be this far from walls (lethal cells) |
| `goal_unknown_clearance_m` | 0.3 | and this far from unknown space |
| `turn_clearance_m` | 0.6 | Room needed to turn in place before a goal behind the robot |
| `goal_timeout_s` + `goal_timeout_per_m` | 60 + 4/m | Time allowed per goal grows with its distance |
| `frontier_blacklist_s` | 30 | A failed frontier is skipped for this long |
| `frontier_max_attempts` | 3 | After 3 failures it is skipped for good. Until then the explorer waits and retries instead of saying "Exploration complete" |
| `replan_on_explored` | true | Drop a goal once its frontier is seen (not next to a wall) |
| `unstick_enable` | false | Back off when boxed in. Off until tested on the robot |
| `recovery_wait_s` | 0 (8 from `wifi_nav.sh` while stuck recovery runs) | After a controller abort, keep the goal this long for [stuck recovery](omni_nav.md)'s step back and retry before dropping the frontier |

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

Sim fixes (see [`gazebo/README.md`](../gazebo/README.md)):

- The robot no longer tilts or floats: an upright hold.
- The robot no longer crawls: headless by default, 5 Hz lidar, `--no-camera`, and a real-time-factor check.
- Lidar scans are no longer empty without the window: rendering goes through `$DISPLAY`.
- Slow turns now move the robot: frictionless feet.
- New `agri_rooms` world with a corridor and three rooms for exploration tests.

Results in the sim, `agri_rooms`:

- One run mapped the doorway, the corridor and all three rooms in about 2 minutes.
- Other runs got stuck as described in the next section.

## Stuck recovery and `--omni` with this explorer (2026-10-27)

Upstream added [stuck recovery and `--omni`](omni_nav.md). Stuck recovery runs
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
`--omni` is the only setup with no aborts. It is still opt-in: MPPI
movement is untested on the real dog. On this VM it also used more CPU
(one missed 10 Hz control loop).

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
- Stuck recovery (on by default) steps back and retries, unless a wall is right behind the dog.
- `--omni` lets MPPI reverse and sidestep: no aborts in the sim (see above).
- Drive the robot a little away with 2D Goal Pose, then START again.

**Planned.**
- **Explorer:** when Nav2 aborts before the robot has moved, back off and retry instead of blacklisting the frontier.
- **Nav2 (go2-nav2-wifi):**
  - Trial `use_rotate_to_heading: true`. It got the robot through the room 2/3 doorway once in the sim.
  - Trial a lower shim `angular_dist_threshold`.

  Neither change is made yet; both need repeat trials first.

## Troubleshooting

See [troubleshooting.md → Explorer](troubleshooting.md#explorer).
