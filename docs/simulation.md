# Simulation: frontier explorer in the CMU Unity sim

[← Back to README](../README.md) · [Status](STATUS.md) · [Roadmap](roadmap.md)

Test `frontier_explorer.py` and the go2-nav2-wifi Nav2 config without the robot.
Everything runs on the host, without Docker.

```bash
./scripts/runtime/sim_explore.sh               # click "Auto Explore: START" in RViz
./scripts/runtime/sim_explore.sh --autostart   # no button needed
```

## What runs

```
autonomy_stack_go2 (CMU)          go2_sim_bridge            go2-nav2-wifi (unmodified)
────────────────────────          ──────────────            ──────────────────────────
Unity Model.x86_64                                          lidar3d_planning.launch.py
  └─ ros_tcp_endpoint ─ /registered_scan ──► /lidar3d/registered, /lidar3d/obstacle_points
vehicleSimulator ──── /state_estimation ──► TF lidar3d_map→base_link, /lidar3d/body_odom
       ▲                                                    lidar3d_controller.launch.py
       └──── /sim/cmd_vel (TwistStamped) ◄── /cmd_vel (Twist) ◄── velocity_smoother
                                                            frontier_explorer.py (scripts/nodes)
```

- Nav2 runs from the real launch files (`GO2_LIDAR3D_NAV=1` profile: footprint
  and Smac lattice planner), so a result in sim carries over to the robot.
- Explorer parameters are in
  [`explorer_sim.yaml`](../src/go2_sim_bridge/config/explorer_sim.yaml). They
  match what [`wifi_nav.sh`](../scripts/runtime/wifi_nav.sh) passes. When you
  change one, change the other.
- CMU's own local planner, terrain analysis and RViz are **not** started.
- The sim camera is republished on `/camera/image/raw` (useful for VLM work).

## Safety

`sim_explore.sh` sets `ROS_DOMAIN_ID=77` and `ROS_LOCALHOST_ONLY=1` and unsets
`CYCLONEDDS_URI`. Sim `/cmd_vel` therefore cannot reach a robot on the same
Wi-Fi. To inspect topics from another terminal, use the same settings:

```bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/files/autonomy_stack_go2/install/setup.bash
source ~/files/Unitree_GO2_Agricultural/install/setup.bash
ros2 topic echo /explore/status
```

## Build

`sim_explore.sh` builds automatically on the first run, or with `--build`.
It builds `go2_nav2` and `go2_description` from `~/files/go2-nav2-wifi` into
this repo's `install/`. That repo's `ws/install` is built in Docker with a
`/ws` prefix, so the host cannot use it.

Prerequisites:
- ROS 2 Humble and `ros-humble-navigation2`
- `~/files/autonomy_stack_go2` built, with the Unity environment unzipped
  (see its README)

## Known quirks

| Symptom | Cause / fix |
|---|---|
| `package 'go2_nav2' not found` | go2_nav2's `package.xml` maintainer email `danya@local` fails catkin_pkg validation. colcon then builds it as plain Python and skips the ament index. `sim_explore.sh` adds it to `AMENT_PREFIX_PATH` by hand. The proper fix is upstream. |
| `ros_tcp_endpoint` dies with `InvalidHandle` | ROS-TCP-Endpoint race when Unity re-registers topics. The launch respawns it, and Unity reconnects. |
| Bridge warns `no /registered_scan yet` | Unity has not connected yet. Check that its window opened and that the endpoint log shows `Connection from 127.0.0.1`. |
| Robot drifts after Ctrl+C | It should not: the bridge sends zero velocity when `/cmd_vel` is quiet for 0.5 s, and again on shutdown. |

## Results so far (2026-10-16, Unity indoor environment)

- **Before the `min_goal_dist` fix:** the explorer always picked a frontier
  0.25 m away, inside the lidar blind ring. Each goal "succeeded" instantly,
  and the robot never moved.
- **After the fix (3.5 min run):** 8 goals sent, 3 reached (the farthest about
  13 m from the start), 4 abandoned (no progress or timeout), 3 Nav2 legs
  failed.

Explorer issues the sim exposed, now fixed in `frontier_explorer.py`:
1. **Blacklist was ineffective.** An abandoned frontier `(0.58, 2.03)` was
   picked again 20 s later, even though `frontier_blacklist_s` is 30. The key
   was the centroid pixel, the check looked at the first 10 cluster pixels,
   and the rolling costmap origin shifts the pixel grid. *Fix:* the blacklist
   now holds world points. A frontier whose centroid is within
   `frontier_blacklist_radius_m` (0.5 m) of one is skipped.
2. **Slow reaction to failure.** When `go2_goal_pose_nav` logged `leg failed`,
   the explorer waited `progress_timeout_s` (20 s). go2_goal_pose_nav
   publishes no result, so the explorer now watches the
   `/compute_path_to_pose` and `/follow_path` action status topics. An
   ABORTED/CANCELED leg blacklists the goal, and the next goal is sent on the
   next tick. A SUCCEEDED `follow_path` counts as reached, because Nav2's
   goal checker (0.40 m) is looser than `goal_xy_tolerance` (0.35 m).
   Goal IDs from older legs are ignored, so a leg queued ahead of ours is not
   blamed on our goal.
3. **No stop on Ctrl+C.** `emergency_stop()` in `main()`'s `finally` raised
   `publisher's context is invalid`. *Fix:* `rclpy.init` is called with
   `SignalHandlerOptions.NO`. SIGINT/SIGTERM raise `KeyboardInterrupt`, the
   stop is published, and only then is the context shut down.

Fixing 2 exposed a fourth problem: the robot can end up inside lethal or
inscribed costmap space. The planner then fails every goal with `Starting
point in lethal space`. With the instant reaction, the explorer burned through
and blacklisted every frontier in about a minute (68 goals sent in that run).
*Fix:* on a planner abort, the explorer reads the costmap at the robot pose.
If the value is at least `stuck_cost_threshold` (99), it does not blacklist
the goal, waits `stuck_backoff_s` (5 s), and, if `unstick_enable` is set,
reverses up to 0.3 m at 0.1 m/s, like Nav2's backup recovery. `unstick_enable`
defaults to false and is turned on only in `explorer_sim.yaml`. **It is not in
`wifi_nav.sh` yet**, so the two parameter sets differ there until it is
tested on the robot. The unstick path did not trigger in the verification run
below, so it has not been exercised in sim yet.

### Results after the explorer fixes (4 min runs, `--autostart`)

| Run | Sent | Reached | Abandoned | Notes |
|---|---|---|---|---|
| Baseline (`min_goal_dist` fix only, 3.5 min) | 8 | 3 | 4 | 20 s wait after each failed leg |
| Fixes 1–3, before the lethal-start fix | 68 | 3 | 65 | Stuck in lethal space after 3 min, then cascaded |
| Fixes 1–3 + lethal-start fix | 12 | 10 | 1 | One goal still active at Ctrl+C |

- No goal was sent within 0.5 m of an active blacklist point in any run.
  Frontier centroids do drift 0.5–0.9 m as the map grows, so a nearby point
  can still be picked. In the last run it was then reached, so the radius was
  left at 0.5 m.
- Failed leg → next goal: at most 1.0 s (one tick at `tick_hz` 1.0).
- Ctrl+C: the explorer logs `Stopped. goals sent=… reached=… abandoned=…`
  after the stop and exits with `process has finished cleanly`, with no
  traceback. go2_goal_pose_nav still prints `Failed to publish log message to
  rosout: publisher's context is invalid` on shutdown. That comes from its own
  code, not the explorer.

## Files

| File | Role |
|---|---|
| [`cmu_bridge.py`](../src/go2_sim_bridge/go2_sim_bridge/cmu_bridge.py) | Topic/TF adapter |
| [`cmu_unity_sim.launch.py`](../src/go2_sim_bridge/launch/cmu_unity_sim.launch.py) | Sim side only (Unity, endpoint, vehicleSimulator, bridge) |
| [`sim_explore.launch.py`](../src/go2_sim_bridge/launch/sim_explore.launch.py) | Sim + go2_nav2 + explorer + RViz |
| [`sim_explore.sh`](../scripts/runtime/sim_explore.sh) | Checks, build, isolated DDS, launch |
