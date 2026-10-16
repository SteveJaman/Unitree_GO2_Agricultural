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

Open explorer issues the sim exposes:
1. **Blacklist is ineffective.** An abandoned frontier `(0.58, 2.03)` was
   picked again 20 s later, even though `frontier_blacklist_s` is 30. The
   blacklist key is the centroid pixel, but the check looks at the first 10
   cluster pixels. The pixel grid also shifts whenever the rolling costmap
   origin moves.
2. **Slow reaction to failure.** When `go2_goal_pose_nav` logs `leg failed`,
   the explorer waits for `progress_timeout_s` (20 s) instead of moving on
   right away.
3. **No stop on Ctrl+C.** `emergency_stop()` in `main()`'s `finally` raises
   `publisher's context is invalid`, so the final stop is never sent. On the
   robot, the velocity_smoother's 1 s timeout is what actually stops it.

## Files

| File | Role |
|---|---|
| [`cmu_bridge.py`](../src/go2_sim_bridge/go2_sim_bridge/cmu_bridge.py) | Topic/TF adapter |
| [`cmu_unity_sim.launch.py`](../src/go2_sim_bridge/launch/cmu_unity_sim.launch.py) | Sim side only (Unity, endpoint, vehicleSimulator, bridge) |
| [`sim_explore.launch.py`](../src/go2_sim_bridge/launch/sim_explore.launch.py) | Sim + go2_nav2 + explorer + RViz |
| [`sim_explore.sh`](../scripts/runtime/sim_explore.sh) | Checks, build, isolated DDS, launch |
