# Full Go2 Mobility: Omnidirectional Navigation (`--omni`)

[← Back to README](../README.md) · [3D Nav](3d_nav.md) · [Troubleshooting](troubleshooting.md)

```bash
./scripts/runtime/wifi_nav.sh --omni                     # or wifi_3d_nav.sh --omni
./scripts/runtime/wifi_3d_nav.sh --omni --camera-obstacles
```

By default, go2-nav2-wifi drives the Go2 like a two-wheeled robot. It plans
forward-only arcs, follows them with Regulated Pure Pursuit (which turns in
place, then walks forward, and never reverses), and its velocity smoother
clamps sideways speed to 0. A quadruped can do much more, and `--omni` turns
that on:

| | Default | `--omni` |
|---|---|---|
| Planner (Smac State Lattice) | `diff` primitives: forward arcs | `omni` primitives: forward, sideways, backward |
| Controller | Regulated Pure Pursuit + rotation shim | **MPPI**, `Omni` motion model (vx, vy, wz together) |
| Walk backwards | no | yes, up to 0.20 m/s |
| Sidestep | no (vy clamped to 0) | yes, up to 0.20 m/s |
| Stuck (leg aborted) | **recovery** (every session): step back 0.3 m, retry the goal | same, and it can also step aside |

The robot side needs no change: `robot_sport_bridge.py` already passes `vy`
to Unitree's `Move()`, and so does the simulator.

## How it is wired

- **[`config/nav2/go2_omni.yaml`](../config/nav2/go2_omni.yaml):** all the
  settings: lattice file, MPPI critics and limits, smoother limits.
- **[`scripts/tools/nav2_omni.py`](../scripts/tools/nav2_omni.py):** run by
  `wifi_nav.sh`. With `--omni` it copies that file to go2-nav2-wifi's
  `go2_nav2/config/go2_omni.yaml` and adds small hooks, tagged
  `# go2-omni`, to `lidar3d_planning.launch.py` and
  `lidar3d_controller.launch.py`. Every session without `--omni`, and the
  shutdown of every `--omni` session, removes them byte-for-byte.
- **MPPI install:** MPPI isn't in the upstream `go2-lidar3d` image.
  `wifi_nav.sh --omni` installs `ros-humble-nav2-mppi-controller` into the
  container the first time (this needs internet). If the install fails, the
  session continues forward-only and prints a warning.
- **[`go2_stuck_recovery.py`](../scripts/nodes/go2_stuck_recovery.py)**
  runs in **every** session; turn it off with `--no-recovery`. It watches
  `follow_path`. When a leg is aborted, it checks the local costmap for a
  free, known patch behind the dog, and with `--omni` also to its left, then
  right. It steps 0.3 m there at 0.12 m/s through the velocity smoother, then
  re-sends the last goal, whether the explorer's frontier or your 2D Goal
  Pose (up to 2 times). If nothing is free, it doesn't move. Without
  `--omni` it only backs up, because the default smoother clamps sideways
  speed. Its log is `/tmp/stuck_recovery.log` in `go2-lidar3d`.

### Why the dog used to stare at walls

Logs from a default-mode session (22:22–22:26) showed a loop:

1. The explorer sends a frontier goal, but the dog is facing an obstacle.
2. Regulated Pure Pursuit's collision check aborts the leg within
   milliseconds: `detected collision ahead!`
3. The goal handler only logs `leg failed`.
4. The explorer waits about 25 s (`No progress — abandoning goal`), picks
   another frontier, and step 2 repeats from the same spot.

Nothing ever moved the dog away; it repeated 6 times in about 2.5 minutes.
The recovery now backs it up 0.3 m on the first abort and retries
immediately.

To retune, edit `go2_nav2/config/go2_omni.yaml` in go2-nav2-wifi during a
session and restart the session. To keep the change, copy it back to
`config/nav2/go2_omni.yaml` here, because each `--omni` start copies this
repo's version over it.

| MPPI setting | Value | Meaning |
|---|---|---|
| `vx_max` / `vx_min` | 0.30 / −0.20 | Forward / backward speed (m/s) |
| `vy_max` | 0.20 | Sideways speed (m/s) |
| `wz_max` | 0.70 | Turn rate (rad/s) |
| `time_steps` × `model_dt` | 40 × 0.1 s | 4 s look-ahead |
| `batch_size` | 1000 | Sampled trajectories per cycle (CPU cost) |
| `PreferForwardCritic.cost_weight` | 2.0 | Weak: forward when free, reverse when it helps. Raise it to reverse less |
| `CostCritic.consider_footprint` | true | Collision check with the real 0.81 × 0.40 m footprint |

## Tested

**Simulator** (`--sim --no-explore --omni`), dog at (−4.5, 0) facing +x:

| Test | Result |
|---|---|
| Goal 0.8 m straight behind | Walked **backwards** to x = −5.03 with heading 0° throughout; every command had vx ≤ 0, so it never turned around |
| Goal 0.8 m to the left | **Sidestepped** to y = 0.54 with heading 0°; vy up to 0.20 m/s |
| Injected leg abort | Recovery found free space behind, stepped back, re-sent the goal; the planner replanned (2 retries, then stopped retrying) |

In the simulator the dog covers only about 20% of the commanded distance,
because Gazebo runs at roughly 20% of real time on this VM. On the robot the
speeds are real.

**Real dog** (no motion, no goals sent): MPPI loaded with the `Omni` model,
the smoother allows `vy` 0.2 m/s, recovery is ready, and combined with
`--camera-obstacles` the camera obstacles keep the costmap current. Shutdown
removes every hook.

**Not yet tested on the real dog: actual movement.** Try it with the remote
in hand:

1. `./scripts/runtime/wifi_3d_nav.sh --no-explore --omni`
2. Set **2D Goal Pose** 1 m behind the dog, facing the same way. It should
   walk backwards, not turn around.
3. Set one 1 m to its side. It should sidestep.
4. Set one past a chair. It should step around it.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `[omni] WARN: MPPI install failed` | No internet in the container. Install once by hand: `docker exec -u root go2-lidar3d apt-get install -y ros-humble-nav2-mppi-controller` |
| Dog wiggles or zig-zags | Lower `vy_std` / `wz_std`, or raise `PathAlignCritic.cost_weight` |
| Reverses too eagerly | Raise `PreferForwardCritic.cost_weight` (e.g. 5) |
| Controller CPU high | Lower `batch_size` (e.g. 600) or `time_steps` (30) |
| Dog faces a wall and does nothing | Check that recovery is on (`Recovery : true` in the start-up banner) and look at `/tmp/stuck_recovery.log`. With `--omni` it can also sidestep and reverse along the path |
| `CRITICAL FAILURE: SERVER controller_server IS DOWN` | The controller missed its 4 s lifecycle heartbeat because the VM was overloaded (seen in the simulator: Gazebo + RViz at load 15 on 6 cores). Nav2 stays down until a restart. Close other heavy programs |
| Recovery never moves | `docker exec go2-lidar3d tail /tmp/stuck_recovery.log`: `no free space` means everything around is occupied or unknown. It never steps into unknown space |
