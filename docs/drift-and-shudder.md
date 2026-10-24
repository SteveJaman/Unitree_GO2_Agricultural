# Drift and shudder during navigation and exploration

[← Back to README](../README.md) · [Status](STATUS.md) · [Troubleshooting](troubleshooting.md)

Symptom (October 2026): during navigation and exploration, the Go2 sometimes
walks into walls. Its pose seems to change as the controller drives it, it
shudders, and the drift gets worse while exploring.

This note collects the evidence so far, the likely causes, the checks added,
and what still needs measuring. None of it has been verified on the robot yet.

## Likely causes, most likely first

### 1. Clock skew between the Jetson and the laptop

Point-LIO runs on the Jetson and stamps TF with the **Jetson's** clock. Nav2
runs on the laptop and looks TF up by the **laptop's** clock. go2-nav2-wifi's
own incident report (`docs/NAVIGATION-INCIDENT-2026-09-12.md`) found a 5 s
skew. Nav2 Humble's RotationShimController stamps its lookups with its own
current time, so it selected an older robot orientation. The robot "turned
substantially and then barely translated".

Delayed heading feedback produces exactly this: over-turning, correcting back
(shudder), then heading off-line, which can end at a wall. Without internet,
the Jetson has no time source; on 2026-10-20 it read February 1970.

**Added:** `wifi_nav.sh` now measures the robot-to-laptop clock offset over
SSH before every run (best of 5 round trips). It stops if the offset exceeds
`CLOCK_MAX_SKEW` (default 0.1 s). Fix it with:

```bash
./scripts/runtime/wifi_nav.sh --sync-clock   # asks for the robot's sudo password
```

This lasts until the robot reboots. For a permanent fix, install chrony on the
robot and point it at the laptop as its time server.

### 2. Point-LIO startup jump

If the robot moves during Point-LIO's first seconds, the estimate can jump
about 1 m (0.97 m in 3 s was measured, `POINTLIO-CONTROLLED-TEST.md`). After a
still start it is centimetre-level (`POINTLIO-LIVE-VALIDATION.md`: 2.9 cm
over 55 s stationary, about 1.4 cm / 0.2° over a 131 s route).

A shifted map puts walls in the wrong place for the rest of the session.
**Keep the robot standing still until the map appears in RViz** before
starting exploration.

### 3. No relocalization, and "2D Pose Estimate" does nothing

The 3D stack has no saved-map localization (`NAVIGATION-ACCEPTANCE-2026-09-12.md`),
and nothing subscribes to `/initialpose`. Any drift is permanent within a
session, and clicking 2D Pose Estimate in RViz does not correct it.

### 4. Exploration turns a lot

Each new frontier usually starts with an in-place turn (rotation shim at
0.6 rad/s; the smoother allows 0.7). Turns are where the L1 + Point-LIO
estimate is weakest, and where any clock skew hurts most (cause 1).
Exploration therefore accumulates more error than A-to-B navigation.

## Testing drift in the Gazebo sim

The sim is ground truth by default. Make it drift:

```bash
SIM_DRIFT_YAW=0.02 SIM_DRIFT_SCALE=0.03 ./scripts/runtime/wifi_nav.sh --sim --host-ip 127.0.0.1 --no-camera
```

`SIM_DRIFT_YAW=0.02` adds about 7° of heading error per full turn;
`SIM_DRIFT_SCALE=0.03` makes every metre read as 1.03 m. The lidar cloud is
placed with the drifted pose, so walls smear and double as with real drift.
Use this to see how the explorer's wall clearance (`goal_clearance_m`,
`turn_clearance_m` in `config/frontier_explorer.yaml`) holds up.

## Next measurements (on the robot)

1. Run with the clock check passing (offset < 0.1 s), and see whether the
   shudder and over-turning go away. This tests cause 1 directly.
2. Record a run (`GO2_RECORD=1`). Compare `/lidar3d/odom` with the factory
   odometry (`/utlidar/robot_odom`) around each turn.
3. Only then tune the controller. Candidates: a lower
   `rotate_to_heading_angular_vel` (0.6 → 0.4), a wider rotation-shim
   hysteresis, or a short stationary pause after each goal so Point-LIO
   settles. Changing these before fixing the clock would only mask cause 1.
