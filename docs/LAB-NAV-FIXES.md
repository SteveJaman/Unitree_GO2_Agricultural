# Go2 3D navigation — lab fixes for go2-nav2-wifi

Fixes for two problems with `./mapping.sh --3d --nav` on our Unitree Go2 Edu:

1. **The dog in RViz didn't match the real dog.** Lying flat, it showed about 20° of tilt, and its heading was about 120° off.
2. **The dog shuddered** while navigating to RViz goals.

After these fixes, it navigates smoothly to 2D Goal Pose targets.

## Setup (once per machine)

```bash
git clone https://github.com/dancher00/go2-nav2-wifi
cd go2-nav2-wifi
bash /path/to/apply-lab-nav-fixes.sh
```

Then follow the repo's normal setup (README and `docs/3D-QUICKSTART.md`) if you haven't already. That covers Docker, the robot Wi-Fi relay, and `./setup-lidar3d.sh`.

What the script does:
- **If the `go2-lidar3d` container already exists:** it installs the two extra Nav2 packages into it and rebuilds `go2_nav2`.
- **If it doesn't exist yet:** run `./setup-lidar3d.sh` afterwards. The patched Dockerfile already includes the packages.
- It adds a `go2sh` alias to `~/.bashrc` (see below).

The script is safe to run twice. To undo it: `git checkout -- docker go2_nav2`. It was tested on upstream commit `4a8a8b9`. If upstream changes, the script stops with an error instead of half-applying.

## What changed and why

| File | Change | Why |
|---|---|---|
| `go2_nav2/config/lidar3d_robot_viz.yaml` | `sensor_from_base` now uses the **official Go2 URDF lidar mount** (radar_joint, pitch 2.8782 rad), inverted | The shipped value came from a one-off calibration on the repo author's robot. On ours it tilted the body by about 20° and rotated the heading by about 120°. Nav2 uses the same value for body odometry, so the controller was steering on the wrong heading. |
| `go2_nav2/launch/lidar3d_controller.launch.py` | DWB → **Regulated Pure Pursuit**, behind the rotation shim, with wider hysteresis (0.8 / 0.2 rad) | DWB plans around the measured velocity, which here is a noisy derivative of the Point-LIO pose over Wi-Fi (about ±0.1 m/s while standing still). RPP follows the path geometrically. |
| same file | Added **nav2 velocity_smoother** (open loop, accel limits, deadband). Controller → `/cmd_vel_nav` → smoother → `/cmd_vel` | Limits acceleration and zeroes the tiny commands that make the Go2 step in place. |
| `docker/Dockerfile.lidar3d` | Installs `nav2-velocity-smoother` and `nav2-regulated-pure-pursuit-controller` | So the packages survive container rebuilds. |

## Running

1. **Stand the dog up before launching.** The map frame is fixed from the floor at startup.
2. Check first with no motion: `./mapping.sh --3d --plan`. The RViz model should face the same way as the real dog.
3. Navigate: `./mapping.sh --3d --nav`. Use **2D Goal Pose** 1–2 m ahead, then increase the distance.
4. Keep the **handheld remote in hand as an emergency stop**, but don't drive with it during a session (see below).
5. **Start a fresh session for each test run.** Ctrl+C, stand the dog, relaunch.

## Working inside the container

All of ROS runs inside Docker with its own network settings (domain 64, CycloneDDS over Wi-Fi). A plain terminal on your machine won't see any topics. Use the alias instead:

```bash
go2sh              # opens a shell in the container with the right environment
ros2 topic list
```

Repo folders are bind-mounted (`go2_nav2/` → `/ws/src/go2_nav2`, `ws/` → `/ws`). Edit on your machine and run in `go2sh`. Rebuild after changing launch or config files:

```bash
cd /ws && colcon build --symlink-install --packages-select go2_nav2
```

## Teleop (instead of the handheld remote)

The handheld remote moves the dog faster and more sharply than Point-LIO (gyro-only) can track, which skews the pose. Drive with the keyboard through the smoother instead:

```bash
go2sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_nav
```

Lower the speed with `z` to about 0.3 m/s linear and 0.5 rad/s angular. Don't teleop while a Nav2 goal is active, because the two will fight.

## Quick checks

```bash
ros2 run tf2_ros tf2_echo lidar3d_map base_link   # standing still: roll and pitch within about ±5°
ros2 topic hz /cmd_vel                            # about 10–20 Hz while driving
ros2 topic echo /cmd_vel --field linear.x         # ramps smoothly, no sign flipping
```

## Troubleshooting

- **No topics in a terminal:** you're outside the container. Use `go2sh`.
- **The model drifts or sinks into the floor, or the red trail tangles:** the SLAM pose is degrading. Usually this follows fast remote driving, or launching before the dog was standing. Restart the session.
- **The VM is slow:** RViz renders on the CPU in the container. Untick the heavy point cloud displays while driving. Use bridged networking (not NAT) and give the VM 4+ cores.
- **Patch error when running the script:** upstream changed. Apply the changes from the table above by hand.
