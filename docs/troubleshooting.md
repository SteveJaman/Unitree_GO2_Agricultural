# Troubleshooting

[← Back to README](../README.md) · [Status](STATUS.md) · [Roadmap](roadmap.md)

Every failure mode encountered during development, with the actual fix.
Sections are self-contained. Search for the error message you see.

**Jump to:** [At a glance](#at-a-glance) ·
[Session / Infrastructure](#session--infrastructure) ·
[Motion](#motion) · [Explorer](#explorer) · [D1-T Arm](#d1-t-arm) ·
[Camera](#camera) · [RViz / VM](#rviz--vm) · [3D twin](#3d-digital-twin) ·
[Known blockers](#known-blockers-and-fixes) ·
[Collision tuning](#collision-threshold-tuning) ·
[Diagnostics cheat sheet](#diagnostics-cheat-sheet)

---

## At a glance

| Symptom | Cause | Fix |
|---|---|---|
| `GO2_CONTAINER: Start the project Docker container first` | Container missing | `cd ~/files/go2-nav2-wifi && ./setup-lidar3d.sh --laptop` |
| `ros2: command not found` on host | ROS runs inside Docker | Use `go2sh` or `docker exec -u root go2-lidar3d bash -lc '…'` |
| `Invalid frame ID "lidar3d_map"` | Session not up, or Point-LIO not forwarding | Check `mapping.sh --3d --nav` log; check Jetson relay |
| `/cmd_vel` has 20 Hz but dog doesn't move | Jetson port 17999 conflict | `wifi_nav.sh` handles it; if `motion.log` says "Address already in use", try `--no-relay` |
| Simulator won't start or RViz is empty in `--sim` | Image, rendering, or `simarm0` | [gazebo/README.md](../gazebo/README.md#troubleshooting), `./gazebo/test_sim.sh` |
| Explorer stays `IDLE` | Button not wired, or costmap silent | [`./scripts/tools/test_explore.sh`](../scripts/tools/test_explore.sh) |
| Camera at ~0.2 fps, dashboard "Camera unavailable", empty 3D map | VM UDP receive buffers too small (Ubuntu default) | Raise `net.core.rmem_max` / `rmem_default`, see [Camera at ~0.2 fps](#camera-at-02-fps-camera-unavailable-in-the-dashboard-empty-3d-map) |
| Camera shows "No Image" in RViz | Robot publisher not running, or display set to Best Effort | `ssh unitree@<robot> tail /tmp/go2_camera_wifi.log`; display **Reliability** `Reliable`. See [Camera](#camera) |
| Arm does not respond to commands | Wrong `--iface`, or motors not enabled | See [Arm does not respond](#arm-does-not-respond) above |
| `could not find the Go2 on enp0s3` | DHCP changed / discovery failed | `wifi_nav.sh --robot-ip <IP>` — see [Robot IP not found](#robot-ip-not-found--dhcp-changed) |
| `DDS discovery failed` (arm) | No DDS match within 20 s | Arm powered and booted? No `--peer`, no ROS sourced; check `diag_arm.sh` |
| RViz renders slowly | CPU rendering | Close the point cloud display while tuning |
| Foxglove 3D panel: `Error creating WebGL context` | VirtualBox has no GPU acceleration (not a code error) | Enable 3D Acceleration + 128 MB video memory, install `virtualbox-guest-utils`, or use Foxglove on Windows / RViz. See [3d_nav.md](3d_nav.md#webgl--hardware-acceleration-error) |

---

## Session / Infrastructure

### `GO2_CONTAINER: Start the project Docker container first`

`mapping.sh` expects a running container. `wifi_nav.sh` creates it.

```bash
cd ~/files/go2-nav2-wifi
./setup-lidar3d.sh --laptop
```

Then re-run:

```bash
cd ~/files/Unitree_GO2_Agricultural
./scripts/runtime/wifi_nav.sh
```

### `ros2: command not found` on the host

ROS 2 runs inside the Docker container. The host has no ROS.

```bash
go2sh
# or, one-off:
docker exec -u root go2-lidar3d bash -lc \
    'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash 2>/dev/null && ros2 topic list'
```

### `Invalid frame ID "lidar3d_map"` on TF lookup

The session isn't up, or Point-LIO isn't forwarding.

1. Check the `mapping.sh --3d --nav` log
2. Check the Jetson relay is in FULL mode (see [Motion](#motion))
3. Inspect the TF tree inside the container:

```bash
go2sh
ros2 run tf2_tools view_frames
# writes frames.pdf — open it to see the tree
```

If `frames.pdf` is empty, no session is running.

### `Address already in use` for port 17999 on the Jetson

[`wifi_nav.sh`](../scripts/runtime/wifi_nav.sh) handles this automatically — it SSHes into the Jetson and
kills stale relays before starting a new one. If you need to do it by hand:

```bash
ssh unitree@<robot-ip>       # shown in the wifi_nav.sh banner
pkill -9 -f 'robot-relay-wifi|robot-motion-wifi|go2_cmd_vel_tcp|robot_sport_bridge'
rm -f /tmp/go2-relay-wifi.sock*
sleep 3
ss -tlnp | grep 17999        # must be empty
```

Then relaunch the session on the VM.

### `SENSOR-ONLY: no cmd_vel TCP server…` in `slam.log`

**Normal.** That is the sensor relay's own banner. The motion relay is a
separate process started by `mapping.sh --3d --nav`. Ignore this line.

The line you actually care about is:

```text
cmd_vel TCP server started on port 17999
```

If that is missing, the motion relay failed. See the
[port 17999 entry](#address-already-in-use-for-port-17999-on-the-jetson) above.

### Container has stale `GO2_HOST_IP` / `GO2_ROBOT_IP`

Both addresses are baked into the container at creation time, and the router
hands out addresses by DHCP. `mapping.sh` reads them from the container. It
corrects a stale host IP itself, but a stale robot IP sends it to the wrong
robot. **`wifi_nav.sh` handles this automatically.** It compares both with the
current host and robot addresses and recreates the container when either
differs.

Check by hand:

```bash
ip -4 -o addr show dev enp0s3                       # the VM's current Wi-Fi IP
docker exec go2-lidar3d printenv GO2_HOST_IP        # must match
docker exec go2-lidar3d printenv GO2_ROBOT_IP       # must be the robot's current IP
docker exec go2-lidar3d cat /tmp/go2-wifi-cyclonedds.xml | grep -i address   # rewritten each session
./scripts/runtime/wifi_nav.sh --check               # reports STALE if they differ
```

If a stale IP is exported in your shell (e.g. `~/.bashrc`), `wifi_nav.sh`
ignores it with a note. Remove it to avoid confusion.

### `could not find the Go2 on enp0s3`

Robot discovery failed. Order tried: last known IP, hostname `unitree`, MAC
prefix scan (`02:00:00`) on `enp0s3`, then an SSH-key probe.

1. Find the address in the router's DHCP lease list, or run `hostname -I` on
   the Jetson, then `./scripts/runtime/wifi_nav.sh --robot-ip <IP>`. It is
   cached for next time.
2. If the Jetson's Wi-Fi MAC doesn't start with `02:00:00` (check `ip link` on
   the Jetson), set `GO2_MAC_PREFIX=<prefix>`.
3. Install `arp-scan` (`sudo apt install arp-scan`) or `nmap` for a faster,
   more reliable scan. Without them, a ping sweep is used.

### `route to robot ... goes via the arm interface enp0s8`

The VM would send Go2 traffic out the arm's Ethernet. Make sure `enp0s3` (Wi-Fi
bridge) and `enp0s8` (arm, `192.168.123.x`) are on different subnets, and that
`enp0s3` holds the route to the robot's network (`ip route get <robot-ip>`).

### `ValueError: numpy.dtype size changed`

The container's pip numpy is newer than the apt scipy was built against.
`wifi_nav.sh` pins `numpy<1.25` in the running container before launch. If pip
can't download (no internet on the robot's Wi-Fi), it prints a warning and
continues. Run the printed command once with internet. It also appends the pin
to the upstream Dockerfiles in `go2-nav2-wifi/docker/`, including
`Dockerfile.lidar3d`.
If the container exits before the pin can be applied, rebuild it:

```bash
docker rm -f go2-lidar3d
cd ~/files/go2-nav2-wifi && ./setup-lidar3d.sh --laptop
```

If scipy still fails after the pin, a pip-installed package was built against
a newer numpy:
`docker exec go2-lidar3d python3 -m pip list | grep -iE 'numpy|scipy|pandas'`.

### `Host key verification failed` / `REMOTE HOST IDENTIFICATION HAS CHANGED`

The scripts pin the robot's SSH key to the alias `go2-robot`, not to an IP, so
DHCP changes don't trigger this. If you replaced the Jetson or reflashed it:
`ssh-keygen -R go2-robot`, then `./scripts/runtime/wifi_nav.sh --setup-ssh`.

### `GO2_CONTAINER` is set to the wrong container

The go2-nav2-wifi repo has two compose services: `go2-humble` (base) and
`go2-lidar3d` (the one you want). `docker compose up` alone starts the
wrong one.

Correct command:

```bash
cd ~/files/go2-nav2-wifi
./setup-lidar3d.sh --laptop
```

Or, manually, from `docker/`:

```bash
docker compose -f docker-compose.yml -f docker-compose.lidar3d.yml -p go2-lidar3d up -d
```

Confirm with `docker ps` — the container name must be `go2-lidar3d`.

---

## Motion

### `/cmd_vel` publishes but the dog doesn't move

The Jetson motion relay failed to bind port 17999. Check on the Jetson:

```bash
ssh unitree@<robot-ip>
ss -tlnp | grep 17999
tail -30 /tmp/robot-relay.log
```

Look for `cmd_vel TCP server started on port 17999`. If it's missing, the
relay started in sensor-only mode. Restart it in FULL mode:

```bash
ssh unitree@<robot-ip>
pkill -9 -f 'robot-relay-wifi|robot-motion-wifi|go2_cmd_vel_tcp|robot_sport_bridge'
rm -f /tmp/go2-relay-wifi.sock*
sleep 3
export GO2_HOST_IP=<vm-wifi-ip>    # `ip -4 -o addr show dev enp0s3` on the VM
bash ~/robot-relay-wifi.sh
```

Or just relaunch `wifi_nav.sh`, which does this automatically.

### Dog moves but doesn't follow the planned route

Check the chain inside the container:

```bash
go2sh
ros2 topic echo /cmd_vel_nav --field linear.x    # smoother input
ros2 topic echo /cmd_vel --field linear.x        # smoother output
```

If `/cmd_vel_nav` has non-zero values but `/cmd_vel` is all zeros, the
velocity smoother is filtering too aggressively. Check its config in
`go2_nav2/launch/lidar3d_controller.launch.py`.

### Dog shudders or steps in place while standing still

The smoother's deadband is too small. The lab-fixes doc added the smoother
specifically to stop this. If it's still happening:

1. Verify the lab fixes are applied (RPP controller, velocity smoother in
   the launch file)
2. Check `/cmd_vel` with no goal active: it should be silent, not 20 Hz of
   zeros

```bash
go2sh
ros2 topic hz /cmd_vel        # expect no data when idle
```

### Dog collides with walls or furniture

Two possibilities:

1. **Collision threshold too tight.** Use
   [`tune_costmap.sh --safe`](../scripts/tools/tune_costmap.sh) to
   restore `inflation_radius=0.70`, `footprint_padding=0.05`, or `--wide`
   (0.85) for crowded spaces. Do not go
   below `inflation_radius=0.20` — that's the Go2's inscribed radius.
2. **Obstacles aren't in the costmap.** Check `/scan` or `/lidar3d/registered`
   is reaching the costmap. Run
   [`verify_costmap.sh`](../scripts/tools/verify_costmap.sh) to confirm the costmap topic publishes.

### `mapping.sh: line 82: GO2_CONTAINER: Start the project Docker container first`

Same as the first entry — the container doesn't exist. Create it with
`./setup-lidar3d.sh --laptop`.

---

## Explorer

Explorer code: [`frontier_explorer.py`](../scripts/nodes/frontier_explorer.py).
Diagnose without the RViz button: [`test_explore.sh`](../scripts/tools/test_explore.sh).

### Explorer stays `IDLE` after clicking START

Either the button isn't publishing, or the explorer isn't subscribed.

```bash
go2sh
ros2 topic echo /explore/resume
# click START in RViz — expect: data: true
```

If no message arrives, the RViz panel isn't wired. Bypass it:

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"
```

If the dog then explores, the bug is in `go2_explore_panel`. Check the
plugin XML and `CMakeLists.txt` in
[`src/go2_explore_panel/`](../src/go2_explore_panel/).

### Explorer `state=EXPLORING`, `goals_sent=0`

The explorer enabled but frontier detection isn't producing anything.
Check in order:

1. **Costmap is publishing:**

   ```bash
   go2sh
   ros2 topic hz /global_costmap/costmap    # expect ~1 Hz
   ```

2. **Explorer is subscribed:**

   ```bash
   ros2 topic info /global_costmap/costmap --verbose
   # Subscription count should be 1
   ```

3. **Frontier detection is running.** Look at the log tail:

   ```bash
   docker exec go2-lidar3d tail -30 /tmp/frontier.log
   # should show: detect_frontiers: known=N frontiers=K
   ```

4. **If `frontiers=0`** while RViz shows unexplored edges:
   - Lower the cluster threshold:

     ```bash
     EXPLORE_MIN_FRONTIER_SIZE=2 ./scripts/runtime/wifi_nav.sh
     ```

   - Or drive the dog manually for 10 s to open up the map
   - Check the costmap topic name — some builds use a different one:

     ```bash
     ros2 topic list | grep costmap
     ```

     Then override:

     ```bash
     EXPLORE_MAP_TOPIC=/global_costmap/costmap_raw ./scripts/runtime/wifi_nav.sh
     ```

### Explorer loops on the same frontier

The blacklist isn't catching failed goals. Verify `step_explore()` computes
`tag` with the map origin offset (the block near `tag = (int(...))`). This
was fixed in the current code — if you still see the loop, the file may
have reverted.

### Explorer finds frontiers but sends no goals

`publish_goal()` early-returns if a goal is already active and hasn't timed
out. Watch `/explore/status` for `goals_sent`. If it stays at 0 while
`FRONTIER:` lines appear, the previous goal is never clearing.

Check `tick_active_goal()`:

- `dist < goal_xy_tol` → goal reached, clears
- `progress_timeout` → abandons, clears
- `goal_timeout` → abandons, clears

If none fire, `get_robot_pose()` is returning `None`:

```bash
go2sh
ros2 run tf2_ros tf2_echo lidar3d_map base_link
```

If this prints nothing, TF is broken. See the
[`Invalid frame ID` entry](#invalid-frame-id-lidar3d_map-on-tf-lookup) above.

### Explorer immediately finishes with `state=DONE`

The map is too sparse, or `min_known_cells` is never reached. Check the
log for `Waiting for map to grow: N/200`. If `N` is stuck at a low value,
the costmap isn't receiving data or is a rolling window that's mostly
unknown.

Solutions:

- Drive the dog manually for 10 s to open up the map
- Lower `EXPLORE_MIN_KNOWN_CELLS=50 ./scripts/runtime/wifi_nav.sh`

### Explorer's STOP button doesn't halt the dog

The emergency stop was fixed to publish on `/cmd_vel` (bypassing the
smoother) so it overrides Nav2's controller. Verify the fix is in the
file:

```bash
grep -A2 "def emergency_stop" scripts/nodes/frontier_explorer.py
```

Should show `self.stop_pub.publish(stop)`, not `self.cmd_vel_pub.publish`.
If it still uses `cmd_vel_pub`, the file reverted — re-apply the fix.

---

## D1-T Arm

Arm code: [`d1_arm_control.py`](../scripts/nodes/d1_arm_control.py),
[`d1_arm_calibration.py`](../scripts/nodes/d1_arm_calibration.py),
[`calibrate_arm.py`](../scripts/tools/calibrate_arm.py) and
[`start_arm.sh`](../scripts/runtime/start_arm.sh). Protocol and runbook:
[d1arm.md](d1arm.md).

The arm is wired to the VM's `enp0s8` (VM `192.168.123.99`, arm `.100`, DDS
domain 0, multicast discovery), **not** to the Go2. The arm scripts run with
the VM's `python3` and the pip `cyclonedds` package. Don't source ROS in that
shell.

### Arm does not respond to commands

1. **Run the preflight:** `./scripts/runtime/start_arm.sh --check` (address + ping).
2. **Run the diagnostic:** `ARM_IFACE=enp0s8 ./scripts/tools/diag_arm.sh`.
   It checks the interface, ping, container, `cyclonedds`, and DDS match.
3. **Check the address.** `enp0s8` needs `192.168.123.99`, which is lost on
   every VM reboot: `sudo ip addr add 192.168.123.99/24 dev enp0s8`.
4. **Check the arm is powered.** 24V DC via the barrel jack; allow ~1 min to boot.
5. **Read joint feedback:**

   ```bash
   ./scripts/runtime/start_arm.sh feedback
   ```

   If feedback arrives but commands do nothing, the motors may not be
   enabled. Run `enable` first.

### `does not reply to ping`

The cable, adapter, or VM networking is wrong, or the arm is still booting.
Check the link light, then cold-boot the VM (power off, not reboot) with
VirtualBox Adapter 2 set to **Intel PRO/1000**.

### `route to 192.168.123.100 does NOT go via enp0s8`

Another interface also has a `192.168.123.x` address, so traffic to the arm
leaves the wrong way. Remove the other address, or set `ARM_IFACE`.

### `DDS discovery failed`

No DDS match within `--match-timeout` (default 20 s). In order:

1. `pip3 show cyclonedds` on the VM. Fix: `pip3 install cyclonedds`.
2. Don't use `--peer` / unicast. This firmware only works with multicast.
3. Make sure ROS isn't sourced in that shell. ROS's `libddsc` can clash with the
   pip `cyclonedds` (`free(): invalid pointer`).
4. Give it a longer wait: `--match-timeout 40`.

### `joint N limit ±L°, got A°`

The target is outside the joint's limit (±135, ±90, ±90, ±135, ±90, ±135).
Use a smaller angle. `sweep --pct` keeps targets inside the limits.

### `wifi_nav.sh --arm` skipped the arm

Navigation continues without the arm. The banner shows `D1 arm: disabled (...)`.

- `enp0s8 not ready`: the interface is missing or has no IPv4 address. Run
  `sudo ip addr add 192.168.123.99/24 dev enp0s8`.
- `share a subnet`: `enp0s3` and `enp0s8` overlap, so arm and Go2 DDS would mix.
  Give `enp0s8` its `192.168.123.99` address.
- `arm start failed`: read `~/files/go2-nav2-wifi/ws/log/arm.log`. A
  `preflight failed` there means the address or ping check failed.

### Arm moves but reports wrong joint angles

Compare the raw telemetry to what you commanded (angles are **degrees**):

```bash
./scripts/runtime/start_arm.sh enable sleep 2 joint 0 20 sleep 3 feedback release
```

`feedback` shows the raw `rt/arm_Feedback` JSON (address 2, funcode 1,
`angle0`..`angle6`). To measure the offset per joint, run
`./scripts/runtime/start_arm.sh --calibrate --test-deg 10 --yes`.

---

## Camera

Full camera setup: [camera.md](camera.md). Relay code:
[`go2_camera_relay.py`](../scripts/nodes/go2_camera_relay.py).

### Relay crashes on start

Check `docker exec go2-lidar3d tail /tmp/camera_relay.log`.

- `No module named 'cv_bridge'`: an old copy of the relay. The current one
  fills the Image itself and needs only `python3-opencv`.
- `No module named 'unitree_go'`: the Unitree messages in
  `/opt/go2-unitree-msgs` weren't sourced. `wifi_nav.sh` sources them now.
- `No module named 'cv2'`:

  ```bash
  docker exec -u root go2-lidar3d apt-get update
  docker exec -u root go2-lidar3d apt-get install -y python3-opencv
  ```

### RViz camera display says "No Image"

1. Is the robot sending frames?

   ```bash
   ssh unitree@<robot-ip> tail -3 /tmp/go2_camera_wifi.log
   ```

   - `NN frames in 10 s` with NN > 0: the robot side is fine, go to step 2.
   - `0 frames in 10 s`: the camera backend gets no video. Check
     `~/bin/go2_camera_jpeg_cli` exists (go2-nav2-wifi
     `robot-build-camera-cli.sh`). Run it alone for a few seconds; it should
     print binary JPEG data.
   - No log: `wifi_nav.sh` couldn't copy or start it. See the `[camera]` lines
     in its output.
2. Is the laptop relay decoding? `docker exec go2-lidar3d tail /tmp/camera_relay.log`
   should show `published N frames`.
3. Is the RViz display set to **Reliability Policy: Reliable**? A Best Effort
   display drops most of the large raw frames (older layouts had Best Effort;
   `wifi_nav.sh` updates them on the next start).

### Robot sends frames, relay publishes none

`/tmp/go2_camera_wifi.log` on the robot says `50 frames in 10 s`, but
`camera_relay.log` never reaches `published 100 frames`. The relay was on the
container's default DDS (FastDDS) and never discovered the robot over Wi-Fi.
`wifi_nav.sh` now starts it with the session's CycloneDDS config (robot as
a peer); update and restart the session. Also check the robot publisher is
still running (`pgrep -f go2_camera_wifi.py` on the robot): it is started
with `setsid`, because with `nohup` alone it died with the SSH session.

### Camera at ~0.2 fps, "Camera unavailable" in the dashboard, empty 3D map

The robot log says `50 frames in 10 s`, but `camera_relay.log` counts 100
frames every few minutes. The laptop drops the frames: check

```bash
grep -A1 '^Udp:' /proc/net/snmp      # RcvbufErrors growing = socket buffers overflow
sysctl net.core.rmem_max             # 212992 = the Ubuntu default, too small
```

A 73 KB frame is ~50 UDP packets; one lost packet loses the frame. The same
overflow starves RTAB-Map in camera mode (`Did not receive data`), so the map
stays empty and YOLO and the VLM get no images. Raise the buffers once (see
[setup.md](setup.md#3d-digital-twin-and-ai--one-time-setup)), then restart
the session:

```bash
printf 'net.core.rmem_max=16777216\nnet.core.rmem_default=8388608\n' | sudo tee /etc/sysctl.d/60-go2-dds.conf
sudo sysctl --system
```

### Camera feed stutters or lags

The robot sends 960x540 JPEG at 5 fps (~0.3 MB/s). On a weak Wi-Fi link,
lower it, e.g. `CAMERA_WIDTH=640 CAMERA_FPS=3 ./scripts/runtime/wifi_nav.sh`
(recalibrate at the new width). Closing heavy RViz point cloud displays also
helps, because RViz renders on the CPU.

---

## RViz / VM

### RViz renders slowly with the point cloud visible

RViz renders on the CPU inside the container. To improve:

- Close the heavy point cloud display while tuning
- Untick displays you aren't using
- Use bridged networking (not NAT) on the VM
- Give the VM 4+ cores

### `view_frames` PDF is empty

`view_frames` needs the TF buffer populated. Wait until a session is fully
up and `/lidar3d/registered` is publishing at ~10 Hz, then re-run.

### RViz opens with the wrong layout

RViz loads its default layout unless told otherwise. To load your saved
layout:

```bash
go2sh
rviz2 -d ~/files/Unitree_GO2_Agricultural/config/tour.rviz
```

If [`config/tour.rviz`](../config/tour.rviz) doesn't exist, see [tour.md](tour.md) for how to save it.

### `rqt_reconfigure` shows no costmap nodes

The costmap nodes must be running for `rqt_reconfigure` to see them. Confirm
with:

```bash
go2sh
ros2 node list | grep costmap
```

If empty, the session isn't fully up. Wait longer, or check the
`mapping.sh --3d --nav` log for errors.

---

## Simulation

Gazebo simulation problems (startup, rendering, `simarm0`, arm emulator): see
[gazebo/README.md → Troubleshooting](../gazebo/README.md#troubleshooting).
`./gazebo/test_sim.sh` checks every simulated topic, motion and the arm.

---

## 3D Digital Twin

`wifi_3d_nav.sh` problems (image missing, dashboard not loading or not
connecting, empty
`/rtabmap/cloud_map`, clicks ignored, camera not detected, wrong map colours,
YOLO obstacles ignored by Nav2, Ollama unreachable, no battery data, `--resume`
maps not joined, CPU load, unsaved map, Foxglove
*"Error creating WebGL context"* in a VirtualBox VM): see
[3d_nav.md → Troubleshooting](3d_nav.md#troubleshooting). Logs are in
`~/files/go2-nav2-wifi/ws/log/3d/` (`rtabmap.log`, `bridge.log`,
`click_to_goal.log`, `yolo.log`, `vlm.log`, `battery.log`, `camera_info.log`,
`lidar_depth.log`, `rosbridge.log`, `web.log`, `image_compressor.log`) and
`docker logs go2-3d`.

---

## Known blockers and fixes

### Robot IP not found / DHCP changed

`wifi_nav.sh` stops with `could not find the Go2 on enp0s3` when discovery fails.
Find the robot's address in the router's DHCP lease list, or run `hostname -I`
on the Jetson, then:

```bash
./scripts/runtime/wifi_nav.sh --robot-ip <IP>
```

The address is cached, so the next run checks it first. If the Jetson's Wi-Fi
MAC doesn't start with `02:00:00`, set `GO2_MAC_PREFIX` to the real prefix.

### Container crashes with `numpy.dtype size changed`

`wifi_nav.sh` pins `numpy<1.25` inside the running container before launch. It
also appends the pin to the upstream Dockerfiles in `go2-nav2-wifi/docker/`,
including `Dockerfile.lidar3d`, so rebuilds keep it. It installs as root, then
restores the file's user; `.dockerignore` files are left alone. If the pip
install fails (often: no internet on the robot's Wi-Fi), it warns and continues.
Run the pin once with internet. If the container exits before the pin can be
applied, rebuild it:

```bash
docker rm -f go2-lidar3d
cd ~/files/go2-nav2-wifi && ./setup-lidar3d.sh --laptop
```

### Jetson relay port 17999 conflict

If the motion relay fails with `OSError: [Errno 98] Address already in use`,
a stale process is holding port 17999. **`wifi_nav.sh` handles this
automatically.** It kills stale relays over SSH before starting the new one.

> With Jetson compute (the default), upstream `mapping.sh` starts its own
> motion bridge on 17999 (`robot-motion-wifi.sh`), so `wifi_nav.sh` no longer
> starts a second FULL relay on the same port. If `motion.log` still shows
> "Address already in use", run with `--no-relay`. Details:
> [docs/STATUS.md](STATUS.md).

If you need to do it manually:

```bash
ssh unitree@<robot-ip>       # shown in the wifi_nav.sh banner
pkill -9 -f 'robot-relay-wifi|robot-motion-wifi|go2_cmd_vel_tcp|robot_sport_bridge'
sleep 3
ss -tlnp | grep 17999        # must be empty
```

### Explorer produces no goals (`goals_sent=0`)

Check in order:

1. **Costmap is publishing:** `ros2 topic hz /global_costmap/costmap` — expect ~1 Hz
2. **Explorer is subscribed:** `ros2 topic info /global_costmap/costmap --verbose` — Subscription count should be 1
3. **Frontier detection is running:** explorer logs `detect_frontiers: known=N frontiers=K` every 5 s
4. If `frontiers=0`, lower `EXPLORE_MIN_FRONTIER_SIZE` or drive the dog manually for 10 s to open up the map

### Dog doesn't move

If `/cmd_vel` publishes at 20 Hz with real values but the dog stands still:

1. Check the Jetson relay: `ssh unitree@<robot-ip> 'ss -tlnp | grep 17999'`
2. Check the relay log: `ssh unitree@<robot-ip> 'tail -20 /tmp/robot-relay.log'`
3. Confirm FULL mode: the log should say `cmd_vel TCP server started on port 17999`

### Arm does not respond

1. **Run the diagnostic:** `ARM_IFACE=enp0s8 ./scripts/tools/diag_arm.sh`.
   It checks the interface, ping, container, `cyclonedds`, and DDS match.
2. **Check the address.** `enp0s8` needs `192.168.123.99`:
   `sudo ip addr add 192.168.123.99/24 dev enp0s8`, then
   `./scripts/runtime/start_arm.sh --check`.
3. **Check the arm is powered.** 24V DC via the barrel jack; allow ~1 min to boot.
4. **Check `cyclonedds`** on the VM: `pip3 show cyclonedds`. Discovery is
   multicast; don't pass `--peer`. Don't source ROS in the same shell.
5. **Read the joint feedback:**

   ```bash
   ./scripts/runtime/start_arm.sh feedback
   ```

   If feedback arrives but commands do nothing, the motors may not be enabled.
   Run `enable` first.
6. **`wifi_nav.sh --arm` skipped the arm?** Read `go2-nav2-wifi/ws/log/arm.log`.
   A preflight failure there means the address or ping check failed; run
   `./scripts/runtime/start_arm.sh --check`. See also
   [Arm — calibration](setup.md#arm--calibration-once).

---

## Collision threshold tuning

Nav2's collision avoidance is controlled by two dynamic parameters on the
costmaps: `inflation_radius` and `footprint_padding`. Recommended values:

| Parameter | Safe | Wide | Tight | Notes |
|---|---|---|---|---|
| `inflation_radius` | `0.70` | `0.85` | `0.45` | Must be ≥ the Go2's inscribed radius (~0.20 m) |
| `footprint_padding` | `0.05` | `0.05` | `0.02` | Extra buffer around the robot outline |

Change at runtime without restarting:

```bash
./scripts/tools/tune_costmap.sh --safe              # restore 0.70 / 0.05
./scripts/tools/tune_costmap.sh --wide              # 0.85 / 0.05, crowded spaces
./scripts/tools/tune_costmap.sh --tight             # 0.45 / 0.02, small spaces
./scripts/tools/tune_costmap.sh 0.65 0.05           # custom
./scripts/tools/tune_costmap.sh --show              # current values
```

Every change also sets `cost_scaling_factor=3.0` for a smoother cost gradient.
`wifi_nav.sh` starts with `SAFETY_INFLATION_RADIUS=0.70`.

Or use `rqt_reconfigure` for sliders:

```bash
go2sh
ros2 run rqt_reconfigure rqt_reconfigure
```

To persist values across sessions, edit `go2_nav2/config/lidar3d_planner.yaml`
and the matching override in `go2_nav2/launch/lidar3d_controller.launch.py`.

---

## Diagnostics cheat sheet

| Symptom | First command |
|---|---|
| Dog doesn't move | `ssh unitree@<robot-ip> 'ss -tlnp \| grep 17999'` |
| Explorer does nothing | `docker exec go2-lidar3d tail -30 /tmp/frontier.log` |
| Costmap missing | `go2sh; ros2 topic list \| grep costmap` |
| TF broken | `go2sh; ros2 run tf2_ros tf2_echo lidar3d_map base_link` |
| Camera empty | `go2sh; ros2 topic hz /camera/image_raw` |
| Arm unresponsive | `ARM_IFACE=enp0s8 ./scripts/tools/diag_arm.sh` |
| Robot / host IP wrong | `./scripts/runtime/wifi_nav.sh --check` (banner shows both IPs) |
| Full reset | `./scripts/runtime/wifi_nav.sh --clean` then relaunch |
| No rooms in the dashboard | `tail ~/files/go2-nav2-wifi/ws/log/3d/rooms.log` |
| 3D map / dashboard empty | `tail ~/files/go2-nav2-wifi/ws/log/3d/rtabmap.log` and `rosbridge.log` |
| Low battery, dog doesn't go home | `tail ~/files/go2-nav2-wifi/ws/log/3d/battery.log` |
| No `/vlm_decision` | `tail ~/files/go2-nav2-wifi/ws/log/3d/vlm.log`; `curl localhost:11434/api/tags` |
| Verify everything | [`./scripts/tools/preflight.sh`](../scripts/tools/preflight.sh) |

---

## Reporting a new issue

Include:

1. The exact command that failed
2. Full error output (do not truncate)
3. Which component (`wifi_nav`, `mapping.sh`, `frontier_explorer`, arm,
   camera, etc.)
4. The session log folder from `~/files/go2-nav2-wifi/ws/log/mapping.*/`

The session log folder contains:

- `slam.log` — Point-LIO and sensor relay output
- `motion.log` — cmd_vel TCP server and sport bridge
- `rviz.log` — RViz stdout/stderr

Those three files answer most questions.

See [STATUS.md](STATUS.md) for session-by-session notes and
[roadmap.md](roadmap.md) for milestones.
