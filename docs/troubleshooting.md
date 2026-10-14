# Troubleshooting

[← Back to README](../README.md) · [Status](STATUS.md) · [Roadmap](roadmap.md)

Every failure mode encountered during development, with the actual fix.
Sections are self-contained. Search for the error message you see.

**Jump to:** [Session / Infrastructure](#session--infrastructure) ·
[Motion](#motion) · [Explorer](#explorer) · [D1-T Arm](#d1-t-arm) ·
[Camera](#camera) · [RViz / VM](#rviz--vm) ·
[Diagnostics cheat sheet](#diagnostics-cheat-sheet)

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
ssh unitree@192.168.1.9
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

```
cmd_vel TCP server started on port 17999
```

If that is missing, the motion relay failed. See the
[port 17999 entry](#address-already-in-use-for-port-17999-on-the-jetson) above.

### Container has stale `GO2_HOST_IP`

The container's DDS bind address is baked in at creation time. If it's
wrong:

```bash
docker rm -f go2-lidar3d
cd ~/files/go2-nav2-wifi
export GO2_HOST_IP=192.168.1.4 GO2_ROBOT_IP=192.168.1.9
./setup-lidar3d.sh --laptop
```

Verify:

```bash
docker exec go2-lidar3d printenv GO2_HOST_IP        # expect 192.168.1.4
docker exec go2-lidar3d cat /tmp/go2-wifi-cyclonedds.xml | grep -i address
```

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
ssh unitree@192.168.1.9
ss -tlnp | grep 17999
tail -30 /tmp/robot-relay.log
```

Look for `cmd_vel TCP server started on port 17999`. If it's missing, the
relay started in sensor-only mode. Restart it in FULL mode:

```bash
ssh unitree@192.168.1.9
pkill -9 -f 'robot-relay-wifi|robot-motion-wifi|go2_cmd_vel_tcp|robot_sport_bridge'
rm -f /tmp/go2-relay-wifi.sock*
sleep 3
unset GO2_HOST_IP
export GO2_HOST_IP=192.168.1.4
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
   restore `inflation_radius=0.35`, `footprint_padding=0.02`. Do not go
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

Arm code: [`d1_arm_control.py`](../scripts/nodes/d1_arm_control.py) and
[`start_arm.sh`](../scripts/runtime/start_arm.sh).

### Arm does not respond to commands

1. **Check the interface name.** Inside the container:

   ```bash
   go2sh
   ip -brief link
   ```

   The `--iface` flag must match the interface the arm is on.

2. **Check the arm is powered.** 24V DC via the barrel jack. The arm's
   controller should show activity on power.

3. **Check the RJ45.** The arm should be on the same network segment as
   the Go2's onboard switch.

4. **Verify the SDK2 is installed:**

   ```bash
   docker exec -u root go2-lidar3d bash -lc 'python3 -c "import unitree_sdk2py; print(\"ok\")"'
   ```

5. **Read joint feedback:**

   ```bash
   docker exec -u root go2-lidar3d bash -lc '
       python3 /tmp/d1_arm_control.py --iface <iface> feedback
   '
   ```

   If feedback arrives but commands do nothing, the motors may not be
   enabled — run `enable` first.

### `ModuleNotFoundError: No module named 'unitree_sdk2py'`

The SDK isn't installed in the container. Install it:

```bash
docker exec -u root go2-lidar3d bash -lc '
    cd /tmp
    git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
    cd unitree_sdk2_python
    export CYCLONEDDS_HOME=/opt/ros/humble
    pip3 install -e .
'
```

### `Could not locate cyclonedds` during SDK2 install

The SDK needs cyclonedds headers. Two options:

**Option A — point at the ROS 2 install:**

```bash
docker exec -u root go2-lidar3d bash -lc '
    cd /tmp/unitree_sdk2_python
    export CYCLONEDDS_HOME=/opt/ros/humble
    pip3 install -e .
'
```

**Option B — build cyclonedds from source:**

```bash
docker exec -u root go2-lidar3d bash -lc '
    cd /tmp
    git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
    cd cyclonedds && mkdir build install && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=../install
    cmake --build . --target install
    cd /tmp/unitree_sdk2_python
    export CYCLONEDDS_HOME=/tmp/cyclonedds/install
    pip3 install -e .
'
```

### Arm moves but reports wrong joint angles in feedback

The arm's DDS topic may be publishing on a different scale or offset. Check
the raw message:

```bash
docker exec -u root go2-lidar3d bash -lc '
    source /opt/ros/humble/setup.bash
    ros2 topic echo /arm_state --once
'
```

Compare to the joint angles you commanded. If there's a scale or sign
difference, the parsing in `d1_arm_control.py::_on_feedback` needs
adjusting.

### Arm reachable from Jetson but not from VM container

The Wi-Fi relay may not be forwarding the arm's DDS domain. Check on the
Jetson:

```bash
ssh unitree@192.168.1.9
ros2 topic list | grep arm
```

If the arm appears on the Jetson but not in the container, the relay needs
to include the arm's DDS topics. This is an upstream
[`go2-nav2-wifi`](https://github.com/dancher00/go2-nav2-wifi) change,
not something the current scripts can fix.

---

## Camera

Full camera setup: [camera.md](camera.md). Relay code:
[`go2_camera_relay.py`](../scripts/nodes/go2_camera_relay.py).

### Relay crashes on start

Missing dependencies inside the container.

```bash
docker exec -u root go2-lidar3d apt-get update
docker exec -u root go2-lidar3d apt-get install -y \
    ros-humble-cv-bridge python3-opencv
```

### RViz Image display is empty

QoS mismatch. The relay publishes with `BEST_EFFORT`; RViz defaults to
`RELIABLE`. In the Image display properties, set **Reliability Policy** to
`Best Effort`. This is the single most common cause.

### `frame not JPEG-decodable` in `camera_relay.log`

Firmware ships raw H.264 instead of JPEG. Try lowering the resolution:

```bash
# In wifi_nav.sh, change -p resolution:=720p to -p resolution:=360p
```

Or install `python3-opencv` with H.264 support (see relay crashes above).

### Camera feed stutters or lags

Wi-Fi bandwidth. The relay publishes raw frames on `/camera/image_raw`
(~27 MB/s at 720p30). Options:

1. Lower resolution (`-p resolution:=360p`)
2. Move to compressed transport — see [camera.md](camera.md)
3. Close the RViz point cloud display while viewing the camera (RViz
   renders on the CPU)

### Camera unavailable warning on Jetson

```
WARN: camera unavailable; install its backend or use GO2_RELAY_CAMERA=0
```

Not a blocker for exploration. Either install the backend or run the relay
with `GO2_RELAY_CAMERA=0` to silence it.

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

## Diagnostics cheat sheet

| Symptom | First command |
|---|---|
| Dog doesn't move | `ssh unitree@192.168.1.9 'ss -tlnp \| grep 17999'` |
| Explorer does nothing | `docker exec go2-lidar3d tail -30 /tmp/frontier.log` |
| Costmap missing | `go2sh; ros2 topic list \| grep costmap` |
| TF broken | `go2sh; ros2 run tf2_ros tf2_echo lidar3d_map base_link` |
| Camera empty | `go2sh; ros2 topic hz /camera/image_raw` |
| Arm unresponsive | `go2sh; python3 /tmp/d1_arm_control.py --iface <iface> feedback` |
| Full reset | `./scripts/runtime/wifi_nav.sh --clean` then relaunch |
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