# Troubleshooting

Every failure mode encountered during development, with the actual fix.

## How to Use This Document

Each section is a self-contained issue. Search for the error message you see. If your problem is not here, the "Reporting a New Issue" section at the bottom lists what information to gather.

---

## ROS 2 Environment

### `AMENT_TRACE_SETUP_FILES: unbound variable`

A shell script uses `set -u` and then sources a ROS 2 setup file. ROS 2 setup files reference undefined variables and abort under `set -u`.

**Fix:** Wrap every source of a ROS 2 setup file with `set +u` and `set -u`.

```bash
set +u
source /opt/ros/humble/setup.bash
source ~/files/autonomy_stack_go2/install/setup.bash
set -u
```

Every script in this repo already does this. If you write a new one, follow the same pattern.

---

### `Package 'go2_integration_pkg' not found`

ROS 2 cannot find the package.

**Causes:**

1. You did not run `source install/setup.bash` after building.
2. The build failed silently.
3. You are in a different shell than the one you built in.

**Diagnose:**

```bash
ros2 pkg prefix go2_integration_pkg
echo $AMENT_PREFIX_PATH
```

If `ros2 pkg prefix` errors, the package is not installed.

**Fix:**

```bash
cd ~/files/Unitree_GO2_Agricultural
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | grep go2_integration_pkg
```

---

### `colcon build` fails with `cannot find -lXXX`

A C++ dependency is missing. The integration package is Python only, so this usually happens when building the autonomy stack or the WebRTC SDK.

**Fix:** Install the missing library. For the autonomy stack, the common missing pieces are:

```bash
sudo apt install -y \
    libusb-dev \
    ros-humble-perception-pcl \
    ros-humble-sensor-msgs-py \
    ros-humble-tf-transformations \
    ros-humble-joy \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl
```

---

### `colcon build` succeeds but `ros2 pkg executables` is empty

The `install(PROGRAMS ...)` block in `CMakeLists.txt` does not list the scripts.

**Check:**

```bash
grep -A 10 "install(PROGRAMS" src/go2_integration_pkg/CMakeLists.txt
```

The list must include all six nodes:

```
move_forward.py
cloud_relay_node.py
cmd_vel_bridge.py
map_node.py
pointcloud_to_scan.py
camera_relay.py
```

If any are missing, add them and rebuild.

---

## CycloneDDS

### `wlp2s0: does not match an available interface`

The CycloneDDS XML pins an interface name that does not exist on this machine.

**Find the real name:**

```bash
ip -o link show | awk -F': ' '{print $2}' | grep -v lo
```

Typical output: `enp0s3`, `enp0s8`, `enp0s9`.

**Fix:** Edit the `<NetworkInterface name="...">` in `config/cyclonedds_ethernet.xml` or `cyclonedds_wireless.xml` to match.

---

### `failed to increase socket receive buffer size to at least 4194304 bytes`

The XML has a `<SocketReceiveBufferSize>` element larger than the kernel allows.

**Fix A (recommended):** Remove the `<SocketReceiveBufferSize>` element from the XML.

**Fix B:** Raise the kernel limit.

```bash
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.core.wmem_max=2147483647

echo "net.core.rmem_max=2147483647" | sudo tee -a /etc/sysctl.d/60-cyclonedds.conf
echo "net.core.wmem_max=2147483647" | sudo tee -a /etc/sysctl.d/60-cyclonedds.conf
sudo sysctl --system
```

---

### `Failed to find a free participant index for domain 0`

CycloneDDS ran out of participant slots. This happens with many nodes and stale participants from crashed runs.

**Fix:**

1. Raise `MaxAutoParticipantIndex` in the XML.

```xml
<Discovery>
  <ParticipantIndex>auto</ParticipantIndex>
  <MaxAutoParticipantIndex>500</MaxAutoParticipantIndex>
  ...
</Discovery>
```

2. Kill stale processes.

```bash
ros2 daemon stop
pkill -9 -f cyclonedds
pkill -9 -f ros2
pkill -9 -f pointlio
pkill -9 -f rviz2
sleep 3
```

3. Relaunch.

If the problem persists after both, reboot the VM.

---

### Two machines can't see each other's topics

Symptom: `ros2 topic list` on host A shows topics. Host B is empty.

**Check in this order:**

1. Same RMW implementation:

```bash
echo $RMW_IMPLEMENTATION   # must be rmw_cyclonedds_cpp on both
```

2. Same domain ID:

```bash
echo $ROS_DOMAIN_ID        # must match on both
```

3. Same CycloneDDS XML:

```bash
echo $CYCLONEDDS_URI
```

4. Network reachability:

```bash
ping <other-host>
```

5. Firewall (rare):

```bash
sudo iptables -L -n | head -20
```

If all checks pass but topics still do not appear, the XML might be pinning the wrong interface. Confirm with `ip addr show`.

---

## Point-LIO

### `IMU Initializing: 100.0%` then silence, `/state_estimation` empty

The most common Point-LIO failure. Three causes.

**Cause 1: `use_sim_time: true`**

Edit `~/files/autonomy_stack_go2/src/slam/point_lio_unilidar/config/utlidar.yaml`:

```yaml
use_sim_time: false
```

Rebuild:

```bash
cd ~/files/autonomy_stack_go2
colcon build --packages-select point_lio_unilidar --symlink-install
```

**Cause 2: Wrong topic names**

Check the config.

```bash
grep -E "lid_topic|imu_topic" \
    ~/files/autonomy_stack_go2/src/slam/point_lio_unilidar/config/utlidar.yaml
```

Must match what the Jetson publishes:

```yaml
lid_topic: "/utlidar/cloud"
imu_topic: "/utlidar/imu"
```

**Cause 3: `transform_everything` crashed**

The autonomy stack includes a Python node that transforms raw topics. If it crashes, Point-LIO gets no data.

Fix: comment out its include in `system_real_robot.launch` and rebuild `vehicle_simulator`.

---

### RViz shows `No transform to fixed frame [map]`

Point-LIO has not published the `map -> camera_init` transform yet.

**Causes:**

1. Point-LIO is stuck (see previous section).
2. It is too early. Wait 30 seconds after launch.
3. The robot has not moved enough for initialization.

**Workaround:** In RViz, change the Fixed Frame from `map` to `body` or `base_link` until the map frame appears.

---

## Mapping Node

### `map_node` starts but `/map/occupancy` is not visible in RViz

**Cause 1: QoS mismatch**

RViz requires `TRANSIENT_LOCAL` durability for the Map display. Check:

```bash
ros2 topic info /map/occupancy -v
```

The publisher must show `Durability: TRANSIENT_LOCAL`. If it shows `VOLATILE`, the node was built from an older version. Rebuild.

**Cause 2: Topic name mismatch in RViz**

In the Map display settings, confirm the Topic field says exactly `/map/occupancy`.

---

### `map_node` runs but no data arrives

**Cause: Cloud topic does not exist**

Check the node startup log. It prints the topics it auto-detected:

```
map_node started
   cloud in : /registered_scan
   odom in  : /state_estimation
```

If the auto-detection picked a topic that has no publisher, force it manually.

```bash
ros2 run go2_integration_pkg map_node.py \
    --ros-args \
    -p cloud_topic:=/utlidar/cloud_deskewed \
    -p odom_topic:=/utlidar/robot_odom
```

**Cause: TF tree is incomplete for RGB fusion**

If `use_camera` is true but no TF connects `map` to the camera frame, the fusion silently skips. The map still builds, but without color. Check the terminal for repeated "TF lookup failed" debug messages.

---

### `map_node` crashes on Ctrl+C

The save routine runs on shutdown. If `open3d` is missing, the mesh reconstruction raises and the node exits uncleanly.

**Fix:** Either install open3d, or disable the mesh step.

```bash
pip install --user open3d
```

The node logs "Reconstructing mesh..." before the crash if this is the cause.

---

### Mesh reconstruction is slow

Poisson reconstruction on 5+ million points takes minutes.

**Fix:** Reduce the input cloud size. Edit `core/mapping.py`:

```python
@dataclass
class MapConfig:
    max_points: int = 2_000_000      # reduce from 8 million
    downsample_voxel_m: float = 0.05  # coarser voxels
    poisson_depth: int = 8            # reduce from 9
```

Smaller values mean faster reconstruction and lower memory.

---

## SLAM Toolbox

### `slam_toolbox` does not start

The package is not installed.

```bash
sudo apt install ros-humble-slam-toolbox
```

If the launch file errors with "executable not found", verify:

```bash
ros2 pkg executables slam_toolbox
```

Should list `async_slam_toolbox_node`.

---

### SLAM map drifts or loops break

**Cause: Odometry is bad or missing.**

Check the odometry topic:

```bash
ros2 topic hz /odom
```

If it is silent, no odometry is being published. SLAM needs it. Point-LIO publishes `/state_estimation`, which is odometry-like. Add a static transform to remap if needed.

**Cause: Scan matches are too sparse.**

In `slam_mapping.launch.py`, the `minimum_time_interval` parameter is 0.2 seconds. At 10 Hz LiDAR, that means every other scan is used. Reduce to 0.1 for denser scans.

---

## Localization (AMCL)

### Robot pose jumps around

**Cause: Wrong initial pose.**

Click `2D Pose Estimate` in RViz. Drag on the map at the robot's actual location and heading. AMCL converges after a few seconds of motion.

**Cause: Scan does not match the map.**

The LiDAR range or height filter may be wrong. Check that `/scan` matches the walls in the map. If not, adjust `min_height` and `max_height` in `pointcloud_to_scan.py`.

---

### `AMCL` does not publish

The lifecycle manager may not have activated it.

```bash
ros2 lifecycle get /amcl
```

Should show `active`. If it shows `unconfigured`, the lifecycle manager failed. Check:

```bash
ros2 node list | grep lifecycle
```

Should list `lifecycle_manager_localization`. If not, `localization.launch.py` failed to start.

---

## Nav2 Navigation

### Nav2 does not start

The full stack needs many packages. Install them.

```bash
sudo apt install -y \
    ros-humble-nav2-bringup \
    ros-humble-nav2-amcl \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-controller \
    ros-humble-nav2-planner \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-behaviors \
    ros-humble-nav2-navfn-planner \
    ros-humble-nav2-regulated-pure-pursuit-controller
```

---

### Nav2 starts but the robot does not move

**Cause 1: No goal set.**

Click `2D Goal Pose` in RViz and drag. A green arrow appears. The planner produces a blue path.

**Cause 2: `/cmd_vel` is not consumed.**

Check the topic:

```bash
ros2 topic hz /cmd_vel
```

If it shows nothing, the controller is not producing commands. Check the costmap:

```bash
ros2 topic echo /local_costmap/costmap --once
```

If the costmap is empty, the local planner has no obstacle data and produces no path.

**Cause 3: Costmap does not see obstacles.**

The costmap layers read from `/scan`. Verify:

```bash
ros2 topic hz /scan
```

If silent, `pointcloud_to_scan.py` is not running or its input topic is wrong. Check its terminal output.

---

### Nav2 goal is rejected

**Cause: Goal is outside the loaded map, or in an obstacle.**

The Nav2 planner only accepts goals in free space within the map bounds. Try a goal closer to the robot.

**Cause: Wrong map loaded.**

`localization.launch.py` loads the map from `map_file`. Confirm the path is correct.

```bash
ls -la ~/go2_maps/my_map.yaml
```

---

## Robot Motion

### Robot does not move when a waypoint is set

**Prerequisite: Robot is standing.**

The autonomy stack does not stand the robot. Use the physical remote.

**Prerequisite: Robot is in Sport Mode.**

Default after standing. Verify in the Unitree app.

**Check `/cmd_vel`:**

```bash
ros2 topic hz /cmd_vel
```

If silent, the planner is not producing commands. If publishing, the issue is between the VM and the Jetson.

**On the Jetson side:**

```bash
ssh unitree@192.168.123.18
source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

ros2 topic hz /api/sport/request
```

If silent, the VM is not sending commands. If publishing, the robot itself is refusing. Check the app for estop, lock state, or low battery.

---

### `move_forward.sh` runs but nothing happens

**Checklist:**

1. Robot is standing.
2. Robot is in Sport Mode.
3. `ping 192.168.123.18` works.
4. `ROS_DOMAIN_ID=0` in the script.
5. The script printed `Moving forward: vx=0.3 m/s for 3.0s`.

If all pass and the robot still does not move, the Unitree Sport API rejected the command.

Verify message type:

```bash
ros2 topic info /api/sport/request
```

Should show `unitree_api/msg/Request`. A different type means the wrong package was sourced.

---

## Camera

### `/camera/image_raw` does not exist

**Cause: WebRTC SDK not running.**

The camera stream comes from the WebRTC SDK, not from native DDS. Start it in a separate terminal.

```bash
source /opt/ros/humble/setup.bash
source ~/files/ros2_ws/install/setup.bash
export ROBOT_IP="192.168.137.33"
export CONN_TYPE="webrtc"

ros2 launch go2_robot_sdk robot.launch.py \
    rviz2:=false nav2:=false slam:=false \
    foxglove:=false joystick:=false teleop:=false
```

Wait for `Robot 0 validated and ready`.

**Cause: Ethernet-only setup.**

If you are running the CMU stack over Ethernet and the WebRTC SDK is not active, the camera stream is unavailable. Only the LiDAR and IMU are published natively. To get the camera over Ethernet, you would need a USB camera on the Jetson, or the WebRTC SDK running in parallel.

---

### Camera image appears in RViz but is black or garbled

**Cause 1: H.264 decoder failure.**

The WebRTC SDK logs many `non-existing PPS 0 referenced` warnings. These are harmless. The actual image appears once the decoder syncs to the keyframe.

**Cause 2: QoS mismatch.**

RViz requires BEST_EFFORT for high-rate image topics. `camera_relay.py` republishes with the correct QoS. Run it.

```bash
ros2 run go2_integration_pkg camera_relay.py
```

Subscribe in RViz to `/camera/image_raw_relayed`, not `/camera/image_raw`.

---

### RGB fusion produces grey points instead of coloured

**Cause: TF lookup fails.**

The `map_node.py` needs a transform from the LiDAR frame to the camera optical frame. Check the TF tree:

```bash
ros2 run tf2_tools view_frames
```

Open `frames.pdf`. Look for a path from `map` to `camera_color_optical_frame`.

If the path is missing, the camera driver is not publishing its static transforms. Check the WebRTC SDK output or add a static publisher.

```bash
ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 \
    map camera_color_optical_frame
```

---

## Wireless

### Jetson has no Wi-Fi interface

**Confirmed by:**

```bash
ssh unitree@192.168.123.18
ip link
```

If no `wlan0`, `wlp*`, or `wlx*` appears, the Jetson has no Wi-Fi chipset. Only the front board has Wi-Fi, and it does not route SSH or DDS.

**Fix:** Add a USB Wi-Fi dongle or use a travel router. See `docs/wireless.md`.

---

### Wi-Fi dongle not detected on the Jetson

```bash
lsusb
```

If the dongle does not appear, try a different USB port. Check `dmesg | tail -20` for kernel messages.

**Known-good chipsets:** Ralink RT5572 (Panda PAU09), Atheros AR9271 (Alfa AWUS036NHA).

**Known-problematic:** Realtek RTL8812BU (requires custom driver compile), TP-Link TL-WN722N v2/v3.

---

### Multicast discovery fails on Wi-Fi

**Cause: Access point blocks multicast.**

Native DDS discovery uses multicast. Most consumer Wi-Fi routers block it.

**Fix:** Edit `config/cyclonedds_wireless.xml` to use explicit peers.

```xml
<Discovery>
  <Peers>
    <Peer address="192.168.1.100"/>
    <Peer address="192.168.1.101"/>
  </Peers>
</Discovery>
```

Also set `<AllowMulticast>false</AllowMulticast>`.

Replace the addresses with the actual Wi-Fi IP of the workstation and the Jetson.

---

## SSH

### `ssh: connect to host 192.168.123.18 port 22: Connection refused`

**Cause 1: Wrong IP.**

The front board at `192.168.137.33` does not run sshd. Use the Jetson's Ethernet IP `192.168.123.18`.

**Cause 2: sshd is bound to an internal interface.**

Verify with `sudo ss -tlnp | grep :22` on the Jetson. If it shows `0.0.0.0:22` or `:::22`, sshd is listening on all interfaces. If it shows only `192.168.123.18:22`, it is bound to Ethernet only. Wi-Fi SSH would fail.

**Fix (only if you need Wi-Fi SSH):** Edit `/etc/ssh/sshd_config` on the Jetson. Comment out any `ListenAddress` line. Restart sshd.

```bash
sudo systemctl restart ssh
```

---

### `Permission denied, please try again` repeatedly

**Cause: Wrong username.**

Windows SSH defaults to your Windows username. Always specify the Unix user.

```bash
ssh unitree@192.168.123.18
```

Password is `123` on the EDU unit.

---

## Git

### `Permission to <user>/<repo>.git denied`

You are using a GitHub password. GitHub requires a Personal Access Token or an SSH key.

**Fix (token):**

1. Go to https://github.com/settings/tokens
2. Generate new token (classic). Scope: `repo`.
3. Copy the token.
4. `git push` and paste the token as the password.

**Fix (SSH, recommended long-term):**

```bash
ssh-keygen -t ed25519 -C "you@example.com"
cat ~/.ssh/id_ed25519.pub
```

Add the output to https://github.com/settings/keys. Then:

```bash
git remote set-url origin git@github.com:USER/REPO.git
git push
```

---

## General

### `bash: ./scripts/foo.sh: Permission denied`

Missing executable bit.

```bash
chmod +x scripts/*.sh
```

---

### `bash: ./scripts/foo.sh: /bin/bash^M: bad interpreter`

Windows CRLF line endings.

```bash
sudo apt install -y dos2unix
dos2unix scripts/*.sh
```

Or:

```bash
sed -i 's/\r$//' scripts/*.sh
```

---

### RViz camera warnings `/camera/image/raw`

Harmless. The CMU RViz config references camera topics that may not exist. Ignore them.

---

### RViz runs at 15 fps or lower

Reduce load:

1. Disable the RegScan display.
2. Set the PointCloud2 decay time to 0.
3. Reduce the Map display alpha.
4. Close unused Image panels.

If RViz is still slow, run it on a lightweight window manager or use `rviz2 -d minimal.rviz`.

---

## Reporting a New Issue

If your problem is not listed, gather this information before filing an issue or asking for help.

1. Full error message.
2. Output of:

```bash
echo "RMW:        $RMW_IMPLEMENTATION"
echo "DOMAIN_ID:  $ROS_DOMAIN_ID"
echo "CYCLONEDDS: $CYCLONEDDS_URI"
```

3. `ros2 node list`
4. `ros2 topic list`
5. The exact command that failed.
6. The last 30 lines of the failing terminal.

With that information, the cause is usually identifiable in one pass.
```

Save it and commit.

```powershell
git add docs/troubleshooting.md
git commit -m "Update TROUBLESHOOTING.md with mapping, Nav2, camera, and wireless sections"
git push