# Troubleshooting

[← Back to README](../README.md) · [Status](STATUS.md) · [Roadmap](roadmap.md)

Every failure mode encountered during development, with the actual fix.
Sections are self-contained. Search for the error message you see.

**Jump to:** [At a glance](#at-a-glance) ·
[Session / Infrastructure](#session--infrastructure) ·
[Robot Wi-Fi](#robot-wi-fi-usb-dongle) ·
[Motion](#motion) · [Explorer](#explorer) · [D1-T Arm](#d1-t-arm) ·
[Camera](#camera) · [Workstation](#native-ubuntu-workstation) · [RViz](#rviz) · [3D twin](#3d-digital-twin) ·
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
| Simulator won't start or RViz is empty in `--sim` | Image, rendering, or `simarm0` | [docs/simulation/gazebo.md](simulation/gazebo.md#troubleshooting), `./gazebo/test_sim.sh` |
| Explorer stays `IDLE` | Button not wired, or costmap silent | [`./scripts/tools/test_explore.sh`](../scripts/tools/test_explore.sh) |
| Camera at ~0.2 fps, dashboard "Camera unavailable", empty 3D map | Host UDP receive buffers too small (Ubuntu default) | Raise `net.core.rmem_max` / `rmem_default`, see [Camera at ~0.2 fps](#camera-at-02-fps-camera-unavailable-in-the-dashboard-empty-3d-map) |
| Camera shows "No Image" in RViz | Robot publisher not running, or display set to Best Effort | `ssh unitree@<robot> tail /tmp/go2_camera_wifi.log`; display **Reliability** `Reliable`. See [Camera](#camera) |
| Arm does not respond to commands | Wrong `--iface`, or motors not enabled | See [Arm does not respond](#arm-does-not-respond) above |
| `(53) The Wi-Fi network could not be found` on the Go2 | No regulatory domain (`country 00`): 5 GHz is passive-scan only | `sudo iw reg set US` + make it permanent. See [Robot Wi-Fi](#the-wi-fi-network-could-not-be-found-53) |
| `Secrets were required, but not provided` on the Go2 | Wi-Fi profile built in pieces; password unreadable | Delete it and recreate it with one `nmcli connection add`. See [Robot Wi-Fi](#secrets-were-required-but-not-provided) |
| `could not find the Go2 on <wifi iface>` | Robot on another Wi-Fi network, DHCP changed, or discovery failed | Same network? Then `wifi_nav.sh --robot-ip <IP>` — see [Robot IP not found](#robot-ip-not-found--dhcp-changed) |
| `SENSOR CLOCK FAULT` in `slam.log`; map and dog stop updating; twin exits after `waiting up to 180 s for a frame` | The Jetson's clock jumped (first NTP sync after boot) | Wait ~2 min after booting the dog, then relaunch. See [Workstation](#sensor-clock-fault-right-after-the-dog-boots) |
| `apt-get install` fails on `nvidia-dkms-open` / `cuda-drivers` | NVIDIA driver build needs gcc-12 | `sudo apt-get install -y gcc-12 && sudo dpkg --configure -a`. See [Workstation](#nvidia-driver-fails-to-build--ftrivial-auto-var-initzero) |
| `DDS discovery failed` (arm) | No DDS match within 20 s | Arm powered and booted? No `--peer`, no ROS sourced; check `diag_arm.sh` |
| RViz renders slowly | CPU rendering | Close the point cloud display while tuning |
| Foxglove 3D panel: `Error creating WebGL context` | No GPU acceleration: NVIDIA driver not loaded (not a code error) | Check `nvidia-smi` / `glxinfo -B`, finish the driver install, or use RViz. See [3d_nav.md](ai_and_vision/3d_nav.md#webgl--hardware-acceleration-error) |

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

### Build fails: `nav2_common` / `nav2_bringup` package configuration could not be found

Navigation2 isn't installed in the sourced ROS 2 environment:

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-nav2-common ros-$ROS_DISTRO-nav2-bringup
# or the whole stack:
sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup
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

Then relaunch the session on the workstation.

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
ip -4 -o addr show dev "$GO2_IFACE"                 # this computer's current Wi-Fi IP (or: hostname -I)
docker exec go2-lidar3d printenv GO2_HOST_IP        # must match
docker exec go2-lidar3d printenv GO2_ROBOT_IP       # must be the robot's current IP
docker exec go2-lidar3d cat /tmp/go2-wifi-cyclonedds.xml | grep -i address   # rewritten each session
./scripts/runtime/wifi_nav.sh --check               # reports STALE if they differ
```

If a stale IP is exported in your shell (e.g. `~/.bashrc`), `wifi_nav.sh`
ignores it with a note. Remove it to avoid confusion.

### `could not find the Go2 on <wifi iface>`

Robot discovery failed. Order tried: last known IP, hostname `unitree`, MAC
prefix scan (`02:00:00`) on the Wi-Fi interface, then an SSH-key probe.

0. Make sure the Go2 is on the **same Wi-Fi network** as the workstation. If
   it is cabled to the workstation, `getent hosts unitree.local` resolving to
   `192.168.123.161` means you only reach it over the cable: `ssh` in there
   and check `nmcli -t -f ACTIVE,SSID dev wifi` and `hostname -I`.
1. Find the address in the router's DHCP lease list, or run `hostname -I` on
   the Jetson, then `./scripts/runtime/wifi_nav.sh --robot-ip <IP>`. It is
   cached for next time.
2. If the Jetson's Wi-Fi MAC doesn't start with `02:00:00` (check `ip link` on
   the Jetson), set `GO2_MAC_PREFIX=<prefix>`.
3. Install `arp-scan` (`sudo apt install arp-scan`) or `nmap` for a faster,
   more reliable scan. Without them, a ping sweep is used.

### `route to robot ... goes via the arm interface <iface>`

The workstation would send Go2 traffic out the arm's Ethernet. Make sure the
Wi-Fi interface (`GO2_IFACE`) and the arm interface (`ARM_IFACE`,
`192.168.123.x`) are on different subnets, and that the Wi-Fi interface holds
the route to the robot's network (`ip route get <robot-ip>`).

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

## Robot Wi-Fi (USB dongle)

The Go2 joins the building Wi-Fi through a USB Wi-Fi dongle (`wlan0`, for
example a TP-Link) managed by NetworkManager on the Jetson. Two separate
problems can stack up and look like one. Run everything below on the robot
(`ssh unitree@<robot>`).

First, check that the hardware is fine: the dongle appears in `lsusb` and
`ip link` shows `wlan0`. If both are there, the problem is configuration, not
the dongle.

### The Wi-Fi network could not be found (53)

**Cause: no regulatory domain.** `iw reg get` shows `country 00`, the "world"
default. That is the most restrictive setting:

- every 5 GHz channel is **passive-scan only**, and the DFS ranges are
  limited further;
- in passive mode the dongle hears the network only when the router happens to
  send a beacon on the channel it is listening to;
- with a weak 5 GHz signal through the robot's body, it keeps missing it.

**Fix:** set the country, so the radio can actively scan and use the right
channels:

```bash
iw reg get                      # country 00 = the problem
sudo iw reg set US              # your country code
```

Make it permanent (survives a reboot):

```bash
echo "options cfg80211 ieee80211_regdom=US" | sudo tee /etc/modprobe.d/cfg80211.conf
```

### `Secrets were required, but not provided`

**Cause: a broken connection profile**, so the password is never used. This
happens when a profile is built in pieces: an attempt with `--ask`, a separate
`nmcli connection add`, a few `nmcli connection modify` commands, and a
`nmcli device wifi connect` interrupted with Ctrl+C. The password then ends up
stored in a way `nmcli connection up` can't read, for example flagged to be
typed in each time. NetworkManager keeps asking for a password it has no way
to prompt for.

**Fix:** delete the profile and recreate it with **one** command that sets
everything at once:

```bash
sudo nmcli connection delete "<SSID>"
sudo nmcli connection add type wifi ifname wlan0 con-name "<SSID>" ssid "<SSID>" \
    wifi-sec.key-mgmt wpa-psk wifi-sec.psk '<password>' connection.autoconnect yes
sudo nmcli connection up "<SSID>"
```

Success looks like this in `journalctl -u NetworkManager`: the network is
found, the 4-way handshake completes, and DHCP gives the robot an address from
the router (for example `192.168.1.4`). Then follow
[Robot IP not found](#robot-ip-not-found--dhcp-changed) if the session
doesn't find the new address.

### Not the cause: the USB device-mode service

Stopping or disabling the Go2's USB service does **not** fix (or break) the
dongle. That service controls USB *device* mode: the USB-C link that makes the
Go2 show up as a network device on a PC. It doesn't affect the USB *host*
ports the dongle is plugged into. Re-enabling it is optional and only matters
if you use that USB-C link.

**In short:** the country code lets the Go2 see the 5 GHz network, and
rebuilding the profile in one clean command lets it log in.

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
export GO2_HOST_IP=<host-wifi-ip>  # `hostname -I` on the workstation
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

If `ros2 topic info -v /explore/resume` lists the explorer but not `rviz`,
the two are on different DDS setups. Anything started with `docker exec` must
get the stack's `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and
`CYCLONEDDS_URI=file:///tmp/go2-wifi-cyclonedds.xml`. `wifi_nav.sh` does
this for the explorer, relays and stop (`DDS_EXEC_ENV`); add both with `-e`
when you run `ros2` commands by hand as root.

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

### Dog stuck in the purple costmap next to a box, `No frontier has a goal clear of walls`

The dog stopped facing an obstacle inside its inflation zone. Nav2's
controller aborts every new goal with `RegulatedPurePursuitController detected
collision ahead!` because it can't turn in place first. The explorer then
blacklists those frontiers. Drive it clear with 2D Goal Pose and click START
again, or try `EXPLORE_UNSTICK_ENABLE=true`. Details and the planned fix:
[explorer.md → Known issue](navigation/explorer.md#known-issue-stuck-next-to-an-obstacle).

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
[d1arm.md](hardware/d1arm.md).

The arm is wired to a port of the workstation (`ARM_IFACE`, auto-detected, e.g.
`enp0s31f6`; host `192.168.123.99`, arm `.100`, DDS
domain 0, multicast discovery), or plugged into the Go2 and reached over
Wi-Fi through the robot's bridge (`--robot <ip>` / `--arm-wifi`, see
[d1arm.md](hardware/d1arm.md#arm-over-wi-fi-plugged-into-the-go2); bridge problems are
in its troubleshooting table). Wired, the arm scripts run with the host's
`python3` and the pip `cyclonedds` package; don't source ROS in that shell.

### Arm does not respond to commands

1. **Run the preflight:** `./scripts/runtime/start_arm.sh --check` (address + ping).
2. **Run the diagnostic:** `./scripts/tools/diag_arm.sh` (interface
   auto-detected; set `ARM_IFACE` to force one).
   It checks the interface, ping, container, `cyclonedds`, and DDS match.
3. **Check the address.** The arm port needs `192.168.123.99`. It comes from
   the `go2-arm` NetworkManager profile whenever the cable is plugged in:
   `sudo nmcli connection up go2-arm` ([setup](setup/setup.md#arm--one-time-setup)).
4. **Check the arm is powered.** 24V DC via the barrel jack; allow ~1 min to boot.
5. **Read joint feedback:**

   ```bash
   ./scripts/runtime/start_arm.sh feedback
   ```

   If feedback arrives but commands do nothing, the motors may not be
   enabled. Run `enable` first.

### `does not reply to ping`

The cable or adapter is wrong, or the arm is still booting (allow ~1 min).
Check the link light (`cat /sys/class/net/<iface>/carrier` is `1`) and the
24V power, then power-cycle the arm.

### `route to 192.168.123.100 does NOT go via <iface>`

Another interface also has a `192.168.123.x` address, so traffic to the arm
leaves the wrong way. Remove the other address, or set `ARM_IFACE`.

### `DDS discovery failed`

No DDS match within `--match-timeout` (default 20 s). In order:

1. `pip3 show cyclonedds` on the host. Fix: `python3 -m pip install --user cyclonedds`.
2. Don't use `--peer` / unicast. This firmware only works with multicast.
3. Make sure ROS isn't sourced in that shell. ROS's `libddsc` can clash with the
   pip `cyclonedds` (`free(): invalid pointer`).
4. Give it a longer wait: `--match-timeout 40`.

### `joint N limit ±L°, got A°`

The target is outside the joint's limit (±135, ±90, ±90, ±135, ±90, ±135).
Use a smaller angle. `sweep --pct` keeps targets inside the limits.

### `wifi_nav.sh --arm` skipped the arm

Navigation continues without the arm. The banner shows `D1 arm: disabled (...)`.

- `<iface> not ready`: the arm interface is missing or has no IPv4 address.
  Plug in the cable and run `sudo nmcli connection up go2-arm`.
- `share a subnet`: the Wi-Fi and arm interfaces overlap, so arm and Go2 DDS
  would mix. Give the arm interface its `192.168.123.99` address.
- `arm start failed`: read `~/files/go2-nav2-wifi/ws/log/arm.log`. A
  `preflight failed` there means the address or ping check failed.

### Arm moves by itself between commands

Seen 2026-10-27: after a command ended, the arm swung or folded with no move
sent. Cause: enable-all (`funcode 5`, mode 80000) sent to an arm that is
**already enabled** drives it back to its last multi-joint (`funcode 2`)
target. An early version of the Wi-Fi bridge re-sent it on every disconnect.

- Update: the bridge now stiffens only a compliant (teach-mode) arm, and
  `fold` skips enable when the arm already is. Re-copy the bridge to the robot
  (`wifi_nav.sh --arm-wifi` does it) and restart it.
- Never put `enable` into a command chain while the arm is already enabled:
  check with `start_arm.sh state` (`enable=1`) first.

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

Full camera setup: [camera.md](ai_and_vision/camera.md). Relay code:
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
[setup.md](setup/setup.md#3d-digital-twin-and-ai--one-time-setup)), then restart
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

## Native Ubuntu workstation

The stack used to run in a VirtualBox VM; it now runs natively on Ubuntu 22.04.
Interface names are auto-detected ([setup](setup/setup.md#workstation)).

### `SENSOR CLOCK FAULT` right after the dog boots

`slam.log` shows the lidar relay running (`registered=7.4 Hz`) and then
`SENSOR CLOCK FAULT: sensor timestamp is in the future ... forwarding stopped`.
From then on no map or pose reaches the workstation: Nav2 logs
`/lidar3d/obstacle_points ... has not been updated`, and the 3D twin prints
`waiting up to 180 s for a frame on /camera/image_raw...` and stops, which
takes the dashboard on port 3000 with it.

The Jetson has no battery-backed clock. It boots with the last saved time and
jumps when it first reaches an NTP server (`journalctl -u systemd-timesyncd`
on the robot shows `Initial synchronization`). On the workstation's Wi-Fi it
has internet, so this happens on every boot. Wait ~2 minutes after powering on
the dog, then relaunch; check with `timedatectl` on the robot
(`System clock synchronized: yes`).

### `robot clock offset ... TOO LARGE`

`wifi_nav.sh` measures the robot clock over SSH. It now reuses one SSH
connection, so the reading is accurate to a few ms. If it still fails, the
clocks really differ: check `timedatectl` on both ends, or run
`./scripts/runtime/wifi_nav.sh --sync-clock` (asks for the robot's sudo
password).

### NVIDIA driver fails to build (`-ftrivial-auto-var-init=zero`)

`apt-get install` of anything fails with errors on `nvidia-dkms-open`,
`nvidia-driver-open`, `nvidia-open`, `cuda-drivers`, and
`/var/lib/dkms/nvidia/*/build/make.log` shows
`cc: error: unrecognized command-line option '-ftrivial-auto-var-init=zero'`.
The 6.8 HWE kernel is built with gcc-12; Ubuntu 22.04 has only gcc-11:

```bash
sudo apt-get install -y gcc-12
sudo dpkg --configure -a
sudo reboot          # then: nvidia-smi
```

### Docker: `permission denied ... docker.sock`

Your user is not in the `docker` group yet: `sudo usermod -aG docker "$USER"`,
then log out and back in.

---

## RViz

### RViz renders slowly with the point cloud visible

RViz renders inside the container (on the GPU once the NVIDIA driver is
loaded, otherwise on the CPU). To improve:

- Close the heavy point cloud display while tuning
- Untick displays you aren't using
- Check `glxinfo -B` shows the NVIDIA renderer, not `llvmpipe`

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

If [`config/tour.rviz`](../config/tour.rviz) doesn't exist, see [tour.md](navigation/tour.md) for how to save it.

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
[docs/simulation/gazebo.md → Troubleshooting](simulation/gazebo.md#troubleshooting).
`./gazebo/test_sim.sh` checks every simulated topic, motion and the arm.

---

## 3D Digital Twin

`wifi_3d_nav.sh` problems (image missing, dashboard not loading or not
connecting, empty
`/rtabmap/cloud_map`, clicks ignored, camera not detected, wrong map colours,
YOLO obstacles ignored by Nav2, Ollama unreachable, no battery data, `--resume`
maps not joined, CPU load, unsaved map, Foxglove
*"Error creating WebGL context"* without a GPU driver): see
[3d_nav.md → Troubleshooting](ai_and_vision/3d_nav.md#troubleshooting). Logs are in
`~/files/go2-nav2-wifi/ws/log/3d/` (`rtabmap.log`, `bridge.log`,
`click_to_goal.log`, `yolo.log`, `vlm.log`, `battery.log`, `camera_info.log`,
`lidar_depth.log`, `rosbridge.log`, `web.log`, `image_compressor.log`) and
`docker logs go2-3d`.

---

## Known blockers and fixes

### Robot IP not found / DHCP changed

`wifi_nav.sh` stops with `could not find the Go2 on <wifi iface>` when discovery fails.
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

1. **Run the diagnostic:** `./scripts/tools/diag_arm.sh` (interface auto-detected).
   It checks the interface, ping, container, `cyclonedds`, and DDS match.
2. **Check the address.** The arm port needs `192.168.123.99`:
   `sudo nmcli connection up go2-arm`, then
   `./scripts/runtime/start_arm.sh --check`.
3. **Check the arm is powered.** 24V DC via the barrel jack; allow ~1 min to boot.
4. **Check `cyclonedds`** on the host: `pip3 show cyclonedds`. Discovery is
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
   [Arm — calibration](setup/setup.md#arm--calibration-once).

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
| Arm unresponsive | `./scripts/tools/diag_arm.sh` |
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
