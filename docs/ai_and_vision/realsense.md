# RealSense D435i on the Go2

[← README](../../README.md) · [Docs index](../README.md) · [Camera](camera.md) · [3D twin](3d_nav.md) · [Troubleshooting](../troubleshooting.md#camera)

The D435i plugged into the **rear USB-C port of the Go2 expansion dock**
publishes colour and depth as soon as the dog is powered on, and the topics
appear in the repo's own **`go2-lidar3d`** container on the workstation. You
don't need SSH or a separate test container. Both ends find each other by
**MAC**, so DHCP address changes don't matter.

| Piece | Where | What |
|---|---|---|
| [`realsense/Dockerfile`](../../realsense/Dockerfile) | robot | `go2-realsense:foxy`: `ros:foxy-ros-base-focal` (arm64) + **librealsense v2.51.1** (RSUSB backend) + **realsense-ros 4.51.1** + `rmw_cyclonedds_cpp` + `image_transport_plugins` |
| [`realsense/entrypoint.sh`](../../realsense/entrypoint.sh) | container | writes the CycloneDDS config (one interface, workstation as unicast peer), starts `realsense2_camera_node` as `/camera/camera` |
| [`realsense/boot/`](../../realsense/boot) | robot | `go2-realsense.service` (systemd) + `go2_realsense_boot.sh`: at power-on, start the container **only if the D435i is on USB**, find the workstation by MAC, restart on a new workstation IP |
| [`scripts/tools/realsense_host.sh`](../../scripts/tools/realsense_host.sh) | workstation | `macs`, `robot-build`, `setup-robot`, `status`, `check`, `view`, `rviz`, `shell`, `push-peer`, `plugins`, `robot-logs` / `-restart` / `-stop` |
| [`scripts/runtime/wifi_nav.sh`](../../scripts/runtime/wifi_nav.sh) | workstation | at every session start: pushes this machine's IP to the robot, installs the image transport plugins in `go2-lidar3d` (`REALSENSE=off` skips both) |
| [`realsense/go2_realsense.sh`](../../realsense/go2_realsense.sh) | robot | manual start/stop for debugging (stop the service first: only one process may own the camera) |

## Quick start

```bash
cd ~/files/Unitree_GO2_Agricultural
./scripts/tools/realsense_host.sh macs                  # once: learn the robot's MAC, show this machine's
./scripts/tools/realsense_host.sh robot-build           # once per image change (~25–40 min on the Jetson)
./scripts/tools/realsense_host.sh setup-robot --hotplug # once: boot service with this machine's MAC (robot sudo password)
# from now on: power on the dog, then
./scripts/tools/realsense_host.sh check                 # /camera topics + rates inside go2-lidar3d
./scripts/tools/realsense_host.sh rviz                  # or: view (rqt_image_view)
```

Inside a normal session (`wifi_nav.sh`, `wifi_3d_nav.sh`) the topics are in
`go2-lidar3d` with nothing extra to run. Open a ROS shell there with
`./scripts/tools/ros_shell.sh` or `realsense_host.sh shell`.

## 1. Architecture

```text
 Go2 Jetson (Foxy host, untouched)                        Ubuntu 22.04 workstation
 ┌──────────────────────────────────────────────────┐     ┌───────────────────────────────────────────┐
 │ systemd: go2-realsense.service (at power-on)      │     │ go2-lidar3d (Humble, the repo's container) │
 │   D435i on USB?  no -> exit, nothing started      │     │   /tmp/go2-wifi-cyclonedds.xml:            │
 │   workstation by MAC (neighbour table + sweep)    │     │   bound to this IP, robot as peer, dom. 64 │
 │   + IP pushed by wifi_nav.sh (/tmp/go2-realsense- │ DDS │   (setup-robot-wifi.sh, upstream)          │
 │     peer); re-checked every 20 s                  │ UDP │                                            │
 │ ┌─ container go2-realsense:foxy ──────────────┐   │────►│   RViz / rqt / RTAB-Map / YOLO nodes       │
 │ │ librealsense 2.51.1 (libusb, user space)     │   │dom64│   + image_transport_plugins (installed     │
 │ │ realsense2_camera_node /camera/camera        │   │     │     by wifi_nav.sh / realsense_host.sh)    │
 │ │ Cyclone 0.7, bound to wlan0, peer = host IP  │   │     │ go2_net.sh: robot by MAC (GO2_ROBOT_MAC),  │
 │ └──────────────────────────────────────────────┘   │     │   then name, cache, prefix scan, SSH key   │
 │ Unitree stack: domain 0 on eth0 — not joined       │     └───────────────────────────────────────────┘
 └──────────────────────────────────────────────────┘
```

Decisions and why:

- **The driver runs on the robot; everything else runs on the workstation.**
  USB can't cross the network, but DDS can. The Jetson only grabs frames,
  aligns depth to colour and compresses them. Point clouds, RTAB-Map, YOLO
  and viewing run in `go2-lidar3d`.
- **A boot service, not an SSH step.** systemd starts it at every boot. It
  starts the container only when the D435i (USB `8086:0b3a`) shows up within
  60 s. Otherwise it exits cleanly and nothing retries. With `--hotplug`, a
  udev rule starts it when the camera is plugged in later. A crashed
  container is restarted, at most 5 times in 10 minutes.
- **Discovery by MAC on both sides.** CycloneDDS needs the other side's IP
  as a unicast peer (Wi-Fi drops multicast), and both IPs come from DHCP.
  - The robot looks up the workstation's MAC (`RS_HOST_MAC`, written by
    `setup-robot`) in its neighbour table. If it is missing, it ping-sweeps
    its /24 (about 2 s). It re-checks every 20 s and restarts the container
    when the IP changes.
  - The workstation finds the robot by `GO2_ROBOT_MAC` (saved by `macs` in
    `~/.config/go2_wifi_nav/network.env`) before trying the hostname, the
    cache and the `02:00:00` prefix scan.
  - `wifi_nav.sh` also pushes the current IP to the robot at every session
    start, as a fallback for a MAC the robot can't see.
- **`go2-lidar3d` needs no new DDS setup.** Upstream's `setup-robot-wifi.sh`
  already binds Cyclone to this machine's IP with the robot as peer on
  domain 64, the same domain the camera uses. What it lacked were the
  compressed image transport plugins (RViz:
  `Unable to load plugin for transport 'image_transport/compressed_sub'`).
  They are installed into the container with apt. A recreated container (new
  IP) loses them, and the next session reinstalls them.
- **librealsense 2.51.1 + realsense-ros 4.51.1.** 4.51.1 is the wrapper
  matched to 2.51.1, from the line that still builds on Foxy. Later releases
  removed Dashing/Eloquent/Foxy support, and `main` / `ros2-development`
  don't build on Foxy. Its default topic names (`/camera/color/...`, no
  doubled `/camera/camera/...`) are the ones `wifi_3d_nav.sh --rgbd` expects.
- **RSUSB backend** (`FORCE_RSUSB_BACKEND=ON`). The camera is driven through
  libusb from user space, so the Jetson kernel needs no RealSense patches and
  the host needs no udev rules for the camera itself.
- **The container brings its own Foxy and Cyclone.** On this robot, stock
  Foxy `rmw_cyclonedds_cpp` crashes next to Unitree's Cyclone 0.10. The
  image ships its own copies, and the entrypoint unsets `LD_LIBRARY_PATH`.
  Domain 0 (Unitree's onboard graph) is refused.

### Profile and bandwidth

On today's **USB 2.1** link (`Device USB type: 2.1`), use colour + depth at
`640,480,15`, with **both infra streams off**. That is about 18 MB/s on USB.
Four streams at 15 fps is about 28 MB/s and produces watchdog errors and
`Out of frame resources!`. With a USB 3 cable (`USB type: 3.2`), use
`640,480,30`.

Over Wi-Fi, subscribe to `/compressed` (JPEG) and `/compressedDepth` (PNG),
best-effort for live use. Raw colour + aligned depth at 640×480@15 is about
23 MB/s, too much for the Go2's dongle. DDS only sends a topic to
subscribers, so raw topics cost nothing on Wi-Fi until something subscribes
remotely. If rates bunch up (`min 0.002 s`, `max 0.3 s`), Wi-Fi power saving
is usually the cause: `./scripts/tools/fix_wifi_powersave.sh --check`.

## 2. Robot side: the Docker image

Check once on the robot (SSH):

```bash
lsusb -d 8086:0b3a                 # Intel RealSense D435i  (must be listed)
lsusb -t | grep -i video           # 5000M = USB 3 (good); 480M = USB 2 (see failure modes)
docker info >/dev/null && echo docker-ok   # JetPack ships Docker; else: DOCKER="sudo docker"
df -h /var/lib/docker              # the image is ~2–3 GB; build needs ~5 GB free
free -h                            # build with RS_BUILD_JOBS=2 if < 4 GB is free
```

Copy and build from the workstation (the robot needs internet for the build):

```bash
cd ~/files/Unitree_GO2_Agricultural
./scripts/tools/realsense_host.sh robot-build   # copies realsense/ to ~/files/rs-test, ~25–40 min the first time
```

Or on the robot: `cd ~/files/rs-test && docker build --network host -t go2-realsense:foxy .`.
`--network host` is required: Docker's build network has no DNS on the Go2
(`Temporary failure resolving 'ports.ubuntu.com'`).

The Dockerfile does the following:

1. **Replaces the expired ROS apt key.** Old `ros:foxy` images carry the key
   that expired on 2025-06-01 (`EXPKEYSIG F42ED6FBAB17C654`). The Dockerfile
   re-adds `packages.ros.org` with the renewed key. Foxy's final debs are
   still served there.
2. **Installs the ROS dependencies by name** (`cv_bridge`, `image_transport`,
   `image_transport_plugins`, `diagnostic_updater`, `tf2_ros`,
   `rmw_cyclonedds_cpp`) rather than through rosdep. rosdep would pull
   `ros-foxy-librealsense2`, a second librealsense; the build fails if that
   package appears.
3. **Builds librealsense v2.51.1** with `FORCE_RSUSB_BACKEND=ON`, no CUDA, no
   GUI, tools on (`rs-enumerate-devices`), and no firmware download at cmake
   time.
4. **Builds realsense-ros tag `4.51.1`** (`--packages-up-to
   realsense2_camera`). It then checks that the wrapper links
   `/usr/local/lib/librealsense2.so`.

USB passthrough (in `realsense/boot/go2_realsense_boot.sh`):

```bash
docker run -d --name go2-realsense --network host --ipc host \
  --privileged -v /dev/bus/usb:/dev/bus/usb -v /run/udev:/run/udev:ro \
  -e ROS_DOMAIN_ID=64 -e RS_DDS_IFACE=wlan0 -e RS_DDS_PEERS=<workstation IP, found by MAC> \
  go2-realsense:foxy camera
```

- `/dev/bus/usb` is bind-mounted as a whole, not passed with `--device`. The
  D435i re-enumerates with a new device number after `initial_reset` or a
  hardware reset, and a single `--device` node would go stale.
  `--privileged` alone snapshots `/dev` at start and misses re-enumeration
  for the same reason.
- `--network host` is required: DDS has to bind the robot's real `wlan0` /
  `eth0`, not a Docker bridge.
- If you'd rather not use `--privileged`, `RS_PRIVILEGED=0` uses
  `--device-cgroup-rule='c 189:* rmw'` (USB device nodes, which covers the
  IMU over libusb as well).

## 3. Workstation side: MACs, boot service, go2-lidar3d

**MACs, once.** `./scripts/tools/realsense_host.sh macs` finds the robot the
usual way and saves its Wi-Fi MAC as `GO2_ROBOT_MAC` in
`~/.config/go2_wifi_nav/network.env`. Every repo script that sources
`go2_net.sh` then finds the robot by it first. It also prints this machine's
MAC.

- **Robot MAC changes on reboot.** Run `macs` again after a robot reboot. If
  the MAC changed, delete that line; the prefix scan and the SSH-key probe
  still find the robot.
- **Workstation MAC is randomised.** A locally administered MAC (second hex
  digit 2, 6, A or E) is often set by NetworkManager. If it changes, the
  robot loses this machine. The `macs` output shows the `nmcli ...
  cloned-mac-address permanent` command that pins the real one.

**Boot service, once.** `./scripts/tools/realsense_host.sh setup-robot
--hotplug` does three things:

1. Copies `realsense/` to `~/files/rs-test` on the robot.
2. Runs `sudo ./install_boot_service.sh --host-mac <this machine's MAC>
   --hotplug` there. The robot's sudo password is asked.
3. Pushes this machine's IP.

It writes only `/opt/go2-realsense/`, `/etc/systemd/system/go2-realsense.service`,
`/etc/go2-realsense.env` and, with `--hotplug`, one udev rule. ROS, Unitree
and network config are untouched. Run it again from a second workstation:
its MAC is added, never replaced. Settings (profiles, peers, waits) live in
`/etc/go2-realsense.env` on the robot. After an edit, run
`./scripts/tools/realsense_host.sh robot-restart`.

**go2-lidar3d.** Nothing to configure. `realsense_host.sh check | view |
rviz | shell` run inside it, with `setup-robot-wifi.sh` rebuilding its DDS
config for the current IPs. If the container doesn't exist yet, they create
it (`setup-lidar3d.sh --laptop`). If its image transport plugins are
missing, `view`, `rviz` and `plugins` install them, fixing the expired ROS
apt key on the way if needed. Inside a session, `wifi_nav.sh` does both at
start-up:

```text
[realsense] robot camera service active; told it this machine is 192.168.1.5
[realsense] go2-lidar3d image transport plugins + rqt ...... OK
```

**Wired instead of Wi-Fi:** put `RS_DDS_IFACE=eth0` in the robot's env file
and use a workstation NIC on 192.168.123.x. Never use the arm's NIC; see
[architecture.md](../architecture.md). Domain 64 uses UDP ports 23400 and up;
Unitree's domain 0 uses 7400 and up, so they don't collide.

## 4. Launch and verify

```bash
# Robot: power on (camera in the rear USB-C port). About a minute later:
./scripts/tools/realsense_host.sh status   # service enabled/active, peers it finds by MAC, container, last log lines
./scripts/tools/realsense_host.sh check    # inside go2-lidar3d: /camera topics, rates, camera_info
./scripts/tools/realsense_host.sh rviz     # Image: /camera/color/image_raw, Transport Hint compressed, Reliability Best Effort
                                           #        /camera/aligned_depth_to_color/image_raw, hint compressedDepth
```

Robot-side log, `./scripts/tools/realsense_host.sh robot-logs` (or
`journalctl -u go2-realsense -f` on the robot):

```text
[go2-realsense] D435i found at USB 1-3.3, 480 Mb/s (USB 2: keep 15 fps or lower).
[go2-realsense] wlan0 is 192.168.1.7/24; DDS domain 64.
[go2-realsense] workstation peer(s): 192.168.1.5 (MAC 1c:bf:ce:22:ca:47)
[go2-realsense] starting go2-realsense (go2-realsense:foxy), peers: 192.168.1.5
[INFO] [camera.camera]: Open profile: stream_type: Depth(0), Format: Z16, Width: 640, Height: 480, FPS: 15
[INFO] [camera.camera]: Open profile: stream_type: Color(0), Format: RGB8, Width: 640, Height: 480, FPS: 15
[INFO] [camera.camera]: RealSense Node Is Up!
```

**Pass criteria:**

- Exactly two `Open profile` lines, Depth and Color.
- In `check`, compressed colour and depth hold about the profile fps.
- `camera_info` reports 640×480 with `frame_id: camera_color_optical_frame`.

**Without the camera plugged in**, `status` shows `no RealSense D435i
(8086:0b3a) on USB after 60 s: not starting the camera node` and no
container. After a DHCP change, the log shows `workstation peer(s) changed:
'…' -> '…': restarting go2-realsense` within about 20 s.

**TF:** the driver publishes `camera_link` → optical frames on `/tf_static`.
Place the camera on the dog with a static transform measured from its mount.
For example, in `realsense_host.sh shell`:
`ros2 run tf2_ros static_transform_publisher --x 0.30 --y 0 --z 0.10 --yaw 0 --pitch 0 --roll 0 --frame-id base_link --child-frame-id camera_link`.

**With the 3D twin:** `TWIN_DEPTH_TOPIC=/camera/aligned_depth_to_color/image_raw
TWIN_CAMINFO_TOPIC=/camera/color/camera_info ./scripts/runtime/wifi_3d_nav.sh --rgbd`.
RTAB-Map subscribes to raw topics, so keep a Wi-Fi-safe profile.

**Manual run for debugging** (on the robot, in `~/files/rs-test`): first
`sudo systemctl stop go2-realsense`, then `./go2_realsense.sh check` (SDK
only) or `GO2_HOST_IP=<ip> ./go2_realsense.sh start`.

## 5. Failure modes

### `terminate called after throwing an instance of 'std::bad_alloc'`

Work through these in order:

1. **SDK or wrapper?** `sudo systemctl stop go2-realsense && ./go2_realsense.sh check` (on the robot, in `~/files/rs-test`).
   If `rs-enumerate-devices` also fails, the problem is USB or librealsense
   (see below), not ROS.
2. **Two librealsense copies** (the usual cause). The wrapper was built
   against one library and loads another at runtime, so a struct size
   mismatch turns into a huge allocation. In the container:
   `ldconfig -p | grep realsense2` must show only `/usr/local/lib`, and
   `ldd /opt/rs_ws/install/lib/librealsense2_camera.so | grep realsense`
   must point there. The image build already checks both. Don't
   `apt install ros-foxy-librealsense2`, don't `pip install pyrealsense2`
   into this image, and don't mount host libraries.
3. **Host libraries leaking in.** `env | grep LD_LIBRARY_PATH` inside the
   container should print nothing. Sourcing the robot's Unitree setup (its
   Cyclone 0.10) over Foxy's Cyclone 0.7 gives `bad_alloc` or a segfault in
   `rmw_create_node`. `ldd $(ros2 pkg prefix rmw_cyclonedds_cpp)/lib/librmw_cyclonedds_cpp.so | grep ddsc`
   must show `/opt/ros/foxy/lib`.
4. **Wrong parameter format.** 4.51.1 profiles are `"640,480,15"` (commas).
   The `640x480x15` form belongs to later wrapper versions.
5. **Real out-of-memory.** Check `dmesg | grep -i oom` and `free -h` on the
   robot. Lower the profile, keep `RS_POINTCLOUD=false` (make clouds on the
   workstation with `depth_image_proc`), and leave the IMU off unless you
   need it. During the image build the symptom is
   `c++: fatal error: Killed signal`; fix it with `RS_BUILD_JOBS=2`.
6. **Subscriber-side `bad_alloc`** (workstation tool crashes, driver keeps
   running). This happens when a custom type is deserialized with a
   different definition, for example `realsense2_camera_msgs/Extrinsics` or
   `Metadata` from a newer Humble package. Subscribe only to `sensor_msgs`
   topics, avoid `ros2 bag record -a`, or build `realsense2_camera_msgs`
   from tag 4.51.1 on the workstation.
7. **Get a backtrace:** `docker exec -it go2-realsense bash`, then
   `ros2 run --prefix 'gdb -batch -ex run -ex bt --args' realsense2_camera realsense2_camera_node --ros-args -r __ns:=/camera -r __node:=camera --params-file /tmp/rs_params.yaml`.
   A trace inside `rs2::` means librealsense; one inside `rmw_` / `dds_` means DDS.

### USB passthrough

| Symptom | Cause → fix |
|---|---|
| `lsusb` on the robot doesn't list `8086:0b3a` | Wrong port: only the rear USB-C on the expansion dock works. Reseat it, flip the USB-C plug, use a USB 3.x-rated cable |
| `[rs] no Intel (8086:) device visible` / `No RealSense devices were found` but the host `lsusb` sees it | `/dev/bus/usb` not mounted, or the container was started before the camera re-enumerated. `sudo systemctl restart go2-realsense` (it bind-mounts the whole tree) |
| `failed to set power state`, `Device or resource busy`, `xioctl(VIDIOC_*)` | Another process holds the camera: an old container (`docker ps -a`), or something on the host using `/dev/video*` (`sudo fuser -v /dev/video*`). Only one process may own the D435i |
| `lsusb -t` shows `480M`, or `Usb Type Descriptor 2.1`; `Couldn't resolve requests` for your profile | USB 2 link: cable, plug orientation or a hub. On USB 2, use `424,240,15` or `640,480,6` until it's fixed |
| `Frames didn't arrive within 5 seconds`, stalls after minutes | Dock power or the link: `RS_INITIAL_RESET=true` (default), lower fps, check `dmesg -w` for USB disconnects while it runs |
| Firmware warning at start | librealsense 2.51.1 recommends D4xx FW 5.13.0.50. Usually harmless; update with `rs-fw-update` if streams misbehave |

### DDS discovery across machines

Topics missing in `go2-lidar3d`, but `docker exec go2-realsense ...` shows them
on the robot:

| Check | How |
|---|---|
| Same domain + RMW on both ends | in `realsense_host.sh shell`: `echo $ROS_DOMAIN_ID $RMW_IMPLEMENTATION`; `docker exec go2-realsense env \| grep -E 'DOMAIN\|RMW'` |
| Robot bound to the right interface, peer = the workstation's current IP | `./scripts/tools/realsense_host.sh status` (peers it finds by MAC); `push-peer` if the MAC lookup misses |
| Workstation bound to the NIC that routes to the robot | `cat $HOME/.ros/go2-realsense-cyclonedds.xml`; `ip route get <robot IP>` |
| Which interface Cyclone actually picked | robot: add `RS_DDS_TRACE=1` to `/etc/go2-realsense.env`, `sudo systemctl restart go2-realsense`, then `docker exec go2-realsense grep -i interface /tmp/cdds.log` |
| Robot container fails at once with `rmw_create_node` / config errors | Cyclone 0.7 rejects unknown elements: only 0.7 syntax in the robot XML (`NetworkInterfaceAddress`, not `<Interfaces>`) |
| Firewall | `sudo ufw status`; allow UDP from the robot: `sudo ufw allow from <robot IP> proto udp` |
| Stale CLI view | `ros2 daemon stop`, or `ros2 topic list --no-daemon` |
| A container on the workstation can't see it | it needs `--network host` and the same three variables |
| Multicast-only setups fail on Wi-Fi | keep the unicast `<Peers>` on both ends (`AllowMulticast spdp`) |

Topics **listed but no data** (`ros2 topic hz` silent):

- **QoS mismatch.** `ros2 topic info -v /camera/color/image_raw` shows the
  publisher's reliability. The driver publishes `SYSTEM_DEFAULT`
  (reliable). A reliable subscriber can't match a best-effort publisher, so
  if you set `RS_QOS=SENSOR_DATA`, subscribe best-effort too (RViz:
  Reliability Policy).
- **Fragment loss.** A raw 640×480 frame is ~600–900 KB, hundreds of UDP
  fragments; one lost fragment loses the frame. Raise `rmem_max` (above),
  lower fps or resolution, or switch to the compressed topics.
- **The link is saturated.** `ros2 topic bw` on the robot vs the
  workstation. Over Wi-Fi, use the `/compressed` + `/compressedDepth`
  topics.
- **The domain-64 graph is crowded.** Cyclone only probes peer ports up to
  `MaxAutoParticipantIndex`. Both ends set 120. A config without it (default
  9) stops discovering once the workstation runs more than ~9 ROS
  processes.

### Boot service and MAC discovery

| Symptom | Cause → fix |
|---|---|
| `status`: service `inactive`, log `no RealSense D435i ... not starting` | Camera not on USB at boot. Plug it in. With `--hotplug` the service starts on its own; otherwise `realsense_host.sh robot-restart` |
| `status`: `peers it finds:` empty | The robot can't see the workstation's MAC. Check `RS_HOST_MAC` in `/etc/go2-realsense.env` against `cat /sys/class/net/<wifi>/address` on the workstation (a randomised MAC changes: pin it, see `macs`). `realsense_host.sh push-peer` works meanwhile |
| Topics stop after the workstation got a new IP | Normal for up to ~20 s (re-check interval) until `workstation peer(s) changed ... restarting`. Longer: `push-peer` |
| Four `Open profile` lines, `uvc streamer watchdog`, `Depth stream start failure`, `Out of frame resources!` | Both infra streams on (the node's own default when not set) overload USB 2. The image's `entrypoint.sh` sets `enable_infra1/2: false`; an image built from an older `entrypoint.sh` → `robot-build` |
| `control_transfer returned error ... Resource temporarily unavailable` | A few at start-up or every few minutes: harmless. Several per second: USB 2 overloaded, same fix as above, or a USB 3 cable |
| `re-enable the stream for the change to take effect` | Harmless: settings applied while the node starts |
| `realsense_host.sh` finds the wrong device as the robot | Another `02:00:00` device on the Wi-Fi: `macs` saves the exact `GO2_ROBOT_MAC` |
| A manual `go2_realsense.sh start` fails with the camera busy | The service already owns it: `sudo systemctl stop go2-realsense` first |
