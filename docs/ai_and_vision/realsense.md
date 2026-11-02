# RealSense D435i on the Go2 (Docker on the Jetson, topics on the workstation)

[← Back to README](../../README.md) · [Camera](camera.md) · [3D twin](3d_nav.md) · [Troubleshooting](../troubleshooting.md#camera)

The D435i is plugged into the **rear USB-C port of the Go2 expansion dock**
(the only port it enumerates on). Its driver runs on the Jetson **inside a
Docker container**, so the robot's ROS 2 Foxy, the Unitree packages and the
factory DDS config are never changed. The topics cross Wi-Fi (or Ethernet) on
this repo's DDS domain **64** with CycloneDDS, the same way the Go2 front
camera does ([camera.md](camera.md)).

| Piece | Where | What |
|---|---|---|
| [`realsense/Dockerfile`](../../realsense/Dockerfile) | robot | `ros:foxy-ros-base-focal` (arm64) + **librealsense v2.51.1** (RSUSB backend) + **realsense-ros 4.51.1** + `rmw_cyclonedds_cpp` + `image_transport_plugins` |
| [`realsense/entrypoint.sh`](../../realsense/entrypoint.sh) | container | writes the CycloneDDS config (one interface, workstation as unicast peer), starts `realsense2_camera_node` as `/camera/camera` |
| [`realsense/go2_realsense.sh`](../../realsense/go2_realsense.sh) | robot | `build`, `start`, `stop`, `logs`, `status`, `check`, `topics`, `hz`, `shell` |
| [`scripts/tools/realsense_host.sh`](../../scripts/tools/realsense_host.sh) | workstation | `deploy`, `robot-build`, `robot-start`, `env`, `check`, `republish`, `view`, `robot-stop` |

## 1. Architecture

```text
 Go2 Jetson (Foxy host, untouched)                         Ubuntu 22.04 workstation (Humble)
 ┌────────────────────────────────────────────────┐        ┌─────────────────────────────────────────┐
 │ D435i ─USB-C (rear dock)─► /dev/bus/usb         │        │ go2-lidar3d / go2-3d containers         │
 │                              │ (bind mount)     │        │   (wifi_nav.sh session: already Cyclone,│
 │ ┌─ container go2-realsense:foxy ─────────────┐  │        │    domain 64, robot as peer)            │
 │ │ librealsense 2.51.1 (libusb, user space)   │  │  DDS   │      └─ wifi_3d_nav.sh --rgbd (RTAB-Map)│
 │ │ realsense2_camera_node 4.51.1 /camera/camera│ ─┼─ UDP ─►│ or a native shell:                      │
 │ │ rmw_cyclonedds_cpp (Cyclone 0.7), dom. 64  │  │ dom 64 │   eval "$(realsense_host.sh env)"       │
 │ │ bound to wlan0 (or eth0), peer = host IP   │  │ unicast│   RViz / rqt / your nodes (Cyclone 0.10)│
 │ └─────────────────────────────────────────────┘  │        │   realsense_host.sh republish (raw)    │
 │ Unitree stack: domain 0 on eth0 — not joined     │        └─────────────────────────────────────────┘
 └────────────────────────────────────────────────┘
```

Decisions and why:

- **Driver on the robot, everything else on the workstation.** USB can't
  cross the network; DDS can. The Jetson only grabs frames, aligns depth to
  colour, and compresses. Point clouds, RTAB-Map, YOLO and viewing run on the
  workstation.
- **librealsense 2.51.1 + realsense-ros 4.51.1.** 4.51.1 is the wrapper
  matched to 2.51.1, from the line that still builds on Foxy. Later releases
  removed Dashing/Eloquent/Foxy support, and `main` / `ros2-development`
  don't build on Foxy. Its default topic names (`/camera/color/...`, no
  doubled `/camera/camera/...`) are the ones `wifi_3d_nav.sh --rgbd` already
  expects. librealsense's recommended D4xx firmware for 2.51.1 is
  **5.13.0.50**. A different firmware usually works and only logs a warning.
- **RSUSB backend** (`FORCE_RSUSB_BACKEND=ON`). librealsense drives the
  camera through libusb from user space. The Jetson kernel needs no RealSense
  patches and the host needs no udev rules. All it takes is `/dev/bus/usb`
  in the container.
- **CycloneDDS on both ends, domain 64, unicast peers, bound to one
  interface.** This copies `scripts/robot/go2_camera_wifi.sh`: Wi-Fi often
  drops multicast, and the Jetson has several interfaces (`eth0`
  192.168.123.x for Unitree, `wlan0` for the lab, `docker0`). Foxy (Cyclone
  0.7) and Humble (Cyclone 0.10) speak the same DDSI-RTPS wire protocol, and
  `sensor_msgs/Image` / `CameraInfo` have the same layout in both, so topics
  work across the two distros. Domain 0 is refused because that is Unitree's
  onboard graph.
- **The container's own Foxy and Cyclone.** On this robot, stock Foxy
  `rmw_cyclonedds_cpp` crashes next to Unitree's Cyclone 0.10
  ([arm_bridge_wifi.sh](../../scripts/robot/arm_bridge_wifi.sh)). The image
  ships its own copies, and the entrypoint unsets `LD_LIBRARY_PATH`, so
  nothing from the host leaks in.

### Bandwidth: pick the profile for the link

Rough numbers per stream at 640×480: colour `rgb8` 0.92 MB/frame, depth
`16UC1` 0.61 MB/frame. JPEG colour is about 40–60 KB/frame and PNG
`compressedDepth` about 150–300 KB/frame (scene-dependent). Measure yours
with `ros2 topic bw`.

| Link | Suggested | Subscribe to |
|---|---|---|
| Go2 Wi-Fi dongle (this repo's default) | `RS_COLOR_PROFILE=640,480,6 RS_DEPTH_PROFILE=640,480,6` (or `424,240,15`) | `/compressed` + `/compressedDepth`, then `realsense_host.sh republish` |
| Ethernet (workstation NIC on 192.168.123.x) | `640,480,30` | raw topics directly |

Raw colour plus aligned depth at 640×480@15 is about 23 MB/s (≈185 Mbit/s).
That is fine on gigabit Ethernet and too much for the Go2's Wi-Fi. DDS only
sends a topic to subscribers, so the raw topics cost nothing on the network
until something subscribes to them remotely.

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
./scripts/tools/realsense_host.sh deploy        # -> unitree@<robot>:~/go2-realsense/
./scripts/tools/realsense_host.sh robot-build   # ~25–40 min the first time on the Orin NX
```

Or on the robot: `cd ~/go2-realsense && RS_BUILD_JOBS=4 ./go2_realsense.sh build`.

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

USB passthrough (in `go2_realsense.sh start`):

```bash
docker run -d --name go2-realsense --network host --ipc host \
  --privileged -v /dev/bus/usb:/dev/bus/usb -v /run/udev:/run/udev:ro \
  -e ROS_DOMAIN_ID=64 -e RS_DDS_IFACE=wlan0 -e RS_DDS_PEERS=<workstation IP> \
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

## 3. Workstation side: DDS, domain, interface

**Inside a `wifi_nav.sh` / `wifi_3d_nav.sh` session:** nothing to set. The
`go2-lidar3d` container already runs Cyclone on domain 64 with the robot as
its peer, so the `/camera/...` topics appear there:

```bash
docker exec go2-lidar3d bash -lc 'source /opt/ros/humble/setup.bash; \
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///tmp/go2-wifi-cyclonedds.xml \
  ros2 topic list | grep ^/camera'
```

**A native shell (Humble), outside the session:**

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp ros-humble-image-transport-plugins \
                 ros-humble-rqt-image-view ros-humble-depth-image-proc
# UDP receive buffers, once (already in docs/setup/setup.md):
printf 'net.core.rmem_max=16777216\nnet.core.rmem_default=8388608\n' | sudo tee /etc/sysctl.d/60-go2-dds.conf
sudo sysctl --system

source /opt/ros/humble/setup.bash
eval "$(./scripts/tools/realsense_host.sh env)"   # finds the robot (go2_net.sh), writes ~/.ros/go2-realsense-cyclonedds.xml
```

The resulting settings (for your own `.bashrc` or a launch file):

```bash
export ROS_DOMAIN_ID=64                          # must equal the robot's
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp     # Cyclone on both ends
export CYCLONEDDS_URI=file://$HOME/.ros/go2-realsense-cyclonedds.xml
unset ROS_LOCALHOST_ONLY
```

```xml
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces><NetworkInterface name="wlx1cbfce22ca47" /></Interfaces>  <!-- the NIC that routes to the robot -->
      <AllowMulticast>spdp</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
      <Peers><Peer address="192.168.1.9"/></Peers>   <!-- robot Wi-Fi IP -->
    </Discovery>
    <Internal><SocketReceiveBufferSize min="8MB"/></Internal>
  </Domain>
</CycloneDDS>
```

The robot side writes the same thing in Cyclone 0.7 syntax
(`<NetworkInterfaceAddress>wlan0</NetworkInterfaceAddress>`), with the
workstation as the peer. Each end lists the other as a peer.

**Wired instead of Wi-Fi:** put the workstation NIC on 192.168.123.x (not
the arm's NIC; see [architecture.md](../architecture.md)) and pin both ends:

```bash
RS_DDS_IFACE=eth0 ./scripts/tools/realsense_host.sh robot-start        # robot binds eth0, still domain 64
GO2_ROBOT_IP=192.168.123.18 RS_HOST_IFACE=enp0s31f6 ./scripts/tools/realsense_host.sh check
```

Domain 64 uses UDP ports 23400+ and Unitree's domain 0 uses 7400+, so they
don't collide on `eth0`.

**Other distros on the workstation:** keep Cyclone. Foxy ↔ Humble is the
tested pair. Jazzy also interoperates for `sensor_msgs` topics, but
`ros2 param` / services across distros are not guaranteed. Don't mix in
Fast DDS on one side.

## 4. Launch and verify

```bash
# ── Workstation ───────────────────────────────────────────────────────────
cd ~/files/Unitree_GO2_Agricultural
./scripts/tools/realsense_host.sh deploy                     # after any change to realsense/
RS_COLOR_PROFILE=640,480,6 RS_DEPTH_PROFILE=640,480,6 \
  ./scripts/tools/realsense_host.sh robot-start               # SSH: starts the container, peer = this machine

# ── Robot (SSH), step by step ──────────────────────────────────────────────
cd ~/go2-realsense
./go2_realsense.sh check          # (container stopped) SDK only: rs-enumerate-devices -s lists the D435i,
                                  #   serial, firmware, "Usb Type Descriptor 3.2"
GO2_HOST_IP=<workstation IP> ./go2_realsense.sh start
./go2_realsense.sh logs           # want: "RealSense Node Is Up!"; no "bad_alloc", no "No RealSense devices were found"
./go2_realsense.sh topics         # /camera/color/image_raw, /camera/depth/image_rect_raw,
                                  # /camera/aligned_depth_to_color/image_raw, */camera_info, */compressed, /tf_static
./go2_realsense.sh hz             # rates on the robot itself (≈ the profile fps)

# ── Workstation ───────────────────────────────────────────────────────────
./scripts/tools/realsense_host.sh check       # discovery, rates of compressed + raw, QoS, camera_info
./scripts/tools/realsense_host.sh republish   # -> /rs_local/color/image_raw, /rs_local/depth/image_raw (raw, local)
./scripts/tools/realsense_host.sh view        # live colour
```

By hand in a configured shell (`eval "$(realsense_host.sh env)"`):

```bash
ros2 daemon stop                                   # forget a graph cached with other settings
ros2 topic list | grep ^/camera
ros2 topic hz /camera/color/image_raw/compressed
ros2 topic bw /camera/aligned_depth_to_color/image_raw/compressedDepth
ros2 topic echo --once /camera/color/camera_info | head -20
```

**Pass criteria:** the same `/camera/*` names on both machines. Both
`color` and aligned `depth` arrive at the robot-side fps (± a frame).
`camera_info` is 640×480 with `frame_id: camera_color_optical_frame`.

**RViz:** fixed frame `camera_link` (the driver publishes `camera_link` →
optical frames on `/tf_static`). Add Image `/rs_local/color/image_raw` and
DepthCloud with depth `/rs_local/depth/image_raw` and colour
`/rs_local/color/image_raw`. To place the camera on the dog, publish where
it is actually mounted (measure it):

```bash
ros2 run tf2_ros static_transform_publisher --x 0.30 --y 0 --z 0.10 --yaw 0 --pitch 0 --roll 0 \
  --frame-id base_link --child-frame-id camera_link
```

**With the 3D twin** (`--rgbd`), use the aligned depth and a Wi-Fi-safe
profile. RTAB-Map subscribes to the raw topics:

```bash
TWIN_DEPTH_TOPIC=/camera/aligned_depth_to_color/image_raw TWIN_CAMINFO_TOPIC=/camera/color/camera_info \
  ./scripts/runtime/wifi_3d_nav.sh --rgbd
```

Stamps come from the Jetson clock, as with the LiDAR. If TF lookups fail
with "extrapolation into the future/past", compare
`ssh unitree@<robot> date +%s.%N` with `date +%s.%N` on the workstation.

## 5. Failure modes

### `terminate called after throwing an instance of 'std::bad_alloc'`

Work through these in order:

1. **SDK or wrapper?** `./go2_realsense.sh stop && ./go2_realsense.sh check`.
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
7. **Get a backtrace:** `./go2_realsense.sh shell`, then
   `ros2 run --prefix 'gdb -batch -ex run -ex bt --args' realsense2_camera realsense2_camera_node --ros-args -r __ns:=/camera -r __node:=camera --params-file /tmp/rs_params.yaml`.
   A trace inside `rs2::` means librealsense; one inside `rmw_` / `dds_` means DDS.

### USB passthrough

| Symptom | Cause → fix |
|---|---|
| `lsusb` on the robot doesn't list `8086:0b3a` | Wrong port: only the rear USB-C on the expansion dock works. Reseat it, flip the USB-C plug, use a USB 3.x-rated cable |
| `[rs] no Intel (8086:) device visible` / `No RealSense devices were found` but the host `lsusb` sees it | `/dev/bus/usb` not mounted, or the container was started before the camera re-enumerated. `./go2_realsense.sh restart` (it bind-mounts the whole tree) |
| `failed to set power state`, `Device or resource busy`, `xioctl(VIDIOC_*)` | Another process holds the camera: an old container (`docker ps -a`), or something on the host using `/dev/video*` (`sudo fuser -v /dev/video*`). Only one process may own the D435i |
| `lsusb -t` shows `480M`, or `Usb Type Descriptor 2.1`; `Couldn't resolve requests` for your profile | USB 2 link: cable, plug orientation or a hub. On USB 2, use `424,240,15` or `640,480,6` until it's fixed |
| `Frames didn't arrive within 5 seconds`, stalls after minutes | Dock power or the link: `RS_INITIAL_RESET=true` (default), lower fps, check `dmesg -w` for USB disconnects while it runs |
| Firmware warning at start | librealsense 2.51.1 recommends D4xx FW 5.13.0.50. Usually harmless; update with `rs-fw-update` if streams misbehave |

### DDS discovery across machines

Topics missing on the workstation, but `./go2_realsense.sh topics` shows them
on the robot:

| Check | How |
|---|---|
| Same domain + RMW on both ends | `echo $ROS_DOMAIN_ID $RMW_IMPLEMENTATION` on the workstation; `docker exec go2-realsense env \| grep -E 'DOMAIN\|RMW'` |
| Robot bound to the right interface, peer = the workstation's current IP | `docker exec go2-realsense cat /tmp/rs-cyclonedds.xml`. The workstation's DHCP address changes: rerun `realsense_host.sh robot-start` |
| Workstation bound to the NIC that routes to the robot | `cat $HOME/.ros/go2-realsense-cyclonedds.xml`; `ip route get <robot IP>` |
| Which interface Cyclone actually picked | robot: `RS_DDS_TRACE=1 ... robot-start`, then `docker exec go2-realsense grep -i interface /tmp/cdds.log` |
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
  topics and `realsense_host.sh republish`.
- **The domain-64 graph is crowded.** Cyclone only probes peer ports up to
  `MaxAutoParticipantIndex`. Both ends set 120. A config without it (default
  9) stops discovering once the workstation runs more than ~9 ROS
  processes.
