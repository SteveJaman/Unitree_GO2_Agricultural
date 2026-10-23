# 3D Digital Twin Navigation (RTAB-Map + YOLO + Foxglove)

[← Back to README](../README.md) · [Roadmap](roadmap.md) · [Tour](tour.md) · [Troubleshooting](troubleshooting.md)

[`scripts/runtime/wifi_3d_nav.sh`](../scripts/runtime/wifi_3d_nav.sh) runs the
normal 2D session and adds a live, dense 3D map of the building on top of it.
When the Go2 camera publishes, the map is coloured from it, and YOLO11s labels
objects and can mark them as obstacles for Nav2. You can view all of it in
Foxglove from the Windows host and send the dog somewhere by clicking in the
3D view.

`wifi_nav.sh` is **not modified**. Use `wifi_nav.sh` when you only need 2D
exploration, and `wifi_3d_nav.sh` when you want the 3D view.

---

## Architecture

```
go2-lidar3d container (wifi_nav.sh, unchanged)       go2-3d container (wifi_3d_nav.sh)
──────────────────────────────────────────────       ─────────────────────────────────
Point-LIO (Jetson) ─► /lidar3d/registered ──────────► RTAB-Map ─► /rtabmap/cloud_map
go2_odom_tf ─► TF lidar3d_map → base_link ──────────►   (TF odometry)   rtabmap_map → lidar3d_map
camera relay ─► /camera/image_raw ──────────────────► go2_camera_info ─► /camera/camera_info
                                                      pointcloud_to_depthimage ─► /camera/lidar_depth
                                                        (LiDAR projected into the image → RTAB-Map RGB-D)
                                                      static TF base_link → camera_link
                                                      yolo_node (YOLO11s) ─► /yolo/detections, /yolo/image,
Nav2 costmap ◄── /yolo/obstacles (opt-in) ◄──────────     /yolo/markers, /yolo/obstacles
Nav2 ◄── /goal_pose ◄───────────────────────────────── go2_click_to_goal ◄── /clicked_point
RViz, explorer                                        foxglove_bridge :8765 ◄──► Foxglove (Windows)
```

- **Point-LIO stays in charge of localization.** Nav2 still plans in
  `lidar3d_map`. RTAB-Map reads Point-LIO's pose from TF as its odometry and
  builds the dense map. Its own frame `rtabmap_map` becomes the parent of
  `lidar3d_map`, and Nav2 does not notice.
- **RTAB-Map runs in its own container (`go2-3d`).** `wifi_nav.sh` tears down
  `go2-lidar3d` when it exits. If RTAB-Map ran inside it, it would be killed
  before saving its database. `wifi_3d_nav.sh` stops `go2-3d` gracefully
  after the session ends, so the map is always written.
- **Same DDS settings as the session.** `go2-3d` copies `ROS_DOMAIN_ID`,
  `RMW_IMPLEMENTATION` and the CycloneDDS config from a running session
  node, so it sees exactly the topics RViz sees. This also works with `--sim`.
- **Foxglove bridge, not rosbridge, by default.** `foxglove_bridge` sends point
  clouds in binary form. rosbridge encodes them as JSON, which is far slower
  for a dense map. `--rosbridge` is available if you need it (port 9090).
- **The camera is optional, detected at start.** In the default `auto` mode,
  `go2-3d` waits up to `TWIN_CAMERA_WAIT` (60 s) for a frame on
  `/camera/image_raw`.
  - **A frame arrives:** RTAB-Map runs in `camera` mode (colour plus visual
    loop closures) and YOLO starts.
  - **No frame:** for example the Jetson camera backend is down
    ([STATUS](STATUS.md#blocked-by-hardware)). The map stays LiDAR-only, so a
    missing camera never stops mapping.

---

## Camera fusion and YOLO

**Colour without a depth camera.** The Go2 front camera is monocular and
publishes no `camera_info`. `go2-3d` fills the gaps:

| Piece | What it does |
|---|---|
| Static TF `base_link → camera_link` | The camera's optical frame (z forward, x right, y down) at the official Go2 URDF `front_camera_joint`, (0.327, 0, 0.043) m. `TWIN_CAMERA_TF="x y z yaw pitch roll"`; skipped if the session already publishes that frame; `none` disables it |
| [`go2_camera_info.py`](../scripts/nodes/go2_camera_info.py) | `/camera/camera_info` with every frame. Pinhole intrinsics from `TWIN_CAMERA_HFOV_DEG` (120° default, 68.75° with `--sim`) |
| `pointcloud_to_depthimage` (rtabmap_util) | Projects `/lidar3d/registered` into the camera → sparse depth image `/camera/lidar_depth` |
| RTAB-Map `camera` mode | RGB + LiDAR depth + scan cloud. `/rtabmap/cloud_map` is coloured; the LiDAR still does the registration |

**The intrinsics and the camera offset are approximations, not a
calibration.** If colours land on the wrong surfaces, or YOLO boxes sit in the
wrong place, calibrate the camera (`camera_calibration`). Then set the real
field of view, or fx/fy/cx/cy on `go2_camera_info.py`, and the real mount in
`TWIN_CAMERA_TF`. This is roadmap item 3.4.

**YOLO11s** ([`twin/yolo_node.py`](../twin/yolo_node.py)) runs on the VM's CPU
at `TWIN_YOLO_RATE` (2 Hz):

| Topic | Content |
|---|---|
| `/yolo/detections` | `vision_msgs/Detection2DArray`, every detection (80 COCO classes) |
| `/yolo/image` | Camera frame with boxes drawn: add it as an Image panel in Foxglove |
| `/yolo/markers` | Orange boxes + labels (`chair 2.1 m (lidar)`) in the 3D view |
| `/yolo/obstacles` | `PointCloud2` in `lidar3d_map`: a 1 m-tall box of points for each obstacle-class detection |

Each obstacle is placed by distance:
- **LiDAR:** the 20th-percentile depth of the LiDAR points that project inside
  the box.
- **Floor:** when there are fewer than 5 such points (glass, thin chair legs),
  the bottom of the box is intersected with the floor plane, taken as
  `base_link` minus 0.30 m.

Obstacle classes default to furniture, people and pets (`TWIN_YOLO_CLASSES`,
or `all`).

**COCO has no "door" or "glass" class.** The stock `yolo11s.pt` will not see
a glass door. For that, train a model on doors and glass and pass it with
`TWIN_YOLO_MODEL=/data/my_model.pt` (any file in the `TWIN_DB_DIR` folder is
visible as `/data/…`).

### Let Nav2 avoid what YOLO sees (opt-in)

Nav2's costmap reads only the topics listed in its own config, which lives in
the upstream go2-nav2-wifi repo. Run this once on the VM:

```bash
./scripts/tools/add-yolo-costmap-source.sh          # adds /yolo/obstacles, rebuilds go2_nav2
./scripts/tools/add-yolo-costmap-source.sh --undo   # back to the original config
```

The script adds a `yolo` observation source (marking only) to the global
costmap's obstacle layer in `go2_nav2/config/lidar3d_planner.yaml`. The local
costmap is copied from it at launch. While YOLO keeps seeing an object, its
cells stay lethal, so the planner routes around them. Once YOLO stops
reporting the object, the LiDAR clears the cells as usual.

Limits:
- **Accuracy:** the obstacles are as accurate as the camera calibration.
- **Range:** at 2 Hz, only objects within `max_range` (6 m) are injected.
- **False positives** block the path too. Watch `/yolo/markers` on the first runs.

---

## One-time setup

On the VM (needs internet, ~3–4 GB download: RTAB-Map, CPU PyTorch, YOLO11s
weights):

```bash
./scripts/runtime/wifi_3d_nav.sh --build      # builds go2-3d:local
./scripts/runtime/wifi_3d_nav.sh --check      # wifi_nav.sh checks + image check
```

On Windows: install the Foxglove desktop app from
[foxglove.dev/download](https://foxglove.dev/download). The open-source fork
[Lichtblick](https://github.com/lichtblick-suite/lichtblick) also works with
the same connection.

---

## Running

```bash
./scripts/runtime/wifi_3d_nav.sh                  # explore + 3D map (wifi_nav.sh defaults)
./scripts/runtime/wifi_3d_nav.sh --no-explore     # you drive by clicking goals
./scripts/runtime/wifi_3d_nav.sh --sim            # against the Gazebo simulation
```

Any `wifi_nav.sh` option is passed through (`--robot-ip`, `--arm`,
`--no-camera`, …). The 3D-specific options are:

| Option | Effect |
|---|---|
| *(default)* | `auto`: Go2 camera fusion + YOLO if `/camera/image_raw` publishes, else LiDAR-only |
| `--camera` | Force camera mode (RTAB-Map waits for frames: no map until the camera works) |
| `--lidar-only` | LiDAR-only map, no camera, no YOLO |
| `--no-yolo` | Camera colour without YOLO (saves CPU) |
| `--rgbd` | RealSense colour + depth instead of the Go2 camera. Needs the RealSense driver publishing `/camera/color/*` and `/camera/depth/*` |
| `--rosbridge` | rosbridge on `:9090` instead of foxglove_bridge on `:8765` |
| `--no-bridge` | RTAB-Map only, no web bridge |
| `--build` | Build the `go2-3d:local` image |
| `--export [DB]` | Export the newest (or the given) database to a PLY point cloud |
| `--stop` | Stop a running `go2-3d` and save its database |

About 20–90 s after RViz opens, the terminal prints a `3D digital twin READY`
block. It shows the RTAB-Map mode (`camera` or `lidar`), the database path,
whether YOLO is on, and the Foxglove address.

**Stop:** Ctrl+C as usual. `wifi_nav.sh` shuts the session down first. Then
RTAB-Map gets up to `TWIN_STOP_TIMEOUT` (60 s) to write the database. Wait for
`[3d] Map saved: …` before closing the terminal.

Each run writes its own database:
`~/files/go2-nav2-wifi/ws/maps/rtabmap/rtabmap-YYYYmmdd-HHMMSS.db`.

---

## Foxglove setup

1. **Open connection → Foxglove WebSocket →** `ws://<VM IP>:8765`.
   The VM IP is the `Foxglove :` line printed by the script, i.e. the VM's
   Wi-Fi address (DHCP, so it changes).
   With `--rosbridge`, choose **Rosbridge** and port `9090`.
2. **Add panel → 3D.** In its settings:
   - **Display frame:** `lidar3d_map`
   - Enable topics:
     - `/rtabmap/cloud_map`: the dense 3D map. Colour by `z` for a height map
     - `/lidar3d/registered`: the live scan
     - `/tf`: the dog's pose
     - `/plan`: Nav2's route
     - `/global_costmap/costmap`: optional
     - `/clicked_goal_markers`: grey cylinder where you clicked, green arrow at the snapped goal
     - `/yolo/markers`: labelled YOLO obstacles
3. **Add panel → Image** → `/yolo/image` for the camera with YOLO boxes. Use
   `/camera/image_raw` instead for the plain feed.
4. **Layout → Save** (or export it to a JSON file to share with the team).

### Click-to-go

1. In the 3D panel, switch to the **2D (top-down) view**.
2. Click **Publish** in the panel toolbar and choose **Point**. The default
   topic is `/clicked_point`.
3. Click the floor where the dog should go.

`go2_click_to_goal` transforms the point into the costmap frame
(`lidar3d_map`) and **snaps it to the nearest navigable floor**. It then faces
the goal along the robot → goal direction and publishes `/goal_pose`, and Nav2
plans and drives there.

Snapping, on `/global_costmap/costmap`:

1. A breadth-first search runs outward from the clicked cell, ring by ring,
   up to `max_snap_distance` (2 m).
2. A cell is accepted only if it **and its 8 neighbours** (a 3×3 block, so
   the body fits) are known and cost < `free_threshold` (50). Unknown space
   is never accepted.
3. Of the accepted cells in the nearest ring, the one closest to the click
   wins.

A click on a desk or wall therefore lands on the free floor next to it.
`/clicked_goal_markers` shows a grey cylinder at the click and a green arrow
at the goal actually sent. If no safe floor is within reach, or no costmap has
arrived yet, no goal is sent and the log says why.

Why top-down: Foxglove projects a click onto the display frame's `z = 0`
plane, which need not be exactly the floor. In an oblique 3D view, a click
aimed at the floor can land short of it. From directly above the error
disappears.

Alternatives:
- **Publish → Pose** with the topic set to `/goal_pose` sends a goal with the
  heading you drag, bypassing the click node.
- **Publish → Pose estimate** publishes `/initialpose`, the same as RViz's
  2D Pose Estimate.

Each click replaces the current goal. The **Publish → Pose** route above
skips the snapping.

Node parameters (see
[`go2_click_to_goal.py`](../scripts/nodes/go2_click_to_goal.py)):

| Parameter | Default | Meaning |
|---|---|---|
| `free_threshold` | `50` | Costmap value (0–100) below which a cell counts as floor |
| `clearance_cells` | `1` | Free radius around the goal cell: 1 = 3×3, 2 = 5×5 |
| `max_snap_distance` | `2.0` | How far (m) a click may be moved |
| `max_goal_distance` | `0` | Ignore goals farther than this from the robot (0 = off) |
| `costmap_topic` | `/global_costmap/costmap` | Grid used for snapping |
| `floor_z` | `0.0` | Height of the green arrow in the map frame |

---

## After the run: export and mesh

```bash
./scripts/runtime/wifi_3d_nav.sh --export          # newest db -> <db>-export/*.ply
python3 scripts/tools/export_tour.py <file.ply> --from-ply -o tour_mesh.ply
```

`--export` runs `rtabmap-export --scan` inside the `go2-3d` image (LiDAR
point cloud). Extra options go in `TWIN_EXPORT_ARGS`, e.g.
`TWIN_EXPORT_ARGS="--voxel 0.02"`. For a run recorded in `camera` mode,
`TWIN_EXPORT_ARGS="--cam_projection"` colours the LiDAR cloud from the stored
camera images. The database also opens in
`rtabmap-databaseViewer` for manual inspection and export.

---

## Tuning (environment variables)

| Variable | Default | Meaning |
|---|---|---|
| `TWIN_CLOUD_TOPIC` | `/lidar3d/registered` | Cloud fed to RTAB-Map |
| `TWIN_RTAB_ARGS` | — | Extra RTAB-Map parameters, e.g. `"--Grid/CellSize 0.1 --Grid/RangeMax 8"` (lighter on the VM) |
| `TWIN_BRIDGE_PORT` | `8765` / `9090` | Bridge port |
| `TWIN_RGB_TOPIC` | `/camera/image_raw` (`--rgbd`: `/camera/color/image_raw`) | Camera image |
| `TWIN_CAMERA_WAIT` | `60` | Seconds to wait for the first camera frame (`auto`) |
| `TWIN_CAMERA_TF` | `0.327 0 0.043 -1.5708 0 -1.5708` | `base_link → camera` optical frame: x y z yaw pitch roll; `none` = don't publish |
| `TWIN_CAMERA_FRAME` | from the image header | Child frame for that TF |
| `TWIN_CAMERA_HFOV_DEG` | `120` (`--sim`: `68.75`) | Horizontal field of view for the approximate intrinsics |
| `TWIN_YOLO_MODEL` | `/opt/yolo/yolo11s.pt` | YOLO weights (baked into the image) |
| `TWIN_YOLO_RATE` | `2` | Inferences per second (CPU) |
| `TWIN_YOLO_CLASSES` | furniture, people, pets | Comma list of classes that become obstacles, or `all` |
| `TWIN_DEPTH_TOPIC` / `TWIN_CAMINFO_TOPIC` | RealSense defaults | `--rgbd` inputs |
| `TWIN_DB_DIR` | `$NAV2_WIFI_DIR/ws/maps/rtabmap` | Where databases go |
| `TWIN_LOG_DIR` | `$NAV2_WIFI_DIR/ws/log/3d` | `rtabmap.log`, `bridge.log`, `click_to_goal.log`, `yolo.log`, `camera_info.log`, `lidar_depth.log`, `camera_tf.log` |
| `TWIN_START_TIMEOUT` | `300` | Seconds to wait for the session's ROS nodes |
| `TWIN_STOP_TIMEOUT` | `60` | Seconds RTAB-Map gets to save on stop |

The RTAB-Map LiDAR defaults (ICP registration, proximity loop closures,
5 cm 3D grid) are in [`twin/start_inside.sh`](../twin/start_inside.sh).

---

## Troubleshooting

| Symptom | Check / fix |
|---|---|
| `image go2-3d:local missing` | `./scripts/runtime/wifi_3d_nav.sh --build` |
| `no ROS nodes in go2-lidar3d … 3D stack NOT started` | The 2D session never came up. Fix that first with `wifi_nav.sh` alone |
| Foxglove connects but shows no topics | VM firewall or wrong IP. Use the IP printed in the READY block; `ss -tlnp \| grep 8765` on the VM |
| `/rtabmap/cloud_map` empty | `tail ~/files/go2-nav2-wifi/ws/log/3d/rtabmap.log`. Look for TF errors (`lidar3d_map → base_link` must resolve) or no data on `/lidar3d/registered` |
| Clicks do nothing | `tail …/ws/log/3d/click_to_goal.log`. `no costmap received yet`: the session's Nav2 is not up. `no navigable floor`: click closer to open floor, or raise `max_snap_distance`. Also confirm the Publish tool's topic is `/clicked_point` |
| `no camera frames … LiDAR-only map, YOLO off` | The camera relay isn't publishing. `tail /tmp/camera_relay.log` in `go2-lidar3d`; see [camera.md](camera.md) |
| Map colours land on the wrong surfaces | Camera intrinsics / `TWIN_CAMERA_TF` are off: calibrate (see [Camera fusion and YOLO](#camera-fusion-and-yolo)) |
| `/camera/lidar_depth` empty | `tail …/ws/log/3d/lidar_depth.log`. TF `lidar3d_map → camera_link` must resolve |
| YOLO boxes in `/yolo/image` but no `/yolo/markers` | Box bottom above the horizon and no LiDAR points inside it, or farther than 6 m. `tail …/ws/log/3d/yolo.log` |
| Nav2 ignores YOLO obstacles | Run `./scripts/tools/add-yolo-costmap-source.sh` once. With a VoxelLayer, check its z range covers the boxes |
| VM CPU pegged | `--no-yolo` or `TWIN_YOLO_RATE=1`; `TWIN_RTAB_ARGS="--Grid/CellSize 0.1 --Grid/RangeMax 8 --Rtabmap/DetectionRate 0.5"` |
| Map not saved after a crash | A leftover `go2-3d` is stopped (and saved) at the start of the next run, or run `--stop` |

`go2-3d` keeps its own log after it stops: `docker logs go2-3d`.

---

## Not done yet

- **Camera calibration.** The intrinsics and mount are approximations until
  roadmap item 3.4 is done.
- **Doors and glass.** These need a custom-trained YOLO model; COCO has neither class.
- **YOLO on the Jetson.** Inference runs on the VM's CPU at 2 Hz; the Jetson GPU
  would be much faster. See [roadmap](roadmap.md#milestone-4--rtab-map-reconstruction).
- **Waypoint queue from Foxglove.** Each click replaces the current goal. For a
  queued route, use the RViz waypoint panel ([Tour](tour.md)).
