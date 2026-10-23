# 3D Digital Twin Navigation (RTAB-Map + YOLO + Web Dashboard)

[← Back to README](../README.md) · [Roadmap](roadmap.md) · [Tour](tour.md) · [Troubleshooting](troubleshooting.md)

[`scripts/runtime/wifi_3d_nav.sh`](../scripts/runtime/wifi_3d_nav.sh) runs the
normal 2D session and adds a live, dense 3D map of the building on top of it.
When the Go2 camera publishes, the map is coloured from it. With `--ai`:
- **YOLO11s** labels objects and can mark them as obstacles for Nav2.
- **A local VLM** describes the view.
- **The battery manager** brings the dog back to the dock on low battery.

You can view all of it in the **web dashboard**, a React + three.js app served
by the session at `http://<VM IP>:3000`. You can also send the dog somewhere
by clicking in the 3D view.

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
Nav2 costmap ◄── /yolo/obstacles (auto with --ai) ◄──     /yolo/markers, /yolo/obstacles     (--ai)
                                                      vlm_reasoning_node ─► Ollama ─► /vlm_decision (--ai)
explorer, Nav2 ◄── /explore/resume False, /goal_pose ─┘ (2x BLOCKED → pause + dock)
                                                      auto_localize ─► /initialpose (0,0,0) once (--ai / --resume)
Nav2 ◄── /goal_pose ◄───────────────────────────────── go2_click_to_goal ◄── /clicked_point
battery_manager (--ai) ◄── /lf/lowstate                save_location (--save-location) ◄── /clicked_point
  └─► /explore/resume False, /goal_pose → dock
battery_manager (always) ─► /battery_state          image_compressor ─► /twin/camera|yolo/compressed (JPEG)
RViz, explorer                                        rosbridge :9090 ◄──► web dashboard :3000 (any browser)
                                                      foxglove_bridge :8765 (--foxglove, alternative)
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
- **The web dashboard is the main interface.**
  - **Served from `go2-3d`:** the dashboard ([`twin/webapp`](../twin/webapp/))
    is built into the image, so the VM needs no Node.js. `go2-3d` serves it on
    port 3000 and runs rosbridge on 9090.
  - **Point clouds** are requested in CBOR (binary) and throttled (map every
    2 s).
  - **Camera images** are sent as 640 px JPEGs at 5 Hz, made by
    [`image_compressor.py`](../twin/image_compressor.py) only while a browser
    watches. A raw 720p frame through rosbridge would be ~2.7 MB.
  - **Foxglove** stays available as an alternative (`--foxglove`).
- **The camera is optional, detected at start.** In the default `auto` mode,
  `go2-3d` waits up to `TWIN_CAMERA_WAIT` (60 s) for a frame on
  `/camera/image_raw`.
  - **A frame arrives:** RTAB-Map runs in `camera` mode (colour plus visual
    loop closures), and with `--ai` YOLO and the VLM start.
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

**YOLO11s** ([`twin/yolo_node.py`](../twin/yolo_node.py), started by `--ai`)
runs on the VM's CPU at `TWIN_YOLO_RATE` (2 Hz):

| Topic | Content |
|---|---|
| `/yolo/detections` | `vision_msgs/Detection2DArray`, every detection (80 COCO classes) |
| `/yolo/image` | Camera frame with boxes drawn (dashboard camera panel, **YOLO** tab) |
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

### Let Nav2 avoid what YOLO sees (automatic with `--ai`)

Nav2's costmap reads only the topics listed in its own config, which lives in
the upstream go2-nav2-wifi repo.
- **With `--ai`:** unless `--no-yolo` is also given, `wifi_3d_nav.sh` runs
  the patch script before the session starts. It is a no-op once applied. If
  it fails, the session still starts and the banner shows
  `YOLO → Nav2 : FAILED`.
- **By hand:**

```bash
./scripts/tools/add-yolo-costmap-source.sh          # adds /yolo/obstacles, rebuilds go2_nav2
./scripts/tools/add-yolo-costmap-source.sh --undo   # back to the original config
```

The change stays after the session. Sessions without `--ai` just have a
`yolo` source with no data, which is harmless. `--undo` removes it.

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
weights, and the web dashboard build):

```bash
./scripts/runtime/wifi_3d_nav.sh --build      # builds go2-3d:local (incl. twin/webapp)
./scripts/runtime/wifi_3d_nav.sh --check      # wifi_nav.sh checks + image check
```

Nothing to install on Windows: the dashboard runs in any browser (Chrome,
Edge, Firefox). Re-run `--build` after changing `twin/webapp`. For quicker
iterations, see [Developing the dashboard](#developing-the-dashboard).

---

## Running

```bash
./scripts/runtime/wifi_3d_nav.sh                  # explore + 3D map (wifi_nav.sh defaults)
./scripts/runtime/wifi_3d_nav.sh --no-explore     # you drive by clicking goals
./scripts/runtime/wifi_3d_nav.sh --ai             # + YOLO, VLM, battery manager
./scripts/runtime/wifi_3d_nav.sh --ai --resume    # after a battery swap: continue the last map
./scripts/runtime/wifi_3d_nav.sh --sim            # against the Gazebo simulation
```

Any `wifi_nav.sh` option is passed through (`--robot-ip`, `--arm`,
`--no-camera`, …). The 3D-specific options are:

| Option | Effect |
|---|---|
| *(default)* | `auto`: Go2 camera fusion if `/camera/image_raw` publishes, else LiDAR-only |
| `--ai` | Also start YOLO11s and the VLM (both need the camera) and the battery manager. See [AI nodes](#ai-nodes---ai) |
| `--no-yolo` / `--no-vlm` / `--no-battery` | With `--ai`: leave that one out |
| `--resume` | Continue the newest database instead of starting a new one. See [Resuming a map](#resuming-a-map---resume) |
| `--save-location` | In a second terminal: label points clicked in the dashboard, saved to `config/semantic_locations.json` |
| `--camera` | Force camera mode (RTAB-Map waits for frames: no map until the camera works) |
| `--lidar-only` | LiDAR-only map, no camera (so no YOLO or VLM) |
| `--rgbd` | RealSense colour + depth instead of the Go2 camera. Needs the RealSense driver publishing `/camera/color/*` and `/camera/depth/*` |
| `--foxglove` | Also start foxglove_bridge on `:8765`, see [Alternative visualization](#alternative-visualization-foxglove) |
| `--no-web` | rosbridge only, no dashboard server on `:3000` |
| `--no-bridge` | RTAB-Map only: no rosbridge, no dashboard |
| `--build` | Build the `go2-3d:local` image |
| `--export [DB]` | Export the newest (or the given) database to a PLY point cloud |
| `--stop` | Stop a running `go2-3d` and save its database |

About 20–90 s after RViz opens, the terminal prints a `3D digital twin READY`
block. It shows the RTAB-Map mode (`camera` or `lidar`), the database path,
the state of YOLO, the VLM and the battery manager, and the **dashboard
address**.

**Stop:** Ctrl+C as usual. `wifi_nav.sh` shuts the session down first. Then
RTAB-Map gets up to `TWIN_STOP_TIMEOUT` (60 s) to write the database. Wait for
`[3d] Map saved: …` before closing the terminal.

Each run writes its own database:
`~/files/go2-nav2-wifi/ws/maps/rtabmap/rtabmap-YYYYmmdd-HHMMSS.db`.
With `--resume`, it continues the newest one instead.

---

## AI nodes (`--ai`)

`--ai` starts the autonomy loop. Each part can be left out (`--no-yolo`,
`--no-vlm`, `--no-battery`, `--no-localize`). Logs are in `ws/log/3d/`:
`yolo.log`, `vlm.log`, `battery.log`, `auto_localize.log`.
- **YOLO11s**, with its obstacles patched into the Nav2 costmap (above).
- **The VLM**, which pauses and docks on a confirmed `BLOCKED`.
- **Battery docking.**
- **Auto-localization**, also with `--resume`.

**Prerequisite:** run `ollama serve` with `ollama pull llava:7b` done (see
[VLM reasoning](#vlm-reasoning-ollama)). The script checks it at start.

### Auto-localization (`--ai`, `--resume`)

[`auto_localize.py`](../scripts/nodes/auto_localize.py) runs once at session
start. It publishes the dock pose (x=0, y=0, yaw=0 in `lidar3d_map`) as
`geometry_msgs/PoseWithCovarianceStamped` on `/initialpose`, then exits, so
nobody has to click "2D Pose Estimate" after a reboot at the dock.
- **Timing:** it waits up to 20 s for a subscriber, so the message is not
  lost to DDS discovery, then publishes it 3 times.
- **Safety:** if TF shows the robot already more than 0.5 m from the origin
  (the session has been driven), it does **not** publish. Re-anchoring a
  robot that is elsewhere would be wrong.
- **If nothing listens:** nothing in this repo subscribes to `/initialpose`,
  and Point-LIO already starts every session at the `lidar3d_map` origin.
  With no subscriber, the log says so and the dock pose is right anyway, as
  long as **the dog starts at the dock**.
- **Opt out** with `--no-localize`.

### Battery manager

[`battery_manager.py`](../scripts/nodes/battery_manager.py) runs inside
`go2-lidar3d`, because it needs the Unitree messages. It reads `bms_state.soc`
from `/lf/lowstate`, the copy the Wi-Fi relay forwards, or from `/lowstate`.

| Battery | Action |
|---|---|
| below `TWIN_BATTERY_LOW` (15 %) | `/explore/resume` False (the explorer stops and cancels its goal). 2 s later, `/goal_pose` to the dock. Re-sent every 30 s until the dog is within 0.5 m |
| below `TWIN_BATTERY_RESUME` (20 %) | Every explorer START is answered with STOP |
| at or above 20 % | Released. The explorer is **not** restarted automatically; click START |

The dock is `TWIN_DOCK="x y yaw"` in `lidar3d_map`, default `0 0 0`.
**`lidar3d_map`'s origin is wherever the dog stood when the session started**,
so start every session at the dock, or set `TWIN_DOCK`. The status is
published on `/battery_manager/status`. If the relay doesn't forward the
battery, `battery.log` warns `no battery data` after 30 s. In `--sim` there
is no battery, so it stays idle.

### VLM reasoning (Ollama)

[`vlm_reasoning_node.py`](../scripts/nodes/vlm_reasoning_node.py) sends the
newest camera frame to an Ollama server (`llava:7b` by default) with the
prompt *"…Do you see a hydroponic station? Or is the path ahead blocked by a
desk?…"*.
- It publishes `HYDROPONIC`, `BLOCKED`, `CLEAR` or `UNKNOWN` on
  `/vlm_decision`, and the full answer on `/vlm_decision/raw`.
- It asks every 5 s at most, and only after the previous answer has arrived.
- While Ollama is unreachable it logs a warning and publishes nothing.

**It acts on `BLOCKED`.** After `TWIN_VLM_CONFIRM` (2) `BLOCKED` answers in a
row:
1. It publishes `False` on `/explore/resume`, so the explorer stops and
   cancels its goal.
2. One second later it publishes `/goal_pose` to the dock (`TWIN_DOCK`,
   default 0 0 0).
3. It then waits 60 s before it can act again.

Any other answer resets the count, since a single misread frame must not
abort a mission. The explorer is not restarted automatically; click START in
the dashboard. `TWIN_VLM_ACT=0` makes the VLM report only. `HYDROPONIC`
does not trigger anything yet.

Ollama setup (once):

```bash
curl -fsSL https://ollama.com/install.sh | sh    # on the VM
ollama pull llava:7b                             # ~4.7 GB
```

llava:7b on the VM's CPU takes tens of seconds per answer. A Windows host with
a GPU is much faster:
1. Install Ollama on Windows.
2. Set `OLLAMA_HOST=0.0.0.0` there.
3. Allow port 11434 through the Windows firewall.
4. On the VM, run
   `TWIN_VLM_URL=http://<Windows IP>:11434/api/generate ./scripts/runtime/wifi_3d_nav.sh --ai`.

Other settings: `TWIN_VLM_MODEL`, `TWIN_VLM_PERIOD`.

### YOLO11s

See [Camera fusion and YOLO](#camera-fusion-and-yolo).

---

## Saving named locations (`--save-location`)

While a session runs, open a second terminal:

```bash
./scripts/runtime/wifi_3d_nav.sh --save-location
```

1. Click a point in the dashboard (**Click-to-go** on, then click the floor).
2. The terminal prints its `lidar3d_map` coordinates and asks for a label,
   such as `Dock` or `Hydroponic_Station`. Enter skips the point.

Labels are saved to [`config/semantic_locations.json`](../config/), with
`x, y, z`, the frame and the time. When the TF exists, the same point is also
stored in `rtabmap_map`. Reusing a label replaces the old entry.

Notes:
- **The dog drives there too.** `go2_click_to_goal` also listens on
  `/clicked_point`, so every labelled click is also a goal. To label without
  driving, publish on `/label_point` instead (with Foxglove's Publish tool,
  `--foxglove`), and run the node with `-p topic:=/label_point` (see
  [`save_location.py`](../scripts/nodes/save_location.py)).
- **`lidar3d_map` coordinates hold only if the dog starts each session from
  the same spot.**

---

## Resuming a map (`--resume`)

After a battery swap, restart with `--resume`:

```bash
./scripts/runtime/wifi_3d_nav.sh --ai --resume
```

1. The newest `.db` in `TWIN_DB_DIR` is copied to `<db>.bak-<time>`.
2. That database is reopened. RTAB-Map adds the new run to it as a new
   session (nothing is deleted).
3. As soon as it recognises a place from the earlier run, it links the two
   and `/rtabmap/cloud_map` shows the combined map.

Recognition works best in `camera` mode, because it uses visual features. With
LiDAR only, the sessions may stay unlinked in the same file. Either way,
**start the dog where the last run started** (the dock): Nav2's `lidar3d_map`
always restarts from the dog's start pose, and the dock goal and saved
locations depend on it.

---

## Web dashboard

Open your browser and navigate to **`http://localhost:3000`** on the VM, or
**`http://<VM IP>:3000`** from the Windows host. The READY block prints the
exact address. The VM IP is DHCP, so it changes.

The dashboard connects to rosbridge on the same host, port 9090. Override it
with `?ws=ws://<host>:9090`. URL options: `?frame=` (fixed frame, default
`lidar3d_map`), `?base=` (robot frame, default `base_link`).

| Part | Shows | Topics |
|---|---|---|
| **Status bar** | rosbridge connection · battery % and voltage (orange/red below 30/15 %, dock state with `--ai`) · commanded speed · VLM decision and its age · map size · explorer state with **START / STOP** | `/battery_state`, `/battery_manager/status`, `/cmd_vel`, `/vlm_decision`, `/explore/status`, `/explore/resume` |
| **3D view** | Dense map (camera colours, else height colours), robot, Nav2 plan (yellow), active goal (green), YOLO boxes with labels, optional live scan | `/rtabmap/cloud_map`, `/tf`, `/tf_static`, `/plan`, `/goal_pose`, `/yolo/markers`, `/lidar3d/registered` |
| **Toolbar** | **Click-to-go**, **Follow robot**, layer toggles (3D map, live scan, plan, YOLO) | `/clicked_point` |
| **Camera panel** | **Camera** / **YOLO** tabs, frame rate | `/twin/camera/compressed`, `/twin/yolo/compressed` |

Mouse: left-drag orbit, right-drag pan, wheel zoom.

The source is in [`twin/webapp`](../twin/webapp/):
- [`RosProvider.jsx`](../twin/webapp/src/RosProvider.jsx): connection,
  automatic reconnect, `useTopic` / `usePublisher`.
- [`Scene.jsx`](../twin/webapp/src/Scene.jsx): the three.js scene.
- [`StatusBar.jsx`](../twin/webapp/src/StatusBar.jsx).
- [`CameraFeed.jsx`](../twin/webapp/src/CameraFeed.jsx).
- [`src/ros/`](../twin/webapp/src/ros/): TF tree and PointCloud2 decoder,
  with no `tf2_web_republisher` needed.

### Click-to-go

1. Turn on **Click-to-go** in the dashboard toolbar. The cursor becomes a
   crosshair.
2. Click the floor where the dog should go. Orbit drags are ignored. An orange
   ring marks the click until Nav2's goal (green arrow) appears.

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

The click is taken on the fixed frame's `z = 0` plane, which need not be
exactly the floor. In a steep oblique view a click aimed at the floor can
land a little short of it, so look down from above for precise goals. Each
click replaces the current goal.

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
| `TWIN_WEB_PORT` / `TWIN_ROSBRIDGE_PORT` / `TWIN_FOXGLOVE_PORT` | `3000` / `9090` / `8765` | Dashboard, rosbridge, foxglove_bridge ports |
| `TWIN_RGB_TOPIC` | `/camera/image_raw` (`--rgbd`: `/camera/color/image_raw`) | Camera image |
| `TWIN_CAMERA_WAIT` | `60` | Seconds to wait for the first camera frame (`auto`) |
| `TWIN_CAMERA_TF` | `0.327 0 0.043 -1.5708 0 -1.5708` | `base_link → camera` optical frame: x y z yaw pitch roll; `none` = don't publish |
| `TWIN_CAMERA_FRAME` | from the image header | Child frame for that TF |
| `TWIN_CAMERA_HFOV_DEG` | `120` (`--sim`: `68.75`) | Horizontal field of view for the approximate intrinsics |
| `TWIN_YOLO_MODEL` | `/opt/yolo/yolo11s.pt` | YOLO weights (baked into the image) |
| `TWIN_YOLO_RATE` | `2` | Inferences per second (CPU) |
| `TWIN_YOLO_CLASSES` | furniture, people, pets | Comma list of classes that become obstacles, or `all` |
| `TWIN_VLM_URL` | `http://localhost:11434/api/generate` | Ollama endpoint |
| `TWIN_VLM_MODEL` / `TWIN_VLM_PERIOD` | `llava:7b` / `5` | VLM model, minimum seconds between requests |
| `TWIN_VLM_ACT` / `TWIN_VLM_CONFIRM` | `1` / `2` | Act on `BLOCKED` (0 = report only) / answers in a row needed |
| `TWIN_AUTOLOCALIZE` | `1` with `--ai`/`--resume` | `/initialpose` at the dock on start (`--no-localize` = 0) |
| `TWIN_BATTERY_LOW` / `TWIN_BATTERY_RESUME` | `15` / `20` | % to return to the dock / to allow exploring again |
| `TWIN_DOCK` | `0 0 0` | Dock `x y yaw` in `lidar3d_map` |
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
| Dashboard does not load | Use the address in the READY block; `ss -tlnp \| grep -E '3000\|9090'` on the VM; `tail …/ws/log/3d/web.log` |
| Dashboard shows "Connecting to rosbridge…" | `tail …/ws/log/3d/rosbridge.log`; VM firewall must allow 9090; or pass `?ws=ws://<VM IP>:9090` |
| Dashboard: no map / robot | Robot needs TF `lidar3d_map → base_link` (session up); map needs RTAB-Map data. Large maps arrive every 2 s; raise `--Grid/CellSize` if the browser struggles |
| Dashboard: camera "no frames" | Camera mode only (see READY `RTAB-Map : camera`); `tail …/ws/log/3d/image_compressor.log` |
| Dashboard: battery "—" | `tail …/ws/log/3d/battery.log` (needs `/lf/lowstate` from the relay) |
| `/rtabmap/cloud_map` empty | `tail ~/files/go2-nav2-wifi/ws/log/3d/rtabmap.log`. Look for TF errors (`lidar3d_map → base_link` must resolve) or no data on `/lidar3d/registered` |
| Clicks do nothing | `tail …/ws/log/3d/click_to_goal.log`. `no costmap received yet`: the session's Nav2 is not up. `no navigable floor`: click closer to open floor, or raise `max_snap_distance`. Also confirm the Publish tool's topic is `/clicked_point` |
| `no camera frames … LiDAR-only map, YOLO off` | The camera relay isn't publishing. `tail /tmp/camera_relay.log` in `go2-lidar3d`; see [camera.md](camera.md) |
| Map colours land on the wrong surfaces | Camera intrinsics / `TWIN_CAMERA_TF` are off: calibrate (see [Camera fusion and YOLO](#camera-fusion-and-yolo)) |
| `/camera/lidar_depth` empty | `tail …/ws/log/3d/lidar_depth.log`. TF `lidar3d_map → camera_link` must resolve |
| YOLO boxes in `/yolo/image` but no `/yolo/markers` | Box bottom above the horizon and no LiDAR points inside it, or farther than 6 m. `tail …/ws/log/3d/yolo.log` |
| Nav2 ignores YOLO obstacles | Banner `YOLO → Nav2 : FAILED`? Run `./scripts/tools/add-yolo-costmap-source.sh` by hand to see why. With a VoxelLayer, check its z range covers the boxes |
| Dog suddenly stops and returns to the dock | `vlm.log` shows `path BLOCKED (confirmed)`? The VLM acted. Raise `TWIN_VLM_CONFIRM` or set `TWIN_VLM_ACT=0`. Otherwise `battery.log` (low battery) |
| `initial pose NOT published` in `auto_localize.log` | The robot was more than 0.5 m from the origin at start: start the session with the dog at the dock |
| `Ollama not reachable` in `vlm.log` | Start Ollama (`ollama serve`), `ollama pull llava:7b`, or set `TWIN_VLM_URL` |
| `no battery data` in `battery.log` | The relay does not forward `/lf/lowstate`; check in `go2sh`: `ros2 topic hz /lf/lowstate` |
| Low battery but the dog doesn't go home | `battery.log` shows `dock goal sent`? Then check Nav2 (`/plan`); the dock is `TWIN_DOCK` in `lidar3d_map` |
| `--resume`: old and new map not joined | Needs a recognised place: `camera` mode, start at the old start point |
| VM CPU pegged | `--no-yolo` / `--no-vlm`, or `TWIN_YOLO_RATE=1`; `TWIN_RTAB_ARGS="--Grid/CellSize 0.1 --Grid/RangeMax 8 --Rtabmap/DetectionRate 0.5"` |
| Map not saved after a crash | A leftover `go2-3d` is stopped (and saved) at the start of the next run, or run `--stop` |

`go2-3d` keeps its own log after it stops: `docker logs go2-3d`.

---

## Developing the dashboard

```bash
cd twin/webapp
npm install
npm run dev        # http://localhost:3000 with hot reload; needs a running session's rosbridge
npm run build      # dist/: go2-3d serves it instead of the image's copy when it is newer
```

Running `npm` needs Node.js 18+ on the machine where you edit (not needed to
*use* the dashboard). Open the dev server with `?ws=ws://<VM IP>:9090` when it
is not on the VM.

---

## Alternative visualization (Foxglove)

The [Foxglove](https://foxglove.dev/download) app (or the open-source
[Lichtblick](https://github.com/lichtblick-suite/lichtblick)) still works.
Start the session with `--foxglove` and connect **Open connection → Foxglove
WebSocket →** `ws://<VM IP>:8765`. You can also use **Rosbridge** on
`ws://<VM IP>:9090`, which is always running.
- **3D panel:**
  - display frame `lidar3d_map`;
  - topics `/rtabmap/cloud_map`, `/tf`, `/plan`, `/yolo/markers`,
    `/clicked_goal_markers`, `/global_costmap/costmap`.
- **Image panel:** `/yolo/image` or `/camera/image_raw`.
- **Click-to-go:** in the 2D (top-down) view, **Publish → Point** on
  `/clicked_point`. It uses the same snapping as the dashboard.
- **Pose goals:** **Publish → Pose** on `/goal_pose` sends a goal without
  snapping; **Publish → Pose estimate** publishes `/initialpose`.

---

## Not done yet

- **Camera calibration.** The intrinsics and mount are approximations until
  roadmap item 3.4 is done.
- **Doors and glass.** These need a custom-trained YOLO model; COCO has neither class.
- **YOLO on the Jetson.** Inference runs on the VM's CPU at 2 Hz; the Jetson GPU
  would be much faster. See [roadmap](roadmap.md#milestone-4--rtab-map-reconstruction).
- **Waypoint queue in the dashboard.** Each click replaces the current goal.
  For a queued route, use the RViz waypoint panel ([Tour](tour.md)).
- **Costmap and named locations in the dashboard.** These aren't drawn yet.
  Use RViz or Foxglove (`--foxglove`) for the costmap.
- **More VLM actions and named locations.** Only `BLOCKED` acts (pause +
  dock). `HYDROPONIC` and `config/semantic_locations.json` are not yet used
  to steer the dog. For example, nothing yet does "go to Hydroponic_Station"
  (roadmap 4.19).
- **Automatic docking/charging.** The battery manager only drives back to the
  dock pose. Swapping or charging the battery is manual.
