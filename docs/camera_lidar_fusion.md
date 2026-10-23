# Camera–LiDAR Fusion: Calibration and Overlay Check

[← Back to README](../README.md) · [Camera](camera.md) · [3D Nav](3d_nav.md) · [Roadmap](roadmap.md#milestone-3--camera-pipeline)

Before camera data can improve collision avoidance or mapping, a camera pixel
must map to the right LiDAR point. That takes two things:

1. **Intrinsics:** the lens model (focal length, optical centre, distortion),
   published as `/camera/camera_info`. Measured with a checkerboard.
2. **Extrinsics:** where the camera sits relative to the LiDAR. It comes from
   TF: `lidar3d_map → base_link → trunk → front_camera`. The URDF puts
   `front_camera` at (0.327, 0, 0.043) m with no rotation. The real camera can
   differ, so the overlay can apply a live correction.

The **overlay check** projects the LiDAR points onto the camera image. When
both calibrations are right, the points land on the objects they hit.

![LiDAR overlay in simulation: wrong camera pitch (top) vs correct (bottom)](images/lidar_overlay_sim.png)

*Simulation. Top: camera pitched 0.1 rad (5.7°) wrong, so points float above
the beds into the sky. Bottom: the same scene after correcting it live with
`ros2 param set`. Red is near, blue is far.*

This is the verification step for the fusion that follows. The coloured
RTAB-Map map (`wifi_3d_nav.sh --fusion`, [3d_nav.md](3d_nav.md#camera--lidar-fusion---fusion))
is only as good as this alignment. See [What comes next](#what-comes-next).

---

## 1. Intrinsic calibration (once per camera and resolution)

You need a printed checkerboard on a flat, rigid board. The default is **9×6
inner corners** (10×7 squares) of **25 mm**. Measure your squares after
printing, because printers scale.

1. Start a normal session with the camera relay (the default):

   ```bash
   ./scripts/runtime/wifi_nav.sh --no-explore
   ```

   Check that frames arrive (`go2sh`, then `ros2 topic hz /camera/image_raw`).

2. Run the calibrator:

   ```bash
   ./scripts/tools/calibrate_camera.sh                                  # 9x6, 25 mm
   ./scripts/tools/calibrate_camera.sh --cols 8 --rows 5 --square 0.030 # other board
   ```

3. Move the board slowly through the image. Cover the centre, all four edges
   and the corners. Hold it near and far, and tilt it left/right and up/down.
   A view is captured automatically whenever the board is found in a new
   enough pose. The coverage bars show what is still missing:

   ```
     view 12/25   X #######...  Y #####.....  size ####......  skew ###.......
   ```

4. After 25 views (or Ctrl+C with at least 8), it solves and prints:

   ```
   RMS reprojection : 0.31 px   (< 0.5 good, < 1.0 usable, > 1.0 redo)
   fx fy            : 919.4 914.6   (horizontal FOV 69.7 deg)
   ```

   If the RMS error is under 1 px, the result is installed as
   [`config/camera/front_camera.yaml`](../config/camera/). Each run's views and
   `undistorted_preview.png` go to `config/camera/runs/<date>/`, which git
   ignores. In the preview, straight edges should look straight.

5. Restart `wifi_nav.sh`. The relay now publishes `/camera/camera_info`
   alongside every image (`[camera] Calibration … -> /camera/camera_info`).

The calibration is only valid at the resolution it was taken at. The relay
uses 720p by default (`CAMERA_RESOLUTION=720p`), and it warns if the frames and
the calibration don't match.

To recalibrate from saved images without the robot:

```bash
python3 scripts/tools/calibrate_camera.py --from-dir config/camera/runs/<date>/views \
    --cols 9 --rows 6 --square 0.025 --out /tmp/recal
```

---

## 2. Overlay check

With a session running:

```bash
./scripts/tools/lidar_overlay.sh           # real camera, /camera/camera_info from the relay
./scripts/tools/lidar_overlay.sh --sim     # simulation: exact Gazebo intrinsics
```

It publishes **`/camera/lidar_overlay`**. Either viewer works, and neither
needs WebGL:

- **RViz:** Add → By topic → `/camera/lidar_overlay` → Image.
- **rqt:** `ros2 run rqt_image_view rqt_image_view /camera/lidar_overlay`.

Every 5 s the terminal reports images, overlays, points per overlay and, if
something is missing, why (`no camera_info`, `no TF to front_camera`, …).

**What good looks like:**

- Points on the floor follow the floor, and the rings get closer together
  with distance.
- Points on a wall, box or person sit on that object. A thin pole gets a
  vertical column of points.
- The colour changes where the depth changes: a near object is red/yellow
  against a blue background.

**What a wrong mount looks like:**

| Symptom | Likely error | Correct with |
|---|---|---|
| Points sit below their objects | pitch: the camera points further down than modelled | raise `offset_rpy` 2nd value (rad) |
| Points sit above their objects | pitch: the camera points further up | lower `offset_rpy` 2nd value |
| Everything shifted left or right | yaw | `offset_rpy` 3rd value |
| Horizon lines tilted | roll | `offset_rpy` 1st value |
| Near objects off, far ones fine | position | `offset_xyz` (m, in `front_camera`: x fwd, y left, z up) |
| Points smeared while turning | timing | Camera and LiDAR stamps disagree. Try `-p use_latest_tf:=true` and compare |

Tune it live from a second terminal, with no restart:

```bash
go2sh
ros2 param set /go2_lidar_overlay offset_rpy "[0.0, 0.03, 0.0]"
ros2 param set /go2_lidar_overlay offset_xyz "[0.0, 0.0, 0.01]"
```

Point the dog at something with clear vertical edges (a door frame, table
legs, a box) about 1–3 m away. Stand still while you tune. Once it lines up,
write down the offsets: they are the camera mount correction (roadmap 3.4),
the input to the fusion steps below.

Other node parameters ([`go2_lidar_overlay.py`](../scripts/nodes/go2_lidar_overlay.py)):

| Parameter | Default | Meaning |
|---|---|---|
| `cloud_history` | `3` | Registered scans drawn together (one Point-LIO scan is sparse) |
| `min_depth` / `max_depth` | `0.3` / `15.0` | Depth range drawn (m) |
| `point_size` | `2` | Dot size (px) |
| `camera_frame` | `front_camera` | TF frame of the camera (URDF convention: x forward) |
| `frame_is_optical` | `false` | `true` if `camera_frame` is already an optical frame (z forward) |
| `camera_info_file` | — | Use a YAML instead of `/camera/camera_info` |

---

## Simulation

`--sim` tests the whole chain without the robot. The Gazebo camera is mounted
exactly at the URDF `front_camera` pose, and its intrinsics are known exactly
([`config/camera/sim_front_camera.yaml`](../config/camera/sim_front_camera.yaml)),
so the correct overlay needs zero offsets. It can't test the lens calibration
(the sim has no checkerboard), only the projection.

On this VM, headless Gazebo (`SIM_GUI=0`) renders a **black camera and an
empty LiDAR scan** with the default `ogre2` engine. Use the older engine:

```bash
SIM_GUI=0 SIM_RENDER_ENGINE=ogre ./scripts/runtime/wifi_nav.sh --sim --no-explore
./scripts/tools/lidar_overlay.sh --sim
```

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `no images on /camera/image_raw` | Relay not running or crashing: `docker exec go2-lidar3d tail /tmp/camera_relay.log` |
| `no camera_info` | No `config/camera/front_camera.yaml` yet (calibrate), or the session started before it existed (restart) |
| `no TF to front_camera` | `robot_state_publisher` not up yet, or the session is still starting |
| Overlay frames arrive slowly | The node subscribes with the image publisher's reliability; at 720p over Wi-Fi a few Hz is normal |
| Calibrator never captures | Board not found: more light, a flatter board, `--cols/--rows` count *inner* corners |
| `image size changed` in the calibrator | The relay resolution changed mid-run; restart the calibration |

---

## What comes next

These steps build on a passing overlay:

1. **Fusion for mapping (done: `wifi_3d_nav.sh --fusion`).** The LiDAR is
   projected into the camera as a depth image, and RTAB-Map runs RGB-D +
   scan and colours the 3D map. See [3d_nav.md](3d_nav.md#camera--lidar-fusion---fusion).
2. **Fusion for collisions.** The camera finds what the LiDAR misses or can't
   classify (people, plants, thin or low objects). The LiDAR points inside
   each detection give its range. The result is published as a point cloud
   and added as a second observation source in the Nav2 costmap.
3. **Language-queryable map (VLMaps-style).** RGB, the LiDAR depth image and
   camera poses are logged in the VLMaps dataset format and mapped offline on
   a GPU machine. The Go2 can then be sent to goals such as "the tomato
   plants".
