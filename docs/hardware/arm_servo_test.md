# Arm distance servo test (RealSense → D1)

Hold the D1 tool 0.50 m from an object the dog's RealSense sees:
`yolo_depth_tracker.py` → `/object_distance` → `distance_servo.py` → MoveIt → `moveit_d1_bridge.py`.
`scripts/runtime/arm_servo_test.sh` runs it in three terminals. The plant-inspection mission
(`inspect_plant.py`) does not run, so nothing else competes for MoveIt.

## Run it

0. The RealSense must be plugged into the rear USB-C port of the dog's expansion dock. Check
   it with `./scripts/tools/realsense_host.sh check` (it should list the `/camera` topics).

1. MoveIt, RViz and the bridge on the real arm. The arm moves to the start pose and holds it:

   ```bash
   ARM_JOINT_SIGNS="-1 1 1 1 1 -1" ./scripts/runtime/arm_servo_test.sh
   ```

   Once the arm is there, it prints the commands for 2) and 3). `--mock` runs the same on the
   mock arm (nothing moves), and `--no-rviz` / `--robot IP` are passed on to `arm_moveit.sh`.

2. The tracker:

   ```bash
   ./scripts/runtime/arm_servo_test.sh tracker -p target_class:=bottle -p show:=true
   ```

   Or, instead of 2), the 3D detector from the test branch (YOLO11s with 3D boxes, in `go2-lidar3d`):

   ```bash
   ./scripts/runtime/arm_servo_test.sh detector --class bottle --rviz
   ```

   It runs `plant_detection.sh --compressed` with `bottle` as the only class, plus
   `scripts/nodes/detections_to_distance.py`, which sends the nearest `bottle` box's depth (z in
   the camera frame) as `/object_distance` on domain 78. `--raw` uses the raw images (too much for
   the Wi-Fi). Anything after `--` goes to the detector. The first run installs its packages into
   `go2-lidar3d` (`detection_setup.sh`, about 1 GB, a few minutes).

3. The servo, planning only first, then for real:

   ```bash
   ./scripts/runtime/arm_servo_test.sh servo -p dry_run:=true
   ```

   ```bash
   ./scripts/runtime/arm_servo_test.sh servo
   ```

Stop them in the order 3), 2), 1). The arm stays enabled and holding; fold it with
`./scripts/runtime/start_arm.sh fold`.

## The camera on this computer instead of the dog (`--local`)

To test the logic without the dog, plug the D435i into a USB 3 port (blue) on this computer:

```bash
./scripts/runtime/arm_servo_test.sh camera
```

```bash
./scripts/runtime/arm_servo_test.sh detector --local --class bottle --rviz
```

Then run 1) (the arm) and 3) (the servo) as usual. `camera` runs ROS Humble's
`realsense2_camera` in `go2-realsense-host:local`, which `realsense/host/Dockerfile` builds on the
first run, with the robot driver's topic names (`/camera/color/image_raw`,
`/camera/aligned_depth_to_color/image_raw`, `/camera/color/camera_info`). With `--local`,
the camera's domain 64 runs on loopback like MoveIt's 78, the detector takes the raw images,
and nothing reaches the dog. `tracker --local` works the same way. The profile is
`SERVO_CAMERA_PROFILE` (default `640,480,15`).

Checked 2026-11-18:

- **Camera:** found on `USB type: 3.2`. Colour, aligned depth and `camera_info` arrived over
  loopback at 15.0 Hz.
- **Detector:** YOLO11s produced 118 detection arrays in 10 s.
- **Relay:** it sent `person 0.47 at 1.090 m` as `/object_distance`.

The real arm wasn't running for that check.

## Start pose

`SERVO_START_JOINTS`, default `0 0.25 0.30 0 -0.55 0` rad (Joint1..Joint6). This puts the
flange 0.34 m ahead of the arm base and 0.32 m up, level, with the tool (`tool0`, between
the fingertips) at x = 0.44 m. The servo's travel is 0.10 m back and 0.15 m forward:

- forward to 0.59 m: reachable;
- past about 0.61 m the Cartesian path plans only 20 %, and the step is dropped.

Joint2 at 90° can't be used as a start pose. It lays the upper arm flat forward and puts the
flange 0.15 m below the arm base, pointing down into the dog's back. It is also Joint2's
limit (±1.57).

"Ahead" is the model's +X. The real arm faces the dog's rear (MoveIt keeps the forward model),
so the tool moves toward the dog's tail.

## Where each part runs

| Part | Runs | DDS |
|---|---|---|
| MoveIt, RViz, `moveit_d1_bridge.py`, `distance_servo.py` | this computer | domain 78, `ROS_LOCALHOST_ONLY=1`, FastDDS |
| Camera (`go2-realsense` service) | the dog's Jetson | domain 64, CycloneDDS, sent to this computer's Wi-Fi address |
| `yolo_depth_tracker.py` | `go2-3d:local` container (YOLO, CycloneDDS) | 64 in, 78 out |
| or `plant_detection.sh` + `detections_to_distance.py` | `go2-lidar3d` (YOLO11s detector, 3D boxes) | 64 in, 78 out |

The tracker uses one Cyclone config for both domains: domain 64 on the Wi-Fi with the robot as
a peer, and domain 78 on loopback. The host's localhost-only FastDDS on 78 talks to that.
Checked 2026-11-18: a Float32 published from `go2-3d` on domain 78 arrived in a host
`ros2 topic echo`. The YOLO weights are cached in `~/.cache/go2-yolo` (`SERVO_MODEL_DIR`).

## The detector and its relay (2026-11-18)

- **Setup:** `detection_setup.sh` installed vision_msgs, cv_bridge, CPU PyTorch 2.5.1 and
  ultralytics 8.4.215 into the recreated `go2-lidar3d`.
- **Detector:** YOLO11s loaded from `/ws/.torch/hub`, then subscribed to the decompressed
  colour and depth.
- **Relay:** with a synthetic `/detections_3d` on domain 64 (a `bottle` at z = 0.62 m and a
  closer `person` at 0.40 m), the host's domain 78 got `/object_distance` 0.62. It sent 11
  distances for 11 arrays.
- **Not run on real images:** the camera's colour stream stalls on its USB 2.1 link (`uvc
  streamer watchdog`), so no colour+depth pair reached either detector. Reseat the camera in the
  rear USB-C port, flip the plug, or use a USB 3 cable, then check
  `./scripts/tools/realsense_host.sh check` before trying again.

## What the loop can show

The camera is fixed to the dog's body, so moving the arm doesn't change the measured
distance. With the object too far away, the servo steps forward until it reaches its
0.15 m limit and stops there. To see it follow, move the object (or the dog) by hand.

## Checked on the mock arm (2026-11-18)

- **Start pose:** `arm_servo_test.sh --mock --no-rviz` reached it (`arm_go_joints.py`:
  "there"), and the instructions printed.
- **Servo:** with a fake 0.70 m published from `go2-3d` on domain 78, it planned +2 cm steps
  (dry run), then executed them up to the forward limit.
- **Tracker:** it started, downloaded `yolo11s.pt` and reached both domains. No camera was
  attached, so it saw no images, and the closed loop on real images is not tested yet.
