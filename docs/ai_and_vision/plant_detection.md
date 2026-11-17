# Plant detection on the arm camera

[← README](../../README.md) · [Docs index](../README.md) · [RealSense](realsense.md) · [D1 arm](../hardware/d1_arm.md) · [Gazebo](../simulation/gazebo.md) · [Status](../status.md)

YOLO detections with 3D bounding boxes from the RealSense D435i that replaces
the D1's gripper. One node, one parameter file, the same topics on the real
Go2 and in Gazebo:

| Piece | What |
|---|---|
| [`src/object_detection_rgbd`](../../src/object_detection_rgbd) | The detector: YOLOv5 on the colour image, then the aligned depth inside each 2D box gives a 3D box. Adapted from [IliaLarchenko/object_detection_rgbd](https://github.com/IliaLarchenko/object_detection_rgbd) (Apache-2.0) |
| [`config/detection/object_detection.yaml`](../../config/detection/object_detection.yaml) | Topics, model, confidence threshold, classes to keep |
| [`scripts/runtime/plant_detection.sh`](../../scripts/runtime/plant_detection.sh) | Runs the detector **inside `go2-lidar3d`**. Needs nothing on the host but Docker |
| [`scripts/tools/detection_setup.sh`](../../scripts/tools/detection_setup.sh) | Installs what the detector needs into `go2-lidar3d` (called by the script above) |
| [`scripts/runtime/plant_detection_host.sh`](../../scripts/runtime/plant_detection_host.sh) | Optional: the same detector in a Python environment on the workstation |
| [`gazebo/worlds/plant_inspection.sdf`](../../gazebo/worlds/plant_inspection.sdf) | Gazebo world with potted plants on a bench |

## Topics

Input, published by the D435i boot service on the Go2 ([realsense.md](realsense.md))
and by the simulation's wrist camera:

| Topic | Type |
|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image`, rgb8 |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image`: 16UC1 millimetres on the robot, 32FC1 metres in Gazebo (the node takes both) |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo`, frame `camera_color_optical_frame` |

Output:

| Topic | Type | Content |
|---|---|---|
| `/annotated_image` | `sensor_msgs/Image` | Colour image with the 2D boxes, class and confidence |
| `/detected_objects` | `vision_msgs/Detection2DArray` | 2D boxes in pixels |
| `/detections_3d` | `vision_msgs/Detection3DArray` | Per object: box centre, size, class, confidence. Published every frame, empty when nothing is seen |
| `/detected_objects_3d` | `visualization_msgs/MarkerArray` | The same boxes as wireframes for RViz |
| `/detected_objects_pointcloud` | `sensor_msgs/PointCloud2` | The depth points counted as the object |

The 3D boxes are axis-aligned in **`camera_color_optical_frame`** (z out of the
lens, x right, y down). Nothing publishes the transform from that frame to the
robot base yet: the arm's TF is on MoveIt's own ROS domain (66). Turning a box
into a pose for the arm is part of the arm-control work.

## Run it in `go2-lidar3d` (standard)

A session must be running, so that the container exists: `wifi_nav.sh` for the
robot, or the simulation below.

```bash
./scripts/runtime/plant_detection.sh --rviz
```

The first run installs the dependencies into the container (needs internet,
a few minutes, about 1 GB):

| Where | What | Survives a recreated container |
|---|---|---|
| apt, in the container | `ros-humble-vision-msgs`, `ros-humble-cv-bridge`, `ros-humble-image-transport-plugins`, `python3-pip` | no (about a minute to redo) |
| `/ws/.detect_pydeps` | CPU PyTorch 2.5.1, torchvision, the Python packages the YOLOv5 code imports, numpy 1.x | yes (`/ws` is `go2-nav2-wifi/ws` on the host) |
| `/ws/.torch/hub` | the YOLOv5 code (`ultralytics/yolov5`) and `yolov5s.pt` | yes |

Only the detector process puts `/ws/.detect_pydeps` on its `PYTHONPATH`; Nav2
and the rest of the container keep their own numpy. After the first run no
internet is needed, which matters on the robot's Wi-Fi network.

| Flag | Effect |
|---|---|
| `--rviz` | RViz in the container with the annotated image, the 3D boxes and the object points ([`config/plant_detection.rviz`](../../config/plant_detection.rviz), fixed frame `camera_color_optical_frame`) |
| `--compressed` | Real robot over Wi-Fi: subscribe to the JPEG colour and PNG depth copies and decompress them in the container (raw images do not fit the link, see [realsense.md](realsense.md)) |
| `--setup-only` | Install the dependencies and exit |
| `--stop` | Stop a detector left running |
| `-- <ros args>` | Passed to the node, e.g. `-- -p confidence_threshold:=0.3` |

Check the output from a ROS shell:

```bash
./scripts/tools/ros_shell.sh go2-lidar3d ros2 topic echo /detections_3d --once
```

## Run it on the workstation (optional)

For a workstation that already has a Python virtual environment with PyTorch.
It installs nothing: when the environment or CycloneDDS is missing it stops and
points to the container script.

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp      # once
./scripts/runtime/plant_detection_host.sh --rviz
```

It needs ROS 2 Humble on the host with `vision_msgs` and `cv_bridge`, and the
environment at `~/venvs/odrgbd` (`DETECT_VENV=...` for another path), created
with `--system-site-packages` so it sees ROS. The DDS settings (domain 64,
CycloneDDS) are copied from the running `go2-lidar3d` or `go2-sim` container.

## In Gazebo

The MoveIt arm of the simulation ([hydroponic_trial.md](../simulation/hydroponic_trial.md))
carries a camera instead of the gripper. `SIM_WRIST_CAMERA=d435i` makes it a
D435i and the simulation republishes it under the real driver's topic names:

| | `d405` (default) | `d435i` |
|---|---|---|
| Body | 23 × 42 × 42 mm | 25 × 90 × 25 mm |
| Image | 424 × 240, 5 Hz | 640 × 480, 10 Hz |
| Horizontal field of view | 87° | 69° (the D435i's colour camera) |
| Range | 0.07 to 3 m | 0.1 to 10 m |

One-time setup (downloads the robot and plant models, builds two images):

```bash
./gazebo/setup_gazebo.sh
./gazebo/setup_gazebo.sh --moveit
./gazebo/setup_gazebo.sh --plants
```

Start the simulation with the plant world, then the detector:

```bash
SIM_MOVEIT=1 SIM_ARM=0 SIM_WRIST_CAMERA=d435i SIM_WORLD=plant_inspection ./gazebo/run_sim.sh up
./scripts/runtime/plant_detection.sh --rviz
```

`plant_inspection` is a small room with a bench of five potted plants behind
the robot's default spawn. The D1 is mounted facing the tail, so with the arm
at MoveIt's `home` pose the camera looks at the middle three plants from about
1.7 m. [`plant_inspection.json`](../../gazebo/worlds/plant_inspection.json)
lists where each plant is. The foliage models are 3DGEMS plants (GPL-3.0, the
collection the lab furniture comes from), downloaded, not in git. They are
untextured, yet YOLOv5s labels them `potted plant` at 0.65 to 0.88 from the
home pose. Regenerate the world with
`python3 gazebo/worlds/build_plant_inspection_world.py`.

Checked on 2026-11-17 (4-core VM, software rendering): from the home pose all
three plants in view were detected, at 1.66 to 1.71 m, matching the bench's
place in the world; turning `Joint1` by 0.35 rad moved the view and the boxes
with it. The simulation ran at 0.14× real time there and the detector put out
about 0.6 frames per second; a machine with a GPU is much faster.

The arm is moved with MoveIt on domain 66, inside the simulation container
([d1_arm.md](../hardware/d1_arm.md), [hydroponic_trial.md](../simulation/hydroponic_trial.md)).

What differs from the robot: the depth image is float metres and has no noise
or holes, colour and depth come from one sensor (so they are registered
exactly), and there are no compressed copies.

## On the robot

With the D435i boot service running ([realsense.md](realsense.md)) and a
session up:

```bash
./scripts/tools/realsense_host.sh check          # the /camera topics arrive
./scripts/runtime/plant_detection.sh --compressed --rviz
```

## What the model detects

`yolov5s` is trained on COCO: 80 everyday classes, among them `person` and
`potted plant`. `filter_classes` in the parameter file keeps those two. Plants
without a pot, such as heads growing out of a hydroponic channel, are not a
COCO class and need a model trained on your own images:

1. Record the colour stream at the station (`ros2 bag record /camera/color/image_raw`),
   extract a few hundred frames at different distances, angles, lighting and
   growth stages, plus some with no plants.
2. Label one class, `plant`, in Roboflow, CVAT or Label Studio; export in
   YOLOv5 format, about 80 % train and 20 % validation.
3. Train from `yolov5s.pt` in a clone of `ultralytics/yolov5` (on a GPU or Colab):
   `python train.py --img 640 --batch 16 --epochs 100 --data plants.yaml --weights yolov5s.pt`
4. Set `weights_path` to the resulting `best.pt` and `filter_classes: ["plant"]`.
   Class names are read from the weights file. For the container, put the file
   under `go2-nav2-wifi/ws/` and give its `/ws/...` path.

## How the 3D box is made

For each 2D box the node:

1. finds the object's depth: the median of the **nearest** 0.3 m deep slab
   (`depth_window`) that holds at least 10 % of the box's valid depth pixels
   (`depth_min_fraction`);
2. keeps the pixels within `depth_window` of that depth and takes their largest
   connected region as the object;
3. turns those pixels into 3D points with the camera intrinsics, and spans the
   box from the 2nd to the 98th percentile of the points on each axis
   (`extent_percentile`), so stray pixels do not stretch it.

Upstream took the *most common* depth in the box instead. Through a plant's
leaves most of the box is the background, so every plant box snapped to the
wall behind it (in the simulation: 2.56 m away and 0 m deep, instead of
1.7 m). `depth_min_fraction: 0` restores the old behaviour.

Something at nearly the same distance as the object and touching it in the
image joins its box: in the simulation the bench top makes the plant boxes
wider than the plants.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `container go2-lidar3d is not running` | Start a session first (`wifi_nav.sh`, or `wifi_nav.sh --sim`) |
| Detector starts, no `/annotated_image` | No colour and depth pair arrives. `./scripts/tools/ros_shell.sh go2-lidar3d ros2 topic hz /camera/color/image_raw`. In Gazebo: `SIM_MOVEIT=1` and `SIM_WRIST_CAMERA=d435i` set? On the robot over Wi-Fi: use `--compressed` |
| Image but no boxes | Nothing above `confidence_threshold`, or the class is not in `filter_classes`. Try `-- -p confidence_threshold:=0.25` |
| 2D boxes but empty `/detections_3d` | No valid depth inside the box (too close: under 0.28 m for the real D435i) |
| RViz shows nothing in 3D | Fixed frame must be `camera_color_optical_frame` |
| Host script: `CycloneDDS is not installed` | `sudo apt install ros-humble-rmw-cyclonedds-cpp`, or use `plant_detection.sh` |
| pip step fails in the container | Needs internet. Rerun, or `./scripts/tools/detection_setup.sh --force` |
