# Development

[← Back to README](../../README.md) · [Scripts](scripts.md) · [Status](../STATUS.md) · [Roadmap](../roadmap.md)

Repository layout, lint hooks, archived code and external projects.

---

## Repository Structure

> Code blocks can't contain clickable links. Use the [Documentation](../../README.md#documentation)
> section above, or browse the folders directly on GitHub.

```text
Unitree_GO2_Agricultural/
├── README.md
├── LICENSE
├── .gitignore
├── .gitattributes
├── .clang-format                     # C++ style for the RViz plugin
├── .pre-commit-config.yaml           # ruff, shellcheck, clang-format, markdownlint
├── config/
│   ├── README.md
│   ├── camera/                       # front_camera.yaml (calibration), sim_front_camera.yaml
│   ├── nav2/go2_omni.yaml            # --omni: MPPI Omni + omni lattice settings
│   ├── foxglove_layout.json          # Foxglove layout (3D twin + camera), --foxglove
│   ├── tour.rviz                     # RViz layout (camera, path, costmap, waypoint panel)
│   └── semantic_locations.json       # named points from --save-location (created on first save)
├── docs/
│   ├── STATUS.md                     # current state, current goals, robot test checklist
│   ├── roadmap.md                    # milestones
│   ├── architecture.md               # scope, data flow, network / DDS
│   ├── troubleshooting.md            # problems, fixes, collision tuning
│   ├── setup/
│   │   ├── setup.md                  # one-time setup, everyday commands, flags
│   │   ├── scripts.md                # scripts reference
│   │   └── development.md            # this file
│   ├── navigation/
│   │   ├── explorer.md               # frontier explorer: run, goal choice, parameters, fixes, results
│   │   ├── drive_modes.md            # --drive hybrid | diff | omni, stuck recovery
│   │   ├── locomotion_comparison.md  # drive-mode trials (sim), how to test on the dog
│   │   ├── drift-and-shudder.md      # pose drift / shudder investigation
│   │   ├── tour.md                   # waypoint mode setup
│   │   └── LAB-NAV-FIXES.md          # lab fixes for upstream go2-nav2-wifi
│   ├── ai_and_vision/
│   │   ├── 3d_nav.md                 # 3D digital twin (RTAB-Map, web dashboard, AI, survey, --resume)
│   │   ├── camera.md                 # live camera setup
│   │   └── camera_lidar_fusion.md    # camera calibration + LiDAR overlay check, --fusion
│   ├── hardware/
│   │   └── d1arm.md                  # D1 arm runbook: direct control, Wi-Fi, Teach & Playback, MoveIt 2
│   ├── simulation/
│   │   ├── gazebo.md                 # Gazebo Harmonic: Go2 + D1
│   │   └── unity.md                  # CMU Unity sim: explorer + Nav2
│   └── images/                       # figures used by the docs
├── scripts/
│   ├── runtime/                      # shell entry points
│   │   ├── wifi_nav.sh               # main orchestrator
│   │   ├── wifi_3d_nav.sh            # the one command: wifi_nav.sh + 3D twin + AI + autonomous survey, --manual, --resume
│   │   ├── wifi_mapping.sh           # mapping-only wrapper
│   │   ├── start_rtabmap.sh          # older manual RTAB-Map launcher
│   │   ├── start_arm.sh              # D1-T arm entry point (preflight, fold, control)
│   │   ├── arm_moveit.sh             # MoveIt 2 for the arm on mock hardware, --inspect demo
│   │   ├── sim_explore.sh            # CMU Unity sim + go2_nav2 + explorer, on the host
│   │   └── go2_net.sh                # shared IP detection / robot discovery
│   ├── nodes/                        # ROS 2 Python nodes
│   │   ├── frontier_explorer.py      # custom explorer
│   │   ├── go2_camera_relay.py       # camera bridge
│   │   ├── go2_floor_filter.py       # floor-plane filter
│   │   ├── go2_click_to_goal.py      # dashboard click -> Nav2 goal
│   │   ├── go2_camera_info.py        # camera_info for the Go2 camera
│   │   ├── battery_manager.py        # AI (default): low battery -> dock
│   │   ├── vlm_reasoning_node.py     # AI (default): Ollama llava -> /vlm_decision; BLOCKED -> dock
│   │   ├── auto_localize.py          # AI (default)/--resume: /initialpose at the dock
│   │   ├── save_location.py          # --save-location: named points
│   │   ├── d1_arm_control.py         # D1-T arm control (chain CLI, lift, sweep)
│   │   ├── d1_arm_calibration.py     # D1-T folded-pose calibration
│   │   └── direct_test_movement.py   # small safe arm movement test
│   ├── robot/                        # run ON the robot (copied by wifi_nav.sh)
│   │   ├── go2_camera_wifi.{sh,py}   # front camera JPEG -> Wi-Fi DDS
│   │   ├── arm_bridge_wifi.{sh,py}   # D1 arm on the Go2 <-> laptop over TCP
│   │   └── unitree_arm/              # ArmString msg package the bridge is built with
│   └── tools/                        # diagnostics and tests
│       ├── auto_walk.sh
│       ├── test_goal_cli.sh
│       ├── test_explore.sh
│       ├── preflight.sh
│       ├── campus_session.sh
│       ├── tune_costmap.sh
│       ├── verify_costmap.sh
│       ├── save_rviz_config.sh
│       ├── export_tour.py
│       ├── add-yolo-costmap-source.sh # YOLO obstacles -> Nav2 costmap (automatic unless --no-ai)
│       ├── apply-lab-nav-fixes.sh    # one-time upstream go2-nav2-wifi fixes
│       ├── diag_arm.sh               # D1 arm diagnostic
│       └── calibrate_arm.py          # D1 arm per-joint offset calibration
├── gazebo/                           # Gazebo simulation (Go2 + D1), see docs/simulation/gazebo.md
│   ├── setup_gazebo.sh / run_sim.sh / test_sim.sh
│   ├── docker/Dockerfile             # go2-sim:local (Humble + Gazebo Harmonic)
│   ├── sim/                          # model builder, robot bridge, D1 emulator
│   ├── worlds/agri_field.sdf
│   └── models/                       # downloaded Unitree models (not in git)
├── twin/                             # go2-3d container for wifi_3d_nav.sh
│   ├── docker/Dockerfile             # go2-3d:local (Humble + RTAB-Map + rosbridge + CPU PyTorch + YOLO11s + dashboard build)
│   ├── start_inside.sh               # starts rosbridge, dashboard server, click-to-goal, rooms, survey, camera TF/info, RTAB-Map, YOLO, VLM; saves DB on stop
│   ├── yolo_node.py                  # YOLO11s detections -> /yolo/* and 3D obstacles
│   ├── image_compressor.py           # JPEG camera copies for the dashboard
│   ├── room_segmentation_node.py     # costmap -> rooms, names/merges -> config/semantic_rooms.json
│   ├── survey_manager.py             # autonomous survey: explore, rooms, visit + name each, report, dock
│   └── webapp/                       # web dashboard: React + three.js (Vite), full-screen map + glass UI, :3000
│       ├── src/Go2Model.jsx          # Unitree Go2 model, leg joints from /joint_states
│       ├── src/ros/cloudWorker.js    # point cloud decoding + voxels in a Web Worker
│       ├── src/ros/quality.js        # performance mode (software renderer detection)
│       └── public/go2/               # decimated go2_description meshes (build_go2_model.py)
├── src/
│   ├── go2_explore_panel/            # RViz panel with Auto Explore toggle
│   ├── d1_moveit_config/             # MoveIt 2 for the D1 on the Go2 (arm_moveit.sh), inspect_plant.py
│   ├── go2_sim_bridge/               # CMU Unity sim adapter + launch (sim_explore.sh)
│   └── go2_vlm/                      # VLM answers -> Nav2 keepout mask + speed limit
└── cmu-legacy/                       # archived CMU-stack code, not used
```

Folders: [`config/`](../../config/) · [`docs/`](../) · [`scripts/`](../../scripts/) · [`twin/`](../../twin/) · [`src/go2_explore_panel/`](../../src/go2_explore_panel/) · [`cmu-legacy/`](../../cmu-legacy/)

---

## Lint and format

Lint and format checks run automatically on `git commit` once pre-commit is
installed:

```bash
pip install pre-commit
pre-commit install
pre-commit run --all-files    # first-time baseline
```

Hooks cover: Python (ruff), shell (shellcheck), C++ (clang-format), Markdown
(markdownlint), and general hygiene (trailing whitespace, EOF newline, large
files). To skip once: `git commit --no-verify`.

Session notes live in [docs/STATUS.md](../STATUS.md).

The Markdown rules are in [`.markdownlint.jsonc`](../../.markdownlint.jsonc).
Lint the docs by hand with:

```bash
npx markdownlint-cli2 README.md "docs/*.md"
```

---

## Legacy

Earlier work based on the CMU `autonomy_stack_go2` running over Ethernet or
the Unity simulation is archived under [`cmu-legacy/`](../../cmu-legacy/) for reference. It is not
part of the active pipeline. See [`cmu-legacy/README.md`](../../cmu-legacy/README.md).

---

## External Dependencies

| Project | Purpose |
|---|---|
| [`dancher00/go2-nav2-wifi`](https://github.com/dancher00/go2-nav2-wifi) | SLAM (Point-LIO) + Nav2 + Wi-Fi relay |
| [RTAB-Map](https://github.com/introlab/rtabmap_ros) | Dense 3D mapping in the `go2-3d` container (`ros-humble-rtabmap-ros`) |
| [three.js](https://threejs.org) / [React Three Fiber](https://github.com/pmndrs/react-three-fiber) / [roslib](https://github.com/RobotWebTools/roslibjs) / [rosbridge](https://github.com/RobotWebTools/rosbridge_suite) | Web dashboard (`twin/webapp`) |
| [Foxglove](https://foxglove.dev) / [`foxglove_bridge`](https://github.com/foxglove/ros-foxglove-bridge) | Alternative 3D visualization (`--foxglove`); [Lichtblick](https://github.com/lichtblick-suite/lichtblick) works too |
| [`cyclonedds`](https://pypi.org/project/cyclonedds/) | Python DDS bindings used by the D1-T arm scripts (`python3 -m pip install --user cyclonedds` on the host) |
| [`JeewanthaSadaruwan/unitree-D1-550-Robot-ARM`](https://github.com/JeewanthaSadaruwan/unitree-D1-550-Robot-ARM) | Reference only: ideas behind an earlier smooth-motion version of the arm control (reimplemented, no code copied) |
