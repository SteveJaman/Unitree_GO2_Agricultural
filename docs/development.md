# Development

[← Back to README](../README.md) · [Scripts](scripts.md) · [Status](STATUS.md) · [Roadmap](roadmap.md)

Repository layout, lint hooks, archived code and external projects.

---

## Repository Structure

> Code blocks can't contain clickable links. Use the [Documentation](../README.md#documentation)
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
│   ├── tour.rviz                     # RViz layout (camera, path, costmap, waypoint panel)
│   └── semantic_locations.json       # named points from --save-location (created on first save)
├── docs/
│   ├── setup.md                      # one-time setup, everyday commands, flags
│   ├── architecture.md               # scope, data flow, network / DDS
│   ├── 3d_nav.md                     # 3D digital twin (RTAB-Map, web dashboard, --ai, --resume)
│   ├── explorer.md                   # frontier explorer
│   ├── scripts.md                    # scripts reference
│   ├── d1arm.md                      # D1 arm runbook
│   ├── camera.md                     # live camera setup
│   ├── tour.md                       # waypoint mode setup
│   ├── troubleshooting.md            # problems, fixes, collision tuning
│   ├── LAB-NAV-FIXES.md              # lab fixes for upstream go2-nav2-wifi
│   ├── development.md                # this file
│   ├── STATUS.md                     # current state, robot test checklist
│   └── roadmap.md                    # milestones
├── scripts/
│   ├── runtime/                      # shell entry points
│   │   ├── wifi_nav.sh               # main orchestrator
│   │   ├── wifi_3d_nav.sh            # wifi_nav.sh + RTAB-Map + web dashboard (3D twin), --ai, --resume
│   │   ├── wifi_mapping.sh           # mapping-only wrapper
│   │   ├── start_rtabmap.sh          # older manual RTAB-Map launcher
│   │   ├── start_arm.sh              # D1-T arm entry point (preflight, fold, control)
│   │   └── go2_net.sh                # shared IP detection / robot discovery
│   ├── nodes/                        # ROS 2 Python nodes
│   │   ├── frontier_explorer.py      # custom explorer
│   │   ├── go2_camera_relay.py       # camera bridge
│   │   ├── go2_floor_filter.py       # floor-plane filter
│   │   ├── go2_click_to_goal.py      # dashboard click -> Nav2 goal
│   │   ├── go2_camera_info.py        # camera_info for the Go2 camera
│   │   ├── battery_manager.py        # --ai: low battery -> dock
│   │   ├── vlm_reasoning_node.py     # --ai: Ollama llava -> /vlm_decision; BLOCKED -> dock
│   │   ├── auto_localize.py          # --ai/--resume: /initialpose at the dock
│   │   ├── save_location.py          # --save-location: named points
│   │   ├── d1_arm_control.py         # D1-T arm control (chain CLI, lift, sweep)
│   │   ├── d1_arm_calibration.py     # D1-T folded-pose calibration
│   │   └── direct_test_movement.py   # small safe arm movement test
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
│       ├── add-yolo-costmap-source.sh # YOLO obstacles -> Nav2 costmap (automatic with --ai)
│       ├── apply-lab-nav-fixes.sh    # one-time upstream go2-nav2-wifi fixes
│       ├── diag_arm.sh               # D1 arm diagnostic
│       └── calibrate_arm.py          # D1 arm per-joint offset calibration
├── gazebo/                           # Gazebo simulation (Go2 + D1), see gazebo/README.md
│   ├── setup_gazebo.sh / run_sim.sh / test_sim.sh
│   ├── docker/Dockerfile             # go2-sim:local (Humble + Gazebo Harmonic)
│   ├── sim/                          # model builder, robot bridge, D1 emulator
│   ├── worlds/agri_field.sdf
│   └── models/                       # downloaded Unitree models (not in git)
├── twin/                             # go2-3d container for wifi_3d_nav.sh
│   ├── docker/Dockerfile             # go2-3d:local (Humble + RTAB-Map + rosbridge + CPU PyTorch + YOLO11s + dashboard build)
│   ├── start_inside.sh               # starts rosbridge, dashboard server, click-to-goal, camera TF/info, RTAB-Map, YOLO, VLM; saves DB on stop
│   ├── yolo_node.py                  # YOLO11s detections -> /yolo/* and 3D obstacles
│   ├── image_compressor.py           # JPEG camera copies for the dashboard
│   ├── room_segmentation_node.py     # costmap -> rooms, names/merges -> config/semantic_rooms.json
│   └── webapp/                       # web dashboard: React + three.js (Vite), full-screen map + glass UI, :3000
├── src/
│   └── go2_explore_panel/            # RViz panel with Auto Explore toggle
└── cmu-legacy/                       # archived CMU-stack code, not used
```

Folders: [`config/`](../config/) · [`docs/`](./) · [`scripts/`](../scripts/) · [`twin/`](../twin/) · [`src/go2_explore_panel/`](../src/go2_explore_panel/) · [`cmu-legacy/`](../cmu-legacy/)

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

Session notes live in [docs/STATUS.md](STATUS.md).

The Markdown rules are in [`.markdownlint.jsonc`](../.markdownlint.jsonc).
Lint the docs by hand with:

```bash
npx markdownlint-cli2 README.md "docs/*.md"
```

---

## Legacy

Earlier work based on the CMU `autonomy_stack_go2` running over Ethernet or
the Unity simulation is archived under [`cmu-legacy/`](../cmu-legacy/) for reference. It is not
part of the active pipeline. See [`cmu-legacy/README.md`](../cmu-legacy/README.md).

---

## External Dependencies

| Project | Purpose |
|---|---|
| [`dancher00/go2-nav2-wifi`](https://github.com/dancher00/go2-nav2-wifi) | SLAM (Point-LIO) + Nav2 + Wi-Fi relay |
| [RTAB-Map](https://github.com/introlab/rtabmap_ros) | Dense 3D mapping in the `go2-3d` container (`ros-humble-rtabmap-ros`) |
| [three.js](https://threejs.org) / [React Three Fiber](https://github.com/pmndrs/react-three-fiber) / [roslib](https://github.com/RobotWebTools/roslibjs) / [rosbridge](https://github.com/RobotWebTools/rosbridge_suite) | Web dashboard (`twin/webapp`) |
| [Foxglove](https://foxglove.dev) / [`foxglove_bridge`](https://github.com/foxglove/ros-foxglove-bridge) | Alternative 3D visualization (`--foxglove`); [Lichtblick](https://github.com/lichtblick-suite/lichtblick) works too |
| [`cyclonedds`](https://pypi.org/project/cyclonedds/) | Python DDS bindings used by the D1-T arm scripts (`pip3 install cyclonedds` on the VM) |
| [`JeewanthaSadaruwan/unitree-D1-550-Robot-ARM`](https://github.com/JeewanthaSadaruwan/unitree-D1-550-Robot-ARM) | Reference only: ideas behind an earlier smooth-motion version of the arm control (reimplemented, no code copied) |
