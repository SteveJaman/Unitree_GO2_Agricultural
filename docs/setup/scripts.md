# Scripts Reference

[← README](../../README.md) · [Docs index](../README.md) · [Setup](setup.md) · [Explorer](../navigation/explorer.md) · [3D twin](../ai_and_vision/3d_nav.md)

Every entry point, ROS node and tool in [`scripts/`](../../scripts/). The
go2-3d container's own files (YOLO node, image compressor, web dashboard)
are in [`twin/`](../../twin/), described in [3d_nav.md](../ai_and_vision/3d_nav.md).

---

## Setup ([`scripts/setup.sh`](../../scripts/setup.sh))

Installs or checks, safe to rerun: Docker (+ compose plugin, docker group),
the upstream clone in `~/files/go2-nav2-wifi`, its lab patches
([`apply-lab-nav-fixes.sh`](../../scripts/tools/apply-lab-nav-fixes.sh)), the host
Python packages (cyclonedds, ruckig, ikpy; `--with-extras` adds ultralytics and
pyrealsense2), Ollama with `llava:7b`, and the images `go2-lidar3d:local` (build
only: `wifi_nav.sh` creates the container for the current IPs) and `go2-3d:local`.
Prints `[✓]` for what is there, `[...] Installing` for what it adds, and a list
of anything left to fix (exit 1). `--with-traversability` also applies the
traversability costmap layout
([`add-traversability-layers.sh`](../../scripts/tools/add-traversability-layers.sh)).

## Runtime ([`scripts/runtime/`](../../scripts/runtime/))

| Script | Purpose |
|---|---|
| [`wifi_nav.sh`](../../scripts/runtime/wifi_nav.sh) | **Main entry point.** Relay + Nav2 + RViz + explorer + camera; `--arm` also folds the D1 arm (`--arm-wifi`: arm plugged into the Go2) |
| [`wifi_3d_nav.sh`](../../scripts/runtime/wifi_3d_nav.sh) | **The one command.** Builds or refreshes its image, starts Ollama and pulls the VLM, then by default runs the AI and the unattended survey (`--manual`, `--no-ai` opt out). Runs `wifi_nav.sh` unchanged plus RTAB-Map (camera fusion when the camera publishes), the web dashboard (`:3000`, rosbridge `:9090`) and click-to-goal in a separate `go2-3d` container; `--manual`, `--no-ai`, `--no-build`, `--resume`, `--save-location`, `--foxglove`, `--build`, `--export`, `--stop`; `--voxel` (3D VoxelLayer, [voxel_costmap.md](../navigation/voxel_costmap.md)); `--semantic` / `--hardware` / `--send-gait` (traversability, [traversability_handoff.md](../hardware/traversability_handoff.md)) |
| [`wifi_mapping.sh`](../../scripts/runtime/wifi_mapping.sh) | Mapping-only wrapper (no explorer) |
| [`sim_explore.sh`](../../scripts/runtime/sim_explore.sh) | **Unity simulation.** CMU Unity sim + real go2_nav2 config + explorer, on the host. See [unity.md](../simulation/unity.md). For the Gazebo sim use `wifi_nav.sh --sim` ([docs/simulation/gazebo.md](../simulation/gazebo.md)) |
| [`start_rtabmap.sh`](../../scripts/runtime/start_rtabmap.sh) | Older manual RTAB-Map launcher inside `go2-lidar3d` (superseded by `wifi_3d_nav.sh`) |
| [`start_arm.sh`](../../scripts/runtime/start_arm.sh) | D1-T arm entry point: preflight (address + ping), then `fold`, `read`, `--calibrate`, or any `d1_arm_control.py` command chain |
| [`go2_net.sh`](../../scripts/runtime/go2_net.sh) | Sourced library: host-IP detection, `find_go2_ip()` robot discovery, interface safeguards |
| [`session_keepalive.sh`](../../scripts/runtime/session_keepalive.sh) | Sourced by go2-nav2-wifi `mapping.sh` (via `wifi_nav.sh`, `GO2_KEEP_SESSION=0` = off): when the SLAM or motion link to the dog ends, halts the robot and prints the cause and the fix instead of closing the whole session |

## Nodes ([`scripts/nodes/`](../../scripts/nodes/))

| File | Purpose |
|---|---|
| [`frontier_explorer.py`](../../scripts/nodes/frontier_explorer.py) | Three-phase explorer: blind lead-in → Yamauchi frontier → visited-cell coverage → return. Optional spin-360 capture for tours |
| [`go2_camera_relay.py`](../../scripts/nodes/go2_camera_relay.py) | Robot JPEG frames (`/go2/front_camera/compressed`) → `/camera/image_raw` (bgr8) + `/camera/camera_info` |
| [`scripts/robot/arm_bridge_wifi.sh`](../../scripts/robot/arm_bridge_wifi.sh) | **Runs on the robot** (copied and started by `wifi_nav.sh --arm-wifi`): D1 arm plugged into the Go2 ↔ laptop over TCP; `--build` once |
| [`scripts/robot/go2_camera_wifi.sh`](../../scripts/robot/go2_camera_wifi.sh) | **Runs on the robot** (copied and started by `wifi_nav.sh`): front camera JPEG → Wi-Fi DDS |
| [`go2_floor_filter.py`](../../scripts/nodes/go2_floor_filter.py) | Floor-plane filter, prevents floor being treated as an obstacle |
| [`go2_stuck_recovery.py`](../../scripts/nodes/go2_stuck_recovery.py) | Every session: on an aborted leg (e.g. facing a wall), step back (or aside, with `--drive hybrid` or `omni`) into free space and retry the goal. Also when the planner refuses goals because the dog stands in an obstacle (pressed to a wall; at most 3 times a minute) |
| [`go2_link_watch.py`](../../scripts/debug/go2_link_watch.py) | Every real-robot session (`GO2_LINK_WATCH=0` = off): says in the terminal when the dog's position is not reaching this computer (none within 20 s, or a stall of 5 s and more) and when it comes back. Listens to the local `/tf` only, so it adds no Wi-Fi traffic |
| [`go2_teleop.py`](../../scripts/nodes/go2_teleop.py) | Dashboard Drive pad → `/cmd_vel` at Nav2's limits, with ramps and a deadman; pauses the explorer and cancels Nav2 goals while driving by hand |
| [`go2_camera_obstacles.py`](../../scripts/nodes/go2_camera_obstacles.py) | `--camera-obstacles`: Depth Anything V2-Small depth, scaled by the LiDAR → `/camera/obstacle_points` for the Nav2 costmap |
| [`go2_fusion_camera.py`](../../scripts/nodes/go2_fusion_camera.py) | `--fusion`: undistorted camera + LiDAR depth image + camera_info with one stamp, for RTAB-Map RGB-D |
| [`go2_lidar_overlay.py`](../../scripts/nodes/go2_lidar_overlay.py) | Projects `/lidar3d/registered` onto the front camera → `/camera/lidar_overlay` (camera–LiDAR alignment check, live mount tuning) |
| [`go2_camera_info.py`](../../scripts/nodes/go2_camera_info.py) | `/camera/camera_info` for a camera without one (pinhole from a field of view, or fx/fy/cx/cy); skipped when the calibrated relay publishes it |
| [`battery_manager.py`](../../scripts/nodes/battery_manager.py) | Always: `/battery_state` for the dashboard. By default: low battery (< 15 %) → pause explorer, go to the dock; explorer blocked until 20 % |
| [`vlm_reasoning_node.py`](../../scripts/nodes/vlm_reasoning_node.py) | Default (not `--no-ai`): camera frame → local Ollama `llava:7b` → `/vlm_decision`; confirmed `BLOCKED` → pause explorer + dock |
| [`auto_localize.py`](../../scripts/nodes/auto_localize.py) | Default / `--resume`: one-shot `/initialpose` at the dock (0, 0, 0) |
| [`save_location.py`](../../scripts/nodes/save_location.py) | `--save-location`: label clicked points → `config/semantic_locations.json` |
| [`go2_click_to_goal.py`](../../scripts/nodes/go2_click_to_goal.py) | Dashboard (or Foxglove) `/clicked_point` (any TF frame) → nearest safe floor (BFS on the global costmap, 3×3 clearance) → `/goal_pose`; markers on `/clicked_goal_markers` |
| [`d1_arm_control.py`](../../scripts/nodes/d1_arm_control.py) | D1-T arm chain-style CLI + `D1Arm` class (JSON over CycloneDDS): state, enable, release, home, power, joint(s), feedback, `lift`, `sweep` |
| [`d1_arm_calibration.py`](../../scripts/nodes/d1_arm_calibration.py) | `--read` current angles / `--move` to `FOLDED_ANGLES`, then release and power off |
| [`direct_test_movement.py`](../../scripts/nodes/direct_test_movement.py) | Small, safe movement test for the arm (uses `d1_arm_control.py`) |

### Traversability nodes ([traversability_sim.md](../simulation/traversability_sim.md))

Started by `wifi_3d_nav.sh --semantic` through
[`traversability_nodes.sh`](../../scripts/runtime/traversability_nodes.sh) (in `go2-lidar3d`;
logs `go2-nav2-wifi/ws/log/3d/traversability/`), and by the sim trials
(`tests/traversability/run_all.sh`, `lab_semantic.sh`).

| Node | What it does |
|---|---|
| [`object_box_fusion.py`](../../scripts/nodes/object_box_fusion.py) | Camera detections + LiDAR → world-fixed 3D boxes with a height (`/objects/boxes_json`); unknown, never low, when the points don't explain a detection; `floor_estimation: fixed \| histogram` |
| [`low_object_clusterer.py`](../../scripts/nodes/low_object_clusterer.py) | LiDAR-only low-object clusters (no camera, no labels) → `/objects/low_clusters`, for the gait backstop; finds flat items such as a keyboard (ported from Yassine Salimi's `traversability_core.py`) |
| [`gait_supervisor.py`](../../scripts/nodes/gait_supervisor.py) | Gait from the boxes ahead (`NORMAL` / `HIGH_STEP` / `CAREFUL` / `STOP`), raised by the costmap and cluster backstop and semantic labels; `CAREFUL` when its inputs go stale. `backend: gazebo` or `sport_client` (`FootRaiseHeight`, inert unless `dry_run: false` and `enable_hardware_backend: true`) |
| [`vlm_heuristic_node.py`](../../scripts/nodes/vlm_heuristic_node.py) | VLM (`qwen2.5vl`, Ollama) labels each tracked object `STEP_OVER` / `CAUTION` / `AVOID`, fused with the LiDAR height ([`semantic_labeler.py`](../../scripts/nodes/semantic_labeler.py)) → `/semantic/obstacles` for `SemanticCostmapLayer` |
| [`sim_box_detector.py`](../../scripts/nodes/sim_box_detector.py) | Sim only: ground-truth 2D detections for classes YOLO lacks (a stand-in, not a vision model) |
| [`yolo_json_node.py`](../../scripts/nodes/yolo_json_node.py) | Sim: YOLO11s on the camera → `/objects/detections_json` for the fusion |

## Arm sweeps

| Script | What it does |
|---|---|
| [`orbit_scanner.py`](../../src/d1_active_perception/d1_active_perception/orbit_scanner.py) | `ros2 run d1_active_perception orbit_scanner`: camera sweep around a plant through MoveIt, with a placeholder at each capture point. Full 180° for a low plant beside the dog (13/13 in Gazebo, no contacts), 120° at rack height ([details](../hardware/d1_arm.md#orbit-sweep-d1_active_perception)) |

## Tools ([`scripts/tools/`](../../scripts/tools/))

| Script | Purpose |
|---|---|
| [`auto_walk.sh`](../../scripts/tools/auto_walk.sh) | Scripted 1 m square patrol via `/goal_pose` — proof the motion path works |
| [`test_goal_cli.sh`](../../tests/test_goal_cli.sh) | Publish a single `PoseStamped` goal from the CLI |
| [`sim_benchmark.py`](../../tests/sim_benchmark.py) | Automated exploration benchmark in the Gazebo `lab_office` world: explores up to 5 simulated minutes, records every Gazebo contact between the dog (body, legs, arm) and the furniture or walls, scores 100 − 10 per collision − 5 per room under 90 % mapped, prints `SCORE: X/100` with the cause of each deduction (`--drive`, `--sim-seconds`) |
| [`test_explore.sh`](../../tests/test_explore.sh) | Diagnose the explorer without the RViz button |
| [`ros_shell.sh`](../../scripts/tools/ros_shell.sh) | ROS 2 shell in `go2-lidar3d` (or `go2-sim`) on the stack's DDS settings, so `ros2 topic list` sees its topics; also runs one command |
| [`preflight.sh`](../../scripts/tools/preflight.sh) | Layered verification: container, DDS, packages, nodes, topics, TF |
| [`campus_session.sh`](../../scripts/tools/campus_session.sh) | All-in-one verify → tune → patch → launch → monitor |
| [`tune_costmap.sh`](../../scripts/tools/tune_costmap.sh) | Change inflation radius and footprint padding at runtime (`--safe`, `--wide`, `--tight`, `--show`) |
| [`verify_costmap.sh`](../../scripts/tools/verify_costmap.sh) | Confirm the costmap topic exists and publishes |
| [`save_rviz_config.sh`](../../scripts/tools/save_rviz_config.sh) | Interactive guide to save the RViz layout to [`config/tour.rviz`](../../config/tour.rviz) |
| [`calibrate_camera.sh`](../../scripts/tools/calibrate_camera.sh) | Headless checkerboard calibration of the front camera → `config/camera/front_camera.yaml` (`/camera/camera_info`) |
| [`lidar_overlay.sh`](../../scripts/tools/lidar_overlay.sh) | Run the camera–LiDAR overlay check next to a session (`--sim` for Gazebo) |
| [`rviz_add_camera.py`](../../scripts/tools/rviz_add_camera.py) | Adds Front camera / LiDAR on camera / 3D map displays to the session RViz layouts (run by `wifi_nav.sh`) |
| [`nav2_camera_source.py`](../../scripts/tools/nav2_camera_source.py) | Adds/removes the `camera` obstacle source in go2-nav2-wifi's `lidar3d_planner.yaml` (run by `wifi_nav.sh`); works alongside other sources such as YOLO's |
| [`nav2_voxel_layer.py`](../../scripts/tools/nav2_voxel_layer.py) | `--voxel`: switches the 3D stack's obstacle layer to a Nav2 `VoxelLayer` on the self-filtered LiDAR, and back (run by `wifi_nav.sh`; byte-exact undo). See [voxel_costmap.md](../navigation/voxel_costmap.md) |
| [`add-traversability-layers.sh`](../../scripts/tools/add-traversability-layers.sh) | `--semantic`: height band in the local costmap, semantic layer in both, old global height band removed; edits go2-nav2-wifi's planner YAML, controller launch file and `setup.py` (`--undo`, `--check`; `TARGET_REPO` for a mock) |
| [`world_objects.py`](../../scripts/tools/world_objects.py) | A Gazebo world's object list (`cluttered_items`, `lab_office`) for the sim's ground-truth detector (`--sim --semantic`) |
| [`check_self_points.py`](../../scripts/tools/check_self_points.py) | Pre-session check: counts the robot's own body points in `/lidar3d/registered` and `/lidar3d/obstacle_points` |
| [`nav_compare/`](../../scripts/tools/nav_compare/) | Drive-mode trials: `sim_trial.sh` (one unattended Gazebo run), `record.sh start/stop` (record any session, sim or real dog), `run_analysis.sh` → `analyze.py` (table + plots). See [locomotion_comparison.md](../navigation/locomotion_comparison.md) |
| [`nav2_omni.py`](../../scripts/tools/nav2_omni.py) | Switches go2-nav2-wifi between forward-only RPP and MPPI Omni ([`config/nav2/go2_omni.yaml`](../../config/nav2/go2_omni.yaml); run by `wifi_nav.sh`) |
| [`build_go2_model.py`](../../scripts/tools/build_go2_model.py) | Unitree's `go2_description` URDF + meshes → decimated `twin/webapp/public/go2/` (PLY + `go2.json`) for the dashboard's animated Go2; run once in the `go2-3d` image (command in the file) |
| [`d435i_sender.py`](../../scripts/tools/d435i_sender.py) | RealSense D435i on the machine it is plugged into (Jetson / Go2 PC): colour + aligned depth, JPEG / PNG over ZeroMQ (`pip install pyrealsense2 pyzmq opencv-python numpy`) |
| [`d435i_receiver.py`](../../scripts/tools/d435i_receiver.py) | On the host (ROS 2 Humble): the ZeroMQ stream → `/camera/color/*` and `/camera/aligned_depth_to_color/*`; for `wifi_3d_nav.sh --rgbd` set `TWIN_DEPTH_TOPIC=/camera/aligned_depth_to_color/image_raw` |
| [`export_tour.py`](../../scripts/tools/export_tour.py) | Open3D Poisson mesh from an RTAB-Map PLY (normals, distance trim, decimation); run by `wifi_3d_nav.sh --export --mesh` |
| [`add-yolo-costmap-source.sh`](../../scripts/tools/add-yolo-costmap-source.sh) | Add `/yolo/obstacles` to the Nav2 costmap config in go2-nav2-wifi; run automatically by `wifi_3d_nav.sh` (unless `--no-ai`) (`--undo` restores) |
| [`relax-costmap.sh`](../../scripts/tools/relax-costmap.sh) | Nav2 costmap in go2-nav2-wifi: clear stray LiDAR marks as far as they are marked, faster inflation decay (obstacle heights unchanged); run by `wifi_3d_nav.sh` at start (`--undo` restores) |
| [`go2_hub.py`](../../scripts/runtime/go2_hub.py) | Started by `wifi_3d_nav.sh` (`--no-hub` = off): serves the dashboard on `:3000` from the start, plus its System tab (health, logs, debug report, from `debug_dashboard.py`), Settings tab (live Nav2 values through `go2_param_tool.py`, changes need the session code) and the start-up progress |
| [`boot_filter.awk`](../../scripts/runtime/boot_filter.awk) | `wifi_3d_nav.sh`'s quiet terminal: all output to `ws/log/3d/boot.log`, on screen only progress, warnings with fixes, problems (`--verbose` = everything) |
| [`go2_param_tool.py`](../../scripts/debug/go2_param_tool.py) | Run by the hub inside `go2-lidar3d`: reads or sets several Nav2 parameters in one process (the ros2 CLI takes ~1 s each) |
| [`debug_dashboard.py`](../../scripts/debug/debug_dashboard.py) | Built into the dashboard's System tab with `wifi_3d_nav.sh`; on its own for a plain `wifi_nav.sh` session. Debug page on <http://localhost:3001>: why the dog isn't moving (LiDAR → TF → goal → planner → costmap → controller → smoother → walking), topic health, costmap, ROS errors, all logs. Read-only probe: [`go2_debug_probe.py`](../../scripts/debug/go2_debug_probe.py) |
| [`mapping_stop_timeouts.py`](../../scripts/tools/mapping_stop_timeouts.py) | Run by `wifi_nav.sh`: patches go2-nav2-wifi `mapping.sh` (tagged, idempotent): SSH keep-alives; a fast Ctrl+C (the robot halts first, the local, Jetson and 3D-twin map saves run at the same time, time limits on each, a second Ctrl+C skips the map copy from the Jetson); and the `session_keepalive.sh` hook |
| [`apply-lab-nav-fixes.sh`](../../scripts/tools/apply-lab-nav-fixes.sh) | One-time patch of an upstream go2-nav2-wifi clone: lidar mount, RPP + velocity smoother (see [lab_nav_fixes.md](../navigation/lab_nav_fixes.md)) |
| [`diag_arm.sh`](../../scripts/tools/diag_arm.sh) | D1 arm diagnostic: interface, ping, container, cyclonedds, DDS match |
| [`teach_arm.py`](../../scripts/tools/teach_arm.py) | D1 arm plant scanning: compliance mode (stiffness 5000), save hand-guided poses to `config/arm_waypoints.json` ([d1_arm.md](../hardware/d1_arm.md#plant-scanning-teach--playback)) |
| [`play_arm.py`](../../scripts/tools/play_arm.py) | D1 arm plant scanning: replay the saved poses, hold each for the camera |
| [`calibrate_arm.py`](../../scripts/tools/calibrate_arm.py) | D1 arm per-joint offset measurement (commanded vs reported angle); run via `start_arm.sh --calibrate` |
| [`fix_wifi_powersave.sh`](../../scripts/tools/fix_wifi_powersave.sh) | Turns Wi-Fi power saving off for good on this computer (NetworkManager + Realtek driver) or, with `--robot`, on the Go2 (you type its sudo password; reboot the dog once). `--check` reports only, `--undo` removes it. Power saving stalls the link to the dog |
| [`check_map_tilt.sh`](../../scripts/tools/check_map_tilt.sh) | Did Point-LIO keep the floor level? Fetches a run's map and trajectory (latest on the robot, a named run, or a local copy), fits the floor per tenth of the run and reports the tilt against the start plus the fastest turn. PASS within `TILT_MAX_DEG` (3°) |
| [`rgbd_test.py`](../../tests/rgbd_test.py) | RealSense check on the machine it is plugged into: depth + colour side by side (needs `pyrealsense2`, USB 3), `q` quits |
| [`test_arm_tracker_pure.py`](../../tests/test_arm_tracker_pure.py) | Experiment without ROS: RealSense frame → YOLO → D1 arm joint commands through `d1_arm_control.py`. Moves the arm: keep a hand near the remote |
