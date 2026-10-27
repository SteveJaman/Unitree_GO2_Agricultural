# Scripts Reference

[← Back to README](../README.md) · [Setup](setup.md) · [Explorer](explorer.md) · [3D twin](3d_nav.md)

Every entry point, ROS node and tool in [`scripts/`](../scripts/). The
go2-3d container's own files (YOLO node, image compressor, web dashboard)
are in [`twin/`](../twin/), described in [3d_nav.md](3d_nav.md).

---

## Runtime ([`scripts/runtime/`](../scripts/runtime/))

| Script | Purpose |
|---|---|
| [`wifi_nav.sh`](../scripts/runtime/wifi_nav.sh) | **Main entry point.** Relay + Nav2 + RViz + explorer + camera; `--arm` also folds the D1 arm |
| [`wifi_3d_nav.sh`](../scripts/runtime/wifi_3d_nav.sh) | **The one command.** Builds or refreshes its image, starts Ollama and pulls the VLM, then by default runs the AI and the unattended survey (`--manual`, `--no-ai` opt out). Runs `wifi_nav.sh` unchanged plus RTAB-Map (camera fusion when the camera publishes), the web dashboard (`:3000`, rosbridge `:9090`) and click-to-goal in a separate `go2-3d` container; `--manual`, `--no-ai`, `--no-build`, `--resume`, `--save-location`, `--foxglove`, `--build`, `--export`, `--stop` |
| [`wifi_mapping.sh`](../scripts/runtime/wifi_mapping.sh) | Mapping-only wrapper (no explorer) |
| [`start_rtabmap.sh`](../scripts/runtime/start_rtabmap.sh) | Older manual RTAB-Map launcher inside `go2-lidar3d` (superseded by `wifi_3d_nav.sh`) |
| [`start_arm.sh`](../scripts/runtime/start_arm.sh) | D1-T arm entry point: preflight (address + ping), then `fold`, `read`, `--calibrate`, or any `d1_arm_control.py` command chain |
| [`go2_net.sh`](../scripts/runtime/go2_net.sh) | Sourced library: host-IP detection, `find_go2_ip()` robot discovery, interface safeguards |

## Nodes ([`scripts/nodes/`](../scripts/nodes/))

| File | Purpose |
|---|---|
| [`frontier_explorer.py`](../scripts/nodes/frontier_explorer.py) | Three-phase explorer: blind lead-in → Yamauchi frontier → visited-cell coverage → return. Optional spin-360 capture for tours |
| [`go2_camera_relay.py`](../scripts/nodes/go2_camera_relay.py) | Robot JPEG frames (`/go2/front_camera/compressed`) → `/camera/image_raw` (bgr8) + `/camera/camera_info` |
| [`scripts/robot/go2_camera_wifi.sh`](../scripts/robot/go2_camera_wifi.sh) | **Runs on the robot** (copied and started by `wifi_nav.sh`): front camera JPEG → Wi-Fi DDS |
| [`go2_floor_filter.py`](../scripts/nodes/go2_floor_filter.py) | Floor-plane filter, prevents floor being treated as an obstacle |
| [`go2_stuck_recovery.py`](../scripts/nodes/go2_stuck_recovery.py) | Every session: on an aborted leg (e.g. facing a wall), step back (and aside with `--omni`) into free space and retry the goal |
| [`go2_camera_obstacles.py`](../scripts/nodes/go2_camera_obstacles.py) | `--camera-obstacles`: Depth Anything V2-Small depth, scaled by the LiDAR → `/camera/obstacle_points` for the Nav2 costmap |
| [`go2_fusion_camera.py`](../scripts/nodes/go2_fusion_camera.py) | `--fusion`: undistorted camera + LiDAR depth image + camera_info with one stamp, for RTAB-Map RGB-D |
| [`go2_lidar_overlay.py`](../scripts/nodes/go2_lidar_overlay.py) | Projects `/lidar3d/registered` onto the front camera → `/camera/lidar_overlay` (camera–LiDAR alignment check, live mount tuning) |
| [`go2_camera_info.py`](../scripts/nodes/go2_camera_info.py) | `/camera/camera_info` for a camera without one (pinhole from a field of view, or fx/fy/cx/cy); skipped when the calibrated relay publishes it |
| [`battery_manager.py`](../scripts/nodes/battery_manager.py) | Always: `/battery_state` for the dashboard. By default: low battery (< 15 %) → pause explorer, go to the dock; explorer blocked until 20 % |
| [`vlm_reasoning_node.py`](../scripts/nodes/vlm_reasoning_node.py) | Default (not `--no-ai`): camera frame → local Ollama `llava:7b` → `/vlm_decision`; confirmed `BLOCKED` → pause explorer + dock |
| [`auto_localize.py`](../scripts/nodes/auto_localize.py) | Default / `--resume`: one-shot `/initialpose` at the dock (0, 0, 0) |
| [`save_location.py`](../scripts/nodes/save_location.py) | `--save-location`: label clicked points → `config/semantic_locations.json` |
| [`go2_click_to_goal.py`](../scripts/nodes/go2_click_to_goal.py) | Dashboard (or Foxglove) `/clicked_point` (any TF frame) → nearest safe floor (BFS on the global costmap, 3×3 clearance) → `/goal_pose`; markers on `/clicked_goal_markers` |
| [`d1_arm_control.py`](../scripts/nodes/d1_arm_control.py) | D1-T arm chain-style CLI + `D1Arm` class (JSON over CycloneDDS): state, enable, release, home, power, joint(s), feedback, `lift`, `sweep` |
| [`d1_arm_calibration.py`](../scripts/nodes/d1_arm_calibration.py) | `--read` current angles / `--move` to `FOLDED_ANGLES`, then release and power off |
| [`direct_test_movement.py`](../scripts/nodes/direct_test_movement.py) | Small, safe movement test for the arm (uses `d1_arm_control.py`) |

## Tools ([`scripts/tools/`](../scripts/tools/))

| Script | Purpose |
|---|---|
| [`auto_walk.sh`](../scripts/tools/auto_walk.sh) | Scripted 1 m square patrol via `/goal_pose` — proof the motion path works |
| [`test_goal_cli.sh`](../scripts/tools/test_goal_cli.sh) | Publish a single `PoseStamped` goal from the CLI |
| [`test_explore.sh`](../scripts/tools/test_explore.sh) | Diagnose the explorer without the RViz button |
| [`preflight.sh`](../scripts/tools/preflight.sh) | Layered verification: container, DDS, packages, nodes, topics, TF |
| [`campus_session.sh`](../scripts/tools/campus_session.sh) | All-in-one verify → tune → patch → launch → monitor |
| [`tune_costmap.sh`](../scripts/tools/tune_costmap.sh) | Change inflation radius and footprint padding at runtime (`--safe`, `--wide`, `--tight`, `--show`) |
| [`verify_costmap.sh`](../scripts/tools/verify_costmap.sh) | Confirm the costmap topic exists and publishes |
| [`save_rviz_config.sh`](../scripts/tools/save_rviz_config.sh) | Interactive guide to save the RViz layout to [`config/tour.rviz`](../config/tour.rviz) |
| [`calibrate_camera.sh`](../scripts/tools/calibrate_camera.sh) | Headless checkerboard calibration of the front camera → `config/camera/front_camera.yaml` (`/camera/camera_info`) |
| [`lidar_overlay.sh`](../scripts/tools/lidar_overlay.sh) | Run the camera–LiDAR overlay check next to a session (`--sim` for Gazebo) |
| [`rviz_add_camera.py`](../scripts/tools/rviz_add_camera.py) | Adds Front camera / LiDAR on camera / 3D map displays to the session RViz layouts (run by `wifi_nav.sh`) |
| [`nav2_camera_source.py`](../scripts/tools/nav2_camera_source.py) | Adds/removes the `camera` obstacle source in go2-nav2-wifi's `lidar3d_planner.yaml` (run by `wifi_nav.sh`) |
| [`nav2_omni.py`](../scripts/tools/nav2_omni.py) | Switches go2-nav2-wifi between forward-only RPP and MPPI Omni ([`config/nav2/go2_omni.yaml`](../config/nav2/go2_omni.yaml); run by `wifi_nav.sh`) |
| [`build_go2_model.py`](../scripts/tools/build_go2_model.py) | Unitree's `go2_description` URDF + meshes → decimated `twin/webapp/public/go2/` (PLY + `go2.json`) for the dashboard's animated Go2; run once in the `go2-3d` image (command in the file) |
| [`export_tour.py`](../scripts/tools/export_tour.py) | Open3D Poisson mesh from an RTAB-Map PLY (normals, distance trim, decimation); run by `wifi_3d_nav.sh --export --mesh` |
| [`add-yolo-costmap-source.sh`](../scripts/tools/add-yolo-costmap-source.sh) | Add `/yolo/obstacles` to the Nav2 costmap config in go2-nav2-wifi; run automatically by `wifi_3d_nav.sh` (unless `--no-ai`) (`--undo` restores) |
| [`apply-lab-nav-fixes.sh`](../scripts/tools/apply-lab-nav-fixes.sh) | One-time patch of an upstream go2-nav2-wifi clone: lidar mount, RPP + velocity smoother (see [LAB-NAV-FIXES.md](LAB-NAV-FIXES.md)) |
| [`diag_arm.sh`](../scripts/tools/diag_arm.sh) | D1 arm diagnostic: interface, ping, container, cyclonedds, DDS match |
| [`calibrate_arm.py`](../scripts/tools/calibrate_arm.py) | D1 arm per-joint offset measurement (commanded vs reported angle); run via `start_arm.sh --calibrate` |
