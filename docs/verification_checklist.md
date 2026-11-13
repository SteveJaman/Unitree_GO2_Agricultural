# Robot Verification Checklist

[← README](../README.md) · [Docs index](README.md) · [Status](status.md) · [Changelog](changelog.md) · [Troubleshooting](troubleshooting.md)

Checks that need the real Go2 (and D1 arm), with the command and the expected result. Items
marked ✅ have passed on the robot; the rest are pending. Newer hardware checks for the
traversability stack are in the [traversability handoff](hardware/traversability_handoff.md),
and for the arm in [d1_arm.md](hardware/d1_arm.md).

## Checks

| # | Check | Command | Expected |
|---|---|---|---|
| 1 | Costmap exists and publishes | `ros2 topic hz /global_costmap/costmap` | ~1 Hz |
| 2 | Explorer subscribes | `ros2 topic info /global_costmap/costmap --verbose` | Sub count = 1 |
| 3 | Frontier detection runs | explorer log | `detect_frontiers: known=N frontiers=K` |
| 4 | Explorer sends goals | `ros2 topic echo /explore/status` | `goals_sent > 0` |
| 5 | Dog moves to frontier | visual | dog walks toward unexplored area |
| 6 | RViz START button publishes | `ros2 topic echo /explore/resume` while clicking | `data: true` |
| 7 | STOP button halts dog | click | stop within 1 s |
| 8 | Full autonomous run | watch | `IDLE → EXPLORING → RETURNING → DONE` |
| 9 | Camera reaches container | `ros2 topic hz /camera/image_raw` | ~10–30 Hz |
| 10 | Arm reachable | `./scripts/tools/diag_arm.sh` (on the Go2: `start_arm.sh --robot <ip> state`) | all PASS — ✅ over Wi-Fi 2026-10-27 |
| 11 | Arm responds to `joint` | `start_arm.sh enable sleep 2 joint 0 20 sleep 3 release` | joint 0 moves 20° — ✅ over Wi-Fi 2026-10-27 (−0.8° → +20.0°) |
| 12 | Joint offsets small | `start_arm.sh --calibrate --test-deg 10 --yes` | offsets within a degree or two |
| 13 | Fold calibration | `start_arm.sh read`, paste, `start_arm.sh fold` | arm folds onto mount, goes limp — ✅ over Wi-Fi 2026-10-27 |
| 14 | Robot discovery | `./scripts/runtime/wifi_nav.sh --check` | banner shows `[discovered]` robot IP |
| 15 | Go2 Jetson MAC prefix | `ip link` on the Jetson | starts `02:00:00` (else set `GO2_MAC_PREFIX`) |
| 16 | Save RViz layout | File → Save Config As | `config/tour.rviz` exists |
| 17 | 3D image builds | `./scripts/runtime/wifi_3d_nav.sh --build` | `go2-3d:local` built |
| 18 | 3D stack starts | `./scripts/runtime/wifi_3d_nav.sh --no-explore` | `3D digital twin READY` printed |
| 19 | Dense map grows | dashboard `http://localhost:3000`, 3D view | map fills in as the dog walks |
| 20 | Click-to-go | dashboard Click-to-go, click the floor, then a desk | `goal (...)` in `click_to_goal.log`; desk click logs `snapped … to free floor`, green arrow beside it |
| 21 | DB saved + export | Ctrl+C, then `wifi_3d_nav.sh --export` | `[3d] Map saved`, PLY in `<db>-export/` |
| 22 | Camera fusion | camera relay up, `wifi_3d_nav.sh` | READY shows `RTAB-Map : camera`; `/rtabmap/cloud_map` coloured |
| 23 | Camera TF / intrinsics | dashboard: coloured map vs camera panel | colours on the right surfaces (else calibrate, roadmap 3.4) |
| 24 | YOLO | stand in front of the dog | `/yolo/image` boxes; `/yolo/markers` `person ~X m` matches a tape measure |
| 25 | YOLO → Nav2 | `--ai` (banner `YOLO → Nav2 : on`), chair in the path | chair shows in `/global_costmap/costmap`, plan goes around it |
| 26 | Battery data | `go2sh; ros2 topic echo /lf/lowstate --field bms_state.soc` | matches the app's battery % |
| 27 | Low-battery dock | `TWIN_BATTERY_LOW=95 wifi_3d_nav.sh --ai` (test threshold), explorer running | explorer stops, dog drives to the start point, START refused |
| 28 | VLM | `ollama serve` + `llava:7b`, `--ai` | `[checks] Ollama llava:7b … OK`; `/vlm_decision` updates in the dashboard |
| 29 | Resume | run, Ctrl+C, `--resume` from the same start spot | READY shows `[resumed …]`; old map visible, joined after revisiting a place |
| 30 | Named locations | `--save-location`, click, type `Dock` | entry in `config/semantic_locations.json` |
| 31 | Dashboard | open `http://<host IP>:3000` from another device | green rosbridge dot, robot moves with the dog, battery % matches the app |
| 32 | Dashboard camera | camera mode, dashboard Camera / YOLO tabs | ~5 fps each |
| 33 | Dashboard STOP | explorer running, click STOP | dog stops, explorer `IDLE` |
| 34 | Auto-localization | `--ai` with the dog at the dock | `auto_localize.log`: `initial pose … published`; no manual 2D Pose Estimate needed |
| 35 | VLM action | explorer running, block the path with a desk / board | `vlm.log`: `BLOCKED 1/2`, then `path BLOCKED (confirmed)`; explorer stops, dog returns to the dock |
| 36 | VLM false alarm | `TWIN_VLM_ACT=0 … --ai`, normal run | count how often `BLOCKED` appears on a clear path before trusting the action |
| 37 | Battery-swap restart | Ctrl+C, swap, dog at the dock, `--ai --resume` | READY shows `[resumed …]`, auto-localized, map continues |
| 38 | Solid map frame rate | dashboard Solid, big map, ⚙ shows fps | ≥ 50 fps on the presentation laptop (else Detail: Medium) |
| 39 | Presentation mesh | after a run: `--export --mesh`, then `--view` | 3D Scan shows closed walls and floor; < 1 min meshing |
| 40 | Return to Dock | explorer running, press Return to Dock | explorer stops, pin at the dock, route drawn, dog drives home |
| 41 | Room segmentation | map two rooms joined by a door | two coloured rooms split at the doorway; `config/semantic_rooms.json` written |
| 42 | Room naming | Rooms → Edit, click a room, robot inside it, `--ai` | dialog shows a VLM suggestion; Save keeps the name after the next re-segmentation |
| 43 | Merge / navigate | merge two adjacent rooms; Navigate to room | one room; the dog drives to the room's open centre |
| 44 | Explorer re-plan | explore a room with an open doorway | log `explored before arrival: re-planning`; fewer stops; `replans=` grows |
| 45 | Explorer STOP during spin | `spin_on_goal:=true`, STOP mid-spin | rotation stops within ~0.1 s |
| 46 | Autonomous survey | plain `wifi_3d_nav.sh`, dog at the dock, two or more rooms, a bed or fridge in one | no input needed: rooms visited and spun in, named (✨), dog docks; `config/survey_report.json` written |
| 47 | Survey keeps user names | name a room by hand, then run a survey | the hand-typed name is unchanged |
| 48 | Survey battery pause | `TWIN_BATTERY_LOW=95 ./scripts/runtime/wifi_3d_nav.sh` | survey shows `PAUSED`, the dog docks; resumes after the battery rises |
| 49 | One command from scratch | fresh clone, no image, Ollama installed but stopped: `./scripts/runtime/wifi_3d_nav.sh` | image built, `llava:7b` pulled, `ollama serve` started, survey runs to the dock |
| 50 | Drive `omni` | `wifi_nav.sh --no-explore --drive omni` (or `wifi_3d_nav.sh --no-explore --omni`), remote in hand; 2D Goal Pose 1 m behind, then 1 m to the side | walks backwards without turning; sidesteps ([drive modes](navigation/drive_modes.md#tested)) |
| 51 | Drive `hybrid` recovery | default drive, explorer next to a wall until a leg aborts | `/tmp/stuck_recovery.log`: steps back or aside 0.3 m, goal re-sent |
| 52 | MoveIt plan | `./scripts/runtime/arm_moveit.sh --inspect` on the workstation | RViz: most viewpoints green, arm never touches the Go2 boxes or the plant |
| 53 | MoveIt → real arm | after a trajectory bridge exists: one planned move, low scaling, arm at home | the real arm follows the plan; joint signs match ([d1_arm.md](hardware/d1_arm.md#from-plan-to-the-real-arm)) |
| 54 | Unity sim | `./scripts/runtime/sim_explore.sh --autostart` | goals reached in the Unity environment; Ctrl+C stops cleanly |
| 55 | Camera Relay starts | `docker logs go2-lidar3d \| grep camera` | No "ModuleNotFoundError: cv2" |
| 56 | Keepalive on Wi-Fi drop | Disconnect dog's Wi-Fi | Robot stops safely; RViz/Web App stay open |
| 57 | Debug Dashboard | dashboard **System** tab (or `python3 scripts/debug/debug_dashboard.py` with plain `wifi_nav.sh`) | green checks for Nav2/TF; Copy debug report works |
| 58 | Dashboard first | `./scripts/runtime/wifi_3d_nav.sh`, open the printed link at once | page opens before the robot is up; start-up overlay ticks through to 3D twin |
| 59 | Settings on the dog | Settings: Wide preset, Apply; then Reset | dog keeps visibly more clearance; Reset restores; values match `tune_costmap.sh --show` |
| 60 | Quiet terminal | normal start with Ollama off | short terminal with the Ollama warning and its fix; everything in `ws/log/3d/boot.log` |

---

## Session plan

```bash
# 1. Power on the dog, stand it up
# 2. Power on the D1 arm if using it (24V DC, RJ45 to the workstation's wired
#    port, address from the go2-arm profile; or plug it into the Go2 and use
#    --arm-wifi / start_arm.sh --robot <ip>)
./scripts/runtime/start_arm.sh --check

# 3. Launch the full stack (drop --arm for navigation only)
cd ~/files/Unitree_GO2_Agricultural
./scripts/runtime/wifi_nav.sh --check          # confirm discovered IPs first
./scripts/runtime/wifi_nav.sh --arm

# 4. In another terminal: verify costmap
go2sh
ros2 topic hz /global_costmap/costmap      # expect ~1 Hz
ros2 topic echo /explore/status            # watch goals_sent

# 5. Click START in RViz
#    Watch explorer log for FRONTIER: lines
#    Watch /explore/status for goals_sent > 0

# 6. Check the arm fold calibration and joint offsets
./scripts/runtime/start_arm.sh read            # paste FOLDED_ANGLES into d1_arm_calibration.py if the mount changed
./scripts/runtime/start_arm.sh --calibrate --test-deg 10 --yes

# 7. Save the RViz layout
./scripts/tools/save_rviz_config.sh
#    Then follow the printed steps in RViz

# 8. Commit any config/tour.rviz that gets saved
```

### Autonomous loop test (3D twin)

```bash
# Dog standing AT THE DOCK (the map origin, dock goal and auto-localization assume it)
# First run needs internet: builds the image, pulls llava:7b, starts ollama serve
./scripts/runtime/wifi_3d_nav.sh                # banner: YOLO → Nav2 on, Ollama OK, Localize auto, Survey AUTO
# Open http://localhost:3000 (READY block) and watch; the survey needs no input
# Step-by-step instead: ./scripts/runtime/wifi_3d_nav.sh --manual
# Safe low-battery test: TWIN_BATTERY_LOW=95 ./scripts/runtime/wifi_3d_nav.sh
# VLM false-alarm check first: TWIN_VLM_ACT=0 ./scripts/runtime/wifi_3d_nav.sh --manual

# After a battery swap: dog back at the dock, then
./scripts/runtime/wifi_3d_nav.sh --resume
```

Work through checks 17–57 above (see [3d_nav.md](ai_and_vision/3d_nav.md)). Keep the
handheld remote as the emergency stop.

No robot? Run the same plan against the simulator (`./gazebo/setup_gazebo.sh`
once, then `./gazebo/test_sim.sh` and `./scripts/runtime/wifi_nav.sh --sim --arm`).
