# Project Roadmap: Autonomous Navigation, Detailed Mapping & Arm Control

Unitree Go2 EDU — full autonomy stack with 3D mapping and D1 servo arm integration.

**Progress:** `[ ]` = not started  |  `[~]` = in progress  |  `[x]` = done

---

## Phase 0 — Foundation & Simulation

**Goal:** Establish the development environment and verify the complete software stack in simulation before touching real hardware.

**Phase status:** `[ ]`

| Done | Step | Task | Status |
|:----:|------|------|--------|
| `[x]` | 0.1 | Set up Ubuntu 22.04 workstation with ROS 2 Humble | Done |
| `[x]` | 0.2 | Clone and build `autonomy_stack_go2` | Done |
| `[x]` | 0.3 | Set up Ethernet DDS connection to Go2 Jetson | Working |
| `[x]` | 0.4 | Verify `/utlidar/cloud` and `/utlidar/imu` at full rate (14.7 Hz / 250 Hz) | Done |
| `[x]` | 0.5 | Build `go2_integration_pkg` (relays, map node, motion test) | Done |
| `[ ]` | 0.6 | Download Unity environment model for simulation | Pending |
| `[ ]` | 0.7 | Test `./scripts/system_simulation_with_mapping.sh` in Unity | Pending |
| `[ ]` | 0.8 | Verify RViz shows live map from `/map/occupancy` | Pending |

**Deliverable:** Working simulation environment that mirrors real-robot behaviour.

---

## Phase 1 — Detailed Mapping (2D + 3D)

**Goal:** Generate high-quality, saved maps from the Go2's L1 LiDAR that can be reloaded for future navigation.

**Phase status:** `[ ]`

### 1.1 — 2D Occupancy Grid Mapping

| Done | Step | Task | Details |
|:----:|------|------|---------|
| `[ ]` | 1.1.1 | Verify `map_node.py` consumes `/registered_scan` from Point-LIO | CMU stack remaps `/cloud_registered` to `/registered_scan` |
| `[ ]` | 1.1.2 | Verify `/map/occupancy` publishes with `TRANSIENT_LOCAL` QoS | Required for RViz to render the map |
| `[ ]` | 1.1.3 | Add map saving to `~/go2_maps/<timestamp>/` | PNG + PLY + mesh.obj |
| `[ ]` | 1.1.4 | Test 2D mapping in Unity simulation | Robot walks, map builds, Ctrl+C saves |
| `[ ]` | 1.1.5 | Tune `MapConfig` (resolution, hit/miss log-odds) | Target: clean walls, no double-image drift |

**Deliverable:** A saved 2D occupancy grid (`.png`) that accurately represents the scanned environment.

### 1.2 — 3D Point Cloud & Mesh Reconstruction

| Done | Step | Task | Details |
|:----:|------|------|---------|
| `[ ]` | 1.2.1 | Install `open3d` on the KSU workstation | `pip install open3d` |
| `[ ]` | 1.2.2 | Enable Poisson mesh reconstruction in `mapping.py` | Already coded; verify it runs |
| `[ ]` | 1.2.3 | Test mesh export (`mesh.obj`) from a simulation run | Open in MeshLab to verify watertightness |
| `[ ]` | 1.2.4 | Add binary PLY export (XYZ + intensity) | Already coded; verify file size is reasonable |
| `[ ]` | 1.2.5 | Test on real robot over Ethernet | Drive Go2 around a room, save, inspect mesh |

**Deliverable:** A watertight 3D mesh (`.obj`) and point cloud (`.ply`) of a real room.

### 1.3 — RGB Colour Fusion (Camera Integration)

| Done | Step | Task | Details |
|:----:|------|------|---------|
| `[ ]` | 1.3.1 | Identify camera topic from Go2 | `/camera/image/raw` (Unity) or via WebRTC SDK |
| `[ ]` | 1.3.2 | Add camera intrinsics configuration | Required to project LiDAR points into image frame |
| `[ ]` | 1.3.3 | Implement `on_image()` in `map_node.py` | Project point cloud, look up pixel colour, attach RGB |
| `[ ]` | 1.3.4 | Verify coloured PLY export | Open in CloudCompare to see colours |
| `[ ]` | 1.3.5 | Consider adding an Intel RealSense D435i | Provides depth + RGB for better fusion |

**Deliverable:** Coloured 3D map with texture, not just greyscale.

---

## Phase 2 — Autonomous Navigation

**Goal:** Enable the Go2 to navigate to goals autonomously within a mapped environment.

**Phase status:** `[ ]`

### 2.1 — Navigation Stack Verification

| Done | Step | Task | Details |
|:----:|------|------|---------|
| `[ ]` | 2.1.1 | Verify `system_real_robot_ethernet.sh` launches full stack | Point-LIO + local planner + terrain analysis |
| `[ ]` | 2.1.2 | Confirm `/state_estimation` publishes at ~10 Hz | SLAM output from Point-LIO |
| `[ ]` | 2.1.3 | Test waypoint navigation in RViz | Set a goal 1-2 m ahead, verify robot moves |
| `[ ]` | 2.1.4 | Test in Unity simulation first | Safer than real robot |
| `[ ]` | 2.1.5 | Stand robot with remote, then send waypoint | Critical: autonomy stack does not stand the robot |

**Deliverable:** Robot walks to a waypoint in simulation, then on real hardware.

### 2.2 — Map Reload & Localisation

| Done | Step | Task | Details |
|:----:|------|------|---------|
| `[ ]` | 2.2.1 | Save a map from Phase 1 | `~/go2_maps/<timestamp>/` |
| `[ ]` | 2.2.2 | Implement map loading in `map_node.py` | Add parameter `load_map_from` |
| `[ ]` | 2.2.3 | Use AMCL or `slam_toolbox` localisation mode | Relocalise within a saved map |
| `[ ]` | 2.2.4 | Test navigating to a goal in a pre-mapped room | Robot should localise, then navigate |

**Deliverable:** Robot can navigate to goals in a previously mapped environment.

### 2.3 — Frontier Exploration (Optional Advanced)

| Done | Step | Task | Details |
|:----:|------|------|---------|
| `[ ]` | 2.3.1 | Integrate frontier-based exploration | Reference: `gcairone/exploration_go2` |
| `[ ]` | 2.3.2 | Robot autonomously explores unknown space | Maps as it goes, no manual waypoints |
| `[ ]` | 2.3.3 | Test in Unity simulation | Verify frontier detection and goal assignment |

**Deliverable:** Robot autonomously explores and maps a new environment.

---

## Phase 3 — D1 Servo Arm Integration

**Goal:** Control the Unitree D1 6-DOF arm from ROS 2 for manipulation tasks.

**Phase status:** `[ ]`

### 3.1 — Hardware Setup

| Done | Step | Task | Details |
|:----:|------|------|---------|
| `[ ]` | 3.1.1 | Mount D1 arm on Go2 EDU | Compatible with all EDU versions |
| `[ ]` | 3.1.2 | Connect via RJ45 or USB-C | D1 supports both interfaces |
| `[ ]` | 3.1.3 | Verify power and communication | Default IP: `192.168.123.100` |
| `[ ]` | 3.1.4 | Test via Unitree Go2 app | App control confirms hardware is working |

### 3.2 — ROS 2 Driver Installation

| Done | Step | Task | Details |
|:----:|------|------|---------|
| `[ ]` | 3.2.1 | Install official D1 ROS 2 driver | `.deb` packages available for Humble ARM64 |
| `[ ]` | 3.2.2 | Verify `/joint_states` and `/arm_command` topics | Standard interfaces for arm control |
| `[ ]` | 3.2.3 | Test joint angle control | Publish to `joint_position` topic |

### 3.3 — Arm Control Nodes

| Done | Step | Task | Details |
|:----:|------|------|---------|
| `[ ]` | 3.3.1 | Create `arm_control_node.py` in `go2_integration_pkg` | Wraps D1 SDK for high-level commands |
| `[ ]` | 3.3.2 | Implement `move_to_pose()` using inverse kinematics | Use Unitree SDK2 for joint control |
| `[ ]` | 3.3.3 | Implement `grip()` and `release()` | End-effector control |
| `[ ]` | 3.3.4 | Add pose recording for teach-and-playback | Physically position arm, record waypoints |

### 3.4 — Loco-Manipulation Integration

| Done | Step | Task | Details |
|:----:|------|------|---------|
| `[ ]` | 3.4.1 | Coordinate navigation and arm control | Navigate to object, then grasp |
| `[ ]` | 3.4.2 | Add camera-based object detection | Use front camera or RealSense |
| `[ ]` | 3.4.3 | Implement pick-and-place pipeline | `navigate_to(object)` then `pick(object)` then `navigate_to(destination)` then `place(object)` |
| `[ ]` | 3.4.4 | Test in simulation first | Unity model may support arm visualisation |
| `[ ]` | 3.4.5 | Test on real robot | Start with simple pick tasks |

**Deliverable:** Robot can navigate to a target, use the D1 arm to pick up an object, and place it elsewhere.

---

## Phase 4 — Integrated Autonomy & Manipulation

**Goal:** Full mission capability — navigate, map, and manipulate in a single mission.

**Phase status:** `[ ]`

| Done | Step | Task | Details |
|:----:|------|------|---------|
| `[ ]` | 4.1 | Define mission planner node | High-level state machine: map, navigate, grasp, place |
| `[ ]` | 4.2 | Integrate SLAM + navigation + arm control | All in one launch file |
| `[ ]` | 4.3 | Add mission logging | Record trajectories, map updates, arm actions |
| `[ ]` | 4.4 | Test end-to-end in simulation | Full mission in Unity |
| `[ ]` | 4.5 | Test end-to-end on real robot | Full mission on campus |
| `[ ]` | 4.6 | Documentation and demo | Record video, write README |

**Deliverable:** A complete autonomous system that can explore, map, navigate, and manipulate.

---

## Phase 5 — Wireless & Field Deployment

**Goal:** Remove the Ethernet tether and enable field operation.

**Phase status:** `[ ]`

| Done | Step | Task | Details |
|:----:|------|------|---------|
| `[ ]` | 5.1 | Add USB Wi-Fi dongle to Jetson | Panda PAU09 or Alfa AWUS036NHA |
| `[ ]` | 5.2 | Configure `wlan0` with `nmcli` | Static IP on same subnet as workstation |
| `[ ]` | 5.3 | Test native Wi-Fi DDS | `system_real_robot_wireless.sh` |
| `[ ]` | 5.4 | Optional: Zenoh bridge | For flaky Wi-Fi, use `zenoh-bridge-ros2dds` |
| `[ ]` | 5.5 | Field test | Outdoor mapping and navigation |

**Deliverable:** Fully untethered autonomous robot.

---

## Timeline Summary

| Done | Phase | Duration | Deliverable |
|:----:|-------|----------|-------------|
| `[ ]` | Phase 0 | 1 week | Working simulation environment |
| `[ ]` | Phase 1 | 2-3 weeks | 2D map + 3D mesh + coloured point cloud |
| `[ ]` | Phase 2 | 2 weeks | Autonomous navigation to goals |
| `[ ]` | Phase 3 | 2-3 weeks | D1 arm ROS 2 control + pick-and-place |
| `[ ]` | Phase 4 | 2 weeks | Integrated mission |
| `[ ]` | Phase 5 | 1 week | Wireless operation |

**Total:** approximately 10-12 weeks for full capability.

---

## Critical Path

```
Phase 0 (Sim) --> Phase 1 (Mapping) --> Phase 2 (Navigation) --> Phase 3 (Arm) --> Phase 4 (Integration)
```

Do not skip Phase 0. Every subsequent phase depends on a working simulation environment. Test everything in Unity before touching real hardware.

---

## Immediate Next Steps (This Week)

| Done | Step | Action |
|:----:|------|--------|
| `[ ]` | 1 | Verify Unity simulation runs: `./scripts/system_simulation_with_mapping.sh` |
| `[ ]` | 2 | Confirm `/map/occupancy` appears in RViz with QoS `TRANSIENT_LOCAL` |
| `[ ]` | 3 | Run a 2-minute mapping session, walk robot in a circle, Ctrl+C to save |
| `[ ]` | 4 | Inspect saved files: `~/go2_maps/<timestamp>/` should contain `map.png`, `map.ply`, `mesh.obj` |
| `[ ]` | 5 | Open `mesh.obj` in MeshLab to verify the room is recognisable |

Once those five steps work, Phase 1 is complete and you can move to real-robot testing.

---

## Topic Reference

| Topic | Type | Direction | Mode |
|-------|------|-----------|------|
| `/utlidar/cloud` | `sensor_msgs/PointCloud2` | input | Ethernet, Unity |
| `/utlidar/imu` | `sensor_msgs/Imu` | input | Ethernet, Unity |
| `/point_cloud2` | `sensor_msgs/PointCloud2` | input | WebRTC |
| `/imu/data` | `sensor_msgs/Imu` | input | WebRTC |
| `/registered_scan` | `sensor_msgs/PointCloud2` | internal | All modes |
| `/state_estimation` | `nav_msgs/Odometry` | internal | All modes |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | output | All modes |
| `/map/occupancy` | `nav_msgs/OccupancyGrid` | output | All modes |
| `/map/points` | `sensor_msgs/PointCloud2` | output | All modes |
| `/api/sport/request` | `unitree_api/Request` | output | Real robot |

## QoS Reference

| Topic | Reliability | Durability | Depth |
|-------|-------------|------------|-------|
| `/utlidar/cloud` | BEST_EFFORT | VOLATILE | 5 |
| `/utlidar/imu` | BEST_EFFORT | VOLATILE | 5 |
| `/registered_scan` | BEST_EFFORT | VOLATILE | 5 |
| `/state_estimation` | BEST_EFFORT | VOLATILE | 10 |
| `/cmd_vel` | RELIABLE | VOLATILE | 10 |
| `/map/occupancy` | RELIABLE | TRANSIENT_LOCAL | 1 |
| `/map/points` | BEST_EFFORT | VOLATILE | 1 |

## Scripts Reference

| Script | Purpose |
|--------|---------|
| `scripts/move_forward.sh` | One-time sanity check: does the robot move? |
| `scripts/system_real_robot_ethernet.sh` | Daily: real robot on campus |
| `scripts/system_simulation.sh` | Daily: Unity simulation only |
| `scripts/system_simulation_with_mapping.sh` | Combined: Unity + autonomy + mapping |
| `scripts/verify_topics.sh` | Diagnostic: which topics are alive |

---

## How to Use This Tracker

1. Copy this file to your team's shared location or keep it in the repo.
2. When you complete a step, change `[ ]` to `[x]` in the leftmost column of that row.
3. When all steps in a phase are `[x]`, change the **Phase status** marker to `[x]`.
4. The **Timeline Summary** checkboxes should be updated when each phase completes.
5. Commit the updated file:
   ```
   git add ROADMAP.md
   git commit -m "Phase N: mark step X as complete"
   git push
   ```

Anyone on the team can `git pull` to see the current progress.