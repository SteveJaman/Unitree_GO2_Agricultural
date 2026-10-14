# Phase 1 Roadmap — Autonomous 3D Tour System

Goal: Go2 autonomously explores an unknown space, captures a dense
colored 3D map, and produces a viewable "home tour" that can be
replayed in a standalone viewer.

Definition of done:
1. Robot explores a 3x3 m room with no human input after START
2. A dense colored PLY is exported
3. The PLY opens in Open3D and looks like a room
4. Every waypoint click in RViz shows a numbered marker and a
   planned path line
5. All of the above runs from one launch command


## Milestone 0 — Stabilize the foundation

Everything else depends on this. Do not skip.

0.1  Jetson relay starts in FULL mode (no --sensors-only)
     Verify: "cmd_vel TCP server started on port 17999" in log

0.2  Port 17999 conflict eliminated before every session
     Verify: ss -tlnp | grep 17999 is empty after pkill

0.3  mapping.sh --3d --nav launches a clean session
     Verify: RViz opens, lidar3d_map -> base_link resolves

0.4  auto_walk.sh completes one square
     Verify: log shows "reached" on all four corners

Blocker: if 0.1-0.2 fail, nothing downstream matters. Fix the
Jetson relay first. This is the single biggest time sink so far.


## Milestone 1 — Visual waypoint system (RViz)

No new code. This is configuration only.

1.1  Add nav2_rviz_plugins/Navigation2 panel
     Verify: Panel shows in Panels -> Add New Panel

1.2  Enable "Waypoint Mode" checkbox
     Verify: Second click adds a numbered marker

1.3  Add Path display on /plan
     Verify: Line updates as robot moves

1.4  Add Path display on /local_plan
     Verify: Line hugs obstacles

1.5  Test 3-waypoint route
     Verify: All markers turn green

Output: You can drop waypoints, see the route, and watch the
robot follow it - no CLI, no auto_walk.sh.


## Milestone 2 — Explorer verification

The explorer code is written and fixed. It has never produced a
goal on a live map.

2.1  Run frontier_explorer.py manually with blind_enable:=false
     Verify: Node logs "FRONTIER: (x,y) size=N",
             goals_sent increments in /explore/status

2.2  Confirm RViz START button publishes /explore/resume
     Verify: ros2 topic echo /explore/resume shows data: true

2.3  Explorer drives to first frontier
     Verify: "Goal reached" in explorer log

2.4  Full exploration of 3x3 m room
     Verify: state=RETURNING then DONE

2.5  Verify no re-sending of blacklisted goals
     Verify: failed count grows, no repeat coordinates

Risk: if 2.1 shows goals_sent=0 after enabling, the frontier
detector is finding nothing. Add debug logging:
    self.get_logger().info(
        f'known={known_cells} frontier_px={int(frontier_mask.sum())} '
        f'clusters={len(frontiers)}')


## Milestone 3 — Camera pipeline

Required for reconstruction. Currently blocked.

3.1  Fix the camera backend on Jetson
     Verify: no "camera unavailable" warning,
             ros2 topic hz /go2_front_camera/image_raw ~15 Hz

3.2  Verify image reaches VM
     Verify: Image visible in RViz

3.3  Mount Intel RealSense D435i (if available)
     Verify: ros2 topic hz /camera/depth/image_raw ~30 Hz

3.4  Calibrate extrinsics camera <-> base_link
     Verify: tf2_echo base_link camera_link streams

Blocker if no RealSense: built-in camera + LiDAR gives sparse
color only. Enough to prototype, not enough for a real estate
quality tour.

Fallback: use only LiDAR + Point-LIO. You'll get a dense
uncolored point cloud. Usable for geometry, not for a tour.


## Milestone 4 — RTAB-Map reconstruction

The pipeline that produces the dense 3D tour.

4.1  Install ros-humble-rtabmap-ros in the container
     Verify: ros2 pkg list | grep rtabmap

4.2  Launch RTAB-Map alongside Nav2 session
     Verify: rtabmap appears in ros2 node list

4.3  Remap RTAB-Map output to /tour/* to avoid /map conflict
     Verify: Nav2 keeps working during RTAB-Map run

4.4  Manual walk in one room, export PLY
     Verify: tour.ply on disk, size > 5 MB

4.5  Open PLY in Open3D
     Verify: python3 -c "import open3d as o3d;
             o3d.visualization.draw_geometries(
             [o3d.io.read_point_cloud('tour.ply')])"

4.6  Poisson mesh from point cloud
     Verify: tour_mesh.ply renders solid in viewer

Compute risk: RTAB-Map is heavier than Point-LIO. If the VM
struggles, run RTAB-Map on a separate machine or throttle the
LiDAR input rate.


## Milestone 5 — Capture automation

Make the explorer pause and capture at each frontier.

5.1  Add spin_360() to frontier_explorer.py
     Verify: Robot spins in place

5.2  Call spin_360() after each goals_succeeded
     Verify: Log shows "Spin started" / "Spin complete"

5.3  Suppress explorer goal publishing during spin
     Verify: goals_sent unchanged during spin

5.4  Wait for RTAB-Map to process spin frames
     Verify: RTAB-Map log shows "Loop closure detected"

Code sketch:
    def spin_360(self, rate=0.4):
        duration = 2 * math.pi / rate
        t0 = time.time()
        twist = Twist()
        twist.angular.z = rate
        while time.time() - t0 < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        self.cmd_vel_pub.publish(Twist())


## Milestone 6 — End-to-end autonomous capture

The full pipeline, no human input.

6.1  One launch command runs everything
     Verify: ./wifi_nav.sh --explore --capture
             All nodes up, RViz shows map

6.2  Explorer autonomously maps the room
     Verify: state=DONE in log

6.3  RTAB-Map produces dense map
     Verify: PLY exported at end

6.4  Post-processing script runs automatically
     Verify: tour_mesh.ply exists

6.5  Web viewer loads the mesh
     Verify: three.js or potree viewer shows scene

Deliverable for Phase 1 demo: run ./wifi_nav.sh --explore --capture,
walk away, come back in 5 minutes, open tour.html, see the room.


## Milestone 7 — Hardening

Polish for repeated use.

7.1  Pre-flight script checks all layers
     Verify: ./preflight.sh prints pass/fail

7.2  Session logs archived per run
     Verify: ws/log/tour-YYYYMMDD-HHMMSS/

7.3  Failure recovery: explorer resumes after goal timeout
     Verify: No manual restart needed

7.4  Config file for tour settings
     Verify: config/tour.yaml - spin rate, PLY path, mesh depth

7.5  README section on running a tour
     Verify: docs/tour.md exists


## Order of work

    Milestone 0  <- foundation, must be green
         |
         v
    Milestone 1  <- 10 min, no code
         |
         v
    Milestone 2  <- verify explorer produces goals
         |
         v
    Milestone 3  <- camera fix (blocker for 4)
         |
         v
    Milestone 4  <- reconstruction proof
         |
         v
    Milestone 5  <- automate capture
         |
         v
    Milestone 6  <- end-to-end
         |
         v
    Milestone 7  <- polish

Milestones 0-2 are this week. 3-4 are next. 5-7 are the stretch.


## What Phase 1 does NOT include

Deferred to Phase 2 or later:

- Semantic classification (YOLO: safe clutter vs fragile)
- Gait modulation (footRaiseHeight on detected obstacles)
- Multi-floor tours
- Live streaming of the tour to a remote client
- Cloud upload / Matterport-style hosted viewer
- Automatic room segmentation and labeling
- Voice or VLM interface

These layer on top of Phase 1. Do not start them until 0-6 are green.


## Success metric for Phase 1

A person who has never seen the robot runs one command, waits
10 minutes, and receives a .ply file that they can open and
recognize as the room the robot just explored.

Everything in this roadmap serves that one sentence.