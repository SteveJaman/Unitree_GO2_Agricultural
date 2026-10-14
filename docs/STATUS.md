# Session Status — 2026-10-13

## What works (verified today)

- VM container `go2-lidar3d` running, DDS bound to 192.168.1.4
- CycloneDDS peers: VM .4 ↔ robot .9
- Lab-nav-fixes applied (RPP controller, velocity smoother, Go2 URDF lidar mount)
- Nav2 planning + controller + smoother all active
- `/goal_pose` → `/cmd_vel_nav` → `/cmd_vel` chain publishing
- `/cmd_vel` produces real non-zero velocities (0.3 m/s observed)
- TF `lidar3d_map → base_link` resolves

## The one remaining blocker

The Jetson motion relay's TCP server never bound to port 17999:

    OSError: [Errno 98] Address already in use

This happens when a stale relay from a previous session is still holding
the port. Result: `/cmd_vel` on the VM has 20 Hz of real commands, but
nothing on the robot is listening — the dog doesn't move.

## The fix (do this before every session)

On the Jetson:

    ssh unitree@192.168.1.9
    pkill -9 -f 'robot-relay-wifi|robot-motion-wifi|go2_cmd_vel_tcp|robot_sport_bridge'
    sleep 3
    ss -tlnp | grep 17999        # MUST print nothing
    exit

Then on the VM:

    cd ~/files/go2-nav2-wifi
    export GO2_HOST_IP=192.168.1.4 GO2_ROBOT_IP=192.168.1.9
    ./mapping.sh --3d --nav

Do NOT start any relay manually — mapping.sh starts both relays itself.
Manually-started relays are what caused the port conflict.

## Do NOT do

- Do not run `./setup-lidar3d.sh` without `--laptop` (hangs on legacy builder)
- Do not run `bash ~/robot-relay-wifi.sh` manually while a session is up
- Do not Ctrl+C `mapping.sh --3d --nav` before RViz finishes loading
- Do not run `start_slam.sh` — it inserts a duplicate SLAM layer that
  conflicts with the upstream Point-LIO pipeline

## When motion works, to test exploration

    cd ~/files/Unitree_GO2_Agricultural/scripts
    ./auto_walk.sh              # scripted square — proves motion path

    # If that works, explorer in a fresh session:
    docker cp frontier_explorer.py go2-lidar3d:/tmp/
    docker exec -u root -it go2-lidar3d bash -lc '
      source /opt/ros/humble/setup.bash && source /ws/install/setup.bash
      python3 /tmp/frontier_explorer.py --ros-args \
        -p map_frame:=lidar3d_map \
        -p goal_topic:=/goal_pose \
        -p cmd_vel_topic:=/cmd_vel_nav \
        -p blind_enable:=false \
        -p coverage_enable:=false
    '

    # then in another shell:
    ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"
