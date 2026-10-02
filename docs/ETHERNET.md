# Ethernet Mode

Direct cable connection between the workstation and the Go2. Highest performance, most reliable transport.

## Performance

| Metric | Value |
|--------|-------|
| LiDAR rate | 14.7 Hz |
| IMU rate | 250 Hz |
| Latency | ~0.4 ms |
| Packet loss | 0% |
| CPU usage | under 20% |

This is the recommended mode for development. Native DDS, no relay, no bridge. Point-LIO receives raw sensor data at full rate.

## Why Ethernet

- Native DDS. The Jetson publishes /utlidar/cloud and /utlidar/imu directly.
- No WebRTC. No H.264 decode. No 100 percent CPU core.
- The CMU autonomy stack was designed for this transport.
- Point-LIO converges quickly at 14.7 Hz.

## Physical Setup

```
+------------------+                    +-----------------+
|  KSU desktop     |  Ethernet cable    |  Go2 robot      |
|  (Windows host)  | ---------------->  |  (rear port)    |
|                  |                    |                 |
|  +-- Ubuntu VM   |                    |  Jetson eth0    |
|      (enp0s8)    |                    |  192.168.123.18 |
|  192.168.123.100 |                    |                 |
+------------------+                    +-----------------+
```

1. Plug an Ethernet cable from the desktop physical NIC to the Go2 rear Ethernet port.
2. In the VM settings, add a second network adapter in Bridged mode bound to that physical NIC.
3. Boot the VM.

## Step 1 - Configure the VM Ethernet Interface

Find the interface name.

```bash
ip -o link show | awk -F': ' '{print $2}' | grep -v lo
```

Typical output: enp0s3 (Wi-Fi or NAT), enp0s8 (Ethernet), possibly enp0s9.

Set a static IP on the Ethernet interface. Assume enp0s8.

```bash
sudo ip addr add 192.168.123.100/24 dev enp0s8
sudo ip link set enp0s8 up
```

Verify.

```bash
ip addr show enp0s8 | grep inet
```

Expected:

```
inet 192.168.123.100/24
```

## Step 2 - Verify Connectivity

```bash
ping -c 3 192.168.123.18
```

Expected: 0 percent packet loss, under 2 ms latency.

```bash
ssh unitree@192.168.123.18
```

Password is 123 on the EDU unit. Type exit to return.

If ping works but SSH refuses, the Jetson's sshd may be bound to an internal interface. See docs/TROUBLESHOOTING.md.

## Step 3 - Make the IP Persistent

Without this, the IP disappears on VM reboot.

```bash
sudo nano /etc/netplan/01-netcfg.yaml
```

Example content. Adjust the interface names.

```yaml
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    enp0s3:
      dhcp4: true
    enp0s8:
      dhcp4: false
      addresses: [192.168.123.100/24]
```

Apply.

```bash
sudo netplan apply
```

## Step 4 - Verify the Jetson Topics

SSH into the Jetson and list topics.

```bash
ssh unitree@192.168.123.18

source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

ros2 topic list | grep utlidar
```

Expected topics from the dog:

```
/utlidar/cloud
/utlidar/cloud_deskewed
/utlidar/imu
/utlidar/robot_odom
/utlidar/robot_pose
```

From the VM, verify the same topics are visible.

```bash
source /opt/ros/humble/setup.bash
source ~/files/Unitree_GO2_Agricultural/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/files/Unitree_GO2_Agricultural/config/cyclonedds_ethernet.xml"
export ROS_DOMAIN_ID=0

ros2 topic list | grep utlidar
ros2 topic hz /utlidar/cloud_deskewed
```

Expected rate: about 10 to 15 Hz.

## Step 5 - Test Motion Without Autonomy

The simplest way to confirm the control chain works.

Prerequisites: the robot must be standing. Use the physical remote to stand it up.

```bash
cd ~/files/Unitree_GO2_Agricultural
./scripts/move_forward.sh
```

The robot should walk forward for 3 seconds, then stop.

If it moves, the DDS and control chain is correct. Continue to Step 6.

If it does not move, see docs/TROUBLESHOOTING.md.

## Step 6 - Launch the Autonomy Stack

```bash
./scripts/system_real_robot_ethernet.sh
```

This script sources ROS 2 and the autonomy stack, sets CYCLONEDDS_URI to cyclonedds_ethernet.xml, and launches vehicle_simulator system_real_robot.launch.

Terminal output includes:

```
[pointlio_mapping-3] Multi thread started
[pointlio_mapping-3] lidar_type: 5
[pointlio_mapping-3] IMU Initializing: 1%
[pointlio_mapping-3] IMU Initializing: 100.0%
[localPlanner-4] [INFO] [localPlanner]: Initialization complete.
```

RViz opens automatically. It shows the point cloud, robot pose, terrain map, and path.

## Step 7 - Verify the SLAM Output

In a second terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/files/autonomy_stack_go2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/files/Unitree_GO2_Agricultural/config/cyclonedds_ethernet.xml"
export ROS_DOMAIN_ID=0

ros2 topic hz /state_estimation
```

Expected: about 10 Hz.

If /state_estimation publishes, Point-LIO is running.

## Step 8 - Stand the Robot

The autonomy stack does not stand the robot up. This is manual.

Use the physical Go2 remote.

1. Press the stand up button.
2. Wait until the robot is stable on all four legs.
3. The robot is now in Sport Mode.

Do not attempt to send motion commands while the robot is lying down.

## Step 9 - Send a Waypoint

In RViz:

1. Find the Waypoint button in the top toolbar.
2. Click it, then click a point on the ground 1 to 2 meters in front of the robot.
3. The robot should walk toward the point.

## Step 10 - Build a Map

To run mapping while the autonomy stack is active, open a third terminal.

```bash
source /opt/ros/humble/setup.bash
source ~/files/Unitree_GO2_Agricultural/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/files/Unitree_GO2_Agricultural/config/cyclonedds_ethernet.xml"
export ROS_DOMAIN_ID=0

ros2 launch go2_integration_pkg mapping.launch.py
```

The map node auto-detects the cloud and odom topics. It prints which topics it chose.

Expected output:

```
map_node started
   cloud in : /registered_scan
   odom in  : /state_estimation
   map out  : /map/occupancy (TRANSIENT_LOCAL)
   pts out  : /map/points (BEST_EFFORT)
   save dir : /home/<user>/go2_maps/20261002_143022
```

In RViz, add two displays:

1. Click Add. Select By topic. Choose /map then Map. Set the color scheme to map.
2. Click Add. Select By topic. Choose /map then PointCloud2.

The 2D occupancy grid appears in grayscale. The 3D cloud appears as accumulated points.

Drive the robot around. The map builds up.

Press Ctrl+C when done. The map is saved to ~/go2_maps/<timestamp>/ with these files:

| File | Content |
|------|---------|
| map.png | Top-down grayscale occupancy grid |
| map.ply | Binary point cloud in XYZ format |
| mesh.obj | Poisson reconstructed mesh |
| points.npy | Raw numpy array of the accumulated cloud |

## Step 11 - Save a Map for Navigation

To use the map for autonomous navigation, save it in the format that map_server expects.

```bash
ros2 run nav2_map_server map_saver_cli -f ~/go2_maps/my_map
```

This writes my_map.yaml and my_map.pgm to ~/go2_maps/.

## Step 12 - Autonomous Navigation with Localization

To navigate the robot to goals against a saved map, run the navigation script.

Terminal 1: keep the robot stack running (from Step 6).

Terminal 2:

```bash
cd ~/files/Unitree_GO2_Agricultural
./scripts/system_navigation.sh ~/go2_maps/my_map.yaml
```

This launches:

- pointcloud_to_scan.py to convert the 3D cloud to /scan
- map_server to load the saved map
- AMCL to localize the robot on that map
- Nav2 to plan and follow paths

In RViz:

1. Click the 2D Pose Estimate button. Drag on the map to set the robot's initial position and heading.
2. Click the 2D Goal Pose button. Drag to send the robot to a target location.

The planner produces a blue path. The controller follows it. The robot walks to the goal.

## Step 13 - Live Camera

The Go2 front camera is available over Ethernet only when the WebRTC SDK is running. In a separate terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/files/ros2_ws/install/setup.bash

export ROBOT_IP="192.168.137.33"
export CONN_TYPE="webrtc"

ros2 launch go2_robot_sdk robot.launch.py \
    rviz2:=false nav2:=false slam:=false \
    foxglove:=false joystick:=false teleop:=false
```

This publishes /camera/image_raw over DDS.

In another terminal, run the camera relay to ensure QoS compatibility with RViz.

```bash
source /opt/ros/humble/setup.bash
source ~/files/Unitree_GO2_Agricultural/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/files/Unitree_GO2_Agricultural/config/cyclonedds_ethernet.xml"
export ROS_DOMAIN_ID=0

ros2 run go2_integration_pkg camera_relay.py
```

In RViz, add an Image display. Select the topic /camera/image_raw_relayed.

If you want RGB color fusion on the map, restart the map node with use_camera enabled.

```bash
ros2 launch go2_integration_pkg mapping.launch.py use_camera:=true
```

The map node projects LiDAR points into the camera image and attaches RGB values. The saved map.ply and mesh.obj contain vertex colors.

## What Working Looks Like

| Check | Expected result |
|-------|-----------------|
| ping 192.168.123.18 | 0 percent loss, under 2 ms |
| ros2 topic hz /utlidar/cloud_deskewed | about 10 to 15 Hz |
| ros2 topic hz /utlidar/imu | about 250 Hz |
| ros2 topic hz /state_estimation | about 10 Hz |
| Robot standing with remote | Yes |
| Waypoint set in RViz | Robot walks |
| Ctrl+C in map node | Saves files to ~/go2_maps/ |
| Nav2 goal set in RViz | Robot plans and follows path |

## One-Terminal Alternative

If you do not want to run three or four terminals, use the combined launcher.

```bash
./scripts/system_simulation_with_camera.sh
```

This starts the mapping node, slam_toolbox, pointcloud_to_scan, camera relay, and RViz in one command. It is designed for Unity simulation but works with the real robot too if the autonomy stack is already running.

## Failure Modes and Fixes

| Symptom | Cause | Fix |
|---------|-------|-----|
| ping fails | Cable or VM adapter mode | Check Bridged adapter in VM settings |
| ping works, SSH refuses | sshd bound to internal interface | See docs/TROUBLESHOOTING.md |
| Topics exist but no data | QoS mismatch | Run ros2 topic info <topic> -v |
| Point-LIO silent after IMU init | use_sim_time true in utlidar.yaml | Set to false and rebuild |
| /map/occupancy not visible in RViz | Wrong QoS | The node uses TRANSIENT_LOCAL; restart the node |
| Robot does not move | Not standing, not in Sport Mode | Use the physical remote |
| Nav2 goal not reached | Bad localization | Click 2D Pose Estimate again |

## Cross-References

- docs/WIRELESS.md - run the same stack over Wi-Fi
- docs/TROUBLESHOOTING.md - detailed fixes for every failure
- docs/ROADMAP.md - development phases and planned features
```

Save it and commit.

```powershell
git add docs/ETHERNET.md
git commit -m "Update ETHERNET.md with mapping, navigation, and camera workflows"
git push