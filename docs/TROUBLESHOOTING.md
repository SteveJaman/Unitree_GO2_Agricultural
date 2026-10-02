# Wireless Mode

Untethered operation. Three approaches, in order of preference.

## Performance Comparison

| Approach | LiDAR rate | Setup | Requires |
|----------|-----------|-------|----------|
| Native Wi-Fi DDS | about 14 Hz | Medium | USB dongle on Jetson |
| Zenoh bridge | about 10 Hz | High | USB dongle plus Zenoh install |
| WebRTC fallback | about 1 Hz | Low | Front board only |

Native Wi-Fi DDS is the target. It behaves like Ethernet once configured. Zenoh is a bridge for flaky links. WebRTC is a last resort.

## The Hardware Blocker

The Go2 has two computers inside.

```
+--------------------------------------+
|  Go2 robot                           |
|                                      |
|  +-----------------------+           |
|  | Front board           |           |  has Wi-Fi
|  | (microcontroller)     |           |  speaks WebRTC
|  | 192.168.137.33        |           |
|  +-----------+-----------+           |
|              | internal Ethernet     |
|  +-----------+-----------+           |
|  | Jetson (NVIDIA Orin)  |           |  NO Wi-Fi
|  | runs ROS 2 Foxy       |           |  only eth0
|  | 192.168.123.18        |           |
|  +-----------------------+           |
+--------------------------------------+
```

The Jetson hosts the LiDAR, the IMU, and Point-LIO. It has no Wi-Fi chipset.

Only the front board has Wi-Fi, and it speaks WebRTC, not native DDS. It does not route SSH or DDS traffic to the Jetson.

Any wireless approach that uses native DDS requires adding a Wi-Fi interface to the Jetson.

## Hardware Options

### Option 1 - USB Wi-Fi dongle on the Jetson

Plug the dongle into the Jetson. It becomes wlan0. This gives the robot a native Wi-Fi interface.

Recommended dongles:

| Dongle | Chipset | Notes |
|--------|---------|-------|
| Panda PAU09 | Ralink RT5572 | Best compatibility, dual-band |
| Alfa AWUS036NHA | Atheros AR9271 | Long range |
| TP-Link TL-WN722N v1 | Atheros AR9271 | Cheapest, must be version 1 |

Avoid TP-Link v2 and v3. They use a different chipset with no in-kernel driver.

Avoid Realtek RTL8812BU. It requires compiling a custom driver on the Jetson.

Confirm the Go2 has a reachable USB port. Most EDU units have one on the side or under the top shell.

### Option 2 - Travel router

A small travel router creates a private Wi-Fi network and provides Ethernet ports for wired devices.

```
Travel router
   |
   |--- (Wi-Fi) --> Go2 Jetson with USB dongle
   |--- (Ethernet LAN) --> workstation
```

This solves the case where the workstation has no Wi-Fi chipset. The workstation connects to the router by Ethernet. The Jetson connects to the router by Wi-Fi.

Recommended router: GL.iNet Beryl AX (GL-MT3000). About 60 USD. Wi-Fi 6, four Ethernet ports.

### Option 3 - Campus network

If the campus Ethernet wall port and the campus Wi-Fi are on the same subnet, both machines can reach each other without extra hardware.

Test first. Plug the workstation into a wall port. Connect the Jetson to the campus Wi-Fi. Try `ping` and `ros2 topic list` from the workstation.

If ping works but topics do not appear, the campus blocks multicast and you need Option 1 or 2.

## Step 1 - Configure Wi-Fi on the Jetson

SSH into the Jetson over Ethernet.

```bash
ssh unitree@192.168.123.18
```

Plug in the dongle. Verify it is detected.

```bash
lsusb
ip link
```

A new interface such as wlan0 should appear.

Install NetworkManager if needed.

```bash
sudo apt update
sudo apt install -y network-manager wpasupplicant
sudo systemctl enable --now NetworkManager
```

Connect to the Wi-Fi network.

```bash
sudo nmcli device wifi rescan
sudo nmcli device wifi list
sudo nmcli device wifi connect "YourSSID" password "YourPassword"
```

Verify the interface has an IP.

```bash
ip addr show wlan0 | grep inet
```

Example output:

```
inet 192.168.137.50/24
```

Make the connection automatic.

```bash
sudo nmcli connection modify "YourSSID" connection.autoconnect yes
sudo nmcli connection modify "YourSSID" connection.autoconnect-priority 100
```

## Step 2 - Update the Wireless DDS Config

Edit config/cyclonedds_wireless.xml.

Two values must be set.

1. NetworkInterface name to the workstation's Wi-Fi adapter, for example wlan0.
2. Peer addresses to the Wi-Fi IPs of both machines.

Example:

```xml
<General>
  <Interfaces>
    <NetworkInterface name="wlan0" priority="default" multicast="default" />
  </Interfaces>
  <AllowMulticast>spdp</AllowMulticast>
  <EnableMulticastLoopback>false</EnableMulticastLoopback>
</General>
<Discovery>
  <Peers>
    <Peer address="192.168.137.50"/>
    <Peer address="192.168.137.100"/>
  </Peers>
  <ParticipantIndex>auto</ParticipantIndex>
  <MaxAutoParticipantIndex>500</MaxAutoParticipantIndex>
</Discovery>
```

Replace the addresses with the actual IPs on your network.

If the access point blocks multicast, change `<AllowMulticast>spdp</AllowMulticast>` to `<AllowMulticast>false</AllowMulticast>`. The explicit peers are then the only discovery path.

## Step 3 - Verify the Jetson Topics

SSH into the Jetson over Wi-Fi.

```bash
ssh unitree@192.168.137.50
```

List topics.

```bash
source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

ros2 topic list | grep utlidar
```

Expected topics:

```
/utlidar/cloud
/utlidar/cloud_deskewed
/utlidar/imu
/utlidar/robot_odom
```

From the workstation, verify the same topics appear.

```bash
source /opt/ros/humble/setup.bash
source ~/files/Unitree_GO2_Agricultural/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/files/Unitree_GO2_Agricultural/config/cyclonedds_wireless.xml"
export ROS_DOMAIN_ID=0

ros2 topic list | grep utlidar
ros2 topic hz /utlidar/cloud_deskewed
```

Expected rate: about 10 Hz. The LiDAR rate is slightly lower over Wi-Fi than over Ethernet.

## Step 4 - Launch the Autonomy Stack

The repo does not ship a dedicated wireless launcher. Run the autonomy stack manually with the wireless DDS config.

```bash
source /opt/ros/humble/setup.bash
source ~/files/autonomy_stack_go2/install/setup.bash
source ~/files/Unitree_GO2_Agricultural/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/files/Unitree_GO2_Agricultural/config/cyclonedds_wireless.xml"
export ROS_DOMAIN_ID=0

ros2 launch vehicle_simulator system_real_robot.launch
```

RViz opens. The point cloud, robot pose, and terrain map appear.

If the same command is needed repeatedly, create a wrapper script in the repo.

```bash
#!/usr/bin/env bash
# system_real_robot_wireless.sh
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
CYCLONEDDS_XML="${WS_ROOT}/config/cyclonedds_wireless.xml"
AUTONOMY_WS="$HOME/files/autonomy_stack_go2"

set +u
source /opt/ros/humble/setup.bash
source "${AUTONOMY_WS}/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${CYCLONEDDS_XML}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "[wireless] CYCLONEDDS_URI = ${CYCLONEDDS_URI}"
exec ros2 launch vehicle_simulator system_real_robot.launch
```

Save as scripts/system_real_robot_wireless.sh. Make it executable.

```bash
chmod +x scripts/system_real_robot_wireless.sh
```

## Step 5 - Run Mapping Over Wi-Fi

Once the autonomy stack is up, run the map node in a second terminal.

```bash
source /opt/ros/humble/setup.bash
source ~/files/Unitree_GO2_Agricultural/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/files/Unitree_GO2_Agricultural/config/cyclonedds_wireless.xml"
export ROS_DOMAIN_ID=0

ros2 launch go2_integration_pkg mapping.launch.py
```

The node auto-detects the cloud and odom topics. It saves the map to ~/go2_maps/<timestamp>/ on Ctrl+C.

## Step 6 - Run Navigation Over Wi-Fi

Use the navigation script with the wireless DDS config.

```bash
cd ~/files/Unitree_GO2_Agricultural

export CYCLONEDDS_URI="file://$HOME/files/Unitree_GO2_Agricultural/config/cyclonedds_wireless.xml"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

./scripts/system_navigation.sh ~/go2_maps/my_map.yaml
```

The same workflow as Ethernet. The only difference is the transport.

## Zenoh Bridge (When Wi-Fi Is Flaky)

Zenoh is a protocol designed for unreliable networks. It tunnels DDS traffic over TCP. This is more resilient to packet loss than native UDP multicast.

Use it when native DDS over Wi-Fi drops data or loses discovery.

### Install Zenoh on both machines

On the Jetson (via SSH over Ethernet or Wi-Fi):

```bash
curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | \
    sudo gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg

echo "deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /" | \
    sudo tee -a /etc/apt/sources.list > /dev/null

sudo apt update
sudo apt install -y zenoh-bridge-ros2dds
```

On the workstation, run the same commands. The package is available for both x86 and ARM.

### Why different DDS domains

Zenoh documentation warns that no direct DDS communication should occur between two bridged hosts. Otherwise duplicate and looping traffic can appear.

Use different ROS_DOMAIN_ID on each side.

- Jetson: ROS_DOMAIN_ID=0
- Workstation: ROS_DOMAIN_ID=42

DDS stays local to each machine. Only Zenoh crosses the Wi-Fi link.

### Run the Zenoh bridge on the Jetson

```bash
ssh unitree@192.168.137.50

source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

zenoh-bridge-ros2dds
```

This starts a Zenoh router that listens on TCP port 7447 by default.

### Run the Zenoh client on the workstation

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42

zenoh-bridge-ros2dds -e tcp/192.168.137.50:7447
```

Replace 192.168.137.50 with the Jetson Wi-Fi IP.

### Verify

In a third terminal on the workstation:

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42

ros2 topic list | grep utlidar
ros2 topic hz /utlidar/cloud_deskewed
```

The Jetson topics appear on the workstation's local DDS domain 42, tunneled over Zenoh.

### Run the autonomy stack with Zenoh

The autonomy stack must use ROS_DOMAIN_ID=42 so it reads from the Zenoh client.

```bash
source /opt/ros/humble/setup.bash
source ~/files/autonomy_stack_go2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42

ros2 launch vehicle_simulator system_real_robot.launch
```

## WebRTC Fallback (Degraded)

Use only when no dongle is available and no travel router can be used.

This mode uses the front board's Wi-Fi. The Jetson stays off the network. WebRTC carries a decoded LiDAR stream to the workstation.

Performance is about 1 Hz LiDAR. Point-LIO may not initialize. Navigation is unreliable.

### Disable video in the WebRTC SDK

The SDK's H.264 decoder consumes 100 percent of one CPU core. Disable it to free CPU for the LiDAR decoder.

Edit ~/files/ros2_ws/src/go2_robot_sdk/go2_robot_sdk/launch/robot.launch.py. Find the go2_driver_node block. Add one parameter.

```python
parameters=[{
    'robot_ip': self.config.robot_ip,
    'token': self.config.robot_token,
    'conn_type': self.config.conn_type,
    'enable_video': False,
}],
```

Rebuild.

```bash
cd ~/files/ros2_ws
colcon build --packages-select go2_robot_sdk --symlink-install
source install/setup.bash
```

### Start the WebRTC SDK

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/files/ros2_ws/install/setup.bash

export ROBOT_IP="192.168.137.33"
export AES_KEY="<your AES key>"
export ROBOT_AES_KEY="<your AES key>"
export CONN_TYPE="webrtc"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/files/Unitree_GO2_Agricultural/config/cyclonedds_wireless.xml"
export ROS_DOMAIN_ID=0

ros2 launch go2_robot_sdk robot.launch.py \
    rviz2:=false nav2:=false slam:=false \
    foxglove:=false joystick:=false teleop:=false
```

Wait for `Robot 0 validated and ready`.

### Start the relay nodes

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/files/Unitree_GO2_Agricultural/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/files/Unitree_GO2_Agricultural/config/cyclonedds_wireless.xml"
export ROS_DOMAIN_ID=0

ros2 launch go2_integration_pkg integrated_robot.launch.py
```

This starts cloud_relay_node and cmd_vel_bridge. It renames /point_cloud2 to /utlidar/cloud and converts TwistStamped to Twist.

### Start the autonomy stack

Terminal 3:

```bash
source /opt/ros/humble/setup.bash
source ~/files/autonomy_stack_go2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/files/Unitree_GO2_Agricultural/config/cyclonedds_wireless.xml"
export ROS_DOMAIN_ID=0

ros2 launch vehicle_simulator system_real_robot.launch
```

### What to expect

- LiDAR rate about 1 Hz. Point-LIO may not converge.
- IMU about 250 Hz. Fine.
- Camera available as /camera/image_raw.
- Control commands work but latency is high.

This mode is for inspection and debugging, not for autonomous navigation.

## Performance Tuning

Apply these on both the Jetson and the workstation.

### Raise kernel socket buffers

Large LiDAR messages drop when the OS buffer is too small.

```bash
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.core.wmem_max=2147483647

echo "net.core.rmem_max=2147483647" | sudo tee -a /etc/sysctl.d/60-cyclonedds.conf
echo "net.core.wmem_max=2147483647" | sudo tee -a /etc/sysctl.d/60-cyclonedds.conf
sudo sysctl --system
```

### Disable Wi-Fi power saving

Power save throttles the radio and causes latency spikes.

```bash
sudo iw dev wlan0 set power_save off
```

Replace wlan0 with the actual interface name. This setting is lost on reboot. Add it to /etc/rc.local or a systemd service for persistence.

### Use a dedicated network

The more devices on the Wi-Fi network, the more contention. A dedicated travel router with only the robot and workstation on it performs better than a shared campus network.

## What Working Looks Like

| Check | Expected result |
|-------|-----------------|
| ssh unitree@<jetson-wifi-ip> | Connects over Wi-Fi |
| ros2 topic hz /utlidar/cloud_deskewed | About 10 Hz |
| ros2 topic hz /utlidar/imu | About 250 Hz |
| ros2 topic hz /state_estimation | About 10 Hz |
| Robot standing with remote | Yes |
| Waypoint set in RViz | Robot walks |

## Failure Modes and Fixes

| Symptom | Cause | Fix |
|---------|-------|-----|
| Jetson has no wlan0 | No Wi-Fi chipset | Add USB dongle or travel router |
| Dongle not detected | Wrong port or unsupported chipset | Try different port, use recommended dongle |
| Topics exist but no data | Multicast blocked | Set explicit peers, disable multicast |
| Topics do not appear at all | Different RMW or domain | Match RMW_IMPLEMENTATION and ROS_DOMAIN_ID |
| Ping works, DDS fails | Firewall or multicast isolation | Use travel router or Zenoh |
| LiDAR rate drops below 5 Hz | Wi-Fi contention | Dedicated network, increase buffers, disable power save |
| Zenoh bridge loops traffic | Same DDS domain on both sides | Use different ROS_DOMAIN_ID on each machine |
| WebRTC LiDAR under 2 Hz | H.264 decoder starved | Disable video in the SDK launch file |

## Cross-References

- docs/ethernet.md - the recommended mode when a cable is possible
- docs/troubleshooting.md - fixes for DDS, mapping, Nav2, and camera issues
- docs/roadmap.md - development phases including wireless deployment