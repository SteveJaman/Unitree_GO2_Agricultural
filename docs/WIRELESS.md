# Wireless Mode

This page covers running the autonomy stack over Wi-Fi with native DDS. A USB Wi-Fi dongle on the Go2 Jetson joins the same network as the VM. Complete docs/SETUP.md first.

## Performance Summary

| Mode | LiDAR rate | Notes |
|------|------------|-------|
| Ethernet | 14.7 Hz | Direct cable, most reliable |
| Wireless | Not measured here | USB Wi-Fi dongle on the Jetson, native DDS |
| Zenoh | Not measured here | TCP bridge for flaky Wi-Fi |
| WebRTC | ~1 Hz | Legacy fallback, degraded |

## When to Use Wireless

- Use Wireless when the robot cannot stay tethered.
- Use Ethernet instead if a cable is possible. See docs/ETHERNET.md.
- Use the Zenoh bridge if Wi-Fi drops packets and DDS discovery fails.

Wireless uses native DDS over UDP. Lost packets hurt it more than a wired link.

## Addresses Used in This Page

| Device | Address |
|--------|---------|
| Go2 Jetson (Ethernet, for SSH setup) | 192.168.123.18 |
| Go2 front board (Wi-Fi) | 192.168.137.33 |
| VM (Wi-Fi) | 192.168.137.201 |
| Jetson Wi-Fi (dongle) | 192.168.137.50 (example) |

The dongle address is an example. Your network may assign a different one. Use the address your Jetson actually receives.

## Network Layout

```
                +------------------+
                |  Wi-Fi network   |
                |  192.168.137.x   |
                +------------------+
                  |              |
        +---------+              +---------+
        |                                  |
+-------------------+          +---------------------+
| VM (ROS 2 Humble) |          | Go2 Jetson (Foxy)   |
| 192.168.137.201   |          | dongle: 192.168.137.50 |
+-------------------+          +---------------------+
```

All devices must share one subnet. DDS multicast discovery needs this.

## Prerequisites

- A USB Wi-Fi dongle plugged into the Jetson
- The VM and the Jetson both on the same Wi-Fi network
- A working Ethernet connection to the Jetson for the one-time setup (see docs/ETHERNET.md)

## Confirm the Dongle on the Jetson

Connect to the Jetson over Ethernet. Replace <user> with the Jetson login.

```bash
ssh <user>@192.168.123.18
```

Check that the dongle is on the USB bus.

```bash
lsusb
```

Check for a wireless interface.

```bash
ip -br link
```

If no wireless interface appears, stop and open docs/TROUBLESHOOTING.md, section 6.

## Join the Wi-Fi Network From the Jetson

Replace <ssid> and <password> with your network values. Run on the Jetson.

```bash
nmcli device wifi connect "<ssid>" password "<password>"
```

Check the address.

```bash
ip -br addr
```

The dongle must hold an address in the 192.168.137.x range. The ROS 2 stack binds to this interface in wireless mode.

## Verify the Link From the VM

Run on the VM. Replace <jetson-wifi-ip> with the address the dongle received.

```bash
ping -c 3 <jetson-wifi-ip>
```

Replies prove the VM and the Jetson share the network.

Also check the front board.

```bash
ping -c 3 192.168.137.33
```

## Select the Wireless DDS Config

Point the VM shell at the wireless DDS config. This pins CycloneDDS to the Wi-Fi interface. Run from the repo root.

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds_wireless.xml
```

Check that the interface named in the config matches your VM Wi-Fi interface.

```bash
ip -br addr
grep -i interface config/cyclonedds_wireless.xml
```

A mismatch is the most common cause of missing topics.

## Verify Jetson Topics Are Visible From the VM

```bash
ros2 topic list
```

You must see topics published by the Jetson. If the list shows only local topics, open docs/TROUBLESHOOTING.md, section 2.

## Launch the Autonomy Stack

Run the wireless launch script from the repo root.

```bash
bash scripts/system_real_robot_wireless.sh
```

The script starts the stack from ~/files/autonomy_stack_go2. That repository is external and must already be built.

## Verify SLAM Output

Open a second terminal. Source ROS 2 and set the same DDS variables as above.

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds_wireless.xml

ros2 topic hz /state_estimation
```

A steady rate means SLAM is publishing pose. Gaps mean packet loss. Move closer to the access point or switch to the Zenoh bridge.

## Stand the Robot

**Stand the Go2 manually before you send any motion command.**

A robot that is not standing will not walk. Clear the area around the robot. A wireless link can drop at any time, so keep the robot in sight and keep the stop method within reach.

## Send a Waypoint in RViz

1. Switch to RViz.
2. Select the waypoint tool in the toolbar.
3. Click a point on the map in front of the robot.

Start with a short distance. A close target limits the damage if the link drops mid-move.

## What Working Looks Like

| Check | Expected result |
|-------|-----------------|
| lsusb on the Jetson | Dongle is listed |
| Jetson dongle address | In the 192.168.137.x range |
| ping from the VM | Replies with no loss |
| ros2 topic list on the VM | Jetson topics are present |
| ros2 topic hz /state_estimation | Steady rate, no long gaps |
| Waypoint sent | Robot walks toward the target |

## Cross-References

- docs/SETUP.md - first-time repo and dependency setup
- docs/ETHERNET.md - the preferred wired mode and Jetson access
- docs/TROUBLESHOOTING.md - fixes for dongle, CycloneDDS, and SLAM errors