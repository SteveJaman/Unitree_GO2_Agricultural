# Ethernet Mode

This page covers running the autonomy stack over a direct Ethernet cable between the VM and the Go2 Jetson. Complete docs/SETUP.md first.

## Performance Summary

| Mode | LiDAR rate | Notes |
|------|------------|-------|
| Ethernet | 14.7 Hz | Direct cable, most reliable |
| Wireless | Not measured here | USB Wi-Fi dongle on the Jetson, native DDS |
| Zenoh | Not measured here | TCP bridge for flaky Wi-Fi |
| WebRTC | ~1 Hz | Legacy fallback, degraded |

## Why Ethernet Is Preferred

- A cable removes Wi-Fi packet loss.
- DDS discovery works without a bridge.
- LiDAR arrives at the full 14.7 Hz that SLAM needs.
- No extra hardware is needed on the Jetson.

Use Ethernet whenever the robot can stay tethered. Fall back to the other modes only when it cannot.

## Physical Setup

Connect the VM host and the Go2 with one Ethernet cable.

```
+---------------------+                    +---------------------+
| VM (ROS 2 Humble)   |                    | Go2 Jetson (Foxy)   |
| 192.168.123.100     |<=== Ethernet ====>| 192.168.123.18      |
+---------------------+                    +---------------------+
```

Both ends sit on the same subnet. DDS multicast discovery needs this.

## Configure the VM Ethernet Interface

Find the interface name first. The name differs per machine.

```bash
# List interfaces and their state
ip -br link
```

Assign the static address. Replace <interface> with your Ethernet interface name.

```bash
# Add the VM address on the robot subnet
sudo ip addr add 192.168.123.100/24 dev <interface>

# Bring the link up
sudo ip link set <interface> up
```

Verify the Jetson answers. A reply proves the cable and subnet are correct.

```bash
ping -c 3 192.168.123.18
```

## Make the IP Persistent

The commands above reset on reboot. Use netplan to keep the address.

Open your netplan file. Replace <netplan-file> with the YAML file in /etc/netplan on your machine.

```bash
sudo nano /etc/netplan/<netplan-file>.yaml
```

Add the interface under the existing network block.

```yaml
network:
  version: 2
  ethernets:
    <interface>:
      dhcp4: false
      addresses:
        - 192.168.123.100/24
```

Apply the change.

```bash
sudo netplan apply
```

Disabling DHCP stops the interface from requesting an address on a cable with no DHCP server.

## Verify Jetson Topics Are Visible From the VM

Point the VM shell at the Ethernet DDS config. This pins CycloneDDS to the correct interface. Run from the repo root.

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds_ethernet.xml
```

List topics.

```bash
ros2 topic list
```

You must see topics published by the Jetson. If the list shows only local topics, stop here and open docs/TROUBLESHOOTING.md.

## Launch the Autonomy Stack

Run the Ethernet launch script from the repo root.

```bash
bash scripts/system_real_robot_ethernet.sh
```

The script starts the stack from ~/files/autonomy_stack_go2. That repository is external and must already be built.

## Verify SLAM Output

Open a second terminal. Source ROS 2 and set the same DDS variables as above.

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds_ethernet.xml

ros2 topic hz /state_estimation
```

A steady rate means SLAM is publishing pose. No output means SLAM has not started or has exited.

## Stand the Robot

**Stand the Go2 manually before you send any motion command.**

Stand the Go2 with the method you normally use for the Go2 EDU. A robot that is not standing will not walk. Clear the area around the robot before you continue.

## Send a Waypoint in RViz

1. Switch to RViz.
2. Select the waypoint tool in the toolbar.
3. Click a point on the map in front of the robot.

Start with a short distance. A close target limits the damage if the planner misbehaves.

## Optional: Motion Test With move_forward.sh

Run this test to check the motion path without the planner.

```bash
bash scripts/move_forward.sh
```

The robot must be standing and the area must be clear. The test isolates motor control from SLAM and planning.

## What Working Looks Like

| Check | Expected result |
|-------|-----------------|
| ping 192.168.123.18 | Replies with no loss |
| ros2 topic list on the VM | Jetson topics are present |
| ros2 topic hz /state_estimation | Steady rate, no gaps |
| RViz map | Builds as the robot moves |
| Waypoint sent | Robot walks toward the target |

## Cross-References

- docs/SETUP.md - first-time repo and dependency setup
- docs/TROUBLESHOOTING.md - fixes for CycloneDDS, SLAM, RViz, and motion errors