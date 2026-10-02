# Troubleshooting

Find your symptom. Run the fix. Run the verify step.

Run all commands from the repo root unless stated otherwise.

## 1. ROS 2 Environment

### Package not found

```
Package 'go2_integration_pkg' not found
```

Cause: the workspace is not sourced in this terminal.

1. Source ROS 2.
2. Source the workspace.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Verify:

```bash
ros2 pkg list | grep go2_integration_pkg
```

The package name must appear.

### AMENT_TRACE_SETUP_FILES unbound variable

```
AMENT_TRACE_SETUP_FILES: unbound variable
```

Cause: a script runs with set -u and sources a ROS setup file that reads an unset variable.

1. Export the variable before sourcing.

```bash
export AMENT_TRACE_SETUP_FILES=""
source /opt/ros/humble/setup.bash
```

Verify: the source command returns with no error.

## 2. CycloneDDS

### VM sees no Jetson topics

Symptom: ros2 topic list shows only local topics.

Cause: CycloneDDS is bound to the wrong interface, or the shell does not use the right config.

1. Check the config in this shell.

```bash
echo $RMW_IMPLEMENTATION
echo $CYCLONEDDS_URI
```

2. Set both for Ethernet mode.

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds_ethernet.xml
```

3. Confirm the interface name inside the XML matches your machine.

```bash
ip -br addr
grep -i interface config/cyclonedds_ethernet.xml
```

Use config/cyclonedds_wireless.xml in wireless mode.

Verify:

```bash
ros2 topic list
```

Jetson topics must appear.

### Socket receive buffer warning

```
failed to increase socket receive buffer size
```

Cause: the kernel caps the receive buffer below what CycloneDDS requests. Large LiDAR messages need the larger buffer.

1. Raise the kernel limit.

```bash
sudo sysctl -w net.core.rmem_max=2147483647
```

2. Restart the stack.

Verify: the warning no longer appears at startup.

### No free participant index

```
Failed to find a free participant index for domain 0
```

Cause: too many DDS participants run on one host, or stale ROS 2 processes hold the indexes.

1. List ROS 2 processes.

```bash
ps aux | grep -i ros
```

2. Stop the stale processes. Replace <pid> with a process id from the list.

```bash
kill <pid>
```

Verify: the stack starts with no participant error.

## 3. Point-LIO

### Exits silently after IMU init

Symptom: the Point-LIO process ends after IMU initialization. No error is printed.

Cause: the node does not receive LiDAR data, or it waits on a simulated clock.

1. Check that the Jetson topics reach the VM.

```bash
ros2 topic list
```

2. Check the LiDAR rate. Replace <lidar-topic> with the LiDAR topic from the list.

```bash
ros2 topic hz <lidar-topic>
```

3. Check the clock parameter. Replace <node-name> with the Point-LIO node name from ros2 node list.

```bash
ros2 param get <node-name> use_sim_time
```

On a real robot, use_sim_time must be false.

Verify:

```bash
ros2 topic hz /state_estimation
```

A steady rate means SLAM is running.

## 4. RViz

### TF errors or missing frames

Symptom: RViz shows transform errors, or the robot model does not appear.

Cause: SLAM is not publishing, or the Fixed Frame names a frame that does not exist.

1. Dump the frame tree.

```bash
ros2 run tf2_tools view_frames
```

2. Open the generated frames PDF. Find the root frame.
3. Set the RViz Fixed Frame to that root frame.

Verify: the TF errors clear in the RViz status panel.

## 5. Robot Motion

### Robot does not move

Cause: the robot is not standing, or no velocity commands arrive.

1. Stand the Go2 manually. A robot that is not standing will not walk.
2. Watch the command topic.

```bash
ros2 topic echo /cmd_vel
```

3. Send a waypoint in RViz. Watch for output.

If /cmd_vel stays silent, the planner is not running. Check section 3 and confirm /state_estimation publishes.

4. Run the motion test to isolate the problem.

```bash
bash scripts/move_forward.sh
```

Verify: the robot walks forward during the test.

## 6. Wireless Hardware

### No Wi-Fi on the Jetson

Cause: the USB dongle is not detected or has no driver.

1. SSH to the Jetson. Replace <user> with the Jetson login.

```bash
ssh <user>@192.168.123.18
```

2. Check that the dongle is on the USB bus.

```bash
lsusb
```

3. Check for a wireless interface.

```bash
ip -br link
```

4. Read kernel messages for driver errors.

```bash
dmesg | tail -n 50
```

Verify: a wireless interface appears and holds an address in the 192.168.137.x range.

## 7. SSH

### Permission denied

```
Permission denied (publickey,password).
```

Cause: wrong user, wrong password, or the key is not installed on the target.

1. Confirm the user name.
2. Copy your key to the target. Replace <user> with the login.

```bash
ssh-copy-id <user>@192.168.123.18
```

Verify: ssh logs in.

### Connection refused

```
ssh: connect to host 192.168.123.18 port 22: Connection refused
```

Cause: the host is reachable but no SSH server answers.

1. Check reachability.

```bash
ping -c 3 192.168.123.18
```

2. If ping succeeds, start the SSH service on the target.

```bash
sudo systemctl start ssh
```

Verify: ssh logs in.

## 8. Git

### 403 denied on push or clone

```
fatal: unable to access '<url>': The requested URL returned error: 403
```

Cause: GitHub no longer accepts account passwords over HTTPS.

1. Create a personal access token in your GitHub account settings, or add an SSH key.
2. For a token, use it as the password when Git prompts.
3. For SSH, switch the remote. Replace <owner> and <repo>.

```bash
git remote set-url origin git@github.com:<owner>/<repo>.git
```

Verify:

```bash
git fetch
```

The command completes with no error.

## 9. Zenoh

### DDS looping

Symptom: the same messages repeat. Topic rates are far above normal.

Cause: native DDS discovery and the Zenoh bridge both carry the same traffic.

1. Stop every running ROS 2 and bridge process.
2. Start the stack in one mode only.
3. Start the bridge on the robot.

```bash
bash scripts/zenoh_bridge_robot.sh
```

4. Start the bridge on the VM.

```bash
bash scripts/zenoh_bridge_vm.sh
```

Verify: topic rates match the expected values.

### Bridge does not connect

Cause: the endpoint in the json5 config is wrong, or the network blocks the link.

1. Check that the VM reaches the robot over Wi-Fi.

```bash
ping -c 3 192.168.137.33
```

2. Compare the endpoint addresses in config/zenoh_robot_config.json5 and config/zenoh_vm_config.json5 with the IPs on your machines.
3. Restart both bridges. Start the robot side first.

Verify: the bridge logs show an established session and ros2 topic list shows robot topics.

## 10. Performance

### LiDAR near 1 Hz

Cause: the system runs in WebRTC mode. WebRTC delivers about 1 Hz LiDAR.

1. Switch to Ethernet mode. See docs/ETHERNET.md.
2. If Ethernet is not possible, use wireless or Zenoh mode.

Verify: the LiDAR rate rises. Ethernet delivers 14.7 Hz.

### CPU at 100 percent

Cause: a runaway process, or software rendering in RViz.

1. Find the process.

```bash
top -o %CPU
```

2. Stop duplicate ROS 2 processes.
3. Close RViz displays you do not need.

Verify: top shows CPU below saturation.

## 11. General

### Permission denied on a script

```
bash: scripts/move_forward.sh: Permission denied
```

Cause: the executable bit is missing.

```bash
chmod +x scripts/*.sh
```

Verify: ls -l scripts shows x on each script.

### Bad interpreter with CRLF

```
/bin/bash^M: bad interpreter
```

Cause: the script has Windows line endings.

```bash
sed -i 's/\r$//' scripts/*.sh
```

Verify: the script runs with no interpreter error.

### RViz is slow

Cause: the VM renders in software.

1. Disable displays you do not need.
2. Lower the point cloud size and decay time.
3. Enable 3D acceleration in the VM settings if available.

Verify: the RViz frame rate improves.

## Reporting a New Issue

Gather this before you report.

- The exact error text from the terminal
- The connection mode in use
- Output of echo $RMW_IMPLEMENTATION and echo $CYCLONEDDS_URI
- Output of ros2 topic list
- Output of ip -br addr on the VM and the Jetson
- The command you ran and the script you launched
- Whether the robot was standing