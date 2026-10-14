# Architecture

This document is the reference for how data moves through Unitree_GO2_Agricultural. It lists every node, topic, message type, QoS policy, and Unitree Sport API command the system uses. Use it to debug data flow and to extend the stack.

The repo is the integration layer between the Unitree Go2 EDU and the CMU autonomy_stack_go2 SLAM and planner stack. It wires sensor acquisition, mapping, SLAM, localisation, and Nav2 navigation into one pipeline.

## The Three Communication Layers

Data moves at three levels. Know which level a fault sits in before you debug it.

### Layer 1 - Inside a Process

Pure Python function calls in `core/mapping.py`. No ROS is involved.

- The mapping logic runs without a ROS install.
- It runs on Windows for offline testing.
- Nodes such as `map_node.py` wrap it and add the ROS interfaces.

### Layer 2 - Between ROS 2 Nodes

Nodes talk over topics. Each topic has a strict contract of three items:

1. Topic name
2. Message type
3. QoS profile

A mismatch in any item fails silently. The topic appears in `ros2 topic list`, but no data arrives. See [QoS Policy Reference](#qos-policy-reference).

### Layer 3 - Between Computers

CycloneDDS carries ROS 2 traffic between the VM and the Go2 Jetson over Ethernet or Wi-Fi.

The XML configs bind DDS to one network interface and name the discovery peers.

| Mode | Config file | Addresses |
|------|-------------|-----------|
| Ethernet | `config/cyclonedds_ethernet.xml` | VM 192.168.123.100, Jetson 192.168.123.18 |
| Wireless | `config/cyclonedds_wireless.xml` | VM 192.168.137.201, front board 192.168.137.33 |

Structure of a CycloneDDS config. Replace ETH_INTERFACE and PEER_IP with your values. The real values live in the files above.

```xml
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="ETH_INTERFACE" />
      </Interfaces>
    </General>
    <Discovery>
      <Peers>
        <Peer address="PEER_IP" />
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

Select a config in every terminal before you run ROS 2 commands.

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds_ethernet.xml
```

## Node Inventory

| Node | Source file | Subscribes to | Publishes to |
|------|-------------|---------------|--------------|
| `move_forward.py` | `go2_integration_pkg` | none | `/api/sport/request` |
| `cloud_relay_node.py` | `go2_integration_pkg` | `/point_cloud2` | `/utlidar/cloud` |
| `cmd_vel_bridge.py` | `go2_integration_pkg` | `/cmd_vel_stamped` | `/cmd_vel` |
| `map_node.py` | `go2_integration_pkg` | cloud topic (auto-detected), odom topic (auto-detected), optional `/camera/image_raw`, optional `/camera/camera_info` | `/map/occupancy`, `/map/points` |
| `pointcloud_to_scan.py` | `go2_integration_pkg` | cloud topic (auto-detected) | `/scan` |
| `camera_relay.py` | `go2_integration_pkg` | `/camera/image_raw`, `/camera/camera_info` | `/camera/image_raw_relayed`, `/camera/camera_info_relayed` |
| `map_saver` (Nav2) | external | `/map` | writes `map.yaml` + `map.pgm` |
| `map_server` (Nav2) | external | none | `/map`, `/map_metadata` |
| `amcl` (Nav2) | external | `/scan`, `/initialpose` | `/amcl_pose`, TF `map->odom` |
| `slam_toolbox` | external | `/scan`, TF | `/map`, `/map_updates`, TF `map->odom` |
| `controller_server` (Nav2) | external | `/plan`, `/local_costmap/costmap` | `/cmd_vel` |
| `planner_server` (Nav2) | external | `/goal_pose`, `/global_costmap/costmap` | `/plan` |
| `bt_navigator` (Nav2) | external | `/goal_pose`, `/navigate_to_pose/_action/*` | action results |

Auto-detection means the node picks the cloud and odom topics that exist at startup. The candidates are:

- Cloud: `/registered_scan`, `/cloud_registered`, `/utlidar/cloud_deskewed`, `/utlidar/cloud`
- Odom: `/state_estimation`, `/utlidar/robot_odom`, `/utlidar/robot_pose`

## Complete Topic Contract

| Topic | Message type | Publisher | Subscriber(s) | QoS | Rate |
|-------|--------------|-----------|---------------|-----|------|
| `/utlidar/cloud` | `sensor_msgs/PointCloud2` | Go2 Jetson firmware | cloud relays, Point-LIO | RELIABLE, VOLATILE, depth 1 | ~15 Hz |
| `/utlidar/cloud_deskewed` | `sensor_msgs/PointCloud2` | Go2 Jetson firmware | map_node | RELIABLE, VOLATILE, depth 1 | ~15 Hz |
| `/utlidar/imu` | `sensor_msgs/Imu` | Go2 Jetson firmware | Point-LIO | RELIABLE, VOLATILE, depth 1 | ~250 Hz |
| `/utlidar/robot_odom` | `nav_msgs/Odometry` | Go2 Jetson firmware | map_node (fallback) | RELIABLE, VOLATILE | ~50 Hz |
| `/utlidar/robot_pose` | `geometry_msgs/PoseStamped` | Go2 Jetson firmware | map_node (fallback) | RELIABLE, VOLATILE | ~50 Hz |
| `/registered_scan` | `sensor_msgs/PointCloud2` | Point-LIO | map_node, pointcloud_to_scan | BEST_EFFORT, VOLATILE, depth 5 | ~10 Hz |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | Point-LIO (alternate name) | map_node | BEST_EFFORT, VOLATILE, depth 5 | ~10 Hz |
| `/state_estimation` | `nav_msgs/Odometry` | Point-LIO | map_node, RViz | BEST_EFFORT, VOLATILE, depth 10 | ~10 Hz |
| `/scan` | `sensor_msgs/LaserScan` | pointcloud_to_scan | slam_toolbox, AMCL, costmaps | BEST_EFFORT, VOLATILE, depth 5 | ~10 Hz |
| `/cmd_vel` | `geometry_msgs/TwistStamped` (Ethernet/sim) or `geometry_msgs/Twist` (WebRTC) | pathFollower or Nav2 controller | Go2 firmware or cmd_vel_bridge | RELIABLE, VOLATILE, depth 10 | variable |
| `/api/sport/request` | `unitree_api/Request` | move_forward, pathFollower | Go2 firmware | RELIABLE, VOLATILE | 50 Hz while active |
| `/map/occupancy` | `nav_msgs/OccupancyGrid` | map_node | RViz | RELIABLE, **TRANSIENT_LOCAL**, depth 1 | 2 Hz |
| `/map/points` | `sensor_msgs/PointCloud2` | map_node | RViz | BEST_EFFORT, VOLATILE, depth 1 | 2 Hz |
| `/map` | `nav_msgs/OccupancyGrid` | slam_toolbox or map_server | AMCL, costmaps, RViz | RELIABLE, TRANSIENT_LOCAL | 1 Hz |
| `/camera/image_raw` | `sensor_msgs/Image` | WebRTC SDK | camera_relay | BEST_EFFORT, VOLATILE, depth 2 | 15 Hz |
| `/camera/image_raw_relayed` | `sensor_msgs/Image` | camera_relay | RViz | BEST_EFFORT, VOLATILE, depth 2 | 15 Hz |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | WebRTC SDK | camera_relay, map_node | BEST_EFFORT, VOLATILE | 15 Hz |
| `/point_cloud2` | `sensor_msgs/PointCloud2` | WebRTC SDK | cloud_relay_node | BEST_EFFORT, VOLATILE | ~10 Hz (WebRTC only) |

Rates are approximate. Measure the real rate with `ros2 topic hz <topic>`.

## QoS Policy Reference

| Policy | Value | Where used | Why |
|--------|-------|------------|-----|
| Reliability | RELIABLE | Go2 firmware topics, control, map | Guarantees delivery for safety-critical or cached data |
| Reliability | BEST_EFFORT | Sensor streams (cloud, scan, image), Point-LIO output | Timeliness over retry. The next frame arrives in 100 ms anyway |
| Durability | VOLATILE | All sensor streams and control | Only currently connected subscribers receive data |
| Durability | TRANSIENT_LOCAL | `/map`, `/map/occupancy` | RViz may subscribe late and must still receive the map |
| History | KEEP_LAST depth 1 | Map and occupancy | Only the latest map matters |
| History | KEEP_LAST depth 5 | Sensor streams | Small buffer for jitter |
| History | KEEP_LAST depth 10 | Control commands | Small buffer without backlog |

### The Golden Rule

A BEST_EFFORT subscriber can receive from a RELIABLE publisher. A RELIABLE subscriber cannot receive from a BEST_EFFORT publisher.

If the contracts mismatch, the connection silently refuses to form. No error is printed.

| Publisher | Subscriber | Connects |
|-----------|------------|----------|
| RELIABLE | RELIABLE | Yes |
| RELIABLE | BEST_EFFORT | Yes |
| BEST_EFFORT | BEST_EFFORT | Yes |
| BEST_EFFORT | RELIABLE | No |
| VOLATILE | TRANSIENT_LOCAL | No |
| TRANSIENT_LOCAL | VOLATILE | Yes, but no late-join history |

Check both ends of a topic with:

```bash
ros2 topic info <topic> -v
```

### TRANSIENT_LOCAL Publisher Example

A latched map topic needs all three settings. This is the profile `/map/occupancy` uses.

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

map_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
```

## Unitree Sport API Command Reference

The robot takes motion commands on `/api/sport/request`. The message type is `unitree_api/Request`.

The values in this section come from the Unitree Sport API definitions. Availability depends on the robot model and firmware version. Confirm each ID against the SDK version on your robot before you use it.

### Request Message Structure

| Field | Type | Meaning |
|-------|------|---------|
| `header.identity.id` | int64 | Request id. Use a unique value to match a response |
| `header.identity.api_id` | int64 | Selects the command |
| `header.lease.id` | int64 | Lease id. Leave 0 for sport commands |
| `header.policy.priority` | int32 | Request priority |
| `header.policy.noreply` | bool | Set true to skip the response |
| `parameter` | string | Stringified JSON payload for the command |
| `binary` | uint8[] | Raw payload for commands that need one |

Each command is one `api_id` plus one JSON `parameter`.

### Posture and Locomotion Commands

| API ID | Command | Parameter JSON | What it does |
|--------|---------|----------------|--------------|
| 1001 | Damp | `{}` | Relax all motors. The robot goes limp |
| 1002 | BalanceStand | `{}` | Hold a balanced standing pose |
| 1003 | StopMove | `{}` | Stop locomotion. The robot keeps standing |
| 1004 | StandUp | `{}` | Stand up from lying |
| 1005 | StandDown | `{}` | Lower to lying |
| 1006 | RecoveryStand | `{}` | Stand up from any fallen pose |
| 1007 | Euler | `{"x": 0.0, "y": 0.0, "z": 0.0}` | Set body roll, pitch, yaw in rad while standing |
| 1008 | **Move** | `{"x": 0.3, "y": 0.0, "z": 0.0}` | Walk with body velocity |
| 1009 | Sit | `{}` | Sit down |
| 1010 | RiseSit | `{}` | Rise from the sitting pose |
| 1011 | SwitchGait | `{"data": 1}` | Change gait |
| 1012 | Trigger | `{}` | Execute a stored action |
| 1013 | BodyHeight | `{"data": 0.0}` | Set body height offset in m |
| 1014 | FootRaiseHeight | `{"data": 0.0}` | Set foot raise height offset in m |
| 1015 | SpeedLevel | `{"data": 0}` | Set speed level: -1 slow, 0 normal, 1 fast |
| 1019 | ContinuousGait | `{"data": true}` | Keep stepping in place when idle |
| 1028 | Pose | `{"data": true}` | Enter or leave the pose mode used with Euler |
| 1035 | EconomicGait | `{"data": true}` | Switch to the energy-saving gait |

SwitchGait values:

| Value | Gait |
|-------|------|
| 0 | idle |
| 1 | trot |
| 2 | trot running |
| 3 | climb stair |
| 4 | trot obstacle |

Documented limits for the numeric parameters:

| Command | Axis or field | Range |
|---------|---------------|-------|
| Euler | roll, pitch | -0.75 to 0.75 rad |
| Euler | yaw | -0.6 to 0.6 rad |
| BodyHeight | data | -0.18 to 0.03 m |
| FootRaiseHeight | data | -0.06 to 0.03 m |

Send Euler only while the robot balances in place. Send BalanceStand first.

### Move Command Axes

The Move command (1008) takes a body-frame velocity. The axes follow the robot, not the map.

| Axis | Meaning | Unit | Sign |
|------|---------|------|------|
| `x` | Forward velocity | m/s | Positive = forward, negative = backward |
| `y` | Lateral velocity | m/s | Positive = left, negative = right |
| `z` | Yaw rate | rad/s | Positive = counterclockwise |

Documented limits are x -2.5 to 3.8 m/s, y -1.0 to 1.0 m/s, and z -4.0 to 4.0 rad/s. Test with small values. The repo example uses 0.3 m/s.

### Sport Mode Watchdog

The robot auto-stops if it receives no Move command for about 500 ms.

Publish Move at 50 Hz while the robot should keep moving. This is why `move_forward.py` publishes at 50 Hz.

Stop sending Move to stop the robot. Send StopMove (1003) to stop at once and stay standing.

### Behavior and Show Commands

These commands trigger preset motions. The repo does not use them. Clear the area before you send one.

| API ID | Command | Parameter JSON | What it does |
|--------|---------|----------------|--------------|
| 1016 | Hello | `{}` | Wave a paw |
| 1017 | Stretch | `{}` | Stretch |
| 1020 | Content | `{}` | Happy gesture |
| 1021 | Wallow | `{}` | Roll on the ground |
| 1022 | Dance1 | `{}` | Dance routine 1 |
| 1023 | Dance2 | `{}` | Dance routine 2 |
| 1029 | Scrape | `{}` | Scrape the ground with a paw |
| 1033 | WiggleHips | `{}` | Wiggle the hips |

### Advanced Motion Commands

These commands carry a risk of falls. The repo does not use them. Use them on flat, open ground with a charged battery and clear space around the robot.

| API ID | Command | Parameter JSON | What it does |
|--------|---------|----------------|--------------|
| 1030 | FrontFlip | `{"data": true}` | Flip forward |
| 1031 | FrontJump | `{"data": true}` | Jump forward |
| 1032 | FrontPounce | `{"data": true}` | Pounce forward |
| 1042 | LeftFlip | `{"data": true}` | Flip to the left |
| 1044 | BackFlip | `{"data": true}` | Flip backward |
| 1301 | Handstand | `{"data": true}` | Enter or leave a handstand |
| 1045 | FreeWalk | `{"data": true}` | Enable or disable free-walk mode |
| 1046 | FreeBound | `{"data": true}` | Enable or disable bounding mode |
| 1047 | FreeJump | `{"data": true}` | Enable or disable jumping mode |
| 1048 | FreeAvoid | `{"data": true}` | Enable or disable free-avoid mode |
| 1049 | ClassicWalk | `{"data": true}` | Enable or disable the classic walk |
| 1050 | WalkUpright | `{"data": true}` | Enable or disable upright walking |
| 1051 | CrossStep | `{"data": true}` | Enable or disable cross-step walking |

### Configuration and Query Commands

| API ID | Command | Parameter JSON | What it does |
|--------|---------|----------------|--------------|
| 1018 | TrajectoryFollow | list of timed path points | Follow a path of timed poses and velocities |
| 1024 | GetBodyHeight | `{}` | Query the body height |
| 1025 | GetFootRaiseHeight | `{}` | Query the foot raise height |
| 1026 | GetSpeedLevel | `{}` | Query the speed level |
| 1027 | SwitchJoystick | `{"data": true}` | Enable or disable remote-controller joystick input |
| 1034 | GetState | `{}` | Query robot state values |
| 1052 | AutoRecoverSet | `{"data": true}` | Enable or disable automatic fall recovery |
| 1053 | AutoRecoverGet | `{}` | Query the automatic fall recovery setting |
| 1054 | SwitchAvoidMode | `{}` | Toggle the obstacle-avoid mode |

Query commands return their value in the response. See [Responses](#responses).

### Recommended Command Sequences

Start motion in this order. Each step needs the one before it.

1. StandUp (1004)
2. BalanceStand (1002)
3. Move (1008) at 50 Hz
4. StopMove (1003)
5. StandDown (1005)

Damp (1001) cuts power to the motors. The robot collapses. Use it only when the robot is on the ground or in a real emergency.

### Publish From the Command Line

Stand the robot. Run a single command with `--once`.

```bash
ros2 topic pub --once /api/sport/request unitree_api/msg/Request \
  '{header: {identity: {api_id: 1004}}}'
```

Walk forward at 0.3 m/s. The `-r 50` flag feeds the watchdog. Stop with Ctrl+C.

```bash
ros2 topic pub -r 50 /api/sport/request unitree_api/msg/Request \
  '{header: {identity: {api_id: 1008}}, parameter: "{\"x\": 0.3, \"y\": 0.0, \"z\": 0.0}"}'
```

Stop and stay standing.

```bash
ros2 topic pub --once /api/sport/request unitree_api/msg/Request \
  '{header: {identity: {api_id: 1003}}}'
```

### Publish From Python

A minimal node that sends Move at 50 Hz.

```python
import json
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request


class MoveCommand(Node):
    def __init__(self):
        super().__init__("move_command")
        self.pub = self.create_publisher(Request, "/api/sport/request", 10)
        # 0.02 s period = 50 Hz, which keeps the watchdog fed
        self.timer = self.create_timer(0.02, self.send)

    def send(self):
        msg = Request()
        msg.header.identity.api_id = 1008
        msg.parameter = json.dumps({"x": 0.3, "y": 0.0, "z": 0.0})
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(MoveCommand())


if __name__ == "__main__":
    main()
```

### Responses

The robot answers on `/api/sport/response` with a `unitree_api/Response` message.

```bash
ros2 topic echo /api/sport/response
```

Read `header.status.code`. A value of 0 means success. Query commands put their result in `data`.

### Other Unitree Topics

The Go2 exposes more interfaces. This repo does not use them. They are listed here for extension work.

| Topic | Message type | Purpose |
|-------|--------------|---------|
| `/sportmodestate` | `unitree_go/msg/SportModeState` | Sport-mode state: position, velocity, gait, body height, foot force. About 50 Hz |
| `/lowstate` | `unitree_go/msg/LowState` | Low-level state: motor angles, IMU, battery. About 500 Hz |
| `/lowcmd` | `unitree_go/msg/LowCmd` | Direct motor commands |
| `/wirelesscontroller` | `unitree_go/msg/WirelessController` | Remote controller stick and button state |
| `/api/motion_switcher/request` | `unitree_api/Request` | Select or release the motion service |
| `/api/obstacles_avoid/request` | `unitree_api/Request` | Obstacle-avoid service |

Motion switcher API IDs:

| API ID | Command | Purpose |
|--------|---------|---------|
| 1001 | CheckMode | Read the active motion mode |
| 1002 | SelectMode | Select a motion mode by name |
| 1003 | ReleaseMode | Release the active motion mode |
| 1004 | SetSilent | Silence the robot voice |
| 1005 | GetSilent | Read the silent setting |

`/lowcmd` bypasses the sport-mode safety logic. Release the active motion mode with ReleaseMode before you send it. The robot does not balance itself while the motion mode is released. Do not use `/lowcmd` and `/api/sport/request` together.

## Camera Intrinsics Defaults

`map_node.py` uses these Go2 front camera defaults when `/camera/camera_info` is unavailable.

| Parameter | Value |
|-----------|-------|
| `fx` | 864.0 |
| `fy` | 864.0 |
| `cx` | 639.2 |
| `cy` | 373.3 |
| Distortion | `[-0.354630, 0.102054, -0.001614, -0.001249, 0.0]` |
| LiDAR to camera extrinsic | 4x4 homogeneous matrix, identity rotation, translation `[0, 0, 0]` |

The intrinsic matrix built from these values:

```yaml
K: [864.0,   0.0, 639.2,
      0.0, 864.0, 373.3,
      0.0,   0.0,   1.0]
```

The actual `/camera/camera_info` message overrides these defaults when it is available.

Fusion projects each LiDAR point into the image plane with the extrinsic and K, then reads the pixel colour.

## The Full Data Flow

1. Go2 firmware publishes `/utlidar/cloud`, `/utlidar/imu`, and `/utlidar/robot_odom` at high rate.
2. Point-LIO subscribes to cloud and IMU. It publishes `/registered_scan` and `/state_estimation`.
3. `map_node.py` subscribes to `/registered_scan` and `/state_estimation` and builds the map.
4. `map_node.py` publishes `/map/occupancy` (TRANSIENT_LOCAL) and `/map/points`.
5. `pointcloud_to_scan.py` converts `/registered_scan` to `/scan`.
6. `slam_toolbox` subscribes to `/scan` and publishes `/map`.
7. `map_saver` saves `/map` to disk as `.yaml` and `.pgm`.
8. On the next run, `map_server` loads the saved map and republishes `/map`.
9. AMCL subscribes to `/scan` and `/map`. It publishes `/amcl_pose` and the TF `map->odom`.
10. The Nav2 planner subscribes to `/goal_pose` and `/map`. It publishes `/plan`.
11. The Nav2 controller subscribes to `/plan` and `/local_costmap/costmap`. It publishes `/cmd_vel`.
12. Go2 firmware subscribes to `/cmd_vel` and drives the motors.

```
Go2 firmware --> /utlidar/cloud, /utlidar/imu --> Point-LIO
                                                    |
                         +--------------------------+
                         |                          |
                  /registered_scan           /state_estimation
                         |                          |
              +----------+----------+               |
              |                     |               |
     pointcloud_to_scan         map_node <----------+
              |                     |
            /scan          /map/occupancy, /map/points --> RViz
              |
     +--------+--------+
     |                 |
slam_toolbox          AMCL <-- map_server <-- saved map
     |                 |
   /map            map->odom
     |                 |
     +--> Nav2 planner --> /plan --> Nav2 controller --> /cmd_vel --> Go2 firmware
```

## Launch File Wiring

| Launch file | Wires |
|-------------|-------|
| `mapping.launch.py` | map_node alone |
| `slam_mapping.launch.py` | slam_toolbox + lifecycle |
| `localization.launch.py` | map_server + AMCL + lifecycle |
| `nav2_bringup.launch.py` | planner + controller + behaviors + BT navigator + lifecycle |
| `integrated_robot.launch.py` | cloud_relay + cmd_vel_bridge (WebRTC fallback) |

## Failure Modes and Diagnostics

| Symptom | Diagnostic | Cause |
|---------|------------|-------|
| Topic exists but no data | `ros2 topic info <topic> -v` | QoS mismatch |
| RViz shows "No map received" | `ros2 topic info /map/occupancy -v` | Publisher not TRANSIENT_LOCAL |
| Point-LIO silent after IMU init | `ros2 topic hz /registered_scan` | Wrong cloud topic or use_sim_time |
| Robot does not move | `ros2 topic hz /cmd_vel` | No planner output or wrong message type |
| Camera black in RViz | `ros2 topic hz /camera/image_raw_relayed` | WebRTC SDK not running or QoS mismatch |
| Robot stops after about half a second | `ros2 topic hz /api/sport/request` | Move rate below 50 Hz, so the watchdog fires |
| Command sent, robot ignores it | `ros2 topic echo /api/sport/response` | Wrong API ID, robot not standing, or command unsupported on this firmware |

## Cross-References

- [README.md](../README.md) - repo overview, modes, and workflows
- [setup.md](setup.md) - first-time clone, dependency, and build setup
- [ethernet.md](ethernet.md) - run the stack over a direct cable
- [wireless.md](wireless.md) - run the stack over a USB Wi-Fi dongle
- [troubleshooting.md](troubleshooting.md) - fixes for environment, DDS, SLAM, and motion errors
- [roadmap.md](roadmap.md) - development phases and planned work