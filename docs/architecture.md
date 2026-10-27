# Architecture and Network

[← Back to README](../README.md) · [Setup](setup.md) · [Scripts](scripts.md) · [Troubleshooting](troubleshooting.md)

How the Go2, the laptop VM and the D1-T arm fit together, and how the
network is found and kept apart. For the 3D digital twin (RTAB-Map, web
dashboard, AI nodes) see [3d_nav.md](3d_nav.md#architecture).

---

## Project Scope

Two integrated capabilities:

1. **Autonomous navigation and mapping** — the Go2 explores and maps an
   environment on its own, driven by a custom frontier explorer.
2. **Crop scanning with the D1-T arm** — the arm holds a hyperspectral
   camera on a custom 3D-printed mount and moves it through a scanning
   pattern over plants, for later plant-analysis data collection.

The two systems are independent today. Integration — an AI model commanding
both the dog and the arm — is the long-term goal. First steps toward it are
in the [3D Digital Twin](3d_nav.md) session:

- a live 3D map;
- YOLO obstacles;
- a local VLM describing the camera view;
- low-battery return to the dock.

---

## Architecture

### Navigation

```text
Robot (Jetson, DHCP)                     Laptop VM (Ubuntu 22.04, DHCP on enp0s3)
─────────────────────────────            ───────────────────────────────────
LiDAR + IMU ─► Point-LIO                 go2-lidar3d container (Docker)
      │                                        │
      └─► /lidar3d/registered ──Wi-Fi relay──► /lidar3d/registered
                                               │
                                        Nav2 (planner/controller)
                                               │
                                        Regulated Pure Pursuit
                                               │
                                           /cmd_vel_nav
                                               │
                                        velocity_smoother
                                               │
                                             /cmd_vel
                                               │
                                        go2_cmd_vel_tcp_client
                                               │
                                     TCP :17999 (Wi-Fi)
                                               │
      go2_cmd_vel_tcp_server ◄─────────────────┘
             │
             ▼
      robot_sport_bridge
             │
             ▼
      Unitree Sport API → motors
```

### Arm

```text
D1-T arm (6 DOF + gripper, 192.168.123.100)
      │
      │ RJ45 → VM enp0s8 (192.168.123.99), DDS domain 0, multicast
      ▼
rt/arm_Command      ◄── JSON commands ──  d1_arm_control.py / d1_arm_calibration.py
rt/arm_Feedback     ── joint angles, acks ──►   (host python3 + pip cyclonedds,
                                                 ROS not sourced, bound to enp0s8)
                                                            ▲
                                  start_arm.sh fold ◄── wifi_nav.sh --arm
```

Key facts:

- **SLAM runs on the Jetson.** The VM never runs `slam_toolbox`. The global
  frame is `lidar3d_map`, produced by Point-LIO onboard.
- **All ROS runs inside Docker** on the VM. The host shell sees no topics.
  Use `go2sh` or `docker exec -u root go2-lidar3d bash -lc '…'`.
- **DDS domain 64** on the Wi-Fi side, domain 0 onboard. The relay bridges them.
- **The explorer reads from `/global_costmap/costmap`**, not `/map`. The
  upstream stack does not publish a `/map` topic — the Nav2 costmap is the
  occupancy grid source.
- **The D1-T arm uses DDS + JSON** on `rt/arm_Command`, not ROS 2 messages.
  The arm control script wraps the raw interface.
- **Arm and dog DDS never mix.** The Go2 runs on `enp0s3` (domain 64), and the
  arm on `enp0s8` (domain 0) with its own CycloneDDS config. The scripts check
  that the two interfaces are different and on separate subnets.

---

## Network

The router assigns addresses by DHCP, so **no IPs are hardcoded**. They are
resolved on every run by [`scripts/runtime/go2_net.sh`](../scripts/runtime/go2_net.sh).

| Node | IP | How it is found |
|---|---|---|
| Laptop VM | DHCP | `GO2_HOST_IP`: read from the Wi-Fi interface `enp0s3` (VirtualBox bridge). Override: `--host-ip` |
| Go2 Jetson | DHCP | `GO2_ROBOT_IP`: last known IP, then hostname `unitree`, then a MAC-prefix scan (`02:00:00`) on `enp0s3`, then an SSH-key probe. Override: `--robot-ip` |
| D1-T arm | `192.168.123.100` | Wired, on `enp0s8`. The nav stack never uses this interface |
| DDS domain | `64` | Wi-Fi side |
| Relay port | `17999` | TCP, `/cmd_vel` |

The container is created with `GO2_HOST_IP` and `GO2_ROBOT_IP` baked into its
environment. `mapping.sh` reads both from there and corrects only a stale host
IP, so a stale robot IP would send it to the wrong robot. When either address
changes, `wifi_nav.sh` recreates the container. Each session then writes its
CycloneDDS config afresh, bound to the current address. If the bind address were
wrong, the container would announce the wrong address and no topics would cross
the wire.

Interface safeguards: `enp0s8` (the arm, `ARM_IFACE`) is refused as the nav
interface. It is never scanned, and if the route to the robot goes through
it, the script stops with an error. The script also checks that the container's
DDS config binds only the Wi-Fi address.

Tunables: `GO2_IFACE` (default `enp0s3`), `ARM_IFACE` (default `enp0s8`),
`GO2_MAC_PREFIX`, `GO2_ROBOT_HOSTNAME`. Discovery is faster and more reliable
with `arp-scan` (`sudo apt install arp-scan`) or `nmap` installed; without
them it falls back to a ping sweep.
