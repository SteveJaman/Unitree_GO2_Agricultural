# Unitree_GO2_Agricultural

Autonomous navigation, 3D mapping and AI layer for the **Unitree Go2 EDU**
quadruped, with a **Unitree D1-T** arm for crop scanning. It runs on top of
[`dancher00/go2-nav2-wifi`](https://github.com/dancher00/go2-nav2-wifi): Point-LIO
SLAM on the Jetson, Nav2 on a laptop VM, bridged over Wi-Fi. It adds:

- a frontier explorer;
- a live 3D digital twin in a web dashboard;
- YOLO and local-VLM reasoning;
- battery-aware docking;
- arm control;
- one-command orchestration of all of it.

## Contents

- [Quick Start](#quick-start)
- [Core Features](#core-features)
- [Documentation](#documentation)
- [System Requirements](#system-requirements)
- [What This Repo Adds Over Upstream](#what-this-repo-adds-over-upstream)
- [License](#license)

## Quick Start

```bash
git clone https://github.com/SteveJaman/Unitree_GO2_Agricultural ~/files/Unitree_GO2_Agricultural
cd ~/files/Unitree_GO2_Agricultural
./scripts/runtime/wifi_3d_nav.sh            # dog powered on, standing at the dock
```

That one command does everything. The first run builds the 3D / AI image and
pulls the VLM model (needs internet). Then the dog maps the area on its own,
splits it into rooms, names each, saves a report and returns to the dock.
`--manual` keeps the AI but leaves starting and steering to you.

Then open **<http://localhost:3000>** on the VM, or `http://<VM IP>:3000` from
Windows. The terminal prints the address.

**First time on a workstation?** Before this you need:

- the upstream stack and its lab fixes;
- passwordless SSH to the robot;
- the arm calibration;
- Ollama for the VLM;
- bigger UDP receive buffers on the VM (one `sysctl` file), or camera
  frames and LiDAR clouds get dropped.

All of it is in **[docs/setup.md](docs/setup.md)**.

Other entry points:

| Command | Use |
|---|---|
| `./scripts/runtime/wifi_nav.sh --arm` | 2D navigation + explorer in RViz, D1 arm folded |
| `./scripts/runtime/wifi_nav.sh --arm-wifi` | Same, with the arm plugged into the Go2 and controlled over Wi-Fi |
| `./scripts/runtime/wifi_3d_nav.sh --resume` | Restart after a battery swap; continue the last map |
| `./scripts/runtime/wifi_nav.sh --sim --arm` | Same stack against the Gazebo simulation |
| `./scripts/runtime/wifi_3d_nav.sh --view` | Show saved 3D scans without the robot |

## Core Features

- **Auto-Explore:** a frontier explorer maps an area by itself; START / STOP
  from the dashboard or RViz.
- **3D web dashboard:** a full-screen live 3D map in the browser (solid,
  shadowed, camera-coloured), Unitree's Go2 model with live leg joints, the
  route and camera. Tap the floor to go there. A performance mode keeps it
  usable on a VM without 3D acceleration. After a run, export a solid
  presentation mesh.
- **Camera obstacles:** `--camera-obstacles` turns monocular depth, scaled by
  the LiDAR, into Nav2 obstacles (glass, table legs). `--fusion` colours the
  map from the calibrated camera.
- **YOLO / VLM AI:** YOLO11s obstacles feed the Nav2 costmap. A local
  `llava:7b` (Ollama) reads the camera and pauses + docks on a blocked path.
- **Battery dock and auto-localization:** below 15 % the dog returns to the
  dock. It localizes itself at start, so there's no "2D Pose Estimate" click.
- **Semantic rooms:** the map is split into rooms (robot-vacuum style) that
  you name (with a VLM suggestion), merge, and send the robot to.
- **Autonomous survey:** `--survey` maps an unknown building, splits it into
  rooms, visits and names each from what YOLO and the VLM see, saves a
  report and docks, with no user input.
- **Arm control:** the D1-T over DDS/JSON (fold, joints, lift, sweep,
  calibration), wired to the workstation or plugged into the Go2 and
  controlled over Wi-Fi (`--arm-wifi`). Teach & Playback: guide the arm by
  hand to frame plants, then replay the scan sweep.
- **Full mobility:** `--omni` lets Nav2 walk the dog backwards and sideways;
  stuck recovery backs it out of dead ends in every session.
- **Simulation:** Go2 + D1 in Gazebo Harmonic for testing without hardware.

## Documentation

| Doc | Contents |
|---|---|
| [Setup](docs/setup.md) | One-time workstation setup, everyday commands, flags and overrides |
| [Architecture](docs/architecture.md) | Project scope, data flow, network and DDS domains, DHCP discovery |
| [3D Digital Twin & AI](docs/3d_nav.md) | Live 3D mapping, web dashboard, semantic rooms, autonomous survey, YOLO, VLM, battery dock, `--resume`, export and mesh |
| [Explorer](docs/explorer.md) | Frontier explorer phases, parameters, manual launch |
| [Scripts](docs/scripts.md) | Every runtime script, ROS node and tool |
| [D1-T arm](docs/d1arm.md) | Arm protocol, commands, joint limits, calibration |
| [Camera](docs/camera.md) | Go2 camera relay, RViz display, who uses the feed |
| [Camera + LiDAR fusion](docs/camera_lidar_fusion.md) | Camera calibration, the LiDAR overlay check, `--fusion`, `--camera-obstacles` |
| [Omni navigation](docs/omni_nav.md) | `--omni`: full Go2 mobility (MPPI reverse + sidestep), stuck recovery |
| [Waypoint tour](docs/tour.md) | RViz waypoint mode |
| [Troubleshooting](docs/troubleshooting.md) | Known problems and fixes, collision tuning, diagnostics |
| [Lab nav fixes](docs/LAB-NAV-FIXES.md) | Fixes applied to upstream go2-nav2-wifi |
| [Development](docs/development.md) | Repository layout, lint hooks, legacy code, external projects |
| [Status](docs/STATUS.md) | Current state and the robot test checklist |
| [Roadmap](docs/roadmap.md) | Milestones and what is next |
| [Simulation](gazebo/README.md) | Gazebo setup, what is simulated, limits |
| [Config](config/README.md) | RViz and Foxglove layouts, camera calibration, saved named locations and rooms |

## System Requirements

- **Laptop VM:**
  - Ubuntu 22.04, 4+ cores;
  - VirtualBox with the Wi-Fi adapter bridged as `enp0s3`, and the arm's
    Ethernet as `enp0s8`;
  - Docker;
  - the upstream [`go2-nav2-wifi`](https://github.com/dancher00/go2-nav2-wifi)
    clone in `~/files/go2-nav2-wifi`.
- **Ollama** with `llava:7b`, for the VLM. `wifi_3d_nav.sh` starts `ollama serve` and pulls the model
  when needed. A
  host with a GPU is much faster.
- **Hardware:** Unitree Go2 EDU (Jetson). D1-T arm and a RealSense camera are
  optional.
- **Viewer:** any modern browser with WebGL (Chrome, Edge, Firefox).

## What This Repo Adds Over Upstream

Upstream provides Point-LIO SLAM, Nav2, the Wi-Fi relay and manual RViz goals.
This repo adds:

- **Autonomous exploration:** frontier explorer, blind lead-in, coverage
  pass, RViz START/STOP panel.
- **3D digital twin:** RTAB-Map with camera fusion, a React + three.js web
  dashboard, click-to-go snapped to safe floor, presentation meshes, `--resume`.
- **AI:** YOLO11s obstacles in the costmap, VLM scene reasoning, battery dock,
  auto-localization, named locations.
- **D1-T arm control:** chained DDS commands, fold / lift / sweep, offset
  calibration, Teach & Playback for plant scans, and control over Wi-Fi with
  the arm plugged into the Go2 (robot-side TCP bridge).
- **Orchestration:**
  - one command each for 2D, 3D, arm and simulation;
  - DHCP-proof networking that keeps the arm and dog networks isolated;
  - live camera over Wi-Fi (+ calibration), floor filter, diagnostics and tuning scripts.
- **Gazebo simulation** of the Go2 + D1 from the official Unitree models.

## License

MIT — see [LICENSE](LICENSE).
