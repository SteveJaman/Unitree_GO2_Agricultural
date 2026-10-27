# Unitree D1 arm: setup and control runbook

Controls a Unitree D1 / D1-T arm over DDS directly from the VM host (no
container). Protocol source: Unitree "D1 Mechanical Arm Services Interface" doc,
cross-checked against working open-source drivers (see References).

**Verified config:** interface `enp0s8`, DDS domain 0, **multicast** discovery.
Or plug the arm into the Go2 and control it over Wi-Fi: see
[Arm over Wi-Fi](#arm-over-wi-fi-plugged-into-the-go2).
Do not use `--peer` (unicast): it breaks discovery on this arm's firmware.

## Files

| File | Role |
|---|---|
| `scripts/runtime/start_arm.sh` | Entry point. Runs a preflight (host address, ping), then hands the command to the right script |
| `scripts/nodes/d1_arm_control.py` | Chain-style CLI + `D1Arm` class: `state`, `enable`, `release`, `home`, `power`, `joint`, `joints`, `sleep`, `feedback`, `status`, `motors`, `lift`, `sweep` |
| `scripts/nodes/d1_arm_calibration.py` | `--read` the folded pose, `--move` fold (power on → enable → move → release → power off) |
| `scripts/tools/calibrate_arm.py` | Measures per-joint offset (commanded vs reported angle) with small ± test moves |
| `scripts/tools/teach_arm.py` | Plant scanning: guide the arm by hand in compliance mode, save scan poses to `config/arm_waypoints.json` |
| `scripts/tools/play_arm.py` | Plant scanning: replay the saved poses (home → each pose, hold for the camera → home) |
| `scripts/tools/diag_arm.sh` | Network + DDS diagnostic |
| `scripts/nodes/direct_test_movement.py` | Small safe movement test (needs `d1_arm_control.py` alongside) |
| `scripts/robot/arm_bridge_wifi.{sh,py}` | **On the robot:** the arm (plugged into the Go2) ↔ laptop over Wi-Fi, TCP 18100 |
| `scripts/robot/unitree_arm/` | One-message ROS 2 package (`ArmString`) the bridge is built with on the robot |

`./scripts/runtime/wifi_nav.sh --arm` runs `start_arm.sh fold` in the
background, alongside navigation; `--arm-wifi` does the same with the arm
plugged into the Go2.

The scripts run with the host's `python3` and need the `cyclonedds` package
there for the wired link: `pip3 install cyclonedds` (not needed over Wi-Fi). Don't source ROS in that shell: ROS's
`libddsc` can clash with the pip package.

## Protocol summary (verified against the doc and working drivers)

- Topics: `rt/arm_Command` (host -> arm), `rt/arm_Feedback` (arm -> host), type
  `unitree_arm::msg::dds_::ArmString_` (one JSON string), DDS domain 0.
- Command: `{"seq":N,"address":1,"funcode":F,"data":{...}}`. `data` is a nested object.
- Angles are **degrees**. Joints 0-5 arm, joint 6 gripper (servo degrees).
- funcode 1 single joint `{"id","angle","delay_ms":0}`; 2 all joints `{"mode":1,"angle0".."angle6"}`;
  4 single enable; 5 enable all `{"mode":80000}` (0 = release); 6 power `{"power":0|1}`;
  7 home (no `data`).
- Feedback: address 2 funcode 1 = joint angles (10 Hz), funcode 3 = status,
  funcode 4 = motor info; address 3 funcode 1 = command received (`recv_status`),
  funcode 2 = executed.
- Joint limits (degrees, match the D1-550 URDF): ±135, ±90, ±90, ±135, ±90, ±135.
  Joint names: 0 base, 1 shoulder, 2 elbow, 3 wrist roll, 4 wrist pitch, 5 wrist yaw.
- Arm default IP `192.168.123.100`.

## Hardware

- **Power:** 24V DC via the DC 5.5-2.1 barrel jack (~1 min boot)
- **Communication:** RJ45 to the VM's `enp0s8`. Arm at `192.168.123.100`, VM at
  `192.168.123.99` (`sudo ip addr add 192.168.123.99/24 dev enp0s8`). DDS
  discovery is multicast. It is kept separate from the Go2's Wi-Fi DDS
- **Debug:** USB-C (serial debug only)

## Network

The arm must be on its own wired interface in the VM (`enp0s8` by default). The
VM side needs the address **`192.168.123.99/24`** on that interface. The Go2's
Wi-Fi bridge (`enp0s3`) must be on a different subnet, otherwise arm and Go2 DDS
traffic mix. `wifi_nav.sh --arm` refuses an overlapping setup.

In VirtualBox, attach the arm's Ethernet as Adapter 2 (Bridged, adapter type
**Intel PRO/1000**). The arm needs about a minute after power-on. Then:

```bash
sudo ip addr add 192.168.123.99/24 dev enp0s8
sudo ip link set enp0s8 up
./scripts/runtime/start_arm.sh --check     # preflight: address + ping
```

The address doesn't persist across VM reboots, so re-add it after a restart. If
the arm doesn't reply to ping, cold-boot the VM (power off, not reboot).

Overrides: `ARM_IFACE` (default `enp0s8`), `ARM_IP` (`192.168.123.100`),
`ARM_HOST_IP` (`192.168.123.99`), `ARM_DOMAIN` (`0`).

## Arm over Wi-Fi (plugged into the Go2)

The arm's address, `192.168.123.100`, is on the Go2's internal network (the
Jetson is `192.168.123.18`). Plug the arm's RJ45 into the Go2's Ethernet port
and the laptop controls it over Wi-Fi, like the dog, with no cable to the
workstation:

```text
D1 arm ──eth0, DDS domain 0──► arm_bridge_wifi.py (Jetson) ──Wi-Fi, TCP 18100──► laptop D1Arm
```

- **Bridge (robot):** [`arm_bridge_wifi.py`](../scripts/robot/arm_bridge_wifi.py)
  talks to the arm with the robot's ROS 2 Foxy (`rclpy`, message
  `unitree_arm/ArmString` = DDS `unitree_arm::msg::dds_::ArmString_`) on
  `eth0`, multicast, as on the workstation. It serves the laptop one JSON
  message per line over TCP: arm commands in, every `rt/arm_Feedback` message
  out.
- **Why TCP:** arm traffic is small JSON at 10 Hz. TCP needs no DDS discovery
  over Wi-Fi and loses nothing (see the camera's Wi-Fi DDS problems in
  [troubleshooting](troubleshooting.md#camera)).
- **Laptop:** `D1Arm` (`d1_arm_control.py`) takes `--robot <Go2 Wi-Fi IP>` or
  `ARM_ROBOT`. Every arm script follows it unchanged: the control CLI, `fold`,
  `read`, offset calibration, `teach_arm.py`, `play_arm.py`. `cyclonedds` is
  not needed on the laptop for this.
- **Safety:** the laptop pings every second. If it disconnects, is silent for
  3 s (`ARM_BRIDGE_TIMEOUT`), or the bridge is stopped while the motors hold
  the arm, the bridge locks it where it is (mode 80000): a Wi-Fi drop never
  drops the camera. A released arm (mode 0) is left alone. Only the laptop
  (`GO2_HOST_IP`) may connect, and only arm commands are forwarded.

**One-time build on the robot** (uses its ROS Foxy; nothing else is
installed). `wifi_nav.sh --arm-wifi` copies the files to `~/go2-agri-arm` and
prints this command if the build is missing:

```bash
ssh unitree@<robot-ip> 'bash ~/go2-agri-arm/arm_bridge_wifi.sh --build'
```

**Every session:**

```bash
./scripts/runtime/wifi_nav.sh --arm-wifi                  # nav + bridge + fold over Wi-Fi
./scripts/runtime/start_arm.sh --robot <robot-ip> state   # any arm command, over Wi-Fi
ARM_ROBOT=<robot-ip> python3 scripts/tools/teach_arm.py   # teach / play the same way
```

Without `wifi_nav.sh`, start the bridge on the robot yourself:
`GO2_HOST_IP=<laptop Wi-Fi IP> bash ~/go2-agri-arm/arm_bridge_wifi.sh` (log:
`/tmp/arm_bridge_wifi.log` when started by `wifi_nav.sh`).

**Power:** the arm needs 24 V DC. Check that the Go2 EDU's power output can
supply it, or keep the arm on its own supply.

Overrides: `ARM_BRIDGE_PORT` (18100), `ARM_BRIDGE_TIMEOUT` (3 s),
`ARM_BRIDGE_IFACE` (`eth0`), `ARM_DOMAIN` (0).

## Run

```bash
ARM_IFACE=enp0s8 ./scripts/tools/diag_arm.sh     # all PASS expected
./scripts/runtime/start_arm.sh --check           # preflight only
./scripts/runtime/start_arm.sh state             # read state
./scripts/runtime/start_arm.sh read              # print FOLDED_ANGLES (calibration)
./scripts/runtime/start_arm.sh fold              # fold onto the mount, then release
./scripts/runtime/start_arm.sh enable sleep 2 joint 0 20 sleep 3 release
```

`start_arm.sh` with no arguments prints its help. Anything other than
`--check`, `fold`, `read` and `--calibrate` is passed to `d1_arm_control.py`.

**Fold calibration.** With the motors released, fold the arm onto its mount by
hand. Run `start_arm.sh read` and paste the printed `FOLDED_ANGLES` line into
`scripts/nodes/d1_arm_calibration.py`. Redo it whenever the mount changes.
`fold` powers on, enables, sends one funcode 2 move to `FOLDED_ANGLES`, waits
5 s, then **releases and powers off**, so the mount must hold the arm.

## Control commands

Commands chain in one process, so the arm is only discovered once:

```bash
A=./scripts/runtime/start_arm.sh

$A state                                   # status (address 2, funcode 3)
$A feedback                                # joint angles
$A motors                                  # motor info (funcode 4)
$A enable sleep 2 joint 0 20 sleep 3 release
$A joints 0 -30 30 0 20 0 0                # 7 values; non-zero joints sent one by one
$A home sleep 6 release                    # all joints to zero
$A power 0                                 # motor power off
$A lift --deg 45 --yes                     # shoulder (joint 1) to +45°
$A sweep --pct 90 --yes                    # each joint to ±90% of its limit and back
$A sweep --joints 0,1 --pct 50 --yes
```

- `joint` and `joints` reject anything beyond the joint limits.
- `joints` sends one funcode 1 command per non-zero joint (0.5 s apart) so
  each target is limit-checked. The gripper value is not sent.
- `lift` and `sweep` ask for confirmation unless you pass `--yes`.
  `--continue-on-error` keeps a sweep going past a failed joint.
- Other flags: `--enable-mode` (default 80000, fully locked), `--match-timeout`
  (DDS discovery wait, default 20 s), `--retries`, `--verbose`,
  `--skip-preflight`.

## Command reference

Every command is `{"seq":N,"address":1,"funcode":F,"data":{...}}`, where
`data` is a nested JSON object, not a string.

| Action | funcode | `data` | Purpose |
|---|---|---|---|
| Single joint | `1` | `{"id":J,"angle":DEG,"delay_ms":0}` | Move joint J (6 = gripper) |
| All joints | `2` | `{"mode":1,"angle0":…,"angle6":…}` | Move all 7 at once |
| Enable all | `5` | `{"mode":80000}` | Motors on, fully locked |
| Compliance | `5` | `{"mode":5000}` | Motors on, low stiffness: holds its weight, yields to a hand (teach) |
| Release | `5` | `{"mode":0}` | Motors off (limp, drag-teach) |
| Home | `7` | *(none)* | Return all joints to zero |

Joint limits (degrees, match the D1-550 URDF): ±135, ±90, ±90, ±135, ±90, ±135.
The scripts reject anything beyond them.

## Parameters

| Flag / env | Default | Notes |
|---|---|---|
| `--iface` / `ARM_IFACE` | `enp0s8` | Interface that reaches the arm |
| `--ip` / `ARM_IP` | `192.168.123.100` | Arm address (preflight ping and route check) |
| `ARM_HOST_IP` | `192.168.123.99` | VM address `start_arm.sh` expects on the interface |
| `--domain` / `ARM_DOMAIN` | `0` | DDS domain |
| `--enable-mode` | `80000` | `enable` value (80000 = fully locked) |
| `--match-timeout` | `20` | Seconds to wait for DDS discovery |
| `--deg` | `45` | `lift` target for the shoulder |
| `--pct` / `--joints` | `90` / `all` | `sweep` range (% of limit) and joints |
| `--yes` | off | Skip the confirmation prompt for `lift` / `sweep` |
| `--peer` | *(none)* | Unicast discovery. **Don't use**: this firmware needs multicast |

## Plant Scanning (Teach & Playback)

Kinesthetic teaching for plant data collection: guide the arm by hand until
the end camera frames a plant, save the pose, and let the arm replay the
whole sweep later. Fully released (`mode 0`) the arm drops under the
camera's weight, so teaching uses **compliance mode** (`funcode 5`,
`{"mode": 5000}`): the motors hold the arm up but yield to a push.

**1. Teach** (one hand on the arm, the other on the keyboard):

```bash
python3 scripts/tools/teach_arm.py          # add poses to the existing sweep
python3 scripts/tools/teach_arm.py --new    # start a new sweep
```

1. The arm powers on and goes compliant: *"Arm is compliant. You can now
   move it by hand."*
2. Move it until the camera frames the plant, then press **Enter**. The six
   joint angles (telemetry address 2, funcode 1) are appended to
   `config/arm_waypoints.json`, and the file is written after every pose.
   Poses beyond the joint limits, or saved without telemetry, are refused.
3. Repeat for every scan angle, in the order you want them visited.
4. Type **q** to finish. The arm locks (`mode 80000`) where it is; Ctrl+C or
   Ctrl+D lock it too, so it never drops.

If the arm sags under the camera, raise `--stiffness` (e.g. 8000); if it is
hard to push, lower it.

**2. Play back:**

```bash
python3 scripts/tools/play_arm.py --dry-run  # check the file and print the poses
python3 scripts/tools/play_arm.py            # run the sweep
```

Every waypoint is checked against the joint limits before anything moves.
Then the arm powers on, locks, homes (`funcode 7`) and visits each pose
with a smooth trajectory (`funcode 2`, `mode 1`). It waits until the pose is
reached (within `--tolerance`, 3°), prints *"Waypoint N reached. [Camera
should capture photo here]"* and holds `--dwell` seconds (3). At the end it
homes again and **stays locked**; `--release` releases it at home (support
it first, or fold it with `start_arm.sh fold` instead). Ctrl+C or an error
locks the arm where it stopped.

`config/arm_waypoints.json` is user data (git-ignored):

```json
{"waypoints": [{"angles": [10.5, -32.0, 45.1, 0.0, 20.0, 0.0], "saved": "2026-10-27T17:05:12"}]}
```

Both scripts take the usual `--iface`, `--ip`, `--domain`,
`--skip-preflight` and `--file` (another waypoint file, e.g. one per crop).

## Offset calibration

`calibrate_arm.py` moves each joint through ±`--test-deg` from zero and
compares commanded vs reported angles. It starts and ends with the motors
released.

```bash
./scripts/runtime/start_arm.sh --calibrate --test-deg 10 --yes
./scripts/runtime/start_arm.sh --calibrate --joints 0,1,2 --output cal.json --yes
```

Flags: `--test-deg` (default 10), `--joints` (`all` or ids), `--settle`
(seconds per step, default 2), `--output` (JSON file).

## Simulation

The same scripts run against a simulated D1 in Gazebo. `gazebo/sim/d1_arm_emulator.py`
speaks this protocol on interface `simarm0` (`192.168.123.51`, arm "at" `.100`):

```bash
./gazebo/run_sim.sh up
export ARM_IFACE=simarm0 ARM_HOST_IP=192.168.123.51
./scripts/runtime/start_arm.sh read
./scripts/runtime/start_arm.sh enable sleep 1 joint 0 20 sleep 3 feedback
```

`wifi_nav.sh --sim --arm` sets these for you.

Sim differences:

- Motion is rate-limited to 60°/s.
- `release` holds the pose.
- Gripper 0–65° maps to 0–30 mm of finger travel.

The official D1 URDF limits are ±134.6° / ±89.95°, slightly inside the ±135° / ±90°
used here, so a ±90° target is clamped by 0.05°. See [gazebo/README.md](../gazebo/README.md).

## Troubleshooting

| Symptom | Meaning | Fix |
|---|---|---|
| `enp0s8 is missing 192.168.123.99` | address lost after reboot | `sudo ip addr add 192.168.123.99/24 dev enp0s8` |
| `192.168.123.100 does not reply to ping` | cable / adapter / still booting | check link; cold-boot the VM with Adapter 2 = Intel PRO/1000 |
| `route to ... does NOT go via enp0s8` | another interface owns the subnet | remove the overlapping address |
| `DDS discovery failed` | no DDS match within `--match-timeout` | arm powered and booted? no `--peer`; no ROS sourced; `pip3 show cyclonedds` |
| commands acked but nothing moves | motors not enabled | `enable` first (`--enable-mode 80000`) |
| `joint N limit ±L°` | target outside the limit | use a smaller angle |
| `free(): invalid pointer` crash | ROS's CycloneDDS mixed with the pip one | run without sourcing ROS |
| `wifi_nav.sh --arm` skipped the arm | see `go2-nav2-wifi/ws/log/arm.log` | run `start_arm.sh --check` |
| `--arm-wifi`: "not built on the robot yet" | the one-time build is missing | `ssh unitree@<robot-ip> 'bash ~/go2-agri-arm/arm_bridge_wifi.sh --build'` |
| `--robot`: "bridge not reachable" | bridge not running, or another laptop IP | on the robot: `tail /tmp/arm_bridge_wifi.log`; it accepts only `GO2_HOST_IP` |
| `--robot`: "no arm feedback" | arm not plugged into the Go2, not powered, or still booting (~1 min) | from the robot: `ping 192.168.123.100` |

## Arm roadmap

| Step | Purpose | Status |
|---|---|---|
| 1. Direct control | Prove the arm responds to JSON commands | Written: chain control, fold, lift/sweep, offset calibration |
| 2. ROS 2 bridge node | Subscribe to `/arm/command` instead of the raw script | Pending |
| 2b. Arm on the Go2 over Wi-Fi | Robot-side TCP bridge, `--robot` / `--arm-wifi` | Written (2026-10-27), tested against a mock arm; ⏳ first run on the robot |
| 3. RViz panel | Slider per joint, alongside the explorer panel | Pending |
| 4. Camera mount | 3D-printed attachment for the hyperspectral camera | Hardware |
| 5. Scan routine | Move the camera through a plant-scanning path | Pending (`run` sequence files are the building block) |
| 6. AI autonomy | Model commands both the dog and the arm | Long-term |

See [docs/roadmap.md](roadmap.md) for the overall project roadmap.

## References

- Unitree D1 doc: <https://support.unitree.com/home/en/developer/D1Arm_services>
- chen37058/Grasp-with-the-Unitree-D1 (C++ SDK grasping demos)
- SofiaAbal/tfg_go2_arm PR #23 (D1-550 driver: enable mode 80000, home funcode 7)
- Robot-Learning-Group/d1t-teleop (pure Python, `unitree_sdk2py` + cyclonedds, hardware-verified)
- kokekokko481576/Go2_deploy PR #77 (rad to degrees JSON bridge, home, range clamping)
- JeewanthaSadaruwan/unitree-D1-550-Robot-ARM (C++ SDK examples on a Go2-mounted D1-550).
  Ideas reimplemented in an earlier version of `d1_arm_control.py` (`glide`), not
  copied (the repo has no license file): quintic
  trajectory streaming, averaged start-pose sampling, the `current_servo_angle`
  feedback topic, wait-until-reached, gripper `delay_ms`, network setup. Where we
  differ: their planner parses `servoN_data` from `rt/arm_Feedback` and silently
  plans from 0 when it is missing. Their `vmax` caps average, not peak, speed.
  Some of their waypoints (joint 5 at -164 deg) exceed their own URDF limits.
