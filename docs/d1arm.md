# Unitree D1 arm: setup and control runbook

Controls a Unitree D1 / D1-T arm over DDS directly from the VM host (no
container). Protocol source: Unitree "D1 Mechanical Arm Services Interface" doc,
cross-checked against working open-source drivers (see References).

**Verified config:** interface `enp0s8`, DDS domain 0, **multicast** discovery.
Do not use `--peer` (unicast): it breaks discovery on this arm's firmware.

## Files

| File | Role |
|---|---|
| `scripts/runtime/start_arm.sh` | Entry point. Runs a preflight (host address, ping), then hands the command to the right script |
| `scripts/nodes/d1_arm_control.py` | Chain-style CLI + `D1Arm` class: `state`, `enable`, `release`, `home`, `power`, `joint`, `joints`, `sleep`, `feedback`, `status`, `motors`, `lift`, `sweep` |
| `scripts/nodes/d1_arm_calibration.py` | `--read` the folded pose, `--move` fold (power on → enable → move → release → power off) |
| `scripts/tools/calibrate_arm.py` | Measures per-joint offset (commanded vs reported angle) with small ± test moves |
| `scripts/tools/diag_arm.sh` | Network + DDS diagnostic |
| `scripts/nodes/direct_test_movement.py` | Small safe movement test (needs `d1_arm_control.py` alongside) |

`./scripts/runtime/wifi_nav.sh --arm` runs `start_arm.sh fold` in the
background, alongside navigation.

The scripts run with the host's `python3` and need the `cyclonedds` package
there: `pip3 install cyclonedds`. Don't source ROS in that shell: ROS's
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

## References

- Unitree D1 doc: https://support.unitree.com/home/en/developer/D1Arm_services
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
