# Unitree D1 arm: setup and control runbook

Controls a Unitree D1 / D1-T arm over DDS from the `go2-lidar3d` container.
Protocol source: Unitree "D1 Mechanical Arm Services Interface" doc, cross-checked
against working open-source drivers (see References).

## Files

| File | Role |
|---|---|
| `scripts/nodes/d1_arm_control.py` | Control node (runs in the container). CLI + `D1Arm` class: joints, `glide`, gripper, `angles`, `run` |
| `scripts/nodes/d1_arm_calibration.py` | `--read` the folded pose, `--move` fold (enable → glide → release) |
| `scripts/runtime/start_arm.sh` | From the host: fold (default), `--read`, `--no-home` (enable only), `--setup-net` |
| `scripts/tools/diag_arm.sh` | Network + DDS diagnostic |
| `scripts/nodes/direct_test_movement.py` | Small safe movement test (needs `d1_arm_control.py` alongside) |

The scripts find the Python files in `scripts/nodes/` (or set `NODES_DIR`).
`./scripts/runtime/wifi_nav.sh --arm` runs `start_arm.sh` automatically,
alongside navigation.

## Protocol summary (verified against the doc and working drivers)

- Topics: `rt/arm_Command` (host -> arm), `rt/arm_Feedback` (arm -> host), type
  `unitree_arm::msg::dds_::ArmString_` (one JSON string), DDS domain 0.
- Command: `{"seq":N,"address":1,"funcode":F,"data":{...}}`. `data` is a nested object.
- Angles are **degrees**. Joints 0-5 arm, joint 6 gripper (servo degrees).
- funcode 1 single joint `{"id","angle","delay_ms":0}`; 2 all joints `{"mode":1,"angle0".."angle6"}`;
  4 single enable; 5 enable all `{"mode":80000}` (0 = release); 6 power; 7 home (no `data`).
- Feedback: address 2 funcode 1 = joint angles (10 Hz), funcode 3 = status;
  address 3 funcode 1 = command received (`recv_status`), funcode 2 = executed.
- Second joint-angle source: topic `current_servo_angle`, type
  `unitree_arm::msg::dds_::PubServoInfo_` (`servo0_data`..`servo6_data`, float32).
- Joint limits (degrees, match the D1-550 URDF): ±135, ±90, ±90, ±135, ±90, ±135.
- Arm default IP `192.168.123.100`.

## Network

The arm must be on its own wired interface in the VM (`enp0s8` by default), with
an address on `192.168.123.x`. The Go2's Wi-Fi bridge (`enp0s3`) must be on a
different subnet, otherwise arm and Go2 DDS traffic mix. `wifi_nav.sh --arm`
refuses an overlapping setup.

In VirtualBox, attach the USB Ethernet adapter to the VM or add a Bridged adapter bound
to that NIC. The arm needs about a minute after power-on. Then:

```bash
./scripts/runtime/start_arm.sh --setup-net    # adds 192.168.123.51/24 if missing, pings the arm
```

It never flushes existing addresses. The address doesn't persist across VM
reboots, so re-run it after a restart. To do it by hand:
`sudo ip addr add 192.168.123.51/24 dev enp0s8 && sudo ip link set enp0s8 up`.

The container must use host networking (`--network host`).

## Run

```bash
ARM_IFACE=enp0s8 ./scripts/tools/diag_arm.sh     # all PASS expected
./scripts/runtime/start_arm.sh --read            # once: calibrate the folded pose (see below)
./scripts/runtime/start_arm.sh                   # fold: enable -> glide -> release (clear the workspace)
docker exec -u root go2-lidar3d env -u CYCLONEDDS_URI \
  python3 /tmp/d1_arm_control.py --iface enp0s8 joint 0 20
```

**Fold calibration.** With the motors released, fold the arm onto its mount by
hand. Run `start_arm.sh --read` and paste the printed `FOLDED_ANGLES` line into
`scripts/nodes/d1_arm_calibration.py`. Until then `start_arm.sh` refuses to fold,
because the shipped placeholder drives joints 1 and 2 to their ±90° limits.
After folding, the motors are **released**, so the mount must hold the arm.

Start with small angles (about 20 degrees).

## Smooth moves, gripper, sequences

```bash
A="docker exec -u root go2-lidar3d env -u CYCLONEDDS_URI python3 /tmp/d1_arm_control.py --iface enp0s8"

$A angles                                  # current pose, 10-sample average
$A enable glide 0 -30 30 0 20 0 0          # smooth move from the measured pose
$A --vmax 40 enable glide 0 -30 30 0 20 0 0
$A gripper 60 2000                         # gripper to 60 deg over 2 s (0 = closed)
$A run /tmp/scan.txt                       # sequence file (docker cp it in first)
```

- `joints` sends one target and lets the arm's controller choose the motion.
  `glide` starts from the **measured** pose. It streams a quintic trajectory
  (zero velocity and acceleration at both ends) at `--rate` Hz (default 50),
  with **peak** joint speed `--vmax` deg/s (default 25). It then waits until
  every joint is within `--tolerance` deg (default 5).
- `glide` **refuses to move without joint feedback**. Planning from a guessed
  start would make the first setpoint a jump.
- Joint angles come from `rt/arm_Feedback` (address 2, funcode 1). If that is
  silent, they come from the `current_servo_angle` topic (`PubServoInfo_`,
  `servo0_data`..`servo6_data`).
- Sequence files take one CLI command per line, and `#` starts a comment:

  ```text
  enable
  glide 0 -30 30 0 20 0 0
  gripper 60 2000
  sleep 2
  ```

`./scripts/runtime/start_arm.sh --setup-net` adds `192.168.123.51/24` to
`enp0s8` if the interface has no `192.168.123.x` address. It never flushes
existing addresses. Re-run it after a VM reboot.

## Troubleshooting

| Symptom | Meaning | Fix |
|---|---|---|
| diag: no interface in 192.168.123.x | arm not on a VM interface, or address lost after reboot | attach adapter, `start_arm.sh --setup-net` |
| no ping from 192.168.123.100 | cable / wrong subnet / still booting | check link, `tcpdump -i <iface>` |
| `DDS match with arm: NO` | discovery failed | `ARM_PEER=192.168.123.100`, pin `cyclonedds==0.10.2`, no ROS sourced |
| `ack: REJECTED` | payload format wrong | check JSON shape against the doc |
| `ack: none` after match | DDS/type mismatch | pin `cyclonedds==0.10.2` |
| arm weak / not holding | enable value | use `--enable-mode 80000` (default) |
| `free(): invalid pointer` crash | ROS's CycloneDDS mixed with SDK's | run without sourcing ROS |

## References

- Unitree D1 doc: https://support.unitree.com/home/en/developer/D1Arm_services
- chen37058/Grasp-with-the-Unitree-D1 (C++ SDK grasping demos)
- SofiaAbal/tfg_go2_arm PR #23 (D1-550 driver: enable mode 80000, home funcode 7)
- Robot-Learning-Group/d1t-teleop (pure Python, `unitree_sdk2py` + cyclonedds, hardware-verified)
- kokekokko481576/Go2_deploy PR #77 (rad to degrees JSON bridge, home, range clamping)
- JeewanthaSadaruwan/unitree-D1-550-Robot-ARM (C++ SDK examples on a Go2-mounted D1-550).
  Ideas reimplemented here, not copied (the repo has no license file): quintic
  trajectory streaming, averaged start-pose sampling, the `current_servo_angle`
  feedback topic, wait-until-reached, gripper `delay_ms`, network setup. Where we
  differ: their planner parses `servoN_data` from `rt/arm_Feedback` and silently
  plans from 0 when it is missing. Their `vmax` caps average, not peak, speed.
  Some of their waypoints (joint 5 at -164 deg) exceed their own URDF limits.