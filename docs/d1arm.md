# Unitree D1 arm: setup and control runbook

Controls a Unitree D1 / D1-T arm over DDS from the `go2-lidar3d` container.
Protocol source: Unitree "D1 Mechanical Arm Services Interface" doc, cross-checked
against working open-source drivers (see References).

## Files

| File | Role |
|---|---|
| `d1_arm_control.py` | Control node (runs in the container). CLI + `D1Arm` class |
| `start_arm.sh` | Enable + home the arm from the host |
| `diag_arm.sh` | Network + DDS diagnostic |
| `direct_test_movement.py` | Small safe movement test (needs `d1_arm_control.py` alongside) |

Keep all four in one folder (or set `NODES_DIR`).

## Protocol summary (verified against the doc and working drivers)

- Topics: `rt/arm_Command` (host -> arm), `rt/arm_Feedback` (arm -> host), type
  `unitree_arm::msg::dds_::ArmString_` (one JSON string), DDS domain 0.
- Command: `{"seq":N,"address":1,"funcode":F,"data":{...}}`. `data` is a nested object.
- Angles are **degrees**. Joints 0-5 arm, joint 6 gripper (servo degrees).
- funcode 1 single joint `{"id","angle","delay_ms":0}`; 2 all joints `{"mode":1,"angle0".."angle6"}`;
  4 single enable; 5 enable all `{"mode":80000}` (0 = release); 6 power; 7 home (no `data`).
- Feedback: address 2 funcode 1 = joint angles (10 Hz), funcode 3 = status;
  address 3 funcode 1 = command received (`recv_status`), funcode 2 = executed.
- Arm default IP `192.168.123.100`.

## Network (the part code cannot fix)

The arm must be on its own wired interface in the VM, with an address on `192.168.123.x`.

```bash
ip -br addr                                   # find the arm-side interface (not your home-LAN one)
sudo ip addr add 192.168.123.51/24 dev <iface>
sudo ip link set <iface> up
ping -c3 192.168.123.100
```

In VirtualBox, attach the USB Ethernet adapter to the VM or add a Bridged adapter bound
to that NIC. The arm needs about a minute after power-on.

The container must use host networking (`--network host`).

## Run

```bash
ARM_IFACE=<iface> ./diag_arm.sh      # all PASS expected
ARM_IFACE=<iface> ./start_arm.sh     # enable + home (clear the workspace first)
docker exec -u root go2-lidar3d env -u CYCLONEDDS_URI \
  python3 /tmp/d1_arm_control.py --iface <iface> joint 0 20
```

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
| diag: no interface in 192.168.123.x | arm not on a VM interface | attach adapter, set IP |
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