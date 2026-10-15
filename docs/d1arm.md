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