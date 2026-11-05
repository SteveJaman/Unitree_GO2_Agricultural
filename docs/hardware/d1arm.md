# Unitree D1 arm: setup and control runbook

[← Back to README](../../README.md) · [Scripts](../setup/scripts.md) · [Gazebo sim](../simulation/gazebo.md) · [Troubleshooting → D1-T Arm](../troubleshooting.md#d1-t-arm) · [Roadmap](../roadmap.md)

Controls a Unitree D1 / D1-T arm over DDS directly from the host (no
container). Protocol source: Unitree "D1 Mechanical Arm Services Interface" doc,
cross-checked against working open-source drivers (see References).

**Verified config:** a wired interface (auto-detected, e.g. `enp0s31f6`), DDS domain 0, **multicast** discovery.
Or plug the arm into the Go2 and control it over Wi-Fi: see
[Arm over Wi-Fi](#arm-over-wi-fi-plugged-into-the-go2).
Do not use `--peer` (unicast): it breaks discovery on this arm's firmware.

## Two ways to move the arm

| | Direct joint control | MoveIt 2 planning |
|---|---|---|
| Entry point | [`start_arm.sh`](../../scripts/runtime/start_arm.sh) → `d1_arm_control.py` | [`arm_moveit.sh`](../../scripts/runtime/arm_moveit.sh) → [`d1_moveit_config`](../../src/d1_moveit_config/) |
| You give | Joint angles in degrees (`joint 1 30`, `joints …`), or poses taught by hand | Where the **gripper** should be: a position and orientation in space |
| Who works out the joints | You do | MoveIt: inverse kinematics (KDL), then a collision-free path (OMPL) |
| Collisions | Only the joint limits are checked: it can hit the dog or a plant | Plans around the Go2's body and the objects in the planning scene |
| Drives | **The real arm**: wired, or over Wi-Fi through the Go2 | The real arm (with `--real`), mock hardware (RViz), or Gazebo (with `--gazebo`) |
| Use it for | Bring-up, calibration, fold, Teach & Playback scans | Designing scan viewpoints around a plant, checking reach, planning |

Direct control is what moves the real arm today. MoveIt is the planning side:
it answers "can the camera reach that viewpoint, and how, without hitting
anything?". Connecting it to the real arm is the open step (see
[From plan to the real arm](#from-plan-to-the-real-arm)).

## Files

| File | Role |
|---|---|
| `scripts/runtime/start_arm.sh` | Entry point. Runs a preflight (host address, ping), then hands the command to the right script |
| `scripts/nodes/d1_arm_control.py` | Chain-style CLI + `D1Arm` class: `state`, `enable`, `release`, `home`, `power`, `joint`, `joints`, `sleep`, `feedback`, `status`, `motors`, `lift`, `sweep` |
| `scripts/nodes/d1_arm_calibration.py` | `--read` the folded pose, `--move` fold (power on → enable → move → release → power off) |
| `scripts/tools/calibrate_arm.py` | Measures per-joint offset (commanded vs reported angle) with small ± test moves |
| `scripts/tools/teach_arm.py` | Plant scanning: guide the arm by hand in compliance mode, save scan poses to `config/arm_waypoints.json` |
| `scripts/tools/play_arm.py` | Plant scanning: replay the saved poses (home → each pose, hold for the camera → home), smooth + PID (`start_arm.sh play`) |
| `scripts/nodes/d1_motion.py` | Shake-free moves: minimum-jerk trajectory streamed as setpoints + outer PID on the measured angles |
| `scripts/nodes/orbit_scan.py` | Orbit scan: scan pose, base sweep −90° → +90° with the wrist counter-rotating to keep the camera on a table-top object |
| `tests/test_arm_tracker_pure.py` | **Visual Servoing Test:** Standalone script using YOLO and RealSense to dynamically track a person. |
| `scripts/tools/release_arm.py` | Release the holding tension: ramps the stiffness down to 0 so the arm sinks slowly, `--now` at once, `--power-off` |
| `scripts/tools/diag_arm.sh` | Network + DDS diagnostic |
| `scripts/nodes/direct_test_movement.py` | Small safe movement test (needs `d1_arm_control.py` alongside) |
| `scripts/robot/arm_bridge_wifi.{sh,py}` | **On the robot:** the arm (plugged into the Go2) ↔ laptop over Wi-Fi, TCP 18100 |
| `scripts/robot/unitree_arm/` | One-message ROS 2 package (`ArmString`) the bridge is built with on the robot |
| `scripts/runtime/arm_moveit.sh` | **MoveIt 2:** builds the MoveIt package and the D1 description, starts RViz + `move_group`; `--real` uses the hardware bridge, `--inspect` runs the plant-inspection demo |
| `src/d1_moveit_config/` | MoveIt 2 config for the D1-550 on the Go2: URDF + SRDF, kinematics, joint limits, controllers, launch, `inspect_plant.py` |
| `scripts/nodes/moveit_d1_bridge.py` | **Hardware Bridge:** Translates MoveIt trajectories (radians) into D1 DDS JSON commands (degrees) using the `d1_motion.py` smoothing. |

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
- Joint limits (degrees, Unitree's published ranges, as in the D1-550 URDF): ±135,
±90, ±90, ±135, ±90, ±135 (`JOINT_LIMITS`). Commands and computed motions (jog,
sweep, orbit, IK scan, periscope) stay inside them; poses taught by hand may go
up to 10° further (`JOINT_REACH`), since the real arm reaches a little past the
published ranges.
  Joint names: 0 base, 1 shoulder, 2 elbow, 3 wrist roll, 4 wrist pitch, 5 wrist yaw.
- Arm default IP `192.168.123.100`.

## Hardware

- **Power:** 24V DC via the DC 5.5-2.1 barrel jack (~1 min boot)
- **Communication:** RJ45 to a wired port of the workstation (e.g. `enp0s31f6`),
  or to the Go2's Ethernet port ([over Wi-Fi](#arm-over-wi-fi-plugged-into-the-go2)).
  Arm at `192.168.123.100`, host at `192.168.123.99` (the `go2-arm`
  NetworkManager profile, see [Network](#network)). DDS
  discovery is multicast. It is kept separate from the Go2's Wi-Fi DDS
- **Debug:** USB-C (serial debug only)

## Network

The arm must be on its own wired interface of the workstation (auto-detected:
the wired NIC on `192.168.123.x`, else the first wired NIC; here `enp0s31f6`).
The host side needs the address **`192.168.123.99/24`** on that interface. The
Go2's Wi-Fi interface must be on a different subnet, otherwise arm and Go2 DDS
traffic mix. `wifi_nav.sh --arm` refuses an overlapping setup.

Give the port its address once with a NetworkManager profile. It persists across
reboots and comes up whenever the cable is plugged in. The arm needs about a
minute after power-on. Then:

```bash
sudo nmcli connection add type ethernet ifname enp0s31f6 con-name go2-arm \
  ipv4.method manual ipv4.addresses 192.168.123.99/24 ipv4.never-default yes ipv6.method ignore
sudo nmcli connection up go2-arm
./scripts/runtime/start_arm.sh --check     # preflight: address + ping
```

If the arm doesn't reply to ping, check its 24V power and the cable, then
power-cycle the arm.

Overrides: `ARM_IFACE` (default: auto-detected), `ARM_IP` (`192.168.123.100`),
`ARM_HOST_IP` (`192.168.123.99`), `ARM_DOMAIN` (`0`).

## Arm over Wi-Fi (plugged into the Go2)

The arm's address, `192.168.123.100`, is on the Go2's internal network (the
Jetson is `192.168.123.18`). Plug the arm's RJ45 into the Go2's Ethernet port
and the laptop controls it over Wi-Fi, like the dog, with no cable to the
workstation:

```text
D1 arm ──eth0, DDS domain 0──► arm_bridge_wifi.py (Jetson) ──Wi-Fi, TCP 18100──► laptop D1Arm
```

- **Bridge (robot):** [`arm_bridge_wifi.py`](../../scripts/robot/arm_bridge_wifi.py)
  talks to the arm with the robot's ROS 2 Foxy (`rclpy`, message
  `unitree_arm/ArmString` = DDS `unitree_arm::msg::dds_::ArmString_`) on
  `eth0`, multicast, as on the workstation. It loads Unitree's CycloneDDS
  (`robot-source-unitree-ros.sh`, copied alongside): `rclpy` with Foxy's
  stock CycloneDDS segfaults creating a node on the Go2. It serves the laptop one JSON
  message per line over TCP: arm commands in, every `rt/arm_Feedback` message
  out.
- **Why TCP:** arm traffic is small JSON at 10 Hz. TCP needs no DDS discovery
  over Wi-Fi and loses nothing (see the camera's Wi-Fi DDS problems in
  [troubleshooting](../troubleshooting.md#camera)).
- **Laptop:** `D1Arm` (`d1_arm_control.py`) takes `--robot <Go2 Wi-Fi IP>` or
  `ARM_ROBOT`. Every arm script follows it unchanged: the control CLI, `fold`,
  `read`, offset calibration, `teach_arm.py`, `play_arm.py`. `cyclonedds` is
  not needed on the laptop for this.
- **Safety:** the laptop pings every second. If it disconnects, is silent for
  3 s (`ARM_BRIDGE_TIMEOUT`), or the bridge is stopped while the arm is
  **compliant** (teach mode, `0 < mode < 80000`), the bridge stiffens it
  (mode 80000): a Wi-Fi drop never lets the camera sag. A fully enabled arm
  already holds by itself and a released one rests, so the bridge sends them
  nothing. Only the laptop (`GO2_HOST_IP`) may connect, and only arm commands
  are forwarded.

**Firmware behaviour (found 2026-10-27):** enable-all (`funcode 5`, mode
80000) sent to an arm that is **already enabled** drives it back to its last
multi-joint (`funcode 2`) target. The first bridge re-sent it on every
disconnect: an interrupted `home` swung the shoulder, and a later one folded
the arm on its own. Never re-enable an enabled arm; `fold` and the bridge now
check first.

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
./scripts/tools/diag_arm.sh                      # all PASS expected
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
until the pose is reached (±3°, up to 30 s), then **releases and powers off**,
so the mount must hold the arm. If the pose is not reached it does not release
(exit 3). It skips power-on / enable when the arm already is.

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

Joint limits (degrees, Unitree's published ranges, as in the D1-550 URDF): ±135,
±90, ±90, ±135, ±90, ±135 (`JOINT_LIMITS`). Commands and computed motions (jog,
sweep, orbit, IK scan, periscope) stay inside them; poses taught by hand may go
up to 10° further (`JOINT_REACH`), since the real arm reaches a little past the
published ranges.
The scripts reject anything beyond them.

## Parameters

| Flag / env | Default | Notes |
|---|---|---|
| `--iface` / `ARM_IFACE` | auto-detected (e.g. `enp0s31f6`) | Interface that reaches the arm |
| `--ip` / `ARM_IP` | `192.168.123.100` | Arm address (preflight ping and route check) |
| `ARM_HOST_IP` | `192.168.123.99` | Host address `start_arm.sh` expects on the interface |
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
whole sweep later. By default the arm is **fully free** while teaching
(`funcode 5`, `{"mode": 0}`): no resistance at all, so it is easy to place, but
nothing holds it up either: **keep a hand on it**. `--stiffness 5000` uses compliance mode instead: the motors hold the arm (and camera) up but yield to
a push.

**1. Teach** (one hand on the arm, the other on the keyboard):

```bash
python3 scripts/tools/teach_arm.py          # add poses (a saved home is kept)
python3 scripts/tools/teach_arm.py --new    # new home + new sweep
```

1. The arm powers on and goes free: *"Arm is free (no stiffness) ... Keep a
   hand on it."* (with `--stiffness N`: *"Arm is compliant"*)
2. **HOME first:** move it to its home / rest pose and press **Enter**. The
   sweep starts and ends there. A file that already has a home keeps it.
3. **Then each scan pose:** move it until the camera frames the plant and
   press **Enter**, or type a name first (e.g. `left leaf`). The six joint
   angles (telemetry address 2, funcode 1) go to `config/arm_waypoints.json`,
   written after every pose. At the prompt: **u** undoes the last pose, **h**
   re-records home, **l** lists home and the poses.
4. Poses are saved exactly as measured, up to ±150° on any joint (the arm
   reaches a little past Unitree's published ranges when placed by hand: its
   fold reads joint 2 at +91.8°). Readings without telemetry or beyond ±150°
   are refused.
5. Type **q** to finish. The arm locks (`mode 80000`) where it is; Ctrl+C or
   Ctrl+D lock it too, so it never drops.

If you want the arm to stay where you leave it while you reach for the
keyboard, use `--stiffness 5000` (raise it if it sags, lower it if it is hard
to push). Over Wi-Fi a dropped link leaves a free arm free (the bridge only
stiffens a partly compliant one); quitting always locks it.

**2. Play back:**

```bash
python3 scripts/tools/play_arm.py --dry-run  # check the file and print the poses
./scripts/runtime/start_arm.sh play          # run the sweep (preflight first)
python3 scripts/tools/play_arm.py            # the same, directly
```

Home and every waypoint are checked (±150°) before anything moves. Then the
arm powers on, locks, goes to the **recorded home** (the factory zero pose,
`funcode 7`, if the file has no home or with `--factory-home`), visits each
pose, prints *"Waypoint N reached"* (with its name) and holds `--dwell`
seconds (1.5).

**Ruckig S-curve motion (default).** Every move is planned with
[Ruckig](https://github.com/pantor/ruckig), an online trajectory generator: a
time-optimal, **jerk-limited (S-curve)** path from the measured angles to the
waypoint, at rest at both ends, all six joints synchronised so they start and
arrive together. Acceleration ramps up and down instead of jumping, which is
what made the arm wobble. The limits (per joint) sit at the top of
[`play_arm.py`](../../scripts/tools/play_arm.py) and are overridable:

| Limit | Constant | Option / env | Default |
|---|---|---|---|
| velocity | `RUCKIG_MAX_VEL` | `--max-vel` / `ARM_MAX_VEL` | 60 °/s |
| acceleration | `RUCKIG_MAX_ACC` | `--max-acc` / `ARM_MAX_ACC` | 90 °/s² |
| jerk | `RUCKIG_MAX_JERK` | `--max-jerk` / `ARM_MAX_JERK` | 300 °/s³ |

The trajectory is sampled at **50 Hz** (`STREAM_HZ`, every 0.02 s) with
`traj.at_time(t)` and sent as `funcode 2` in **mode 0** (direct tracking),
for exactly `traj.duration` seconds; nothing waits on telemetry during the
move (one reading afterwards reports the final error). A 90° move takes about
2.5 s; each setpoint is at most 1.2° from the last.

Ruckig runs **on the workstation only** (`pip install --user ruckig`); the
Go2 needs nothing new. With the arm on the Go2, the sampled points go over
Wi-Fi in **one** `traj_points` message and the robot's bridge plays them at
the exact 50 Hz over its wired link (streamed point by point over Wi-Fi,
latency spikes would bunch them and shake the arm). Wired, or with an older
bridge, the laptop streams them itself.

**Ctrl+C** during a move stops the stream at once (the robot's playback too,
`traj_stop`) and **releases the arm** (`funcode 5`, mode 0): it goes limp, so
keep a hand under it. `--on-interrupt lock` (`ARM_ON_INTERRUPT=lock`) holds
it where it stopped instead.

**Min-jerk + PID (`--planner minjerk`).** The earlier planner, still
available (`ARM_PLANNER=minjerk`). One big `funcode 2` jump makes the servos
accelerate hard, overshoot and ring around the target, and corrections near
the target make it hunt. So every move ([`d1_motion.py`](../../scripts/nodes/d1_motion.py)):

1. follows a **minimum-jerk trajectory** from the measured angles to the
   target (zero speed and acceleration at both ends), timed so no joint
   exceeds `--max-vel` (30 °/s) or `--max-acc` (60 °/s²); all joints arrive
   together;
2. is sent as small setpoints. **With the arm on the Go2, the trajectory runs
   on the robot** (the bridge's `traj` request, 50 Hz over its wired link):
   streamed from the laptop, Wi-Fi latency spikes (200 ms measured) bunch the
   setpoints into bursts and the arm shakes on every transition. The robot
   sends them in mode 0 (direct, `--robot-mode`): mode 1 restarts the
   firmware's ramp at every step, the arm falls behind and lurches at the end
   (simulated wrist wobble ±20° after arriving; mode 0: none). Wired, or with
   an older bridge, the laptop streams them at 20 Hz in `--mode` (1);
3. runs an **outer PID** per joint on (target − measured) **once the move is
   done** (correcting the lag mid-move fights the servos), which removes
   gravity sag on the shoulder and elbow. P and D act only outside a
   `--deadband` (0.5°); the light wrist joints (3–5) get no P or D, only a
   slow integral and a 1° deadband (`--wrist-kp/-ki/-deadband`). The integral
   and the whole correction (±6°) are capped, and a correction is re-sent
   only when it changes by more than 0.2°;
4. **settles**: it holds the target until every joint stays within
   `--tolerance` (2°) for 0.5 s, or `--move-timeout` passes ("not reached …;
   moving on").

While holding each pose it prints `holding: steady` or the joints that wander
(`--jitter-warn`, 0.8°).

**Restart the arm bridge on the robot once** to get the `traj` and
`traj_points` features (the
bridge is copied when `start_arm.sh` starts it):
`ssh unitree@ubuntu.local "pkill -TERM -f arm_bridge_[w]ifi"`, then
`./scripts/runtime/start_arm.sh --check`.

In a simulation of an underdamped, sagging arm (10 Hz telemetry, 0.2° noise),
a single jump of the base by 87° overshot 42° and rang; the smooth PID move
overshot 1.3° with no ringing and finished within 1.2° (the jump left 2.5° of
sag). The cost is time: an 87° move takes ~5 s at 30 °/s.

| Option | Env (also for `start_arm.sh play`) | Default | Effect |
|---|---|---|---|
| `--planner` | `ARM_PLANNER` | `ruckig` | `minjerk`: the min-jerk + PID move |
| `--max-vel` | `ARM_MAX_VEL` | 60 °/s (minjerk: 30) | faster / slower moves |
| `--max-acc` | `ARM_MAX_ACC` | 90 °/s² (minjerk: 60) | gentler starts and stops when lower |
| `--max-jerk` | `ARM_MAX_JERK` | 300 °/s³ | ruckig: smoother ramps when lower |
| `--on-interrupt` | `ARM_ON_INTERRUPT` | `release` | Ctrl+C: `lock` holds the arm instead of releasing it |
| `--kp` / `--ki` / `--kd` | `ARM_KP` / `ARM_KI` / `ARM_KD` | 0.3 / 0.4 / 0.02 | correction strength; lower `kp` if it still oscillates, raise `ki` if it stops short |
| `--deadband` | `ARM_DEADBAND` | 0.5° | larger if it twitches at rest |
| `--no-pid` | `ARM_PID=0` | off | one jump per pose, no trajectory (the old behaviour) |

At the end it goes home again and **stays locked**; `--release` releases it at home (support
it first, or fold it with `start_arm.sh fold` instead). An error locks the arm
where it stopped; Ctrl+C releases it (see above).

To take the holding tension off afterwards (or after teaching, which leaves
the arm locked):

```bash
python3 scripts/tools/release_arm.py              # stiffness ramped to 0 over 3 s: the arm sinks slowly
python3 scripts/tools/release_arm.py --power-off  # ... and motor power off
```

`--now` releases at once (the arm drops), `--ramp` / `--steps` tune the ramp.
It asks first unless `--yes`.

`config/arm_waypoints.json` is user data (git-ignored):

```json
{"home": {"angles": [1.8, -88.8, 90.0, 1.3, 23.0, 0.6], "saved": "2026-10-29T19:05:02"},
 "waypoints": [{"angles": [10.5, -32.0, 45.1, 0.0, 20.0, 0.0], "name": "left leaf", "saved": "2026-10-29T19:05:12"}]}
```

Both scripts take the usual `--iface`, `--ip`, `--domain`,
`--skip-preflight` and `--file` (another waypoint file, e.g. one per crop).

## Orbit scan (`start_arm.sh orbit`)

A hardcoded scan around a table-top object, no teaching needed:

```bash
./scripts/runtime/start_arm.sh orbit                 # preflight, asks before moving
./scripts/runtime/start_arm.sh orbit --yes           # no prompt
python3 scripts/nodes/orbit_scan.py --dry-run        # print the 30 steps, no arm
```

1. Power on, enable (mode 80000), home.
2. Move to `SCAN_POSE = [0, -40, 60, 0, 30, 0]` (camera facing down).
3. Sweep joint 0 (base) from −90° to +90° in 30 steps (about 6.2° each). Each
   step is one multi-joint move that also sets joint 5 (wrist) to
   `-base * 0.8`, so the camera stays on the object.
4. Home and release (mode 0).

The movement runs in a `try/finally`: on Ctrl+C, an error or a lost link the
arm is homed and then **always released** (mode 0). Homing first keeps the
limp arm from dropping out of the scan pose. Clear the space around the arm
first: the base turns ±90°.

Works on either link like every `start_arm.sh` command: wired (the interface
is auto-detected, override with `--iface` / `ARM_IFACE`), or with the arm
plugged into the Go2 over Wi-Fi (chosen automatically when the wired port
has no cable link, or `start_arm.sh --robot <Go2 IP> orbit`). Options:
`--dwell` (s held per step, default 0.3), `--tolerance` (°, default 3),
`--move-timeout` (s, default 15). The pose, range, step count and wrist gain
are constants at the top of `orbit_scan.py`.

## IK scan (`start_arm.sh scan`)

Aim the wrist camera at a point and sweep across it, solved with inverse
kinematics ([`arm_ik_scanner.py`](../../scripts/nodes/arm_ik_scanner.py), `ikpy`)
instead of fixed angles. The dashboard's Inspect mode runs it for you; by hand:

```bash
python3 scripts/nodes/arm_ik_scanner.py --target 0.55 0 0 --dry-run   # solve, print, no arm
./scripts/runtime/start_arm.sh scan --target 0.55 0 0                  # preflight, confirm, run
```

- The target is in the Go2's `base_link` (m: x forward, y left, z up).
- The chain is the D1 URDF (`twin/webapp/public/d1_description`) from `base_link`
  to `Link6` plus a camera point 0.09 m along the gripper axis. The camera is placed
  above and in front of the target looking down (or on the shoulder line for
  distant targets), 0.15–0.45 m away.
- Aim points: the target, then left to right across it (`--side 0.1`,
  `--steps 5`), then the target again. Each pose is one multi-joint move
  (funcode 2, mode 1), `--delay 0.5` s apart.
- Refused before anything moves: more than 3° off aim, a joint at its limit, the
  camera closer than 0.15 m, or a link inside the Go2's body box.
- Like `orbit`, a `try/finally` homes the arm and releases it (mode 0) on Ctrl+C
  or an error.

**Check on the real arm first.** The angle mapping (protocol angle *i* in degrees
= URDF `Joint(i+1)` in radians, same zero) comes from the Gazebo emulator and has
not been confirmed on the hardware. Run `--dry-run`, then a first run with the
space clear, and check the 3D arm in the dashboard matches the real one.

## Visual Servoing Tracker (`test_arm_tracker_pure.py`)

A standalone test script that proves the camera can actuate the arm in real-time without ROS 2, MoveIt, or Nav2. It relies on **visual servoing**: if a person moves to the left of the camera's view, the arm rotates left to keep them centered.

**Requirements:**

- The RealSense D435i must be plugged directly into the workstation (via USB).
- Python packages: `pip install pyrealsense2 ultralytics opencv-python`
- The arm must be reachable (wired DDS or via the Go2 Wi-Fi bridge).

**How to run:**

```bash
python3 tests/test_arm_tracker_pure.py
```

**How it works:**

1. **Boot:** The script connects to the arm and moves it to a "READY" pose (J1: -40°, J2: 60°).
2. **Track:** It grabs frames from the RealSense, runs YOLO11s to find a `person`, and calculates the `(X, Y)` pixel offset of the person from the center of the image.
3. **Actuate:**
   - **Joint 0 (Base):** Sweeps left/right to center the person horizontally.
   - **Joint 1 (Shoulder):** Tilts up/down to center the person vertically.
   - It uses a `deadzone` of 30 pixels and a `step_size` of 1.5° to prevent jerky movements.
4. **Exit:** Pressing `q` or `Ctrl+C` stops the loop and safely releases the arm motors.

**Note:** This is a raw logic test. It does not use inverse kinematics to calculate exact 3D coordinates; it just adjusts joint angles until the YOLO bounding box is centered in the frame.

## The arm in the web dashboard

The 3D dashboard ([3d_nav.md](../ai_and_vision/3d_nav.md#web-dashboard)) draws the D1 on
the Go2's back with live joint angles and a translucent blue cone for the
wrist camera's view. It has no arm buttons: run the orbit scan and the
periscope pose with `start_arm.sh orbit` / `periscope` (the Orbit Scan and
Periscope Scan buttons and the camera's Wrist view were removed on 2026-11-01;
the wrist camera is not mounted).

The arm has no ROS `JointState` of its own (it speaks JSON on raw DDS), so
[`scripts/nodes/arm_state_bridge.py`](../../scripts/nodes/arm_state_bridge.py), on the host:

- reads `rt/arm_Feedback`: wired DDS on the arm's Ethernet, or, with the arm
  plugged into the Go2, the bridge's **read-only monitor port** (TCP 18101,
  `ARM_BRIDGE_PORT + 1`), so watching never blocks `start_arm.sh`;
- publishes `/d1_arm/joint_states` (`sensor_msgs/JointState`, `Joint1`..`Joint6`
  in radians, fingers `Joint7_1`/`Joint7_2`) and `/d1_arm/status` through rosbridge;
- runs `start_arm.sh orbit --yes` on `{"op":"orbit"}` from `/d1_arm/command`
  (any ROS client; the dashboard no longer has a button), `start_arm.sh scan --target dx dy dz --yes`
  on `{"op":"scan_target","target":[dx,dy,dz]}` (inspection_mission.py), and
  Ctrl+C's either on `{"op":"stop"}`.
  `--no-commands` ignores all commands (status only).

`wifi_3d_nav.sh` starts it with every session (`TWIN_ARM=0` turns it off;
log `ws/log/3d/arm_state.log`, scan log `arm_orbit.log`). The arm bridge on the
robot gets the monitor port the next time `start_arm.sh` or `wifi_nav.sh
--arm-wifi` (re)starts it.

**No robot:** `./scripts/tools/mock_arm_twin.sh` serves the dashboard with a
fake arm (`tests/mock_sim.py --arm --tf`) on an isolated ROS domain.
Open `http://localhost:3000`, then, in another terminal, play the same sweep as
`orbit_scan.py`:

```bash
docker exec go2-3d-mock-arm bash -c "source /opt/ros/humble/setup.bash; ros2 topic pub --once /d1_arm/command std_msgs/msg/String '{data: \"{\\\"op\\\": \\\"orbit\\\"}\"}'"
```

The model is Unitree's `d1_description` (BSD), decimated from 133k to 9k
triangles by [`scripts/tools/build_d1_model.py`](../../scripts/tools/build_d1_model.py)
into `twin/webapp/public/d1_description/`, and loaded with `urdf-loader`.

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
used here, so a ±90° target is clamped by 0.05°. See [docs/simulation/gazebo.md](../simulation/gazebo.md).

## MoveIt 2: Cartesian planning (`arm_moveit.sh`)

[`src/d1_moveit_config`](../../src/d1_moveit_config/) is a MoveIt 2 setup for
the D1-550 on the Go2's back. Instead of joint angles, you give MoveIt a target
for the gripper (a position and an orientation in space). MoveIt solves the
inverse kinematics and plans a collision-free motion to it. It can run on mock
hardware (the arm moves in RViz only), Gazebo (for simulation), or the real arm
(with `--real`). With `--real`, `moveit_d1_bridge.py` takes the place of the
mock controller and plays MoveIt's trajectories on the arm; there is no mock arm
behind it.

### Install and run

One-time, on the host (no Docker):

```bash
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers
git clone https://github.com/JeewanthaSadaruwan/unitree-D1-550-Robot-ARM ~/files/unitree-D1-550-Robot-ARM   # d1_550_description
```

```bash
./scripts/runtime/arm_moveit.sh              # build if needed, then RViz + MoveIt
./scripts/runtime/arm_moveit.sh --inspect    # same, then the plant-inspection demo
ARM_JOINT_SIGNS="1 1 1 1 1 1" ./scripts/runtime/arm_moveit.sh --real   # MOVES THE ARM: see "Joint signs"
./scripts/runtime/arm_moveit.sh --real --robot <go2-ip>                # arm plugged into the Go2 (or ARM_ROBOT)
./scripts/runtime/arm_moveit.sh --real --camera                        # camera on the wrist flange, as the sim trial
./scripts/runtime/arm_moveit.sh --build      # force a rebuild first
./scripts/runtime/arm_moveit.sh --no-rviz    # headless (move_group + controllers only)
```

What `arm_moveit.sh` does:

1. Checks for ROS 2 Humble, MoveIt, ros2_control and the controllers, and
   prints the `apt install` line if one is missing.
2. Builds `d1_550_description` (from `D1_ARM_DIR`, default
   `~/files/unitree-D1-550-Robot-ARM`) and `d1_moveit_config` into this repo's
   `install/`, the first time or with `--build`.
3. Isolates DDS: `ROS_DOMAIN_ID=78` (`ARM_ROS_DOMAIN_ID`),
   `ROS_LOCALHOST_ONLY=1`, no `CYCLONEDDS_URI`. Nothing it publishes can reach
   the robot (domain 64) or the arm (domain 0). Only `--real` touches the
   arm, through the bridge.
4. Launches [`demo.launch.py`](../../src/d1_moveit_config/launch/demo.launch.py):
   `robot_state_publisher`, `move_group` (OMPL), `ros2_control` with the
   `arm_controller` and `gripper_controller` trajectory controllers, and RViz.

To use the same ROS graph from another terminal:

```bash
export ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run d1_moveit_config inspect_plant.py --ros-args -p plan_only:=true
```

### Planning in RViz

In the **MotionPlanning** panel, with planning group `arm`:

1. Drag the interactive marker at the gripper (`tool0`) to where the camera
   should be, and rotate it to where it should look.
2. **Plan**: MoveIt solves IK for that pose and searches a collision-free
   path. The preview animates the motion; a red arm means no valid plan.
3. **Execute**: the mock arm follows the trajectory.

Named states: `home` for the arm (all joints 0: upper arm vertical, forearm
forward), `open` / `closed` for the gripper (group `gripper`).

### Cartesian goals vs joint control

With direct control you command joints, for example `joint 1 30` or
`joints 0 -30 30 0 20 0 0`. The resulting camera position depends on all six
angles together. Nothing checks the arm against the dog's body or the plant;
only the joint limits are enforced.

With MoveIt you command the **end effector** in Cartesian space:

- **Goal:** a position for `tool0` (within 1 cm in `inspect_plant.py`) and an
  orientation (its x axis, the camera's view direction, within ~9°; roll about
  it is free).
- **IK:** the KDL solver
  ([`kinematics.yaml`](../../src/d1_moveit_config/config/kinematics.yaml),
  5 ms resolution, 50 ms timeout) finds joint angles that reach it.
- **Path:** OMPL searches a joint-space path from the current pose that avoids
  every collision object: the Go2 trunk, the space its legs occupy, and
  anything added to the planning scene (the bed and plant in the demo).
- **Timing:** the trajectory is time-parameterised from
  [`joint_limits.yaml`](../../src/d1_moveit_config/config/joint_limits.yaml),
  scaled to 30 % by default because the arm rides on a walking robot.

So the **end point** is Cartesian, but the path between poses is planned in
joint space and is not a straight line. MoveIt's straight-line
`compute_cartesian_path` (for example a linear sweep along a leaf row) is not
used yet.

| Units and names | Direct control | MoveIt |
|---|---|---|
| Arm joints | `0`–`5` | `Joint1`–`Joint6` |
| Gripper | joint `6`, servo degrees | `Joint_L` / `Joint_R`, finger travel in metres (0–0.033) |
| Angles | degrees | radians |
| Limits | ±135, ±90, ±90, ±135, ±90, ±135 | from the D1-550 URDF (±134.6° / ±89.95°) |

The joint order matches. The signs have **not** been checked against the
firmware: see [Joint signs](#joint-signs).

### Model

- **`world`** is the ground under a standing Go2.
- **The Go2** is two fixed collision boxes: the trunk, 0.29 m up, and the space
  its legs occupy. Plans keep the arm off the dog.
- **The D1 base** sits 0.057 m above the trunk origin, matching
  `gazebo/sim/build_model.py`.
- **`tool0`** is between the fingertips, 0.10 m past `Empty_Link6`, with x
  pointing out of the gripper: a wrist camera would look along it.

| File | Role |
|---|---|
| [`d1_go2.urdf.xacro`](../../src/d1_moveit_config/config/d1_go2.urdf.xacro) | Go2 boxes + the unchanged D1-550 URDF + `tool0` |
| [`d1_go2.ros2_control.xacro`](../../src/d1_moveit_config/config/d1_go2.ros2_control.xacro) | `mock_components/GenericSystem`; the block a real-arm plugin would replace |
| [`d1_go2.srdf`](../../src/d1_moveit_config/config/d1_go2.srdf) | Groups `arm` (`base_link` → `tool0`) and `gripper`, named states, disabled collision pairs |
| [`kinematics.yaml`](../../src/d1_moveit_config/config/kinematics.yaml) | KDL IK solver |
| [`joint_limits.yaml`](../../src/d1_moveit_config/config/joint_limits.yaml) | Velocity / acceleration limits, 0.3 default scaling |
| [`moveit_controllers.yaml`](../../src/d1_moveit_config/config/moveit_controllers.yaml), [`ros2_controllers.yaml`](../../src/d1_moveit_config/config/ros2_controllers.yaml) | `FollowJointTrajectory` controllers for the arm and gripper |
| [`moveit_controllers_real.yaml`](../../src/d1_moveit_config/config/moveit_controllers_real.yaml) | `hardware:=real`: the arm only, with the execution tolerances a real arm needs |
| [`demo.launch.py`](../../src/d1_moveit_config/launch/demo.launch.py) | MoveIt demo launch (OMPL pipeline) |
| [`inspect_plant.py`](../../src/d1_moveit_config/scripts/inspect_plant.py) | Plant-inspection demo |
| [`moveit_d1_bridge.py`](../../scripts/nodes/moveit_d1_bridge.py) | The hardware bridge for `--real` |

### Plant-inspection demo (`--inspect`)

[`inspect_plant.py`](../../src/d1_moveit_config/scripts/inspect_plant.py)
performs a **discrete 3-point scan** of a plant. Its defaults are the
[hydroponic trial's](../simulation/hydroponic_trial.md) rack and lettuce (rack
0.864 m high, plant centre at 0.954 m, camera 0.15 m from it), placed **behind**
the dog, where the arm faces (mount yaw 180°). The dog parks with its **tail**
0.333 m from the rack's front edge.

**Flow (7 steps):**

1. **Pre-grasp** — extend from stow to the Front pose.
2. **Front** (0° azimuth) — camera aimed at the plant centre, 15° look-up. Hold 3 s for capture.
3. **Left** (−30° azimuth). Hold 3 s.
4. **Right** (+30° azimuth). Hold 3 s.
5. **Front** — return to the front view (confirmation shot).
6. **Retract** — the arm is already at pre-grasp.
7. **Stow** — fold arm to walking pose (`J2 = −80°, J3 = 45°, J5 = 80°`).

The stow pose keeps the arm folded tight over the dog's back:
`STOW_ANGLES = [0.0, −1.396, 0.785, 0.0, 1.396, 0.0]` (radians). The mock arm
starts there too, like the folded real arm.

| Parameter | Default | Meaning |
|---|---|---|
| `plant_x`, `plant_y` | `-0.465`, `0.0` | Plant position (world: ground under the dog, x forward) |
| `bed_height`, `bed_depth`, `bed_width` | `0.864`, `0.254`, `1.02` | The rack, centred on the plant |
| `canopy_above_bed`, `canopy_radius`, `standoff` | `0.09`, `0.055`, `0.095` | Plant centre, its size, camera distance from its surface |
| `look_up_deg` | `15.0` | Camera look-up angle at each pose (degrees) |
| `azimuth_deg` | `30.0` | Left and Right, either side of Front (degrees) |
| `tool_link` | `tool0` | Link aimed at the plant; `camera_link` with `--camera` |
| `dwell_s` | `3.0` | Pause at each of the 3 viewpoints (the capture moment) |
| `velocity_scaling` | `0.3` | Speed and acceleration scaling (0.2 with `--real`) |

```bash
# Sim / mock hardware:
./scripts/runtime/arm_moveit.sh --inspect

# Real arm (velocity scaled to 20 %, bridge auto-started):
ARM_JOINT_SIGNS="1 1 1 1 1 1" ./scripts/runtime/arm_moveit.sh --real
```

Against the solid rack box, ±30° at 15° look-up reaches 3/3 viewpoints with the
gripper and the camera; ±35° also does with the camera. ±45° at 25° reaches only
1/3: the camera ends up just above the box. The trial's ±45° orbit ran against
Gazebo's detailed rack, not this solid box. The arc itself (`compute_cartesian_path`) only runs in
[`full_mission_test.py`](../../src/d1_moveit_config/scripts/full_mission_test.py),
in Gazebo.

### Orbit sweep (`d1_active_perception`)

[`orbit_scanner.py`](../../src/d1_active_perception/d1_active_perception/orbit_scanner.py)
moves the camera through 13 poses on a horizontal arc around a plant, −90° to
+90° from the side facing the arm, aimed at the plant centre. After each one it
settles and prints a placeholder where the camera capture will go. A pose that
cannot be planned is logged and skipped.

```bash
colcon build --symlink-install --packages-select d1_active_perception
source install/setup.bash
export ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1

ros2 launch d1_moveit_config demo.launch.py end_effector:=camera   # terminal 1
ros2 run d1_active_perception orbit_scanner --ros-args -p planning_frame:=world -p radius:=0.25   # terminal 2
```

**The arm cannot complete 180°.** Planned on the mock arm with no rack in the
scene (2026-11-05):

| Target (`-0.45, 0`), radius | Reached of 13 | Covers |
|---|---|---|
| Defaults: `base_link`, z 0.85, r 0.35 | 0 | Nothing: `base_link` is turned 180°, so this is ahead of the dog and 0.96 m from the arm base |
| `world`, z 0.85, r 0.35 or 0.30 | 5 | ±30° |
| `world`, z 0.85, r 0.25 | 7 (6 when executed) | ±45° |
| `world`, z 0.75, r 0.25 | 7 | ±45° |
| `world`, z 0.60, r 0.20 | 7 | ±45° |

Near the edge of reach the count varies from run to run.

- `ee_link` defaults to `camera_optical_frame` (z forward). `camera_link` has x
  forward, so the look-at orientation would aim the wrong axis.
- `camera_link` and `camera_optical_frame` exist only with `end_effector:=camera`.
- It adds no rack or plant to the planning scene and has not been run on the
  real arm or in Gazebo. Do not run it near a real rack.
- `--real` does not start it: it runs `inspect_plant.py`.

### From plan to the real arm

**Implemented; tested on a simulated arm, pending a physical test.**

`demo.launch.py hardware:=real` starts `move_group` without `ros2_control`, and
[`moveit_d1_bridge.py`](../../scripts/nodes/moveit_d1_bridge.py) is the only
`/arm_controller/follow_joint_trajectory` server and the only `/joint_states`
publisher. `arm_moveit.sh --real` starts MoveIt only once the bridge reports the
real arm's joints, and stops if the bridge cannot reach the arm.

**It starts by moving a folded arm to stow.** MoveIt's model counts the real
fold as a self-collision (the wrist against `Empty_Link1`) and plans nothing from
it, so with `ARM_START_STOW=1`, which `--real` sets, the bridge glides the arm to
stow at 15 °/s before MoveIt starts. That path was checked pose by pose in
MoveIt: the only contacts are between the arm's own links near the fold, and it
never touches the dog's boxes. It runs only when the shoulder, elbow and wrist
pitch are within 8° of `FOLDED_ANGLES` (the roll joints 0, 3 and 5 within 35°):
from any other pose the arm stays where it is, and the log names the joint that
is off. Refold it with `start_arm.sh
fold` after a run. Ctrl+C during this move stops it, and the arm holds where it
is. A bridge started by hand does not move the arm unless you set
`ARM_START_STOW=1`.

On start-up the bridge powers and enables the arm only if it needs to: enable-all
on an enabled arm drives it back to its last target. It holds the gripper at the
angle it finds. For each trajectory it:

1. Checks it before anything moves: all six joints, within the joint limits,
   at most 30 °/s, and starting within 5° of where the arm is.
2. Resamples it at 50 Hz with MoveIt's own timing, so the arm never stops
   between waypoints.
3. Plays it in mode 0. With the arm on the Go2 the whole trajectory goes to the
   robot in one `traj_points` message (streamed over Wi-Fi, the arm shakes);
   wired, it streams from the laptop.
4. Finishes with `smooth_move`, whose outer PID removes gravity sag, and reports
   success only within 2° of the goal.

It stops at once and reports the failure to MoveIt on a cancel, when the arm
falls 20° behind where the path was 1 s ago (something is blocking it; the 1 s
covers Wi-Fi and firmware delay), or when telemetry stops
for 1 s. Shutting down leaves the arm **enabled and holding**; fold it with
`start_arm.sh fold`.

`python3 -m pytest tests/test_moveit_bridge.py` tests all of this on a simulated
arm. With `--serve` the simulated arm can stand in for the real one behind
`demo.launch.py hardware:=real`.

### Joint signs

`--real` refuses to start without `ARM_JOINT_SIGNS`, six values of `1` or `-1`
for `Joint1`–`Joint6` (firmware angle = sign × MoveIt angle). The D1-550 URDF
MoveIt uses and Unitree's own `d1_description` (dashboard, IK scan) agree on
joints 2, 3, 4 and 5 and **disagree on Joint1 and Joint6**; nothing here has
measured which matches the firmware. A wrong sign sends the arm to the other side
of the one MoveIt checked for collisions.

Test it once, with the arm clear of the dog and a hand on the power. Joint
numbers are `0`–`5` in the command:

```bash
A=./scripts/runtime/start_arm.sh
$A state                       # enable_status 1: skip the next line, never re-enable
$A enable sleep 2
$A joint 0 10 sleep 3 feedback # base
$A joint 5 30 sleep 3 feedback # wrist roll
$A joint 5 0 sleep 3 joint 0 0 # back
```

| Command | Seen from | MoveIt's direction | Sign |
|---|---|---|---|
| `joint 0 10` | above | clockwise | `1` if it turned clockwise, else `-1` |
| `joint 5 30` | behind the wrist, looking along the arm toward the gripper | clockwise | `1` if it turned clockwise, else `-1` |

Set the other four to `1`, for example `ARM_JOINT_SIGNS="-1 1 1 1 1 -1"` if both
are reversed. Check the mount at the same time: at `home` (all zeros) the forearm
must point over the dog's **tail**, as in `d1_go2.urdf.xacro` (yaw 180°). The
dashboard, `twin/d1_tf_urdf.py` and `arm_ik_scanner.py` still assume it points
forward.

## Troubleshooting

| Symptom | Meaning | Fix |
|---|---|---|
| `<iface> is missing 192.168.123.99` | cable unplugged, or the profile is down | plug in the cable; `sudo nmcli connection up go2-arm` |
| `192.168.123.100 does not reply to ping` | cable / adapter / still booting | check link and 24V power; wait a minute after power-on; power-cycle the arm |
| `route to ... does NOT go via <iface>` | another interface owns the subnet | remove the overlapping address |
| `DDS discovery failed` | no DDS match within `--match-timeout` | arm powered and booted? no `--peer`; no ROS sourced; `pip3 show cyclonedds` |
| commands acked but nothing moves | motors not enabled | `enable` first (`--enable-mode 80000`) |
| `joint N limit ±L°` | target outside the limit | use a smaller angle |
| `free(): invalid pointer` crash | ROS's CycloneDDS mixed with the pip one | run without sourcing ROS |
| `wifi_nav.sh --arm` skipped the arm | see `go2-nav2-wifi/ws/log/arm.log` | run `start_arm.sh --check` |
| `--arm-wifi`: "not built on the robot yet" | the one-time build is missing | `ssh unitree@<robot-ip> 'bash ~/go2-agri-arm/arm_bridge_wifi.sh --build'` |
| `--robot`: "bridge not reachable" | bridge not running, or another laptop IP | on the robot: `tail /tmp/arm_bridge_wifi.log`; it accepts only `GO2_HOST_IP` |
| Bridge exits at once, `exit 139` / segfault | Foxy's stock CycloneDDS instead of Unitree's | `robot-source-unitree-ros.sh` must sit next to the bridge in `~/go2-agri-arm` (`wifi_nav.sh --arm-wifi` copies it) |
| `arm_moveit.sh`: `moveit_ros_move_group missing` (or another package) | MoveIt / ros2_control not installed | `sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers` |
| `arm_moveit.sh`: `D1 description not found` | the D1-550 description repo is elsewhere | `D1_ARM_DIR=<path> ./scripts/runtime/arm_moveit.sh` |
| MoveIt: Plan fails / viewpoint marked red | pose out of reach, or every IK solution collides | move the target closer to the arm, loosen the orientation, or raise `planning_time` |
| MoveIt topics invisible from another terminal | different DDS domain | `export ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1` first |
| `arm_moveit.sh --real`: `needs ARM_JOINT_SIGNS` | the joint signs are not set | run the [joint sign test](#joint-signs) |
| `arm_moveit.sh --real`: `moveit_d1_bridge stopped` | the bridge could not reach the arm; nothing moved | check as for `DDS discovery failed`; on the Go2, `--robot <go2-ip>` |
| `arm_moveit.sh --real`: first plan fails with error `-2` | the arm is in a pose MoveIt counts as a collision, and not close enough to the fold to be moved | move it to stow: `start_arm.sh joints 0 -80 45 0 80 0 0` |
| Bridge: `stow is too far from the fold` | `ARM_JOINT_SIGNS` is wrong for joint 2, 3 or 5 | redo the [joint sign test](#joint-signs) |
| Bridge: `the trajectory starts N deg from where the arm is` | the arm moved outside MoveIt, or a wrong sign | restart `--real`; check `ARM_JOINT_SIGNS` |
| Bridge: `N deg behind the path` | the arm is blocked or the signs are wrong | clear the arm; check `ARM_JOINT_SIGNS` |
| `--robot`: "no arm feedback" | arm not plugged into the Go2, not powered, or still booting (~1 min) | from the robot: `ping 192.168.123.100` |

## Arm roadmap

| Step | Purpose | Status |
|---|---|---|
| 1. Direct control | Prove the arm responds to JSON commands | Written: chain control, fold, lift/sweep, offset calibration |
| 2. ROS 2 bridge node | Subscribe to `/arm/command` instead of the raw script | Pending |
| 2b. Arm on the Go2 over Wi-Fi | Robot-side TCP bridge, `--robot` / `--arm-wifi` | Verified on the robot (2026-10-27): state, `joint`, `home`, `fold` over Wi-Fi; `power 0` open |
| 3. RViz panel | Slider per joint, alongside the explorer panel | Pending |
| 3b. MoveIt 2 planning | Cartesian goals, collision-free plans around the Go2 and the plant (`arm_moveit.sh`) | Implemented; **real-arm bridge built (`moveit_d1_bridge.py`, pending physical test)** |
| 4. Camera mount | 3D-printed attachment for the hyperspectral camera | Hardware |
| 5. Scan routine | Move the camera through a plant-scanning path | Teach & Playback on the real arm; `inspect_plant.py` 3-point scan in MoveIt |
| 6. AI autonomy | Model commands both the dog and the arm | Long-term |

See [docs/roadmap.md](../roadmap.md) for the overall project roadmap.

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
