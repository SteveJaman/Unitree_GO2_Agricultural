# Step over by crawling: low-level leg control on the real Go2

[← Docs index](../README.md) · [Learned locomotion plan](learned_locomotion_plan.md) · [Traversability handoff](traversability_handoff.md)

Status (2026-11-16): `--stand` and `--lift FR` **run on the real dog**. The full crawl
(`--step-over`) is not written yet.

## Why a crawl

The Go2's sport mode can't raise its feet on this firmware: `FootRaiseHeight` returns
3203, and the remote's L1 + ↑ had no effect when measured. So
[`step_over_crawl.py`](../../scripts/robot/step_over_crawl.py) drives the 12 joints
itself in **low-level mode**.

In low-level mode Unitree's balance controller is **off**. A crawl stays up without it:

- only one foot is in the air at a time;
- before each foot lifts, the body moves over the triangle of the other three, at
  least 3.8 cm inside it in the default plan.

It is slow (about 100 s for one object) and it does not react to slips. It is a
controlled step-over demo, not a walking controller. Walking with high steps is the
[learned controller plan](learned_locomotion_plan.md).

## Stages

Do each stage only after the one before it worked.

| Stage | What moves | Needs |
|---|---|---|
| `--plan` | nothing (laptop) | — |
| `--stand` | sport mode off, the dog stands on joint control, lies down, sport mode back | dog on the floor, 1 m clear, someone on the remote, a hand or harness the first time |
| `--lift LEG` | the body over three feet, one foot up 10 cm, hold, down | a hand by that shoulder, or a harness |
| `--step-over` (to do) | the crawl over foam (5, 8 then 10 cm), then a real object | harness, then a spotter |

## `--plan`

```bash
python3 scripts/robot/step_over_crawl.py --plan
python3 scripts/robot/step_over_crawl.py --plan --object-height 0.10 --object-depth 0.08 --csv /tmp/plan.csv
```

It prints every keyframe:

- the body position;
- the stability margin (how far inside the standing triangle the body is while a foot
  is up);
- all 12 joint angles.

It flags anything outside the joint limits (URDF, with 0.05 rad kept clear), out of
reach, under the 2.5 cm stability margin, or an object too deep to cross in one step.

Default object: 8 cm high and 10 cm deep, its near edge 12 cm ahead of the front feet.
The plan:

- 103 keyframes, about 107 s.
- **Each leg crosses in one 22 cm step with a 13 cm lift** (the object height plus
  5 cm). Other steps lift 6 cm.
- Worst stability margin 3.8 cm; 0 problems.
- Heights 3–15 cm plan cleanly. Objects deeper than about 13 cm don't: the crossing
  step would need more than 26 cm (`--max-stride`), and the plan says so.

Checked by `tests/test_step_over_crawl.py`, 8 tests:

- the inverse and forward kinematics agree;
- the leg model matches the sim's `leg_gait.py`;
- the stand pose is close to Unitree's;
- the stability maths is right;
- the full plan crosses with all four legs and no foot lands on the object.

## `--stand` (the first hardware run)

It runs **on the dog**. The control loop runs 500 times a second and must not cross
Wi-Fi.

**One-time setup on the dog** (not done yet; the dog was offline). Copy the SDK and
the script to it:

```bash
scp -r unitree_sdk2_python scripts/robot/step_over_crawl.py unitree@<dog-ip>:~/
```

Then, on the dog:

```bash
ssh unitree@<dog-ip>
cd ~/unitree_sdk2_python && pip3 install --user -e .
python3 -c "import unitree_sdk2py; print('ok')"
ip -4 addr          # the dog's internal network (192.168.123.x) is usually eth0
```

`unitree_sdk2py` needs the Python `cyclonedds` package. If `pip` can't build it, see
the SDK's README for `CYCLONEDDS_HOME`.

**Before running:**

- No navigation session running: `wifi_3d_nav.sh` / `wifi_nav.sh` stopped. Both
  command the same motors.
- The dog standing on the floor, 1 m clear around it.
- Someone on the remote. The first time, a hand on the dog or a harness.

**Run:**

```bash
ssh unitree@<dog-ip>
python3 ~/step_over_crawl.py --stand --iface eth0
```

It asks you to type `yes`, then:

1. sport mode lies the dog down;
2. sport mode is released, and the joints are under the script's control
   (**Unitree's balance is off from here**);
3. it moves to the lying pose, then stands at 0.30 m over 2.5 s per move (Unitree's
   gains: kp 60, kd 5);
4. it holds for 5 s;
5. it lies down again, with the joints going soft;
6. it hands back to sport mode. `--restand` also stands the dog up in sport mode.

**Safety stops.** Ctrl+C, `/lowstate` older than 0.1 s, or a tilt over 25° makes the
joints go soft. The dog settles onto its belly and sport mode is handed back.

**What to look for:**

- the stand is smooth;
- the dog is level while it holds (the script prints the tilt);
- no buzzing or shaking in the joints;
- sport mode comes back. After the run, the remote should stand and walk the dog
  again.

## Results on the real dog (2026-11-16)

| Run | Settings | Tilt with the foot up | Joint error, standing legs | Outcome |
|---|---|---|---|---|
| `--stand` | kp 60 | 3.6° | — | smooth stand, held 5 s, sport mode back |
| `--lift FR` v1 | 5 cm shift, kp 60 | 10.6°, then 13.3° after | — | leaned toward the lifted leg, near the 15° stop |
| `--lift FR` v2 | 7.5 cm shift, kp 80, logged | roll +2.7°, pitch −9.8° | 0.14 rad RMS (rear thighs and calves 0.2 rad short) | still leaning |
| `--lift FR` v3 | + integral correction `--ki 1.0` | roll +2.3°, pitch +0.3° | **0.009 rad RMS** | level within 1° after; corrections ≤ 0.24 rad |

**All four legs with the v3 settings** (10 cm lift, 7.5 cm shift, kp 80, `--ki 1.0`,
tilt stop 15°):

| Leg | Worst roll, pitch (shift + lift) | Held: roll, pitch | Standing-leg error (RMS) | Largest correction |
|---|---|---|---|---|
| FR | 5.8°, 2.8° | +2.3°, +0.3° | 0.010 rad | 0.24 rad |
| FL | 2.0°, 1.6° | −0.5°, +0.2° | 0.008 rad | 0.21 rad |
| RR | 1.4°, 1.9° | +0.2°, −0.7° | 0.003 rad | 0.18 rad |
| RL | 3.0°, 2.3° | −1.0°, −0.7° | 0.004 rad | 0.18 rad |

No safety stop tripped, and every run handed sport mode back. The lifted foot tracked
its target within 0.002 rad.

**What fixed it.** The v2 log showed the joints sagging under the dog's weight. Pure
position control (kp) only pushes back once a joint is already off its target, and
the rear thighs and calves sat about 0.2 rad short even standing on four legs. A slow
integral correction per joint (`--ki`, capped at 0.35 rad) nudges each command until
the joint reaches its target. It fades out over the lie-down, so there is no jump.

Logs: `~/go2-agri-lowlevel/lift_FR_v2.csv` and `lift_FR_v3.csv` on the dog. Each row
has roll and pitch, plus every joint's reference, its lag and its correction.

## Setup on the dog (done 2026-11-16)

- The dog has no internet. The CycloneDDS Python binding (0.10.2, source from PyPI)
  and `typing_extensions` were copied from the laptop and built on the dog. The dog's
  clock had reset to 1970, so the build needs `SOURCE_DATE_EPOCH`.
- **The dog has two CycloneDDS 0.10.2 builds**, `/usr/local` and
  `~/cyclonedds_ws/install`, and their `features.h` differ. A binding compiled against
  one and run against the other crashed on every write. It is built against
  `~/cyclonedds_ws/install`:

  ```bash
  SOURCE_DATE_EPOCH=1763000000 CYCLONEDDS_HOME=$HOME/cyclonedds_ws/install/cyclonedds \
      pip3 install --user --no-deps --no-index --no-build-isolation cyclonedds-0.10.2.tar.gz
  ```

- The SDK package is in `~/go2-agri-lowlevel/sdk`. Copy it there; don't pip-install
  it, or it pulls in opencv.
- The launcher [`crawl_on_dog.sh`](../../scripts/robot/crawl_on_dog.sh), installed as
  `~/go2-agri-lowlevel/crawl.sh`, sets `LD_LIBRARY_PATH` and `PYTHONPATH`.

```bash
~/go2-agri-lowlevel/crawl.sh --check                       # read only
~/go2-agri-lowlevel/crawl.sh --stand
~/go2-agri-lowlevel/crawl.sh --lift FR --lift-margin 0.075 --kp-lift 80 --ki 1.0 --max-tilt 15 --log ~/lift.csv
```

## Stepping over objects in a navigation session (`--step-over-crawl`)

Built 2026-11-16, checked off the dog only (unit tests and a ROS run in dry mode). This
is the real-dog version of the poster's last step, where the gait supervisor picks the
foot lift. Sport mode can't lift the feet, so the crawl does it.

```bash
./scripts/runtime/wifi_3d_nav.sh --semantic --manual --step-over-crawl          # dry: log only
./scripts/runtime/wifi_3d_nav.sh --semantic --manual --step-over-crawl align    # pause, line up, resume
./scripts/runtime/wifi_3d_nav.sh --semantic --manual --step-over-crawl crawl    # real step-over
```

1. **Pick.** [`step_over_coordinator.py`](../../scripts/nodes/step_over_coordinator.py)
   runs with the traversability nodes. It watches `/semantic/labels_json` (each
   object's map position, LiDAR height, footprint, label) while Nav2 drives forward.
   It picks an object whose near edge is 0.35–1.30 m ahead and within 0.20 m of the
   heading, and that is:
   - 3–10 cm high: lower is walked over normally, taller goes around;
   - at most 10 cm deep, the crawl's crossing step;
   - not AVOID and not protected.

   The log says why it skipped each other object.
2. **Pause and line up.** It cancels `/follow_path` and `/compute_path_to_pose`, then
   turns to face the object and drives slowly until the front feet are 12 cm before its
   near edge.
3. **Hand off.** It writes `ws/log/3d/step_over/request_<id>.json`. The laptop runs
   [`step_over_executor.py`](../../scripts/tools/step_over_executor.py), which runs
   `crawl.sh --step-over` with that height, depth and gap on the dog over SSH. The crawl
   ends with `--resume-session`: stand up, ClassicWalk, remote ignored.
4. **Resume.** It republishes the goal.
   - If the crawl stopped (lean, tilt, temperature), Nav2 is **not** resumed. The log
     says why. A new goal from RViz starts again.

Logs:

- `ws/log/3d/traversability/crawl.log`: the coordinator's decisions;
- `ws/log/3d/step_over_executor.log`: each crawl's result;
- `~/go2-agri-lowlevel/nav_<id>.csv` on the dog: the joint log.

**Before `crawl` mode:** the crawl must pass its own tests on the dog. That means one
full cycle with no object, then foam, several times in a row with no stop. Until then
use `dry` (proves detection and selection) and `align` (proves the hand-off) in normal
sessions.

## Feed-forward and the one-leg step-over (2026-11-16, evening)

| Run | Settings | Result |
|---|---|---|
| `--stand`, 0.28 m | feed-forward 0.8, `--ki 1.0` | joint error **0.011 rad** RMS (0.139 without feed-forward), roll 0.0°, pitch −0.1° |
| `--leg-over FR`, no object | feed-forward 0.8 | forward step and the hold beyond were level (≤ 1.4°); lifting the foot again leaned to 6°: the abort lowered the dog safely. The rear-left calf got 16 N m and over-extended: the centre of mass is not where the model puts it (the arm). |
| `--leg-over FR`, no object | **feed-forward 0.4** | complete; worst lean 4.9° |
| **`--leg-over FR` over a 6 cm × 5 cm object** | feed-forward 0.4 | **complete**: the foot rose 11 cm, crossed the object and landed beyond it, held 5 s, came back; worst lean 4.6°, back within 0.5° |

The rear hip motors run hot: 61–72 °C after a few runs, and they keep heating while the
dog **stands** in sport mode. Lie it down between runs; it cools about 1 °C a minute.

## Video run sheet

Do these on the dog in this order. The lines run on the dog (`ssh unitree@<dog-ip>`);
`~/go2-agri-lowlevel/` has the script and the SDK.

1. **Before filming:** the dog lying down, motors cool. `crawl.sh --check` prints the joint
   states, and `crawl.sh --stand` refuses to start above 55 °C (`--start-temp`). Someone by
   the dog, remote ready.
2. **Per-leg control:** stand, shift onto three feet, front-right foot up 10 cm and down
   (about 30 s):

   ```bash
   ~/go2-agri-lowlevel/crawl.sh --lift FR --lift-margin 0.075 --kp-lift 80 --ki 1.0 --ff 0.4 --yes
   ```

3. **The step-over shot.** Put an object about 6 cm high and 5 cm deep on the floor, its
   near edge about 6 cm in front of the front-right foot, straight ahead of it. Then:

   ```bash
   ~/go2-agri-lowlevel/crawl.sh --leg-over FR --stand-height 0.28 --lift-margin 0.075 \
       --object-distance 0.06 --object-depth 0.05 --object-height 0.06 --landing-margin 0.03 \
       --ff 0.4 --ki 1.0 --kp-lift 80 --lean-abort 6 --max-tilt 12 --yes
   ```

   Film from the side, at foot height, so the foot's clearance over the object shows.
4. **Perception beside it:** a `--semantic` session with `--step-over-crawl dry`. RViz or
   the dashboard shows the labelled objects, and the terminal shows "would step over …".
5. Between takes, lie the dog down and wait for `--check` to show the rear hips under 55 °C.

**Not ready to film:** the full crawl (`--step-over`). It fell twice before the
feed-forward existed, and has not run since.

## The poster's claims on the real dog

| Poster claim | Real-dog evidence | How to film it |
|---|---|---|
| LiDAR measures height; YOLO + VLM label | `/semantic/labels_json` while standing (verified 2026-11-11) | RViz or the dashboard beside the dog |
| Protected objects: the VLM can only make it more careful | the plant is AVOID, cost 254 (2026-11-16) | the dog goes around the plant |
| Gait supervisor picks the foot lift | `--step-over-crawl dry` logs "would step over" | terminal with the log beside the dog |
| Per-leg control (foot lift) | `--lift` on each leg: 10 cm, level within 2.3° | `--lift FR` (≈30 s) |
| Stepping over an object | **`--leg-over FR` over a 6 cm object: done 2026-11-16**, lean at most 4.6°; the full crawl (`--step-over`) not yet passed | `--leg-over FR` (≈45 s) |

## Risks

- **The dog can fall.** Unitree's balance is off for the whole low-level stage.
- **Wrong network interface.** The script stops with "no /lowstate" before releasing
  sport mode.
- **The hand-back to sport mode fails.** The dog stays lying with soft joints. Use
  the remote, or reboot the dog.
- The mode name comes from `CheckMode` before release. If it was empty, the script
  asks for `normal`.
