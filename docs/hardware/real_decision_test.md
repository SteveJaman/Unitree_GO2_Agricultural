# Real-dog test: the traversability decisions

[← Docs index](../README.md) · [Traversability handoff](traversability_handoff.md) · [Sim record](../simulation/traversability_sim.md) · [Learned locomotion plan](learned_locomotion_plan.md)

Status: **planned** (2026-11-16). Nothing below has been run yet.

## What it tests, and what it does not

The sim's step-over was a kinematic gait: Gazebo moved the body, and `leg_gait.py`
animated the legs. The real Go2 can't raise its feet on command (see
[learned_locomotion_plan.md](learned_locomotion_plan.md)). Everything **before** the
legs runs on the real dog today, and this test measures it there:

| Stage | Question | Sim result to compare |
|---|---|---|
| Detection | Does YOLO find the object, with the right class? | — |
| Height | Is the LiDAR-measured height right? | median error 1.2 cm, 164 of 181 boxes (91 %) in the right band |
| Label | Is the published label right (height band, VLM, protected classes)? | 0 unsafe of 149, accuracy 0.48 |
| Gait | Would the supervisor command the right gait? | backstop: 0 of 5893 samples NORMAL while the costmap saw something |
| Costmap | What cost does the object's footprint get? | — |

The legs are not tested. Unitree's controller keeps walking normally and the gait
choice is logged only. These are real-hardware numbers for the decision logic, not
for the step-over.

## Bands on the real dog

From the measured step clearance (ClassicWalk 4.1–6.5 cm, `TRAV_STEP_OVER` = 0.03 m)
and the supervisor's rules (foot clearance = step-over + 2 cm margin):

| True height | Expected band and label | Expected gait |
|---|---|---|
| ≤ 3 cm | step over, STEP_OVER | HIGH_STEP (it fits under the lift) |
| 3–22 cm | caution, CAUTION | CAREFUL |
| ≥ 22 cm | blocked, AVOID | STOP (≥ 0.25 m, or AVOID on a measured object) |
| Protected class, any height | AVOID if measured, else CAUTION | STOP / CAREFUL |

## Objects

Measure each object's height with a ruler (its highest point above the floor) and
write it down. Suggested set, spanning every band and both cutoffs:

| Object | Approx. height | Band |
|---|---|---|
| Flat cable or mat | 1 cm | step over |
| Book lying flat | 3 cm | the step-over cutoff |
| Dry-erase marker box | 5–6 cm | caution |
| Shoe | 10–12 cm | caution |
| Mug | 10 cm | caution (YOLO: "cup") |
| Cardboard box | 25 cm | blocked |
| Fake plant (small) | 8 cm | protected → AVOID |
| Fake plant (tall) | 30+ cm | protected, blocked |

## Procedure

**Before the session:**

1. Wi-Fi: check the dog's link (`ssh unitree@<dog> 'ping -c 50 -i 0.2 <router>'`).
   The worst delay should stay under about 1 s.
2. Camera calibration (`./scripts/tools/calibrate_camera.sh`), strongly recommended:
   without it the boxes use the 110° lens estimate and can sit off their objects,
   which shows up as height and position error here.
3. Clear floor, about 3 m long, with someone on the remote for L2 + B.

**Session:**

```bash
./scripts/runtime/wifi_3d_nav.sh --semantic --manual
```

Drive with the dashboard's **Drive** pad (`http://localhost:3000`). The remote is
switched off while a session runs. Nav2 goals are not used: the operator controls
speed and stopping.

**One approach per object, head on.** The gait supervisor looks at a 1.5 m × 0.3 m
corridor straight ahead, so the object must be in line with the dog.

1. Place the object 2.5 m straight ahead of the dog.
2. Note the object's distance and its offset left/right, measured from the dog's
   front.
3. Start the recorders (below), then drive slowly (about 0.2 m/s) toward it.
   **Stop about 0.6 m short**, wait 10 s for the VLM, then back off.
4. Stop the recorders. Note the object, its true height and the run number.

Do every object 3 times. Then change the order and the floor spot, and do them all
again. That gives 6 approaches per object.

**Recorders** (to build: one script that starts these in the session's
`go2-lidar3d` container with its DDS settings, and copies the output to
`results/real_logic/<object>_<run>/`):

- `tests/traversability/record_json_topic.py /objects/boxes_json boxes.jsonl`: 3D
  boxes, with measured height and position.
- `tests/traversability/record_semantic.py OUT`: every label, its source
  (geometry, VLM, protected) and the VLM answer.
- `tests/traversability/record_json_topic.py /gait/status gait.jsonl`: the gait,
  its source and the backstop.
- `tests/traversability/save_costmap.py OUT 1`: the local and global costmaps once a
  second.
- The dog's pose (`lidar3d_map → base_link`) at the start, to place the object's
  true position in the map.

## Analysis (to build: `tests/traversability/real_logic_analyze.py`)

For each approach:

- **Truth:** place the object in the map from the starting pose plus its measured
  distance and offset.
- **Detection:** the box nearest the true position (within 0.4 m), plus its class,
  and the distance at which it was first seen.
- **Height:** the box's height against the ruler height, and whether it lands in the
  right band.
- **Label:** the label published most at 0.6–1.5 m, against the expected label.
- **Gait:** the gait while the object is 0.6–1.5 m ahead, against the expected gait.
- **Costmap:** the highest cost on the object's footprint, from the local and global
  costmaps.

Report per object and per band, in the same tables as the sim:

- the detection rate;
- the median and mean height error;
- band accuracy;
- label accuracy;
- **unsafe labels**: a ≥ 22 cm or protected object published as STEP_OVER;
- gait agreement;
- objects whose global cost is below lethal although they should be blocked. This
  is the case that walked the dog into the plant;
- **memory:** whether the object's box, label and costmap mark are **still there when
  the dog has stopped 0.6 m short**, where the camera can no longer see a low object.
  The fusion now keeps tracks within 1.5 m of the dog (`track_keep_near_m`) instead of
  expiring them 3 s after the camera loses them.

## What would count as passing

| | Target |
|---|---|
| Height error (median) | ≤ 2 cm |
| Band accuracy | ≥ 85 % |
| Unsafe labels | 0 |
| Protected plant | AVOID in ≥ 5 of 6 approaches |
| Gait agreement | ≥ 90 % of samples |

Misses near the 3 cm and 22 cm cutoffs are expected: the sim's 17 wrong-band boxes
were all objects just above the 12 cm cutoff. Report them as boundary cases.

## Time

About 1 hour to build the recorder and analysis scripts, and about 2 hours with the
dog: 8 objects × 6 approaches × about 2 minutes, plus setup.
