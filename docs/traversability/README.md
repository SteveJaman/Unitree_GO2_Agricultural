# Traversability-aware navigation: what was built and how to run it

Branch: `ysalimi-VLM-traversability` (pushed to `protected/ysalimi-VLM-traversability`).
Everything here was run in simulation only (Gazebo). No physical Go2 was used.

## What it does

`scripts/nodes/traversability_node.py` decides whether the dog may step over a low
object (cable, mat) or must avoid it (toolbox, phone, vase).

1. The LiDAR cloud is split into floor and objects. The floor height is estimated from
   the points (5 mm histogram bins), objects are clustered on an 8 cm grid and tracked.
2. Height rule: an object at or above `step_limit` (8 cm) is `avoid`. Anything lower is a
   candidate for stepping over.
3. Optional VLM check (Ollama): a camera crop of each low object is sent once, in a
   background thread, and the model answers with a label (`step_over`, `caution`, `avoid`)
   and a `fragile` flag. The VLM can only make a decision stricter, never looser. An object
   waiting for its verdict is held at `caution`.
4. Depth source: `height_source` is `lidar`, `depth` or `auto`. Both paths have been run in the sim (the sim has an optional depth camera, see below).
   run in the sim; the sim has no depth sensor.

Outputs: `/traversability/obstacles`, `/traversability/markers`, `/traversability/status`,
`/gait_mode`. Parameters are in `config/traversability.yaml`.

## Files

| Path | Purpose |
|---|---|
| `scripts/nodes/traversability_core.py` | floor estimate, clustering, tracker, decision rule, VLM prompt and parser |
| `scripts/nodes/traversability_node.py` | the ROS 2 node (LiDAR + optional depth + VLM) |
| `config/traversability.yaml` | default parameters |
| `scripts/tools/test_traversability_core.py` | offline test on a synthetic scene, no ROS needed |
| `gazebo/worlds/build_lab_world.py` | generates `traversability_lab.sdf`: a 7 x 4 m room with five test objects |
| `scripts/experiments/run_goals.sh` | sends the three test goals and waits for the dog at each one |
| `scripts/experiments/analyze_run.py` | matches a run log to the true objects and prints a compact summary |
| `scripts/experiments/summarize_run.py` | per-track summary of a run log |
| `scripts/experiments/replay_rules.py` | re-decides a run log with and without the operator lists (no sim needed) |
| `scripts/tools/test_operator_rules.py` | offline test of the operator lists |
| `gazebo/worlds/traversability_lab2.sdf` | the lab with four extra low objects (`build_lab_world.py --extra`) |
| `docs/traversability/results/` | saved run logs and analysis output |

Node options added for the experiments:

- `log_path` writes one JSON line per tick (pose, every track, decision, VLM verdict).
- `crop_dir` saves each crop that is sent to the VLM.
- `sim_camera` / `cam_hfov` / `cam_mount`: the sim publishes no `camera_info`, so the node
  builds the intrinsics from the camera's field of view (640 x 360, 1.2 rad, mounted at
  0.327, 0, 0.043 m on `base_link`). This is used automatically when no `camera_info` arrives.
- `min_range` (0.6 m) drops points close to the robot, which are its own body.
- `cv2` is optional: without it the node runs but cannot cut crops, so the VLM never runs.

## Test objects

`build_lab_world.py` places these objects (world coordinates, metres):

| Object | Position | Size (x, y, height) | Collision |
|---|---|---|---|
| cable | (-2.0, 0.0) | 0.03 x 0.9 x 0.03 | no |
| mat | (-1.0, 0.0) | 0.8 x 1.0 x 0.02 | no |
| toolbox | (0.2, 0.9) | 0.3 x 0.5 x 0.12 | yes |
| phone | (0.2, -0.9) | 0.15 x 0.08 x 0.08 | yes |
| vase | (1.2, 0.0) | 0.12 x 0.12 x 0.30 | yes |

The cable and mat have no collision shape, so the dog can drive over them. They are plain
coloured boxes, so a VLM cannot recognise a "phone" in this sim.

## Running it (Ubuntu VM, ROS 2 Humble, Docker)

Offline test first:

    python3 scripts/tools/test_traversability_core.py

Generate the world and launch the sim with the camera on. Leave this terminal running.

    python3 gazebo/worlds/build_lab_world.py
    export SIM_WORLD=traversability_lab SIM_SPAWN="-2.8 0 0" SIM_CAMERA=1
    ./scripts/runtime/wifi_nav.sh --sim 2>&1 | tee ~/launch.log

Use `SIM_CAMERA=0` for a LiDAR-only run (lighter, but no VLM crops).

Prepare the container (it is new after every relaunch):

    docker exec -u root go2-lidar3d pip install opencv-python-headless==4.7.0.72
    docker cp scripts/nodes/traversability_core.py go2-lidar3d:/tmp/
    docker cp scripts/nodes/traversability_node.py go2-lidar3d:/tmp/
    docker exec go2-lidar3d bash -lc 'source /opt/ros/humble/setup.bash; ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"'

The last line pauses the auto-explorer, which otherwise sends its own goals. Newer OpenCV
builds fail on the container's older numpy, hence the pinned version.

### Run A: height rule only (no VLM)

    docker exec -it go2-lidar3d bash -lc 'source /opt/ros/humble/setup.bash; cd /tmp && python3 traversability_node.py --ros-args -p use_vlm:=false -p cloud_topic:=/lidar3d/obstacle_points -p h_tall:=0.2 -p min_seen:=3 -p log_path:=/tmp/run_A_novlm.jsonl'

### Run B: height rule + VLM

The VLM runs in Ollama. On a VM with a slow connection, serve the model from another
machine on the same network. On that machine:

    OLLAMA_HOST=0.0.0.0 ollama serve        # keep it open; use `caffeinate -d` on a Mac
    ollama pull qwen2.5vl                   # qwen2.5vl was far better than llava:7b in the team's tests

Check from the VM: `curl -m 10 http://<MAC_IP>:11434/api/tags`. Then:

    docker exec -it go2-lidar3d bash -lc 'source /opt/ros/humble/setup.bash; cd /tmp && python3 traversability_node.py --ros-args -p use_vlm:=true -p vlm_url:=http://<MAC_IP>:11434/api/generate -p vlm_model:=qwen2.5vl:latest -p vlm_timeout:=180.0 -p cloud_topic:=/lidar3d/obstacle_points -p h_tall:=0.2 -p min_seen:=3 -p log_path:=/tmp/run_B_vlm.jsonl -p crop_dir:=/tmp/crops_B'

### Driving the test route

In a second terminal, once the node prints its first `[lidar] floor` line:

    ./scripts/experiments/run_goals.sh

Goals are sent in the `lidar3d_map` frame: (-1.6, -0.7), (-0.9, -0.85), then back to (-2.8, 0).
The script prints the dog's distance from each goal every 5 seconds and moves on when it is
within 0.3 m. The first goal puts the phone in the camera's view. After `all goals done`,
wait about 2 minutes for the VLM to finish its queue, then stop the node with Ctrl+C.

### Saving and analysing a run

    docker cp go2-lidar3d:/tmp/run_B_vlm.jsonl ~/run_B_vlm.jsonl
    docker cp go2-lidar3d:/tmp/crops_B ~/crops_B
    python3 scripts/experiments/analyze_run.py ~/run_B_vlm.jsonl B > ~/an_B.txt
    head -16 ~/an_B.txt

`analyze_run.py` assigns a track to an object if it lies within 0.2 m of its footprint. It
reports, per object: how long it was judged traversable, caution or avoid, its height range,
its VLM labels, and the dog's path.

## Depth camera in the sim (case 1)

The sim has an optional depth camera (`depth_camera` sensor, 320 x 180, 5 Hz, same mount and
field of view as the RGB camera). It is off unless `SIM_DEPTH=1`.

    export SIM_WORLD=traversability_lab SIM_SPAWN="-2.8 0 0" SIM_CAMERA=1 SIM_DEPTH=1
    ./scripts/runtime/wifi_nav.sh --sim 2>&1 | tee ~/launch_depth.log

Check that frames arrive before starting the node. Use `echo`, not `ros2 topic hz`, which receives
nothing from the image topics here:

    docker exec go2-lidar3d bash -lc 'source /opt/ros/humble/setup.bash; ros2 topic list >/dev/null; sleep 4; timeout 40 ros2 topic echo --once /camera/depth/image_raw --field header.stamp'

Then the node, depth only, no VLM (same container preparation as above):

    docker exec -it go2-lidar3d bash -lc 'source /opt/ros/humble/setup.bash; cd /tmp && python3 traversability_node.py --ros-args -p use_vlm:=false -p height_source:=depth -p sim_depth:=true -p depth_stride:=1 -p cloud_topic:=/lidar3d/obstacle_points -p h_tall:=0.2 -p min_seen:=3 -p log_path:=/tmp/run_D_depth.jsonl'

- `sim_depth:=true`: the sim publishes no `camera_info` and the camera frame has x forward, so the
  node builds the intrinsics from `depth_hfov` (1.2 rad) and the optical-frame rotation from the base pose.
- If no depth points arrive, the node warns `no depth points` every 10 s.
- The depth camera sees about 34 degrees either side and 21 degrees up and down, mounted about
  0.3 m high, so it cannot see the floor closer than roughly 0.8 m ahead. Small objects are only
  visible while the dog faces them from further away.
- `run_goals.sh` takes goals as arguments and sends a goal again after 30 s without progress.

## Extra objects (traversability_lab2)

`python3 gazebo/worlds/build_lab_world.py --extra` also writes `traversability_lab2.sdf`
(`SIM_WORLD=traversability_lab2`). It adds four objects, all below the 8 cm step limit:

| Object | Position | Size (x, y, height) | Should the robot cross it? | Collision |
|---|---|---|---|---|
| laptop | (0.8, -0.5) | 0.33 x 0.23 x 0.03 | no, valuable | yes |
| glasses case | (0.6, 0.4) | 0.16 x 0.06 x 0.05 | no, fragile | yes |
| power strip | (1.6, -1.1) | 0.30 x 0.06 x 0.04 | yes | no |
| book | (-0.2, -1.3) | 0.22 x 0.15 x 0.03 | yes | no |

To see them, the route needs a third goal:
`./scripts/experiments/run_goals.sh "-1.6 -0.7" "-0.9 -0.85" "-0.3 0.0" "-2.8 0.0"`.
Add more objects by adding a line to `EXTRA` in `build_lab_world.py`, and the same footprint to
`analyze_run.py` and `replay_rules.py`.

## Operator lists

A low object can be fine to cross by height and still be something its owner does not want touched.
Appearance does not tell the robot why an object matters, so the owner states it in the config.

    always_avoid: 'phone,laptop,glasses'   # comma-separated words
    always_safe: 'cable,mat,power strip,book'
    unknown_policy: vlm                    # 'vlm': the VLM decides unlisted objects; 'strict': avoid anything not in always_safe

or `-p always_avoid:='phone,laptop' -p always_safe:='cable,mat' -p unknown_policy:=vlm`.

How a decision is made, in order:
1. Height at or above `step_limit`: avoid. The lists never override this.
2. No VLM verdict yet: caution. With the VLM off, a low object is traversable and the lists have no effect.
3. A word from `always_avoid` is in the VLM's object name: avoid. Then `always_safe`: traversable, even if the VLM said fragile.
4. `unknown_policy: strict`: avoid anything else. Otherwise the VLM's label and fragile flag decide.

The lists match on the name the VLM gives. In the sim every object is a plain box, so the real VLM
cannot name most of them and the lists rarely fire with it. Each log line records the rule
that fired (`rule`). Offline test: `python3 scripts/tools/test_operator_rules.py`.

Replay a saved run with and without the lists, with the true object name as the name
(a recogniser that names things correctly), no sim needed:

    python3 scripts/experiments/replay_rules.py docs/traversability/results/run_D5_depth.jsonl D5 --lab2

## Results

Kept runs. Shares are the fraction of ticks an object was judged traversable / caution / avoid (an
object counts at the strictest decision among its tracks). No run was repeated under identical
conditions, and the depth runs below show how much one run can differ from another.

| Run | Sensor | VLM | World | Notes |
|---|---|---|---|---|
| A | LiDAR | no | lab | |
| B3 | LiDAR | yes (qwen2.5vl on a Mac) | lab | |
| D | depth | no | lab | a goal was dropped, 4.5 min stall mid-route |
| D4 | depth | no | lab | clean |
| D5 | depth | no | lab2 | four extra objects |
| E | depth | yes (qwen2.5vl on a Mac) | lab2 | operator lists on; the route finished in 469 s |

Discarded: B and B2 (the model host went to sleep, the dog got stuck) and D3 (no detections at all;
probably the VM was paused or throttled while the host was in use, not confirmed).

Lab world, trav / caution / avoid (%):

| Object (true height) | A | B3 | D | D4 |
|---|---|---|---|---|
| cable (3 cm) | 100/0/0 | 0/100/0 | 100/0/0 | not seen |
| mat (2 cm) | 100/0/0 | 0/100/0 | 100/0/0 | 100/0/0 |
| toolbox (12 cm) | 42/0/58 | 11/54/36 | 24/0/76 | 0/0/100 |
| phone (8 cm) | 97/0/3 | 66/9/25 | 96/0/4 | 0/0/100 |

Heights read: LiDAR cable and mat 2-3 cm, toolbox 6-7 cm, phone 3-6 cm. Depth: toolbox 12 cm
(median, D and D4), phone 8 cm.

Run D5 (depth, lab2):

| Object (true height) | Depth read, median (range) | trav / caution / avoid (%) |
|---|---|---|
| mat (2 cm) | 2 cm | 100/0/0 |
| toolbox (12 cm) | 8 cm (3-12) | 54/0/46 |
| phone (8 cm) | 8 cm (3-8) | 37/0/63 |
| laptop (3 cm) | 3 cm (3-12) | 71/0/29 |
| glasses case (5 cm) | 5 cm (3-10) | 98/0/2 |
| power strip (4 cm) | 4 cm (4-5) | 100/0/0 |
| book (3 cm) | 3 cm (2-8) | 98/0/2 |

Replay with the operator lists and correct object names (`replay_rules.py`), trav / caution / avoid (%):
run A toolbox 42/0/58 becomes 0/0/100 and phone 97/0/3 becomes 0/0/100; run B3 cable and mat go from
0/100/0 (held at caution while unverified) to 100/0/0; run D5 toolbox, phone, laptop and glasses case
all become 0/0/100 while mat and power strip stay 100/0/0 and book 98/0/2.

Run E (depth + VLM, lab2, operator lists live), trav / caution / avoid (%):

| Object | Should be | Recorded | Correct names + lists (replay) |
|---|---|---|---|
| mat | cross | 66/34/0 | 100/0/0 |
| power strip | cross | 4/96/0 | 100/0/0 |
| book | cross | 33/61/7 | 93/0/7 |
| toolbox | avoid | 37/0/63 | 0/0/100 |
| phone | avoid | 19/0/81 | 0/0/100 |
| laptop | avoid | 38/34/28 | 0/0/100 |
| glasses case | avoid | 45/43/12 | 0/0/100 |

- The VLM labelled every object `step_over`, not fragile, so it protected nothing: live, the laptop was
  crossed 38% of the time and the glasses case 45%.
- Live, the lists changed no decision. The avoid list never fired, and the safe list only matched fragments
  the VLM had already called step_over.
- The VLM's names are unreliable: laptop fragments were called "cable" and "green mat", the phone and toolbox
  fragments "green mat", the glasses case "purple cube". A safe list keyed on the model's name can therefore
  clear a dangerous object that is mislabelled with a safe word, and a safe match currently overrides a
  fragile flag. Letting a fragile or avoid verdict win over the safe list is a possible fix.
- Until an object is verified it stays at caution, so the power strip was at caution 96% of the time and the
  book 61%.
- The replay column "logged VLM name, with lists" only counts ticks where a verdict exists, which is biased
  towards fragments reading under 8 cm, so it is not reported.

What the results show:
- LiDAR resolves 2-3 cm objects but under-reads the toolbox and phone, because it only hits their
  near face, so the height rule alone often crosses them.
- Depth reads the heights of what it sees to within 1-2 cm (toolbox 12 cm, laptop 3 cm, power strip
  4 cm), but the decision depends on the viewpoint. The toolbox was avoided 76%, 100% and 46% of
  the time in D, D4 and D5, and the phone 4%, 100% and 63%. Read single-run percentages as ranges.
- The phone is exactly 8 cm, the step limit, so small noise flips it: a threshold case that height
  alone cannot settle.
- Low but valuable objects (laptop, glasses case) are crossed by the height rule 71% and 98% of the
  time. This is what the operator lists are for. With correct names they are avoided 100% of the
  time; the replay only shows this for correct names, it is not a live test.
- With the VLM on (B3), unverified objects stay at caution. That is safer for the toolbox, but it
  holds the cable and mat at caution because the camera covers only about 70 degrees ahead. The
  real model's names for the sim's plain boxes match nothing on the lists.
- The vase (30 cm) is above `h_tall` and is left to the Nav2 costmap. In runs A and D two 2-5 cm
  fragments next to its base were tracked and judged traversable; they are probably returns at its
  base (not confirmed) and are not the vase.
- The depth camera did not see the cable in D4 and D5 (its field of view and 0.8 m floor blind
  zone); LiDAR did.

## Limitations

- One run per condition, and the depth runs differ a lot, so the numbers are indicative.
- Heights and decisions are matched to objects within 0.2 m of their footprint, which is generous:
  some fragments next to an object (for example 8-12 cm ones beside the 3 cm laptop and book) are
  probably edge artifacts or neighbours, so the min and max heights are noisy.
- Nav2 goals set no heading, so what the camera sees depends on how the dog turns. One run had a
  goal dropped and a 4.5 minute stall, which skews its time shares.
- The sim objects are untextured boxes, so the VLM's fragility check and the name-based lists cannot
  be tested live. The list results are an offline replay with correct names.
- The lists were added after runs A, B3, D and D4; those runs used no lists.
- Each object appears as several track fragments, and results are counted per tick.
- The costmap source script (`scripts/tools/add-traversability-costmap-source.sh`) has not been
  applied, so Nav2 does not yet act on the node's output.
- The live VLM run (E) could not test the lists, because the model cannot recognise the sim's plain boxes and
  its names are unreliable; the list results come from the replay with correct names.
- No user study was run, so nothing here supports claims about ease of use or trust; explanation and
  override tools for operators are future work.

## Troubleshooting

- `wifi_nav.sh is already running`: a leftover process holds `/tmp/wifi_nav.$USER.lock`.
  Run `pkill -f wifi_nav.sh; fuser -k /tmp/wifi_nav.$USER.lock`.
- `ros2` commands from a fresh `docker exec` can show no topics for a few seconds. Run
  `ros2 topic list > /dev/null; sleep 4` first.
- `no TF lidar3d_map -> base_link` at node start is harmless; it clears after a few seconds.
- `Ollama call failed ... Connection refused`: the host stopped serving (sleep, or the Ollama
  app restarted). Check `curl http://<MAC_IP>:11434/api/tags`. The node retries by itself.
- The dog circles or ignores goals: pause the explorer with `/explore/resume` (see above), or
  relaunch the sim to reset the dog and the map frame. Do not teleport it, because that breaks
  the SLAM frame.
- The costmap shows nothing for the small objects: it ignores anything under
  `min_obstacle_height` (0.15 m). Add `/traversability/markers` in RViz to see the node's output.
- Runs that stall, show no detections or have gaps can mean the VM was paused or throttled. Keep the
  VM window in front, do not use the host during a run, and set Windows and Ubuntu to never sleep.
- `ros2 topic hz` on an image topic reports "not published yet" even when frames arrive. Check
  with `ros2 topic echo --once <topic> --field header.stamp` instead.
- If Nav2 closes a goal at once ("Navigation finished OK (0.1s)") the dog does not move; the
  updated `run_goals.sh` sends the goal again after 30 s without progress.
