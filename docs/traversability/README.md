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
4. Depth source: `height_source` is `lidar`, `depth` or `auto`. Only the LiDAR path has been
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

## Results (one run per condition)

Share of the time each object was judged, run A (height rule only) against run B3 (with VLM):

| Object | Should be | A: trav / caution / avoid | B3: trav / caution / avoid |
|---|---|---|---|
| cable | traversable | 100 / 0 / 0 | 0 / 100 / 0 |
| mat | traversable | 100 / 0 / 0 | 0 / 100 / 0 |
| toolbox | avoid | 42 / 0 / 58 | 11 / 54 / 36 |
| phone | avoid | 97 / 0 / 3 | 66 / 9 / 25 |
| vase | avoid | not tracked (above the 20 cm cutoff) | not tracked |

- The LiDAR sees the cable and mat at the right place and height (2-3 cm).
- The LiDAR reads the toolbox at 6-7 cm (true 12 cm) because only its front face is hit, so the
  height rule alone calls it traversable 42% of the time.
- The phone is read at 3-6 cm (true 8 cm) and the VLM calls the red box a "red object",
  `step_over`. The phone ends `traversable` in both runs.
- With the VLM on, unverified objects stay at `caution`, which is safer for the toolbox but
  holds most cable and mat fragments (14 of 15 and 22 of 26) at `caution`, because the camera
  covers only about 70 degrees ahead.

## Limitations

- One run per condition, no repeats. Run A's path was not logged, and the start poses of the
  runs differ slightly.
- Sim objects are untextured boxes, so the VLM's fragility check cannot be tested properly.
- Each object appears as several track fragments, and results are counted per fragment.
- LiDAR height under-reads small objects from the side, and the tracker averages it, so the
  decision can flicker around the 8 cm limit.
- A first VLM run was discarded because the Ollama host went to sleep mid-run.
- The depth camera case is not tested: the sim has no depth sensor (it would need edits to
  `gazebo/sim/build_model.py` and `bridge.yaml`).
- The costmap source script (`scripts/tools/add-traversability-costmap-source.sh`) has not been
  applied, so Nav2 does not yet act on the node's output.

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
