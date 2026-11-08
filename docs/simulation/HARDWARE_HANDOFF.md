# Traversability: Hardware Handoff

[← Simulation record](traversability_sim.md) · [← Back to README](../../README.md)

What the Gazebo simulation established for the Go2 traversability stack, what it
did not, and what the first physical trials must check. Read this before the
hardware work. Evidence links point to [traversability_sim.md](traversability_sim.md),
the single simulation record. Simulation closed 2026-11-08 on branch
`native-ubuntu-setup`.

## 1. What is proven in sim

| Claim | Evidence |
|---|---|
| **Height bands classify correctly at the real foot-lift limits.** Synthetic objects into the local costmap: 0.10 m → 40 (step over), 0.14 m → 200 (caution), 0.20 m → 200 in `HeightBandLayer` and 254 in the full costmap, 0.30 m → 254. Bands: step over ≤ 0.12 m, caution 0.12–0.22 m, lethal ≥ 0.22 m. The obstacle layer's `min_obstacle_height: 0.15` makes the effective caution band 0.12–0.15 m, deliberately. | [Step-over band](traversability_sim.md#step-over-band-narrowed-to-the-real-foot-lift) |
| **Safety fusion: the VLM cannot lower a cost below the LiDAR band.** 0 unsafe published labels on LiDAR-measured objects: 149 trial-objects at n = 10, 0 in every Phase 2 attempt and in all three offline prompt evaluations. The raw VLM called a tall measured object `STEP_OVER` 6 times in Phase 4; the LiDAR floor caught every one. | [Semantic fusion](traversability_sim.md#semantic-fusion-final), [Phase 4](traversability_sim.md#safety-metric-trial-matrix-phase-4) |
| **Two-signal lethal rule.** A VLM `AVOID` is lethal only when LiDAR measured the object; an unmeasured object is held at CAUTION (200), flagged `low_confidence`, with a `CAREFUL` gait. In Phase 4, 55 trial-objects were unmeasured, 4 of them tall, all held at CAUTION. | [Semantic fusion](traversability_sim.md#semantic-fusion-final) |
| **The backstop stops under-cautious gait when the costmap sees something.** Gait `NORMAL` while the costmap had an object in the corridor: 0 of 1465 samples with the backstop on vs 217 of 1822 (11.9 %) off (fixed route, 3 pairs); 0 of 5893 across 20 backstop-on trials at n = 10. Residual `NORMAL` samples are objects neither sensor saw (1–3.6 %). | [Backstop](traversability_sim.md#backstop-measurement-phase-3), [Phase 4](traversability_sim.md#safety-metric-trial-matrix-phase-4) |
| **The stack clears low items more often than a low-stepping baseline.** Items ≤ 0.10 m: 25/47 passes cleared (53 %) vs 15/58 (26 %), p = 0.008, h = 0.57, n = 10 per mode. `HIGH_STEP` cleared 12 of 16 passes; misses ended about 2 cm below the item top. This is the only significant difference between modes; foot strikes per metre and goals do not differ. | [Phase 4](traversability_sim.md#safety-metric-trial-matrix-phase-4) |
| **Gait supervisor thresholds and interface.** `HIGH_STEP` ≤ 0.10 m (0.12 m lift minus 0.02 m margin), `CAREFUL` above, `STOP` ≥ 0.25 m; unmeasured height → `CAREFUL`. Runs with `dry_run` (default true) and `backend: gazebo`; `backend: sport_client` raises `NotImplementedError`. 33 unit tests pass (labeller, fusion, leg gait). | [Step-over band](traversability_sim.md#step-over-band-narrowed-to-the-real-foot-lift), `scripts/nodes/gait_supervisor.py` |
| **MPPI step-over critic builds and runs** against `nav2_mppi_controller` 1.1.20 (in the `go2-3d` image). | [MPPI critic](traversability_sim.md#mppi-step-over-critic-built-null-result-on-these-routes) |

## 2. What is implemented but not validated

- **Semantic layer (VLM).** Integrated end-to-end: `SemanticCostmapLayer` in the
  global and local costmaps, two-signal lethal rule, locked prompt
  (`verify_oneword`, `qwen2.5vl`). Not shown to improve navigation: across three
  fixed-route attempts the VLM's misclassification of small Gazebo-rendered
  objects (a mug called "furniture", a pot called "basketball hoop") closed
  low-clutter corridors or caused RPP aborts. Raw VLM accuracy 0.47–0.68 on
  these crops. At n = 10 the semantic mode reached 21/30 goals vs 27/30 for
  baseline (p = 0.11, not significant) with a median 1.5 RPP aborts per trial.
  The architecture is sound; the perception input is not yet reliable enough for
  safety-critical routing. See
  [Semantic mode: what it does and does not establish](traversability_sim.md#semantic-mode-what-it-does-and-does-not-establish)
  and [VLM prompt selection](traversability_sim.md#vlm-prompt-selection).
- **MPPI step-over critic.** No measurable effect: 0 vs 0 step-over cells crossed
  with and without the critic (cost band 30–60, then 38–42). The routes tried
  never forced a step-over decision. The trial stack uses RotationShim + RPP,
  which has no critics.

## 3. What is assumed, not proven

- **Foot lift 0.12 m.** Taken from the SDK's `FootRaiseHeight` ceiling, never
  measured on this robot. Every band and gait threshold derives from it.
- **Legs are kinematic.** The sim's legs follow a scripted trot with gravity off
  for the legs: no balance, no gait stability, and a foot strike never trips the
  robot. Foot-strike counts are geometric contacts, not falls.
- **Foothold landing.** The blue mug (0.095 m) was cleared on 0 of 3 `HIGH_STEP`
  passes in one check although 12 cm is enough: the foot landed inside the
  footprint. Foothold margin may need widening; deferred to hardware.
- **Perception.** Gazebo rendering, ideal LiDAR returns after the URDF
  self-filter, no motion blur, fixed lighting. Several detections were a
  ground-truth stand-in (classes YOLO lacks, and the modelled laptop).
- **Known perception gaps in sim:** the keyboard (0.024 m, 4 mm above the ground
  tolerance) and book (0.051 m) are rarely seen by either sensor; the toolbox
  (0.30 m) was missed by every mode on one seed.

## 4. Hardware trial hypotheses

1. Does the height band classify real cables, mats and shoes correctly with the
   real LiDAR (step over ≤ 0.12 m, caution above)?
2. Is 0.12 m the real foot-raise ceiling, or lower? Measure before trusting any band.
3. Does the `CAREFUL` gait (12 cm lift, slow) clear a real cable and a real shoe?
4. Does the backstop prevent trips on objects the camera misses: 0 cases of
   `NORMAL` gait while the costmap shows an obstacle in the corridor?
5. Does the safety fusion still give 0 unsafe published labels on measured
   objects with real camera crops?
6. Does `HIGH_STEP` clear items ≤ 0.10 m at least as often as in sim (12 of 16),
   and do misses come from foothold placement rather than lift?

## 5. Parameters likely to change for the real dog

| Parameter | Sim value | Expected change |
|---|---|---|
| `step_over_height_max` (HeightBandLayer, fusion, labeller, analysis) | 0.12 m | Measure the real lift first; may need to go below 0.12. Keep the gait margin (`foot_margin` 0.02 m). |
| `lethal_height_min` | 0.22 m | Revisit with the measured lift and trunk clearance. |
| `min_obstacle_height` (obstacle layer) | 0.15 m | Keep unless the measured lift changes; it is what makes 0.15 m+ impassable. |
| `min_range` (HeightBandLayer) | 0.25 m | Real LiDAR near-field noise differs from the sim's self-filtered scan; likely raise. |
| `ground_tolerance` | 0.02 m | Real floor and LiDAR noise; tune so flat items are not lost or invented. |
| `backstop_enabled` | true | Keep. This is the safety net. |
| `confidence_min` (VLM) | 0.7 | The locked one-word prompt returns no confidence, so this does not filter; raise it or switch prompts for real crops if VLM labels are used. |
| `foot_margin` / foothold avoidance | 0.02 m / 0.08 m | Widen if real feet land on items. |

## 6. Files needing hardware changes, and metrics to reuse

**Files:**

- `scripts/nodes/gait_supervisor.py`: implement the `sport_client` backend
  (currently raises `NotImplementedError`), sending gait as `FootRaiseHeight` and
  speed. Keep `dry_run` as the default.
- `scripts/runtime/wifi_3d_nav.sh`: add a `--semantic` toggle in the option
  `case` block (around line 151) that starts the fusion, gait supervisor and,
  optionally, the VLM node, and loads the semantic-mode costmaps.
- `scripts/nodes/vlm_heuristic_node.py`: `backend` accepts only `gazebo`; the robot
  camera path needs its own backend value.
- Trial configs under `tests/traversability/` (`controller_traversability*.yaml`,
  `planner_traversability_semantic.yaml`) are sim configs; the robot uses the
  `go2_nav2` launch parameters.

**Metrics (all in `tests/traversability/`), with their definitions:**

| Metric | Definition | Tool |
|---|---|---|
| Cells entered per band | Distinct 0.05 m cells under the robot centre, by the band of the true object footprint | `analyze.py` |
| Time per band | Seconds the robot centre spent on cells of each band | `analyze.py` |
| Foot strikes per metre | Contact episodes between a calf/foot and an object, per metre walked | `analyze.py` |
| Foot clearance rate | Share of foot passes over an item whose lowest point stayed above the item top, items ≤ 0.10 m | `analyze.py` |
| NORMAL when care was needed, split | `NORMAL` gait samples where the true corridor needs care: (a) costmap saw it, a backstop failure; (b) neither sensor saw it, a perception gap | `analyze.py` |
| Backstop events | `/gait/status` samples with `backstop_raised` | `analyze.py`, `object_costs.py` |
| Unsafe published labels | LiDAR-measured object ≥ 0.22 m published `STEP_OVER` or `CAUTION`; unmeasured objects counted separately | `analyze_semantic.py` |
| VLM confusion matrix | True band vs published label and vs raw VLM answer, per object per trial | `analyze_semantic.py`, `eval_vlm_frames.py` |
| Costmap cost per object | Raw local and global cost on the object's footprint | `object_costs.py`, `height_band_check.py` |

On the robot the "true" heights and footprints come from measuring the test
objects; the scripts read them from a world JSON in the same format as
`gazebo/worlds/cluttered_items.json`. Reset the gait to `NORMAL` between trials:
an unreset gait made half of one sim baseline arm walk with the high lift.
