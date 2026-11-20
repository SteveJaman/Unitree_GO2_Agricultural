# Results: multi-shell plant inspection from a parked quadruped (simulation)

[← README](../../README.md) · [Docs index](../README.md) · [Orbit frontier](orbit_frontier.md) ·
[Species ladder](species_ladder.md) · [Early stop](early_stop.md) · [Videos](video_index.md) ·
[Equations](../theory/equations.md) · [Optimization](../theory/optimization.md)

## Contribution

In a physics simulation of a Unitree Go2 carrying a D1-550 arm with a wrist RealSense D435i on our
lab's NFT 4-6 rack, a quadruped parked tail-first can inspect a plant from three elevation shells
and see 49-79 % of its robot-facing surface, with the plant detected in every frame; the binding
limit is the arm's dexterity on the view sphere, not its radial reach; and stopping a shell once
the detector is certain saves 45-62 % of the arm's (relative) energy, but only at a 20-32 %
coverage cost.

## Setup

Gazebo Harmonic with the realistic configuration (effort-controlled arm under gravity, trotting
legs, body lag), MoveIt 2 (RRTConnect, collision-checked IK), the D435i at 15 Hz on the claw.
The camera orbits the plant centroid P = (d, 0, h_p) at radius r over three shells: eq (0°,
az −90…90), top (+15°, az −60…60), under (−20°, az −60…60), 30° steps, 17 fixed waypoints, 5 wrist
frames per executed waypoint. Coverage (E4) is the fraction of 3000 visible-half mesh samples seen
(depth-occlusion, ≥ 30 px plant extent, sharpness filtered). Energy (E3) is a **relative** index:
Σ∫|τω|dt over each move plus Σ|τ|·t_hold over the 1 s hold.

## 1. Reach frontier (Table 1, F1, F2)

| d (m) | r (m) | Class | Collision-free azimuth limit eq / top / under | Planar R(θ) limit |
|---|---|---|---|---|
| 0.35 | 0.25-0.40 | blocked: the dog's rear legs reach the reservoir | ±46-27 / ±18-none / ±30-none | 107° to ≥ 180° |
| 0.40 | 0.25 | all shells | ±44 / ±9 / ±28 | 135.9° |
| 0.40 | 0.30 | all shells | ±40 / ±13 / ±23 | 118.9° |
| 0.40 | 0.35 | all shells | ±36 / ±2 / ±16 | 107.4° |
| 0.40 | 0.40 | fail (top, under) | ±31 / none / none | 98.3° |
| 0.45 | 0.25 | all shells | ±41 / ±15 / ±25 | 116.7° |
| 0.45 | 0.30 | all shells | ±39 / ±6 / ±22 | 105.8° |
| 0.45 | 0.35 | all shells | ±36 / ±9 / ±18 | 97.5° |
| 0.45 | 0.40 | fail (top) | ±32 / none / ±13 | 90.6° |
| 0.50 | 0.25 | all shells | ±38 / ±24 / ±30 | 102.4° |
| 0.50 | 0.30 | all shells | ±37 / ±14 / ±20 | 95.0° |
| 0.50 | 0.35 | all shells | ±35 / ±4 / ±17 | 88.9° |
| 0.50 | 0.40 | skipped (R(90°) = 0.640 m > 0.625 m) | | 83.7° |

- 10 of 16 cells reach all three shells with the top shell at +15°; **none** did at +30°
  (`results/orbit_frontier_top30/`).
- **The planar reach model fails as a predictor:** R(θ) ≤ 0.605 m over-predicts the observed limit
  by 55-138° in every cell. It is a necessary condition only.
- **Execution is stricter than kinematics:** in 24 executed orbits (Phase 2), (0.50, 0.25) never
  executed its top waypoint (the effort-controlled arm times out on the move), and (0.50, 0.30)
  executed 12/20 feasible waypoints (60 %, Wilson 95 % CI [39, 78] %). (0.40, 0.25) executed
  20/20 (CI [84, 100] %) with all three shells in every run: coverage 0.519, energy index 12.4.

Figures: [F1](../images/orbit_paper/F1_reach_frontier.png) (frontier heatmap with R(θ) contours),
[F2](../images/orbit_paper/F2_semisphere.png) (reached waypoints on the semi-sphere, per species).

## 2. Species ladder (Table 2, F3, F5)

| Species | mAP50 | mAP50-95 (floor) | Coverage, all shells (mean ± sd, 5 plants) | eq | top | under | Unique detections / frames eq · top · under |
|---|---|---|---|---|---|---|---|
| lettuce (control) | 0.988 | 0.815 (0.80) | 0.487 ± 0.032 | 0.304 | 0.233 | 0.201 | 51/75 · 20/25 · 35/35 |
| basil | 0.993 | 0.898 (0.75) | 0.785 ± 0.028 | 0.450 | not reached | 0.705 | 44/65 · – · 95/105 |
| spinach | 0.980 | 0.869 (0.70) | 0.785 ± 0.047 | 0.485 | 0.182 | 0.676 | 16/75 · 14/25 · 70/75 |
| tomato | 0.952 | 0.766 (0.70) | 0 (no waypoint reachable) | – | – | – | – |
| strawberry | 0.981 | 0.859 (0.65) | 0.782 ± 0.042 | 0.435 | 0.230 | 0.708 | 11/75 · 8/25 · 65/85 |

- All five detectors meet their floors; the lettuce control was not retrained.
- Every executed frame detected its plant (665/665).
- **The under shell earns its cost on every reachable species** (most unique detections; the
  highest coverage on the low, spreading plants). The top shell's single waypoint is cheap and
  adds unique views on lettuce (20/25) and spinach (14/25).
- **The tomato (0.60 m tall, centroid 0.61 m above the arm's shoulder) is out of reach** from
  this platform: every waypoint fails IK or collides.

Figures: [F3](../images/orbit_paper/F3_species_shell.png) (coverage and unique detections per
species and shell), [F5](../images/orbit_paper/F5_wrist_frames.jpg) (wrist frames with boxes).

## 3. Energy and the stopping policy (Table 3, F4)

| Species | Policy | Energy saving (mean [95 % CI]) | Coverage loss, relative | Same detection set | Accepted (≥ 30 % saving, ≤ 2 % loss, same set) |
|---|---|---|---|---|---|
| lettuce | early stop | 44.9 [39.0, 51.0] % | 20.3 [14.1, 26.4] % | 5/5 | 0/5 |
| basil | early stop | 61.8 [54.9, 68.5] % | 31.5 [29.9, 34.1] % | 5/5 | 0/5 |
| spinach | early stop | 49.1 [45.3, 53.7] % | 24.5 [19.5, 28.9] % | 5/5 | 0/5 |
| strawberry | early stop | 51.9 [47.2, 56.3] % | 28.3 [22.3, 32.9] % | 5/5 | 0/5 |
| lettuce | 3-stop (az 0) | 46.6 % | 21.7 % | 5/5 | 0/5 |
| basil | 3-stop | 68.3 % | 46.8 % | 5/5 | 0/5 |
| spinach | 3-stop | 62.7 % | 32.2 % | 5/5 | 0/5 |
| strawberry | 3-stop | 65.7 % | 33.1 % | 5/5 | 0/5 |

The entropy early stop (E5: a shell stops when H < 0.3 bit for 3 frames) fails acceptance on all
four testable species: the energy saving is significant (Mann-Whitney U, p = 0.008 against the full
sweep for every species, n = 5 per group), the detection set is unchanged (Fisher p = 1), but the
coverage loss is 10-16 times the limit. The detector is certain within the first waypoint, so each
shell stops before its other azimuths; the loss falls on the eq and under shells. Early stop and
the fixed 3-stop sweep lie on about the same energy-coverage trade-off (F4). Evaluated by offline
replay of the recorded orbits; a physical early-stop orbit is in the videos (V3c).

Figure: [F4](../images/orbit_paper/F4_energy_pareto.png). Statistics:
[`results/orbit_stats.md`](../../results/orbit_stats.md).

## 4. Distance servo (F6)

The servo study (commit 4f36620, [servo demo](servo_demo.md)) is reused, not redone:
[F6](../images/orbit_paper/F6_servo_robustness.png).

## 5. Videos

See [video_index.md](video_index.md) for paths. Five screen captures:

- V1: the full trial, tiled with the three RViz views (200 s, plus a 92 s composed version).
- V3a: a lettuce full orbit (476 s).
- V3c: the lettuce early-stop orbit (52 s).
- V3b: a strawberry under-shell orbit (201 s).

Plus 12 V2 clips: the wrist camera per species and shell, with the YOLO box (4-20 s each).

## Limitations

- **Simulation only.** No real plants were inspected; the rack is our NFT 4-6, the lettuce is a
  scanned model, and the plants' leaves are static meshes (no wind, no growth, no occlusion by
  moving leaves).
- **Energy is relative.** The index ranks policies within the simulation; it is not battery
  energy until one scalar per term is calibrated on the real D1. All savings are percentages.
- **Species models are stand-ins.** Basil, spinach, tomato and strawberry are geometric stand-ins
  generated for this study; their detectors' mAP measures the pipeline on those models, not field
  accuracy. The detectors also fire on leaf fragments and on neighbouring plants (visible in F5),
  which coverage scoring ignores (only the box overlapping the target plant counts).
- **Low trial-to-trial variance.** The simulation is close to deterministic; repeats measure
  simulation noise. The species study varies the plant (5 per species) instead.
- **Fixed waypoint order and grid.** Waypoints are not reordered (deferred, C2/C3); with a 30° grid
  the eq shell gets 3 views and top 1 view at the best cell.
- **Per-plant policy.** The early stop is a per-plant stopping rule evaluated offline, not
  unsupervised multi-plant autonomy.

## Figures and tables for a Paper 2 draft

| Paper item | Source | Shows |
|---|---|---|
| Fig. 1 setup | `docs/images/servo_demo/world_rack.jpg`, V1 | the dog, arm, camera and rack |
| Fig. 2 reach frontier | F1 | observed vs analytic limit over d × r and shells |
| Fig. 3 view sphere | F2 | reached waypoints per species |
| Fig. 4 shell value | F3 | coverage and unique detections per species × shell |
| Fig. 5 energy trade-off | F4 | full vs 3-stop vs early stop |
| Fig. 6 example views | F5 | wrist frames with detections |
| Fig. 7 servo | F6 | distance servo robustness (prior study) |
| Table 1 | §1 above, `results/orbit_frontier/frontier.md` | frontier per cell |
| Table 2 | §2 above, `results/species_map.md` | detectors and coverage per species |
| Table 3 | §3 above, `results/orbit_stats.md` | policies with CIs and tests |
| Equations | `docs/theory/equations.md` | E1-E8 |
| Supplementary video | V1, V3 | the trial and orbits |

## Acceptance checklist

- [x] Sim health check passed and screenshot saved: colour 13.9 Hz, depth 11.7 Hz, tracker
  12 pairs/s, 4 windows ([sim_health](sim_health.md)).
- [x] `species_config.yaml` covers 5 species.
- [ ] Frontier table complete for all 16 cells, **R(θ) matches within 5°: fails.** The table is
  complete (§1), but R(θ) over-predicts the observed limit by 55-138° in every cell.
- [x] At least one cell reaches all 3 shells on lettuce: 10 cells, with the top shell at +15°
  (none at +30°).
- [x] Species mAP floors met: all five.
- [x] Unique-detection table populated per species per shell (tomato: no frames, out of reach).
- [ ] Early stop meets 30 % energy / 2 % coverage / no lost detections on all 5 species: fails on
  lettuce, basil, spinach and strawberry; tomato not testable. Failures are documented per
  species and per shell ([early_stop](early_stop.md)).
- [x] All figures regenerate from CSV with one command (`scripts/tools/make_paper_figures.py`).
- [x] Stats table present with Wilson + Mann-Whitney + Fisher
  ([`results/orbit_stats.md`](../../results/orbit_stats.md)).
- [x] V1, V2, V3 MP4s exist at the documented paths and play
  ([video_index](video_index.md)).
- [x] `equations.md` and `optimization.md` committed.
- [x] `results_orbit.md` committed with contribution sentence and limitations.
