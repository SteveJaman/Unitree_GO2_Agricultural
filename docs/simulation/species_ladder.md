# Species ladder (Phase 3)

[← README](../../README.md) · [Docs index](../README.md) · [Orbit frontier](orbit_frontier.md) ·
[Sim health + species](sim_health.md) · [Equations](../theory/equations.md)

**Status (2026-11-19): gate passes.** Every detector meets its floor, and the per-shell
unique-detection table is filled in for all five species. Tomato is the exception for coverage:
the arm cannot reach any tomato waypoint (see below).

## 3a. Detectors

300+ wrist-view frames per species from `scripts/tools/lettuce_dataset.py --from-weights
lettuce_yolo11s.pt --prefix <species>_` in the species world (`cropking_nft46_<species>`),
fine-tuned for 30 epochs, evaluated on the held-out split (every 8th frame, never trained on) by
`scripts/tools/species_eval.py`. The lettuce control was not retrained.

| Species | Train frames | Held-out frames | mAP50 | mAP50-95 | Floor (mAP50-95) | Result |
|---|---|---|---|---|---|---|
| lettuce (control) | 445 | 64 | 0.988 | 0.815 | 0.80 | pass |
| basil | 342 | 49 | 0.993 | 0.898 | 0.75 | pass |
| spinach | 350 | 50 | 0.980 | 0.869 | 0.70 | pass |
| tomato | 350 | 50 | 0.952 | 0.766 | 0.70 | pass |
| strawberry | 350 | 50 | 0.981 | 0.859 | 0.65 | pass |

Data: [`results/species_map.csv`](../../results/species_map.csv). No rung fails, so no rung needs
extra frames from a particular shell. The plant models other than lettuce are geometric
stand-ins (`gazebo/models/*_plant/README.md`), so these numbers measure the pipeline, not
field accuracy.

## 3b. Orbits: 5 plants per species

The 3-shell orbit (17 fixed waypoints, top shell at +15°) at **(d, r) = (0.40, 0.25)**: the best
cell in execution in Phase 2 (all three shells executed in every run). It was run around the
five front plants (sites 1-5 of channel 0) of each species world, scored with each species' own
detector (`coverage_eval.py --per-shell`) and `scripts/tools/unique_detections.py`.

**Unique detection (the region match).** For a frame f of shell s in run k, the detection region
R_f is the set of the plant's visible-half surface samples (3000 per plant, fixed seed) that are
seen in f (in the image, not occluded: rendered depth within 1.5 cm) **and** inside f's YOLO box
for that plant (IoU ≥ 0.1 with the plant's projected box). U is the union of the regions of every
frame of the **other** shells of the same run. f is a unique detection when
|R_f \ U| ≥ 20 samples and |R_f \ U| ≥ 20 % of |R_f|: this shell's detection covers a part of
the plant that no other shell's detection covered.

| Species | Coverage, all shells | eq | top | under | Unique detections / frames: eq | top | under | Energy index per orbit: eq | top | under |
|---|---|---|---|---|---|---|---|---|---|---|
| lettuce | 0.487 | 0.304 | 0.233 | 0.201 | 51 / 75 | 20 / 25 | 35 / 35 | 7.4 | 1.8 | 4.8 |
| basil | 0.785 | 0.450 | **0** (not reached) | 0.705 | 44 / 65 | – | 95 / 105 | 6.8 | 0 | 10.5 |
| spinach | 0.785 | 0.485 | 0.182 | 0.676 | 16 / 75 | 14 / 25 | 70 / 75 | 7.3 | 1.8 | 8.8 |
| tomato | **0** | 0 | 0 | 0 | – | – | – | 0 | 0 | 0 |
| strawberry | 0.782 | 0.435 | 0.230 | 0.708 | 11 / 75 | 8 / 25 | 65 / 85 | 7.3 | 1.7 | 9.8 |

Coverage is the mean over the 5 plants. Unique detections are summed over the 5 orbits. The
energy index is the mean per orbit, a relative number (E3). Every executed frame had a detection
of its plant (conf ≥ 0.25): lettuce 135/135, basil 170/170, spinach 175/175, strawberry 185/185.
Data: [`results/species/`](../../results/species/) (`<species>_trials.csv` per waypoint,
`_runs.csv` per run and shell, `_frames.csv.gz` per frame, `unique.csv`).

### What did not execute, and why

- **Tomato: nothing.** The tomato is 0.60 m tall, which puts its centroid at 1.09 m, 0.61 m above
  the D1's shoulder on the dog and beyond the arm's 0.605 m reach once the camera has to look at
  it. All eq and top waypoints fail IK (70/85); the under waypoints that solve put the arm inside
  the plant (`COLLISION`, 15/85). The orbit has to be re-targeted (the lower canopy or fruit
  band), or the arm mounted higher, to inspect a tomato. This is a reach limit of the platform,
  not of the detector, which passes its floor.
- **Basil: no top shell.** The basil is 0.29 m tall (centroid 0.93 m, 0.09 m above the lettuce's): the top az-0 waypoint
  solves but its move times out under effort control (`EXEC_FAIL`, 4/5) or fails IK.
- On the other species the orbit executes what Phase 2 predicts at (0.40, 0.25): eq az −30/0/+30,
  top az 0, under az 0 on lettuce, and also az ±30 on most basil, spinach and strawberry plants. The cause
  is not established: their centroids are at about the lettuce's height (0.85 m; basil 0.93 m)
  and their planning-scene spheres are not smaller (spinach 0.126 m, lettuce 0.110 m radius).

## Verdict: does each shell earn its cost?

- **Under: yes, for every reachable species.** It has the most unique detections
  (lettuce 35/35, basil 95/105, spinach 70/75, strawberry 65/85 frames) and, on the low, spreading
  plants (basil, spinach, strawberry), the highest coverage (0.68-0.71). It costs the most energy
  on those plants (8.8-10.5 per orbit) because it executes up to three waypoints there.
- **Eq: yes for lettuce and basil, marginal for spinach and strawberry.** On lettuce it is the
  largest shell (0.30 coverage, 51/75 unique frames). On spinach and strawberry the under shell
  already sees most of what eq sees (16/75 and 11/75 unique frames), at about 7.3 per orbit.
- **Top: yes for lettuce, partly for spinach and strawberry.** One waypoint (1.7-1.8 per orbit)
  gives 20 of 25 unique frames on lettuce (its leaves cup upward), 14/25 on spinach and 8/25 on
  strawberry. It is not reachable on basil or tomato.
- **Tomato: no shell is reachable** from this platform at this cell.

Phase 4 tests whether stopping a shell early (once the detector's posterior is certain) keeps
this information at a lower energy cost.
