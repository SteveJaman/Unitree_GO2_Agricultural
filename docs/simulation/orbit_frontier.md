# Three-shell orbit frontier (Phase 2): stopped at the gate

[← README](../../README.md) · [Docs index](../README.md) · [Sim health + species](sim_health.md) ·
[Servo demo](servo_demo.md)

**Status (2026-11-19): Phase 2 stopped.** No cell of the d × r matrix reaches all three shells
on the lettuce: the top shell is out of reach everywhere. Per the plan's stop condition, the
orbit executions, coverage numbers and Phases 3-8 wait for a decision on the shells (below).

## Question

Can the D1-550 with the D435i on its claw orbit a lettuce on the lab's NFT 4-6 from the parked
Go2, over three elevation shells around the plant centroid (h_p = 0.844 m)?

| Shell | Elevation range | Waypoints (az, deg) | Waypoint elevation |
|---|---|---|---|
| under | −25 to −10° | −60 −30 0 30 60 | −20° |
| eq | −10 to +10° | −90 −60 −30 0 30 60 90 (the 180° arc) | 0° |
| top | +20 to +40° | −60 −30 0 30 60 | +30° |

Camera at C = P + r(−cos el cos az, cos el sin az, sin el) around P = (d, 0, h_p), looking at P,
camera up as near vertical as the look-at allows (roll 0, ±30° tried).

## Method

`scripts/runtime/run_orbit_sweep.sh` over d ∈ {0.35, 0.40, 0.45, 0.50} × r ∈ {0.25 … 0.40} m:

1. **Analytic** R(90°) = √(d² + r²) against 0.605 m: (0.50, 0.40) skipped (0.640 m); (0.50, 0.35)
   marginal (0.610 m).
2. **Collision-free frontier** (`scripts/nodes/orbit_waypoints.py --mode ik-frontier`, in the sim):
   with MoveIt's planning scene for the dog parked at d (Go2 body and legs, rack, the plants), the
   azimuth limit per shell: damped-least-squares IK continued in azimuth (5° steps, then 1°), every
   solution checked with `/check_state_validity`; KDL (`/compute_ik`) as a last resort.
3. **Reach-only frontier** (`scripts/tools/orbit_reach_check.py`, host): the same IK with joint
   limits only (no collisions), roll 0, and the reachable elevations at az 0.
4. **Dog vs rack**: the parked dog's rear legs (0.37 m behind its centre) against the reservoir
   under the channels.

## Result

| d (m) | r (m) | class | collision-free eq / top / under | reach-only eq / top / under | az 0 elevations reachable | planar R(θ) limit |
|---|---|---|---|---|---|---|
| 0.35 | 0.25 | blocked (dog vs rack) | ±46 / none / ±30 | ±42 / none / ±23 | −32..+20° | ≥ 180° |
| 0.35 | 0.30 | blocked | ±41 / none / ±22 | ±37 / none / ±15 | −31..+16° | 137.0° |
| 0.35 | 0.35 | blocked | ±34 / none / ±8 | ±31 / none / none | −31..+11° | 119.6° |
| 0.35 | 0.40 | blocked | ±27 / none / none | ±24 / none / none | −11..+7° | 107.4° |
| **0.40** | **0.25** | fail (top) — **best partial** | **±44 / none / ±28** | ±39 / none / ±11 | **−30..+22°** | 135.9° |
| 0.40 | 0.30 | fail (top) | ±40 / none / ±23 | ±36 / none / ±17 | −29..+18° | 118.9° |
| 0.40 | 0.35 | fail (top) | ±36 / none / ±16 | ±33 / none / ±11 | −28..+15° | 107.4° |
| 0.40 | 0.40 | fail (top, under) | ±31 / none / none | ±28 / none / none | −16..+11° | 98.3° |
| 0.45 | 0.25 | fail (top) | ±41 / none / ±25 | ±21 / none / ±20 | −29..+22° | 116.7° |
| 0.45 | 0.30 | fail (top) | ±39 / none / ±22 | ±34 / none / ±16 | −27..+19° | 105.8° |
| 0.45 | 0.35 | fail (top) | ±36 / none / ±18 | ±32 / none / ±13 | −27..+17° | 97.5° |
| 0.45 | 0.40 | fail (top) | ±32 / none / ±13 | ±29 / none / ±9 | −25..+13° | 90.6° |
| 0.50 | 0.25 | fail (top) | ±38 / none / ±30 | ±32 / none / ±27 | −29..+22° | 102.4° |
| 0.50 | 0.30 | fail (top) | ±37 / none / ±20 | ±20 / none / ±16 | −26..+20° | 95.0° |
| 0.50 | 0.35 | fail (top) | ±35 / none / ±17 | ±31 / none / ±8 | −25..+18° | 88.9° |
| 0.50 | 0.40 | skipped (R(90) 0.640 m) | | ±29 / none / ±10 | −24..+15° | 83.7° |

The collision-free limits can exceed the reach-only ones because the frontier also tries the
camera rolled ±30° about its viewing axis; the reach-only check keeps it upright.

Data: [`results/orbit_frontier/`](../../results/orbit_frontier/) (per-cell frontier JSON,
`reach.json`, `frontier.md`, `cells.tsv`).

## Findings

1. **The top shell is out of reach.** With the camera looking down at the plant, the highest the
   claw camera gets is about +22° above the centroid, at az 0 only (d ≥ 0.40, r = 0.25); the top
   waypoints sit at +30°. Lowering the top waypoints does not rescue the shell's azimuth range
   (reach-only, best cells): el +10° → ±27° (d 0.50, r 0.25), +15° → ±22°, +20° → ±12°.
2. **The 180° eq arc is out of reach.** The best unblocked cell reaches ±44° (d 0.40, r 0.25),
   not ±90°.
3. **The limit is dexterity, not radial reach.** At the observed eq limits the camera is only
   0.2-0.3 m from the J1 axis, well inside 0.605 m: what stops the orbit is turning the camera
   sideways onto the plant while keeping it level, with the plant's centroid 0.36 m above the
   D1's shoulder (0.48 m on the dog, 0.844 m on the rack).
4. **The planar R(θ) does not predict the frontier.** R(θ) ≤ 0.605 m over-predicts the reach-only
   eq limit by 55° to over 138° in every cell, so the gate's "analytic and observed fail azimuths
   within 5°" fails. R(θ) is a necessary condition (a radial reach bound), not the binding one.
5. **d = 0.35 is blocked by the dog itself**: its rear legs would reach into the reservoir tub.

## Decision needed (not taken)

The plan's options, and what the data says about each:

| Option | Effect on the top shell | On the eq arc |
|---|---|---|
| Reduce the shells' elevations | a top shell at +10 to +15° reaches ±22-27° at best | no change |
| Raise the standoff ceiling (larger d) | no: the top is limited by height and look-down, not standoff | the eq limit falls with d |
| (not in the plan) lower the plant relative to the arm (a lower tier, a taller arm mount) | addresses the cause (the plant 0.36 m above the shoulder) | likely wider |
| (not in the plan) a downward-pitched camera mount on the claw | lets the camera look down without raising the wrist | to be checked |

## Engineering notes

- **IK.** MoveIt's KDL plugin (50-200 ms, seeded from the previous waypoint) and the repo's
  damped-least-squares IK were both used. A first version of the DLS IK used the usual
  orientation error Σ x_i × x_i*, which is zero for every 180° rotation too: it "found" solutions
  with the camera turned away (one test orbit executed them; its frames looked at the light bar).
  The error is now the axis-angle of R*·Rᵀ and a solution is accepted only within 0.02 rad;
  the table above is from the corrected IK (frontier run `frontier4`). An earlier frontier run
  (`frontier3`) with the flawed IK showed far wider, wrong limits and is discarded.
- **Execution under effort control.** The effort-controlled arm lags its trajectory; MoveIt's
  execution monitor (1.2 × the planned duration) cancelled slow but correct moves, so with
  `SIM_REALISM=1` it now allows 3 × + 3 s. Long swings still time out occasionally
  (`EXEC_FAIL(-6)`), and are logged as such.
- **Planner.** The MoveIt config names no planner set for the group; `planner_id
  "RRTConnect"` is OMPL's RRTConnect.
- **Watchdog.** A waypoint over 180 s or a Gazebo clock stalled for 10 s exits 42; the sweep
  restarts the simulation (`ABORTED_SIM_HANG`), more than 5 restarts stop it.

## Not done (waits for the decision)

- Orbit executions over the feasible cells (`run_orbit_sweep.sh` stage 3) and
  `results/orbit_trials.csv`: no cell is feasible on all three shells.
- `scripts/nodes/coverage_eval.py` is written and runs; its only test input was the orbit run with
  the flawed IK (camera turned away, coverage ≈ 0), so it is not yet validated on real orbit frames.
- The species datasets and detectors of Phase 3 were generated in the background
  (`~/.cache/go2-yolo/<species>_dataset`, `<species>_yolo11s.pt`), not evaluated or committed.
