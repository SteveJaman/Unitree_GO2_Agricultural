# Early-stopping policy (Phase 4): fails the coverage criterion

[← README](../../README.md) · [Docs index](../README.md) · [Species ladder](species_ladder.md) ·
[Equations](../theory/equations.md) (E5)

**Status (2026-11-19): gate fails on 4 of 4 reachable species** (tomato has no executed waypoints
to test). The policy saves 45-62 % of the energy index and never loses a detection, but loses
20-32 % of the coverage against a limit of 2 %. Per the plan's stop condition (acceptance fails on
more than 2 of 5 species), this is reported before Phase 5.

## Policy

`scripts/nodes/autonomy_policy.py --mode early_stop` (E5). The detector is single-class, so the
per-frame class posterior is (c, 1 − c), with c the confidence of the box on the plant (no box:
0.02; c clipped to [0.02, 0.98]). It is fused over the shell's frames on the log-odds, and
H(b) = −b log₂ b − (1 − b) log₂(1 − b). The current shell stops when **H < 0.3 bit for 3
consecutive frames**, or when its waypoints are done; the next shell starts afresh. The waypoint
order is the fixed one (not reordered).

**Evaluation: offline replay** of the 20 recorded full orbits of Phase 3 (5 plants × lettuce,
basil, spinach, strawberry at (0.40, 0.25)), truncated where the policy stops. The waypoint where
it stops is paid in full; skipped waypoints cost nothing. For comparison, `--mode three_stop` keeps
only the az-0 waypoint of each shell. Acceptance per orbit: energy saving ≥ 30 %, relative coverage
loss ≤ 2 %, the same detection set (the shells in which the plant was detected, conf ≥ 0.25).

## Result

| Species | Orbits | Energy saving (mean) | Coverage loss, relative (mean) | Same detection set | Accepted | 3-stop: saving / loss |
|---|---|---|---|---|---|---|
| lettuce | 5 | 44.9 % | 20.3 % | 5/5 | 0/5 | 46.6 % / 21.7 % |
| basil | 5 | 61.8 % | 31.5 % | 5/5 | 0/5 | 68.3 % / 46.8 % |
| spinach | 5 | 49.1 % | 24.5 % | 5/5 | 0/5 | 62.7 % / 32.2 % |
| strawberry | 5 | 51.9 % | 28.3 % | 5/5 | 0/5 | 65.7 % / 33.1 % |
| tomato | 0 | – | – | – | not testable (no waypoint executes) | – |

Coverage lost per shell (absolute, fraction of the visible half seen by that shell's frames):

| Species | eq | top | under |
|---|---|---|---|
| lettuce | 0.183 | 0.000 | 0.034 |
| basil | 0.160 | not reached | 0.312 |
| spinach | 0.209 | 0.001 | 0.215 |
| strawberry | 0.167 | 0.002 | 0.298 |

Data: [`results/policy/early_stop.csv`](../../results/policy/early_stop.csv) and
[`three_stop.csv`](../../results/policy/three_stop.csv) (per orbit: frames, waypoints, energy,
coverage and detection set for the full sweep and the policy, and where each shell stopped).

## Why it fails

- **The stopping statistic measures detection certainty, not coverage.** Every frame detects the
  plant at about 0.7-0.9 confidence. After 3 frames the fused posterior is above 0.95
  (H < 0.3 bit), so each shell stops during its **first** waypoint (frames f3-f4) or, when the
  posterior carries over, at the first frame of the next. The remaining azimuths, which see other
  sides of the plant, are never visited.
- **It fails on the eq and under shells**, the shells that have more than one waypoint
  (eq az −30/0/+30, under up to −60/−30/0). The top shell has a single reachable waypoint, so
  stopping cannot lose anything there.
- **The detection set is preserved by construction.** The policy stops only after confident
  detections, and the detectors find the plant in every frame, so "no false negatives" holds
  trivially. It does not test whether a later view would have found something else, such as low
  fruit under the canopy.
- **It is roughly on the 3-stop trade-off.** Early stop and the fixed 3-stop sweep (az 0 per
  shell) give similar savings and losses; early stop is slightly better on coverage for basil,
  spinach and strawberry because it sometimes visits a second under-shell waypoint.

## What would pass (not run)

A stopping rule on **coverage gain** rather than detection certainty: stop a shell when the next
waypoint's predicted new visible-half samples (E6, the NBV information gain) fall below a
threshold. That is the next-best-view work deferred to C2.
