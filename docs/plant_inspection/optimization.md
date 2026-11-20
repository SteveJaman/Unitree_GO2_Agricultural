# Orbit inspection: optimization

[Overview](README.md) · [Frontier](frontier.md) · [Species](species.md) · [Early stop](early_stop.md) ·
[Videos](videos.md) · [Equations](equations.md) · [Optimization](optimization.md) ·
[Sim health](sim_health.md) · [Docs index](../README.md)

## Decision variables

Per plant: the stand-off d and the orbit radius r (one cell of the d × r matrix), the set of
waypoints visited per shell, and when to stop a shell. Across plants: the visiting order
(deferred).

## Constraint set

| Constraint | Form | Where it is checked |
|---|---|---|
| Radial reach | R(θ) ≤ 0.605 m (E1); cells with R(90°) > 0.625 m skipped | `run_orbit_sweep.sh` stage 1 |
| Kinematic feasibility | the camera pose C(θ, φ), looking at P, has an IK solution within the joint limits, position error < 1 mm, orientation error < 0.02 rad | `orbit_waypoints.py` (DLS IK, KDL fallback) |
| Collision-free | the IK solution passes MoveIt's `/check_state_validity` with the dog, rack and plants in the planning scene; the plan from the previous waypoint is collision-free (RRTConnect) | `orbit_waypoints.py` |
| Base clearance | the parked dog's rear legs clear the rack's reservoir (blocks d = 0.35 m) | `orbit_waypoints.py base_clear` |
| Shell feasibility | a shell counts for a cell when at least its az-0 waypoint is feasible; a cell is "all-shells" when every shell is | `orbit_frontier_table.py` |
| Time budget | T ≤ T_budget per plant; 180 s per waypoint is the watchdog's hard cap | `orbit_waypoints.py` watchdog |
| Information | I ≥ I_min (coverage fraction, or no detection lost vs the full sweep) | `autonomy_policy.py` acceptance |

## Solvers actually used

- **Orbit (d, r) and waypoints: enumeration.** The 16 cells are enumerated; per cell and shell
  the azimuth frontier is found by continuation (5° steps, then 1°), and the orbit visits the
  fixed waypoint list (eq −90…90, top and under −60…60, step 30) in its fixed order, skipping
  infeasible ones. The brief forbids reordering this round; order optimization is deferred.
- **Early stop: a threshold rule on an entropy statistic** (E5), evaluated against the full sweep
  on the same recorded orbits. It is a per-plant stopping policy, not a planner, and not a claim
  of unsupervised autonomy.
- **Next-best view: greedy** maximization of IG per unit energy (E6). Defined, not run (C2).
- **Multi-plant tour: TSP** over the Nav2 travel costs (E7); with ≤ 10 plants exact enumeration
  or a 2-opt heuristic suffices. Defined, not run (C3).

## "Optimal" means Pareto-optimal

The manuscript frames optimality as the Pareto front of time T, energy E and information I
(E8), not a single weighted optimum: the weights w_t, w_e, w_i depend on the mission (battery,
throughput, inspection depth) and are not ours to fix. Figure F4 shows energy against the number
of stops for the 8-stop full sweep, the 3-stop (az 0 per shell) sweep and the early-stop policy,
per species; a policy is on the front when no other uses less energy for at least the same
coverage and detection set.

## Limitations

- **Simulation energy is relative.** E3 is computed from the simulated effort-controlled joints
  (gains, friction and masses from the D1 URDF). Its first term is mechanical work, its second a
  torque-time proxy for holding current, so the sum is a cost index that ranks policies within
  the simulation. It is **not** comparable to battery joules until one scalar per term is
  calibrated on the real D1 (motor current against torque, at the hold and during moves). All
  savings are reported as relative percentages.
- **Offline policy evaluation.** The early-stop policy is evaluated by replaying recorded full
  orbits in their visiting order and truncating them; the skipped waypoints' energy is removed,
  the move from the stop to the next shell's first waypoint is approximated by the recorded one.
- **Plant models.** Lettuce is the scanned control; basil, spinach, tomato and strawberry are
  geometric stand-ins (`gazebo/models/*_plant/README.md`).
- **Deferred:** waypoint ordering, NBV (C2) and multi-plant tours (C3).
