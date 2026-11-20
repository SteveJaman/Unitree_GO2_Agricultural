# Orbit inspection: equations (E1–E8)

[Overview](README.md) · [Frontier](frontier.md) · [Species](species.md) · [Early stop](early_stop.md) ·
[Videos](videos.md) · [Equations](equations.md) · [Optimization](optimization.md) ·
[Sim health](sim_health.md) · [Docs index](../README.md)

Notation: the arm frame has its origin at the D1's base on the dog's back, x pointing at the plant
(the arm is modelled facing forward; the dog parks tail-first), z up. Lengths in m, angles in
degrees unless noted, torques in N·m, angular velocities in rad/s, time in s.

## E1: reach, R(θ), and the feasibility condition

The plant centroid is P = (d, 0, h_p): d is the stand-off of the arm base from the plant along x,
and h_p = 0.844 m is the centroid's height on the rack (the arm frame shares the world's z here).
The camera orbits P at radius r. In the horizontal plane, a camera at azimuth θ (0 = between
the arm and the plant) sits at

  C(θ) = P + r(−cos θ, sin θ, 0)

and its horizontal distance from the arm's base axis is (law of cosines, triangle base, P, C):

  **R(θ) = √((d − r cos θ)² + (r sin θ)²) = √(d² + r² − 2 d r cos θ)**  [m]

The D1-550's reach is 0.605 m (shoulder to claw tip). A waypoint is radially feasible when

  **R(θ) ≤ 0.605 m**, i.e. **cos θ ≥ (d² + r² − 0.605²) / (2 d r)**

so the planar limit is θ_max = arccos((d² + r² − 0.605²)/(2dr)), and 180° when the right side
is ≤ −1. At θ = 90°, R(90°) = √(d² + r²): `run_orbit_sweep.sh` skips cells with R(90°) > 0.625 m
(0.605 m plus a 2 cm margin).

E1 is a necessary condition only. The observed reach-only limits (IK with joint limits, no
collisions) are 55° to more than 138° tighter in every cell
([frontier](frontier.md)): the binding constraint is turning the camera
sideways onto a plant 0.36 m above the shoulder while keeping it level (wrist dexterity), not
radial reach. The wrist-centre variant W = C − a x_c − b z_c, |W − S| ≤ L (a = 0.163 m,
b = 0.054 m, S the shoulder at 0.4796 m, L = 0.479 m) is computed by
`scripts/tools/orbit_reach_check.py` alongside.

## E2: shells and the observable semi-sphere

The camera position on the orbit sphere at azimuth θ and elevation φ:

  **C(θ, φ) = P + r(−cos φ cos θ, cos φ sin θ, sin φ)**,  viewing direction x_c = (P − C)/r

camera up z_c the world vertical projected orthogonal to x_c (roll 0; ±30° also tried).

| Shell | Elevation range | Waypoint elevation | Waypoint azimuths |
|---|---|---|---|
| under | −25° ≤ φ < −10° | −20° | −60 … 60, step 30 |
| eq | −10° ≤ φ ≤ +10° | 0° | −90 … 90, step 30 |
| top | +10° < φ ≤ +20° | +15° (was +30°, unreachable everywhere; changed 2026-11-19) | −60 … 60, step 30 |

The **observable semi-sphere** is the half of the sphere facing the robot,
S_obs = {C(θ, φ): |θ| ≤ 90°, −25° ≤ φ ≤ +20°}; its plant-side counterpart is the **visible half**
of the plant surface, the mesh samples s with s_x ≤ P_x (the far half faces the rack and is never
seen from a front orbit). The reachable part of S_obs per cell is the set of (θ, φ) whose
waypoints pass IK, collision checking and planning (the frontier).

## E3: arm energy

  **E = Σ_i ∫ |τ_i ω_i| dt + E_hold**,  **E_hold = Σ_i |τ_gravity,i| · t_hold**

τ_i is joint i's effort (N·m) and ω_i its velocity (rad/s), from `/joint_states` of the
effort-controlled arm (SIM_REALISM=1). The integral (J) runs over the move and the settling
(`exec_energy_j`, trapezoid-free left sum over the 50–100 Hz samples). E_hold is evaluated over
the 1 s hold before capture with τ_gravity,i the mean |τ_i| over the hold (the arm is static, so
the effort is the gravity load) and t_hold = 1 s (`hold_energy_j`).

Units: the first term is mechanical work in J; E_hold is a torque-time integral in N·m·s, a proxy
for the motor's I²R holding cost (∝ τ² for a DC motor, linear here). The sum is therefore a
**relative cost index**, comparable between policies and cells in this simulation, and not a
battery energy. Converting it to joules on hardware requires one scalar per term, calibrated on
the real D1 (see [optimization](optimization.md), "Limitations").

## E4: coverage fraction

N = 3000 points s_j sampled on the plant mesh, area-weighted with a fixed seed, placed with the
plant's pose and scale in the world SDF; V = {j: s_j,x ≤ P_x} is the visible half. In frame f
with intrinsics K and camera pose (R_f, t_f) the sample's pixel and depth are
u_j = π(K R_fᵀ (s_j − t_f)), z_j. Sample j is **seen** in f when

- u_j is in the image and z_j > 0.1 m;
- |D_f(u_j) − z_j| ≤ 0.015 m (D_f the rendered depth: not occluded);
- the plant's projected box in f is ≥ 30 px on its shorter side (resolution);
- f is sharp: Var(∇² I_f) ≥ 15 (variance of the Laplacian of the grey image).

  **c(F) = |∪_{f ∈ F} {j ∈ V: j seen in f}| / |V|**

for a set of frames F: a waypoint (its 5 frames), a shell (union of its waypoints) or a run.
Frames failing the sharpness test are dropped and counted. Implementation:
`scripts/nodes/coverage_eval.py`.

## E5: entropy early stop

At each frame the detector gives class probabilities p = (p_1 … p_K) for the plant (for a
single-class detector, K = 2: p = (c, 1 − c) with c the highest confidence of a box overlapping
the plant's projected box, IoU ≥ 0.1, and c = 0 when there is none). With the per-frame
posterior fused over the frames so far by a recursive Bayes update on the log-odds,

  ℓ_t = ℓ_{t−1} + log(c_t / (1 − c_t)),  ℓ_0 = 0 (prior 0.5),  b_t = σ(ℓ_t),  c_t clipped to [0.02, 0.98], c_t = 0.02 without a box

the entropy in bits is

  **H(p) = −Σ_i p_i log₂ p_i**, here H(b_t) = −b_t log₂ b_t − (1 − b_t) log₂(1 − b_t)

**Stop rule:** the current shell ends when H < 0.3 bit for k = 3 consecutive frames, or when its
waypoints are exhausted; the next shell starts with its own posterior reset (ℓ = 0). H < 0.3 bit
means b > 0.947 or b < 0.053. Implementation: `scripts/nodes/autonomy_policy.py --mode early_stop`.

## E6: next-best-view information gain (defined; implemented in a follow-up)

For a candidate waypoint w with predicted visibility set S(w) ⊆ V (E4's test on the mesh, with
the rendered depth replaced by a z-buffer of the mesh) and the coverage so far U,

  **IG(w) = Σ_{j ∈ S(w) \ U} H(q_j)**,  q_j the probability sample j's surface state is unknown (1 bit per unseen sample when q_j = 0.5)

and the greedy next-best view is w* = argmax_w IG(w) / (E_move(w) + E_hold), E_move from E3 with
the planned trajectory. **Implemented in a follow-up (C2)**: the brief keeps the waypoint order
fixed this round, so no NBV run exists.

## E7: multi-plant tour cost

Plants i = 1 … n at parking poses p_i; base travel cost c_ij (Nav2 path length × cost per metre)
and per-plant arm cost a_i = Σ_{w ∈ orbit_i} (E_move(w) + E_hold(w)) (E3). The tour is a TSP:

  **min_π  Σ_{k=1}^{n−1} c_{π(k) π(k+1)} + c_{0 π(1)} + Σ_i a_i**,  π a permutation of the plants

The arm term does not depend on the order, so the tour is a TSP on c alone plus a constant; with
early stopping a_i becomes a random variable and the expected tour cost is Σ c + Σ E[a_i].
Defined here for Paper 2's multi-plant framing; no multi-plant run this round (the policy is
per plant).

## E8: Pareto objective

  J = w_t T + w_e E − w_i I  (**E8**)

subject to I ≥ I_min (coverage or detections), R(θ) ≤ 0.605 m for every waypoint (E1), every
waypoint collision-free, and T ≤ T_budget. T is the orbit time (plan, execution, settling, hold and
capture, in s), E from E3 (relative index), I the information gathered (coverage fraction E4, or
the number of unique detections). Sweeping the weights w ≥ 0 traces the Pareto front of
(T, E, −I); the manuscript reports the front itself (F4) rather than one weighted optimum.
