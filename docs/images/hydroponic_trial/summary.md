# Hydroponic trial summary (hydroponic_report.py)

## Trial B: Mission Score 99.6 / 100

Collisions 0 (40/40) · completion 99.1 % (39.6/40) · worst Cartesian deviation 0.02 % (20.0/20)

| timestamp | phase | exec_time_s | ee_distance_m | planned_length_m | success | cartesian_fraction | mean_dev_mm | max_dev_mm | dev_pct | end_error_mm | joint_velocity_spikes | notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2026-11-02T15:20:42 | Navigation | 15.77 | 0.0005 | 0.0 | True |  |  |  |  |  | 0 | Nav2 arrived 0.255 m off; precision approach to 5 mm, yaw -0.4 deg |
| 2026-11-02T15:21:02 | Approach | 5.82 | 0.3246 | 0.3246 | True |  | 0.0 | 0.0 | 0.0 | 0.7 | 0 | pre-grasp 0.30 m from the plant, camera 25 deg up at its centre (IK branch chosen for the orbit) |
| 2026-11-02T15:21:08 | Extend | 6.12 | 0.1503 | 0.1503 | True | 1.0 | 0.02 | 0.04 | 0.02 | 0.0 | 0 | Cartesian fraction 1.000 |
| 2026-11-02T15:21:15 | Orbit | 0.17 | 0.0 | 0.427 | False | 0.946 |  |  |  | 0.0 | 0 | 90 deg at 0.15 m in 1/6 segments, 0 wrist frames; stopped in +0 -> -45 deg at fraction 0.95 (0.95 without collision checking: the arm's reach (IK)) |
| 2026-11-02T15:21:14 | Retract | 6.11 | 0.1503 | 0.1503 | True | 1.0 | 0.02 | 0.04 | 0.02 | 0.0 | 0 | Cartesian fraction 1.000 |
| 2026-11-02T15:21:22 | Home | 5.82 | 0.325 | 0.325 | True |  | 0.0 | 0.0 | 0.0 | 2.3 | 0 | back to the start pose (stow) |

## Trial A: Mission Score 61.7 / 100

Collisions 0 (40/40) · completion 54.3 % (21.7/40) · worst Cartesian deviation n/a (0.0/20)

| timestamp | phase | exec_time_s | ee_distance_m | planned_length_m | success | cartesian_fraction | mean_dev_mm | max_dev_mm | dev_pct | end_error_mm | joint_velocity_spikes | notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2026-11-02T15:25:05 | Navigation | 13.5 | 0.0007 | 0.0 | True |  |  |  |  |  | 0 | Nav2 arrived 0.108 m off; precision approach to 5 mm, yaw +0.5 deg |
| 2026-11-02T15:25:28 | Approach | 9.76 | 0.5914 | 0.5914 | True |  | 0.01 | 0.02 | 0.0 | 4.6 | 0 | pre-grasp 0.30 m from the plant, camera 25 deg up at its centre (IK branch chosen for the orbit) |
| 2026-11-02T15:25:38 | Extend | 0.11 | 0.0 | 0.1501 | False | 0.258 |  |  |  | 150.1 | 0 | Cartesian fraction 0.258 (MoveIt error code 1) |
| 2026-11-02T15:25:39 | Orbit | 0.0 | 0.0 | 0.427 | False | 0.0 |  |  |  |  | 0 | skipped: Extend failed |
| 2026-11-02T15:25:40 | Retract | 0.0 | 0.0 | 0.0 | False | 0.0 |  |  |  |  | 0 | skipped: Extend failed |
| 2026-11-02T15:25:40 | Home | 9.78 | 0.5899 | 0.5899 | True |  | 0.01 | 0.02 | 0.0 | 2.0 | 0 | back to the start pose (stow) |
