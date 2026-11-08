Per mode over trials: mean ± SD [min–max]; p from Welch's t-test against `baseline`.

| Metric | `baseline` (n=5) | `traversability` (n=5) | `traversability_semantic` (n=5) | p vs baseline: traversability | p vs baseline: traversability_semantic |
|---|---|---|---|---|---|
| Goals reached (of 3) | 2.80 ± 0.45 [2.00–3.00] | 1.80 ± 1.30 [0.00–3.00] | 2.40 ± 0.89 [1.00–3.00] | 0.167 | 0.406 |
| Total time (s) | 118.42 ± 19.99 [89.10–141.10] | 152.30 ± 68.34 [95.20–270.00] | 130.60 ± 22.86 [97.70–160.10] | 0.339 | 0.397 |
| Distance walked (m) | 18.58 ± 4.57 [12.62–24.22] | 12.58 ± 5.49 [5.89–19.84] | 15.67 ± 5.28 [8.57–21.47] | 0.098 | 0.378 |
| SPL | 0.88 ± 0.15 [0.62–1.00] | 0.57 ± 0.41 [0.00–0.99] | 0.75 ± 0.29 [0.30–0.97] | 0.164 | 0.404 |
| Foot strikes on step-over items | 3.80 ± 4.15 [0.00–10.00] | 2.60 ± 4.77 [0.00–11.00] | 3.40 ± 4.10 [0.00–10.00] | 0.683 | 0.882 |
| Foot strikes, all objects | 5.00 ± 5.29 [0.00–11.00] | 4.40 ± 5.22 [0.00–11.00] | 5.20 ± 4.21 [0.00–10.00] | 0.861 | 0.949 |
| Trunk hits | 0.60 ± 0.89 [0.00–2.00] | 0.20 ± 0.45 [0.00–1.00] | 1.00 ± 2.24 [0.00–5.00] | 0.406 | 0.725 |
| Objects touched | 1.80 ± 1.79 [0.00–4.00] | 1.20 ± 1.64 [0.00–4.00] | 1.80 ± 1.92 [0.00–5.00] | 0.596 | 1.000 |
| Nav2 collision aborts | 0.20 ± 0.45 [0.00–1.00] | 5.00 ± 10.63 [0.00–24.00] | 2.60 ± 4.72 [0.00–11.00] | 0.370 | 0.320 |
| Plans refused: start in lethal space | 0.00 ± 0.00 [0.00–0.00] | 1.80 ± 4.02 [0.00–9.00] | 0.00 ± 0.00 [0.00–0.00] | 0.374 |  |
| Goals re-sent | 1.00 ± 1.73 [0.00–4.00] | 6.60 ± 8.71 [0.00–21.00] | 3.00 ± 3.94 [0.00–9.00] | 0.226 | 0.342 |
| Closest to a tall object or wall (m) | 0.21 ± 0.23 [0.00–0.56] | 0.13 ± 0.09 [0.00–0.21] | 0.18 ± 0.12 [0.00–0.31] | 0.486 | 0.786 |
| Sim real-time factor | 0.63 ± 0.01 [0.61–0.64] | 0.59 ± 0.02 [0.57–0.61] | 0.59 ± 0.02 [0.57–0.61] | 0.008 | 0.005 |

Band metrics, robot centre on 0.05 m cells, bands from the true object footprints (step over ≤ 0.15 m, lethal ≥ 0.25 m):

| Metric | `baseline` median [IQR] | `traversability` median [IQR] | `traversability_semantic` median [IQR] |
|---|---|---|---|
| step_over_cells_entered | 4.00 [0.00–5.00] | 0.00 [0.00–1.00] | 0.00 [0.00–0.00] |
| caution_cells_entered | 0.00 [0.00–0.00] | 0.00 [0.00–0.00] | 0.00 [0.00–0.00] |
| lethal_cells_entered | 0.00 [0.00–0.00] | 0.00 [0.00–0.00] | 0.00 [0.00–0.00] |
| time_on_step_over_s | 0.70 [0.00–1.08] | 0.00 [0.00–0.06] | 0.00 [0.00–0.00] |
| time_on_caution_s | 0.00 [0.00–0.00] | 0.00 [0.00–0.00] | 0.00 [0.00–0.00] |
| time_on_lethal_s | 0.00 [0.00–0.00] | 0.00 [0.00–0.00] | 0.00 [0.00–0.00] |

`baseline` legs reached: 14/15

`traversability` legs reached: 9/15

`traversability_semantic` legs reached: 12/15

Fisher exact, per leg, `traversability` vs `baseline`: p = 0.080

Fisher exact, per leg, `traversability_semantic` vs `baseline`: p = 0.598

Fusion summary: {"box_observations": 69, "mean_abs_height_err_m": 0.073, "median_abs_height_err_m": 0.017, "mean_xy_err_m": 0.17, "band_correct": 69, "gait_samples": 7214, "gait_match_boxes": 0.357, "gait_match_costmap": 0.326}
