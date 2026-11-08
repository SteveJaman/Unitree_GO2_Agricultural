Per mode over trials: mean ± SD [min–max]; p from Welch's t-test against `baseline`.

| Metric | `baseline` (n=10) | `traversability` (n=10) | `traversability_semantic` (n=10) | p vs baseline: traversability | p vs baseline: traversability_semantic |
|---|---|---|---|---|---|
| Goals reached (of 3) | 2.70 ± 0.48 [2.00–3.00] | 2.50 ± 0.97 [0.00–3.00] | 2.10 ± 1.20 [0.00–3.00] | 0.570 | 0.168 |
| Total time (s) | 123.12 ± 27.29 [87.00–188.50] | 133.29 ± 21.71 [96.10–179.10] | 149.06 ± 34.38 [95.80–221.40] | 0.369 | 0.079 |
| Distance walked (m) | 18.61 ± 3.83 [12.27–24.02] | 17.20 ± 5.64 [4.27–22.69] | 16.38 ± 5.73 [3.97–24.52] | 0.521 | 0.322 |
| SPL | 0.86 ± 0.17 [0.58–1.00] | 0.79 ± 0.32 [0.00–1.00] | 0.66 ± 0.39 [0.00–0.98] | 0.531 | 0.149 |
| Foot strikes on step-over items | 3.40 ± 3.69 [0.00–11.00] | 2.60 ± 3.24 [0.00–8.00] | 2.60 ± 3.47 [0.00–11.00] | 0.613 | 0.623 |
| Foot strikes, all objects | 5.00 ± 3.74 [0.00–11.00] | 4.50 ± 4.06 [0.00–11.00] | 4.70 ± 4.45 [0.00–11.00] | 0.778 | 0.872 |
| Trunk hits | 0.20 ± 0.42 [0.00–1.00] | 0.30 ± 0.67 [0.00–2.00] | 0.20 ± 0.42 [0.00–1.00] | 0.697 | 1.000 |
| Objects touched | 2.10 ± 1.60 [0.00–5.00] | 1.70 ± 1.42 [0.00–4.00] | 1.70 ± 1.49 [0.00–4.00] | 0.561 | 0.570 |
| Nav2 collision aborts | 0.70 ± 1.89 [0.00–6.00] | 1.90 ± 4.63 [0.00–15.00] | 2.90 ± 4.25 [0.00–14.00] | 0.463 | 0.160 |
| Plans refused: start in lethal space | 0.00 ± 0.00 [0.00–0.00] | 0.00 ± 0.00 [0.00–0.00] | 0.00 ± 0.00 [0.00–0.00] |  |  |
| Goals re-sent | 1.40 ± 2.07 [0.00–5.00] | 2.50 ± 3.98 [0.00–13.00] | 4.40 ± 4.30 [0.00–12.00] | 0.451 | 0.068 |
| Closest to a tall object or wall (m) | 0.09 ± 0.18 [0.00–0.56] | 0.07 ± 0.10 [0.00–0.27] | 0.11 ± 0.17 [0.00–0.50] | 0.807 | 0.761 |
| Sim real-time factor | 0.63 ± 0.02 [0.61–0.66] | 1.11 ± 1.65 [0.54–5.81] | 0.59 ± 0.01 [0.58–0.62] | 0.389 | 0.000 |

Band metrics, robot centre on 0.05 m cells, bands from the true object footprints (step over ≤ 0.12 m, lethal ≥ 0.22 m):

| Metric | `baseline` median [IQR] | `traversability` median [IQR] | `traversability_semantic` median [IQR] |
|---|---|---|---|
| step_over_cells_entered | 2.00 [0.25–2.00] | 0.00 [0.00–1.50] | 0.00 [0.00–1.00] |
| caution_cells_entered | 0.00 [0.00–0.75] | 0.00 [0.00–0.00] | 0.00 [0.00–0.00] |
| lethal_cells_entered | 0.00 [0.00–0.00] | 0.00 [0.00–0.00] | 0.00 [0.00–0.00] |
| time_on_step_over_s | 0.53 [0.07–0.65] | 0.00 [0.00–0.45] | 0.00 [0.00–0.29] |
| time_on_caution_s | 0.00 [0.00–0.09] | 0.00 [0.00–0.00] | 0.00 [0.00–0.00] |
| time_on_lethal_s | 0.00 [0.00–0.00] | 0.00 [0.00–0.00] | 0.00 [0.00–0.00] |
| normal_when_tall_ahead_rate |  | 0.01 [0.00–0.04] | 0.00 [0.00–0.01] |
| normal_when_tall_ahead_count | 0.00 [0.00–0.00] | 3.00 [1.25–9.25] | 1.00 [0.00–4.00] |
| normal_costmap_saw | 0.00 [0.00–0.00] | 0.00 [0.00–0.00] | 0.00 [0.00–0.00] |
| normal_unseen | 0.00 [0.00–0.00] | 3.00 [1.25–9.25] | 1.00 [0.00–4.00] |
| backstop_raised_samples | 0.00 [0.00–0.00] | 197.00 [95.25–220.00] | 231.50 [204.25–289.75] |

`baseline` legs reached: 27/30

`traversability` legs reached: 25/30

`traversability_semantic` legs reached: 21/30

Fisher exact, per leg, `traversability` vs `baseline`: p = 0.706

Fisher exact, per leg, `traversability_semantic` vs `baseline`: p = 0.104

Fusion summary: {"box_observations": 181, "mean_abs_height_err_m": 0.054, "median_abs_height_err_m": 0.012, "mean_xy_err_m": 0.161, "band_correct": 164, "gait_samples": 14576, "gait_match_boxes": 0.51, "gait_match_costmap": 0.499}
