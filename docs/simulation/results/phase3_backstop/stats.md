Per mode over trials: mean ± SD [min–max]; p from Welch's t-test against `traversability`.

| Metric | `traversability` (n=3) | `traversability_nobackstop` (n=3) | p vs traversability: traversability_nobackstop |
|---|---|---|---|
| Goals reached (of 3) | 3.00 ± 0.00 [3.00–3.00] | 2.00 ± 1.73 [0.00–3.00] | 0.423 |
| Total time (s) | 127.77 ± 5.46 [121.60–132.00] | 140.87 ± 27.04 [121.20–171.70] | 0.492 |
| Distance walked (m) | 19.93 ± 0.24 [19.68–20.16] | 14.56 ± 9.21 [3.92–20.00] | 0.419 |
| SPL | 1.00 ± 0.00 [0.99–1.00] | 0.67 ± 0.58 [0.00–1.00] | 0.423 |
| Foot strikes on step-over items | 9.00 ± 1.73 [7.00–10.00] | 6.67 ± 3.51 [3.00–10.00] | 0.380 |
| Foot strikes, all objects | 16.00 ± 1.73 [15.00–18.00] | 12.67 ± 8.74 [3.00–20.00] | 0.579 |
| Trunk hits | 0.00 ± 0.00 [0.00–0.00] | 0.00 ± 0.00 [0.00–0.00] |  |
| Objects touched | 5.00 ± 0.00 [5.00–5.00] | 4.33 ± 2.08 [2.00–6.00] | 0.635 |
| Nav2 collision aborts | 0.67 ± 0.58 [0.00–1.00] | 1.00 ± 1.00 [0.00–2.00] | 0.649 |
| Plans refused: start in lethal space | 0.00 ± 0.00 [0.00–0.00] | 4.67 ± 8.08 [0.00–14.00] | 0.423 |
| Goals re-sent | 0.67 ± 0.58 [0.00–1.00] | 4.67 ± 7.23 [0.00–13.00] | 0.439 |
| Closest to a tall object or wall (m) | 0.00 ± 0.00 [0.00–0.00] | 0.05 ± 0.09 [0.00–0.15] | 0.423 |
| Sim real-time factor | 0.57 ± 0.00 [0.56–0.57] | 0.55 ± 0.03 [0.51–0.57] | 0.422 |

Band metrics, robot centre on 0.05 m cells, bands from the true object footprints (step over ≤ 0.12 m, lethal ≥ 0.22 m):

| Metric | `traversability` median [IQR] | `traversability_nobackstop` median [IQR] |
|---|---|---|
| step_over_cells_entered | 7.00 [6.00–8.00] | 9.00 [6.00–10.00] |
| caution_cells_entered | 9.00 [8.50–9.00] | 8.00 [4.00–9.00] |
| lethal_cells_entered | 0.00 [0.00–0.00] | 0.00 [0.00–0.00] |
| time_on_step_over_s | 2.48 [1.87–2.56] | 2.21 [1.50–2.54] |
| time_on_caution_s | 2.26 [2.22–2.33] | 2.04 [1.02–2.39] |
| time_on_lethal_s | 0.00 [0.00–0.00] | 0.00 [0.00–0.00] |
| normal_when_tall_ahead_rate | 0.01 [0.01–0.01] | 0.19 [0.10–0.24] |
| normal_when_tall_ahead_count | 4.00 [3.50–5.50] | 86.00 [43.00–114.50] |
| normal_costmap_saw | 0.00 [0.00–0.00] | 77.00 [38.50–108.50] |
| normal_unseen | 4.00 [3.50–5.50] | 3.00 [1.50–6.00] |
| backstop_raised_samples | 232.00 [224.50–254.00] | 0.00 [0.00–0.00] |

`traversability` legs reached: 9/9

`traversability_nobackstop` legs reached: 6/9

Fisher exact, per leg, `traversability_nobackstop` vs `traversability`: p = 0.206

Fusion summary: {"box_observations": 73, "mean_abs_height_err_m": 0.067, "median_abs_height_err_m": 0.013, "mean_xy_err_m": 0.151, "band_correct": 67, "gait_samples": 4168, "gait_match_boxes": 0.7, "gait_match_costmap": 0.505}
