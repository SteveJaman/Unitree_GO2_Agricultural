Safety metrics per mode (medians over trials, [IQR]). Navigation efficiency is not the metric of interest; goals and path are for completeness.

| Metric | `baseline` | `traversability` | `traversability_semantic` |
|---|---|---|---|
| Unsafe published labels, LiDAR-measured (semantic log) | n/a (no semantic layer) | n/a (no semantic layer) | 0 |
| Foot strikes per metre walked, median [IQR] | 0.24 [0.16–0.35] | 0.23 [0.04–0.40] | 0.24 [0.03–0.43] |
| Foot strikes on step-over items, median [IQR] | 2.00 [0.50–4.75] | 1.50 [0.00–4.25] | 1.50 [0.00–3.75] |
| Trunk hits, median [IQR] | 0.00 [0.00–0.00] | 0.00 [0.00–0.00] | 0.00 [0.00–0.00] |
| Backstop raised the gait (samples), median [IQR]; not compared (baseline has no supervisor) | 0.00 [0.00–0.00] | 197.00 [95.25–220.00] | 231.50 [204.25–289.75] |
| Foot clearance on items ≤ 0.10 m (passes cleared) | 15/58 | 25/47 | 15/37 |
| NORMAL gait when care was needed (rate), median [IQR] |  | 0.01 [0.00–0.04] | 0.00 [0.00–0.01] |
| NORMAL while the costmap saw it (backstop failure; target 0 with backstop on), total samples | 0 of 0 | 0 of 2359 | 0 of 3534 |
| NORMAL, unseen by costmap and camera (perception gap), total samples | 0 of 0 | 86 of 2359 | 23 of 3534 |
| RPP collision aborts, median [IQR] (known limitation, not a failure) | 0.00 [0.00–0.00] | 0.50 [0.00–1.00] | 1.50 [1.00–2.00] |
| Goals reached (completeness) | 27/30 | 25/30 | 21/30 |
| Path walked per trial, m, median [IQR] (completeness) | 19.70 [15.91–21.19] | 18.66 [15.36–21.32] | 16.09 [14.07–19.87] |

Pairwise tests. Significant only with p < 0.05 and an effect above the small threshold (Cohen's h > 0.2 for proportions, r > 0.1 for Mann-Whitney).

| Comparison | Metric | Test | p | Effect | 95% CI of difference | Significant |
|---|---|---|---|---|---|---|
| `traversability` vs `baseline` | Goals reached (per leg) | two-proportion z (cc) | 0.704 | h = -0.20 | -0.24 to +0.10 | no |
| `traversability` vs `baseline` | Foot clearance ≤ 0.10 m (per pass) | two-proportion z (cc) | 0.008 | h = +0.57 | +0.09 to +0.46 | yes |
| `traversability` vs `baseline` | Foot strikes per metre | Mann-Whitney U | 0.761 | r = 0.08 | -0.27 to +0.24 | no |
| `traversability` vs `baseline` | Path walked (m) | Mann-Whitney U | 0.821 | r = 0.06 | -5.48 to +4.52 | no |
| `traversability` vs `baseline` | Step-over cells entered | Mann-Whitney U | 0.290 | r = 0.23 | -2.50 to +1.50 | no |
| `traversability_semantic` vs `baseline` | Goals reached (per leg) | two-proportion z (cc) | 0.107 | h = -0.52 | -0.40 to -0.00 | no |
| `traversability_semantic` vs `baseline` | Foot clearance ≤ 0.10 m (per pass) | two-proportion z (cc) | 0.202 | h = +0.31 | -0.05 to +0.34 | no |
| `traversability_semantic` vs `baseline` | Foot strikes per metre | Mann-Whitney U | 0.849 | r = 0.05 | -0.27 to +0.26 | no |
| `traversability_semantic` vs `baseline` | Path walked (m) | Mann-Whitney U | 0.473 | r = 0.17 | -6.69 to +2.19 | no |
| `traversability_semantic` vs `baseline` | Step-over cells entered | Mann-Whitney U | 0.232 | r = 0.26 | -2.00 to +0.50 | no |
| `traversability_semantic` vs `traversability` | Goals reached (per leg) | two-proportion z (cc) | 0.360 | h = -0.32 | -0.34 to +0.08 | no |
| `traversability_semantic` vs `traversability` | Foot clearance ≤ 0.10 m (per pass) | two-proportion z (cc) | 0.351 | h = -0.25 | -0.34 to +0.09 | no |
| `traversability_semantic` vs `traversability` | Foot strikes per metre | Mann-Whitney U | 1.000 | r = 0.01 | -0.31 to +0.34 | no |
| `traversability_semantic` vs `traversability` | Path walked (m) | Mann-Whitney U | 0.623 | r = 0.12 | -6.39 to +3.41 | no |
| `traversability_semantic` vs `traversability` | Step-over cells entered | Mann-Whitney U | 0.929 | r = 0.03 | -2.50 to +1.01 | no |

