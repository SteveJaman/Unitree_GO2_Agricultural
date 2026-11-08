Safety metrics per mode (medians over trials, [IQR]). Navigation efficiency is not the metric of interest; goals and path are for completeness.

| Metric | `traversability` | `traversability_nobackstop` |
|---|---|---|
| Unsafe published labels, LiDAR-measured (semantic log) | n/a (no semantic layer) | n/a (no semantic layer) |
| Foot strikes per metre walked, median [IQR] | 0.76 [0.75–0.83] | 0.77 [0.76–0.88] |
| Foot strikes on step-over items, median [IQR] | 10.00 [8.50–10.00] | 7.00 [5.00–8.50] |
| Trunk hits, median [IQR] | 0.00 [0.00–0.00] | 0.00 [0.00–0.00] |
| Backstop raised the gait (samples), median [IQR] | 232.00 [224.50–254.00] | 0.00 [0.00–0.00] |
| Foot clearance on items ≤ 0.10 m (passes cleared) | 36/83 | 39/68 |
| NORMAL gait when care was needed (rate), median [IQR] | 0.01 [0.01–0.01] | 0.19 [0.10–0.24] |
| NORMAL while the costmap saw it (backstop failure; target 0 with backstop on), total samples | 0 of 1465 | 217 of 1822 |
| NORMAL, unseen by costmap and camera (perception gap), total samples | 14 of 1465 | 12 of 1822 |
| RPP collision aborts, median [IQR] (known limitation, not a failure) | 1.00 [0.50–1.00] | 1.00 [0.50–1.50] |
| Goals reached (completeness) | 9/9 | 6/9 |
| Path walked per trial, m, median [IQR] (completeness) | 19.95 [19.81–20.05] | 19.75 [11.84–19.88] |

Pairwise tests. Significant only with p < 0.05 and an effect above the small threshold (Cohen's h > 0.2 for proportions, r > 0.1 for Mann-Whitney).

| Comparison | Metric | Test | p | Effect | 95% CI of difference | Significant |
|---|---|---|---|---|---|---|
| `traversability_nobackstop` vs `traversability` | Goals reached (per leg) | two-proportion z (cc) | 0.206 | h = -1.23 | -0.64 to -0.03 | no |
| `traversability_nobackstop` vs `traversability` | Foot clearance ≤ 0.10 m (per pass) | two-proportion z (cc) | 0.122 | h = +0.28 | -0.02 to +0.30 | no |
| `traversability_nobackstop` vs `traversability` | Foot strikes per metre | Mann-Whitney U | 0.700 | r = 0.27 | -0.14 to +0.26 | no |
| `traversability_nobackstop` vs `traversability` | Backstop samples | Mann-Whitney U | 0.064 | r = 0.80 | -276.00 to -217.00 | no |
| `traversability_nobackstop` vs `traversability` | Path walked (m) | Mann-Whitney U | 0.700 | r = 0.27 | -16.24 to +0.32 | no |
| `traversability_nobackstop` vs `traversability` | Step-over cells entered | Mann-Whitney U | 0.825 | r = 0.18 | -6.00 to +6.00 | no |

