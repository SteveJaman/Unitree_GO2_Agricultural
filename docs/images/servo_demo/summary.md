# Distance servo in simulation: results

Target 0.50 m ± 2 cm (camera to the front of the lettuce). Mean [95 % CI] (bootstrap); success with the 95 % Wilson interval; two-sided Mann-Whitney U and Fisher's exact tests.

## Nominal

| Metric | Proportional (n=10) | Fixed step (n=10) | Open loop (n=10) |
|---|---|---|---|
| Success | 10/10 (100%, [72%, 100%]) | 10/10 (100%, [72%, 100%]) | 10/10 (100%, [72%, 100%]) |
| Final error, true (cm) | 2.37 [1.52, 3.22] | 1.76 [1.08, 2.46] | 1.99 [1.86, 2.12] |
| Final error, measured (cm) | 1.25 [1.03, 1.50] | 0.86 [0.44, 1.31] | 0.17 [0.10, 0.25] |
| Arm moves | 3.70 [3.30, 4.10] | 3.80 [3.40, 4.30] | 1.00 [1.00, 1.00] |
| Time to settle (s) | 7.50 [6.64, 8.40] | 8.10 [7.04, 9.20] | 3.02 [2.68, 3.42] |
| Overshoot (cm) | 0.00 [0.00, 0.00] | 0.02 [0.00, 0.06] | 0.30 [0.23, 0.37] |
| Tracker error vs truth (cm) | 2.06 [1.93, 2.19] | 2.03 [1.89, 2.18] | 2.03 [1.63, 2.44] |

### Success rate (settled within the trial time) (nominal)

| Policy | Settled | Rate | 95 % CI (Wilson) |
|---|---|---|---|
| proportional | 10/10 | 100% | [72%, 100%] |
| fixed_step | 10/10 | 100% | [72%, 100%] |
| open_loop | 10/10 | 100% | [72%, 100%] |

### Pairwise tests (two-sided; Mann-Whitney U, Fisher's exact for success) (nominal)

| Comparison | Final error, true | Arm moves | Time to settle | Success |
|---|---|---|---|---|
| proportional vs fixed_step | p = 0.385 | p = 0.837 | p = 0.307 | p = 1 |
| proportional vs open_loop | p = 1 | p = 5.11e-05 | p = 0.000182 | p = 1 |
| fixed_step vs open_loop | p = 0.623 | p = 5.47e-05 | p = 0.000182 | p = 1 |

### By start (nominal)

| Policy | Start | n | Settled | Final error, true (cm) | Arm moves | Time to settle (s) |
|---|---|---|---|---|---|---|
| proportional | far | 5 | 5/5 | 3.73 [3.46, 3.99] | 3.60 [3.00, 4.40] | 7.56 [6.16, 9.36] |
| proportional | near | 5 | 5/5 | 1.00 [0.88, 1.12] | 3.80 [3.40, 4.00] | 7.44 [6.60, 7.96] |
| fixed_step | far | 5 | 5/5 | 2.65 [2.13, 3.42] | 4.40 [4.00, 4.80] | 9.60 [8.72, 10.48] |
| fixed_step | near | 5 | 5/5 | 0.86 [0.40, 1.33] | 3.20 [3.00, 3.60] | 6.60 [6.08, 7.40] |
| open_loop | far | 5 | 5/5 | 1.80 [1.72, 1.89] | 1.00 [1.00, 1.00] | 3.48 [3.08, 3.96] |
| open_loop | near | 5 | 5/5 | 2.18 [2.11, 2.24] | 1.00 [1.00, 1.00] | 2.56 [2.36, 2.72] |

## Disturbed (the dog slides ±6 cm 4 s into each trial)

| Metric | Proportional (n=10) | Fixed step (n=10) | Open loop (n=10) |
|---|---|---|---|
| Success | 7/10 (70%, [40%, 89%]) | 7/10 (70%, [40%, 89%]) | 0/10 (0%, [0%, 28%]) |
| Final error, true (cm) | 2.70 [1.85, 3.51] | 2.18 [1.46, 2.87] | 7.61 [6.44, 8.75] |
| Final error, measured (cm) | 2.09 [1.39, 2.92] | 1.40 [0.71, 2.22] | 6.94 [6.74, 7.15] |
| Arm moves | 4.80 [3.70, 6.00] | 4.80 [3.40, 6.10] | 1.00 [1.00, 1.00] |
| Time to settle (s) | 9.51 [6.43, 12.83] | 8.80 [5.57, 12.26] | – |
| Overshoot (cm) | 1.37 [0.49, 2.30] | 1.24 [0.39, 2.22] | 3.76 [1.59, 5.96] |
| Tracker error vs truth (cm) | 2.12 [1.94, 2.31] | 2.11 [1.95, 2.27] | 2.08 [1.89, 2.26] |

### Success rate (settled within the trial time) (disturbed)

| Policy | Settled | Rate | 95 % CI (Wilson) |
|---|---|---|---|
| proportional | 7/10 | 70% | [40%, 89%] |
| fixed_step | 7/10 | 70% | [40%, 89%] |
| open_loop | 0/10 | 0% | [0%, 28%] |

### Pairwise tests (two-sided; Mann-Whitney U, Fisher's exact for success) (disturbed)

| Comparison | Final error, true | Arm moves | Time to settle | Success |
|---|---|---|---|---|
| proportional vs fixed_step | p = 0.623 | p = 0.939 | p = 0.442 | p = 1 |
| proportional vs open_loop | p = 0.000183 | p = 6.2e-05 | n too small | p = 0.0031 |
| fixed_step vs open_loop | p = 0.000183 | p = 6.11e-05 | n too small | p = 0.0031 |

### Trials that did not settle (disturbed)

| Policy | Start | Disturbance | Cause |
|---|---|---|---|
| fixed_step | near (-7.9 cm) | -6.0 cm | travel limit |
| fixed_step | far (+7.9 cm) | +6.0 cm | travel limit |
| fixed_step | near (-7.6 cm) | -6.0 cm | travel limit |
| open_loop | near (-8.5 cm) | +6.0 cm | no feedback (open loop) |
| open_loop | near (-8.0 cm) | -6.0 cm | no feedback (open loop) |
| open_loop | near (-9.9 cm) | +6.0 cm | no feedback (open loop) |
| open_loop | far (+5.7 cm) | +6.0 cm | no feedback (open loop) |
| open_loop | near (-9.3 cm) | -6.0 cm | no feedback (open loop) |
| open_loop | far (+6.5 cm) | -6.0 cm | no feedback (open loop) |
| open_loop | near (-9.7 cm) | +6.0 cm | no feedback (open loop) |
| open_loop | far (+5.9 cm) | +6.0 cm | no feedback (open loop) |
| open_loop | far (+9.3 cm) | -6.0 cm | no feedback (open loop) |
| open_loop | far (+9.9 cm) | +6.0 cm | no feedback (open loop) |
| proportional | far (+7.7 cm) | +6.0 cm | travel limit |
| proportional | near (-8.7 cm) | -6.0 cm | travel limit |
| proportional | near (-9.8 cm) | -6.0 cm | travel limit |

### By start (disturbed)

| Policy | Start | n | Settled | Final error, true (cm) | Arm moves | Time to settle (s) |
|---|---|---|---|---|---|---|
| proportional | far | 5 | 4/5 | 2.41 [0.92, 3.90] | 5.40 [3.20, 7.40] | 10.80 [5.40, 16.20] |
| proportional | near | 5 | 3/5 | 2.98 [2.52, 3.46] | 4.20 [3.60, 4.80] | 7.80 [6.60, 8.60] |
| fixed_step | far | 5 | 4/5 | 2.29 [1.12, 3.49] | 5.20 [3.00, 7.40] | 10.30 [4.60, 16.00] |
| fixed_step | near | 5 | 3/5 | 2.07 [1.21, 2.81] | 4.40 [3.20, 5.80] | 6.80 [6.20, 7.80] |
| open_loop | far | 5 | 0/5 | 7.55 [6.13, 8.92] | 1.00 [1.00, 1.00] | nan [nan, nan] |
| open_loop | near | 5 | 0/5 | 7.68 [5.98, 9.36] | 1.00 [1.00, 1.00] | nan [nan, nan] |
