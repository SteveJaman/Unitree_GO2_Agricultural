# Trial statistics (analyze.py)

Per drive mode over trials: mean ± SD [min–max]; p from Welch's t-test.

| Metric | `hybrid` (n=3) mean ± SD [min–max] | `omni` (n=3) mean ± SD [min–max] | p (Welch t) |
|---|---|---|---|
| Exploration time (s) | 107 ± 24 [79–124] | 200 ± 48 [150–245] | 0.057 |
| Reported "complete" (share) | 1.00 ± 0.00 [1.00–1.00] | 1.00 ± 0.00 [1.00–1.00] |  |
| Mapped area (m²) | 172 ± 1 [171–172] | 165 ± 5 [159–168] | 0.127 |
| Coverage of the floor (%) | 100 ± 0 [100–100] | 98 ± 3 [95–100] | 0.321 |
| Rooms ≥ 80 % mapped at "complete" (of 7) | 7.0 ± 0.0 [7.0–7.0] | 6.7 ± 0.6 [6.0–7.0] | 0.423 |
| Time to 90 % of the final map (s) | 89 ± 15 [75–104] | 117 ± 30 [100–152] | 0.237 |
| Rooms walked into | 0.7 ± 0.6 [0.0–1.0] | 1.3 ± 0.6 [1.0–2.0] | 0.230 |
| Distance walked (m) | 17.37 ± 4.62 [12.50–21.70] | 28.80 ± 3.97 [25.10–33.00] | 0.032 |
| Time standing still (%) | 10.7 ± 0.6 [10.0–11.0] | 23 ± 16 [5–35] | 0.311 |
| Explorer goals sent | 7.7 ± 1.5 [6.0–9.0] | 11.3 ± 4.0 [7.0–15.0] | 0.253 |
| Nav2 aborts | 0.0 ± 0.0 [0.0–0.0] | 0.3 ± 0.6 [0.0–1.0] | 0.423 |
| Stuck-recovery retries | 0.0 ± 0.0 [0.0–0.0] | 0.3 ± 0.6 [0.0–1.0] | 0.423 |
| Goals dropped: no progress | 0.0 ± 0.0 [0.0–0.0] | 1.7 ± 1.5 [0.0–3.0] | 0.199 |
| Exploration: closest approach (m) | 0.18 ± 0.03 [0.14–0.20] | 0.12 ± 0.02 [0.10–0.13] | 0.065 |
| Exploration: contacts (< 2 cm) | 0.0 ± 0.0 [0.0–0.0] | 0.0 ± 0.0 [0.0–0.0] |  |
| Exploration: time within 10 cm (s) | 0.0 ± 0.0 [0.0–0.0] | 0.0 ± 0.0 [0.0–0.0] |  |
| Tour: goals reached | 7.3 ± 1.2 [6.0–8.0] | 5.0 ± 1.0 [4.0–6.0] | 0.058 |
| Tour: total time (s) | 309 ± 13 [296–322] | 516 ± 77 [429–572] | 0.039 |
| Tour: time per reached goal (s) | 37 ± 4 [33–40] | 57 ± 8 [47–63] | 0.040 |
| Tour: path efficiency | 0.92 ± 0.04 [0.88–0.95] | 0.85 ± 0.08 [0.76–0.92] | 0.321 |
| Tour: SPL | 0.83 ± 0.11 [0.71–0.91] | 0.55 ± 0.13 [0.44–0.69] | 0.043 |
| Tour: Nav2 aborts | 4.0 ± 1.0 [3.0–5.0] | 5.7 ± 1.5 [4.0–7.0] | 0.200 |
| Tour: closest approach (m) | 0.10 ± 0.06 [0.04–0.15] | 0.06 ± 0.10 [-0.06–0.13] | 0.599 |
| Tour: contacts (< 2 cm) | 0.0 ± 0.0 [0.0–0.0] | 0.3 ± 0.6 [0.0–1.0] | 0.423 |
| Tour: time within 10 cm (s) | 2.3 ± 4.0 [0.0–7.0] | 1.0 ± 1.7 [0.0–3.0] | 0.639 |
| Commands mainly forward (%) | 78 ± 4 [74–81] | 43 ± 10 [32–52] | 0.017 |
| Commands mainly reverse (%) | 0.0 ± 0.0 [0.0–0.0] | 29 ± 8 [24–38] | 0.023 |
| Commands mainly sideways (%) | 0.0 ± 0.0 [0.0–0.0] | 28 ± 17 [10–43] | 0.101 |

Per tour leg: reached 22/24 (`hybrid`) vs 15/24 (`omni`), Fisher exact p = 0.036; time per reached leg, median 41 s (n=22) vs 62 s (n=14), Mann-Whitney U p = 0.0074.

| Room: mapped when exploration ended | `hybrid` mean % [min–max] | `omni` mean % [min–max] |
|---|---|---|
| corridor | 100 [100–100] | 100 [100–100] |
| N1 open office | 100 [100–100] | 99 [96–100] |
| N2 lab | 100 [100–100] | 100 [100–100] |
| N3 conference | 100 [100–100] | 100 [100–100] |
| S1 break room | 100 [100–100] | 100 [100–100] |
| S2 storage | 100 [100–100] | 100 [100–100] |
| S3 workshop | 100 [100–100] | 98 [96–100] |
| S4 office | 99 [99–99] | 84 [56–98] |

| Tour goal | `hybrid` reached · mean time | `omni` reached · mean time |
|---|---|---|
| N1 open office | 3/3 · 41 s | 3/3 · 57 s |
| N2 lab | 3/3 · 43 s | 3/3 · 38 s |
| N3 conference | 2/3 · 42 s | 0/3 (stuck, timeout) |
| S4 office | 3/3 · 27 s | 1/3 · 92 s |
| S3 workshop | 2/3 · 41 s | 2/3 · 74 s |
| S2 storage | 3/3 · 54 s | 2/3 · 61 s |
| S1 break room | 3/3 · 39 s | 1/3 · 79 s |
| start | 3/3 · 16 s | 3/3 · 43 s |
