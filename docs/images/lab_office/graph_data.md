# Graph data (analyze.py)

The numbers behind each plot; the same as CSV in [`data/`](data/).

## Mapped area over time (coverage.png)

[`data/coverage_curves.csv`](data/coverage_curves.csv)

| trial | drive | t_s | area_m2 |
|---|---|---|---|
| lab_hybrid1 | hybrid | 1.0 | 30.6 |
| lab_hybrid1 | hybrid | 6.0 | 83.1 |
| lab_hybrid1 | hybrid | 30.1 | 126.2 |
| lab_hybrid1 | hybrid | 35.1 | 135.7 |
| lab_hybrid1 | hybrid | 53.1 | 146.4 |
| lab_hybrid1 | hybrid | 67.1 | 153.1 |
| lab_hybrid1 | hybrid | 87.2 | 170.3 |
| lab_hybrid1 | hybrid | 120.1 | 171.5 |
| lab_hybrid2 | hybrid | 0.5 | 30.6 |
| lab_hybrid2 | hybrid | 7.5 | 78.3 |
| lab_hybrid2 | hybrid | 14.6 | 97.2 |
| lab_hybrid2 | hybrid | 24.6 | 121.0 |
| lab_hybrid2 | hybrid | 55.6 | 147.5 |
| lab_hybrid2 | hybrid | 75.4 | 171.1 |
| lab_hybrid3 | hybrid | 1.0 | 30.6 |
| lab_hybrid3 | hybrid | 30.1 | 124.1 |
| lab_hybrid3 | hybrid | 39.1 | 132.3 |
| lab_hybrid3 | hybrid | 50.1 | 140.5 |
| lab_hybrid3 | hybrid | 72.2 | 145.8 |
| lab_hybrid3 | hybrid | 104.1 | 164.6 |
| lab_hybrid3 | hybrid | 112.9 | 172.2 |
| lab_omni1 | omni | 0.5 | 30.6 |
| lab_omni1 | omni | 30.6 | 122.4 |
| lab_omni1 | omni | 44.6 | 148.3 |
| lab_omni1 | omni | 99.7 | 156.3 |
| lab_omni1 | omni | 131.6 | 161.1 |
| lab_omni1 | omni | 147.7 | 164.0 |
| lab_omni1 | omni | 172.7 | 164.0 |
| lab_omni1 | omni | 194.7 | 164.0 |
| lab_omni1 | omni | 217.7 | 164.0 |
| lab_omni1 | omni | 241.5 | 167.4 |
| lab_omni2 | omni | 0.8 | 30.6 |
| lab_omni2 | omni | 7.9 | 73.4 |
| lab_omni2 | omni | 20.9 | 94.3 |
| lab_omni2 | omni | 32.9 | 110.5 |
| lab_omni2 | omni | 45.9 | 116.1 |
| lab_omni2 | omni | 54.9 | 119.7 |
| lab_omni2 | omni | 79.9 | 135.8 |
| lab_omni2 | omni | 87.0 | 143.2 |
| lab_omni2 | omni | 112.9 | 146.2 |
| lab_omni2 | omni | 135.0 | 146.3 |
| lab_omni2 | omni | 152.0 | 156.6 |
| lab_omni2 | omni | 158.0 | 157.1 |
| lab_omni2 | omni | 164.0 | 157.1 |
| lab_omni2 | omni | 175.0 | 157.6 |
| lab_omni2 | omni | 180.0 | 157.7 |
| lab_omni2 | omni | 201.0 | 167.5 |
| lab_omni3 | omni | 0.1 | 30.6 |
| lab_omni3 | omni | 31.2 | 125.7 |
| lab_omni3 | omni | 37.2 | 129.0 |
| lab_omni3 | omni | 82.3 | 142.4 |
| lab_omni3 | omni | 100.3 | 144.6 |
| lab_omni3 | omni | 146.2 | 159.2 |

## Command mix per drive, % of non-zero commands (commands.png)

[`data/commands.csv`](data/commands.csv)

| drive | forward | turn | reverse | sidestep |
|---|---|---|---|---|
| hybrid | 78.3 | 21.7 | 0.0 | 0.0 |
| omni | 43.0 | 0.0 | 29.0 | 28.0 |

## Explorer goal outcomes per trial (outcomes.png)

[`data/outcomes.csv`](data/outcomes.csv)

| trial | drive | reached | dropped_early | nav2_aborts | no_progress | recovery_retries |
|---|---|---|---|---|---|---|
| lab_hybrid1 | hybrid | 0 | 7 | 0 | 0 | 0 |
| lab_hybrid2 | hybrid | 1 | 5 | 0 | 0 | 0 |
| lab_hybrid3 | hybrid | 0 | 8 | 0 | 0 | 0 |
| lab_omni1 | omni | 4 | 5 | 0 | 3 | 0 |
| lab_omni2 | omni | 5 | 5 | 0 | 2 | 0 |
| lab_omni3 | omni | 4 | 3 | 1 | 0 | 1 |

## Floor known per room when exploration ended, % (room_coverage.png)

[`data/room_coverage.csv`](data/room_coverage.csv)

| trial | drive | corridor | N1 open office | N2 lab | N3 conference | S1 break room | S2 storage | S3 workshop | S4 office |
|---|---|---|---|---|---|---|---|---|---|
| lab_hybrid1 | hybrid | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 100.0 | 98.7 |
| lab_hybrid2 | hybrid | 100.0 | 99.5 | 99.9 | 99.9 | 99.8 | 99.9 | 100.0 | 98.6 |
| lab_hybrid3 | hybrid | 100.0 | 100.0 | 100.0 | 100.0 | 99.9 | 100.0 | 100.0 | 98.9 |
| lab_omni1 | omni | 100.0 | 100.0 | 99.5 | 100.0 | 100.0 | 100.0 | 100.0 | 98.3 |
| lab_omni2 | omni | 100.0 | 99.9 | 100.0 | 100.0 | 99.9 | 99.9 | 96.0 | 98.1 |
| lab_omni3 | omni | 100.0 | 95.9 | 100.0 | 100.0 | 99.9 | 100.0 | 99.0 | 55.7 |

## Every tour leg (tour.png)

[`data/tour_legs.csv`](data/tour_legs.csv)

| trial | drive | goal | result | time_s | path_m | shortest_m | efficiency | spl | nav2_aborts |
|---|---|---|---|---|---|---|---|---|---|
| lab_hybrid1 | hybrid | N1 open office | reached | 17.3 | 4.22 | 4.23 | 1.0 | 1.0 | 0 |
| lab_hybrid1 | hybrid | N2 lab | reached | 42.6 | 8.08 | 7.16 | 0.89 | 0.89 | 1 |
| lab_hybrid1 | hybrid | N3 conference | stuck | 54.4 | 0.77 | 6.62 |  | 0.0 | 1 |
| lab_hybrid1 | hybrid | S4 office | reached | 34.9 | 8.58 | 7.9 | 0.92 | 0.92 | 0 |
| lab_hybrid1 | hybrid | S3 workshop | stuck | 45.1 | 0.0 | 8.16 |  | 0.0 | 1 |
| lab_hybrid1 | hybrid | S2 storage | reached | 50.8 | 11.36 | 10.82 | 0.95 | 0.95 | 0 |
| lab_hybrid1 | hybrid | S1 break room | reached | 34.4 | 7.55 | 6.8 | 0.9 | 0.9 | 0 |
| lab_hybrid1 | hybrid | start | reached | 16.4 | 2.8 | 2.97 | 1.06 | 1.0 | 0 |
| lab_hybrid2 | hybrid | N1 open office | reached | 52.7 | 11.82 | 10.51 | 0.89 | 0.89 | 0 |
| lab_hybrid2 | hybrid | N2 lab | reached | 41.8 | 8.02 | 7.2 | 0.9 | 0.9 | 1 |
| lab_hybrid2 | hybrid | N3 conference | reached | 40.9 | 7.53 | 6.62 | 0.88 | 0.88 | 1 |
| lab_hybrid2 | hybrid | S4 office | reached | 22.3 | 4.5 | 4.29 | 0.95 | 0.95 | 0 |
| lab_hybrid2 | hybrid | S3 workshop | reached | 38.8 | 8.99 | 8.16 | 0.91 | 0.91 | 0 |
| lab_hybrid2 | hybrid | S2 storage | reached | 49.9 | 11.84 | 8.49 | 0.72 | 0.72 | 1 |
| lab_hybrid2 | hybrid | S1 break room | reached | 46.0 | 8.79 | 6.8 | 0.77 | 0.77 | 2 |
| lab_hybrid2 | hybrid | start | reached | 15.4 | 2.89 | 3.04 | 1.05 | 1.0 | 0 |
| lab_hybrid3 | hybrid | N1 open office | reached | 53.2 | 10.84 | 10.3 | 0.95 | 0.95 | 0 |
| lab_hybrid3 | hybrid | N2 lab | reached | 45.2 | 7.68 | 7.2 | 0.94 | 0.94 | 1 |
| lab_hybrid3 | hybrid | N3 conference | reached | 42.8 | 7.36 | 6.62 | 0.9 | 0.9 | 1 |
| lab_hybrid3 | hybrid | S4 office | reached | 24.8 | 4.46 | 4.24 | 0.95 | 0.95 | 0 |
| lab_hybrid3 | hybrid | S3 workshop | reached | 42.3 | 8.91 | 8.16 | 0.92 | 0.92 | 0 |
| lab_hybrid3 | hybrid | S2 storage | reached | 61.1 | 12.09 | 8.49 | 0.7 | 0.7 | 2 |
| lab_hybrid3 | hybrid | S1 break room | reached | 35.4 | 7.44 | 6.85 | 0.92 | 0.92 | 0 |
| lab_hybrid3 | hybrid | start | reached | 17.4 | 2.77 | 2.9 | 1.05 | 1.0 | 0 |
| lab_omni1 | omni | N1 open office | reached | 0.2 | 0.0 | 0.0 |  | 1.0 | 0 |
| lab_omni1 | omni | N2 lab | reached | 40.3 | 9.74 | 7.36 | 0.76 | 0.76 | 0 |
| lab_omni1 | omni | N3 conference | stuck | 57.4 | 1.97 | 6.62 |  | 0.0 | 1 |
| lab_omni1 | omni | S4 office | stuck | 61.3 | 1.43 | 6.77 |  | 0.0 | 0 |
| lab_omni1 | omni | S3 workshop | reached | 86.7 | 6.46 | 5.87 | 0.91 | 0.91 | 2 |
| lab_omni1 | omni | S2 storage | reached | 44.0 | 9.62 | 8.54 | 0.89 | 0.89 | 0 |
| lab_omni1 | omni | S1 break room | timeout | 75.2 | 6.87 | 6.64 |  | 0.0 | 1 |
| lab_omni1 | omni | start | reached | 63.7 | 14.24 | 7.13 | 0.5 | 0.5 | 0 |
| lab_omni2 | omni | N1 open office | reached | 75.5 | 11.82 | 10.91 | 0.92 | 0.92 | 0 |
| lab_omni2 | omni | N2 lab | reached | 40.7 | 9.69 | 7.25 | 0.75 | 0.75 | 0 |
| lab_omni2 | omni | N3 conference | timeout | 82.0 | 2.75 | 6.62 |  | 0.0 | 1 |
| lab_omni2 | omni | S4 office | timeout | 91.7 | 5.55 | 7.05 |  | 0.0 | 3 |
| lab_omni2 | omni | S3 workshop | timeout | 82.2 | 10.46 | 5.88 |  | 0.0 | 0 |
| lab_omni2 | omni | S2 storage | reached | 77.6 | 8.22 | 7.37 | 0.9 | 0.9 | 1 |
| lab_omni2 | omni | S1 break room | timeout | 75.5 | 5.12 | 6.64 |  | 0.0 | 1 |
| lab_omni2 | omni | start | reached | 47.2 | 3.32 | 3.2 | 0.97 | 0.97 | 1 |
| lab_omni3 | omni | N1 open office | reached | 94.8 | 10.53 | 9.21 | 0.88 | 0.88 | 1 |
| lab_omni3 | omni | N2 lab | reached | 31.9 | 7.49 | 7.2 | 0.96 | 0.96 | 0 |
| lab_omni3 | omni | N3 conference | timeout | 82.2 | 3.16 | 6.62 |  | 0.0 | 1 |
| lab_omni3 | omni | S4 office | reached | 91.6 | 8.26 | 6.87 | 0.83 | 0.83 | 2 |
| lab_omni3 | omni | S3 workshop | reached | 61.0 | 9.21 | 8.16 | 0.89 | 0.89 | 0 |
| lab_omni3 | omni | S2 storage | timeout | 89.5 | 6.47 | 8.49 |  | 0.0 | 1 |
| lab_omni3 | omni | S1 break room | reached | 78.6 | 11.7 | 11.0 | 0.94 | 0.94 | 1 |
| lab_omni3 | omni | start | reached | 18.1 | 2.84 | 2.88 | 1.02 | 1.0 | 0 |

## Distance from the body to the nearest wall or furniture, m (clearance.png)

[`data/clearance.csv`](data/clearance.csv)

| drive | phase | samples | min | p1 | p5 | p10 | p25 | p50 | p75 | p90 | pct_below_0.10m | pct_below_0.25m |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hybrid | explore | 1602 | 0.135 | 0.195 | 0.209 | 0.26 | 0.372 | 0.451 | 0.555 | 0.647 | 0.0 | 8.7 |
| hybrid | tour | 4863 | 0.044 | 0.111 | 0.163 | 0.165 | 0.296 | 0.394 | 0.498 | 0.554 | 0.72 | 17.7 |
| omni | explore | 3004 | 0.102 | 0.143 | 0.22 | 0.259 | 0.291 | 0.384 | 0.514 | 0.579 | 0.0 | 9.1 |
| omni | tour | 7979 | -0.058 | 0.168 | 0.221 | 0.243 | 0.307 | 0.382 | 0.421 | 0.522 | 0.16 | 10.9 |
