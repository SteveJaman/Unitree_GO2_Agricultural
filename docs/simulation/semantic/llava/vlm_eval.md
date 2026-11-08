# VLM labeller on saved frames (llava:7b)

30 frames, 28 crops of 13 objects, confidence_min 0.7, latency {'median': 1.76, 'p90': 1.83, 'max': 17.57}, errors 0

**VLM alone (below confidence_min counts as UNKNOWN)** (n = 28, accuracy 0.393)

| true \ label | STEP_OVER | CAUTION | AVOID | UNKNOWN |
|---|---|---|---|---|
| STEP_OVER | 0 | 16 | 0 | 1 |
| CAUTION | 0 | 2 | 0 | 0 |
| AVOID | 0 | 0 | 9 | 0 |

unsafe_tall_labelled_low = 0, step_over_when_true_blocked = 0, UNKNOWN = 1

**LiDAR height band alone** (n = 28, accuracy 0.786)

| true \ label | STEP_OVER | CAUTION | AVOID | UNKNOWN |
|---|---|---|---|---|
| STEP_OVER | 13 | 0 | 0 | 4 |
| CAUTION | 1 | 0 | 0 | 1 |
| AVOID | 0 | 0 | 9 | 0 |

unsafe_tall_labelled_low = 0, step_over_when_true_blocked = 0, UNKNOWN = 5

**Combined (published label)** (n = 28, accuracy 0.357)

| true \ label | STEP_OVER | CAUTION | AVOID | UNKNOWN |
|---|---|---|---|---|
| STEP_OVER | 0 | 13 | 0 | 4 |
| CAUTION | 0 | 1 | 0 | 1 |
| AVOID | 0 | 0 | 9 | 0 |

unsafe_tall_labelled_low = 0, step_over_when_true_blocked = 0, UNKNOWN = 5
