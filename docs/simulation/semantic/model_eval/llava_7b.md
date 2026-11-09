# VLM labeller on saved frames (llava:7b, prompt verify_oneword)

AVOID on true step-over crops: 0 of 14

30 frames, 28 crops of 13 objects, confidence_min 0.7, latency {'median': 0.99, 'p90': 1.03, 'max': 70.73}, errors 0

**VLM alone (below confidence_min counts as UNKNOWN)** (n = 28, accuracy 0.25)

| true \ label | STEP_OVER | CAUTION | AVOID | UNKNOWN |
|---|---|---|---|---|
| STEP_OVER | 0 | 14 | 0 | 0 |
| CAUTION | 0 | 5 | 0 | 0 |
| AVOID | 0 | 7 | 2 | 0 |

unsafe_tall_labelled_low = 7, step_over_when_true_blocked = 0, UNKNOWN = 0

**LiDAR height band alone** (n = 28, accuracy 0.786)

| true \ label | STEP_OVER | CAUTION | AVOID | UNKNOWN |
|---|---|---|---|---|
| STEP_OVER | 11 | 0 | 0 | 3 |
| CAUTION | 1 | 2 | 0 | 2 |
| AVOID | 0 | 0 | 9 | 0 |

unsafe_tall_labelled_low = 0, step_over_when_true_blocked = 0, UNKNOWN = 5

**Combined (published label)** (n = 28, accuracy 0.5)

| true \ label | STEP_OVER | CAUTION | AVOID | UNKNOWN |
|---|---|---|---|---|
| STEP_OVER | 0 | 14 | 0 | 0 |
| CAUTION | 0 | 5 | 0 | 0 |
| AVOID | 0 | 0 | 9 | 0 |

unsafe_tall_labelled_low = 0, step_over_when_true_blocked = 0, UNKNOWN = 0
