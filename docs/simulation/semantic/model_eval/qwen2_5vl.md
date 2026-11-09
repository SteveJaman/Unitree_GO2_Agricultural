# VLM labeller on saved frames (qwen2.5vl, prompt verify_oneword)

AVOID on true step-over crops: 2 of 14

30 frames, 28 crops of 13 objects, confidence_min 0.7, latency {'median': 2.49, 'p90': 2.59, 'max': 28.75}, errors 1

**VLM alone (below confidence_min counts as UNKNOWN)** (n = 28, accuracy 0.643)

| true \ label | STEP_OVER | CAUTION | AVOID | UNKNOWN |
|---|---|---|---|---|
| STEP_OVER | 11 | 0 | 2 | 1 |
| CAUTION | 1 | 0 | 1 | 3 |
| AVOID | 1 | 0 | 7 | 1 |

unsafe_tall_labelled_low = 1, step_over_when_true_blocked = 1, UNKNOWN = 5

**LiDAR height band alone** (n = 28, accuracy 0.786)

| true \ label | STEP_OVER | CAUTION | AVOID | UNKNOWN |
|---|---|---|---|---|
| STEP_OVER | 11 | 0 | 0 | 3 |
| CAUTION | 1 | 2 | 0 | 2 |
| AVOID | 0 | 0 | 9 | 0 |

unsafe_tall_labelled_low = 0, step_over_when_true_blocked = 0, UNKNOWN = 5

**Combined (published label)** (n = 28, accuracy 0.714)

| true \ label | STEP_OVER | CAUTION | AVOID | UNKNOWN |
|---|---|---|---|---|
| STEP_OVER | 9 | 3 | 2 | 0 |
| CAUTION | 0 | 2 | 1 | 2 |
| AVOID | 0 | 0 | 9 | 0 |

unsafe_tall_labelled_low = 0, step_over_when_true_blocked = 0, UNKNOWN = 2
