**Published label (VLM combined with LiDAR height)**: 58 trial-objects, accuracy 0.638, unsafe_tall_labelled_low 0, step_over_when_true_blocked 0

| true \ label | STEP_OVER | CAUTION | AVOID | UNKNOWN |
|---|---|---|---|---|
| STEP_OVER | 20 | 0 | 0 | 7 |
| CAUTION | 0 | 0 | 0 | 3 |
| AVOID | 0 | 0 | 17 | 3 |
| NO_OBJECT | 0 | 1 | 2 | 5 |

**Raw VLM answer**: 45 trial-objects, accuracy 0.733, unsafe_tall_labelled_low 5, step_over_when_true_blocked 2

| true \ label | STEP_OVER | CAUTION | AVOID | UNKNOWN |
|---|---|---|---|---|
| STEP_OVER | 21 | 1 | 0 | 0 |
| CAUTION | 0 | 2 | 0 | 0 |
| AVOID | 2 | 3 | 10 | 1 |
| NO_OBJECT | 0 | 3 | 2 | 0 |

