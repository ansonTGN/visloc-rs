# ABS_QR dense pipeline comparison

Status: **PASS**

Event: frame 4, iteration 0, trial 0; state DOF 75; visual factors 61.

| Stage | H exact | b exact |
|---|---:|---:|
| visual_total | 5625/5625 | 75/75 |
| imu_total | 5625/5625 | 75/75 |
| prior_before | 5625/5625 | 75/75 |
| prior_after | 5625/5625 | 75/75 |
| final | 5625/5625 | 75/75 |

The verifier compares the serialized f32 bit patterns directly; no decimal parsing or tolerance is used.
