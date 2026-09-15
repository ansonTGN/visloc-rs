# M7am post-M3 exact-ID VIO measurement

This is a release-mode, sensor-only replay of `MH_01_easy` through the current
integrated Basalt adapter after the M3 frontend allocator fix.  The upstream
reference is commit `0f3b2b52c807f70ff4e2973ce253c73329eea7bc` and its frozen
400-frame core trace at
`target/basalt_upstream_mh01_400f_core_trace_20260821T000012Z/trace.jsonl`.

The engine command was run three times with the same pinned inputs:

```text
target/release/examples/basalt_euroc_vio_demo.exe
  --euroc-dir E:\datasets\euroc_mav\machine_hall\MH_01_easy
  --calibration target/euroc_ds_calib.json
  --config target/euroc_config.json
  --out-dir target/basalt_m7_post_m3_run{5,20,80}
  --max-frames {5,20,80}
```

The run is `basalt-compat`, sensor-only, and mapper-off.  The demo has no GT
input or mapper command path.  Ground truth was opened only after all engine
processes exited, by `scripts/evaluate_euroc_trajectory.py` with
`--max-diff-ns 10000000 --tum-time-unit s`.

## Run artifacts

The complete machine-readable comparison is
`target/basalt_m7_post_m3_comparison.json`.

| prefix | frames | observations | IMU delivered | measured release runtime | trajectory SHA-256 |
|---|---:|---:|---:|---:|---|
| `target/basalt_m7_post_m3_run5` | 5 | 1,114 | 42 | 0.952475 s | `a018df64a3292864024dbc6f264413f1996b722cfd279f83df2452d7e16d4bf0` |
| `target/basalt_m7_post_m3_run20` | 20 | 4,724 | 192 | 4.402742 s | `8ae3da96c4075cbdcd1053c12f757d9a59f5343a09ae404586c34737a3e49aa4` |
| `target/basalt_m7_post_m3_run80` | 80 | 24,256 | 792 | 44.385640 s | `635aeeb39c47812a5ac62caaf448529c71305dc12c1c7fcde72d10d17cf9d1b2` |

The timing pass used the already-built release executable and separate output
directories.  Its 5/20/80 trajectory hashes are identical to the primary
replay hashes above.

## Post-exit GT evaluation

The 80-frame evaluator artifact is
`target/basalt_m7_post_m3_run80_evaluation.json` (SHA-256
`a6c2ade41b2970e7e80844bb5d50cb00d7d7df29dc167f4a32d894ecad49497a`).
It associates 58/80 poses (`coverage=0.725`):

| metric | post-M3 |
|---|---:|
| SE(3) ATE RMSE | 0.3054599484 m |
| Sim(3) ATE RMSE (diagnostic) | 0.1237837697 m |
| Sim(3) scale (diagnostic) | 0.1757042834 |
| consecutive SE(3) RPE translation | 0.0210836058 m |
| consecutive Sim(3) RPE translation (diagnostic) | 0.0232064965 m |
| consecutive RPE rotation | 0.0256535146 deg |

The 5- and 20-frame prefixes have zero GT associations: the first EuRoC GT
timestamp is later than frame 19.  Consequently no three-pose SE(3) ATE is
defined for those prefixes; this is recorded explicitly in the comparison
JSON rather than treating a zero-association prefix as a score.

Compared with the previous `post_huber_sqrt80` result (SE(3) ATE
`0.3286738118 m`, Sim(3) scale `0.1571841970`), post-M3 changes are
`-0.0232138634 m` ATE and `+0.0185200863` scale.  Coverage remains 58/80.
RPE translation changes from `0.0225766207 m` to `0.0210836058 m`, diagnostic
Sim(3) RPE from `0.0236958954 m` to `0.0232064965 m`, and rotation from
`0.0256250036 deg` to `0.0256535146 deg`.

## Exact-ID and upstream-core comparison

The M3 first-80 endpoint gate remains exact: upstream and Rust cam0/cam1
counts and ID sets agree for all 80 frames, with zero unknown mismatches and
zero remaps.  The pinned endpoint fixture SHA is
`cee7e2918a8e9383949508f74b6ec956f6c8a2a8e053648d15d05544d2205b2c`; the
post-fix Rust endpoint artifact SHA is
`0af491bbb6ec5dad8d4719a9864a7aabf85bebcd9b7a2d5c00af645dd1a215ee`.

For frames 4/7/9/14/19 below, “pose error” is the Rust post-M3 pose versus the
matching upstream-core-trace pose.  These frames are before the first GT
timestamp, so they are not GT errors.

| frame | translation error (m) | rotation error (deg) | upstream KF/state/pose/landmark | Rust KF/state/pose/landmark |
|---:|---:|---:|---|---|
| 4 | 0.001051540 | 0.711603 | 1/2/1/58 | 1/2/1/58 |
| 7 | 0.003248876 | 0.626787 | 2/2/1/139 | 1/2/1/139 |
| 9 | 0.006670942 | 0.631139 | 2/2/2/114 | 2/2/2/114 |
| 14 | 0.019953913 | 0.722250 | 3/2/2/78 | 2/2/2/78 |
| 19 | 0.037390680 | 0.775713 | 3/2/3/55 | 3/2/3/55 |

The upstream core trace records LM iteration/rejection/lambda but not LM costs;
Rust records the full trial decision/cost trace.  The selected-frame values
are:

| frame | upstream LM (iter/reject/lambda) | Rust LM (iter/reject/lambda) | Rust initial → final cost | Rust decisions |
|---:|---|---|---:|---|
| 4 | 8/0/`1.0e-6` | 8/1/`6.02257e-6` | 4215.973 → 250.710 | A A A R A A A A |
| 7 | 8/3/`6.4e-5` | 8/7/`6.55443` | 378.337 → 334.551 | R R R R A R R R |
| 9 | 7/0/`1.0e-6` | 8/7/`6.83821` | 352.897 → 316.558 | R R R R A R R R |
| 14 | 8/0/`1.0e-6` | 8/6/`0.816769` | 714.064 → 687.095 | R R R A R R R A |
| 19 | 4/0/`1.23457e-6` | 8/6/`1.60581` | 1306.038 → 1250.187 | R R R R A R R A |

At 80 frames both traces make 12 keyframe decisions and 76 optimization/
marginalization attempts.  The upstream final window is 2 states, 7 poses,
124 landmarks; Rust is 2 states, 7 poses, 153 landmarks.  Both report 10 lost
landmarks at frame 79; upstream marginalizes 1 KF + 1 state, while Rust
marginalizes 2 pose records and no all-state record.  These are the remaining
window/landmark structural differences, not an endpoint-ID mismatch.

## M3 delta against prior Rust artifacts

The prior q1 20-frame trace is numerically unchanged at the selected frames
(maximum post-M3 versus q1 translation difference `1.75e-9 m`).  The first
observable M3 lifecycle change is frame 18: q1 pre-M3 rejects ID 723 and adds
83 points (`retained=73`, `rejected=87`); post-M3 retains ID 723 and adds 82
points (`retained=74`, `rejected=86`).  By frame 19 aggregate observation,
pose, and count values are unchanged to numerical precision, although the
corrected ID lifecycle is reflected in the exact endpoint gate.

No production code was changed for this measurement task, and no commit or
push was made.
