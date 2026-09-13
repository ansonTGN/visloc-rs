# M7at f32/window exactness handoff

This bounded replay used the pinned Basalt `0f3b2b52...` binary and the final
Rust release build.  It does not change thresholds, acceptance rules, golden
values, or ground-truth tuning.

## Production state

* The temporary unordered-landmark sidecar is removed.  Production assembly
  keeps the Rust canonical landmark representation; the comparator treats
  worker emission order as unordered and indexes payloads by track ID.
* `DoubleSphereCamera<float>::unproject` is ported with the pinned scalar
  temporaries and left-associative `norm2 = alpha * sqrt2 + 1 - alpha`.
  `camera::tests::raw_f32_unprojection_preserves_pinned_expression_bits`
  locks the observed float bits.
* The f32 visual/IMU pose paths retain the source-proven Sophus SO3 quaternion
  action.  `vio/landmarks.rs` now uses a safe `sophus_packet` implementation
  for the Sophus QuaternionProduct lane/FMA order and packed normalization,
  with a runtime AVX/FMA dispatch and scalar fallback.  A controlled nalgebra
  `transform_vector` build was worse on the same 80-frame replay (SE3 ATE
  0.190220071 m versus 0.188602755 m), so it is not adopted.
* The f32 visual Jacobian accumulation boundary is retained because the pinned
  AOM owns float blocks.  Removing that branch produced byte-identical 80-frame
  metrics in this replay, but is not a reason to discard the source-faithful
  boundary.  No mapper file was changed; concurrent M6 estimator/window hooks
  remain present.

## Native quaternion and triangulation boundary

The authoritative native reference is `target/m7at_se3_probe.cpp`, compiled
against the pinned Basalt/Sophus/Eigen headers.  Quaternion components below
are in Eigen/Sophus `xyzw` order:

* normalized `SinvS` quaternion: `2f9fd648,b1a4b598,30391c34,3f800000`;
* relative-stage translation: `3de1c150,b87a2400,39ac1888`;
* relative-stage quaternion: `3befaa96,39eadb91,3a8bfffa,3f7ffe35`.

The focused Rust exact tests match the standalone `SinvS` quaternion and the
relative-stage tuple above.  The frame-0 trace repeats every relative-stage
bit in `M7_TRI_BITS`:

```text
t=3de1c150,b87a2400,39ac1888
q=3befaa96,39eadb91,3a8bfffa,3f7ffe35
```

Frame-0 track 1 then reaches the DLT boundary with these Rust trace bits:

```text
p0=bf2a7307,be9af9d0,3f2e94d6
p1=bf2d0c0d,be93697d,3f2da937
```

and produces `result=Some(([-0.6658390164375305,-0.3025904893875122,
0.6819804310798645], 0.14654171466827393))`.  The native SE3 probe does not
emit the full DLT/H matrix, so no native H bit claim is made here: `SinvS` and
the relative-stage `t/q` are authoritative native intermediates, while `p0`
and `p1` are the corresponding frame-trace DLT inputs.

Artifacts:

* native source: `target/m7at_se3_probe.cpp`;
* frame 0 trace: `target/m7_simd_frame0_trace/trace.jsonl`;
* frame 0 console capture (including `M7_TRI_BITS`):
  `target/m7_simd_frame0_trace/`.

## Current frame-4 window trace

The current five-frame release replay is in
`target/m7_simd_frame4_trace/trace.jsonl`.  Frame 4 has
`observations=252`, `created=39`, `retained=128`, and `rejected=33`; the window
status is `success` and the LM pass has eight accepted steps and zero rejected
steps.

| iteration | cost before | actual cost | decision |
| ---: | ---: | ---: | :--- |
| 0 | 4215.98095703125 | 412.95391845703125 | Accepted |
| 1 | 412.95391845703125 | 277.4891662597656 | Accepted |
| 2 | 277.4891662597656 | 259.7135009765625 | Accepted |
| 3 | 259.7135009765625 | 253.04356384277344 | Accepted |
| 4 | 253.04356384277344 | 250.47190856933594 | Accepted |
| 5 | 250.47190856933594 | 249.311767578125 | Accepted |
| 6 | 249.311767578125 | 248.7567901611328 | Accepted |
| 7 | 248.7567901611328 | 248.47421264648438 | Accepted |

The current frame-4 window costs are:

* initial: `4215.98095703125`;
* final: `248.47421264648438`;
* accept/reject column: `AAAAAAAA` (8 accepted, 0 rejected).

The authoritative upstream initial cost recorded for this boundary is
`4215.9326171875`; the current Rust initial cost is higher by
`0.04833984375`.  No authoritative upstream final-cost artifact is recorded
for this native-probe comparison, so the final-cost comparison is intentionally
left unspecified.  The prior clean upstream/Rust trace artifacts remain
historical context only:
`target/m7at_final_rust_f4.jsonl`, `target/m7at_clean_upstream_f4.jsonl`,
`target/m7at_clean_upstream_f4b.jsonl`, `target/m7at_final_diff_f4.json`, and
`target/m7at_final_diff_f4b.json`.

## Final 80-frame replay

Artifacts: `target/m7at_final_run80/summary.txt` and
`target/m7at_final_run80/evaluation.json`.  The engine exited normally with
80/80 frames, 36820 IMU samples loaded, 792 delivered, and 24256 observations.
Frame 35 created 22 tracks, had 179 live landmarks, 2 active states and 5
  active poses, with successful writeback.  Frame 79 ended with 2 states, 7
  poses, 153 landmarks, and keyframe 49 scheduled for marginalization.

The post-exit evaluation associated 58/80 poses (`0.725` coverage):

| metric | value |
| --- | ---: |
| SE3 ATE RMSE | 0.1886027552 m |
| Sim3 ATE RMSE | 0.1120028177 m |
| Sim3 scale | 0.3438060089 |
| consecutive SE3 translation RPE | 0.0135585897 m |
| Sim3 translation RPE (diagnostic) | 0.0187862000 m |
| rotation RPE | 0.0297379473 deg |

The historical `.163797` run remains a better metric, but its source state
predates the final camera/order audit and is not used as a fidelity oracle.
The next narrow boundary is the first f32 estimator/window state assembly
(`state.velocity[0]` before solver), then track-1 residual-x; the current trace
shows no control-flow divergence to justify a threshold or lifecycle change.

## Post-SIMD first-80 replay (2026-08-22 JST)

This fresh release replay used the current M7 safe Sophus SIMD path together
with the M6 scalar-f32 integration.  The engine remained GT-free; the ground
truth path was passed only to the evaluator after the engine exited.

Exact engine command (exit status `0`):

```text
& 'C:\Users\rsasa\.cargo\bin\cargo.exe' run --release --example basalt_euroc_vio_demo -- --euroc-dir 'E:\datasets\euroc_mav\machine_hall\MH_01_easy' --calibration 'target\euroc_ds_calib.json' --config 'target\euroc_config.json' --out-dir 'target\m7_postsimd_run80' --max-frames 80
```

The post-exit evaluator also exited `0`:

```text
python scripts\evaluate_euroc_trajectory.py --ground-truth-csv 'E:\datasets\euroc_mav\machine_hall\MH_01_easy\mav0\state_groundtruth_estimate0\data.csv' --trajectory 'target\m7_postsimd_run80\trajectory.tum' --max-diff-ns 10000000 --tum-time-unit s --out-json 'target\m7_postsimd_run80\evaluation.json'
```

Artifacts are `target/m7_postsimd_run80/{summary.txt,evaluation.json,
trace.jsonl,trajectory.csv,trajectory.tum,marg_data/}`.  The SHA-256 values
for the primary comparison artifacts are `evaluation.json`:
`F3B8366414368F2E740E3B098CBABE4A6FE9666B93E3FAE480702FA292CBE3A6`,
`trace.jsonl`:
`C775C693BEF4FDE69853DD9DAFC3B6F982FB53B19BC5469A2DA9ABCC1B497D0F`, and
`trajectory.tum`:
`6BF11CB853B3DBB6BE480A06126F74B917C457AE4BF1096167383EFC08FCA80C`.
The run loaded 36820 IMU samples, delivered 792, emitted 24256
observations, and processed 80/80 frames.

The valid post-exit metrics and deltas against `target/m7at_final_run80` are:

| metric | post-SIMD | m7at final | delta (post-SIMD - final) |
| --- | ---: | ---: | ---: |
| coverage | 58/80 = 0.725 | 58/80 = 0.725 | 0 |
| SE3 ATE RMSE | 0.1906464472 m | 0.1886027552 m | +0.0020436919 m |
| Sim3 ATE RMSE | 0.1123625583 m | 0.1120028177 m | +0.0003597406 m |
| Sim3 scale | 0.3390212174 | 0.3438060089 | -0.0047847915 |
| consecutive SE3 translation RPE | 0.0137077776 m | 0.0135585897 m | +0.0001491878 m |
| Sim3 translation RPE (diagnostic) | 0.0189126139 m | 0.0187862000 m | +0.0001264139 m |
| rotation RPE | 0.0299122524 deg | 0.0297379473 deg | +0.0001743051 deg |

For context, the existing pinned-upstream 80-frame trajectory evidence is
`target/m7perf_final_upstream_80_r1_nomarg/trajectory.csv` (pinned Basalt
run, no `--marg-data`).  Applying the same evaluator association/alignment
helper to its EuRoC-format columns gives `58/80`, SE3 ATE `0.0054666951 m`,
Sim3 ATE `0.0053780410 m`, scale `1.0071956461`, consecutive translation RPE
`0.0018914579 m`, diagnostic Sim3 translation RPE `0.0018720116 m`, and
rotation RPE `0.0491913838 deg`.  Relative to that pinned evidence, the
post-SIMD deltas are respectively `+0.1851797521 m`, `+0.1069845173 m`,
`-0.6681744287`, `+0.0118163197 m`, `+0.0170406023 m`, and
`-0.0192791314 deg`; coverage is unchanged.  The upstream path is retained
as evidence rather than a newly rerun engine artifact in this task.

The post-SIMD frame-4 trace is in
`target/m7_postsimd_run80/trace.jsonl`: window status `success`, LM initial
cost `4215.98095703125`, final cost `248.47421264648438`, eight accepted and
zero rejected steps (`AAAAAAAA`).  The pinned upstream frame-4 trace
`target/m7al_upstream_f4_20260821T000200Z/iteration.jsonl` also records
`AAAAAAAA` and initial cost `4215.9326171875`; the post-SIMD initial-cost
delta is `+0.04833984375`.  No authoritative upstream final cost is recorded.

## Post-M7 FMA first-80 authoritative replay (2026-08-22 JST)

This is a fresh release replay after the M7 f32 IMU/Sophus exactness work.  It
uses the pinned MH_01_easy dataset, `target/euroc_ds_calib.json`, and
`target/euroc_config.json`; its unique output directory is
`target/m7_postm7_fma_run80_20260822_114950/`.

The engine command exited `0`:

```text
& 'C:\Users\rsasa\.cargo\bin\cargo.exe' run --release --example basalt_euroc_vio_demo -- --euroc-dir 'E:\datasets\euroc_mav\machine_hall\MH_01_easy' --calibration 'target\euroc_ds_calib.json' --config 'target\euroc_config.json' --out-dir 'target\m7_postm7_fma_run80_20260822_114950' --max-frames 80
```

The post-exit evaluator exited `0` with the same association protocol:

```text
python scripts\evaluate_euroc_trajectory.py --ground-truth-csv 'E:\datasets\euroc_mav\machine_hall\MH_01_easy\mav0\state_groundtruth_estimate0\data.csv' --trajectory 'target\m7_postm7_fma_run80_20260822_114950\trajectory.tum' --max-diff-ns 10000000 --tum-time-unit s --out-json 'target\m7_postm7_fma_run80_20260822_114950\evaluation.json'
```

The retained artifacts are `summary.txt`, `trace.jsonl`, `trajectory.csv`,
`trajectory.tum`, `evaluation.json`, and `marg_data/` in that directory.  The
summary records `sensor_only=true`, `frames_processed=80`, `imu_samples_loaded=36820`,
`imu_samples_delivered=792`, and `observations_emitted=24256`.  A direct-binary
timing replay in `target/m7_postm7_fma_timing80_20260822_115056/` also exited
`0` in `32.310038` seconds and produced byte-identical trace and TUM
trajectory files.  SHA-256 for the primary replay is:

| artifact | SHA-256 |
| --- | --- |
| `evaluation.json` | `096056629BD4C396A16010A818B38F5D6AC9D40CB2C343C9D42D05BA2D7B38B7` |
| `trace.jsonl` | `21AAF512A60CEB68804D745F4C95A651FD801F8C3C40CA5CC6AEDFAF4CB76859` |
| `trajectory.tum` | `134820620D7DA4D6BCEB2F109FA89D1EACF2BF55A679ED360904380754A8EB2D` |

The primary metrics, the previous Rust post-SIMD run, the older M7 final Rust
run, and the pinned upstream 80-frame trajectory are:

| metric | post-M7 FMA | post-SIMD Rust | M7 final Rust | pinned upstream |
| --- | ---: | ---: | ---: | ---: |
| coverage | 58/80 = 0.725 | 58/80 = 0.725 | 58/80 = 0.725 | 58/80 = 0.725 |
| SE3 ATE RMSE | 0.1923934265 m | 0.1906464472 m | 0.1886027552 m | 0.0054666951 m |
| Sim3 ATE RMSE | 0.1126085188 m | 0.1123625583 m | 0.1120028177 m | 0.0053780410 m |
| Sim3 scale | 0.3351715620 | 0.3390212174 | 0.3438060089 | 1.0071956461 |
| consecutive SE3 translation RPE | 0.0138348630 m | 0.0137077776 m | 0.0135585897 m | 0.0018914579 m |
| Sim3 translation RPE (diagnostic) | 0.0190154951 m | 0.0189126139 m | 0.0187862000 m | 0.0018720116 m |
| rotation RPE | 0.0302234275 deg | 0.0299122524 deg | 0.0297379473 deg | 0.0491913838 deg |

Relative to the prior Rust post-SIMD replay, the post-M7 deltas are coverage
`0`, SE3 ATE `+0.0017469793 m`, Sim3 ATE `+0.0002459605 m`, scale
`-0.0038496554`, SE3 RPE `+0.0001270854 m`, Sim3 RPE `+0.0001028812 m`, and
rotation RPE `+0.0003111751 deg`.  Relative to the pinned upstream trajectory,
the corresponding deltas are `+0.1869267314 m`, `+0.1072304778 m`,
`-0.6720240841`, `+0.0119434051 m`, `+0.0171434835 m`, and
`-0.0189679563 deg`; coverage is again unchanged.

Fresh frame 4 has `observations=252`, `created=39`, `retained=128`, and
`rejected=33`.  Its successful window has initial cost
`4215.86328125`, final cost `248.47415161132812`, and eight accepted / zero
rejected steps (`AAAAAAAA`).  The prior post-SIMD frame-4 costs were
`4215.98095703125` and `248.47421264648438` (also `AAAAAAAA`); the pinned
upstream frame-4 artifact records `4215.9326171875` and `AAAAAAAA`, with no
authoritative upstream final cost.  The first exact state-bit divergence from
the prior Rust post-SIMD trace is frame 1: `qw` bits
`1058565991` vs `1058565992` and `qz` bits `3140134795` vs `3140134796`, each
exactly one ULP lower in the fresh replay.  All frame-1 frontend/control
fields and track-ID arrays remain identical.  The first LM accept/reject
sequence change is frame 15 (`RRRARRRA` fresh versus `RRRRARRA` prior); the
first LM iteration-count change is frame 37 (`RRRRRARA` versus six rejected
iterations), where landmark writeback is `179` fresh versus `0` prior.

Frame 35 itself remains a successful, structurally comparable solve:
`observations=382`, `created=22`, `retained=197`, `rejected=19`, two active
states, five active poses, `landmark_count=179`, and `landmark_writeback=184`.
The fresh frame-35 LM costs are `17351.8984375` to `16828.322265625` with
`RRRRRAAA`; the frontend observation and track-ID arrays are identical to the
prior post-SIMD replay for all 80 frames.  Thus the narrowed boundary is the
source-faithful f32 state/solver arithmetic after the frame-1 quaternion ULP,
not feature lifecycle or a compat fallback; no further production change is
justified by this replay.

No engine command supplied a ground-truth, rescue, or compatibility input, and
the engine summary/trace contain none of those markers.  The only GT mention
is the evaluator's explicit `ground_truth_used_after_engine_exit=true` record
in `evaluation.json`.

## Post-M7 direct IMU/predict boundary audit (2026-08-22 JST)

The fresh direct pinned upstream trace is
`target/m7_postm7_upstream_direct5_20260822_imu_trace.jsonl` (SHA-256
`B158F8AF04856058B63BB0014C646D36339F8A4FFEFF50C281B2EDBEBC7A8309`).  It
was produced by the pinned upstream RelWithDebInfo engine on the sensor-only
MH_01 input.  The matching fresh Rust release replay is
`target/m7_postm7_fma_fix_run5_20260822_1305/`; its frame-4 detail trace is
`target/m7_postm7_fma_fix_detail_20260822.jsonl` (SHA-256
`E19D07B55BC3F24EF1681717DE29C8103CC3023758CBB009AF3A542E68FD75AB`).

The production FMA edit is now exact through the complete frame-0→1
preintegration interval.  The pinned C++ `IntegratedImuMeasurement<float>`
probe and Rust release trace agree on every step's quaternion and velocity,
and on the final delta:

| quantity | pinned upstream bits | Rust post-FMA bits |
| --- | --- | --- |
| `delta_position` | `3c1df76d,ba397d97,bb5ce9dd` | identical |
| `delta_rotation (xyzw)` | `bb322eb8,3ab5f9f8,ba19f851,3f7fffaf` | identical |
| `delta_velocity` | `3ec46faa,bcf45e4c,be0fadf7` | identical |

At this pre-point-action checkpoint, the first remaining direct difference was
after propagation, in the frame-1 `predictState`/writeback arithmetic.  The
quaternion and x/y state lanes were exact, while the z lanes were four ULP low
in Rust: `t.z` was upstream `bb0527fc` versus Rust `bb0527f8`, and `v.z` was
upstream `bda6dcd8` versus Rust `bda6dcd4`.  The authoritative C++ boundary probe records
`R(q0)*delta_p = 39c8bb60,b8cebae8,3c279e64` and
`R(q0)*delta_v = 3cabe7c0,bbc3c000,3ed16b71`, followed by
`p = 39c8bb60,b8cebae8,bb0527fc` and
`v = 3cabe7c0,bbc3c000,bda6dcd8`.  This narrowed the mismatch to the
Eigen/Sophus f32 point-action and packet writeback boundary; the subsequent
point-action closure is documented below.

The exact preintegration fix is the Eigen packet evaluator's fused first
translation sum.  The pinned header expression at `preintegration.h:98-100`
contracts `p + v*dt` before adding the acceleration term.  Rust now spells
that operation as `old_velocity.{x,y,z}.mul_add(dt, position.{x,y,z})` and
then adds `0.5*a*dt*dt`; the diagnostic C++ state probe and Rust candidate
match all ten per-step position records.  The retained audit artifacts are
`benchmarks/basalt/m7aq_predict_state_probe.cpp`,
`target/m7aq_predict_state_probe_O2_20260822.log` (SHA-256
`604AFD6F2919BF84C91F6FBA7C9C15832F0F13BF05B5B2208944459244A81D2E`), and
`benchmarks/basalt/m7aq_predict_boundary_probe.cpp` with
`target/m7aq_predict_boundary_probe_O2_20260822.log` (SHA-256
`34366559C6E47C4B5AC08238DD69CDBE95678D1A28FCBAEC5D908F729E976D59`).

The frame-4 initial-cost decomposition remains dominated by visual factors:

| category | pinned upstream | Rust post-FMA | difference |
| --- | ---: | ---: | ---: |
| visual objective (f32 grouped) | `4215.9326171875` | `4215.86328125` | `+0.0693359375` |
| IMU objective (diagnostic f32 audit) | `0.0000021118818854` | `0.0000161930817381` | `+0.0000140811998527` |
| prior / bias | `0` | `0` | `0` |
| total initial cost | `4215.9326171875` | `4215.86328125` | `+0.0693359375` |

The upstream visual f64 factor sum is `4215.932365298271`; Rust's is
`4215.863408446312`, while f32 accumulation produces the totals above.  The
first material visual-factor difference in the fresh detail comparison is
track `1`, observation `0`, `raw_residual[0]`: upstream
`-0.008089065551757812`, Rust `-0.008056640625` (absolute
`3.24249267578125e-05`).  The fresh Rust frame-4 solve remains structurally
unchanged (`70` factors, `1243` rows, `58` landmarks, `AAAAAAAA`), so the
remaining cost delta is not a compat/GT rescue or a landmark lifecycle
change.

## Post-M7 Sophus point-action closure (2026-08-22 JST)

The retained pinned native boundary probe was compiled from the pinned
Eigen/Sophus tree with `-std=c++17 -O2/-O3 -march=native` and
`-DEIGEN_DONT_PARALLELIZE`. Both optimization levels produced the same
point-action bits:

| fixed point | pinned `SO3<float>::operator*` bits |
| --- | --- |
| `delta_p` | `39c8bb60,b8cebae8,3c279e64` |
| `delta_v` | `3cabe7c0,bbc3c000,3ed16b71` |

The pinned header operation is `uv = q.vec().cross(p); uv += uv; return
p + q.w() * uv + q.vec().cross(uv)`. The authoritative native assembly
spells the three first cross lanes as scalar fused multiply-subtracts, then
does the lane doubling, three fused `q.w * uv + p` operations, and separate
cross additions; the O3 packet path is bit-equivalent. The safe-Rust helper
test in `estimator.rs` matches both fixtures in all six lanes. The production
`sophus_rotate_f32` now uses that explicit `mul_add`/cross-lane order. This
is source-faithful and is guarded by
`sophus_point_action_matches_pinned_fixtures`.

The fresh five-frame release replay is
`target/m7_postm7_point_action_fix_run5_20260822_1415/`, with detail trace
`target/m7_postm7_point_action_fix_detail_20260822.jsonl`. It used the pinned
MH_01_easy/calibration/config, `--max-frames 5`, and `sensor_only=true`; no
compatibility or ground-truth input was supplied. The summary reports
`frames_processed=5`, `imu_samples_delivered=42`, and `observations_emitted=1114`.
Frame 1 is now exact against the pinned state trace in every audited lane:

| state | Rust bits after point-action | pinned bits |
| --- | --- | --- |
| `t=(x,y,z)` | `39c8bb60,b8cebae8,bb0527fc` | identical |
| `q=(w,x,y,z)` | `3f186f67,bd5cdea6,bf4d341f,bb2aa78b` | identical |
| `v=(x,y,z)` | `3cabe7c0,bbc3c000,bda6dcd8` | identical |

Frame 4 remains structurally stable at `70` factors / `1243` rows / `58`
landmarks, with `39` created, `128` retained, `33` rejected observations,
and `61` landmark writebacks. Its initial grouped f32 cost is
`4215.86279296875`, final cost `248.47604370117188`, and LM sequence
`AAAAAAAA` (8 accepted, 0 rejected). The corresponding visual f64 and IMU
diagnostic objectives are `4215.862805724144` and `0.00001619137355767565`.
For comparison, the pinned upstream initial cost is `4215.9326171875`, while
the previous Rust post-FMA replay was `4215.86328125`; the point-action fix
closes frame-1 state bits but does not close the remaining frame-4 solver
delta.

The fresh detail comparison against the previous Rust post-FMA detail
(`target/m7_postm7_fma_upstream_detail_20260822.jsonl`, which is a Rust
post-FMA artifact despite its historical filename) reports the first exact
visual residual change at frame 4, iteration 0 / trial 0 accepted, track 9,
observation 9, `raw_residual[1]`: previous Rust `4.056125640869141`, fresh
Rust `4.056095123291016` (projection y `54.96826171875` versus
`54.968231201171875`). The normalized solver comparator's first material
payload boundary is `global.h[0][5]`: previous Rust `33106.6328125`, fresh
Rust `33106.49609375`; there is no control-flow divergence. These remaining
solver differences are downstream of the now-exact frame-1 point action, so
no further production edit is justified in this bounded phase.

Audit artifacts include the native O2/O3 assembly and logs named
`target/m7aq_predict_boundary_probe_native_O{2,3}_20260822.{s,log}` and the
fresh summary/trace/detail/diff files. The fresh SHA-256 values are:

| artifact | SHA-256 |
| --- | --- |
| run summary | `0993ACA2277143402503ED46B94FFC8F1FBD38368C292AD2C9653A470C737128` |
| run trace | `BC29C42FF757C2BFBCF9F70D098AD60E5FB143512ADAE204217D86E9583E37D8` |
| detail trace | `9EA3AA928BE7EC498E4257AA5933E383FAFB9346485297E61C41430F30794ADA` |
| iteration diff | `AB24D8D4D7072D44E8CD9672FB2D5471134315D00FE18D6BA867D5BCBB145745` |

## Direct fixed-operand visual-factor boundary (2026-08-22 JST)

The retained diagnostic `benchmarks/basalt/m7aq_visual_factor_boundary_probe.cpp`
recreates the pinned `computeRelPose` and `linearizePoint` path for the frame-0
track-1/camera-1 fixture. It was compiled against the pinned Eigen/Sophus tree
with `-O2` and `-O3` (`-march=native`, `-DEIGEN_DONT_PARALLELIZE`); the relative
pose and matrix point bits were identical at both optimization levels. The
probe also calls the actual pinned `basalt::linearizePoint`, not just a duplicate
projection formula. Its fixed source-path result is:

| stage | pinned bits |
| --- | --- |
| stereographic bearing | `bf2a746e,be9aed26,3f2e9644` |
| `T_t_h * [bearing; rho]` point | `bf2fc73d,be94aaaa,3f2ec6b2` |
| Double-Sphere projection | `41da6d22,42d70d2e` |
| raw residual | `bc228000,3f915340` |

The current Rust fixed fixture remains on the f32 `F32Pose` composition plus
direct Sophus point action. It produced projection
`[27.30322265625,107.52577209472656]`, raw residual
`[-0.009983062744140625,1.1353836059570313]`, Huber weight
`0.8807255625724792`, and landmark Jacobian
`[[1474.766845703125,-67.23104858398438,-70.91448211669922],
[-53.62813186645508,1588.6524658203125,13.087924003601074]]`.
Therefore the earliest fixed-operand boundary is the relative point
construction (`T_t_h` homogeneous matrix multiply versus the Rust composed
SE3/direct point-action path), before Double-Sphere projection, Huber weighting,
or QR. The independent artifact comparison still identifies the first raw
factor mismatch at track 1 / observation 0 x (upstream
`-0.008089065551757812`, Rust `-0.008056640625`); the point-action closure's
fresh-vs-previous-Rust change remains track 9 / observation 9 y
(`4.056125640869141` to `4.056095123291016`). The first corresponding
landmark-Jacobian bit difference is track 1 / observation 0 `J[0,0]`:
upstream `1595.60986328125`, Rust `1595.610107421875`.

Huber and whitening follow the pinned scalar formula once raw residuals are
formed; no separate weighting-only mismatch was isolated. The first reduced
system boundary is global `H[0][5]`: pinned upstream `33102.3125`, fresh Rust
`33106.49609375` (the fresh-vs-previous-Rust delta was `-0.13671875`). No
source-faithful Rust matrix-path edit was proven to improve all exact fixture
bits while preserving the now-exact frame-1 state, so no production visual
change was made and no new 5-frame replay was warranted. The temporary Rust
dump test was removed; the C++ probe and its target logs remain as auditable
evidence.

## Post-M7 relative-point matrix/projection closure (2026-08-22 JST)

This bounded follow-up audited the remaining fixed visual boundary without
opening another estimator or scalar boundary.  The pinned native probe was
compiled from the fixed Basalt/Eigen/Sophus tree at both `-O2` and `-O3`
(`-march=native`, `-DEIGEN_DONT_PARALLELIZE`); the two logs agree on every
reported fixed operand.  The relative-pose tests now assert every intermediate
quaternion/translation lane (`target_camera_from_imu`,
`target_imu_from_anchor_imu`, `target_camera_from_anchor_imu`, and the final
camera-to-camera pose), not only the final point.

The first concrete operation difference was Eigen's
`QuaternionBase::toRotationMatrix` expression tree.  The Rust path now has a
small source-faithful helper with the pinned cross-lane `mul_add` operations;
the f32 homogeneous path uses that matrix and performs the literal
`T_t_h * [bearing; rho]` product.  The Double-Sphere path likewise keeps the
native FMA lanes for `d1`, `d2`, `norm`, predicted pixels, and the diagonal
Jacobian terms.  This is production code, not a diagnostic fallback.

The fixed source-path comparison is now:

| stage | pinned O2/O3 bits | Rust focused test |
| --- | --- | --- |
| `T_t_h` rotation/translation lanes | `3f7ffea4,ba71d6ad,3bd0c3e1,00000000,3a87645a,3f7ff61a,bc8e2141,00000000,bbd0358a,3c8e2e4d,3f7ff4ce,00000000,bde1e254,baf1ecc0,ba884364,3f800000` | exact |
| homogeneous point | `bf2fc73d,be94aaaa,3f2ec6b2` | exact |
| Double-Sphere prediction | `41da6d22,42d70d2e` | exact |
| raw residual | `bc228000,3f915340` | exact |
| camera projection Jacobian | `43aa6a6b,c29101bc,c2917182,43f04ca7,439beda6,43037b7f` | exact |

The anchored-factor focused test also reproduces the pinned prediction and raw
residual.  The first remaining landmark-J discrepancy is downstream of this
exact point/projection boundary: the unweighted source landmark-J vector is
`44446eae,c1e49388,c20f4748,445399f4,c21720bd,40df22e2`, while the Rust fixed
matrix product first differs at storage lane 2 as `c20f474c` (+4 ULP).  The
other audited source-J lanes remain unchanged.  Since this phase was restricted
to quaternion composition/inverse/translation/homogeneous and camera lanes,
that residual is recorded rather than expanding the scope into another
stereographic/Jacobian operation.

The retained native logs are
`target/m7aq_visual_factor_boundary_probe_O2_20260822.log` (SHA-256
`D2492700C59BFE363160D1D07A0AD60ECE9E1D4BB560CAD50C97CDFBB2FA03F5`) and
`target/m7aq_visual_factor_boundary_probe_O3_20260822.log` (SHA-256
`043BF70C8A65A60CDCFA8C5A6447BC1615E80019751AD16A11BBAC1BE5EF40D6`).
The temporary Rust dump test was replaced by five deterministic focused tests:
`m7_relative_pose_chain_matches_pinned_lanes`,
`m7_eigen_quaternion_matrix_matches_pinned_lanes`,
`m7_homogeneous_relative_point_matches_pinned_lanes`,
`m7_double_sphere_boundary_matches_pinned_lanes`, and
`m7_anchored_factor_matches_pinned_projection_and_raw`.

A fresh release replay used the pinned MH_01_easy/calibration/config with
`--max-frames 5` and `sensor_only=true`.  Frame 1 remains exact in the audited
pose/velocity/rotation fixture.  Frame 4 remains structurally identical at
`70` factors, `1243` rows, `58` landmarks, and `AAAAAAAA`; its initial total
cost is `4215.86376953125` (grouped visual objective
`4215.86303949356079`, IMU `0.00001619137355767565`, prior/bias zero).  The
first reduced-H entry `[0,5]` is `33107.35546875` (`0x4701535b`) when the
detail-trace f64 value `33107.3566264264155` is cast to f32.  This is compared
with the pinned upstream cost `4215.9326171875` and H entry `33102.3125`; the
source-faithful matrix/projection edit improves the fixed operand evidence but
does not claim full frame-4 solver parity.

Replay artifacts are retained under
`target/m7_relative_matrix_fma_run5_detail_20260822/` and
`target/m7_relative_matrix_fma_detail_20260822.jsonl`; their SHA-256 values are
`6400B80557E79AFC642456DC72B2694F465F6BABE027FBC22E28E265211EADE9`,
`7446987671C7E437564E5077FD094628B41F403A0B39B7C978D784D44154467C`, and
`3A834BE3C615475AA6776B4F636D5AFCCFFED4A11553381A37B028279D5EF5B1` for the
summary, trace, and detail trace respectively.

## Post-M7 landmark-J reduction closure (2026-08-22 JST)

The next fixed-operand audit split the remaining source landmark Jacobian into
its four actual stages.  The pinned O2/O3 probe now retains `source_Jup`,
`source_Jpp`, the camera `Jp`, and the assembled `Jp * Jpp` output.  Both
optimization levels report the same bits:

| stage | pinned bits | Rust focused test |
| --- | --- | --- |
| stereographic `Jup` active lanes | `3f9e8bb7,be4e4fe2,3f8f59cf,be4e4fe2,3fcb92dc,3f024aa3` | exact |
| homogeneous landmark block `Jpp` active lanes | `3f9d9adf,be3b8c06,3f90c8ab,be4fefe0,3fccb288,3ef5c0e4` | exact |
| camera projection `Jp` first 3 columns | `43aa6a6b,c29101bc,c2917182,43f04ca7,439beda6,43037b7f` | exact |
| assembled unweighted landmark J | `44446eae,c1e49388,c20f4748,445399f4,c21720bd,40df22e2` | exact |

The first three rows were already exact before this phase.  The remaining
`c20f474c` Rust lane was the final reduction only: Eigen's O2 assembly for
`generic_dense_assignment_kernel::assignCoeff` loads the `k=2` and `k=0`
products, contracts the `k=3` and `k=1` terms within those pairs, and then
adds the two pair sums.  This is the fixed `2x4 * 4x3` tree from
`linearizePoint`; the fourth row/column are zero, but cannot be replaced by a
left-associated `2x3 * 3x3` nalgebra product at the f32 boundary.

Rust now uses `eigen_landmark_jacobian_f32`, which spells that pairwise tree
with the single pinned `mul_add` in each active pair.  The production factor
uses this helper, and the six fixed M7 tests pass, including the new
`m7_landmark_jacobian_intermediates_match_pinned_lanes`.  The earlier exact
pose, homogeneous point, Double-Sphere prediction/J, and frame-1 tests remain
green.

The fresh release five-frame replay is
`target/m7_landmark_jac_run5_detail_20260822/`, with detail trace
`target/m7_landmark_jac_detail_20260822.jsonl`.  It remains structurally
identical (`70` factors, `1243` rows, `58` landmarks, `AAAAAAAA`) and has
initial total cost `4215.86376953125` (visual grouped objective
`4215.86303949356079`, IMU `0.00001619137355767565`, prior/bias zero) and
final cost `248.4753875732422`.  The reduced-H `[0,5]` f64 entry is
`33107.3558176903680`, whose f32 cast is still `33107.35546875`
(`0x4701535b`); upstream is `33102.3125`.  Thus this exact fixed-J repair
changes the f64 audit row but not its f32 cast or initial objective.

On the full fresh frame-4 ordered factors, the first J mismatch remains track
`1` / observation `0`, `J[0,0]`: upstream `1595.60986328125`
(`0x44c77384`) versus Rust `1595.610107421875` (`0x44c77386`, +2 ULP).  This
is outside the now-exact fixed source fixture (the fresh run's landmark/state
input is already different at that earlier frontend boundary); the fixed
source `Jup -> Jpp -> Jp*Jpp` chain itself is now exact.  The first fresh raw
residual remains the previously recorded track-1/observation-0 difference, so
no further boundary was opened.

Updated audit hashes:

| artifact | SHA-256 |
| --- | --- |
| pinned O2 probe log | `0E04C8B0A520B992C29AC154DD329F37AF151B292FBEEAD447FB8F9ACB471802` |
| pinned O3 probe log | `0E04C8B0A520B992C29AC154DD329F37AF151B292FBEEAD447FB8F9ACB471802` |
| fresh detail trace | `7A9976EAC82465612B080C846FA4FC2037191AFB52B70B84AE4F839FB7498650` |
| fresh run summary | `EE4BCB0E4312366AB5C9DA8CA60830860199AFC0CEE97DB53D6C60819FB197D1` |
| fresh run trace | `045D262EB71C3B001B1301E04086D1B63C8D591E6EEB5394C55208FE5A1A38B7` |

The retained disassembly audits are
`target/m7aq_visual_factor_native_O2_audit_20260822.s` and
`target/m7aq_visual_factor_native_O3_audit_20260822.s`.  The temporary Rust
dump diagnostics remain removed; only deterministic tests and the auditable
native probe outputs are retained.

## Verification

`cargo build --release --example basalt_euroc_vio_demo` passes.  The latest
fixed-J focused tests pass 6/6 in both debug and release; the prior full
`cargo test --release -p visloc-basalt --lib` baseline passed 138 tests (1
ignored) before these six focused additions.  The frame-0 replay used
`--max-frames 1`; the frame-4 replay used `--max-frames 5`.  The
`cargo test --release -p visloc-basalt --tests` command passes every runnable
package test (external/oracle tests remain documented as ignored).

## Pose-J weight-order audit (2026-08-22 JST)

The pinned `computeRelPose`/`linearizePoint` probe was completed at both
`-O2` and `-O3` with `-march=native` and `-DEIGEN_DONT_PARALLELIZE`. The two
source logs are byte-identical:

* `target/m7aq_visual_factor_boundary_probe_pose_O2_20260822.log`;
* `target/m7aq_visual_factor_boundary_probe_pose_O3_20260822.log`;
* SHA-256: `8513F31308373EF6C4B2073D43296BCA8A49EC0A69199D57F75E9113DD8F0497`.

The logs retain complete host/target pose-J lanes, weighted relative pose-J,
and weighted host/target products. Signed-zero behavior is part of the
fixture: `source_d_rel_d_t` contains `80000000` at its column-1/row-5
structural-zero lane, while the host chain uses positive zeros. The raw
un-negated target chain records the complementary signed-zero pattern, so
these lanes cannot be normalized away.

Basalt scales `d_res_d_xi` in place by `sqrt_weight` and then evaluates the
separate fixed `2x6 * 6x6` host/target products. The current production path
forms unweighted relative J, evaluates host/target products, and scales
afterward. The safe reduction variants tried here did not reproduce every
source host/target lane (first signed zeros, then nonzero reduction lanes), so
no production change was made. The temporary ignored pose-J test and debug
prints were removed rather than retained as a non-passing audit. The six
existing fixed M7 tests remain active and pass in release (`6 passed, 0
failed, 0 ignored`); no new five-frame replay was warranted without a proven
production edit, and QR was not opened.

## Exact triangulation/SVD closure (2026-08-22 JST)

The pinned first-track native call was replayed through the actual
`T_0_1`/DLT/`JacobiSVD<Matrix4f>`/`StereographicParam<float>::project` path.
The production-only change is confined to `pipelines/basalt/src/vio/landmarks.rs`:
the f32 Sophus rotation matrix and DLT rows use the proven Eigen expression
trees, and the fixed-size Jacobi rotations use the pinned scalar FMA order for
both left/right updates and the 2x2 helper.  No `aom.rs` or frame-0 helper was
changed.

The focused test
`vio::landmarks::tests::m7_first_track_triangulation_matches_pinned_native_bits`
asserts all DLT `A` lanes, the raw V column, normalized homogeneous result,
stereographic projection, and rho.  Release landmark tests pass `10/10`.
The first-track endpoint is exact:

```text
raw V col3: bf28a750,be994a0a,3f2cbdf9,3e147902
H:          bf2a746e,be9aed26,3f2e9645,3e160ef3
projected:  becaaef7,be383810
direction:  (-0.39586612582206726,-0.1799013614654541)
rho:        0.14654140174388885
```

The fresh five-frame release replay is retained under
`target/m7_triang_exact_fresh5_v2_20260822/` with detail trace
`target/m7_triang_exact_fresh5_detail_v2_20260822.txt`.  It exits normally
with frame 4 at 252 observations, 39 created, 128 retained, 33 rejected;
the window has 70 factors, 1243 rows, and 58 landmarks.  The grouped visual
cost is `4215.863777041435`, IMU cost is `1.6192426468132866e-05`, the LM
initial/final costs are `4215.8642578125` / `248.4746856689453`, and all eight
LM trials are accepted (`AAAAAAAA`).  The current reduced-H entry `H[0,5]`
is `33111.40234375`.

For the all-landmark start comparison, the authoritative source is
`target/m7al_upstream_f4_20260821T000200Z/iteration.jsonl`, filtered to
`iteration=0`, against the Rust frame-4 `iteration_start` snapshot in the
fresh detail trace.  Both sets contain 61 IDs.  Track 1 is exact in all three
f32 fields (direction x/y and rho).  In ascending ID order, the first
difference is track 2:

```text
field          upstream value       upstream bits  Rust value            Rust bits
direction.y   -0.14973284304141998  be195391       -0.14973285794258118   be195392
rho            0.19006994366645813  3e42a1b2       0.19007034599781036    3e42a1cd
```

The exact initial landmark triples are 7/61 (IDs
`1,20,66,82,90,111,128`); the first-ID result is a diagnostic comparison,
not a tuning target.  The full zero-tolerance solver comparison retains 24
snapshots with no structural/control-flow divergence; its first factor
difference is track 1 observation 0 `landmark_jacobian[0][1]`:
upstream `-66.76364135742188` (`c28586fc`) versus Rust
`-66.76361083984375` (`c28586f8`).
