# M7ar MH01 staged runtime (80 frames)

This is the post-M7aq Rust release replay with the frozen MH01_easy inputs.
The engine was GT-free: only the EuRoC image/IMU directory, calibration, and
config were passed to `basalt_euroc_vio_demo`; GT was opened afterward by the
parity evaluator.

Command:

```text
cargo run --release --example basalt_euroc_vio_demo -- \
  --euroc-dir E:\\datasets\\euroc_mav\\machine_hall\\MH_01_easy \
  --calibration target/euroc_ds_calib.json \
  --config target/euroc_config.json \
  --out-dir target/basalt_m7ar_run80 --max-frames 80
```

Artifacts: `target/basalt_m7ar_run80/trace.jsonl`, `trajectory.csv`,
`trajectory.tum`, and `evaluation.json`.

## Runtime and evaluation

The run processed 80/80 frames, delivered 792 IMU samples, emitted 24,256
observations, and produced 80 trajectory rows. Peak RSS and wall time were not
captured by this direct demo invocation.

| metric | M7ar 80f | pre-fix 80f |
|---|---:|---:|
| GT coverage | 58/80 = 0.725 | 0.725 |
| SE(3) ATE RMSE (m) | 0.1637971122 | 0.30546 |
| Sim(3) ATE RMSE (m, diagnostic) | 0.1067870394 | 0.12378 |
| Sim(3) scale (diagnostic) | 0.4102085111 | 0.1757 |
| consecutive SE(3) RPE translation (m) | 0.0116186798 | — |
| consecutive Sim(3) RPE translation (m) | 0.0170141121 | — |
| consecutive RPE rotation (deg) | 0.0265297583 | — |

The first state-translation divergence from the pinned upstream core trace is
frame 4 (Euclidean translation difference `8.2467058e-6 m`); frames 0–3 are
within `1e-8 m`. The first structural divergence is frame 35: both make the
same keyframe decision and have 2 states, but Rust has 179 landmarks where
upstream has 191. At frame 79 Rust has 2 states, 7 poses, and 151 landmarks;
upstream has 2 states, 7 poses, and 124 landmarks. Both have 12 keyframes over
80 frames. These are lifecycle/landmark-window differences, not a scalar IMU
golden mismatch. The improved metric versus pre-fix does not support another
unproven estimator or threshold change in this bounded task.

## Verification

The focused scalar golden and full library suite were green before this replay:

```text
cargo test --release -p visloc-basalt --lib \
  vio::estimator::tests::upstream_f32_interval_keeps_integer_endpoint_and_sophus_exp_order
cargo test --release -p visloc-basalt --lib       # 128 passed
cargo test --release -p visloc-basalt --tests     # passed; external-data tests ignored
```

The 400-frame staged command is prepared for the next compile-safe boundary:
the same command with `--out-dir target/basalt_m7ar_run400 --max-frames 400`.
No mapper or marginalization source was changed for M7ar.

## Frame-35 lifecycle audit

The pinned trace shows no keyframe-predicate divergence at the first structural
gap. Frames 30--34 have identical connected/unconnected counts and landmark
counts in the two traces; frame 35 also has identical predicate inputs
(`connected=126`, `unconnected=93`) and both select a keyframe. The divergence
is in keyframe insertion, before any subsequent state/pose marginalization:

| frame | upstream KF | Rust KF | upstream connected/unconnected | Rust connected/unconnected | upstream landmarks/new/lost | Rust landmarks/created/lost |
|---:|:---:|:---:|---:|---:|---:|---:|
| 30 | no | no | 141/57 | 141/57 | 154/0/9 | 154/27/9 |
| 31 | no | no | 137/71 | 137/71 | 150/0/4 | 150/28/4 |
| 32 | no | no | 134/77 | 134/77 | 148/0/2 | 148/21/2 |
| 33 | no | no | 131/79 | 131/79 | 145/0/3 | 145/14/3 |
| 34 | no | no | 131/85 | 131/85 | 145/0/0 | 145/14/0 |
| 35 | yes | yes | 126/93 | 126/93 | 191/51/5 | 179/22/5 |
| 36 | no | no | 176/50 | 164/62 | 191/0/0 | 179/22/0 |
| 37 | no | no | 172/56 | 161/67 | 189/0/2 | 178/21/1 |
| 38 | no | no | 164/61 | 153/72 | 186/0/3 | 175/26/3 |
| 39 | no | no | 151/58 | 143/66 | 170/0/16 | 162/33/13 |
| 40 | no | no | 148/68 | 140/76 | 166/0/4 | 158/27/4 |

Thus the first causal mismatch is the set of accepted new landmarks in
`collect_observations`, not `decide_keyframe`: upstream inserts 51 at frame 35
while Rust inserts 22 (and has 179 live landmarks after five lost records are
removed). The Rust path already follows the pinned candidate ordering and the
documented `0 < inverse_distance < 3` acceptance contract. The available
upstream artifact contains endpoint tracks and aggregate lifecycle counts, but
not per-candidate triangulation status/depth/host records. Since changing the
rho gate, SVD, or host history without that evidence would be speculative, no
production change was made in this audit. Frame 36 is the first downstream
connectivity divergence (164/62 vs 176/50).

As a bounded falsification check, the estimator rho gate was temporarily
changed from `<3` to the pinned setup-opt oracle's `<=2` and replayed as
`target/basalt_m7ar_rho2_run80`. Frame 35 remained exactly 22 created / 179
live landmarks and the final count remained 151, so this gate is not the
cause. The temporary change was reverted; no production estimator change was
left from this experiment.

## Pinned `addNewLandmarks` source comparison

The pinned source snapshot is available at
`target/basalt_trace_work/src/vi_estimator/sqrt_keypoint_vio.cpp` (its source
hash is recorded in `m7q_state_generation_oracle.md`). Its frame-processing
order was checked against `Estimator::collect_observations`: the current
optical-flow result is inserted before connectivity/keyframe selection;
candidates are gathered from retained `prev_opt_flow_res` in ordered
`TimeCamId` order; raw camera vectors and camera-extrinsic relative poses are
used for the squared baseline gate; and finite `0 < rho < 3` points are hosted
at current cam-0 with every retained observation added. These semantics match
the Rust path. The pinned aggregate trace has no candidate IDs, input
matrices, rho, or rejection reasons, so it cannot distinguish Eigen
`BundleAdjustmentBase<float>::triangulate` numerical acceptance from retained
history data. No threshold, ordering, or lifecycle change is justified by the
available evidence.

## Candidate-level oracle (frame 35)

The disposable upstream instrumentation was built from the pinned checkout's
`build/core-relwithdebinfo` Ninja target and restored automatically afterward.
The preserved build script is
`benchmarks/basalt/m7ar_upstream_candidate_oracle.sh`; its upstream output is
`target/m7ar_upstream_candidates.txt`. Rust candidate logging was run on the
same 36-frame input; raw diagnostics are `target/m7ar_rust_candidates.txt`
and `target/m7ar_rust_candidates_all.txt`.

The upstream logger emitted 51 baseline-passing candidates at frame 35; Rust
emitted 39. There were no Rust-only candidates. The first missing upstream
candidate is track `1669`, candidate `(timestamp=1403636581163555584,
camera=0)`: upstream relative-translation squared norm is
`0.0027474984526634216`, i.e. baseline `0.052˙4 m`, while Rust's corresponding
history records are `0.0402447320520878 m` (below the exact `0.05 m` gate), so
Rust rejects it before triangulation. The first common candidate's rho also
shows only a small Eigen-vs-nalgebra difference: track `1880`, current cam-1,
upstream `0.56335318088531494` versus Rust `0.56335389614105225`.

This proves the first structural cause is an earlier pose-history divergence
feeding the baseline gate, not a keyframe predicate, candidate ordering, or
rho threshold. It cannot be faithfully corrected in `addNewLandmarks` without
fixing the preceding estimator pose solve; no lifecycle workaround or
threshold relaxation was applied.
