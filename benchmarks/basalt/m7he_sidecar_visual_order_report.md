# M7He libstdc++ unordered landmark traversal parity

The production landmark database now keeps its public `BTreeMap` payload and
an order-only sidecar that mirrors the pinned GCC 11 `std::unordered_map`
implementation.  The sidecar uses the identity hash, the sparse
`_Prime_rehash_policy` table, the global node chain, bucket-head insertion,
rehash, erase, and reinsert semantics.  It contains no MH01 track IDs or
track-count branches.  The estimator consumes this order for visual landmark
linearization, and the unconnected candidate set uses the same source
container model.

## Fresh MH01 frame-4 check

The stale release executable was discarded.  The latest PE used for this run
was `target/release/examples/basalt_euroc_vio_demo.exe`, SHA-256
`A80D7C0999419C0C1DFA61FD896F7CED392B96838DB5EB83F5019F04AF790831`, built at
16:59:57 on 2026-08-24.  The one-thread, five-frame replay wrote
`target/m7he_sidecar_fresh5_detail_v3.jsonl`; it is semantically identical
to the earlier v2 detail after omitting only the self-referential
`trace_path` field.  Full provenance is in
`target/m7he_sidecar_fresh5_v3_provenance.json`.

The diagnostic `landmark_factors` array is sorted by `track_id` for stable
JSON, so its list order is not the solver order.  Sorting its records by
`row_span[0]` gives all 61 native IDs, beginning
`120, 119, 21, 14, 73, 13, 72, 3, ...` and ending `..., 109, 110, 111, 118`.

For the direct global visual check, each authoritative native per-landmark
`local_H`/`local_b` block was folded with binary32 additions in that row-span
order and compared with the clean native visual-boundary aggregate:

| traversal | H bit mismatches | b bit mismatches |
| --- | ---: | ---: |
| sidecar row-span order | 0 | 0 |
| native captured order | 0 | 0 |
| sorted BTree order (control) | 730 | 22 |

The machine-readable results are
`target/m7he_sidecar_fresh5_visual_compare_v3.json` (latest trace) and its
v2 native-fold reference; v3 provenance confirms that the comparison remains
authoritative after the latest rebuild.

## Combined H/b frontier

For the next boundary, a clean native GDB capture was taken at the
`get_dense` return after the visual reduction, IMU, prior, and damping terms
had been added.  Comparing that column-major capture with the Rust frame-4
`iteration_start` snapshot (transposing the Rust row-major JSON matrix) gives
1,931 H bit differences and 69 b bit differences.  The native full-vs-visual
split itself changes 2,115 H entries and 69 b entries.  This is retained as
the IMU/prior/damping frontier; it does not invalidate the visual sidecar
result above.  See
`target/m7he_sidecar_fresh5_combined_compare_v2.json` (v1 is the identical
pre-rebuild comparison).

The same fresh trace reaches the LM boundary with eight accepted trials and
no rejection/restore branch.  Every trial applies a 75-DOF state step and 61
landmark back-substitutions.  The first transition is
`lambda=9.999999747e-5 -> 3.333333370e-5`, cost
`4215.93212890625 -> 413.0161437988281`; the final transition is
`lambda=9.999999975e-7 -> 9.999999975e-7`, cost
`248.7586212158203 -> 248.47608947753906`, with step norm
`0.007605708669871092`.  The complete apply/cost/lambda record is
`target/m7he_sidecar_fresh5_lm_boundary_v1.json`.

A clean native `vio_debug=true` five-frame capture has the same LM branch
shape: eight accepted trials, no rejected trial, and no restore.  Its
rounded per-trial values are retained in
`target/m7he_native_lm_debug5_v1.json` (raw output:
`target/m7he_native_lm_debug_1t.txt`).  Native and Rust therefore agree on
the apply/accept/lambda schedule at this boundary; their exact costs and
state bits remain subject to the combined nonvisual frontier above.

For the missing branch, a clean pinned native run with `vio_debug=true` was
extended to 80 frames.  The first single-rejection boundary (58 landmarks,
437 observations) has seven accepted trials followed by iteration 7
`rejected_restore` at lambda `1e-6`; native reports
`num_it_rejected=1`.  The solver restores the backed-up state after the
rejected evaluation and would increase lambda by `lambda_vee` before another
backtrack.  The raw native logger and rounded, machine-readable boundary are
retained in `target/m7he_native_lm_debug80_1t.txt` and
`target/m7he_native_lm_reject_restore_v1.json`; later windows in the same
capture exercise repeated backtracking rejects.  The Rust unit test
`vio::aom::tests::lm_reject_restores_state_and_increases_lambda` covers this
restore contract, while exact cross-implementation state bits remain behind
the combined IMU/prior/damping mismatch.

## 80-frame LM branch and sqrt/FEJ/MargData boundary

The fresh v3 release executable was replayed for all 80 camera frames.  The
generic parser `benchmarks/basalt/m7he_lm_branch_compare.py` compares the
76 active Rust LM runs (frames 4..79) against
`target/m7he_native_lm_debug80_1t.txt` at frame/iteration granularity.  The
machine-readable artifact is `target/m7he_lm_branch_compare_v1.json` (SHA-256
`6C0E54EF95A490C9F2F0F711D6A34C5AD6C96CEEB1B899EA6DB3BC8778D46CB4`):

| comparison | result |
| --- | ---: |
| common LM runs | 76 / 76 |
| frame branch strings equal | 1 |
| frame branch strings divergent | 75 |
| displayed lambda mismatches | 0 |
| first divergence | frame 5, iteration 3: native A / Rust R |

The first divergence is a cost-contract boundary, not evidence of an
IMU-only branch error.  Native frame 5 starts with linearized error
`-3.1381e4`, while Rust's complete square-root factor objective is
`+376.64947509765625` (prior `1.21809`, visual `353.70529`, IMU `21.7261`).
Pinned Basalt's `linearizeMargPrior`/`computeMargPriorError` evaluates the
prior as `0.5*delta^T H delta + delta^T b` and deliberately drops the constant
`0.5*r^T*r`; that value may therefore be negative.  Rust currently reports
the complete positive square-root residual norm through its generic
factor-cost path.  The artifact records this as the next sqrt-prior/FEJ cost boundary;
the branch schedule must not be called exact until that convention and the
remaining IMU/H/b frontier are reconciled.

The selected frame-5 detail capture is
`target/m7he_sidecar_fresh6_frame5_detail_v1.jsonl` (SHA-256
`EAB46945B4C140E6B9BCFCCFEC9370477D469EEDB1253B41E9B0F25410E2A339`).  It
contains the same 8 Rust trials as the parser, with three accepted trials
followed by five rejected clone evaluations.  Rust's generic restore contract
is explicit: `trial_cost` clones `WindowProblem`, installs the full manifold
state and landmark increments only in the clone, and `accept_step` is the only
mutating commit path.  A rejected trial therefore leaves both state and
landmark values unchanged; the synthetic unit test cited above independently
checks state restoration and lambda growth.

The 80-frame MargData boundary is summarized by
`benchmarks/basalt/m7he_margdata_boundary.py` and
`target/m7he_margdata_boundary_v1.json` (artifact SHA-256
`90032DD3263710FE06B97932083D61A199899B21CC8586D789A436E999F277A2`; script
SHA-256
`F72969C93CBD23CF8415A14EA628428C2C8565D030098F225CE938F5F5A39BCA`).  The
latest trace has 80 marginalization diagnostics, 76 successful window solves,
and exactly five queue-facing packets at frames `51, 58, 65, 72, 79`.
Every packet is schema v3 with a contiguous 72-column mixed AOM (seven 6-DoF
pose blocks plus two 15-DoF state blocks), FEJ flags `[true,false,false]`,
`used_imu=true`, canonical images, and no diagnostic-only carried `prior` in
the queue view.  The packet shapes are:

| packet frame | sqrt J rows × cols | row groups `[prior, visual, IMU, bias]` | `kfs_to_marg` |
| ---: | ---: | --- | ---: |
| 51 | 2108 × 72 | `[57, 2036, 9, 6]` | `[14]` |
| 58 | 2372 × 72 | `[57, 2300, 9, 6]` | `[42]` |
| 65 | 2848 × 72 | `[57, 2776, 9, 6]` | `[35]` |
| 72 | 3338 × 72 | `[57, 3266, 9, 6]` | `[56]` |
| 79 | 2958 × 72 | `[57, 2886, 9, 6]` | `[49]` |

For all five packets, recomputing `JᵀJ`/`Jᵀr` from the serialized square-root
system agrees with the absolute H/b snapshot within binary32 roundoff (the
largest observed H difference is `9.54e-7`; b is below `3.08e-8`).  Structural
comparison against the pinned upstream packet projections is exact for frames
51, 58, and 65; frame 72's and 79's keyframe-removal lists differ because the
frontend/landmark trajectory has already diverged, while AOM dimensions,
state timestamps, FEJ flags, IMU flag, and wire-prior omission remain valid.
This is the independent MargData/FEJ structure boundary before numeric
trajectory parity.

## Tests

`cargo test -p visloc-basalt --lib` covers the generic insertion/rehash and
mixed erase/reinsert semantics, plus sparse-prime requests through 10,000 and
2,000,000.  The focused unordered-order tests are:

```text
vio::landmarks::tests::libstdcxx_unordered_order_matches_gcc11_growth_and_rehash
vio::landmarks::tests::libstdcxx_unordered_order_handles_erase_and_reinsert
vio::landmarks::tests::libstdcxx_unordered_order_uses_sparse_prime_growth_beyond_fixture_sizes
```

An independent rebuild on 2026-08-24 ran the shared prefix filter directly:

```text
cargo test -p visloc-basalt --lib libstdcxx_unordered_order_ -- --nocapture
test result: ok. 3 passed; 0 failed; 0 ignored; 188 filtered out
```

The v3-snapshot full-library verification was `190 passed, 0 failed, 1
ignored`; the later prior-contract test added one library test.  The current
post-order-edit verification is recorded below as `191 passed, 0 failed, 1
ignored`.

The boundary-focused tests were also rerun after the v3 replay:

```text
upstream_f32_sqrt_marginalization_matches_eigen_golden_boundaries ... ok
fej_local_boxminus_is_zero_and_first_order_consistent ... ok
lm_reject_restores_state_and_increases_lambda ... ok
mapper_packet_requires_keyframe_removal ... ok
```

Each command reported `1 passed, 0 failed` (190 filtered library tests).

## LM prior contract and v4 first-divergence audit

The pinned `ba_base.cpp` contract is now represented directly in the Rust
window solver.  For a square-root prior with stored `(J,b)`, the current
linearized error is

```text
deltaᵀ Jᵀ (0.5 J delta + b)
```

The FEJ-independent `0.5*rᵀr` term is deliberately omitted, so the value may
be negative.  Rust keeps the complete positive residual norm for the row
stack/H/b construction, but replaces only the prior's LM objective with this
constant-free value.  The f32 path casts J, b, and the accumulated upstream
`delta` before evaluating the expression.  FEJ points and accumulated local
increments are retained in the state/pose sidecars; local box-minus is not
used as a substitute for Basalt's `Pose*WithLin::delta`.

The v4 release replay (`target/m7he_sidecar_run80_v6`) was compared against
`target/m7he_native_lm_debug80_1t.txt` by
`target/m7he_lm_branch_compare_v4.json`:

| comparison | result |
| --- | ---: |
| common LM runs | 76 / 76 |
| frame branch strings equal | 11 / 76 |
| displayed lambda mismatches | 7 |
| first divergence | frame 8, iteration 4: native R / Rust A |

The prefix through frame 7 is exact (`frame 4 AAAAAAAA`, frame 5
`AAAAAAAA`, frame 6 `AAAAAAR`, frame 7 `AAAAARRR`).  At frame 8, iteration 4,
the native f32 logger rounds the trial's `f_diff` to zero and rejects it,
whereas Rust sees a small positive improvement and accepts it; both then
reject the remaining trials.  The prior contract is therefore frozen as a
generic source-equivalent implementation.  The remaining boundary is the
known IMU/covariance/H/b ULP frontier (the first covariance term contributes
51/810 and the final term 57/810 in the current audit), not a track-specific
branch rule.

The focused source-contract test
`vio::window::tests::marginal_prior_error_drops_constant_and_allows_negative_values`
checks both the negative reduced value and the positive complete residual;
the f32 sqrt marginalization and rejected-trial restore tests remain green.
The post-cleanup full library run reports `191 passed, 0 failed, 1 ignored`.

## LM source-order replay v5

The LM objective fold now follows the pinned `LinearizationAbsQR` category
order: visual landmark factors first, IMU/bias factors second, and the
constant-free marginal-prior error appended last.  The AOM prior row is
excluded from the generic positive residual fold; it is not folded and then
subtracted as a post-hoc correction.  This is a generic category/order rule,
not a fixture-specific branch adjustment.

A fresh release build (`basalt_euroc_vio_demo.exe`, SHA-256
`F641FA443B49A1E08E582E58CD2A363234EBA16CC731DB2B277589A517AD9FB5`) was
replayed through 80 frames.  The trace is
`target/m7he_sidecar_run80_v7/trace.jsonl` (SHA-256
`6A0CC5F9B3B388A590B08102DCDB7EA75E060BEAAFEDEB1BDD661750579D5C98`).
Comparison with the pinned one-thread native logger is in
`target/m7he_lm_branch_compare_v5.json` (SHA-256
`7AD31A72E7088DAFA8D31C5EDE456C437AF9EA097BB0296E6162E997D08959D5`):

| comparison | result |
| --- | ---: |
| common LM runs | 76 / 76 |
| frame branch strings equal | 11 / 76 |
| frame branch strings divergent | 65 / 76 |
| displayed lambda mismatches | 7 |
| first divergence | frame 8, iteration 4: native R / Rust A |

The source-order correction does not change the displayed f32 cost or branch
at this fixture's first boundary: both association paths round to the same
binary32 value at frame 8, iteration 4.  It is retained because it is the
upstream contract for other scales and category counts.  The remaining first
boundary is therefore still the generic IMU/covariance/H/b ULP frontier; no
iteration- or-track-specific special case was added.

## Frame-8 category decomposition and IMU-only counterfactual

The opt-in detail trace now emits `category_costs` for the source categories
`prior`, `visual`, `imu`, and `bias` at every LM snapshot.  Trial snapshots use
the same complete cloned problem as `trial_cost`, including FEJ/local-delta
sidecars and back-substituted landmark increments; this keeps the category
prior at the actual candidate state rather than at the accepted base state.
The focused prior-contract test and the full library run both pass after this
diagnostic correction (`191 passed, 0 failed, 1 ignored`).

The native GDB capture was extended to select the fifth dense boundary with
139 active landmarks, corresponding to native frame 8 / iteration 4.  It is
`target/m7he_native_frame8_categories_iter4.json` (SHA-256
`D61A4DFAA1A26DDF3964176337988704DCC149CE58D586EE51EE8BD6EB4481D5`).  The
Rust v5 frame-8 detail/trace inputs are
`target/m7he_frame8_categories_detail_v5.txt` (SHA-256
`4FFE5A5749211C814CA36AC8961A6A32256E33FE48C67368F7DD0D18D4EED84E`) and
`target/m7he_frame8_categories_run_v5/trace.jsonl` (SHA-256
`8F72919CFB820764AB3D56CFDC9411B360AD17C24E06F074C66C4A53B59A75E2`).
The generic comparator is
`benchmarks/basalt/m7he_lm_category_compare.py` (SHA-256
`CA53DD4615646385530C4C035931B7255E2269301083C13447B6E356EB83B9D0`), and
the stage-4 comparison is
`target/m7he_lm_category_compare_v6_iter4.json` (SHA-256
`4512ABB214F153F9452E5139B2E3118610E31DA917B85D303D05C8F1A9076A2A`).

At Rust frame 8 / iteration 4, the accepted base objective decomposes as
`prior=-41500.62109375`, `visual=75.12142181396484`,
`imu=0.16751083731651306`, `bias=3.1010390557639766e-6`,
`total=-41425.33203125`.  The trial candidate is
`prior=-41500.53515625`, `visual=75.03792572021484`,
`imu=0.15545392036437988`, `bias=3.1295755889004795e-6`,
`total=-41425.33984375`; the LM decision remains a small accepted decrease
of `-0.0078125` in serialized f32.  The native logger rounds this boundary to
`f_diff=0`, `l_diff=0.0015454`, and rejects it.

The stage comparison must be interpreted with the structural caveat that the
native window has 139 landmarks while the Rust trajectory has 128.  For the
IMU increment alone, native cumulative `(visual+IMU)-visual` first differs at
H[6,6]: Rust `378793344` (`4db49f6c`) versus native `378793536`
(`4db49f72`), absolute difference 192.  The category comparator reports the
full max differences (`H=2.000624e9`, `b=4217.50`) and therefore does not
attribute the whole mismatch to IMU; the visual/window structure difference is
material.

To answer the causal branch question, the diagnostic-only delta artifact
`target/m7he_frame8_iter4_imu_hb_delta.json` (SHA-256
`96603F3E4D12F9E3A621E450D75F8433B8B77553DFB812327DD419AC697EEB5D`)
contains only `(native IMU increment - Rust IMU increment)` in f32 bit form.
An environment-guarded, default-no-op `LmProblem` hook applies that delta to
the reduced f32 H/b at the requested frame/iteration; it is not a production
estimator rule.  The override replay changes frame 8 from Rust `AAAAARRR` to
`AAAARARR`, exactly matching native `AAAARARR` for all eight trials.  Its
trace is `target/m7he_frame8_imu_override_detail_v6.txt` (SHA-256
`2931A783959C52D08DC5FDCE9B55C51A83AB2D42E176DDA7DE3B2026CF102BE4`),
`target/m7he_frame8_imu_override_run_v6/trace.jsonl` (SHA-256
`69E981CEC93B3091926809BE3048E3666B0B883AB46E3C7711CFED1875D2DEAA`), and
`target/m7he_lm_branch_compare_imu_override_v6.json` (SHA-256
`CAE05726E3D04D422A41061F284E7138F9DCA9A396DC9EAD1E3F32402C1E5705`).
This confirms that the captured IMU H/b boundary is sufficient to flip this
near-zero branch, while not claiming complete trajectory parity because the
visual/window landmark sets are already different.

The replay is ready to rerun after the IMU covariance/LDLT update.  The
normal (unpatched) stage comparator command is:

```text
python benchmarks/basalt/m7he_lm_category_compare.py --rust-detail target/m7he_frame8_categories_detail_v5.txt --rust-trace target/m7he_frame8_categories_run_v5/trace.jsonl --native-categories target/m7he_native_frame8_categories_iter4.json --native-stdout target/m7he_native_lm_debug80_1t.txt --frame 8 --stage-iteration 4 --iteration 4 --out target/m7he_lm_category_compare_v6_iter4.json
```

The current WSL release executable used for the v5/v6 diagnostic runs is
`target/release/examples/basalt_euroc_vio_demo` (SHA-256
`8D997A63A269BBA7DB8E95F7B79BBB96468AD9119E193A59458E441B1F87B6FA`).

The IMU counterfactual replay uses the same command-line run as the v5
sensor-only replay with these additional environment variables:
`VISLOC_BASALT_DIAGNOSTIC_IMU_HB=target/m7he_frame8_iter4_imu_hb_delta.json`,
`VISLOC_BASALT_DIAGNOSTIC_FRAME=8`, and
`VISLOC_BASALT_DIAGNOSTIC_ITERATION=4`.  Leaving those variables unset gives
the ordinary estimator path.

```text
VISLOC_BASALT_DETAIL_TRACE=target/m7he_frame8_imu_override_detail_v6.txt VISLOC_BASALT_DETAIL_FRAME=8 VISLOC_BASALT_DETAIL_ITERATIONS=1 VISLOC_BASALT_DIAGNOSTIC_IMU_HB=target/m7he_frame8_iter4_imu_hb_delta.json VISLOC_BASALT_DIAGNOSTIC_FRAME=8 VISLOC_BASALT_DIAGNOSTIC_ITERATION=4 ./target/release/examples/basalt_euroc_vio_demo --euroc-dir /mnt/e/datasets/euroc_mav/machine_hall/MH_01_easy --calibration target/euroc_ds_calib.json --config target/euroc_config.json --out-dir target/m7he_frame8_imu_override_run_v6 --max-frames 9 --no-marg-data
```

## Post-covariance run8 and ordinary frame-8 replay

The fresh release executable after the assignment-tail correction is
`target/release/examples/basalt_euroc_vio_demo`, SHA-256
`0B17694D581183F4929211A56EC45A9359CC6292AC933521D2ABE5000D75B69E`.
With all diagnostic/override environment variables unset, the ordinary
frame-8 replay is captured in `target/m7he_frame8_normal_v7_detail.txt`
(SHA-256 `1031E73F8AB1A120D7A51223BD175738912224E76BB3EFA4990390C07767D72B`)
and `target/m7he_frame8_normal_v7/trace.jsonl` (SHA-256
`061D89E615A63B359C51C2F33AD0F3F1925F755FA6E4D4CEAE56C61B87DE3C94`).
The generic LM artifact is
`target/m7he_lm_branch_compare_normal_v7.json` (SHA-256
`6AE8EDBCB24AE41C493DC7FF12227EFBA070A128864E03778AA0C9B9C6FCD3B9`).

Frames 4--7 remain exact in branch shape (`AAAAAAAA`, `AAAAAAAA`,
`AAAAAAAR`, `AAAAARRR`). The first ordinary divergence is still frame 8,
iteration 4: native `AAAARARR` versus Rust `AAAAARRR`. This preserves the
known near-zero LM boundary and does not regress the earlier exact prefix.

The same ordinary release binary was then replayed through 80 camera frames
with all diagnostic and override variables unset. The trace is
`target/m7he_normal_v7_80/trace.jsonl` (SHA-256
`E237B6FE57AA41DCA30CAD4DC04E640745ACEE8C0AE1F7E312B5FA2759C11712`), and
the 76-run comparison is
`target/m7he_lm_branch_compare_normal_v7_80.json` (SHA-256
`C7CE9035B311BD709220297EAB0020034DB265A96B8830D40A73909DEEEF3E18`).
All 76 native/Rust LM runs are present; 8 frame branch strings are equal and
68 diverge. The first divergence remains frame 8 / iteration 4
(`AAAARARR` native versus `AAAAARRR` Rust), so the covariance correction does
not move the known LM frontier. Later trajectory cascades are retained as
diagnostic evidence, not treated as independent first-boundary failures.
