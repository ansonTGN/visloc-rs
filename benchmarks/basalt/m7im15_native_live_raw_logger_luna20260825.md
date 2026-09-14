# M7 live native IMU boundary logger (2026-08-25)

An isolated detached worktree was created at
`target/m7im15-native-logger-20260825` from pinned commit
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`. The original checkout
`/root/visloc-basalt-clean-m7cr-20260823` was clean before and after; only the
detached worktree's `include/basalt/linearization/imu_block.hpp` was patched.
The logger is env-gated (`M7IM15_NATIVE_RAW_LOG`), first-call filtered to
`start_t=1403636579763555584`, and records final residual after the
`isLinearized()` branch, derivative blocks, and cached W as binary32 bits.

Artifacts:

- [raw binary](../../target/m7im15_native_raw_live_luna20260825.bin) (1244 bytes)
- [parsed/reconstructed stages](../../target/m7im15_native_raw_reconstructed_luna20260825.json)
- [live comparison](../../target/m7im15_native_live_raw_compare_luna20260825.json)
- [temporary logger source](../../target/m7im15-native-logger-20260825/include/basalt/linearization/imu_block.hpp)
- [parser](../../target/m7im15_reconstruct_native_raw_luna.cpp)
- [comparison script](../../work/m7im15_compare_live_raw.ps1)

The temporary RelWithDebInfo `basalt_vio` was built with GCC, `-O2 -g -DNDEBUG
-march=native`, and the pinned vcpkg dependencies, then run one-thread,
max-frame 5. Raw 9x30 assembly is `[d_start | d_bg | d_ba | d_end | zeros]`.
Applying Eigen's ordinary f32 product reproduces the live native record-0
first-9 residual exactly (9/9). The Jacobian reconstruction is 251/270 exact;
the first difference is column-major index 75, `41f442f7` vs `41f442f6`
(19 one-bit packet-product differences), so this validates layout and W but
does not claim a fully exact Eigen packet schedule.

Against the Rust R2 boundary capture, live-native counts are: raw residual
7/9 exact (first difference index 1: native `2e800000`, Rust `00000000`), raw
Jacobian 261/270 (first index 27: `3a7bcc9e` vs `3a7bcc9f`), W 75/81 (first
index 6: `c17600af` vs `c17600b0`), whitened residual 3/9, and whitened
Jacobian 242/270 (first index 15: `406a1cbf` vs `406a1cc0`). No production
arithmetic change was made.

## Live input-boundary capture (R3)

The logger was then extended in the same detached worktree with an
`M7IM15_NATIVE_INPUT_LOG` record containing the FEJ flags, start/end
PoseVelBias states (linearized and current), preintegration delta state,
covariance, and bias-state derivatives, all as binary32 bits. The capture is
the same live frame-0-to-1 factor (`start_t=1403636579763555584`) as native
record 0. The native flags are `[start linearized=true, end linearized=false]`.

The Rust R3 diagnostic was extended only with covariance f32 bits. After
normalizing quaternion ordering, the native/Rust input comparison is exact:
from state 16/16, to state 16/16, delta state 10/10, covariance 81/81.
Therefore the first raw residual mismatch (index 1, 7/9 exact) and raw
Jacobian mismatch (index 27, 261/270 exact) occur during residual/Jacobian
evaluation, after identical inputs. W differs at index 6 (75/81 exact) while
covariance is exact, identifying the whitening arithmetic/schedule as a
separate downstream difference. Native reconstruction still gives residual
9/9 exact and J 251/270 under ordinary Eigen multiplication.

Current artifacts and hashes (SHA-256):

- `m7im15_native_raw_live_luna20260825_input2.bin`:
  `6F19512031E02C2BA08F2490E0192874919DDEB8C1F851B91C8F3BF856289CCF`
- `m7im15_native_input_live_luna20260825.bin`:
  `41FFE300003316BFE4285029AEDE065EAE4E555D337BB07C781F22D908200A5E`
- `m7im15_native_input_stage_compare_luna20260825_r3.json`:
  `9E6144C601B39663A0464EBEC7A5AB547758180DC8361C54E360A6F78A9BB781`
- `m7im15_native_live_raw_compare_luna20260825_r3.json`:
  `977DE62BD32DF87986588C557A95B949EC8825357705C74679648AAAFE02C3C2`
- Rust R3 diagnostic JSON:
  `81F28BC2EF0F05C124F94F59ED40F6A357E2224E1C4E39EDF8308ABB53005CF1`
- rebuilt Linux ELF `target/release/examples/basalt_euroc_vio_demo`:
  `358C6F2684A652ACEE86F284B7E19E00495DF28216CEDA8D4D4D7654F2B12463`

Validation: `cargo fmt --check`; focused release test 1 passed; full release
library tests 201 passed, 1 ignored, 0 failed. The original pinned checkout
remains at `0f3b2b52c807f70ff4e2973ce253c73329eea7bc` with zero status lines.
No production arithmetic change, commit, or push was performed.

## Faithful-port gate (follow-up)

The pinned source order was independently checked against
`preintegration.h`: residual and derivative blocks are formed first, then
each block is multiplied by `get_sqrt_cov_inv()` in `ImuBlock::linearizeImu`.
The Rust path already uses the corresponding f32 stages and explicit Eigen
packet-order helpers. The live trace does not yet prove the missing packet
trees: raw residual is 7/9, raw Jacobian 261/270, and W 75/81. Consequently
the requested 9/9, 270/270, 81/81 gate is not met, and no epsilon correction,
bit patch, or unproven production arithmetic change was made. A further
native intermediate trace of SO3 exp/log, LDLT pivots/solve panels, and each
9x30 product kernel is required before changing the implementation.

## Expression trace follow-up

The detached GCC11 logger was extended with `M7IM15_NATIVE_EXPR_TRACE`. The
new trace records source-order f32 inputs/outputs for `dt`, bias correction,
`R0_inv`, position/velocity terms, relative SO3 log, delta terms, and the
position/rotation/gyro-bias Jacobian 3x3 blocks. The first live frame-0-to-1
capture is 315 bytes and parses as:

- [native expression trace](../../target/m7im15_native_expr_trace_luna20260825.json)
- [binary trace](../../target/m7im15_native_expr_trace_luna20260825.bin)
- [parser](../../work/m7im15_parse_expr_trace.ps1)

The trace confirms the pinned source evaluation order, but the current Rust
diagnostic has no matching named intermediate fields, so it cannot yet prove
the first expression-level bit difference. The existing raw comparison remains
the authoritative stage boundary (7/9 residual, 261/270 J, 75/81 W). No
production change was made from this trace. LDLT pivot/L/D and whitening packet
partials still require instrumentation inside the pinned preintegration header
before a faithful Rust edit can be justified.

## Rust/native expression comparison R4

Rust `preintegration_f32_expression_trace` now emits the same named fields
and column-major bit ordering. The R4 capture is in
`target/m7im15_rust_imu_factor_input_frame4_link0_linux_luna20260825_r4.json`;
the field comparison is
`target/m7im15_expr_trace_compare_luna20260825_r4.json`.

The first concrete arithmetic difference leading to the live raw-J mismatch
at flat index 27 (`J_start_rotation[0,0]`) is the preceding position
intermediate `dR0_translation` at component 1: native `ba397d96`, Rust
`ba397d97`. `R0_inv` itself is 9/9 exact, while `J_start_position` is 9/9
exact. Thus the discrepancy is introduced in the position translation
expression before the `hat(tmp) * R0_inv` rotation Jacobian product; no
production correction is justified yet. Other R4 field counts: dt 1/1,
relative log 3/3, delta position/velocity 3/3 each, rotation log 3/3.
The native derivative trace also exposes a FEJ/current-state derivative
selection difference in the gyro-bias block, so it must not be patched as a
floating-point schedule issue.

## Position-expression closure

An operand trace was added for the translation expression. The native source
uses the left-associated scalar schedule
`((Scalar(0.5) * g) * dt) * dt`, whereas Rust had used
`g * (Scalar(0.5) * dt * dt)`. With the native operand bits, the gravity and
translation operands were each first different at component 2 by one ULP.
The Rust helper was changed to the source-faithful left-associated schedule.

R6 result on the same live factor:

- expression fields: translation/gravity operands 3/3, `dR0_translation` 3/3,
  `J_start_rotation` 9/9;
- raw residual: 9/9 (previously 7/9);
- raw Jacobian: 267/270 (remaining mismatches are indices 99, 107, 110 in
  gyro/accelerometer bias derivative columns, consistent with FEJ/current
  derivative selection rather than the translation arithmetic);
- whitened residual: 8/9; W remains 75/81.

The native operand binary is
`target/m7im15_native_expr_trace_luna20260825_operands.bin`; the R6 field
comparison is `target/m7im15_expr_trace_compare_luna20260825_r6.json` and the
stage comparison is `target/m7im15_native_live_raw_compare_luna20260825_r6.json`.

## Remaining raw-J mapping

Column-major mapping is exact: flat 99 = `(row=0,col=11)` = gyro-bias
derivative row 0/column 2; 107 = `(row=8,col=11)` = gyro-bias derivative row
8/column 2; 110 = `(row=2,col=12)` = accel-bias derivative row 2/column 0.
The native input logger and Rust R8 diagnostic now emit the full 9x3
`d_state_d_bg`/`d_state_d_ba` operands in column-major order. After correcting
the diagnostic serialization order, the only operand mismatches are exactly:

- `d_state_d_bg` flat 18: native `b7502827`, Rust `b7502825` (raw J 99)
- `d_state_d_bg` flat 26: native `37922b6b`, Rust `37922b6c` (raw J 107)
- `d_state_d_ba` flat 2: native `35f03db6`, Rust `35f03db4` (raw J 110)

Thus FEJ selection is not the cause of the three remaining live differences;
they originate in preintegration derivative integration before `ImuBlock`.
No source-faithful derivative integration expression has yet been isolated,
so no speculative arithmetic edit was made. The safe live result remains raw
residual 9/9 and raw J 267/270.

## Rust per-update derivative trace (2026-08-25 R9)

The Rust f32 queue now records the same relative endpoint timestamp and
column-major 27-bit BG/BA arrays after each update. The capture
`target/m7im15_rust_dstate_trace_frame4_linux_luna20260825_r9_input.json`
contains 10 updates. The update-wise comparison is
`target/m7im15_dstate_update_compare_luna20260825_r9.json`.

Update 0 is exact for both matrices (BG 27/27, BA 27/27). The first
divergence is update 1 at `t_ns=9999872`: BG flat index 9 differs by one ULP
(`native 0x34fdef7d`, `Rust 0x34fdef7c`); BA remains 27/27 exact. Flat index 9
is the position/bias-gyro block, column 1. The governing pinned update is
`new_d_state_d_bg = -G + F * old_d_state_d_bg`; this identifies the first
affected operation boundary, but the native F/G operand trace was not present
in this capture, so no source-faithful arithmetic edit is justified. Later
update exact counts are BG/BA: 23/24, 22/23, 24/25, 24/23, 24/23, 23/22,
25/24, 25/26. No production derivative change was made.

## Per-update dstate capture (2026-08-25)

The detached pinned logger overlay was rebuilt with GCC11/Eigen and run for
five frames using `M7IM15_NATIVE_DSTATE_TRACE`. The binary artifact
`target/m7im15_native_dstate_trace_luna20260825.bin` parses to
`target/m7im15_native_dstate_trace_luna20260825.json`: 10 integration updates,
timestamps 4,999,936 through 49,999,872 ns, with column-major 9x3
`d_state_d_bg` and `d_state_d_ba` bits after every update. The final record
matches the independent native input capture 27/27 for both derivative
matrices, confirming logger placement/layout.

The final record compared with Rust R8 is recorded in
`target/m7im15_dstate_trace_compare_luna20260825.json`: BG 2/27 exact (first
difference flat 1), BA 5/27 exact (first difference flat 1). This is a
same-live native-vs-Rust final comparison; Rust currently lacks the matching
per-update trace, so the first divergent IMU sample/formula is not yet
proven. No derivative arithmetic change was made.

## Native F/G operand trace (2026-08-25)

The rebuilt pinned GCC11/Eigen detached logger emitted F, G, old BG,
`F*old_BG`, new BG, and new BA for all 10 updates:
`target/m7im15_native_dstate_operand_trace_luna20260825.json`. Comparison
against the Rust R9 per-update trace is in
`target/m7im15_native_operand_compare_luna20260825_r9.json`.

At update 1 (`t_ns=9999872`), old BG is 27/27 exact against Rust update 0,
while new BG is 25/27 with the first difference at flat 9 (native
`0x34fdef7d`, Rust `0x34fdef7c`). Native flat-9 intermediate bits are
`F*old_BG=0x34d34df3`, `G=0xb3aa8628`, and final `new_BG=0x34fdef7d`.
Thus the first divergence is introduced during the second update's
`-G + F*old_BG` evaluation, not the old-state input. The capture does not yet
include the three scalar multiply-add partials, so it does not justify an
evaluation-order production change; no arithmetic fix was applied.

## Rust F/G/product trace (2026-08-25 R10)

Rust now emits f32 F (81), G (27), and `F*old_BG` (27) bits in each update.
The native/Rust comparison is
`target/m7im15_fg_compare_luna20260825_r10.json`.

For all 10 updates F is 81/81 exact and G is 27/27 exact. Product counts
are 27, 25, 23, 21, 24, 24, 24, 23, 25, 25. The first divergence is
therefore conclusively in the `F * old_d_state_d_bg` matrix product at update
1, flat index 9: native `0x34d34df3`, Rust `0x34d34df2`. F and G inputs are
exact, so the subsequent `-G + product` is not the first differing operation.
No speculative arithmetic patch was made; Eigen packet/GEMM partial tracing
would be required before a faithful fix.

## 9x9-by-9x3 Eigen packet helper (R11)

The existing covariance `eigen_matrix_product_9_f32` schedule was specialized
to a 9x9-by-9x3 helper: rows 0..7 use even/odd accumulators with FMA and the
row-8 tail uses source-order k reduction. Re-running the Rust diagnostic with
this source-faithful helper produced
`target/m7im15_fg_compare_luna20260825_r11.json`: update 0 and update 1
products are 27/27 exact; later product counts are 24,24,26,24,24,23,24,23.
All F/G fields remain 81/81 and 27/27 exact. Compared with the native
reconstructed raw-J fixture, R11 is 266/270 exact (first differing indices
81,90,96,99), so the helper fixes the first product divergence but does not
yet close all accumulated derivative differences. Full release lib tests:
201 passed, 1 ignored. No epsilon or bit patch was used.

## Effective-assignment capture (2026-08-25)

The logger was rebuilt and rerun with an additional post-assignment field
`new_BG + G` in `target/m7im15_native_dstate_operand_effective_luna20260825.json`.
Direct final BG comparison against Rust R11 is 27,27,24,25,26,24,24,23,24,23
per update; update 1 is fully fixed by the 9x3 helper. The post-assignment
`new_BG + G` field cannot recover the exact pre-add product because float
addition is not invertible, so it is not used as a product-bit oracle. No
further arithmetic change was made.

## GCC11/Eigen exact oracle (2026-08-25)

Build manifest (actual detached `flags.make`, Eigen version/config) is
`target/m7im15_native_build_manifest_luna20260825.txt`. A noinline oracle
using exact `Eigen::Matrix<float,9,9,ColMajor>` and
`Eigen::Matrix<float,9,3,ColMajor>` types, compiled with GCC11 `-O3 -g
-march=native -mfma -DEIGEN_DONT_PARALLELIZE -std=c++17`, evaluates
`out = -G + F * old` directly. On the saved ten native fixtures it matches
all 270 final BG values exactly (`exact=270 total=270`).

Oracle binary and disassembly are
`target/m7im15_eigen_oracle_luna20260825` and
`target/m7im15_eigen_oracle_luna20260825.asm.txt`. The assembly shows AVX2
packet loads/stores and FMA-capable Eigen-generated fixed-size evaluation;
the direct expression context, rather than a separately materialized product,
is the proven 270/270 schedule. No Rust helper replacement was made yet,
because the current Rust helper materializes `F*old` before `-G`, unlike this
exact oracle expression.

## Product schedule search (2026-08-25)

A scalar f32 candidate search over the saved native F/G/old/final fixtures is
in `target/m7im15_schedule_search_luna20260825.json`. Candidate exact counts
over all 270 outputs were: sequential k-order FMA-shaped accumulation 254,
reverse 224, pairwise 194, even/odd packet reduction 246, and reverse-pair
228. No candidate reached 270/270, so no candidate was promoted to
production. The first sequential mismatch remains update1 flat9, confirming
that the native Eigen operation uses a more specific fused/GEMM evaluation
than these scalar reconstructions.

## Eigen GEMM microkernel disassembly (2026-08-25)

The oracle `general_matrix_matrix_product::run` calls
`gebp_kernel<float,float,long,...,24,4,...>::operator()` (0x45ea/0x474f).
The extracted files are `target/m7im15_eigen_gemm_run_luna20260825.asm.txt`
and `target/m7im15_eigen_gebp_kernel_luna20260825.asm.txt`. This is Eigen's
AVX2 `mr=24,nr=4` kernel with packed RHS and multiple YMM accumulators;
disassembly shows repeated `vfmadd231ps`/`vfmadd132ps` and edge handling for
the 9x3 case. The output is seeded with `-G` before GEMM, so the kernel adds
directly into the existing C buffer.

Seeded scalar/2/3/4-accumulator schedules were also searched; none reached
270/270 (best 233/270). A full AVX2 gebp packet port is required for exact
Rust parity; no speculative production change was made.

## Pinned Eigen source extraction

Relevant pinned source slices are in
`target/m7im15_eigen_kernel_source_extract_luna20260825.txt`. For AVX float,
`gebp_traits` selects `nr=4`, `LhsPacketSize=8`, `mr=3*8=24`,
`LhsProgress=8`, `RhsProgress=1`. The 9x3 request enters the 24-row/4-column
kernel with masked edge panels: LHS is packed in 8-row packets and RHS in
four-column panels; packet accumulators are initialized from the existing
`-G` C block (beta path), depth-k FMAs update them, and only 9 rows/3 columns
are stored. The source and assembly agree on `vfmadd231ps`/`vfmadd132ps`
updates and edge stores. A complete portable Rust reproduction still requires
the masked 24x4 accumulator path; scalar seeded variants did not reach
270/270, so production remains unchanged.

## Complete Eigen operator/packer extracts (2026-08-25)

The earlier 20,602-byte slice was incomplete and is superseded. A
brace-balanced extraction from `gebp_kernel::operator()` through its matching
closing brace is now in
`target/m7im15_eigen_gebp_operator_complete_luna20260825.txt` (55,064 bytes,
SHA256 `7051fb4dd2a5f13eea954f814edc5b58130834c8531ec2a2380a04f571acee68`).
It contains the Process 1 block and remaining-column edge blocks; brace depth
ended at zero. Complete packer slices are in
`target/m7im15_eigen_packers_complete_luna20260825.txt` (16,412 bytes,
SHA256 `a03f1b282b5d734783ebd4521d1e530a2c8ff824e11576fe04f336aa6b660404`).

The earlier 253/270 portable schedule result is discarded because it used
the incomplete extract. No production replacement was made from that result.

## Fresh live expression frontier (R22)

The old R21 reconstructed-native expression oracle is superseded for live
frontier claims. A fresh detached GCC11 run selected global
`ImuBlock<float>` call ordinal 0 with
`M7IM15_NATIVE_EXPR_TRACE_CALL_ORDINALS=0`; the complete comparison is
[`m7im15_live_expr_frontier_report_20260825.md`](m7im15_live_expr_frontier_report_20260825.md)
and machine-readable output is
[`target/m7im15_live_expr_frontier_20260825.json`](../../target/m7im15_live_expr_frontier_20260825.json).
All live states, delta position/velocity/quaternion, covariance, and bias
Jacobians are exact. Of 300 named expression values, 299 are exact; the sole
mismatch is `dR0_velocity[0]` (`3ec46fab` native versus `3ec46fac` Rust), while
`R0_inv` and `velocity_operand` are exact. Thus the first proven live frontier
is the Eigen 3x3-by-3 GEMV evaluation, not an input or preintegration state.
The old reconstructed-oracle result is retained only as historical provenance
and must not be read as the live arithmetic schedule.
