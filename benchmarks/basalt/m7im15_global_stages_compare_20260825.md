# M7IM15 global H/b stage comparison (2026-08-25)

This audit compares frame 4, iteration 0, `iteration_start` from the fresh
native and Rust solver-frontier artifacts.  The pinned Basalt source is
commit `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.  Both sides are compared as
recorded binary32 lanes using semantic `(row, column)` coordinates; Eigen and
Rust storage are both declared column-major.

## Initial frontier and first mismatch

The comparison script is
`work/m7im15_compare_global_stages_20260825.py`.  Its initial run, before the
checkpoint-only fix, selected Rust JSONL line 1 and produced:

| stage | H exact | b exact |
| --- | ---: | ---: |
| visual accumulator | 5625/5625 | 75/75 |
| visual + IMU | 5625/5625 | 75/75 |
| + pose damping | 5550/5625 | 75/75 |
| + marginal prior | 5550/5625 | 75/75 |

The first semantic mismatch was stage `plus_pose_damping`, H coordinate
`(row=0,col=0)`: native `4db4db5c` (`379284352.0`) versus Rust
`4db4e136` (`379332288.0`), 1498 binary32 ulps and an absolute difference of
`47936.0`.  All 75 mismatches were diagonal lanes.  The Rust diagnostic had
incorrectly added the LM `damping_diag` to the direct `get_dense_H_b`
checkpoint.

## Fix and contract

The Rust checkpoint now mirrors the pinned native call order:

1. visual landmark `Reductor` result;
2. IMU accumulator;
3. native pose-damping hook (a no-op in this driver because
   `setPoseDamping(lambda)` is disabled);
4. marginal prior.

LM `H_copy = H + Hdiag_lambda` remains a separate solver boundary and is not
part of `normal_system_stages`.  A focused regression,
`vio::window::tests::normal_system_stage_checkpoint_excludes_lm_solver_damping`,
asserts stage-3 and stage-4 binary32 bits with a non-zero synthetic prior and
would fail if solver damping is folded into stage 3.

## Artifact provenance

| artifact | SHA-256 |
| --- | --- |
| `target/m7im15_native_global_stages_frame4_iter0_20260825.json` | `e98469ce72144e856897b95ca335b846f54e090f0b96c517aa83284964ffe600` |
| native binary referenced by JSON (`target/m7im15_native_global_stages_frame4_iter0_20260825.bin`) | `9e37b43ce3efa8d06f9e2e5255190730edffd4aa1d10da1cb0695e6f8fe8bdd1` |
| `target/m7im15_rust_global_stages_frame4_iter0_20260825.jsonl` | `be92e2a976984f0ffb38781a9e97c7040cc668ce61d0dfab300b2db16e2d5aee` |
| comparison JSON (initial run) | `a37b51cde2c4b5aa561e6ef14121a0bd5845c634a64aab2969571b8b3a48bd8f` |

The Rust frontier must be recaptured after this source fix; the comparison
JSON and this table should then be refreshed.  The stage-1/stage-2 exact
counts above establish that visual accumulation and the IMU boundary were
already bit-exact; the only initial mismatch was diagnostic stage semantics.
