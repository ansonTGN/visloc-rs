# M7IM15 frame-4 production-local IMU consistency

Pinned commit: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`  
Artifact: `target/m7im15_frame4_diag_production_consistency_20260825.json`

The first JSONL record of `target/m7im15_earliest_rust_imu_20260825.jsonl`
(`frame_id=4`, `iteration=0`) contains four IMU blocks. For each block I
compared the diagnostic `factor_input.whitened_jacobian_bits` (9x30,
column-major index `c*9+r`) with the first nine rows of the production
`local_jacobian.bits` (15x30, index `c*15+r`). The diagnostic whitened residual
was compared with production `residual.bits[0..9]`.

Result: every block is exact: 270/270 Jacobian entries and 9/9 residual entries.
The aggregate is 1080/1080 Jacobian and 36/36 residual, with zero mismatches.

The earlier native comparison reports block 0's full 15x30 local Jacobian as
431/450 exact, i.e. `native mismatch19`, first differing at index 123 (row 3,
column 8). Because the Rust diagnostic-to-production IMU 9-row lane is exact,
that 19-count native-vs-Rust difference is not helper-production drift. Index
123 is itself in the IMU 9-row lane, so the remaining issue is upstream/native
factor/input arithmetic or pairing, not the local 15-row assembly.

Source audit confirms one path: `window.rs:2403-2415` calls the production
`whitened_preintegration_factor_upstream_f32_fej`; `factors.rs:106-136` and
`factors.rs:204-227` use the same f32 stage primitives and order; and
`aom.rs:3292-3353` copies those nine rows into the local stack before adding
the six bias rows. No production source was modified for this audit.
