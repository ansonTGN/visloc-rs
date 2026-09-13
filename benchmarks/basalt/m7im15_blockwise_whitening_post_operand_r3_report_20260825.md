# M7IM15 blockwise IMU whitening parity (2026-08-25)

Pinned Basalt commit: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.

The detached native logger was rerun with `M7IM15_NATIVE_INPUT_CALL_ORDINALS=0,1,2,3`; the fresh `M7IM15INP2` capture contains all four records. The production native oracle is the direct `ImuBlock<float>::Jp/r/H/b` capture, never the historical reconstructed artifact.

## Boundary counts

| block | raw r | raw J | W | wide W*r | wide W*J | production Jp | production r | H | b |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 9/9 | 270/270 | 81/81 | 9/9 | 270/270 | 450/450 | 15/15 | 900/900 | 30/30 |
| 1 | 8/9 | 267/270 | 81/81 | 4/9 | 267/270 | 435/450 | 10/15 | 842/900 | 7/30 |
| 2 | 7/9 | 265/270 | 81/81 | 0/9 | 260/270 | 440/450 | 6/15 | 828/900 | 6/30 |
| 3 | 7/9 | 263/270 | 81/81 | 2/9 | 261/270 | 429/450 | 8/15 | 802/900 | 6/30 |

First scan-order mismatch: `{'block': 1, 'group': 'input_stages', 'field': 'raw_residual', 'difference': {'index': 1, 'native': 'ae800000', 'rust': '00000000'}}`.

The Rust production path now forms the four source-shaped products independently. For 9x9 start/end blocks it preserves Eigen's complete four-column panels (Packet8 even/odd accumulators and swapped scalar-row tail); the 9x3 bias products use the scalar remainder path. The diagnostic wide `W*raw_J` remains unchanged and is retained only as a separate stage witness.

## Artifacts

- Comparison JSON: `target/m7im15_blockwise_whitening_compare_post_operand_r3_20260825.json`
- Fresh native local-input binary: `target/m7im15_native_live_gemv_fix_input_local4_20260825.bin` (SHA-256 `7decba8e65d25804fb765e9ecaaa73fd7b21533ca0ebb7714fbf59c7ed8fc23f`)
- Fresh native input anchor: `target/m7im15_native_live_gemv_fix_input4_20260825.bin` (SHA-256 `41ffe300003316bfe4285029aede065eae4e555d337bb07c781f22d908200a5e`)
- Direct native local products: `target/m7im15_native_local_product_frame4_iter0_post_expr_20260825.json` (SHA-256 `0783739455da9c9bd5031f1c3cd0f4272f275de9cdaa834491aaac85c30b230f`)
- Rust sidecar: `target/m7im15_rust_blockwise_frame4_iter0_post_operand_r3_20260825.jsonl` (SHA-256 `0883bfb2ca3ed4bf70d4434366fe32c8018088bf576f075c838b400d2bf0d487`)

## Operand/quaternion frontier

The Rust source now follows the bounded source schedule for the translation
operand: `d = fl(p1-p0)`, `h = fl(0.5*g)`, `u = fma(-v,dt,d)`,
`h = fl(h*dt)`, and `out = fma(-h,dt,u)`.  All three translation lanes and
all nine `R0_inv` lanes are exact for the fresh ordinal-1 comparison.  The
first remaining named expression mismatch is `dR0_translation[1]`, native
`ba346a77` versus Rust `ba346a76`; this propagates to raw residual lane 1
(`ae800000` versus `00000000`).  The next Jacobian mismatch is
`J_start_rotation[0]`, native `3a7ec767` versus Rust `3a7ec766`.

The complete ordinal-1 comparison is
`target/m7im15_ordinal1_expr_compare_post_operand_r3_20260825.json` (SHA-256
`9df1504943fdbd6df39074b459b5508b08841bd791a9efcf2623da57ab2afe46`).  Its
aggregate is 937/957 exact values; 20 values remain mismatched.  The fresh
detail trace is
`target/m7im15_rust_blockwise_frame4_iter0_post_operand_r3_detail_20260825.jsonl`
(SHA-256 `754dae5d674f343edae6ea0833988cb77c9c8e93cbcee8d65f03dac8aad9db06`).
The release executable used for this run is SHA-256
`69acff932b16fc91d21fcbeaf3a9182150cde6df9679fa7f7c7d3b9d2dd39bb2`.

The four-ordinal expression scan is
`target/m7im15_expr_schedule_compare_post_operand_r3_all4_20260825.json`
(SHA-256 `23df5c5a4188b3b92d842a15c94ce06168a0bcee854aeaf5d9dc2fc6d5d89c9a`).
Across ordinals 0--3, `translation_operand` is 12/12 exact and `R0_inv` is
36/36 exact.  The remaining schedule frontier is downstream GEMV: the
aggregate `dR0_translation` count is 10/12 and `J_start_rotation` is 30/36.

Focused release IMU tests: **10 passed, 0 failed**.  Full release
`visloc-basalt` library tests: **201 passed, 1 ignored, 0 failed**.  A
workspace-wide `cargo fmt --all -- --check` remains non-clean because of the
pre-existing mixed newline style and unrelated formatting diffs; no unrelated
files were reformatted.
