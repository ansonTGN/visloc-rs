# M7IM15 blockwise IMU whitening parity (2026-08-25)

Pinned Basalt commit: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.

The detached native logger was rerun with `M7IM15_NATIVE_INPUT_CALL_ORDINALS=0,1,2,3`; the fresh `M7IM15INP2` capture contains all four records. The production native oracle is the direct `ImuBlock<float>::Jp/r/H/b` capture, never the historical reconstructed artifact.

## Boundary counts

| block | raw r | raw J | W | wide W*r | wide W*J | production Jp | production r | H | b |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 9/9 | 270/270 | 81/81 | 9/9 | 270/270 | 450/450 | 15/15 | 900/900 | 30/30 |
| 1 | 8/9 | 258/270 | 81/81 | 4/9 | 254/270 | 422/450 | 10/15 | 720/900 | 7/30 |
| 2 | 7/9 | 265/270 | 81/81 | 0/9 | 260/270 | 440/450 | 6/15 | 828/900 | 6/30 |
| 3 | 6/9 | 238/270 | 81/81 | 0/9 | 225/270 | 393/450 | 6/15 | 560/900 | 6/30 |

First scan-order mismatch: `{'block': 1, 'group': 'input_stages', 'field': 'raw_residual', 'difference': {'index': 1, 'native': 'ae800000', 'rust': '00000000'}}`.

The Rust production path now forms the four source-shaped products independently. For 9x9 start/end blocks it preserves Eigen's complete four-column panels (Packet8 even/odd accumulators and swapped scalar-row tail); the 9x3 bias products use the scalar remainder path. The diagnostic wide `W*raw_J` remains unchanged and is retained only as a separate stage witness.

## Artifacts

- Comparison JSON: `target/m7im15_blockwise_whitening_compare_20260825.json`
- Fresh native local-input binary: `target/m7im15_native_live_gemv_fix_input_local4_20260825.bin` (SHA-256 `7decba8e65d25804fb765e9ecaaa73fd7b21533ca0ebb7714fbf59c7ed8fc23f`)
- Fresh native input anchor: `target/m7im15_native_live_gemv_fix_input4_20260825.bin` (SHA-256 `41ffe300003316bfe4285029aede065eae4e555d337bb07c781f22d908200a5e`)
- Direct native local products: `target/m7im15_native_local_product_frame4_iter0_20260825.json` (SHA-256 `6179ffe5d23a7e48840422706a17586e72cb77a8adb4a9ef3b7f98e4d160da54`)
- Rust sidecar: `target/m7im15_rust_blockwise_frame4_iter0_20260825.jsonl` (SHA-256 `a46fc830948ee2fd0ec565167969a5ba6c48230c650319d42441f349e05fdef6`)
