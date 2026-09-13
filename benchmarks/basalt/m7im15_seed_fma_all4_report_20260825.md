# M7IM15 source-confirmed GEMV seed/FMA schedule (2026-08-25)

Pinned Basalt commit: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
Fresh Rust release max-5 frame-4/iteration-0 sidecar is compared with direct detached native GCC11 captures. No reconstructed oracle was used.

## Source schedule

Eigen's fixed 3x3 coefficient product emits each dot as seed `k=1`, FMA `k=2`, FMA `k=0`; the native helper disassembly contains `mulss rhs[1]`, `vfmadd231ss rhs[2]`, `vfmadd231ss rhs[0]`. The exact same schedule is the unique source-compatible recommendation among the tested reductions that matches all translation/velocity lanes for live ordinals 0 and 1.

## Expression gates

- Aggregate named expressions: `364/364` exact; first mismatch: `None`.
| block | named expression | raw r | raw J | W | wide W*r | wide W*J | delta matrix | production Jp | production r | H | b |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 91/91 | 9/9 | 270/270 | 81/81 | 9/9 | 270/270 | 8/9 | 450/450 | 15/15 | 900/900 | 30/30 |
| 1 | 91/91 | 9/9 | 270/270 | 81/81 | 9/9 | 270/270 | 6/9 | 438/450 | 15/15 | 876/900 | 30/30 |
| 2 | 91/91 | 9/9 | 270/270 | 81/81 | 9/9 | 270/270 | 6/9 | 450/450 | 15/15 | 900/900 | 30/30 |
| 3 | 91/91 | 9/9 | 270/270 | 81/81 | 9/9 | 270/270 | 6/9 | 438/450 | 15/15 | 876/900 | 30/30 |

First post-schedule input/product mismatch: `{'block': 0, 'group': 'input_stages', 'field': 'delta_rotation_matrix', 'difference': {'index': 4, 'native': '3f7ffefc', 'rust': '3f7ffefd'}}`.

## Artifacts

- Comparison: `target/m7im15_seed_fma_all4_compare_20260825.json` (SHA-256 `c2755d03051969ccf9ecc912b92e8ab5d357bf58b4ca58a4608f3286021bfe4b`)
- Report: `benchmarks/basalt/m7im15_seed_fma_all4_report_20260825.md`
- Rust sidecar: `target/m7im15_rust_seed_fma_v1_20260825.jsonl` (SHA-256 `0f5ebfc1f4b768da0507fe21c9d059db3f22150c0b6a38815e7507f64ce48a0b`)
