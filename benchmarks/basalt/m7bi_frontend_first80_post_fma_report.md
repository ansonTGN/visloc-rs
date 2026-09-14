# M7bi first-80 frontend parity after FMA/order fix

Date: 2026-08-23 JST  
Scope: release `stereo_diag` endpoint trace, frames 0--79, against the
pinned Basalt endpoint fixture at commit
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.

## Inputs and run

The authoritative fixture is
`benchmarks/basalt/mh01_stereo_endpoint_v1_first80.jsonl`.  Its declared and
computed SHA-256 both equal
`cee7e2918a8e9383949508f74b6ec956f6c8a2a8e053648d15d05544d2205b2c`.

The fresh post-FMA Rust trace is
`target/m7bi_rust_endpoint_first80_post_fma_20260823.jsonl` (SHA-256
`1470decf41abc0493ec1e735102bd3914a54f1107353de7385f7ed2eb4dba036`).  The
comparison artifact is
`target/m7bi_frontend_first80_post_fma_20260823.json`.

The trace was generated with the pinned MH_01 dataset, calibration, and
configuration using the release binary
`target/release/examples/stereo_diag.exe`.  PowerShell
`Measure-Command` measured 8.040 s for the direct release-binary pass.  The
fresh `cargo run --release` invocation measured 21.975 s including a 10.84 s
release compile; no before/after runtime baseline was available, so this is a
current runtime measurement only.

## Structural parity

All 80 records are present.  Schema, frame index, timestamp, camera counts,
track-ID sets, and track-ID order have zero differences.  The common endpoint
population is:

| camera | points | coordinate fields |
|---:|---:|---:|
| cam0 | 15,347 | 30,694 |
| cam1 | 8,909 | 17,818 |
| total | 24,256 | 48,512 |

Thus the post-FMA change neither creates nor removes an endpoint or changes
the first-80 tracking structure.

## Exact f32 coordinate parity

Coordinates are compared by their decoded f32 bit patterns, not by a numeric
tolerance.

| metric | cam0 | cam1 | total |
|---|---:|---:|---:|
| exact point pairs | 5,436 / 15,347 | 914 / 8,909 | 6,350 / 24,256 |
| exact coordinate fields | 14,998 / 30,694 | 4,875 / 17,818 | 19,873 / 48,512 |
| exact pair fraction | 35.42% | 10.26% | 26.18% |
| exact field fraction | 48.86% | 27.36% | 40.97% |

The first post-FMA coordinate mismatch is frame 0, cam1, track 3, field `x`:

```text
pinned: 0x42395ac4 = 46.33863830566406
Rust:   0x42395ac3 = 46.33863449096680
delta:  1 ULP = 3.814697265625e-6 px
```

The pre-FMA baseline used for the effect comparison is
`target/rust_mh01_endpoint80_fixed.jsonl` (SHA-256
`0af491bbb6ec5dad8d4719a9864a7aabf85bebcd9b7a2d5c00af645dd1a215ee`).  Its
first mismatch was frame 0, cam1, track 10, field `y`:

```text
pinned: 0x42f51a8b = 122.55184173583984
Rust:   0x42f51a8c = 122.55184936523438
delta:  1 ULP = 7.62939453125e-6 px
```

## FMA/order effect

The post-FMA trace improves total exact parity relative to that baseline:

| metric | pre-FMA | post-FMA | change |
|---|---:|---:|---:|
| exact point pairs | 6,321 | 6,350 | **+29** |
| exact coordinate fields | 19,733 | 19,873 | **+140** |
| exact pair fraction | 26.0595% | 26.1791% | **+0.1196 pp** |
| exact field fraction | 40.6765% | 40.9651% | **+0.2886 pp** |
| maximum absolute delta | 0.0012664794921875 px | 0.001220703125 px | **-0.0000457763671875 px** |
| maximum coordinate ULP distance | 166 | 122 | **-44** |

The pair gain is cam0 `+7` and cam1 `+22`.  At field level, `x` gains 211
exact fields while `y` loses 71, for a net gain of 140.  The result is an
overall improvement, with structural parity unchanged; it is not an exact
first-80 endpoint match.
