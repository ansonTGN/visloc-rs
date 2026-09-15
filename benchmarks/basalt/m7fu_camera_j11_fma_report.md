# M7fu live ordinal-109 camera-J11 FMA

Date: 2026-08-24 JST  
Status: **Complete — source-faithful general J11 contraction accepted.**

## Change

The general DoubleSphere camera Jacobian now uses the same explicit FMA
association as the accepted J00 contraction:

```rust
jacobian[(1, 1)] = (-yy).mul_add(d_norm_d_r2, 1.0_f32 / norm) * fy;
```

This is unconditional production arithmetic: no value/ordinal/camera branch,
`debug!`, or `unsafe` was added.

## Permanent live fixture

The fixture/test pair is:

- [`m7fu_live_ordinal109_camera_jp.json`](../../pipelines/basalt/tests/fixtures/m7fu_live_ordinal109_camera_jp.json)
- `vio::aom::tests::m7fu_live_ordinal109_camera_jacobian_and_jp_are_bitwise_exact`

It records the clean-native M7ft in-context ordinal-109 camera-J/Jpp/product:

```text
camera J (2x4): 43c3a1b0,c2917418,c291e436,43e06dcc,437d6358,432a65a0,00000000,00000000
Jpp (4x3):       3fba6356,be4b3e16,3f722147,00000000,be534c85,3fcdb62b,3f27a9f7,00000000,bde1c0f0,3997d300,b9e136c8,3f800000
raw Jp (2x3):    444df834,c2074d44,c2000f98,4453fe6c,c22d09a2,41012d40
weighted Jp:     44cdf834,c2874d44,c2800f98,44d3fe6c,c2ad09a2,41812d40
```

The first mismatch before the change was camera-J lane `r1c1`: native
`43e06dcc`, Rust `43e06dce`; Jpp was already 12/12 exact.

## Verification

Focused M7 gates:

```text
cargo test -p visloc-basalt --lib m7 -- --nocapture
29 passed, 0 failed
```

Full Basalt library:

```text
cargo test -p visloc-basalt --lib
177 passed, 0 failed, 1 ignored
cargo test -p visloc-basalt --lib --release
177 passed, 0 failed, 1 ignored
```

The release example was rebuilt from the current source and replayed for five
MH_01_easy frames. The run processed 5/5 frames, delivered 42 IMU samples,
and emitted 1,114 observations. The fresh detail is
[`target/m7fu_fresh5_detail.jsonl`](../../target/m7fu_fresh5_detail.jsonl).

The exact keyed visual comparison retained the full projection boundary and
closed the weighted-Jp frontier:

| boundary | exact | total |
|---|---:|---:|
| projection pairs | 584 | 584 |
| projection lanes | 1,168 | 1,168 |
| weighted Jp lanes | **3,504** | **3,504** |
| raw Jp lanes | 3,266 | 3,504 |

The machine-readable Jp result is
[`target/m7fu_remaining_jp.json`](../../target/m7fu_remaining_jp.json).

Aggregate frame-4 H/b comparison against the clean native capture is 3,295 /
5,625 H lanes and 6 / 75 b lanes. H moved by four exact lanes from the M7fr
candidate (3,299 / 5,625); this small aggregate shift is accepted because the
live camera-J/Jp boundary is now exact. The first H mismatch remains native
`4de48a64` versus Rust `4de48a6f`; b remains 6/75 exact.

No commit or push was performed.
