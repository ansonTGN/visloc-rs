# M7ec same-time stereo factor exactness

Date: 2026-08-23 JST  
Status: **Pass — the clean track-2 same-timestamp stereo factor is bitwise
exact through q/t, homogeneous point, Double-Sphere projection/raw residual,
and the raw landmark Jp.**

## First mismatch and change

The clean q/t packet was already exact (`7/7`):

```text
q_xyzw: bbefaaa9 b9eadbb0 ba8bff3a 3f7ffe35
t_xyz:  bde1c0f0 3997d300 b9e136c8
```

The first mismatch was the homogeneous point's z lane.  The current source
used the algebraically equivalent expression
`[2u, 2v, 1-r2] / (1+r2)` for the f32 stereographic bearing.  Upstream
`StereographicParam<float>::unproject` materializes the common scale first,
`norm_inv = 2 / (1 + r2)`, then computes `[u*norm_inv, v*norm_inv,
norm_inv-1]`.  For clean track 2 this changed the bearing z and therefore the
point z from `3f3c2d9b` to the clean `3f3c2d9d`.

`StereographicDirection::bearing_f32` now follows that general source
operation order.  No frame, track, value, or camera branch was added.

## Exact fixture

[`m7ec_clean_track2_stereo_factor.json`](../../pipelines/basalt/tests/fixtures/m7ec_clean_track2_stereo_factor.json)
records the minimal f32 input bits and clean output bits from
`target/m7eb_stereo_qt.json` and `target/m7dr_track2_clean_factor.json`.
The deterministic unit test
`vio::aom::tests::m7ec_clean_track2_stereo_factor_is_bitwise_exact` rebuilds
the current AOM/Sophus chain and checks, in order:

```text
q/t 7/7
point xyz 3/3
projection uv 2/2
raw residual uv 2/2
raw landmark Jp 6/6
```

The fixture test compares the native raw Jp after removing the exact fixture
weight (`sqrt_weight=2`) from the public weighted row.  It exercises the
explicit same-timestamp stereo predicate
(`same_timestamp=true`, `same_time_cam_id=false`); the existing exact
same-TimeCamId identity test remains unchanged.

## Verification

Focused M7 suite:

```text
cargo test --release -p visloc-basalt --lib vio::aom::tests::m7 -- --nocapture
12 passed, 0 failed
```

Full Basalt library suite:

```text
cargo test --release -p visloc-basalt --lib
168 passed, 0 failed, 1 ignored
```

The focused suite retains M7dg track-1 exact projection/point coverage and
M7dx clean relative-pose Jacobian coverage (`72/72` matrix lanes).  No unsafe
code, production debug output, commit, or push was added.
