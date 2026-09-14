# M7be: pinned Track 3 landmark boundary

Date: 2026-08-22 JST  
Pinned Basalt revision: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`

## Result

`pipelines/basalt/src/vio/landmarks.rs` now has a permanent exact Track 3
fixture, `m7_third_track_triangulation_matches_pinned_native_bits`.  It locks
the complete landmark boundary: raw camera bearings, the relative camera pose,
DLT matrix `A`, Eigen JacobiSVD's final raw `V` column, homogeneous
normalization, inverse distance, and stereographic projection.  The fixture
passes with the existing general f32 Eigen/Sophus implementation and leaves
the Track 1, 2, and 4 fixtures unchanged.

## Native Track 3 trace

The authoritative upstream record is the iteration-0 Track 3 entry in
`target/m7al_upstream_f4_20260821T000200Z/iteration.jsonl`:

```text
cam0 pixel = (38, 207)
cam1 pixel = (46.3386383056641, 220.440353393555)
direction = (-0.3713243901729584, -0.047964297235012054)
rho = 0.14130617678165436
```

The pinned native camera/DLT probe gives these f32 lanes (hex bit patterns):

```text
f0 = bf26bda3, bdacb6dc, 3f410c36
f1 = bf2939fc, bd90e159, 3f3f3be4

T_candidate_from_target =
  3f7fffd3 3b0c6cef ba66c162 bde1c0f0
  bb0b910f 3f7ff8d7 3c6facec 3997d306
  3a6ef276 bc6fa4e5 3f7ff8f6 b9e136c3
  00000000 00000000 00000000 3f800000

A (row-major) =
  bf410c36 80000000 bf26bda3 80000000
  80000000 bf410c36 bdacb6dc 80000000
  bf3f633f 3c04309b bf290a3d 3da938a4
  3ac81016 bf3ef2bb bda73ea0 b942f6a9

raw V[:,3] = bf251a77, bdaa9cc6, 3f3f26ea, 3e0f4631
normalized = bf26be5b, bdac4eac, 3f410d0d, 3e10b291
stereographic = bebe1e3b, bd447636
```

## Why the fresh runtime still reports 40/61

The fresh Rust detail trace records the cam1 Track 3 pixel as
`(46.3386344909668, 220.440353393555)`, one f32 ulp below the upstream
`46.3386383056641`.  Feeding that actual Rust pixel through the pinned native
camera/DLT path produces the corresponding lanes

```text
f1 = bf2939fe, bd90e159, 3f3f3be3
normalized = bf26be5b, bdac4ead, 3f410d0c, 3e10b311
stereographic = bebe1e3b, bd447637
```

which widens to the observed Rust direction/rho.  Thus the isolated landmark
implementation is exact for identical inputs; the remaining full-replay
Track 3 difference is upstream of `landmarks.rs` in the stereo observation
pixel.  No track-specific correction is appropriate.

## Verification

The exact fixture was run with:

```text
C:\Users\rsasa\.cargo\bin\cargo.exe test -p visloc-basalt --release --lib vio::landmarks::tests::m7_third_track_triangulation_matches_pinned_native_bits -- --nocapture
```

Result: **1 passed, 0 failed**.
