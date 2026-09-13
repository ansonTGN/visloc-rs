# M7cb existing-stereo GEMV exactness closure

Date: 2026-08-23 JST

## Decision

No production arithmetic change was justified. The existing general
`gemv_packet8` implementation in `pipelines/basalt/src/patch.rs` matches the
pinned Eigen 5.0.1 result for the newly covered existing-stereo L3/I0 input in
all three output lanes. The only source change in `patch.rs` is the regression
test; `update.rs` was not modified.

## Exact existing-stereo fixture

The fixture is
`pipelines/basalt/tests/fixtures/m7cb_existing_stereo_gemv.json`. It records
the native frame-1/cam-1/track-3 forward L3/I0 GEMV operands:

- source image: frame 0, timestamp `1403636579763555584`;
- source point: `0x42395ac4,0x435c70bb` (level-3 point
  `0x40b95ac4,0x41dc70bb`);
- native Eigen 5.0.1 O3 `3x52` inverse-Jacobian product;
- all 52 native residual bits from the existing forward trace;
- expected increment: `0xbeb5dbcd,0xbd7fc4c0,0x3b582340`.

The test `packet8_gemv_matches_native_existing_stereo_l3_i0_all_lanes`
supplies those bits to the general helper and passes for x, y, and theta.
Thus the requested native theta `0x3b582340` is reproduced by the current
packet-8 reduction when the native coefficients and residual are held fixed;
there is no evidence for a valid track/camera-specific or alternate general
reduction patch in this boundary.

## Verification

Focused tests passed:

```text
cargo test -p visloc-basalt patch --release --lib
  6 passed
cargo test -p visloc-basalt update --release --lib
  7 passed
cargo test -p visloc-basalt --test m7bd_obs_pixel_exact --release -- --ignored
  1 passed
cargo test -p visloc-basalt --test m7bw_cam0_temporal_exact --release
  1 passed
```

A fresh two-frame release endpoint run completed in `0.634 s`. Frame 0,
cam-1, track 3 is exact at `x=0x42395ac4,y=0x435c70bb`; frame 1 remains the
known post-composition frontier at `x=0x422f68f7,y=0x435c045a` versus native
`x=0x422f68f8,y=0x435c045a`.

The fresh 80-frame run completed in `8.167 s` and produced 80 records. Its
decoded f32 comparison is byte-for-byte identical to the M7ca baseline
`target/m7ca_rust_endpoint_first80.jsonl`:

```text
structure: exact
exact coordinate fields: 36,654 / 48,512
exact point pairs:       16,237 / 24,256
maximum ULP difference:  47
maximum absolute delta:  0.00048828125 px
first mismatch: frame 1, cam1, track 3, x
  native 0x422f68f8, Rust 0x422f68f7
M7cb-vs-M7ca decoded fields/pairs: 48,512/48,512 and 24,256/24,256
```

The fixture parsed as 3 rows × 52 coefficients, 52 residuals, and 3 expected
lanes. SHA-256 for both `target/m7cb_first80.jsonl` and
`target/m7ca_rust_endpoint_first80.jsonl` is
`681c90bc646ef2af92f04f91b4fa8aa4c211f8acc17358c3918c038e134b8403`.

No endpoint or aggregate regression was introduced, and no commit or push
was made.
