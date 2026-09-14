# M7he native Huber boundary

This bounded audit captures the robust scalar chain in the pinned clean
`libbasalt.so` rather than inferring a weight from downstream QR rows.  The
target is
`LandmarkBlockAbsDynamic<float,6>::linearizeLandmark()::operator()<DoubleSphereCamera<float>>`
at ELF offset `0x275af0`, from clean binary
`89e0324ccd04c3b615bf7e945aa32480a2f86524987aa00eaf152dcd6b1d242c`.

The GDB capture (`target/m7he_capture_native_huber.cmd`) stopped at the native
register boundaries for all 22 cross-time observations whose pre-QR rows had
been mismatching.  The resulting machine-readable capture is
`target/m7he_native_huber_capture.json`; the durable tracked fixture is
[`m7he_native_huber_schedule.json`](../../pipelines/basalt/tests/fixtures/m7he_native_huber_schedule.json).

## Native schedule

The clean disassembly at `0x275c6d..0x275c89` is:

```text
vmulss       x, x, x2
vfmadd231ss  y, y, x2       # x2 = y*y + x*x, one fused add
vaddss       +0, x2, x2
```

The remaining outlier path is:

```text
vsqrtss      x2, norm, norm
vdivss       norm, huber_delta, huber_weight
call sqrtf   sqrt_weight_numerator = sqrtf(huber_weight)
vdivss       sigma, sqrt_weight_numerator, sqrt_weight
```

The direct capture saw 22/22 exact values at each boundary: residual squared,
norm, Huber weight, outer square-root numerator, and final whitened weight.
The source-wide Rust spelling is therefore:

```rust
y.mul_add(y, x * x)
```

There is no track- or observation-specific branch.  The track-120 fixture is
also unchanged (`squared=3f8cba0d`, `sqrt_weight=3ffa0138`); an asymmetric
track-73 witness is asserted in `m7_frame4_visual_f32_vector2_norm_matches_eigen_packet`.

Representative direct register capture, track 73 / observation 2:

| stage | bits |
| --- | --- |
| raw residual | `be24d800,3f9279c0` |
| residual squared | `3faaef5d` |
| norm | `3f93eaf6` |
| Huber weight | `3f5d8746` |
| sqrt numerator | `3f6e242c` |
| final sqrt weight | `3fee242c` |

## Fresh all-track result

After changing only the production f32 Vector2 reduction, a fresh release
five-frame detail trace was generated at
`target/m7he_post_weight_detail.jsonl`.  The clean M7he comparator reports:

```text
preqr_all=0/108080 exact_tracks=61/61
apply_all=0/87600 rhs=0/1168 exact_tracks=61/61
```

The exact track-120 Q2 block is `0/1500`, and its RHS is `0/20`.

Focused and full library test results are recorded with the implementation
checkpoint; no commit or push was performed.
