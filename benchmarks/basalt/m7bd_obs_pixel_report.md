# M7bd pinned optical-flow observation pixel

Date: 2026-08-22 JST  
Scope: frame 0 → frame 1, cam1, track 1, direct raw-u16 pyramid/LSSD endpoint.

## Result

The endpoint now reproduces the pinned Basalt native trace exactly:

```text
frame 0: x = 0x41eb67a9 = 29.425615310668945, y = 0x42d6b68d
frame 1: x = 0x41da8172 = 27.31320571899414,  y = 0x42d4c7e1 = 106.39038848876953
```

The Rust fresh trace previously ended at `x = 0x41da8171` (one ULP lower),
with the same y bits.

## Earliest scalar difference

The pinned checkout is Basalt commit `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
The existing endpoint fixture is
`mh01_stereo_endpoint_v1_first80.jsonl`.

Tracing level 3, source frame 0, before patch normalization found the first
difference in raw bilinear interpolation, sample 12:

```text
upstream: 9137.7021484375  (0x460ec6cf)
Rust:     9137.703125       (0x460ec6d0)
```

The input u16 pixels, pyramid values, coordinates, and valid-sample order were
identical.  The pinned `Image::interp<float>` source expression is compiled by
the native AVX/FMA build as one rounded product followed by three fused
multiply-adds:

```text
term1 = (ddx * dy) * p01
acc   = fma(ddx * ddy, p00, term1)
acc   = fma(dx  * ddy, p10, acc)
out   = fma(dx  * dy,  p11, acc)
```

`RawU16Image::interp` and the four central-difference interpolants now spell
that contract with `f32::mul_add`.  All 52 level-3 raw samples and gradients
then match the pinned bits.  The next proven literal-order difference was the
mean-normalized SE(2) Jacobian correction: upstream evaluates
`grad_sum * data / sum`, while Rust had evaluated `grad_sum * (data / sum)`;
`patch.rs` now preserves the upstream source order.

## Exact regression

The ignored external-data test is
`pipelines/basalt/tests/m7bd_obs_pixel_exact.rs`:

```text
$env:VISLOC_BASALT_MH01_ROOT = 'E:\datasets\euroc_mav\machine_hall\MH_01_easy'
$env:VISLOC_BASALT_CALIBRATION = (Resolve-Path target\euroc_ds_calib.json).Path
$env:VISLOC_BASALT_CONFIG = (Resolve-Path target\euroc_config.json).Path
cargo test -p visloc-basalt --test m7bd_obs_pixel_exact `
  mh01_frame1_cam1_track1_pixel_matches_pinned_bits -- --ignored --nocapture
```

It asserts the x/y f32 bit patterns above rather than a tolerance.  The
existing first-80 endpoint golden test remains the broader count/trajectory
gate.
