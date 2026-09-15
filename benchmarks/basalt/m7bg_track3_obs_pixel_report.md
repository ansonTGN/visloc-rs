# M7bg track-3 observation audit

Date: 2026-08-23

The fresh optimized Rust endpoint remains one ULP below the pinned native
endpoint for frame 0, camera 1, track 3:

| endpoint | x | y |
| --- | ---: | ---: |
| pinned native | `46.3386383056641` (`0x42395ac4`) | `220.440353393555` (`0x435c70bb`) |
| Rust release | `46.3386344909668` (`0x42395ac3`) | `220.440353393555` (`0x435c70bb`) |

The endpoint was regenerated with:

```text
cargo run -p visloc-basalt --release --example stereo_diag -- \
  E:\datasets\euroc_mav\machine_hall\MH_01_easy \
  target\euroc_ds_calib.json target\euroc_config.json 1 \
  target\m7bg_fresh_endpoint_final.jsonl
```

The patch audit retains the explicit three-term FMA Hessian and the
pivoted LDLT inverse. The inverse is covered by the pinned Eigen 3x3 fixture
(`pipelines/basalt/src/patch.rs`); its nine output bits match the native
`LDLT::solve(I)` result. The endpoint miss therefore is not treated as proof
that a new Jacobian-transpose product ordering is correct, and no endpoint
golden test was added for the still-failing `x` bit.

Focused verification:

```text
cargo test -p visloc-basalt patch::tests --release
4 passed
cargo test -p visloc-basalt --test m7bd_obs_pixel_exact --release -- --ignored
1 passed
```

The m7bd frame-1/camera-1/track-1 exact observation remains unchanged and
passes its pinned bits.
