# M7ez Double-Sphere `d1_2` exactness

Date: 2026-08-23 JST  
Status: **Pass — native scalar grouping restored; no commit or push.**

## Change

`project_double_sphere_with_jacobian_f32` in
`pipelines/basalt/src/vio/aom.rs` now materializes the native scalar sequence
`zz = z*z`, `d1_2 = r2 + zz`, `d1 = sqrt(d1_2)`. The previously accepted
`k`, `d2`, `norm`, `d_norm_d_r2`, `tmp2`, and J00 groupings are unchanged.
There is no value-specific branch. The ordinal-3 camera-J fixture and the
ordinal-38 projection fixture are in `pipelines/basalt/tests/fixtures/`.

## Focused verification

```text
cargo test -p visloc-basalt m7 --lib --release
23 passed, 0 failed, 0 ignored

cargo test -p visloc-basalt m7dx --lib --release
1 passed, 0 failed
cargo test -p visloc-basalt m7ec --lib --release
1 passed, 0 failed
cargo test -p visloc-basalt m7et --lib --release
1 passed, 0 failed
cargo test -p visloc-basalt m7ez --lib --release
2 passed, 0 failed
```

The M7dg relative-pose/anchored boundaries are included in the 23-test M7
filter; there is no test whose name contains the historical `m7dg` label.
The new tests assert ordinal-3 camera J (six active lanes) and ordinal-38
projection (two lanes) bitwise.

Full release Basalt library:

```text
cargo test -p visloc-basalt --lib --release
171 passed, 0 failed, 1 ignored
```

The one ignored test is the pre-existing M8c diagnostic test; M7ez adds no
ignored tests. No debug output or `unsafe` code was added.

## Fresh-five replay

The release example was rebuilt with:

```text
cargo build --release -p visloc-rs --example basalt_euroc_vio_demo
```

The sensor-only MH01 replay used the established five-frame/detail-frame-4
configuration (`VISLOC_BASALT_DETAIL_ITERATIONS=1`, frame 4, iteration 0).
It processed 5/5 frames, delivered 42 IMU samples, emitted 1,114 total
observations, and produced 61 visual factors / 584 visual observations at
frame 4.

Against clean native `target/m7ef_clean_visual_all.json`, all 584 records
matched by exact direction/rho/pixel keys. The fresh visual comparison is:

| buffer | exact | total |
|---|---:|---:|
| projection pairs | 577 | 584 |
| projection scalar lanes | 1,161 | 1,168 |
| weighted landmark Jp lanes | 3,370 | 3,504 |
| raw landmark Jp lanes | 3,140 | 3,504 |
| H lanes (clean `m7ct`, transposed Rust) | 3,253 | 5,625 |
| b lanes (clean `m7ct`) | 6 | 75 |

The first remaining projection mismatch is ordinal 110 (track 21,
frame-4/cam-1), one ULP in the second pixel lane. The first weighted-Jp
mismatch is ordinal 43 (track 119, same-timestamp stereo), lane 1. This
closes the prior ordinal-3 camera-J / ordinal-38 projection frontier without
altering Jpp or the landmark product.

Artifacts:

- [visual comparison JSON](../../target/m7ez_visual_all_comparison.json)
- [visual comparison report](m7ez_visual_all_compare_report.md)
- [fresh detail JSONL](../../target/m7ez_fresh5_detail.jsonl)
- [fresh run summary](../../target/m7ez_fresh5_run/summary.txt)

No commit or push was performed.
