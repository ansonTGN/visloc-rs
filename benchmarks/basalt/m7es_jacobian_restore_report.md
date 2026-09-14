# M7es Jacobian restore

Date: 2026-08-23 JST  
Status: **Pass — standard release focused gates and full Basalt library pass.**

## Scope

Restored the general f32 Double-Sphere projection-Jacobian operation grouping
in `pipelines/basalt/src/vio/aom.rs`. The source-faithful inner reduction for
`J(0,0)` is:

```rust
(-xx).mul_add(d_norm_d_r2, 1.0_f32 / norm) * fx
```

The `J(1,1)` diagonal remains the literal source grouping
`fy * (1.0 / norm - yy * d_norm_d_r2)`, and the existing general f32
homogeneous landmark reduction is unchanged. No input-specific branch,
debug/probe code, unsafe code, ignored test, commit, or push was added.

## Verification

With `RUSTFLAGS` removed:

```text
cargo test --release -p visloc-basalt --lib vio::aom::tests::m7 -- --nocapture
12 passed, 0 failed, 157 filtered out

cargo test --release -p visloc-basalt --lib
168 passed, 0 failed, 1 ignored
```

The four previously failing exact gates now pass: Double-Sphere boundary,
landmark-J intermediates, same-timestamp stereo homogeneous Jacobian, and the
clean track-2 M7ec factor. Native track-2 camera-J lane `43b08705` and final
landmark-J lane `c1ccfe70` are exact.
