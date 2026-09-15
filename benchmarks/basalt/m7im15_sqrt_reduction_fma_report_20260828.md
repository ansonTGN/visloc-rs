# M7IM15 sqrt reduction: pinned Eigen AVX schedule

Date: 2026-08-28 JST  
Pinned Basalt revision: `0f3b2b52`

## Proven schedule

The pinned Eigen `squaredNorm()` path is `squared_norm_functor<float>` followed
by `LinearVectorizedTraversal` (`Eigen/src/Core/Redux.h:275-357`).  For AVX
`Packet8f`, the compiled O3 kernel uses:

1. one `vmulps` seed for each of two packet accumulators;
2. `vfmadd231ps` for each subsequent packet in the two-way unrolled loop;
3. `vaddps` to merge the two accumulators and `predux(Packet8f)`, whose
   low/high halves are added before the Packet4 horizontal reduction;
4. a four-value remainder as `vmulps` followed by ordered scalar `vaddss`;
5. any remaining scalar values as `vfmadd231ss`.

This is the generated code at `squared_norm_impl::run` in
`/tmp/m7im15_sqrt_apply_probe` (`0x10ae0`): packet FMA at `0x10b58`, merge at
`0x10b6c`, AVX reduction at `0x10b80-0x10b9a`, four-value remainder at
`0x10c53-0x10c7b`, and scalar FMA at `0x10c91`.

## Fix and fixture

`upstream_f32_packet8_squared_norm` now models that generic schedule: packet
accumulation uses `upstream_f32_packet8_mul_add`, and the scalar remainder
uses the four-value multiply/add peel followed by scalar `mul_add` values.
The logical segment origin remains `aligned_start = start`; no physical
allocation alignment peel is used.

Fixture:

`m7im15_sqrt_norm_fixture_v1.txt` contains all 60 exact pre-reflector tails
from the 96x60 Q2 replay and their Eigen tail-norm bits.  SHA-256:
`f7ed56d9af27423dffe1a75060fdc7d9cc7be9575ef2d8f65463e4a309b4c9eb`.

## Verification

Focused Rust tests (debug and optimized Windows/MSVC profiles):

```text
cargo test -p visloc-basalt --lib upstream_f32_packet8_squared_norm_matches_m7im15 -- --nocapture
2 passed, 0 failed
```

The regenerated Rust trace and the pinned Eigen probe trace are both 1,407,157
bytes and SHA-256
`a994acb46d7342bd68232a68035be329a132b7073e127299e6152f7bc2ec5480`.
Comparison is exact for all 60 records: metadata 60/60, Q/J 60/60, and RHS
60/60.  In particular, k58 tail norm is `42c7ff6c` (the prior Rust result was
`42c7ff6b`).
