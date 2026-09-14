# m7bm track3 optical-flow follow-up

Status: the first remaining scalar is isolated, but the track3 endpoint is not
yet exact. The fresh endpoint remains `cam1 track_id=3`
`x=0x42395ac3, y=0x435c70bb`; the pinned native target is
`x=0x42395ac4, y=0x435c70bb`.

## Scope and fixture

- Dataset: EuRoC MH_01_easy, frame 0, cam0 source `(38,207)`.
- Native trace: `target/m7bg_native_flow_probe.txt`.
- Fresh Rust endpoint: `target/m7bm_track3_flow_next_endpoint.jsonl`.
- Fresh one-frame release run: `43.887 s` wall time including recompilation;
  the endpoint record has 135 cam0 and 61 cam1
  observations.

## Proven production contractions

The following general expression/order fixes are retained:

1. `H⁻¹Jᵀ` uses Eigen's fixed-size coefficient order `[1,2,0]`, with the
   second and first products fused into the running value. All 156 level-3
   coefficients match the native trace.
2. The affine sample warp spells the Eigen 2x2 product as first-column
   product, second-column FMA, then translation. This fixes the first post-update
   sample-point discrepancy: after L3/I0, native and Rust residual scalars 1 and
   50 are both `3db375f0` and `be8dd83a` respectively.

The diagnostic balanced 52-term reduction matches L3/I0 (`400fc2da,3f9be212,3ea0024b`
and `40d8ad14,41db5ebe`) but does not match the native vectorized L3/I1 GEMV, so
it is not retained as a production correction.

## Earliest remaining scalar

The next mismatch is L3/I1's increment y, after the residual is exact:

| value | native | Rust |
|---|---:|---:|
| I1 increment x | `bf47a90b` | `bf47a90b` |
| I1 increment y | `bd615534` | `bd615532` |
| I1 increment theta | `be91b7c1` | `be91b7c1` |

The difference is therefore in the GEMV reduction itself, not image
interpolation, SE2 exponential, composition, bounds, or convergence. A pinned
Eigen 5.0.1 probe gives the native vectorized product
`bf47a90b,bd615534,be91b7c1`; the balanced tree gives
`bf47a90b,bd615532,be91b7c1`. With `EIGEN_DONT_VECTORIZE`, Eigen gives a third
result (`bf47a90c,bd61553c,be91b7c1`), so replacing the native vectorized
reduction with another generic fold would not be faithful.

The remaining native path is the fixed-size 3x52-by-52x1 Eigen GEMV compiled
with packet size 8. Its generated scalar-FMA/unrolled accumulator order is the
first unresolved reduction; no hardcoded track correction was added.

## Verification

The focused patch tests and the existing track1/patch fixtures should be run
after any future GEMV-order change. This report intentionally stops at the
first unresolved scalar rather than claiming endpoint exactness.
