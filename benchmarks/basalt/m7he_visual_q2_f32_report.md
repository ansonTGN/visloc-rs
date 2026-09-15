# m7he visual Q2 f32 boundary report

Date: 2026-08-24

This audit covers the first frame-4, iteration-0 visual landmark QR on the
clean Basalt target pinned at commit
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.  It is an audit artifact only;
there is no commit or push from this work.

## Changes retained

- `aom.rs` uses an Eigen-compatible f32 two-vector norm for the visual Huber
  boundary: `x.mul_add(x, y * y)`.  This fixes the observed native weighted
  residual bits without changing the f64 path or IMU factors.
- The f32 landmark nullspace projection is `pub(crate)` and the detail/visual
  trace dispatches to it when `ScalarMode::UpstreamF32`; its f32 matrices are
  serialized as f64 JSON values only after projection.  The trace metadata
  records `landmark_projection: f32`.
- `LandmarkHouseholderF32` uses an explicit sequential f32 FMA for the tail
  squared norm, and uses `c0.mul_add(c0, tail_sq_norm)` for the final norm
  before `sqrt`, matching the pinned Eigen `vfmadd231ss` boundaries.  Both
  changes are global scalar-mode operations; no per-track lane heuristic is
  used.

The similarly named `window.rs::upstream_f32_packet_squared_norm` is retained
only by the separate f32 sqrt-marginalization implementation.  It is not
called by `landmark_nullspace_projection_f32`, Q2, or the detail trace, and
was not used as a Q2 candidate.

For `ScalarMode::UpstreamF32`, `window.rs::project_landmark_factor` calls the
same `aom::landmark_nullspace_projection_f32` used by production reduction;
the trace therefore serializes production Q2 values rather than running a
second QR implementation.

## Native comparison

The durable capture and comparator are:

- `target/m7he_preqr_all61_1t.json` — clean one-thread pre-QR storage;
- `target/m7he_apply_all61_1t.json` — clean one-thread Householder/apply
  stages for all 61 blocks;
- `target/m7he_compare_q2.py` — pre-QR and final Q2/rhs comparator;
- `target/m7he_fresh5_f32_fma_c0fma_detail.jsonl` — fresh five-frame Rust
  trace with f32 projection, tail FMA, and final c0 FMA.

Reproduction:

```text
python target/m7he_compare_q2.py target/m7he_first_landmark_vtable.json --rust target/m7he_fresh5_f32_fma_c0fma_detail.jsonl --preqr target/m7he_preqr_all61_1t.json --apply-final target/m7he_apply_all61_1t.json
```

The final all61 result is:

```text
preqr_all=591/108080 exact_tracks=43/61
apply_all=4417/87600 rhs=130/1168 exact_tracks=43/61
track120: q2=0/1500, rhs=0/20
```

For comparison, before the tail FMA A/B the corresponding final counts were
8321/87600, 249/1168, and 29/61 exact tracks.  After the tail FMA but before
the final `c0.mul_add` boundary they were 5158/87600, 155/1168, and 39/61;
the final FMA resolves the remaining four exact-input tracks.  The pre-QR
count is unchanged by either Householder step and therefore separates
observation construction from QR.

The first native landmark local H/b capture for track 120 is identical at
one and four worker threads.  This rules out TBB local-product scheduling as
the first boundary for that factor; aggregate H/b differences are not used
as causal evidence.

## Boundary conclusion

The four exact-pre-QR tracks that initially diverged after QR (`10`, `47`,
`54`, `91`) are now bit-exact after the two scalar FMA boundaries.  Clean
Eigen disassembly shows the row-major column-stride tail path accumulating
with scalar `vfmadd231ss`, followed by a second `vfmadd231ss` for `c0²` before
the square root.  This is the common schedule used by production and the
f32 detail trace, so no value-dependent packet or lane heuristic is needed.

The remaining 18 non-exact tracks are already non-exact at pre-QR (the
robust visual observation boundary); every track whose pre-QR input is exact
has exact final Q2/rhs.  Householder application and TBB local-product
nondeterminism are not supported as the cause by these captures.

## Verification

```text
cargo test -p visloc-basalt --release --lib m7_ -- --nocapture
17 passed

cargo test -p visloc-basalt --release --lib
185 passed, 1 ignored
```

The existing Huber f32 bit fixture and track-1 Householder fixture both pass.
No IMU factor code was changed.

Superseded probe scripts, intermediate fresh5 traces, duplicate raw local
H/b/storage dumps, and old run directories were removed.  The final trace,
clean captures, comparator, and first-landmark JSON remain under `target/`.
