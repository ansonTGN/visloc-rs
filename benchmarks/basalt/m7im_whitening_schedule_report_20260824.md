# M7 IMU whitening/product schedule (read-only source audit)

Date: 2026-08-24 JST  
Scope: pinned Basalt `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`, Eigen headers from
`/root/visloc-basalt-oracle-0f3b2b52`, built with `g++ 11.4`, `-O3 -g -DEIGEN_DONT_PARALLELIZE -march=skylake`.

## Selection facts

The pinned Eigen `EIGEN_CACHEFRIENDLY_PRODUCT_THRESHOLD` is 8 and the AVX
packet is `Packet8f`; FMA is enabled and `EIGEN_ARCH_DEFAULT_NUMBER_OF_REGISTERS`
is 16.  Therefore dimensions 9 are `Large`.

- `W(9x9) * r(9x1)`: `GemvProduct`, column-major fast GEMV.
- `W(9x9) * J(9x30)`: `GemmProduct`; `9 + 9 + 30 = 48 >= 20`, so the
  coefficient fallback is not selected.  Finite blocking is `mc=9,nc=30,kc=9`.
- `Jᵀ(30x9) * J(9x30)`: `GemmProduct`, finite blocking `mc=30,nc=30,kc=9`.
- `Jᵀ(30x9) * r(9x1)`: `GemvProduct`, row-major fast GEMV because `Jᵀ` is a
  row-major transpose view.

Authoritative source locations in the pinned Eigen tree:

- `Eigen/src/Core/GeneralProduct.h`: product type and threshold selection.
- `Eigen/src/Core/products/GeneralMatrixVector.h`: column/row-major GEMV.
- `Eigen/src/Core/products/GeneralMatrixMatrix.h`: finite GEMM blocking and
  GEMV fallback for one-column destinations.
- `Eigen/src/Core/products/GeneralBlockPanelKernel.h`: AVX GEBP packing and
  micro-kernel; `Eigen/src/Core/arch/AVX/Reductions.h` and `.../SSE/Reductions.h`:
  packet reductions.

## Exact helper schedules

For a zero destination and `alpha=1`, use f32 fused operations (`x.mul_add(y,
acc)`) only where the Eigen source calls `pmadd`.

### Column-major 9x9 × 9x1

Rows 0..7 are one `Packet8f` accumulator. For `k=0..8`:

```text
c[lane] = fma(W[row=lane,k], r[k], c[lane])
```

Store lanes 0..7. Row 8 is the scalar tail, left-associated in source order
`k=0..8` (`c += W[8,k] * r[k]`; GCC may contract this scalar `c +=` under the
oracle flags, so the fixture must verify bits rather than assume a separate
mul/add).

### Column-major 9x9 × 9x30

GEBP packs LHS rows 0..7 as 8-lane packets and RHS in groups of four output
columns. For output rows 0..7 and every column, the accumulator is exactly:

```text
c = 0;
for k=0..8 { c = fma(W[row,k], J[k,col], c); }
```

For columns 0..27 the source kernel keeps the peeled depth terms in two
independent packet accumulators before the final depth tail:

```text
even = fma(W[row,0], J[0,col], 0)
for k=2,4,6: even = fma(W[row,k], J[k,col], even)
odd  = fma(W[row,1], J[1,col], 0)
for k=3,5,7: odd  = fma(W[row,k], J[k,col], odd)
c = f32(even + odd)       # packet padd
c = fma(W[row,8], J[8,col], c)
```

The scalar expression above is an algebraic description only; the even/odd
split is required for bit identity.

The row-8 tail is special for columns 0..27 (groups of four). AVX swapped
traits process two depth values per packet and combine four packet accumulators:

```text
p0 = fma(A0, B0, 0) + fma(A1, B1, 0)
p1 = fma(A2, B2, 0) + fma(A3, B3, 0)
p2 = fma(A4, B4, 0) + fma(A5, B5, 0)
p3 = fma(A6, B6, 0) + fma(A7, B7, 0)
c  = (p0 + p1) + (p2 + p3)
c  = fma(A8, B8, c)
```

The additions above are ordinary packet `padd`, not FMA. For the final two
columns (28,29), the scalar remainder uses the direct left-to-right FMA loop
over `k=0..8`. This row-8 four-column tree is the likely hidden mismatch if a
plain per-element `mul_add` loop fixes rows 0..7 but leaves a few WJ lanes.

In the swapped tail, each packet lane is interleaved as
`[col0-k0,col1-k0,col2-k0,col3-k0,col0-k1,...]`; the four two-depth packet
blocks are merged lane-wise as `(C0+C1)+(C2+C3)`, then the lower and upper
AVX halves are added to obtain the four output columns. Every `+` in this
description is an f32 packet add; only the term products use FMA.

The exact diagnostic model is implemented as
`gemm_w_j_gebp_eigen_avx_exact` in
`benchmarks/basalt/m7im_whitening_schedule_fixture.py`. It accounts for the
non-tail 1x4 kernel's even/odd accumulator split and the swapped scalar-row
packet tail (including packet-add and half-reduction order). Against the
frame-4 oracle, it produces `W*J` with `0/270` differing bits.

Mismatch localization on the same oracle input makes the schedule boundary
explicit (flat index is `row * 30 + column`):

| candidate | total | first flat index | per-output-row counts (rows 0..8) |
|---|---:|---:|---|
| scalar FMA, `k=0..8` | 37 | 93 (`r3,c3`) | `0,0,0,7,3,8,3,10,6` |
| row-8 packet tree only | 35 | 93 (`r3,c3`) | `0,0,0,7,3,8,3,10,4` |
| separate multiply/add | 56 | 34 (`r1,c4`) | `0,5,0,10,8,8,10,8,7` |
| `gemm_gebp_eigen_avx_exact` | **0** | — | `0,0,0,0,0,0,0,0,0` |

Thus the residual 35--37-bit population is not confined to the scalar row;
the first eight packet lanes require the peeled even/odd accumulator split,
while row 8 additionally requires the swapped-tail interleave and reduction.

### 30x9 × 9x30 (`JᵀJ`)

GEBP uses `mr=24`, `nr=4`, `LhsProgress=8`; rows split as 24 packet lanes,
4 half-packet lanes, and 2 quarter-packet lanes. Every output element still
has a per-element k accumulator with `k=0..8` FMA order; packetization changes
parallel lanes, not the k order. A faithful scalar model is therefore:

```text
H[i,j] = 0;
for k=0..8 { H[i,j] = fma(J[k,i], J[k,j], H[i,j]); }
```

Do not exploit symmetry or accumulate only one triangle: Eigen's GEBP computes
the complete 30x30 destination in its row/column panel order. Once the exact
`W*J` schedule above is used, this model compares `JᵀJ` to the oracle at
`0/900` differing bits; the earlier large mismatch was entirely caused by
feeding the 37-bit-mismatching `W*J` candidate into the second product.

### Row-major 30x9 × 9x1 (`Jᵀr`)

Rows are processed in groups of 8, then 4, then 2. For each row, columns 0..7
form a `Packet8f` FMA accumulator; Eigen's AVX reduction is:

```text
q0 = lane0 + lane4; q1 = lane1 + lane5;
q2 = lane2 + lane6; q3 = lane3 + lane7;
packet_sum = (q0 + q2) + (q1 + q3);
result = packet_sum + Jt[row,8] * r[8];
```

The final scalar add is source-level `+=` (`cj.pmul`), not another packet
FMA. The row grouping affects only execution order between independent rows.
With the exact `W*J` and `W*r` candidates, this tree compares `Jᵀr` at `0/30`
differing bits. The machine-readable diagnostic result is recorded in
`target/m7im_whitening_schedule_fixture_20260824.current2.json`.

## Verification recommendation

Add a diagnostic-only fixture that consumes the existing frame-4 oracle's
`sqrt_information`, `raw_residual`, and `jacobian` bit arrays and emits the
four helper outputs separately: `W*r`, `W*J`, `JᵀJ`, `Jᵀr`. Compare IEEE bits
against `frame4_factor` in `target/m7im_cov_ldlt_oracle_20260824.json`; do not
use the widened legacy fields. Keep this fixture outside production and gate
the production helper only after all four arrays are exact.
