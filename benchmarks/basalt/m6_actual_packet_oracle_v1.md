# M6 actual MargData packet oracle

Reference: pinned Basalt `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`, EuRoC
MH_01_easy, first 80 camera frames.  The upstream Cereal packets were dumped
with `upstream_marg_dump.cpp`; Rust packets came from the fresh current-code
replay at `target/basalt_m6_run80_packets_postfix/marg_data`.

The machine-readable result is [m6_actual_packet_oracle_v1.json](m6_actual_packet_oracle_v1.json).
Reproduce it with:

```text
python benchmarks/basalt/m6_packet_oracle.py \
  --rust-dir target/basalt_m6_run80_packets_postfix/marg_data \
  --upstream-dir work/m6_upstream_packet_oracle \
  --output benchmarks/basalt/m6_actual_packet_oracle_v1.json
```

## Queue and structural contract

The Rust run emits exactly five mapper files: frames **51, 58, 65, 72, 79**.
The other 75 frame records remain available in `trace.jsonl` as diagnostics but
are not written to the mapper queue.  Every selected packet has:

- a 72-column mixed AOM (seven 6-DoF pose blocks followed by two 15-DoF
  navigation blocks), with the newest state retained in the frame table but
  excluded from the marginalization AOM;
- FEJ flags `[true, false, false]` for the serialized full states;
- eight `kfs_all` entries and 16 raw image records (two cameras per KF);
- `used_imu=true`, one pose target equal to the selected `kfs_to_marg` entry,
  and one velocity/bias target equal to the oldest serialized navigation state;
  `states_to_marg_all` is empty for these KF-removal packets;
- Rust diagnostic row groups whose sum equals the emitted square-root row count.

After timestamp-to-frame normalization, AOM order, frame-state IDs, FEJ flags,
image/KF IDs, IMU flag, and target bookkeeping match the pinned packet at
frames 51 and 58.  At frames 65, 72, and 79 the state-table timestamps still
match exactly, but KF policy diverges (`kfs_to_marg` is respectively 21, 56,
49 in Rust versus 35, 28, 56 in this upstream run).  This is an algorithmic
frontend/landmark-connectivity divergence, not a packet-ordering error.

## Numeric result

The upstream packet serializes `abs_H/abs_b`; Rust stores the same absolute
system plus diagnostic square-root `J/r`.  The oracle accounts for Eigen
row-major versus nalgebra column-major storage.  H/b shapes are 72×72 and 72
for all five packets, but values are not equal because the Rust run has a
different direct-KLT track set and solver trajectory.  The reported maximum
absolute H/b differences are retained in the JSON artifact.  Frame-table pose
values are also reported numerically while IDs are compared structurally; no
tolerance was relaxed and no GT was used.

The raw in-process diagnostic retains the carried Rust square-root `prior` so
per-frame FEJ/prior diagnostics are not lost.  The mapper writer now calls
`MargData::to_mapper_packet()`, which omits this Rust-only field from the JSON
wire packet: pinned upstream `MargData` has no serialized prior.  `used_imu` follows the pinned
queue value (`true`) for actual packets, while state-only diagnostics retain
their causal active-link value.

## SqrtToSqrt exact real-packet boundary

`benchmarks/basalt/m6_sqrt_boundary_frame51_v1.txt` is a deterministic slice of
the Rust frame-51 packet listed above.  Its SHA-256 is
`ED7361C42D24006200D7B60D6C158F60309DDAA9834EC7CE93F37419C1535695`.
The source JSON SHA-256 is
`2DD1015F2B4FF4C9472D13464D98DC42CD32DAEDB168ED9BF471729BAD415BDA`, and the
source path is
`target/basalt_m6_run80_packets_postfix/marg_data/frame_000051.json`.
The fixture retains rows
`0..58, 93, 94, 451, 544, 2093, 2094, 2095, 2105, 2106, 2107` from the
2108×72 `aom_sqrt_jacobian`, all 15 leaving columns
`12..17, 48..56`, and the 57 complementary columns.  The row count is 69,
which makes the pinned rank walk retain 54 rows after the 15-column leaving
block.

The standalone oracle is an Eigen 5.0.1 reproduction of the pinned
`MargHelper<float>::marginalizeHelperSqrtToSqrt` ordering and calls
`makeHouseholderInPlace`/`applyHouseholderOnTheLeft`; it does not link the
upstream shared library because crossing its separately compiled Eigen ABI
caused an alignment failure.  The authoritative source is
`benchmarks/basalt/upstream_sqrt_marginalization_oracle.cpp`, SHA-256
`7B0893E9D310FCAF6DA60D99190301D2D5AAC1FB48C5B39115F40B22B6C3E622`.

The checked oracle executables are the pinned WSL builds copied to
`target/m6_sqrt_oracle/`:

| executable | build | SHA-256 |
| --- | --- | --- |
| `upstream_sqrt_marginalization_oracle_default` | Eigen default SIMD | `461E190AE7C7EDB6757D560567D82955448C25D3BB85591FC07D7441FAC81A89` |
| `upstream_sqrt_marginalization_oracle_scalar` | `-DEIGEN_DONT_VECTORIZE` control | `7860A2046783BA6208107657ED29DABD18F404246B69230B10C64D8D3779FD83` |

Build and run commands from the pinned WSL checkout are:

```text
c++ -std=c++17 -O2 -DEIGEN_DONT_PARALLELIZE \
  -isystem /root/visloc-basalt-oracle-0f3b2b52/thirdparty/vcpkg/buildtrees/eigen3/src/5.0.1-d487a628b0.clean \
  -o /tmp/upstream_sqrt_marginalization_oracle \
  benchmarks/basalt/upstream_sqrt_marginalization_oracle.cpp
c++ -std=c++17 -O2 -DEIGEN_DONT_PARALLELIZE -DEIGEN_DONT_VECTORIZE \
  -isystem /root/visloc-basalt-oracle-0f3b2b52/thirdparty/vcpkg/buildtrees/eigen3/src/5.0.1-d487a628b0.clean \
  -o /tmp/upstream_sqrt_marginalization_oracle_scalar \
  benchmarks/basalt/upstream_sqrt_marginalization_oracle.cpp
/tmp/upstream_sqrt_marginalization_oracle m6_sqrt_boundary_frame51_v1.txt
/tmp/upstream_sqrt_marginalization_oracle_scalar m6_sqrt_boundary_frame51_v1.txt
```

The first executable uses Eigen's default vectorization and is the
authoritative pinned-runtime arithmetic.  Both builds produce rank/output
shape 69 total rank, 15 marginal rank, and 54×57 J with a 54-element rhs.  The
production Rust `ScalarMode::UpstreamF32` path is
`sqrt_marginalize_upstream_f32_packet` in `pipelines/basalt/src/vio/window.rs`.
It is a safe `Packet4f` operation-order emulation: the complete input stack is
cast to `f32`, packet reductions and GEMV updates follow the pinned Eigen
order, and values are widened only at the API boundary.  The crate remains
`#![deny(unsafe_code)]`.

The real fixture test compares the complete output, not a tolerance sample:
all 3,078 J values plus 54 rhs values (3,132 `f32` bit patterns) match the
default Eigen executable exactly, with maximum ULP distance **0**.  It also
requires the 54×57/54 shape, rank 69/15 result, and selected diagnostic bits
below.  The row-major-J-then-rhs FNV-1a digest is
`0x8869c7ca4903ab17`.

| quantity | default Eigen SIMD / production Rust bits | maximum ULP |
| --- | ---: | ---: |
| `||J||` | `131323.828` (`1207975669`) | 0 |
| `||r||` | `380.384003` (`1136537895`) | 0 |
| `J(0,0)` | `1176616427` | 0 |
| `J(0,1)` | `1127794638` | 0 |
| `J(0,56)` | `3212872861` | 0 |
| `J(1,0)` | `0` | 0 |
| `J(27,28)` | `3310529030` | 0 |
| `J(53,56)` | `3242393197` | 0 |
| `r(0)` | `3215659860` | 0 |
| `r(27)` | `1080990096` | 0 |
| `r(53)` | `3217203659` | 0 |

The scalar executable is retained only as an operation-order diagnostic; it is
not the production acceptance reference.  The serialized pinned packets
expose `abs_H/abs_b`, not the upstream square-root J/r or a numeric carried FEJ
point; they expose FEJ flags only, and frame 51 has
`packet_prior_present=false`.  The Rust boundary test therefore uses a zero
FEJ point solely to exercise the real packet arithmetic and does not claim a
FEJ-point comparison.

## Implemented Eigen 5.0.1 packet order

The following is the operation order observed from the pinned Eigen 5.0.1
headers and a fresh disassembly of the authoritative default oracle built with
`g++ 11.4`, `-std=c++17 -O2 -DEIGEN_DONT_PARALLELIZE`, and no `-march`.  That
target selects SSE2 `Packet4f` (four `f32` lanes); `pmadd` is
`padd(pmul(a,b),c)`, not FMA.  Relevant source is
`Eigen/src/Householder/Householder.h`, `Eigen/src/Core/Redux.h`,
`Eigen/src/Core/InnerProduct.h`, `Eigen/src/Core/GeneralProduct.h`,
`Eigen/src/Core/products/GeneralMatrixVector.h`, and
`Eigen/src/Core/ProductEvaluators.h` in the pinned include tree.

### Reductions and reflector construction

* `tail.squaredNorm()` (`Householder.h:65..85`) uses the dynamic linear
  vectorized reduction (`Redux.h:275..323`).  In the emitted fixture path the
  loads are `movups`, so there is no scalar alignment head: packet lanes start
  at element 0.  Each packet is squared with `mulps`.  Two packet
  accumulators are initialized at offsets 0 and 4 and updated in 8-element
  steps.  They are combined as `p0 += p1`, then reduced horizontally with
  `movhlps/addps` followed by `shufps/addps`.  The scalar tail is processed in
  increasing order from `4 * floor(size/4)` through `size-1`; sizes below 4
  use the scalar loop from element 0.  The generic Redux code has an
  `first_default_aligned` path, but this dynamic `VectorBlock` instantiation
  emits the unaligned/no-head form above.
* `makeHouseholder` then performs scalar `c0*c0`, scalar addition with the
  reduced tail norm, and scalar `sqrtss`; the sign branch and
  `essential = tail / (c0-beta)` are scalar.  The zero/tiny branch is taken
  when the tail norm is at most `numeric_limits<float>::min()` and the
  imaginary part of `c0` is zero.  The essential-vector assignment uses the
  dense-assignment alignment walk: a scalar head (0..3 elements) reaches a
  16-byte destination boundary, then unaligned source `divps`/aligned
  destination stores in groups of four, followed by the scalar tail.
* A vector dot (`essential.adjoint()*qr.bottom`) uses
  `InnerProduct.h:129..176`: four packet accumulators at offsets 0, 4, 8,
  and 12, unaligned loads, then 16-element loop steps.  Remainder packets
  are merged in this order: `p2 += p3`, `p1 += p2`, `p0 += p1`; only then is
  `predux(p0)` performed, followed by the scalar tail from
  `4 * floor(size/4)`.  There is no alignment head.

### `applyHouseholderOnTheLeft`

The pinned source call is:

```text
tmp.noalias() = essential.adjoint() * bottom;
tmp += this->row(0);
this->row(0) -= tau * tmp;
bottom.noalias() -= tau * essential * tmp;
```

For the matrix `bottom`, the vector-on-the-left product is transposed into
the row-major GEMV path (`GeneralProduct.h:259..267`,
`GeneralMatrixVector.h:293..466`).  If `K=essential.size()` and
`C=bottom.cols()`:

* output rows are processed in groups of 8, then 4, then 2, then 1;
* `fullColBlockEnd = 4 * floor(K/4)`; for each `j=0,4,...` one unaligned
  RHS packet is loaded and reused for every row accumulator in that group;
* each row accumulator executes packet `mulps` then `addps` in increasing
  `j` order, is horizontally reduced with Eigen's SSE `predux`, and then
  receives scalar products for `j=fullColBlockEnd..K-1`;
* matrix and RHS packet loads are explicitly `Unaligned`; there is no packet
  head.  The generic product assignment zeroes `tmp` before GEMV's
  `res += alpha * product` updates.

In the emitted dynamic column-major `Block` instantiation, the two row-0
operations (`tmp += row(0)` and `row(0) -= tau*tmp`) are scalar strided loops
(`addss`, then `mulss/subss`) over columns.  They must not be replaced by a
contiguous packet loop for this fixture.

The bottom outer update uses `ProductEvaluators.h:259..319` and the dense
assignment loop:

* first materialize the scaled essential vector from element 0 using
  unaligned `movups`, `mulps`, and aligned temporary stores; finish its
  `K % 4` scalar tail;
* visit bottom columns in increasing order.  For each contiguous destination
  column, consume a scalar head `h = (-ptr/sizeof(float)) & 3` (0..3) so the
  destination reaches 16-byte alignment;
* process the aligned middle with unaligned source loads, aligned destination
  `movaps` load/store, and `subps` in four-lane packets; finish the scalar
  tail.  A non-`f32`-aligned destination is rejected by Eigen's debug
  assertions.

The RHS/vector overload follows the same dot, scalar row-0, and aligned-head
bottom-update rules.  Its bottom vector update is the same scalar-head,
Packet4f-middle, scalar-tail sequence; the source essential loads remain
unaligned while the contiguous destination uses aligned packet accesses.
The fixture is 69x72, so these conditions occur repeatedly as `K` shrinks;
the packet boundary is recomputed for every Householder step and every
contiguous bottom column, not once for the whole matrix.

## Production activation and verification

`marginalize_mixed_prior_with_mode` dispatches `ScalarMode::UpstreamF32` to
`sqrt_marginalize_upstream_f32`, which dispatches directly to
`sqrt_marginalize_upstream_f32_packet`; this is the production default used by
the compatibility estimator.  No tolerance or expected bit pattern is
loosened.

Exact checks run on 2026-08-22 JST:

```text
& 'C:\Users\rsasa\.cargo\bin\cargo.exe' test -p visloc-basalt --lib upstream_f32_sqrt_marginalization_matches_eigen_golden_boundaries -- --nocapture
  1 passed, 0 failed (138 filtered)

& 'C:\Users\rsasa\.cargo\bin\cargo.exe' test -p visloc-basalt --lib upstream_f32_real_m6_packet_slice_matches_default_eigen -- --nocapture
  1 passed, 0 failed (138 filtered)

& 'C:\Users\rsasa\.cargo\bin\cargo.exe' test -p visloc-basalt --test m6_packet_contract -- --nocapture
  1 passed, 0 failed

& 'C:\Users\rsasa\.cargo\bin\cargo.exe' test --release -p visloc-basalt --test m6_packet_contract -- --nocapture
  1 passed, 0 failed

& 'C:\Users\rsasa\.cargo\bin\cargo.exe' test --release -p visloc-basalt --lib
  138 passed, 1 ignored, 0 failed

& 'C:\Users\rsasa\.cargo\bin\cargo.exe' test --release -p visloc-basalt --tests --no-run
  13 release test executables compiled, 0 failed
```

The two focused unit tests are the synthetic five-case Eigen boundary set and
the complete real frame-51 packet slice respectively.  The release library
count is 139 tests (138 passing plus one intentionally ignored); the packet
contract integration test passes in both debug and release (one test each).
The no-run command compiles the library test target plus all 12 integration
targets without executing external-data tests.
