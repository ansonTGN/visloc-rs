# M7ca exact Sophus SE2f exponential replay

Date: 2026-08-23 JST  
Scope: the 20 recorded MH_01 frame-0 -> frame-1, cam0, track-2 temporal
increments, plus targeted values around the float small-angle branch.

## Decision

The fixed operation order is proven for both the recorded general branch and
the targeted small-angle cases. `pipelines/basalt/src/update.rs` now uses the
explicit f32 FMA spelling in `Se2::exp`:

```text
x = sin_over_theta.mul_add(tx, -(one_minus_cos_over_theta * ty))
y = one_minus_cos_over_theta.mul_add(tx, sin_over_theta * ty)
```

No threshold, tracking branch, unsafe code, debug output, or other production
path was changed. The existing Rust `SMALL_ANGLE = 1e-5` threshold was
retained exactly as requested. The pinned Sophus implementation itself uses
`abs(theta) < Constants<float>::epsilon()`; the fixtures deliberately include
values below epsilon, exactly at epsilon, and between epsilon and `1e-5` to
exercise both implementations' branch paths without changing either threshold.

## Native/Rust SE2 probe

The native probe is `benchmarks/basalt/m7ca_se2_exp_probe.cpp`, compiled against
the pinned Sophus/Eigen headers with:

```text
g++ -std=c++17 -O3 -march=native -DEIGEN_DONT_PARALLELIZE
```

The Rust replay is `pipelines/basalt/examples/m7ca_se2_exp_probe.rs`. The
machine-readable native fixture and regression test are:

```text
pipelines/basalt/tests/fixtures/m7ca_se2_exp_native.json
pipelines/basalt/tests/m7ca_se2_exp_exact.rs
```

Before the patch, all 20 rotation quadruples and all 20 y translations already
matched native; x differed by one ULP at six records:

| record | native x | pre-patch Rust x |
|---|---:|---:|
| L3/I0 | `0xbeb2bd73` | `0xbeb2bd72` |
| L2/I0 | `0xbc6ae242` | `0xbc6ae243` |
| L1/I3 | `0xbcceba2a` | `0xbcceba2b` |
| L1/I4 | `0x3c527fe7` | `0x3c527fe8` |
| L0/I2 | `0xbce5aaa4` | `0xbce5aaa5` |
| L0/I3 | `0x3c2f6b1c` | `0x3c2f6b1d` |

After the explicit order change:

```text
recorded general cases: 20/20 exact in all 4 rotation + 2 translation fields (120/120)
small-angle cases:       8/8 exact in all 4 rotation + 2 translation fields (48/48)
```

The small-angle native outputs include signed zero at theta zero and the
epsilon-boundary values, so this is not only a nonzero-angle general-branch
check.

## Composed warp and endpoint replay

The post-patch `m7bw_temporal_probe` replay was compared with the pinned native
temporal trace:

```text
per-iteration composed translation pairs: 20/20 exact (40/40 scalars)
per-level world translation pairs:         4/4 exact (8/8 scalars)
final cam0 track-2 endpoint:                0x422e3d26, 0x42ea1cb7 exact
```

`benchmarks/basalt/m7ca_composed_warp_probe.cpp` also replays the 20 updates
through native Eigen `AffineCompact2f`. Its full compact linear lanes retain
the pre-existing affine packet 1-ULP differences (95/120 iteration fields and
19/24 world fields exact); those differences are unchanged from the pre-M7ca
Rust trace and are outside this `Se2::exp` change. All composed translations,
which are the M7bw endpoint boundary, remain exact.

The fresh first-80 `stereo_diag` endpoint after this patch contained 80/80
records with exact IDs/count structure. Direct f32 comparison against
`mh01_stereo_endpoint_v1_first80.jsonl` found:

```text
exact point pairs:    16,237 / 24,256
exact coordinate fields: 36,654 / 48,512
maximum absolute delta: 0.00048828125 px
maximum ULP distance: 47
first remaining mismatch: frame 1, cam1, track 3, x
  native 0x422f68f8, Rust 0x422f68f7
```

The focused first-80 golden test passed; there was no endpoint regression.

## Verification

All focused release tests passed after the patch:

```text
cargo test -p visloc-basalt patch --release --lib                         5 passed
cargo test -p visloc-basalt update --release --lib                        7 passed
cargo test -p visloc-basalt --test m7bw_cam0_temporal_exact --release     1 passed
cargo test -p visloc-basalt --test m7ca_se2_exp_exact --release            2 passed
cargo test -p visloc-basalt --test m7bd_obs_pixel_exact --release --ignored 1 passed
cargo test -p visloc-basalt --test mh01_stereo_golden --release --ignored  2 passed
```

The M7bw fixture test now checks the current native-order result while
retaining the prior direct-product bit as a provenance baseline. No commit or
push was made.
