# M7et f32 landmark-J Packet4 exactness

Date: 2026-08-23 JST  
Status: **Pass — fixed-size product boundary implemented and all focused/full
library gates are green.**

## Implementation

`eigen_landmark_jacobian_f32` now accepts the source-shaped `2x4` camera
Jacobian and homogeneous `4x3` landmark Jacobian.  It spells Eigen's observed
`Packet4f` order as four safe scalar lanes:

```text
[row0-col0, row1-col0, row0-col1, row1-col1]
```

Each pair uses `k0 + FMA(k1, ...)` and `k2 + FMA(k3, ...)`, followed by the
packet lane add.  Column 2 uses the same two FMA/add pair reductions as the
scalar tail.  The production call site retains the homogeneous fourth row,
including `Jpp[3,2] = 1`, and uses no unsafe code, intrinsics, value-specific
branch, debug output, ignored test, commit, or push.

The clean ordinal-0 fixture is
[`m7et_ordinal0_jp_packet4.json`](../../pipelines/basalt/tests/fixtures/m7et_ordinal0_jp_packet4.json).
It records the native `2x4` camera J, `4x3` homogeneous Jpp, and six raw
column-major landmark-J lanes from the M7eo native probe:

```text
camera J (row-major): 43e6b4aa,3fe6b2f7,c1b056e5,00000000,
                      3fe604cb,43e4157f,426062c3,00000000
Jpp (column-major):   3ffe9a0c,3bbf6405,bdc31742,00000000,
                      3bbf6405,3ffcfc84,3e78fb07,00000000,
                      00000000,00000000,00000000,3f800000
raw Jp (column-major):4465f922,3f652460,3f65d1e0,4464cfbf,00000000,00000000
```

## Verification

Focused release M7 suite, including ordinal 0, the existing intermediate,
same-timestamp, and M7ec track-2 gates:

```text
cargo test --release -p visloc-basalt --lib vio::aom::tests::m7 -- --nocapture
13 passed, 0 failed, 0 ignored
```

Full release Basalt library:

```text
cargo test --release -p visloc-basalt --lib
169 passed, 0 failed, 1 ignored
```

The one ignored test is the pre-existing M8c diagnostic test; no new ignored
test was added.  The focused ordinal-0 test asserts the camera J shape, Jpp
shape/content, and all six raw Jp lanes exactly.

## Fresh-five replay

Because the focused candidate was adopted, the release example was rebuilt and
replayed for five MH01 frames:

```text
cargo build --release -p visloc-rs --example basalt_euroc_vio_demo
VISLOC_BASALT_DETAIL_ITERATIONS=1 \
VISLOC_BASALT_DETAIL_FRAME=4 \
VISLOC_BASALT_DETAIL_TRACE=target/m7et_jp_packet4_fresh5_detail.jsonl \
target/release/examples/basalt_euroc_vio_demo.exe \
  --euroc-dir E:/datasets/euroc_mav/machine_hall/MH_01_easy \
  --calibration target/euroc_ds_calib.json \
  --config target/euroc_config.json \
  --out-dir target/m7et_jp_packet4_fresh5_run --max-frames 5
```

Runtime processed 5/5 frames, delivered 42 IMU samples, emitted 1,114
observations, and retained the established 61 visual factors / 584 frame-4
visual observations.  The fresh iteration-start snapshot cost is
`4215.93310546875`.

The ordinal-matched comparison against clean native `m7ef` gives:

| boundary | exact | total |
|---|---:|---:|
| projection pairs | 564 | 584 |
| raw residual pairs | 564 | 584 |
| raw Jp scalar lanes | 2,636 | 3,504 |
| weighted Jp scalar lanes | 2,838 | 3,504 |
| aggregate H lanes | 3,241 | 5,625 |
| aggregate b lanes | 6 | 75 |

The first raw Jp mismatch remains ordinal 0 (`4465f922,...` native versus
`4465f920,...` in the serialized fresh-five factor), so this isolated
Packet4 boundary fixture must not be read as an all-584 visual parity claim;
the runtime detail does not expose every native Jpp intermediate.  The first
projection/raw mismatch remains ordinal 38.  H is aggregate and cannot assign
its mismatch to an individual visual ordinal.

Fresh-five artifacts are ignored `target/` outputs:
`target/m7et_jp_packet4_fresh5_detail.jsonl` and
`target/m7et_jp_packet4_fresh5_run/`.
