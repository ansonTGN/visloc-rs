# M7cv clean frame-4 estimator-state oracle

Date: 2026-08-23 JST  
Status: **Bounded capture stopped before the first native state record; no clean-state claim**

## What was completed

The requested clean pinned artifacts were selected from M7cr:

* Basalt commit `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`;
* `basalt_vio` SHA-256 `89e0324ccd04c3b615bf7e945aa32480a2f86524987aa00eaf152dcd6b1d242c`;
* `libbasalt.so` SHA-256 `55f842642b370edf4025afd1d66c7b9d9011e5b4ce08efeb9d76be401b45b2cc`;
* `MH_01_easy --max-frames 5`, targeting the first frame-4
  `LinearizationAbsQR<float,6>::get_dense_H_b(...) const` entry.

Source/debug layout is resolved without production edits. The linearization
object's `estimator` member is a `BundleAdjustmentBase<float>*`; its
`frame_states` is an ordered `std::map<int64_t,
PoseVelBiasStateWithLin<float>>`. Each mapped state contains the current and
linearized `PoseVelBiasState<float>`; the state fields are timestamp, Sophus
SE3 translation, Eigen/Sophus quaternion coefficients in `xyzw` order,
world velocity, gyro bias, and accel bias. The AOM is five 15-DoF blocks at
offsets 0, 15, 30, 45, and 60, with timestamps recorded in the JSON artifact.

## Exact capture failure

The single bounded GDB inferior reached the startup/frame-4 path, but the
first entry command (`ptype /o` and typed pretty-print expansion for the
large optimized template object) consumed several minutes at roughly 95% GDB
CPU. The inferior was paused at the first H/b breakpoint and emitted no
`M7CV` line before termination at 4m47s. Therefore:

* `native_states` is empty;
* no native f32 lane, pointer, map node, or live AOM line is claimed from
  M7cv;
* no clean-binary first divergence can be established from this run.

The raw partial capture is retained at
`target/m7cv_clean_state_capture.out` (SHA-256
`9ce93b51730b5562a4bbbe1038d9e0e06d79250495c25510d87e4f06d67bdc0d1`). No
`gdb` or `basalt_vio` process remains live after termination.

## Retained comparison, clearly non-new evidence

For useful handoff context only, `target/m7at_clean_upstream_f4.jsonl`
already contains a retained upstream float32 iteration-0 state block. It is
not a record emitted by the terminated M7cv run and is not relabeled as a
new clean capture. Casting the M7cm Rust iteration-0 values to f32 gives 80
state lanes and four retained-reference mismatches:

| first/other lane | retained reference | M7cm Rust | delta |
|---|---:|---:|---:|
| frame 2 quaternion `w` (xyzw lane 3) | `3f17e7a6` | `3f17e7a4` | -2 ULP |
| frame 4 quaternion `w` (xyzw lane 3) | `3f159719` | `3f159717` | -2 ULP |
| frame 4 velocity `y` | `bc8bed06` | `bc8bed08` | +2 ULP |
| frame 4 velocity `z` | `be67f1c6` | `be67f1c8` | +2 ULP |

The first retained-reference difference is frame 2 quaternion `w`.
Because M7cv captured no native state bytes, this is a comparison against
the retained trace—not an assertion that those lanes are the clean binary's
first divergence. The machine-readable record, including every retained
state f32 lane and the failure provenance, is
`target/m7cv_clean_frame4_states.json`.

No Rust production source, native source, binary, library, rebuild, commit,
or push was performed.
