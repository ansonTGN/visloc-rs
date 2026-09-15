# M7ct clean native frame-4 H/b oracle

Date: 2026-08-23 JST  
Status: **Pass — one bounded clean-native GDB capture, no production edits**

This artifact captures the first frame-4 `basalt::LinearizationAbsQR<float,
6>::get_dense_H_b(...) const` return from the separate clean Basalt worktree
used by M7cr. The run used `MH_01_easy`, four native threads, and
`--max-frames 5`. The GDB return breakpoint was derived dynamically from the
live symbol entry (`$entry_pc + 0x450`); no diagnostic source, binary, library,
Rust source, rebuild, commit, or push was touched.

## Provenance and clean verification

| item | value |
|---|---|
| clean checkout | `/root/visloc-basalt-clean-m7cr-20260823` |
| Basalt commit | `0f3b2b52c807f70ff4e2973ce253c73329eea7bc` |
| binary | `build/core-relwithdebinfo/basalt_vio` |
| `basalt_vio` SHA-256 | `89e0324ccd04c3b615bf7e945aa32480a2f86524987aa00eaf152dcd6b1d242c` |
| `libbasalt.so` SHA-256 | `55f842642b370edf4025afd1d66c7b9d9011e5b4ce08efeb9d76be401b45b2cc` |
| m7cr manifest equality | both hashes equal the manifest values exactly |
| GDB run | one inferior, `MH_01_easy --max-frames 5`; exited after the captured return |
| post-run processes | no `basalt_vio` or `gdb` process |

The clean binary, clean library, and raw GDB output were each scanned for
`BA_REL_DIAG`, `RUNTIME_REL`, `BASALT_TRACE_JSONL`, and
`BASALT_IMU_TRACE_JSONL`; all were clear. The diagnostic M7cp capture is used
only as a comparison reference and retains the diagnostic checkout's
pre-existing markers as documented by M7cp.

## Live symbol boundary and dimensions

The clean symbol entry was `0x7ffff70cfab0`, with clean `libbasalt.so` mapped
at `0x7ffff6e00000` (entry offset `0x2cfab0`). The dynamically derived return
was `0x7ffff70cff00` (offset `0x2cff00`). The raw capture recorded one entry
and one return. The complete returned buffers are:

| buffer | logical shape | scalar/layout | bytes |
|---|---:|---|---:|
| H | 75 × 75 | binary32 Eigen column-major (`H(0,0), H(1,0), ...`) | 22,500 |
| b | 75 × 1 | binary32 Eigen dynamic vector (`b[0], ...`) | 300 |

The immediately readable pose-damping fields were both zero:
`pose_damping_diagonal = 0x00000000` and
`pose_damping_diagonal_sqrt = 0x00000000`. Cost/lambda are estimator-level
values, not members of this linearization object; no stable cost/lambda
boundary was available at this hook. The Rust snapshot's
`cost.before = 4215.86474609375` and `lambda = 9.999999747378752e-05` remain
comparison metadata only.

## AOM and landmark index record

The captured AOM had `items=5` and `total_size=75`, in this exact order:

| ordinal | timestamp (ns) | frame | offset | DoF |
|---:|---:|---:|---:|---:|
| 0 | 1403636579763555584 | 0 | 0 | 15 |
| 1 | 1403636579813555456 | 1 | 15 | 15 |
| 2 | 1403636579863555584 | 2 | 30 | 15 |
| 3 | 1403636579913555456 | 3 | 45 | 15 |
| 4 | 1403636579963555584 | 4 | 60 | 15 |

`landmark_block_idx` contained 61 `size_t` entries (488 bytes), ranging from
offset 0 through 1148. Its binary hash is identical to the M7cp diagnostic
capture (`94401dd165389d0c6ee21a0b2a30ffaa1faf7566325079525c0ce517738584e2`).
The AOM raw object includes run-specific pointers; its order and dimensions,
rather than pointer bytes, are the portable record.

## Clean versus diagnostic and Rust

All comparisons cast Rust JSON values to binary32 and transpose Rust's
row-major H into the native Eigen column-major sequence before comparing.

| comparison | H bit mismatches | H maximum absolute delta | b bit mismatches | b maximum absolute delta |
|---|---:|---:|---:|---:|
| clean M7ct vs diagnostic M7cp | 488 / 5625 (8.68%) | 4.8125 | 15 / 75 (20.00%) | 0.0078125 |
| clean M7ct vs current Rust M7cm | 2451 / 5625 (43.57%) | 960.0 | 69 / 75 (92.00%) | 97.6171875 |
| diagnostic M7cp vs current Rust M7cm | 2451 / 5625 (43.57%) | 960.0 | 69 / 75 (92.00%) | 97.6171875 |

The clean/diagnostic H comparison first differs at native sequence index 1
(`H(1,0)`: clean `47020e8f`, diagnostic `47020e85`) and has its largest delta
at `H(20,4)` (4.8125). The b comparison first differs at `b[0]` by one
binary32 bit (`45de4e52` vs `45de4e51`) and has a maximum delta of 0.0078125.

The clean-vs-Rust mismatch counts are exactly the same as the diagnostic-vs-
Rust counts. Thus the small clean/diagnostic numerical variation does not
explain the outstanding Rust mismatch: the clean oracle still differs from
Rust at 2451 H entries and 69 b entries, with the same 960.0 H and 97.6171875
b maximum deltas reported by M7cp.

## Artifacts

- Machine-readable capture: [`target/m7ct_clean_frame4_hb.json`](../../target/m7ct_clean_frame4_hb.json)
- Comparison summary: [`target/m7ct_clean_frame4_hb_comparisons.json`](../../target/m7ct_clean_frame4_hb_comparisons.json)
- H binary32 dump: [`target/m7ct_clean_frame4_H.f32`](../../target/m7ct_clean_frame4_H.f32), SHA-256 `6b7a252647e984a13b134a357fddd9f3a6c43f85d748a0591ede87efa726d943`
- b binary32 dump: [`target/m7ct_clean_frame4_b.f32`](../../target/m7ct_clean_frame4_b.f32), SHA-256 `db14f26f17d01bd16e61772ffe4280a113515c98365e013ea535ff7806c41a57`
- AOM raw dump: [`target/m7ct_clean_frame4_aom.bin`](../../target/m7ct_clean_frame4_aom.bin), SHA-256 `5e0dd5f0572f2ffcb97d60f9b5f9906acfd5a3c0dbc309210232aa3f1beec40d`
- Landmark index dump: [`target/m7ct_clean_frame4_landmark_idx.bin`](../../target/m7ct_clean_frame4_landmark_idx.bin), SHA-256 `94401dd165389d0c6ee21a0b2a30ffaa1faf7566325079525c0ce517738584e2`
- Raw GDB output: [`target/m7ct_capture.out`](../../target/m7ct_capture.out), SHA-256 `8c0355d6b5930188f37663900b42b935a0c5d4bb1e5ef01e97ac8284f58d1d9e`

