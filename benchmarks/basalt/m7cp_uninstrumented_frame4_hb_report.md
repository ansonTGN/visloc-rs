# M7cp uninstrumented frame-4 H/b capture

Date: 2026-08-23 JST. The one bounded capture completed successfully against
the existing pinned Basalt executable with `--max-frames 5`. At the first
frame-4 `LinearizationAbsQR<float, 6>::get_dense_H_b` return it retained the
complete 5625-word H matrix and 75-word b vector as binary32 IEEE-754 bits.
The machine-readable artifact is
[`target/m7cp_uninstrumented_frame4_hb.json`](../../target/m7cp_uninstrumented_frame4_hb.json).

## Provenance and run boundary

- Pinned checkout: `/root/visloc-basalt-oracle-0f3b2b52`, commit
  `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
- Dataset: `MH_01_easy`; `num-threads 4`; `--max-frames 5`; frame 4.
- Existing executable SHA-256:
  `edb6586f7ec03fa055b262840db9ae8216fb4ec18d9291aa6e1dc4714d06188f`.
- Existing `libbasalt.so` SHA-256:
  `0cea5175689d28605a96cf1aad934a43f3c6487076eb490870280ebb78c83032`.
- No Rust/upstream production source was edited, rebuilt, or instrumented.
  The executable is the same diagnostic build described by M7co, so its
  pre-existing `BA_REL_DIAG`/`RUNTIME_REL` text remains in the raw output; no
  diagnostic changes were made for this capture.
- Raw GDB output: `target/m7cp_capture.out`, SHA-256
  `d7cb3978a8beb05514c589e98c9ded2190665165a3c1def2127884998eeb4b21`.

## Live address and ABI record

GDB had `disable-randomization on`, but the base and call addresses were still
recorded from the live inferior. The executable’s `libbasalt.so` mapping began
at `0x7ffff6e00000`.

| item | live address | offset from lib base |
|---|---:|---:|
| `get_dense_H_b` entry | `0x7ffff70f0fb0` | `+0x20f0fb0` |
| exact return after `b = std::move(r.b_)` | `0x7ffff70f1400` | `+0x20f1400` |
| `this` | `0x7fffe801d640` | — |
| H Eigen descriptor | `0x7ffff5353bf0` | — |
| b Eigen descriptor | `0x7ffff5353ba0` | — |
| H data | `0x7fffe8042aa0` | — |
| b data | `0x7fffe80482c0` | — |

The return breakpoint was derived dynamically as `$entry_pc + 0x450`, rather
than relying on a stale absolute address. The descriptors reported H as
75×75 and b as 75 rows (Eigen’s vector descriptor has columns 0). H is
default-column-major; the raw sequence is `H(0,0), H(1,0), ...`, while b is
`b[0], b[1], ...`.

## AOM and dimensions

The live AOM pointer was `0x7ffff5353c70`, with `items=5` and
`total_size=75`. GDB’s map print gave this exact ordered map:

| timestamp (ns) | frame | offset | DoF |
|---:|---:|---:|---:|
| 1403636579763555584 | 0 | 0 | 15 |
| 1403636579813555456 | 1 | 15 | 15 |
| 1403636579863555584 | 2 | 30 | 15 |
| 1403636579913555456 | 3 | 45 | 15 |
| 1403636579963555584 | 4 | 60 | 15 |

`landmark_block_idx` covered 61 `size_t` entries (488 bytes), with offsets
retained in the JSON and in
`target/m7cp_frame4_landmark_idx.bin` (SHA-256
`94401dd165389d0c6ee21a0b2a30ffaa1faf7566325079525c0ce517738584e2`). The
raw 64-byte AOM object is in `target/m7cp_frame4_aom.bin` (SHA-256
`33cf226cf406a35b6613ae6578579aff8402fd41374fd92fa4d7ae0d0b9e8288`).

The complete solver buffers are:

- `target/m7cp_frame4_H.f32`, 22,500 bytes (5625 words), SHA-256
  `562397068432f324afe5617cf8cacbf1b9633ff691ffe020a244a028d8f9bfc1`.
- `target/m7cp_frame4_b.f32`, 300 bytes (75 words), SHA-256
  `3f607b296590d21c9545af9c7919270fedcc6a94e6d78de0ea7e7f4cafd50534`.

The immediately readable pose-damping fields in the linearization object were
both zero (`pose_damping_diagonal = 0x00000000`, square root likewise). The
LM cost and lambda are estimator-level values, not members of this object, so
no direct uninstrumented cost/lambda was available at this hook. The Rust
comparison’s `cost.before=4215.86474609375` and
`lambda=9.999999747378752e-05` are comparison metadata only, not claims about
the native capture.

## Rust iteration-0 comparison

The comparator casts each Rust JSON value to binary32 and transposes its
row-major JSON H into the native Eigen column-major sequence before comparing
bits. Both the current `target/m7cm_fresh5_detail_20260823.jsonl` snapshot and
the requested `target/m7ce_fresh5_detail_20260823.jsonl` snapshot produced the
same summary below.

| buffer | bitwise mismatches | first mismatch | maximum absolute float delta |
|---|---:|---|---:|
| H (5625) | 2451 | H[0,0]: native `4de48a64` vs Rust `4de48a6f` (479284352 vs 479284704) | H[34,34]: native `4e0184d8` vs Rust `4e0184e7`, 960.0 |
| b (75) | 69 | b[0]: native `45de4e51` vs Rust `45de4e23` (7113.78955078125 vs 7113.76708984375) | b[49]: native `47bd2f14` vs Rust `47bd5fe3`, 97.6171875 |

The full bit arrays, metadata, both comparison records, and all capture hashes
are in the JSON artifact. The GDB inferior exited cleanly and a post-run
process check found no `basalt_vio` or `gdb` process.
