# M7db clean frame-4 estimator-state capture

Date: 2026-08-23 JST  
Status: **Captured — five clean native frame states, raw float32 lane bits**

This artifact records the first frame-4 `linearizeProblem(bool*)` entry from
the pinned clean Basalt binary used by M7cr/M7ct. The existing frame-4 filter
was reused: the `LinearizationAbsQR<float,6>*` is in `rdi`, its estimator is
read at `this+0xf0`, and capture is selected only when the aligned
`frame_states` map count is five. No `ptype`, pretty printer, typed object
expansion, native source edit, Rust edit, rebuild, commit, or push was used.

## Clean provenance

| item | value |
|---|---|
| checkout | `/root/visloc-basalt-clean-m7cr-20260823` |
| upstream commit | `0f3b2b52c807f70ff4e2973ce253c73329eea7bc` |
| `basalt_vio` SHA-256 | `89e0324ccd04c3b615bf7e945aa32480a2f86524987aa00eaf152dcd6b1d242c` |
| `libbasalt.so` SHA-256 | `55f842642b370edf4025afd1d66c7b9d9011e5b4ce08efeb9d76be401b45b2cc` |
| run | `MH_01_easy`, four native threads, `--max-frames 5`, `/marg_data_m7db` |
| live `L` | `0x7fffe801d430` |
| live estimator / `frame_states` map | `0x5555558d3e50` |
| live AOM | `0x7ffff5374d70` |
| map root / count | `0x7fffe8002ee0` / `5` |
| walked map nodes | `5` |

## Raw layout and capture

The aligned `std::map` header arithmetic is root `map+0x10` and count
`map+0x28`. Each map node's mapped value begins at `node+0x28`; the script
dumped 96-byte raw windows at `node+0x78` and `node+0xd8` for the linearized
and current state slots. In those direct little-endian word windows the
requested fields are:

| field | 32-bit word indices |
|---|---:|
| quaternion (xyzw) | 2..5 |
| translation (xyz) | 6..8 |
| velocity (xyz) | 10..12 |
| gyro bias (xyz) | 13..15 |
| accel bias (xyz) | 16..18 |

The first two words are the state alignment/timestamp boundary; timestamps
were verified from the captured 64-bit timestamp words and the five-entry AOM
order. The direct state field words are identical in `state_linearized` and
`state_current` for all five records. Every gyro- and accel-bias lane is
`00000000`.

The machine-readable record is
[`target/m7db_clean_frame4_states.json`](../../target/m7db_clean_frame4_states.json).
The raw GDB text is
[`target/m7db_clean_state_capture.out`](../../target/m7db_clean_state_capture.out)
(SHA-256
`68bb355e7de55ef6a0876b816f63f8a37d2a2ef7d25c86c2963d8481d2e5e135`), and the
raw binary window record is
[`target/m7db_clean_frame4_states_raw.bin`](../../target/m7db_clean_frame4_states_raw.bin)
(SHA-256
`77563853717c97f122a78a02c2f29a4ea6035cd84720b9e1f1f9a97bc3b49f4b`).

The first bounded diagnostic output is retained as
[`target/m7db_clean_state_capture_failed_offset.out`](../../target/m7db_clean_state_capture_failed_offset.out).
Its cause was an eight-byte-shifted map-count read (`map+0x20`, which is the
right-child pointer for this aligned map), so it emitted no state record. It
was not used as native evidence.

## Native state lane record

The five AOM entries are timestamps
`1403636579763555584`, `1403636579813555456`,
`1403636579863555584`, `1403636579913555456`, and
`1403636579963555584`, respectively. All requested native f32 bit patterns,
including pose quaternion `xyzw`, translation `xyz`, velocity, gyro bias, and
accel bias, are in the JSON artifact under both `state_linearized` and
`state_current`.

## Rust M7cm comparison

Comparison uses `target/m7cm_fresh5_detail_20260823.jsonl`, snapshot
`iteration=0`, `phase=iteration_start`; Rust f64 values are cast to f32 before
bit comparison. There are **4 mismatches / 75 lanes**. The first mismatch is:

| frame | field/lane | clean native | Rust M7cm | delta |
|---:|---|---|---|---:|
| 2 | quaternion `xyzw[3]` | `3f17e7a6` | `3f17e7a4` | -2 ULP |

The remaining mismatches are frame 4 quaternion `xyzw[3]`
(`3f159719` vs `3f159717`, -2 ULP), velocity `y`
(`bc8bed06` vs `bc8bed08`, +2 ULP), and velocity `z`
(`be67f1c6` vs `be67f1c8`, +2 ULP). No live `gdb` or `basalt_vio` process
remains.
