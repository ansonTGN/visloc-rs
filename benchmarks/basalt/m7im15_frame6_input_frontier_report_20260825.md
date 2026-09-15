# M7IM15 frame-6/iteration-6 IMU input frontier

Date: 2026-08-25 (JST)  
Pinned native source: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`  
Comparison artifact: `target/m7im15_frame6_input_compare_20260825.json`

## Pairing and layout

The two native records are paired with the two Rust `local_blocks` in order:

| Rust block | Native global `ImuBlock` ordinal | Active offsets | Link | `start_t_ns` / `dt_ns` |
| --- | ---: | --- | --- | --- |
| 0 | 60 | `[6, 21]` | 4 → 5 | `1403636579963555584` / `49999872` |
| 1 | 61 | `[21, 36]` | 5 → 6 | `1403636580013555456` / `50000128` |

Both records match on `start_t_ns`, `dt_ns`, `end_t_ns`, `from_linearized`, and
`to_linearized` (2/2 exact for every metadata field).  The native logger's
Eigen matrices are column-major; the comparison uses the corresponding Rust
`factor_input` `*_bits` arrays without transposition.  State lanes are flattened
as `translation + quaternion_xyzw + velocity + bias_gyro + bias_accel` (16
float32 bit patterns).  Raw fields use `raw_r_fej_bits`, `raw_r_bits`, and
`raw_jacobian_bits`, not the older block aliases.

## Result

The first non-exact input is the **from-FEJ (linearized) state**, at flat index
0: **`translation[0]`** of native ordinal 60.  Thus the frame-6 LM branch
divergence is already present at the state frontier, before comparing the IMU
delta/covariance or residual/J output.  The same lane is also non-exact for
ordinal 61; all four state lanes are non-exact for both blocks.  The native
capture has no explicit FEJ boolean, so FEJ/current flags themselves are not
invented in the comparison; the four captured state lanes are compared directly.

Selected aggregate bit counts over both blocks (exact / total):

| Component | Exact / total | First mismatch (block 0) |
| --- | ---: | --- |
| State `from_fej` | 0 / 32 | flat 0: `bb21bf38` → `bad65ca6` |
| State `from_current` | 0 / 32 | flat 0: `bb1713d4` → `bae1b29e` |
| State `to_fej` | 0 / 32 | flat 0: `bae60197` → `ba9ce211` |
| State `to_current` | 0 / 32 | flat 0: `bae60197` → `ba9ce211` |
| Delta rotation matrix | 0 / 18 | 0: `3f7ffcd3` → `3f7ffc90` |
| Delta position | 0 / 6 | 0: `3c2dc1d4` → `3c2fbe7a` |
| Delta velocity | 0 / 6 | 0: `3edaaf74` → `3edd2d15` |
| `d_state_d_bg` | 0 / 54 | 0: `b16ace96` → `b09be4e4` |
| `d_state_d_ba` | 18 / 54 | 0: `baa3d66e` → `baa3d666` |
| Covariance | 8 / 162 | 1: `2814a0c1` → `281705e2` |
| Raw FEJ residual | 0 / 18 | 0: `3a80eff0` → `3992f120` |
| Raw selected residual | 0 / 18 | 0: `b741d800` → `360a8000` |
| Raw Jacobian | 288 / 540 | 0: `3eaaff0c` → `3ead956c` |
| Square-root information | 78 / 162 | 6: `c181568f` → `c182d700` |
| Whitened residual | 0 / 18 | 0: `bc98d451` → `bd8e2179` |
| Whitened Jacobian | 180 / 540 | 3: `c4e93009` → `c4e9d9d3` |

The detailed per-block exact/mismatch counts and first differences are in the
JSON artifact.  This is a diagnostic-only comparison; no production Rust code
was changed, and no commit or push was performed.

## Artifact hashes

```text
target/m7im15_frame6_input_compare_20260825.json
  sha256 C64EBA14E3EB70A4DEE499991FD97CD8A3BE3FA8794B12B36C50F0760B33CB54
target/m7im15_native_input_local_frame6_iter6_20260825.json
  sha256 B7978F136019B4DB21C5077C0134DE874B0C168C8D54597AD418B79844B53C10
target/m7im15_rust_local_capture_frame6_iter6_20260825_v3_final2.jsonl
  sha256 1F1D2F2373D33EDB274E85318E45A37DF7DD8CFB4203B6948B69006F8A83F762
```
