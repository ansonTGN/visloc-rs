# M7ft live ordinal-109 camera-J/Jpp/Jp frontier

Date: 2026-08-24 JST  
Status: **Complete — bounded clean-native in-context capture plus temporary
current-Rust intermediate probe; production arithmetic unchanged.**

## Selection and provenance

This is the M7fs first weighted-Jp mismatch: clean-native ordinal **109**,
serialized factor **4**, track **9**, observation **1**, relation
`same_timestamp_stereo`, host `0/0`, target `0/1`.  The exact key is pixel
`42c9e67c,42895a4f`, direction `be9dd414,be637e0a`, inverse distance
`3e21489f`.  The live four-thread run observed that key at worker interleave
ordinal **209**; the exact binary32 key, rather than that nondeterministic
ordinal, identifies the requested factor.

The native capture used the pinned clean artifact from M7fp/M7cr:

- checkout `/root/visloc-basalt-clean-m7cr-20260823`
- commit `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`
- `basalt_vio` SHA-256 `89e0324ccd04c3b615bf7e945aa32480a2f86524987aa00eaf152dcd6b1d242c`
- `libbasalt.so` SHA-256 `55f842642b370edf4025afd1d66c7b9d9011e5b4ce08efeb9d76be401b45b2cc`
- `MH_01_easy`, four native threads, `--max-frames 5`, `/marg_data_m7ct`

No standalone clean replay supplied the operands.  GDB stopped at the actual
in-context fixed-size Eigen product: `0x27576f` before the call and `0x275774`
after it.  At the pre-product stop, `rsi == rbp-0x110` was the live camera J,
`rdx == rbp-0xb0` the live Jpp, and `rdi` the output Jp; all pointer checks
were true.

## Live native values

Words are little-endian f32, Eigen column-major:

```text
camera J (2x4): 43c3a1b0,c2917418,c291e436,43e06dcc,437d6358,432a65a0,00000000,00000000
Jpp (4x3):       3fba6356,be4b3e16,3f722147,00000000,be534c85,3fcdb62b,3f27a9f7,00000000,bde1c0f0,3997d300,b9e136c8,3f800000
raw Jp (2x3):    444df834,c2074d44,c2000f98,4453fe6c,c22d09a2,41012d40
```

## Current-Rust probe and first mismatch

A temporary in-tree unit probe evaluated the current Rust f32 helper chain with
the live native transform, camera parameters, direction, and rho.  It printed
the same camera-J/Jpp/product intermediates and was removed immediately after
the focused run (`1 passed, 0 failed`).  Its values were:

```text
camera J (2x4): 43c3a1b0,c2917418,c291e436,43e06dce,437d6358,432a65a0,00000000,00000000
Jpp (4x3):       3fba6356,be4b3e16,3f722147,00000000,be534c85,3fcdb62b,3f27a9f7,00000000,bde1c0f0,3997d300,b9e136c8,3f800000
raw Jp (2x3):    444df834,c2074d48,c2000f98,4453fe6e,c22d09a2,41012d40
weighted Jp:     44cdf834,c2874d48,c2800f98,44d3fe6e,c2ad09a2,41812d40
```

| boundary | exact | first/differences |
|---|---:|---|
| camera J | 7/8 | lane 3 (`r1c1`): native `43e06dcc`, Rust `43e06dce`, 2 ULP |
| Jpp | 12/12 | exact |
| raw Jp | 4/6 | lane 1 (`r1c0`) 4 ULP; lane 3 (`r1c1`) 2 ULP |
| weighted Jp | 4/6 | lane 1 4 ULP; lane 3 2 ULP |

Therefore the first mismatch is already at the camera projection Jacobian,
lane 3 (`r1c1`).  Jpp is exact 12/12; the camera-J difference then propagates
through the product to the two raw/weighted Jp mismatches.  The weighted
frontier is not a weight-only discrepancy (`sqrt_weight=2`, `40000000`).

## Artifacts and hygiene

- [GDB command](../../target/m7ft_ordinal109_live.cmd)
- [raw GDB output](../../target/m7ft_ordinal109_live.out)
- [machine-readable capture and comparison](../../target/m7ft_ordinal109_live.json)

The temporary Rust test and its output were removed after capture.  No
production source, clean source, build output, commit, or push was changed.
