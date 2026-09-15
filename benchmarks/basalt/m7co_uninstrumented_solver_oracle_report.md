# M7co solver-oracle recovery report

Date: 2026-08-23 JST  
Scope: the pinned Basalt checkout `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`,
MH_01_easy, `--max-frames 5`, frame 4's first `get_dense_H_b` call.

## Disposition

The capture is useful for locating and identifying the solver buffers, but it
is **not** a complete initial H/b/cost oracle.  The return hook dumped only 64
float words of H and 64 of b; H is 75x75 (5625 words), b is 75x1 (75 words),
and no cost, damping diagonal, or solve step was captured.  Consequently no
machine-readable exact initial H/b/cost oracle is emitted and no solver
numeric conclusion is drawn from the partial prefix.

The raw capture is [`target/m7co_probe.out`](../../target/m7co_probe.out)
(SHA-256 `108c67f62bff8e069b9cb3d9558944fdb4419f69d3774e24ba8d68240b2ce6b5`).
The entry-only precursor is [`target/m7co_entry_only.out`](../../target/m7co_entry_only.out)
(SHA-256 `0cb0c159f15e13de9efb6ccc839ece91e95fd040c3e90f65f36d8d41c957c63f`).

No production Rust source was changed.  The one additional GDB run used the
existing binary/library and the already-known return breakpoint; it did not
rebuild or add instrumentation.  The inferior and GDB exited, and a final
process check found no `basalt_vio` or `gdb` process.

## Exact symbol and breakpoints

The entry breakpoint in `m7co_entry_only.cmd` and `m7co_probe.cmd` is:

```text
basalt::LinearizationAbsQR<float, 6>::get_dense_H_b(
    Eigen::Matrix<float, -1, -1, 0, -1, -1>&,
    Eigen::Matrix<float, -1, 1, 0, -1, 1>&) const
```

With the fixed ASLR base reported by the earlier M7 captures,
`libbasalt.so` base is `0x7ffff6e00000`:

| location | live address | library offset | source/role |
|---|---:|---:|---|
| symbol entry | `0x7ffff70f0fb0` | `+0x20f0fb0` | `linearization_abs_qr.cpp:544` |
| exact return tbreak | `0x7ffff70f1400` | `+0x20f1400` | `linearization_abs_qr.cpp:598`, after `H = std::move(r.H_); b = std::move(r.b_)` |

The return address is the exact pointer breakpoint already present in
`target/m7co_probe.cmd`; no address was inferred or added during the run.

## Registers and object layout

At the SysV AMD64 entry ABI, the capture printed:

```text
rdi (this) = 0x7fffdc01d640
rsi (H object) = 0x7ffff5353bf0
rdx (b object) = 0x7ffff5353ba0
```

The precursor run had the same H/b object addresses and a different stack
allocation for `this` (`0x7fffe801d640`), confirming that these are live
arguments, not hard-coded data pointers.

The returned Eigen descriptors are dynamic, default-column-major matrices:

```text
H object @ 0x7ffff5353bf0
  +0x00 data = 0x7fffdc0408a0
  +0x08 rows = 75
  +0x10 cols = 75

b object @ 0x7ffff5353ba0
  +0x00 data = 0x7fffdc0460a0
  +0x08 rows = 75
  +0x10 cols = 0 (Eigen vector descriptor)
```

Thus the sequential H dump is column-major (`H(0,0), H(1,0), ...`) and the
b dump is `b[0], b[1], ...`.  `m7co_probe.cmd` used `x/64wx` for each buffer,
so only the first 64 words were retained.

The `LinearizationAbsQR` object fields needed to identify the active problem
were also captured:

```text
this + 0xd8  landmark_block_idx.begin() = 0x7fffdc03d8b0
this + 0xe0  landmark_block_idx.end()   = 0x7fffdc03da98
this + 0x110  aom pointer              = 0x7ffff5353c70
```

The index span is `0x1e8` bytes, i.e. 61 `size_t` entries.  The AOM words at
`0x7ffff5353c70` include `+0x28 = 5`, `+0x30 = 5`, and `+0x38 = 0x4b`
(five state blocks and total size 75), agreeing with the frame-4 reference.

The first 64 captured words, retained here as exact IEEE-754 binary32 bits,
are:

```text
H:
4de48a64 47020e83 c8ad6b51 461e060b c9c3ed5d 47014eb0 4b107f7d 3e799637
bf6ba7c6 45dfb93b 47793df7 45658fe9 434dfaae 43c00b04 c1d1ef3e cdb4bc93
c4c44bcb 4830977a c60c693f 498caa0c c84df974 4b107f6e be883f6b bf9ae38e
00000000 00000000 00000000 00000000 00000000 00000000 c7ee5760 c5dd7b58
47a85cd6 c3fd0fd0 4867122e c641db30 00000000 00000000 00000000 00000000
00000000 00000000 00000000 00000000 00000000 c7945875 c64c06f7 4749b916
4414b3ba 47f760c2 47ad6348 00000000 00000000 00000000 00000000 00000000
00000000 00000000 00000000 c753af3a c634ec03 470e89d0 c3db309b

b:
45de4e52 c5d8733e c7632796 4601bc01 c86cdf07 46d0c76d bbb6033d b9eb71e2
3c07eabe bbcf6e23 bf2103e1 bdc8226d 3980f02f b7eef33b 398df94c c4b226f8
44aae899 c5617693 c4b91913 c6fd9a1d c585070d bbe4655e b8224dfa 3c581d31
3db11955 3f5a1a8e bacebdbc b90d7c1c 37876d2e 368cb85b c4ff4c10 448543cc
4665bb31 c605dcb3 47600780 c48f89db 3c65f48a 3a493786 bc60718f 3b56ca7e
bc10f9b8 3d2193ce 3a36c34a b7d8a30a b99a54bc c4e9aeb2 44fe53fe 46a92a02
c538d271 47bd2f14 c6076b03 bc3258ce 3a4fce5b 3d40acff be290384 bf9bc8fa
3c61484e 398a2cb7 b795c639 37a16064 c4de17c6 4519a648 46c67666 45922bf3
```

## Comparison with M7al

The comparison reference is the M7al upstream snapshot at line 124 of
`target/m7al_upstream_f4_20260821T000200Z/iteration.jsonl` (full trace
SHA-256 `18e87cff493f758729f91e8270c9968e55bc8fb33cdcf1c3307fce9119711f7a`
as recorded in the M7al report).  It has the same frame-4 shape: five AOM
blocks, 61 landmarks, and a 75-column H/b system.  Comparing the captured
prefix in column-major order gives:

| prefix | captured words | bitwise differences | maximum absolute float delta |
|---|---:|---:|---:|
| H | 64 of 5625 | 19/64 | 0.3125 |
| b | 64 of 75 | 12/64 | 0.03125 |

First differences against that reference are:

```text
H[1,0]:  captured 47020e83 (33294.51), reference 47020e8b (33294.54)
H[3,0]:  captured 461e060b (10113.51), reference 461e0604 (10113.50)
b[1]:    captured c5d8733e (-6926.405), reference c5d87342 (-6926.407)
```

The M7al instrumented iteration-start record reports `cost.before =
4215.9326171875` and `lambda = 9.99999974737875e-05`; its trial/accepted
record reports model cost `1866.80517578125`, actual cost `412.95501708984375`,
and step norm `0.22672636806964874`.  None of those cost values was captured
by M7co, so they are reference metadata, not uninstrumented measurements.

## Instrumented-field validity boundary

The existing executable used for this run is the M7bo diagnostic build:

```text
basalt_vio SHA-256  edb6586f7ec03fa055b262840db9ae8216fb4ec18d9291aa6e1dc4714d06188f
libbasalt.so SHA-256 0cea5175689d28605a96cf1aad934a43f3c6487076eb490870280ebb78c83032
```

The external checkout is dirty in the five diagnostic files listed by M7bo
(`ba_utils.h`, `linearization_abs_qr.cpp`, `sqrt_keypoint_vio.cpp`, and the
two landmark-block headers).  The M7co output visibly contains `BA_REL_DIAG`
and `RUNTIME_REL`, so it is not a pristine no-diagnostic build even though
this GDB run added no new instrumentation.

Use the earlier artifacts as follows:

| field/artifact | disposition |
|---|---|
| `get_dense_H_b` symbol, ABI registers, Eigen descriptors, dimensions, and the 64-word prefix above | usable provenance/layout facts from the live call |
| M7al record ordering, AOM/landmark counts, row-span/schema, and iteration-control shape | usable structural reference only |
| M7al full H/b, damping, step, cost, and exact LM acceptance as a clean native numeric oracle | **invalid for that claim**; the trace hooks/codegen and TBB reduction context perturb numeric values; the M7co prefix already differs |
| M7bo selected relative-pose runtime tuple | usable for that exact call boundary; it was emitted by the actual upstream call and is documented as such in M7bo |
| M7ch/M7ck post-`addLandmark` direction/rho words | usable stored-endpoint facts (M7ck reports 61/61 binary32 agreement), but not a general clean arithmetic trace |
| `BA_REL_DIAG`/`RUNTIME_REL` text and timing/stat counters | diagnostic evidence only, not solver-oracle fields |

The remaining exact oracle would require a fresh run whose return command dumps
all 5625 H words and all 75 b words, plus a known cost boundary.  That would
be a separate capture and is outside this bounded recovery.
