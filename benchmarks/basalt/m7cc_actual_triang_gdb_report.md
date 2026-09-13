# M7cc actual-call triangulation GDB report

Date: 2026-08-23 (JST)  
Scope: pinned Basalt `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`, existing
`build/core-relwithdebinfo/basalt_vio` and `libbasalt.so`, frame 0 / track 1.

## Result

The required authoritative actual call was **not captured** in this bounded
attempt.  No production Rust source, upstream source, or native binary was
rebuilt or instrumented in this turn.  The two completed GDB artifacts were
`target/m7cc_actual_gdb.out` and `target/m7cc_entry_all.out`; all processes
were stopped afterward.  The partial corrected-address retry was stopped
before it reached a breakpoint and is not evidence.

The known target fixture remains:

```text
f0 bits = bf2a7307 be9af9d0 3f2e94d6
f1 bits = bf2d0c0d be93697d 3f2da937
T_0_1 qxyzw = 3befaa96 39eadb91 3a8bfffa 3f7ffe35
T_0_1 t     = 3de1c150 b87a2400 39ac1888
```

The prior fresh5 authoritative endpoint still supplies the only confirmed
observable divergence:

```text
direction x: Rust becaaef7, native becaaef6
direction y: Rust be383810, native be38380d
rho:         Rust 3e160ef3, native 3e160f08
```

Because the actual-call operands were not observed, this does not establish
whether the first differing scalar is in pose composition, `P2`, `A`, SVD,
normalization, or the sign path.  No general `landmarks.rs`/`camera.rs` patch
is justified.

## What the completed runs actually measured

The handoff’s absolute addresses were from an older load bias.  In the live
WSL mapping used by the attempted probe, the pinned library load base was
`0x7ffff6e00000`, so the requested offsets would have been:

| requested offset | live address |
|---:|---:|
| `+0x4d7400` triangulate entry | `0x7ffff72d7400` |
| `+0x50b613` call-site setup | `0x7ffff730b613` |
| `+0x50b668` call | `0x7ffff730b668` |
| `+0x4d7f3c` norm start | `0x7ffff72d7f3c` |
| `+0x4d7f78` post norm | `0x7ffff72d7f78` |
| `+0x4d7fbb` post sign/return | `0x7ffff72d7fbb` |

`m7cc_actual_gdb.out` and `m7cc_entry_all.out` instead used
`0x7ffff70e6208`, `0x7ffff711a41b`, `0x7ffff711a470`, and
`0x7ffff70e6d44/6d80/6dc3`.  Consequently they did not stop at the requested
offsets in this process.  GDB reported the first of those stale locations as
an inlined Sophus `so3.hpp:372` location, not the triangulate entry.

The stale entry stop printed 81 unrelated packets, with the repeated first
packet:

```text
rdi words = bbed3c0f 3bf71cd4 3f33a827 3f365a1e
rsi words = bb19188b 3c55012e 3f33d4ed 3f362af6
```

No packet matched the target ray bits; there was no `MATCH ENTRY`, no
call-site packet, and no normalization packet.  These words must not be
treated as `T_0_1`, `P2`, `A`, `V.col(3)`, or any authoritative triangulation
operand.

The binary also emitted pre-existing `BA_REL_DIAG`/`RUNTIME_REL` stderr lines.
Those were already present in the pinned binary and were not used as actual
triangulation evidence; this turn did not add or remove that upstream build.

## ABI note for the next bounded probe

The disassembly of the pinned emitted function returns the 16-byte
`Eigen::Matrix<float,4,1>` through `xmm0/xmm1`.  Its prologue is:

```text
mov r12,rdi
mov rbx,rsi
mov rsi,rdx
```

and its epilogue returns through `xmm0/xmm1`.  Thus, for this concrete binary,
the three C++ reference arguments are `$rdi` (f0), `$rsi` (f1), and `$rdx`
(`T_0_1`); there is no hidden sret `$rdi`.  The next probe should verify this
at the corrected live entry before filtering.  At the corrected entry, filter
the first three words of `$rdi`/`$rsi` against the target ray bits, then dump
`$rdx` and the requested norm stack locations.  At the call-site setup, use
the post-move argument registers (or the equivalent `$r14`, `[$rbp-0x560]`,
`$rbx` values immediately before the moves), rather than stale absolute
addresses.

Until that corrected uninstrumented capture exists, the first exact internal
divergence remains unresolved; the final direction/rho mismatch above is only
the first observable endpoint divergence.
