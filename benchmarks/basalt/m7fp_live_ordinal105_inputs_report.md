# M7fp live ordinal-105 in-context Jp/Jpp capture

Date: 2026-08-23 JST  
Status: **Complete — bounded clean-native read-only GDB capture.**

## Scope and provenance

This capture used the pinned, unmodified clean artifact:

- checkout: `/root/visloc-basalt-clean-m7cr-20260823`
- Basalt commit: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`
- `basalt_vio` SHA-256: `89e0324ccd04c3b615bf7e945aa32480a2f86524987aa00eaf152dcd6b1d242c`
- `libbasalt.so` SHA-256: `55f842642b370edf4025afd1d66c7b9d9011e5b4ce08efeb9d76be401b45b2cc`
- dataset/config: `MH_01_easy`, `--max-frames 5`, four native threads
- marginal data: `/marg_data_m7ct`

The selection is the exact M7ef ordinal-105 key: pixel
`42bc0000,42580000`, direction `be9dd414,be637e0a`, inverse distance
`3e21489f`.  The exact key arrived at observed ordinal **114** in this live
four-thread run because the global worker interleave is not stable across
runs; the key words, rather than a changed ordinal, identify the requested
factor.  Its transform is the identity packet from the M7fn same-TimeCam
case.

No source, clean build, commit, or push was changed.  The capture reads raw
little-endian inferior memory/register words only; no `ptype`, pretty-printer,
or standalone probe supplied the live operands.

## Product boundary and stack resolution

The dynamic library base was `0x7ffff6e00000`.  Relevant live addresses were:

| boundary | ELF offset | live address |
|---|---:|---:|
| `linearizePoint` entry | `0x275030` | `0x7ffff7075030` |
| pre-product call | `0x27576f` | `0x7ffff707576f` |
| post-product boundary | `0x275774` | `0x7ffff7075774` |

The in-context call is the fixed-size Eigen `Matrix<float,2,4> * Matrix<float,4,3>`
assignment.  The disassembly establishes the operands immediately before the
call:

```text
0x27569c: lea    -0xb0(%rbp),%r15
0x275757: lea    -0x110(%rbp),%rsi
0x275761: mov    %r15,%rdx
0x275764: mov    %r12,%rdi
0x27576f: call   ... Product<Matrix<float,2,4>,Matrix<float,4,3>>
0x275774: mov    $0x1,%eax
```

Therefore, at the stopped pre-call instruction, `rsi == rbp-0x110` is the
actual camera projection Jacobian, `rdx == rbp-0xb0` is the actual `Jpp`, and
`rdi` is the `d_res_d_p` output.  The capture verified all three pointer
relationships in memory.  Matrices below are raw f32 words in Eigen
column-major order.

## Actual in-context values

Camera projection Jacobian, 2x4:

```text
43c863f9,c295504e,c295c15d,43e0b131,4379e614,433391d6,00000000,00000000
```

Homogeneous landmark Jacobian, 4x3 (`Jpp`), read at `rbp-0xb0` before the
product call:

```text
3fba8d4f,be563110,3f710856,00000000,
be563110,3fcc6805,3f2db603,00000000,
00000000,00000000,00000000,3f800000
```

Final raw landmark Jacobian, 2x3, read from the caller-provided output after
the call returned at `0x275774`:

```text
4450c412,c206f0e0,c2075714,4455c63b,00000000,00000000
```

The machine-readable capture retains the live `rbp`, `rsp`, `rdi`, `rsi`, and
`rdx` values and the pointer-equality checks:
[`target/m7fp_live_ordinal105_inputs.json`](../../target/m7fp_live_ordinal105_inputs.json).

## Comparison with M7fn and the impossible enumeration

The camera-J words are 8/8 identical to the M7fn isolated record.  The live
Jpp is **11/12** identical: only lane 0 differs, live `3fba8d4f` versus the
M7fn isolated `3fba8d50` (one ULP).  The live final raw Jp is 6/6 identical to
M7fn's clean-native expected Jp:

```text
M7fn isolated Jpp: 3fba8d50,be563110,3f710856,00000000,be563110,3fcc6805,3f2db603,00000000,00000000,00000000,00000000,3f800000
live in-context Jpp: 3fba8d4f,be563110,3f710856,00000000,be563110,3fcc6805,3f2db603,00000000,00000000,00000000,3f800000

M7fn clean raw Jp: 4450c412,c206f0e0,c2075714,4455c63b,00000000,00000000
live raw Jp:       4450c412,c206f0e0,c2075714,4455c63b,00000000,00000000
```

M7fo enumerated legal scalar/FMA reduction trees using the **isolated M7fn
J/Jpp words** and the expected raw Jp.  It reported no tree producing the
isolated second lane `c206f0e0`; the retained pair schedule produced
`c206f0e4`, and its best tested left-to-right chain produced `c206f0e3`.
That impossibility is not evidence against the live Eigen call: the
enumeration used a different operand set, because its Jpp lane 0 was
`3fba8d50` while the actual in-context stack operand is `3fba8d4f`.  The
enumeration therefore cannot be used to infer the packet/reduction schedule
of this call.  The live call, with the live stack operands, produced the
clean raw Jp words above.

This closes the requested provenance question: M7fn's standalone Jpp words
must not be substituted for the actual in-context Jpp when evaluating the
ordinal-105 product frontier.

## Artifacts and hashes

- GDB command: [`target/m7fp_live_ordinal105_inputs.cmd`](../../target/m7fp_live_ordinal105_inputs.cmd), SHA-256 `D83CF30832B4B6A42006DFD533C7A94B2EA2B607B1E1B3EF11CA4F5CB7AF6A8C`.
- Raw GDB output: [`target/m7fp_live_ordinal105_inputs.out`](../../target/m7fp_live_ordinal105_inputs.out), SHA-256 `374FA4386BDA6AA8705FCDDCDF64F0F524A35836026058D2350CF787C0D72C8A`.
- JSON capture: [`target/m7fp_live_ordinal105_inputs.json`](../../target/m7fp_live_ordinal105_inputs.json), SHA-256 `264223E8367CAE78F7293749DA49610E100C372B848C5F8775EFC718A6FD9767`.

The final process check found no live `gdb` or `basalt_vio` process.
