# M7ey velocity-formula frontier

Date: 2026-08-23 JST  
Status: **explicit none; read-only algebraic audit**

## Verdict

The pinned `predictState` source order is reproduced exactly by

```text
velocity_new = (velocity_old + gravity * dt) + R_old * delta_velocity
```

and, at the Rust f32 boundary, by

```text
gravity.mul_add(dt, velocity_old) + rotated_delta_velocity
```

That formula gives the clean frame-3 → 4 velocity
`3da7a8b8,bc8bed06,be67f1c6` and preserves the existing frame-1 golden
`3cabe7c0,bbc3c000,bda6dcd8` **when the clean/O3 delta-v is supplied**.

There is no implementable general predictState-only candidate for the current
Rust path. Current Rust receives the O2 preintegration delta-v
`3ede21be,bcb31cdc,be20c273`, while clean native receives
`3ede21c0,bcb31cdc,be20c273`. With the O2 delta, the exact source-order
formula is `3da7a8b8,bc8bed08,be67f1c8`, the known +2 ULP y/z mismatch. The
first difference is upstream in preintegration packet 6 velocity-x, as
already established by M7du; changing only the final update cannot generally
repair it without changing the earlier arithmetic contract.

No production source, test, build output, or commit was changed.

## Inputs and direct reconstruction

The authoritative clean M7eg/M7db inputs are:

| operand | x | y | z |
|---|---:|---:|---:|
| `state0.velocity` | `3d850448` | `bc998d12` | `be4a560c` |
| gravity | `00000000` | `00000000` | `c11cf5c3` |
| `dt` | `3d4cccef` | — | — |
| clean/O3 `delta_velocity` | `3ede21c0` | `bcb31cdc` | `be20c273` |
| current/O2 `delta_velocity` | `3ede21be` | `bcb31cdc` | `be20c273` |

The shared gravity product is
`gravity * dt = 00000000,00000000,befb22fc`; adding it to the old velocity
gives `3d850448,bc998d12,bf302701`.

## Sophus point-action f32 bits

For `q_old = bd5e760f,bf4e5e85,bbc2d95f,3f16d68a` (Eigen xyzw), the canonical
Eigen/Sophus cross/FMA schedule is `uv = q.vec().cross(dv); uv += uv;`
followed by `dv + q.w * uv + q.vec().cross(uv)`. The key intermediates are:

| intermediate | current O2 delta-v | clean O3 delta-v |
|---|---:|---:|
| first cross negative products | `390853f5,3c0bb2c3,beb3112b` | `390853f5,3c0bb2c3,beb3112c` |
| first cross FMA | `3e0175be,bc35f74f,3eb3acd1` | `3e0175be,bc35f74f,3eb3acd2` |
| doubled first cross | `3e8175be,bcb5f74f,3f33acd1` | `3e8175be,bcb5f74f,3f33acd2` |
| second cross negative products | `390a7fec,bd1c22b9,be50b917` | `390a7fec,bd1c22ba,be50b917` |
| second cross FMA | `bf10e00f,3d15fa27,3e51f558` | `bf10e010,3d15fa28,3e51f558` |
| `q.w * 2uv + dv` | `3f15349d,bd0f2a22,3e835a72` | `3f15349e,bd0f2a22,3e835a73` |
| `R_old * delta_velocity` | `3c8a91c0,3ada00a0,3eec551e` | `3c8a91c0,3ada00c0,3eec551f` |

Thus the one-ULP x difference in preintegrated delta-v propagates into the
small y lane by `0x20` and into z by one ULP before the final velocity add.

## Final velocity schedule matrix

The native O2/O3 probe variants and the f32 Rust reconstruction produce:

| final association | current/O2 delta-v | clean/O3 delta-v |
|---|---:|---:|
| `(old + g*dt) + R*dv` (source) | `3da7a8b8,bc8bed08,be67f1c8` | `3da7a8b8,bc8bed06,be67f1c6` |
| `old + (g*dt + R*dv)` | `3da7a8b8,bc8bed08,be67f1c8` | `3da7a8b8,bc8bed06,be67f1c6` |
| `fma(g,dt,old) + R*dv` | `3da7a8b8,bc8bed08,be67f1c8` | `3da7a8b8,bc8bed06,be67f1c6` |
| `fma(g,dt,old + R*dv)` | `3da7a8b8,bc8bed08,be67f1c7` | `3da7a8b8,bc8bed06,be67f1c5` |
| `fma(g,dt,old) + R*dv` (reverse operand spelling) | `3da7a8b8,bc8bed08,be67f1c8` | `3da7a8b8,bc8bed06,be67f1c6` |

The clean target y/z row is therefore the source-order row with clean delta-v;
no final association turns the current/O2 row into the clean row. In
particular, moving the rotated increment inside the gravity FMA changes z in
the wrong direction and leaves y unchanged.

## Frame-1 golden guard

The same source-order/reference formula uses frame-1 inputs
`q=bd582e43,bf4d686f,00000000,3f182ffd`, `dv=3ec46faa,bcf45e4c,be0fadf7`,
`dt=3d4cccaa`, and zero old velocity. Its point action is
`3cabe7c0,bbc3c000,3ed16b71`, and every source-preserving schedule above
returns the existing golden `3cabe7c0,bbc3c000,bda6dcd8`. The attempted
inside-FMA schedule also happens to preserve frame 1, but remains wrong on
frame-3 → 4 z (`be67f1c7`/`be67f1c5`) and cannot repair y.

## Decision

The clean formula is retained as a reference reconstruction, not as a new
candidate: the Rust path already has the source-order predictState helper.
The remaining discrepancy is the M7du preintegration boundary (`dv.x`
`3ede21be` versus clean `3ede21c0`, first differing at packet 6 velocity-x).
Promoting a final-update spelling would either leave the mismatch unchanged or
change a lane in the wrong direction, and a one-lane preintegration FMA
already failed the pinned frame-1 golden. Therefore the implementable general
candidate set for this frontier is **none**.

Machine-readable details are in
[`target/m7ey_velocity_formula_frontier.json`](../../target/m7ey_velocity_formula_frontier.json).
