# M7IM15 operand/quaternion schedule oracle

Pinned commit: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.

This bounded oracle uses fresh detached native expression traces for ordinals
0, 1, 2, and 3. It does not use the withdrawn reconstructed native artifact.
The Rust comparison sidecar is `target/m7im15_rust_blockwise_frame4_iter0_20260825.jsonl`.

## Translation operand

The native Eigen lazy expression lowers to this binary32 schedule:

```text
d = fl(p1 - p0)
h = fl(0.5f * g)
u = fma(-v, dt, d)
h = fl(h * dt)
out = fma(-h, dt, u)
```

This is confirmed by the detached `ImuBlock<float>` disassembly (`vmulss`,
`vsubss`, `vfnmadd231ss`, `vmulss`, `vfnmadd132ss`) and exact on both
same-live gates: ordinal 0 is 3/3 and ordinal 1 is 3/3. The plain Rust
left-associated expression is not equivalent at ordinal 1 (native
`39e0cf9f,b8b53dd1,3c2ed520`; Rust `39e0cfa0,b8b53dd0,3c2ed520`).

Ordinal-2/3 traces are retained as non-gating evidence; no same-live native
state-input sidecar exists for those captures, so they are not claimed as
independent exactness gates.

## Quaternion to rotation matrix

Sophus delegates `SO3::matrix()` to Eigen
`QuaternionBase::toRotationMatrix()`. The deterministic Rust spelling that
matches the independently aligned ordinals 0, 1, and 2 is to retain the
Eigen temporary products (`tx`, `ty`, `tz`, `tw*`, `t**`) and explicitly fuse
the six signed off-diagonal terms:

```text
m01 = (-tz).mul_add(w, txy)
m02 = ty.mul_add(w, txz)
m10 = tz.mul_add(w, txy)
m12 = (-tx).mul_add(w, tyz)
m20 = (-ty).mul_add(w, txz)
m21 = tx.mul_add(w, tyz)
```

The diagonal `1 - (sum)` terms remain separate binary32 operations. The
explicit-FMA probe is 9/9 for ordinals 0, 1, and 2; ordinal 3 lacks a
same-live native quaternion-input sidecar, so its result is deliberately not
used as proof. This proves a sufficient schedule, not uniqueness of the
compiler's legal contraction choices.

Machine-readable evidence: [`m7im15_operand_quat_schedule_oracle_20260825.json`](../../target/m7im15_operand_quat_schedule_oracle_20260825.json).
