# M7fq bearing-J diagonal FMA audit

Date: 2026-08-24 JST  
Status: **Rejected and reverted.**

## Candidate

The candidate changed both active stereographic diagonal expressions in
`StereographicDirection::bearing_jacobian_f32` to explicit source-level
contractions:

```rust
(-x2).mul_add(norm_inv2, norm_inv)
(-y2).mul_add(norm_inv2, norm_inv)
```

The authoritative live ordinal-105 operands were replayed with the identity
same-TimeCam transform. The candidate produced the captured in-context
`Jpp` lane 0 `3fba8d4f` (the isolated M7fn value `3fba8d50` is not used) and
the exact live raw `Jp`:

```text
Jpp: 3fba8d4f,be563110,3f710856,00000000,be563110,3fcc6805,3f2db603,00000000,00000000,00000000,00000000,3f800000
Jp:  4450c412,c206f0e0,c2075714,4455c63b,00000000,00000000
```

The transient fixture/test also passed the existing ordinal-0, M7ec, and
other M7 boundaries, but was removed when the production candidate failed the
aggregate acceptance gate.

## Verification

Candidate focused M7 suite: **28 passed, 0 failed** (the 27 existing gates
plus the transient ordinal-105 live replay). Candidate full Basalt library:
**176 passed, 0 failed, 1 ignored** in both debug and release.

Fresh-five replay used the established MH_01 frame-4 iteration-start trace
and retained all 584 exact projection/raw-residual observations.

| quantity | M7fl baseline | diagonal-FMA candidate |
|---|---:|---:|
| projection lanes | 1,168 / 1,168 | **1,168 / 1,168** |
| raw residual lanes | 1,168 / 1,168 | **1,168 / 1,168** |
| weighted landmark Jp lanes | 3,423 / 3,504 | **3,476 / 3,504** |
| weighted landmark Jp six-lane rows | 536 / 584 | **571 / 584** |
| raw landmark Jp lanes | 3,189 / 3,504 | **3,240 / 3,504** |
| raw landmark Jp six-lane rows | 384 / 584 | **408 / 584** |
| aggregate H lanes | 3,301 / 5,625 | **3,299 / 5,625** |
| aggregate b lanes | 6 / 75 | 6 / 75 |

Although weighted/raw `Jp` improved, aggregate `H` regressed by two exact
lanes. The required simultaneous improvement over `3,423 / 3,301` therefore
failed, so the diagonal contractions and transient fixture/test were reverted.

Artifacts from the rejected diagnostic replay remain under `target/`:

- `target/m7fq_fresh5_detail.jsonl`
- `target/m7fq_hb_comparison.json`

No commit or push was performed.
