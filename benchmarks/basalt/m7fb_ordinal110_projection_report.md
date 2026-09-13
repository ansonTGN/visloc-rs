# M7fb ordinal-110 projection mismatch

Date: 2026-08-23 JST  
Status: complete read-only probe; temporary instrumentation removed.

## Result

The current M7ez first projection mismatch is ordinal **110**, track **21**,
observation 9, target frame 4 / camera 1. Native projects to
`430a717f,43515578`; current Rust projects to `430a717f,43515579`. The raw
residual is correspondingly `3fbde200,408a2f80` versus
`3fbde200,408a2fa0`.

The first divergence is earlier than Double-Sphere projection:

```text
T_t_h[2,1]       clean bc20b37c   current bc20b37b   (+1 ULP current->clean)
target point y   clean bdc8bcda   current bdc8bcd9   (+1 ULP current->clean)
```

All other captured `T_t_h` lanes and the x/z/rho target-point lanes are exact.

## Double-Sphere packet

Using the exact clean point and the pinned native assembly/source schedule gives:

| intermediate | clean | current |
|---|---|---|
| `zz` | `3f420cd1` | `3f420cd1` |
| `r2` | `3e87691e` | `3e87691e` |
| `d1` | `3f816e4c` | `3f816e4c` |
| `k` | `3f27be46` | `3f27be46` |
| `d2` | `3f553ce0` | `3f553ce0` |
| `norm` | `3f41fae1` | `3f41fae1` |
| `mx` | `bf2a8f97` | `bf2a8f97` |
| `my` | `be04758b` | `be04758a` |
| pixel `(u,v)` | `430a717f,43515578` | `430a717f,43515579` |

`yy` differs (`3c1d67a0` clean vs `3c1d679e` current), but the subsequent
`r2`/`d1` rounding is unchanged. Thus the Double-Sphere grouping is not the
first mismatch; `my` is the first camera-stage consequence of the earlier
point-y difference.

The clean schedule is the pinned `DoubleSphereCamera<float>::project`
sequence `xx,yy,zz,r2,d1_2,d1,k,d2,norm,mx,my,pixel`; the clean binary emits
scalar/FMA instructions at the listed offsets in the JSON artifact. Current
M7ez already uses the same effective grouping (`mul_add` for `k`, `d2`,
`norm`, and pixel), so no production camera arithmetic was changed.

## General recommendation

Align the general f32 relative-pose quaternion/matrix materialization boundary
with the pinned Eigen/Sophus register schedule for all visual factors. This is
the shared source boundary that produces `T_t_h[2,1]`; a track-specific branch
or Double-Sphere rewrite would only mask the downstream one-ULP symptom.

Machine-readable details: [`target/m7fb_ordinal110_projection.json`](../../target/m7fb_ordinal110_projection.json). Clean/current inputs are retained in [`target/m7ef_clean_visual_all.json`](../../target/m7ef_clean_visual_all.json) and [`target/m7ez_fresh5_detail.jsonl`](../../target/m7ez_fresh5_detail.jsonl).
