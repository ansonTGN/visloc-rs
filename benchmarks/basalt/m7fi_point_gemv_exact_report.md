# M7fi homogeneous point GEMV audit

Date: 2026-08-23 JST  
Status: **Held at green baseline; production GEMV not retained**

The requested ordinal-13 and ordinal-311 fixtures were added and the source-shaped
materialized `4x4 * 4x1` product reproduces both clean point4 records.  Ordinal 13
also reproduces projection/raw, homogeneous Jpp, raw Jp, and weighted Jp.  The
focused fixture tests are green:

```text
m7fi_ordinal13_homogeneous_point_and_jp_are_bitwise_exact: ok
m7fi_ordinal311_homogeneous_point_and_projection_are_bitwise_exact: ok
```

During integration, replacing the visual-factor point path exposed a conflict
with retained M7 oracle gates.  The explicit even/odd packet reduction fixed the
two new points but regressed five retained gates; the safe materialized nalgebra
fixed-size product regressed the retained ordinal-110, M7ec, and anchored-factor
gates.  The production path was therefore restored to the existing specialized
rotation-plus-translation implementation, per the green-baseline constraint.

The retained M7 suite is green at **27 passed, 0 failed** (149 filtered).  No
fresh-five replay or 584-row aggregate is claimed for this held candidate because
the production candidate was not retained.  No commit or push was performed.
