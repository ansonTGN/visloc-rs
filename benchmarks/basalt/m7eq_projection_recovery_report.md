# M7eq projection recovery

Date: 2026-08-23

## Scope

`project_double_sphere_with_jacobian_f32` in `pipelines/basalt/src/vio/aom.rs` was restored toward the pinned Basalt/native evaluation order after the interrupted M7ej experiment. The recovery kept bearing and relative-pose code unchanged and did not retain an unproven standalone Jp candidate.

The restored projection path uses the pinned scalar order for:

- `r2 = x*x + y*y`;
- `d1 = sqrt(z.mul_add(z, r2))`;
- `k = xi.mul_add(d1, z)`;
- `d2 = sqrt(k.mul_add(k, r2))`;
- `norm = alpha.mul_add(d2, (1-alpha)*k)`;
- final pixel projection with `fx.mul_add(mx, cx)` / `fy.mul_add(my, cy)`.

The Jacobian norm derivative keeps the accepted native operation grouping `(xi*k)/d1 + 1`; diagonal terms use the accepted multiply-add form.

## Verification

Last run (no further rebuild after this run):

```text
RUSTFLAGS='-C target-cpu=native' cargo test -p visloc-basalt --release m7 -- --nocapture
```

Result: **16 passed, 4 failed**; 149 filtered out. The remaining failures are one-bit/two-bit Jacobian lane differences:

- `m7_double_sphere_boundary_matches_pinned_lanes`: lane 3 `43f04ca8` vs `43f04ca7`;
- `m7_landmark_jacobian_intermediates_match_pinned_lanes`: lane 1 `c1e49390` vs `c1e49388`;
- `m7_same_timestamp_stereo_homogeneous_jacobian_matches_pinned_lanes`: lane 4 `c2978b07` vs `c2978b08`;
- `m7ec_clean_track2_stereo_factor_is_bitwise_exact`: lane 2 `c1ccfe68` vs `c1ccfe70`.

Passing checks include the M7dg anchored projection/raw fixture and M7dx clean track-1 relative-pose Jacobians. The accepted prior binary also passed all 11 legacy M7 checks, including the projection/raw checks. A full test run was not completed in this recovery window; only the focused native M7 run above is recorded.

No commit or push was performed.

No `unsafe`, debug macro, or temporary debug print was added to the production source.
