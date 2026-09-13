# M7dt same-timestamp stereo factor audit

Date: 2026-08-23 JST  
Status: baseline retained; no source-faithful production candidate reached the clean target.

The authoritative fixture is `target/m7dr_track2_clean_factor.json`:

| stage | clean target (f32 bits) | retained M7dk baseline |
| --- | --- | --- |
| `T_t_h * [bearing; rho]` xyz | `bf26f362 be7d2383 3f3c2d9d` | `bf26f362 be7d2383 3f3c2d9b` |
| Double-Sphere projection | `42518143 43046250` | `4251812c 4304624e` |
| raw residual | `3b7e4000 bd078000` | `3b788000 bd07a000` |
| weighted `d_res_d_p` (six column-major lanes) | `44486188 c1c7d418 c1ccfe70 44561706 c21c3dda 40c8c0c7` | `44c8618a c247d420 c24cfe6c 4456170b c29c3dda 4148c0cf` |

The audit covered the same-timestamp stereo relative-pose composition, camera
extrinsic inverse/compose order, homogeneous point action, and the
Double-Sphere projection/Jacobian arithmetic. Direct-expression variants did
not improve the complete point/projection/raw/Jacobian tuple and were reverted;
the retained source is the M7dk/M7dg baseline. No track/frame/value-specific
branch or debug/probe output remains in `aom.rs`, and no commit or push was
performed.

The clean target fixture is retained as the acceptance oracle. A new passing
production exact test was not added because the baseline is intentionally not
claimed exact and a red/ignored probe would only preserve exploratory code.
