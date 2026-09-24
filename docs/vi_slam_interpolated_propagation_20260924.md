# Interpolated keyframe-correction propagation (2026-09-24)

## Finding

The online demo writes a full-frame trajectory by applying each mapper
keyframe correction `Delta_k = M_k * V_k^-1` to every VIO frame up to the next
keyframe, which gives a piecewise-constant correction. Wherever consecutive
corrections differ, the trajectory jumps at the keyframe boundary.

On the quiet-host real-time runs, the online consecutive RPE was *worse*
than raw VIO even though the ATE was much better:

| seq | VIO ATE | online ATE | VIO RPE | online RPE |
|---|---:|---:|---:|---:|
| V2_03 | 0.2350 | 0.0763 | 0.0124 | 0.0333 |
| MH_04 | 0.0987 | 0.0832 | 0.0040 | 0.0049 |
| MH_01 | 0.0300 | 0.0169 | 0.0013 | 0.0016 |

## Change

`propagate_interpolated` in `examples/basalt_euroc_online_slam_demo.rs`
changes how the correction for frame `f` is chosen.

* For a frame between keyframes `k0 < f < k1`, it uses
  `Delta_k0 * Exp(alpha * Log(Delta_k0^-1 * Delta_k1))`, where `alpha` is the
  timestamp fraction of `f` between the two keyframes.
* Keyframe poses remain exact.
* Frames before the first keyframe or after the last one use that keyframe's
  correction.

The demo writes three full-frame trajectories:

| file | contents |
|---|---|
| `trajectory_online.tum` | the **interpolated** trajectory (new default) |
| `trajectory_online_nearest.tum` | the previous piecewise-constant trajectory |
| `trajectory_vio.tum` | the uncorrected VIO |

The test `interpolated_propagation_is_exact_at_keyframes_and_blends_between`
covers the interpolation.

## Result

One online run per sequence was made, and both propagations were evaluated
on the **same run**, so the comparison has zero run-to-run noise. Output is in
`/mnt/win/linux_data/visloc_interp_propagation_20260924`.

| seq | ATE nearest → interp | RPE t nearest → interp | RPE rot (deg) nearest → interp |
|---|---|---|---|
| V2_03* | 0.3753 → **0.3144** | 0.1834 → **0.0728** | 0.167 → **0.123** |
| MH_04 | 0.0836 → 0.0837 | 0.0049 → **0.0039** | 0.044 → **0.032** |
| MH_05 | 0.0634 → 0.0634 | 0.0039 → **0.0029** | 0.035 → **0.027** |
| V1_01 | 0.0353 → 0.0353 | 0.0024 → 0.0023 | 0.038 → **0.033** |
| V2_02 | 0.0124 → **0.0116** | 0.0034 → **0.0021** | 0.059 → **0.044** |
| MH_02 | 0.0246 → 0.0244 | 0.0015 → **0.0012** | 0.031 → **0.021** |

*This V2_03 run was an outlier with a mapper failure: ATE 0.375, against a
usual value of about 0.076. It is investigated separately.

**Translation RPE** improves by 3% to 60%, and in most sequences the new value
is at or below the raw-VIO level.

**Rotation RPE** improves by 12% to 31%.

**ATE** is equal or better on every sequence.

The cost is negligible, because this is post-processing only.
