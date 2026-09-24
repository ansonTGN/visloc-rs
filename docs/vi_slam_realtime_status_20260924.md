# Online SLAM real-time status (2026-09-24)

Goal: make the online SLAM demo keep up with EuRoC in real time, with
processing wall time at or below the dataset duration. Beyond that point,
accuracy takes priority.

## Merged speed work

| PR | Change | Effect |
|---|---|---|
| #205 | Block-sparse global-BA pose Hessian | Mapper optimize time −56% with urgent keyframes (V2_03) |
| #209 | `filter_outliers` rebuilds the observation index once | MH_01: 101 s → 0.7 s |
| #210 | `+avx2,+fma` codegen by default (`.cargo/config.toml`) | VIO about 1.9× faster; `fmaf` calls had been about 31% of CPU |
| #211 | Q2 model-decrease payload wired into the lean reducer | VIO CPU −7 to −23%, byte-identical |
| #212 | Visual `JᵀJ` accumulated on the Jacobian support only | About −6%, bit-identical |

All VIO changes keep trajectories byte-identical. They were checked on
MH_01, MH_04 and V2_03.

## Current status

The runs used main with `--pipeline --threads 4`, kf5, periodic 100×4, and
loop rotation 30. The host was an idle i5-1145G7 (4 cores, 8 threads,
thermally throttled laptop).

| seq | data | VIO wall | total wall | total / data | ATE |
|---|---:|---:|---:|---:|---:|
| V2_03 | 116.7 s | 71.0 s | 92.9 s | **0.80** | 0.0763 |
| MH_04 | 101.5 s | 101.6 s | 111.2 s | **1.10** | 0.0832 |
| MH_01 | 184.0 s | 217.8 s | 272.8 s | 1.48 | 0.0169 |

`--pipeline` overlaps the frontend with the estimator. On V2_03 it cut the
total from 172 s to 125 s with a loaded host, and it helped less on the MH
sequences. It is recommended for online use.

For MH_04, the excess over 1.0 is the post-VIO final optimize, about 10 s.

For MH_01, VIO-only with `--pipeline` takes **150.9 s, or 0.82× the data
duration**. The online VIO slows to 218 s because the mapper, whose matching
costs about 210 s of work, competes for the same 4 physical cores. The mapper
then needs another 55 s after VIO ends: 24 s to drain its backlog and about
31 s for the final optimize. On this laptop, reaching real time on MH_01 means
cutting total VIO + mapper work by about 1.5×, not rescheduling it.

## Experiments that did not help (not merged)

* **Dedicated mapper rayon pool**, keeping mapper jobs out of the VIO's global
  pool. MH_01 VIO went from 218 s to 208 s, but the total rose from 273 s to
  308 s because the mapper backlog grew. The limit is physical cores, not pool
  sharing.
* **`--threads 8`** under load: VIO went from 135 s to 145 s on MH_04, which
  is no gain.
* **Per-factor `state_support` with a sparse model-decrease GEMV**. It was
  bit-identical, and `lm_model_decrease` fell 20%, but the end-to-end change
  on MH_04 600 frames was about 1% (six alternating runs, within noise). It
  was dropped to avoid complexity with no measurable gain.

## Host notes

On this machine, a stray 24-day `python3 -` process
(`gnssplusplus-library`) and a 6-day `colmap_incremental_mapper` had each
occupied a core. Real-time measurements need those stopped.

## Next

Speed is paused here: V2_03 runs in real time, MH_04 is nearly real time, and
MH_01 is limited by the hardware. Work moves on to accuracy.
