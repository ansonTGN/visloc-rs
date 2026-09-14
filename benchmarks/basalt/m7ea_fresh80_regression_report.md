# M7ea M7dx fresh-80 regression

Date: 2026-08-23 JST  
Scope: read-only release replay of the current M7dx worktree. No production
source, tests, commit, or push was changed.

## Inputs and artifacts

The current `stereo_diag` release binary was rebuilt after the M7dx AOM
change, then replayed on MH_01_easy frames 0--79:

```text
target/release/examples/stereo_diag.exe
  E:\datasets\euroc_mav\machine_hall\MH_01_easy
  target/euroc_ds_calib.json target/euroc_config.json 80
  target/m7ea_first80.jsonl
```

The direct binary wall time measured by PowerShell `Measure-Command` was
**10.035059 s**. The M7cm report and retained first-80 artifacts do not
record a comparable wall-time measurement, so no runtime ratio is asserted.

Machine-readable results are in
[`target/m7ea_comparison.json`](../../target/m7ea_comparison.json), and the
current endpoint is
[`target/m7ea_first80.jsonl`](../../target/m7ea_first80.jsonl).

| artifact | bytes | SHA-256 |
|---|---:|---|
| current M7ea endpoint | 1,415,296 | `7d5cbc8a1b5d94107b2508fa7c44228974f20951468c08a3da8a889d50290d81` |
| retained M7cm endpoint | 1,415,296 | `7d5cbc8a1b5d94107b2508fa7c44228974f20951468c08a3da8a889d50290d81` |
| frozen native endpoint | 1,418,014 | `cee7e2918a8e9383949508f74b6ec956f6c8a2a8e053648d15d05544d2205b2c` |

## First-80 endpoint comparison

Coordinates were decoded to binary32 and compared by exact bit pattern in
record order, camera order, stored point order, then `x`/`y`.

| metric | cam0 | cam1 | total |
|---|---:|---:|---:|
| endpoint points | 15,347 | 8,909 | 24,256 |
| coordinate fields | 30,694 | 17,818 | 48,512 |
| exact point pairs | 15,221 | 8,816 | **24,037** |
| exact coordinate fields | 30,527 | 17,692 | **48,219** |
| maximum ULP | 7 | 22 | **22** |
| maximum absolute delta | 0.00018310546875 px | 0.00048828125 px | **0.00048828125 px** |

All 80 records are present. Schema, timestamps, camera counts, track-ID
sets, and track-ID order mismatches are zero. The current endpoint exactly
reproduces the retained M7cm aggregate: 48,219/48,512 fields, 24,037/24,256
pairs, maximum 22 ULP, and the same endpoint SHA-256. Thus M7dx introduces
no stereo-endpoint regression relative to M7cm.

The first mismatch against the frozen endpoint artifact is frame 9, cam1,
track 481, `y`: fixture `0x428105e7` (64.51152801513672), current
`0x428105e8` (64.51153564453125), one ULP. The retained M7cm report prose
prints those two labels in the opposite direction; the machine-readable
fixture/artifact comparison above is the source of the stated direction.

## Fresh five-frame topology and state/H/b metadata

The current estimator replay used the M7dx release binary with detail
capture restricted to frame 4, iteration 0 onward. It processed 5 frames,
42 IMU samples, and 1,114 observations. The detail artifact is
[`target/m7ea_fresh5_detail.jsonl`](../../target/m7ea_fresh5_detail.jsonl)
(25 records: one header plus 24 frame-4 snapshots).

Against the retained M7dx detail
(`target/m7dx_fresh5_detail.jsonl`), all snapshot phase/iteration/decision
and row/topology metadata are exact:

| frame-4 iteration-start quantity | current | exact vs M7dx |
|---|---:|---:|
| factors / rows | 70 / 1,243 | yes |
| visual factors | 61 | yes |
| visual observations / rows | 584 / 1,168 | yes |
| state columns; H shape | 75; 75×75 | yes |
| accepted LM iterations | 8 (`AAAAAAAA`) | yes |
| `cost.before` | 4215.93115234375 | yes |
| compact state lanes | 75 | 75/75 binary32 lanes |
| full quaternion state lanes | 80 | 80/80 binary32 lanes |
| global H lanes | 5,625 | 5,625/5,625 binary32 lanes |
| global b lanes | 75 | 75/75 binary32 lanes |

The detail comparison also finds all 93,225 reduced-row lanes and 1,243
reduced-RHS lanes exact versus M7dx. This confirms the M7dx relative-pose
Jacobian sign-placement change preserves the established fresh5 topology
and state/H/b boundary.

## Integrity

No source edits, debug output, commit, or push were made for this regression
measurement. The only requested report/measurement artifact added is this
report; generated replay artifacts remain under `target/`.
