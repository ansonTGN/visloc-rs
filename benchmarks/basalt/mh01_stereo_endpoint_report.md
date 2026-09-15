# MH_01 first-80 stereo endpoint golden

This fixture pins the upstream Basalt optical-flow endpoint observations for
frames 0--79 of `MH_01_easy`, using commit
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`, the sensor-only input, and the
pinned EuRoC calibration/configuration.  The original first-20 fixture remains
unchanged for backwards-compatible consumers; this is the M3 first-80 gate.

## Pinned fixtures

- Upstream endpoint JSONL:
  `benchmarks/basalt/mh01_stereo_endpoint_v1_first80.jsonl`
- Endpoint SHA-256:
  `cee7e2918a8e9383949508f74b6ec956f6c8a2a8e053648d15d05544d2205b2c`
- Endpoint SHA-256 file:
  `benchmarks/basalt/mh01_stereo_endpoint_v1_first80.jsonl.sha256`
- Schema: `basalt.stereo_endpoint.v1`
- Records: 80; each record contains the upstream timestamp plus `cam0`/`cam1`
  arrays of `{track_id,x,y}` endpoint observations.
- ID/coordinate swap allowlist:
  `benchmarks/basalt/mh01_stereo_endpoint_v1_first80_id_swaps.jsonl`
- Swap fixture SHA-256:
  `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`
- Swap records: 0 after the M3 allocator fix.  The allowlist remains checked
  in as an empty JSONL fixture so any future ID mismatch is an unknown hard
  failure rather than an implicit remap.

The endpoint trace was emitted by an observational hook in the upstream
`basalt_opt_flow` executable.  The hook does not alter tracking decisions.  The
instrumentation patch is recorded in the local oracle audit as
`target/upstream_opt_flow_endpoint.diff` (SHA-256
`79c8eb566d6ffad5d3a97ee920badbc26060c7c9f6d6b157e3d79779960e9611`) and was
applied to the fixed upstream checkout only.  The source endpoint artifact has
348 records; this gate pins its exact first 80 records.  The independent
upstream count trace remains
`target/basalt_upstream_mh01_400f_core_trace_20260821T000012Z/trace.jsonl`.

## Rust parity result

The ignored Rust gate replays the same 80 images and first asserts the exact
upstream `cam0`/`cam1` count sequence.  It then applies only the checked-in
frame/ID/coordinate map and compares every resulting endpoint pair.  Before
the M3 fix, direct ID-set differences occurred in 123 frame/camera records
(cam0: 3,244 upstream-only and 3,244 Rust-only IDs; cam1: 2,834 and 2,834).
After the fix, no unlisted ID or coordinate difference is accepted.

| camera | endpoint pairs | median (px) | p95 (px) | max (px) |
|---:|---:|---:|---:|---:|
| cam0 | 15,347 | 2.44259805199922500e-5 | 1.83105468750000000e-4 | 1.33660372914194043e-3 |
| cam1 | 8,909 | 4.31583728751554915e-5 | 2.46040580697587559e-4 | 9.36148782359495042e-4 |

The enforced limits are median `<=0.25 px` and p95 `<=1.0 px` per camera.
After the fix, all 80 frame/camera ID sets are exact: 15,347 cam0 and 8,909
cam1 common endpoint pairs, with zero upstream-only or Rust-only IDs.  The
first-20 subset is also exact (cam0 n=3,335, median `3.814697265625e-6`, p95
`9.34600830078125e-5`, max `2.5694754209974535e-4`; cam1 n=1,389, median
`1.7059844799040144e-5`, p95 `1.0235906807942409e-4`, max
`3.2332794342677446e-4`).

## M3 first ID allocator divergence

The checked-in first-divergence fixture is
`benchmarks/basalt/mh01_stereo_frontend_first_divergence_v1.json` (SHA-256
`43c6d1bf66e1476f06556bc57d54e8ff76e7d66d87b657d58eea48d84f0f4310`).  It
records the exact upstream FAST candidate vector, grid cell, response and
sort order, the frame-18 temporal/stereo lifecycle, and allocator `next_id`
values.  The upstream observation hooks were run from the fixed checkout at
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`; their captured artifacts are
`target/upstream_fast_trace.csv` (SHA-256
`50915f677a8bad1e90eb076b13846982d645b2b3116b9e873d54a32ac78cb189`) and
`target/upstream_flow_trace.txt` (SHA-256
`58b8f5fa7f19cfee8cfa569b3ed6152b9569b071b6732810d1257b77189a53e5`).

The first visible ID-set difference in the pre-fix run was frame 18 cam0:
upstream-only ID `723`, Rust-only ID `1050`.  The unique cause is earlier:
frame 13 cell origin `(701,315)` contains two response-15 corners `(707,356)`
and `(709,353)`.  Upstream sorts the complete `cv::FAST` vector and only then
applies the full-image `EDGE_THRESHOLD=19`, selecting `(707,356)`.  Rust
filtered seven out-of-edge candidates before response-only sorting, changing
the equal-response introsort permutation and selecting `(709,353)`.  That
three-pixel corner drift eventually makes Rust reject ID 723 at frame 18
(`fb²=1.528449177742 >= 0.04`) while upstream retains it (`fb²=0.00524139`).

The production fix is the minimal literal ordering correction in
`pipelines/basalt/src/fast.rs`: preserve all cell FAST candidates through the
response sort, then apply the existing full-image edge check while accepting
the quota.  No threshold, matching tolerance, ID allowlist, or nearest-point
logic changed.  With this fix frame 18 allocates 82 points (`next_id 968 ->
1050`) in both implementations; the 8,112 prior remaps disappear.

## Verification

With absolute paths to the official `E:\datasets\euroc_mav` MH_01 input,
pinned calibration/config, and the frozen upstream count trace:

```powershell
$env:VISLOC_BASALT_MH01_ROOT = 'E:\datasets\euroc_mav\machine_hall\MH_01_easy'
$env:VISLOC_BASALT_CALIBRATION = 'C:\Users\rsasa\Workspace\visloc-rs\target\euroc_ds_calib.json'
$env:VISLOC_BASALT_CONFIG = 'C:\Users\rsasa\Workspace\visloc-rs\target\euroc_config.json'
$env:VISLOC_BASALT_TRACE = 'C:\Users\rsasa\Workspace\visloc-rs\target\basalt_upstream_mh01_400f_core_trace_20260821T000012Z\trace.jsonl'
& 'C:\Users\rsasa\.cargo\bin\cargo.exe' test -p visloc-basalt --test mh01_stereo_golden mh01_first80_stereo_endpoints_match_upstream_trace -- --ignored --nocapture
```

Observed result after the M3 fix: `1 passed`; runtime `293.89 s` on the
external image archive.  The ordinary (non-ignored) test suite remains
unaffected.
