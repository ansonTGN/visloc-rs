# M7dm visual projection/raw frontier taxonomy

Date: 2026-08-23 JST  
Status: **Complete read-only taxonomy; no production source change**

## Outcome

The frame-4, iteration-0 detail has 61 visual factors and 584 observations. Current M7dg/M7dj versus same-topology m7cm, after binary32 casting, gives **307/584 exact projection/raw pairs** and **277/584 non-exact pairs**. Projection and raw residual have the same pair status and lane mask for every observation.

| pair class | observations | projection lanes | raw lanes |
|---|---:|---:|---:|
| both lanes exact (`11`) | 307 | 614 | 614 |
| one lane exact (`10`/`01`) | 200 | 200 | 200 |
| both lanes differ (`00`) | 77 | 0 | 0 |
| total | 584 | 814/1168 exact | 814/1168 exact |

The machine-readable 584-row record is [target/m7dm_visual_frontier.json](../../target/m7dm_visual_frontier.json).

## Relation taxonomy

The source distinguishes exact `TimeCamId` equality from timestamp equality. Same timestamp with a different camera is retained as stereo, not identity.

| relation | total | exact pair | one lane | both differ |
|---|---:|---:|---:|---:|
| `cross_time` | 462 | 202 | 185 | 75 |
| `same_timecam` | 61 | 61 | 0 | 0 |
| `same_timestamp_stereo` | 61 | 44 | 15 | 2 |

- `same_timecam`: 61/61 exact.
- `same_timestamp_stereo`: 44/61 exact; 15 one-lane and 2 both-lane non-exact.
- `cross_time`: 202/462 exact; 185 one-lane and 75 both-lane non-exact.

## Host/target frame and camera

All 584 host records are frame 0/cam 0. Target breakdown:

| target frame | target cam | total | exact pair | one lane | both differ |
|---:|---:|---:|---:|---:|---:|
| 0 | 0 | 61 | 61 | 0 | 0 |
| 0 | 1 | 61 | 44 | 15 | 2 |
| 1 | 0 | 60 | 45 | 12 | 3 |
| 1 | 1 | 61 | 5 | 40 | 16 |
| 2 | 0 | 58 | 40 | 13 | 5 |
| 2 | 1 | 60 | 17 | 33 | 10 |
| 3 | 0 | 57 | 43 | 13 | 1 |
| 3 | 1 | 56 | 1 | 29 | 26 |
| 4 | 0 | 55 | 43 | 10 | 2 |
| 4 | 1 | 55 | 8 | 35 | 12 |

Frame-0/cam-0 is the identity branch; frame-0/cam-1 is same-time stereo; frames 1–4 are cross-time.

## Observation order

Observation order is serialized order within each landmark factor; later orders have fewer rows because observations are sparse.

| observation order | total | exact pair | one lane | both differ |
|---:|---:|---:|---:|---:|
| 0 | 61 | 61 | 0 | 0 |
| 1 | 61 | 44 | 15 | 2 |
| 2 | 61 | 45 | 12 | 4 |
| 3 | 61 | 5 | 40 | 16 |
| 4 | 61 | 40 | 15 | 6 |
| 5 | 60 | 17 | 33 | 10 |
| 6 | 58 | 44 | 13 | 1 |
| 7 | 55 | 2 | 28 | 25 |
| 8 | 54 | 41 | 10 | 3 |
| 9 | 52 | 8 | 34 | 10 |

Order 0 is 61/61 exact; order 3 is 5/61 exact and order 7 is 2/55 exact. This is replay correlation, not proof of a track-independent arithmetic rule.

## Landmark/track order

`landmark_order` is the 0-based factor order. `status_string`: E = pair exact, U = one lane exact, M = both lanes differ, in observation order.

| landmark order | track id | observations | E | U | M | status string |
|---:|---:|---:|---:|---:|---:|:---|
| 0 | 1 | 6 | 2 | 0 | 4 | `EEMMMM` |
| 1 | 2 | 10 | 4 | 3 | 3 | `EUEMUMEUEM` |
| 2 | 3 | 8 | 5 | 3 | 0 | `EEUUEUEE` |
| 3 | 4 | 10 | 6 | 3 | 1 | `EEEUEMEUEU` |
| 4 | 9 | 10 | 4 | 4 | 2 | `EUEUEUEMUM` |
| 5 | 10 | 10 | 3 | 2 | 5 | `EEUMUMMMEM` |
| 6 | 11 | 10 | 5 | 4 | 1 | `EEEUEUEMUU` |
| 7 | 12 | 10 | 4 | 4 | 2 | `EEEUMUUMEU` |
| 8 | 13 | 7 | 5 | 2 | 0 | `EEEUEEU` |
| 9 | 14 | 6 | 3 | 3 | 0 | `EEUUUE` |
| 10 | 18 | 10 | 4 | 4 | 2 | `EEMUEUEMUU` |
| 11 | 19 | 10 | 5 | 2 | 3 | `EEUMEUEMEM` |
| 12 | 20 | 10 | 4 | 3 | 3 | `EUUMEUEMEM` |
| 13 | 21 | 10 | 6 | 4 | 0 | `EEEUEUEUEU` |
| 14 | 22 | 10 | 3 | 6 | 1 | `EUEUMUEUUU` |
| 15 | 23 | 7 | 5 | 1 | 1 | `EEEMEUE` |
| 16 | 27 | 9 | 3 | 3 | 3 | `EEUUMMUME` |
| 17 | 28 | 10 | 4 | 3 | 3 | `EEEMUMEUUM` |
| 18 | 29 | 10 | 5 | 2 | 3 | `EEEMMUEMEU` |
| 19 | 30 | 10 | 6 | 1 | 3 | `EEMMEUEMEE` |
| 20 | 31 | 10 | 6 | 4 | 0 | `EEEUUEEUEU` |
| 21 | 37 | 10 | 4 | 4 | 2 | `EUUMEEUMEU` |
| 22 | 38 | 10 | 4 | 5 | 1 | `EUUUEUEMEU` |
| 23 | 40 | 10 | 5 | 4 | 1 | `EUEUEMEUEU` |
| 24 | 46 | 10 | 6 | 4 | 0 | `EUEUEEEUEU` |
| 25 | 47 | 7 | 3 | 4 | 0 | `EEEUUUU` |
| 26 | 48 | 10 | 4 | 4 | 2 | `EMEUUUEMEU` |
| 27 | 49 | 10 | 7 | 2 | 1 | `EEEUEEEMEU` |
| 28 | 54 | 10 | 6 | 3 | 1 | `EUEUEEUMEE` |
| 29 | 55 | 10 | 8 | 2 | 0 | `EEEEEEEUEU` |
| 30 | 58 | 10 | 8 | 2 | 0 | `EEEUEUEEEE` |
| 31 | 63 | 10 | 6 | 4 | 0 | `EEEUUEEUEU` |
| 32 | 64 | 10 | 8 | 1 | 1 | `EEEMEEEUEE` |
| 33 | 66 | 10 | 8 | 2 | 0 | `EEEEEEUUEE` |
| 34 | 72 | 10 | 6 | 3 | 1 | `EEMUEEEUEU` |
| 35 | 73 | 10 | 5 | 5 | 0 | `EUEUEUEUEU` |
| 36 | 74 | 10 | 4 | 5 | 1 | `EUEUUEEMUU` |
| 37 | 75 | 10 | 6 | 4 | 0 | `EEEUEUEUUE` |
| 38 | 82 | 10 | 5 | 4 | 1 | `EEEMUUEUEU` |
| 39 | 83 | 10 | 5 | 2 | 3 | `EEEUEMUMME` |
| 40 | 84 | 10 | 5 | 5 | 0 | `EEEUEUUUEU` |
| 41 | 87 | 10 | 8 | 0 | 2 | `EEEEMEEMEE` |
| 42 | 90 | 10 | 5 | 5 | 0 | `EEUEEUEUUU` |
| 43 | 91 | 10 | 6 | 4 | 0 | `EEEUEUEUEU` |
| 44 | 92 | 10 | 7 | 1 | 2 | `EEEEEMEMEU` |
| 45 | 93 | 10 | 6 | 3 | 1 | `EEEUEEUUEM` |
| 46 | 99 | 10 | 4 | 6 | 0 | `EEEUEUUUUU` |
| 47 | 100 | 10 | 6 | 4 | 0 | `EEEUEUEUEU` |
| 48 | 101 | 10 | 6 | 3 | 1 | `EEEUEEUMEU` |
| 49 | 102 | 10 | 6 | 4 | 0 | `EEUUEEEUEU` |
| 50 | 103 | 5 | 2 | 3 | 0 | `EUEUU` |
| 51 | 108 | 9 | 4 | 3 | 2 | `EEEUUUEMM` |
| 52 | 109 | 10 | 4 | 5 | 1 | `EUEUEUEMUU` |
| 53 | 110 | 10 | 3 | 6 | 1 | `EUUMUUEUEU` |
| 54 | 111 | 10 | 6 | 3 | 1 | `EEEUEUEMEU` |
| 55 | 117 | 10 | 4 | 4 | 2 | `EMEUUUEMEU` |
| 56 | 118 | 10 | 3 | 6 | 1 | `EEUUUUUUEM` |
| 57 | 119 | 10 | 6 | 2 | 2 | `EEEMEUEMEU` |
| 58 | 120 | 10 | 5 | 3 | 2 | `EUEUEMEUEM` |
| 59 | 128 | 10 | 6 | 3 | 1 | `EEEMEUEUEU` |
| 60 | 131 | 10 | 5 | 2 | 3 | `EEEMEUEUMM` |

| landmark-order bin | observations | E | U | M |
|---|---:|---:|---:|---:|
| 0–9 | 87 | 41 | 28 | 18 |
| 10–19 | 96 | 45 | 29 | 22 |
| 20–29 | 97 | 53 | 36 | 8 |
| 30–39 | 100 | 61 | 32 | 7 |
| 40–49 | 100 | 59 | 35 | 6 |
| 50–59 | 94 | 43 | 38 | 13 |
| 60–60 | 10 | 5 | 2 | 3 |

The 61 clean landmark records are exact against m7cu, so this order taxonomy is not explained by a changed landmark store. It remains a downstream factor arithmetic comparison against m7cm.

## Clean comparison boundary and limits

- `target/m7dd_clean_relpose.json` and `target/m7dh_clean_point_jac.json` are the clean-native track-1 fixture (host frame 0/cam 0 → target frame 1/cam 1, pixel bits `41da8172,42d4c7e1`). Current m7dj projection `41da6d17,42d70d32` and raw `bc22d800,3f915440` are exact 2/2 each there.
- `target/m7cu_clean_landmarks.json` captures the clean frame-0 landmark boundary: 61/61 direction/rho records match.
- `target/m7ct_clean_frame4_hb.json` is a clean aggregate H/b boundary; it cannot assign native projection/raw values to each observation.
- Therefore 307/584 is deliberately labeled **m7dj versus m7cm**, not an all-factor clean-native count.

## Single smallest next frontier

Capture one clean native factor in the first still-unclosed branch, same-timestamp stereo: **track 2, observation 1, host (frame 0, cam 0), target (frame 0, cam 1)**.

| filter | value |
|---|---|
| track | `track_id=2`, landmark order `1`, observation order `1` |
| host TimeCam | frame timestamp `1403636579763555584` (`0x137ab808528d0d00`), cam `0` |
| target TimeCam | same timestamp `1403636579763555584` (`0x137ab808528d0d00`), cam `1` |
| pixel f32 bits | `42517d4a,43046ac8` |
| current m7dj projection/raw | projection `4251812c,4304624e`; raw `3b788000,bd07a000` |
| m7cm baseline projection/raw | projection `4251812c,4304624f`; raw `3b788000,bd079000` |

At native `linearizePoint`, use pixel plus the four TimeCam key words as the ABI filter, then verify `track_id=2` at the caller/landmark boundary. This one capture tests the unclosed same-timestamp stereo camera-prefix/suffix path; do not extrapolate to the other 583 rows.

## Source branch inspected

Current `pipelines/basalt/src/vio/aom.rs` SHA-256 is `b1ef84893051bc76316e74cb20eb4069eed612df1a16b313c32cbcce6061fad3`. The M7dg path keeps an explicit `same_time_cam_id` identity branch (lines 837–845); the nonidentity path uses source-context camera quaternion products (lines 847–855); same-timestamp pose Jacobian suppression is separate (lines 889–901). No source, binary, build, commit, or push was changed by this analysis.

## Provenance

Current detail SHA-256: `c6b560d64e785b40294e3d5dff4136fb75835142755b567a98eb199fcc4ce7c3`; m7cm baseline SHA-256: `978c79b646232080d37697dc66fb25bfeb216578b0fe9f4c89403e93de1c1d1c`. Clean native commit: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`; clean basalt_vio SHA-256: `89e0324ccd04c3b615bf7e945aa32480a2f86524987aa00eaf152dcd6b1d242c`.
