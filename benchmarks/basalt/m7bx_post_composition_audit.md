# m7bx post-composition read-only audit

Date: 2026-08-23 JST

## Scope and input validation

This is a read-only audit of the m7bt production artifact after the packet-8 and composition changes. No production source, benchmark source, commit, or push was changed.

The authoritative endpoint is aligned by line index because its frozen JSONL omits frame_index. All four endpoint inputs parsed as 80 valid JSONL records.

| artifact | bytes | SHA-256 |
|---|---:|---|
| native | 1418014 | cee7e2918a8e9383949508f74b6ec956f6c8a2a8e053648d15d05544d2205b2c |
| m7bt | 1415329 | 7e46e29bbf64fe699d0db638534c900207cd2586e96f9e9429ce2b3e0266edcd |
| m7bq packet8 | 1415012 | c03d158872ccb4114959ef510c9f4e21a1edf94e49cddffa3ed431702c8f1c3d |
| m7bi baseline | 1415018 | 1470decf41abc0493ec1e735102bd3914a54f1107353de7385f7ed2eb4dba036 |

## Endpoint structure

All comparisons have 80/80 records, 15,347 cam0 points, 8,909 cam1 points, and 24,256 point pairs (48,512 scalar fields). Schema, timestamp, camera-count, track-ID set, and track-ID order mismatches are zero for m7bt, m7bq, and m7bi versus native.

## Exact endpoint parity versus native

| artifact | cam0 pairs | cam0 fields | cam1 pairs | cam1 fields | total pairs | total fields | max ULP | max abs px |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| m7bt | 11287/15347 | 24780/30694 | 4929/8909 | 11837/17818 | 16216/24256 | 36617/48512 | 47 | 0.00048828125 |
| m7bq | 5459/15347 | 14948/30694 | 903/8909 | 4744/17818 | 6362/24256 | 19692/48512 | 203 | 0.00154876708984375 |
| m7bi | 5436/15347 | 14998/30694 | 914/8909 | 4875/17818 | 6350/24256 | 19873/48512 | 122 | 0.001220703125 |

m7bt therefore has 16,216/24,256 exact pairs and 36,617/48,512 exact fields. Its camera field fractions are cam0 80.73% and cam1 66.43%.

## ULP distribution and worst cases

The following bins count mismatched scalar fields only; positive/negative counts are candidate-bit minus native-bit sign counts. The ULP sum includes all fields for the mean reported in the JSON artifact.

| stream | 1 | 2 | 3–4 | 5–8 | 9–16 | 17–32 | 33–64 | 65+ | max ULP | ULP sum | +/− |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| m7bt cam0 x | 1447 | 512 | 349 | 113 | 39 | 0 | 0 | 0 | 15 | 4707 | 1144/1316 |
| m7bt cam0 y | 1447 | 749 | 657 | 456 | 129 | 14 | 2 | 0 | 39 | 9779 | 1753/1701 |
| m7bt cam1 x | 1262 | 609 | 408 | 161 | 38 | 0 | 0 | 0 | 16 | 5204 | 1254/1224 |
| m7bt cam1 y | 1279 | 678 | 707 | 555 | 230 | 44 | 10 | 0 | 47 | 12359 | 1766/1737 |

m7bt worst ULP: cam1 frame 53, track 100, y, 0.000179290771484375 px (-47 signed ULP); bits native 4258aed3, Rust 4258aea4.
m7bt worst absolute delta: cam1 frame 66, track 1467, x, 0.00048828125 px (-16 signed ULP).
For comparison, m7bq reaches 203 ULP and 0.00154876708984375 px; m7bi reaches 122 ULP and 0.001220703125 px.

## Gain/loss relative to packet8 and m7bi

| comparison | exact-pair delta | exact-field delta | field gain/loss | pair gain/loss | ULP-sum delta | max-ULP delta |
|---|---:|---:|---:|---:|---:|---:|
| m7bt vs m7bq | 9854 | 16925 | 18558/1633 | 10212/358 | -65914 | -156 |
| m7bt vs m7bi | 9866 | 16744 | 18508/1764 | 10229/363 | -62680 | -75 |

Here gain means a field/pair that was non-exact against native in the baseline and exact in m7bt; loss is the reverse. m7bt changes 28,799 fields versus m7bq and 28,691 versus m7bi, while preserving endpoint structure. The composition fix is therefore a large numerical parity gain, not a structural lifecycle change.

## Runtime

| artifact | measured wall time | measurement provenance |
|---|---:|---|
| m7bt | 14.188 s | PowerShell Measure-Command direct release first80 replay |
| m7bq | 7.122 s | reported 80-frame replay |
| m7bi | 8.040 s | PowerShell Measure-Command direct release-binary pass |
| native fixture | not available | frozen artifact, not timed |

The m7bt measurement is 1.992× m7bq and 1.765× m7bi. These are separately recorded release runs rather than a same-process controlled benchmark, so they are runtime evidence, not a causal performance attribution.

## Requested frame-1 cam1 track-117 residual taxonomy

The requested residual is frame 1, cam1, track 117, y: native 4249983c, m7bt 42499838, packet8 42499835, m7bi 42499834. The x lane is native/m7bt/packet8/m7bi 44302fd9/44302fd9/44302fd9/44302fd9.

Track 117 is present in cam0 and cam1 at frame 0 and remains in cam1 at frame 1. Artifact evidence therefore classifies this as an existing-stereo first retained observation, not a new-stereo birth. The nominal path is ExistingStereoForwardSe2Ic → ExistingStereoBackwardSe2Ic → ExistingStereoFbSquared.

The endpoint artifact has no forward/backward/FB substep values, and the retained m7bt trace is track3-only; the direction of the first scalar cannot be classified from artifacts alone. A line/track-order scan finds frame 1 cam1 track 3 x as the first globally differing endpoint scalar; track117/y is the requested post-composition residual boundary, not a claim that no earlier endpoint scalar differs.

## Fresh5 detail cross-check

Requested current detail: target/m7bt_composition_fresh5_detail_20260823.jsonl (71798593 bytes, 7119fa451b9b9d5c26766a46e19bab6a6198257c4b5dd4c8f5e2689658cd32af). Authoritative detail: target/m7_postm7_fma_upstream_detail_20260822.jsonl (71785214 bytes, ef1dadd57dfce9160738d60f0d102001e245656dc3714efe3c51cf1a65b25a7a). Both contain 25 valid JSONL records (header plus 24 snapshots), 61 initial landmarks, and 584 initial observations with zero topology mismatch; all eight LM iterations are accepted (AAAAAAAA).

| initial metric | Rust current | authoritative | exact / total |
|---|---:|---:|---:|
| pixels, exact pairs | 217 | — | 217/584 |
| pixels, exact fields | 667 | — | 501 mismatched; 667/1168 exact |
| landmark direction lanes | 36 | — | 36/122 |
| landmark rho | 4 | — | 4/61 |
| complete landmark triples | 1 | — | 1/61 |
| global H entries | 4629 | — | 4629/5625 |
| global b entries | 6 | — | 6/75 |
| initial cost.before | 4215.8642578125 | 4215.86328125 | +0.0009765625, 2 ULP |

The prior m7bs 39/61 result is a different comparison: m7bs used target/m7_gemv_fresh5_detail_20260823.jsonl against target/m7al_upstream_f4_20260821T000200Z/iteration.jsonl. That baseline reported 216/584 pixel pairs, 660/1,168 fields, 94/122 direction lanes, 41/61 rho, 39/61 complete triples, 3,175/5,625 H, and 6/75 b. Relative to those reported counts, m7bt is +1 pixel pair, +7 pixel fields, −58 direction lanes, −37 rho, −38 triples, +1,454 H entries, and unchanged b exactness. Because the authoritative detail artifact/schema changed, these deltas are a provenance comparison, not an isolated causal effect claim.

## Track-1 standalone-fixture provenance caveat

Frame-0 endpoint cam1 track 1 is exact: native and m7bt both have x/y bits 41eb67a9,42d6b68d. Nevertheless, the standalone FMA/JacobiSVD fixture records Rust direction bits becaaef7,be383810, while the actual authoritative fresh5 detail records becaaef6,be38380d; m7bt fresh5 detail records the standalone values becaaef7,be383810.

Thus frame-0 endpoint exactness does not prove fresh5 landmark-direction provenance. The standalone fixture fixes its own camera/DLT/JacobiSVD operands, whereas the actual detail direction also depends on the full frame-4 pose/triangulation operand path. The corresponding rho bits are m7bt 3e160ef3 versus authoritative 3e160f08.

## Conclusion

Composition materially improves endpoint parity: 16,216 exact pairs, 36,617 exact fields, max 47 ULP, and max absolute delta 0.00048828125 px, with exact structure preserved. The requested track117/y residual is existing-stereo but direction-untraceable. Fresh5 confirms the expected endpoint/H improvement while landmark bit exactness is not monotonic; the old 39/61 figure is tied to a different authoritative detail provenance. No production source was modified.

