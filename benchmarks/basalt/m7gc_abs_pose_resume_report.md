# M7gc source-faithful absolute-pose Jacobian resume

Date: 2026-08-24 JST  
Status: **Implemented and verified; one upstream IMU-state boundary remains.**

## Result

The absolute-pose visual Jacobian now follows the pinned clean Eigen operation
order in the production Rust path.  The focused M7 suite and the complete
release `visloc-basalt` library suite pass, and a fresh five-frame MH-01 replay
was compared against the clean visual and relative-pose captures.

The fresh replay has exact keyed coverage for all 584 observations.  Its direct
clean relative-pose chain is exact in 13,982/14,016 binary32 lanes (1,138/1,168
blocks).  The corrected aggregate comparator is exact in 87,566/87,600 state
lanes, with 34 genuine cross-time mismatches.  Identity rows and the
same-timestamp stereo aggregate cancellation are both exact.  The remaining 34
individual lanes (30 blocks) are exclusively frame-4 `jp_anchor` cross-time
rows: 22 lanes/20 blocks in cam0 and 12 lanes/10 blocks in cam1.  No
`jp_target` lane remains mismatched.

## Source operation order

The pinned clean `2x6 * 6x6` assignment was disassembled from the clean
`libbasalt.so` and confirmed by the retained M7aq native boundary probe.  For
each output scalar its f32 tree is:

```text
high = (left[k4] * right[k4])
high = fma(left[k5], right[k5], high)
high = fma(left[k3], right[k3], high)
low  = (left[k1] * right[k1])
low  = fma(left[k2], right[k2], low)
low  = fma(left[k0], right[k0], low)
out  = high + low
```

The source-faithful path first multiplies every `d_res_d_xi` lane by the f32
`sqrt_weight`, then evaluates that tree separately for anchor and target
blocks.  This preserves the source's rounding boundary and signed-zero
behavior; it does not scale a completed product.

## Production and oracle changes

- `pipelines/basalt/src/vio/aom.rs` adds the explicit pair-tree helper and the
  pre-product whitening helper used by the visual factor.
- The focused `m7ga_weighted_pose_product_matches_native_packet_tree` test uses
  retained native post-product M7aq words for both `jp_anchor` and `jp_target`,
  including signed-zero lanes.
- Identity pose blocks remain exact zero, and the existing same-timestamp
  stereo cancellation tests remain exact.
- `benchmarks/basalt/m7fw_compare_clean_rel_jacs_all.py` mirrors the same f32
  scale-before-product and pair-tree order for the fresh replay comparison.

## Fresh5 aggregate gates

| boundary | exact | total | mismatch detail |
|---|---:|---:|---|
| clean H | 3,306 | 5,625 | 2,319 lanes; max absolute delta 704 |
| clean b | 6 | 75 | 69 lanes; max absolute delta 29.619140625 |
| compact 15-DoF state | 74 | 75 | frame 4 translation-z: clean `bcdc2b7f`, Rust `bcdc2b7e` (+1 ULP) |
| full aggregate state rows (corrected M7gg comparator) | 87,566 | 87,600 | 34 lanes, all cross-time |
| aggregate identity | 9,150 | 9,150 | exact |
| aggregate same-timestamp stereo | 9,150 | 9,150 | exact |
| aggregate cross-time | 69,266 | 69,300 | 34 lanes |

The earlier aggregate count came from a stale comparator that scaled a completed
2×6 product.  It is superseded: the source-faithful comparator scales each
`d_res_d_xi` lane first and then evaluates the M7fw Eigen pair-tree product.
The remaining compact-state translation-z ULP is the narrowest upstream input
boundary.  The 34-lane pose residual is localized to the corresponding frame-4
anchor chain, so the next investigation point is IMU position integration, not
the visual product or stereo policy.

## Verification

```text
cargo test --release -p visloc-basalt --lib
179 passed, 0 failed, 1 ignored

cargo build --release --example basalt_euroc_vio_demo
passed

cargo test --release -p visloc-basalt --lib 'vio::aom::tests::m7' -- --nocapture
23 passed, 0 failed
```

No commit or push was performed.  No unsafe code or debug residue was added.

## Artifacts

- Fresh replay detail: [`target/m7gb_fresh5_detail.jsonl`](../../target/m7gb_fresh5_detail.jsonl)
- Corrected aggregate remeasurement: [`target/m7gg_clean_native_remeasure.json`](../../target/m7gg_clean_native_remeasure.json)
- Corrected aggregate report: [`m7gg_clean_native_remeasure_report.md`](m7gg_clean_native_remeasure_report.md)
- Clean relative-pose comparison: [`target/m7gb_clean_rel_jacs_comparison.json`](../../target/m7gb_clean_rel_jacs_comparison.json)
- Comparison report: [`m7gb_clean_rel_jacs_report.md`](m7gb_clean_rel_jacs_report.md)
- Native post-product probe log: [`target/m7aq_visual_factor_boundary_probe_pose_O2_20260822.log`](../../target/m7aq_visual_factor_boundary_probe_pose_O2_20260822.log)
- Native probe source fixture: [`m7aq_visual_factor_boundary_probe.cpp`](m7aq_visual_factor_boundary_probe.cpp)
