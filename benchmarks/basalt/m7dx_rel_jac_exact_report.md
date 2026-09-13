# M7dx clean relative-pose Jacobian exactness

Date: 2026-08-23 JST  
Status: **Pass — clean track 1 `d_rel_d_h` and `d_rel_d_t` are 72/72 exact, including signed zero.**

## Change

`pipelines/basalt/src/vio/aom.rs` now keeps the upstream sign on the left
operand of the fixed-size 6x6 product.  The generic helper receives a unit
sign and computes

```text
d_rel_d_h = (+tmp.Adj())  * RR_h
d_rel_d_t = (-tmp2.Adj()) * RR_t
```

This matches the C++ expression ordering.  Negating the completed product
would produce the same numerical values but changes the sign bits of the
zero lanes emitted by the six-term Eigen reduction.  No frame, track, camera,
or observation-specific branch was added.

## Exact fixture

[`pipelines/basalt/tests/fixtures/m7dx_clean_track1_relpose_jacobian.json`](../../pipelines/basalt/tests/fixtures/m7dx_clean_track1_relpose_jacobian.json)
contains the clean pinned frame-4 / iteration-0 / track-1 operands and both
raw 6x6 matrices in Eigen column-major binary32 bit order.  The test
[`m7dx_clean_track1_relative_pose_jacobians_are_bitwise_exact`](../../pipelines/basalt/src/vio/aom.rs)
replays the f32 AOM chain and checks all 72 words.  The target matrix retains
the authoritative single `0x80000000` lane at column-major index 11; all
other zero lanes retain their positive-zero bits.

## Verification

Focused M7 suite:

```text
cargo test --release -p visloc-basalt --lib vio::aom::tests::m7 -- --nocapture
11 passed, 0 failed
```

Full library suite:

```text
cargo test --release -p visloc-basalt --lib
167 passed, 0 failed, 1 ignored
```

The existing M7dg relative q/t, homogeneous point, projection/raw, and
same-timestamp exact tests remain green.  The retained M7dk full-state pose
baseline was not changed.  A fresh five-frame replay was also run with the
same MH01 inputs and compared at frame 4 / iteration 0 / `iteration_start`
against `target/m7dk_fresh5_detail.jsonl`:

```text
frames processed: 5
visual factors / observations: 61 / 584
state columns: 75
global H rows: 75 (75x75 = 5,625 serialized lanes), exact vs M7dk: 5,625
global b rows: 75, exact vs M7dk: 75
cost.before: 4215.93115234375 in both runs
```

The fresh detail trace is `target/m7dx_fresh5_detail.jsonl`; its generated
run directory is `target/m7dx_fresh5_run`.

Replay command:

```text
cargo run --release --example basalt_euroc_vio_demo -- --euroc-dir E:/datasets/euroc_mav/machine_hall/MH_01_easy --calibration target/euroc_ds_calib.json --config target/euroc_config.json --out-dir target/m7dx_fresh5_run --max-frames 5
```

## Integrity

No unsafe code, production debug output, commit, or push was added.  The
fixture and test are general and source-faithful; they do not special-case
the clean factor at runtime.
