# M7em Jp recovery

Date: 2026-08-23 JST

## Outcome

The interrupted `m7ej_identity_jp_exact_probe` and its temporary fixture were
removed from `pipelines/basalt/src/vio/aom.rs` and
`pipelines/basalt/tests/fixtures/`.  The probe's `println!` diagnostics and
unused target fields are not retained.

No production Jp arithmetic candidate was retained.  The accepted M7ec/M7dg/
M7dx production path already uses the fixed Eigen-style landmark reduction
(`eigen_landmark_jacobian_f32`).  The interrupted ordinal-0 identity probe was
run before cleanup; its current raw landmark Jp was

```text
4465f920,3f652458,3f65d1e8,4464cfbd,00000000,00000000
```

while the clean `target/m7ef_clean_visual_all.json` ordinal-0 target is

```text
4465f922,3f652460,3f65d1e0,4464cfbf,00000000,00000000
```

Thus the interrupted state did not establish an exact 6/6 raw or weighted
candidate.  Production was left at the accepted baseline rather than keeping
a target-specific arithmetic tweak.

## Hygiene

`aom.rs` has no `M7EJ`, `probe`, `println!`, `eprintln!`, or `dbg!` residue.
No ignored test, debug output, commit, or push was added.

## Verification

The existing M7ec/M7dg/M7dx exact fixtures/tests remain in place.  The focused
release-library filter `vio::aom::tests::m7` ran 12 tests: 11 passed and 1
failed (`m7ec_clean_track2_stereo_factor_is_bitwise_exact`).  Full release-
library verification ran 169 tests: 167 passed, 1 failed (the same M7ec test),
and 1 was ignored.
