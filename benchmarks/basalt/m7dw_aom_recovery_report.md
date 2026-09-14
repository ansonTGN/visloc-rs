# M7dw AOM recovery / M7dt cleanup

Date: 2026-08-23 JST

## Scope and result

Recovered `pipelines/basalt/src/vio/aom.rs` after the interrupted M7dt
working state. The temporary `m7dr_probe_current_track2_factor` test and its
diagnostic output were not retained. The only interrupted-test residue present
in the source was a duplicated `#[test]` attribute; one copy was removed.

The accepted M7dg implementation remains intact:

- same-`TimeCamId` identity handling is still explicit;
- the general f32 relative-pose chain still uses the source-context
  `Packet4f` camera-prefix and host-extrinsic-suffix quaternion products;
- same-timestamp pose-Jacobian suppression remains separate from the camera
  identity branch.

No M7dt production candidate was retained: the clean track-2 fixture does not
establish an exact production replacement for the M7dk/M7dg baseline. The
M7dk full-pose window sidecar is outside this AOM cleanup and was left
unchanged.

## Hygiene

The final `aom.rs` scan is clean for `M7DR`, `M7DT`, `probe`, `eprintln!`,
`println!`, and `dbg!` (case-insensitive). No commit or push was performed.

Final AOM SHA-256:

```text
9b1eab8e379d8536abd1f1b74af550e5d8f506b8a8c137c1cc46b0472514ace7
```

## Verification

Focused M7dg exact suite:

```text
cargo test --release -p visloc-basalt --lib "vio::aom::tests::m7_" -- --nocapture
10 passed, 0 failed
```

Full release library suite:

```text
cargo test --release -p visloc-basalt --lib
166 passed, 0 failed, 1 ignored
```
