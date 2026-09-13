# M7an f32 boundary / Eigen-init cleanup report

Date: 2026-08-21 (JST)  
Scope: Basalt VIO estimator/window only. Mapper and unrelated workspace changes
were not touched.

## Result

The active default compatibility path is the source-faithful float path, and
the catastrophic Eigen-init experiment is not active. The fresh release
replay processed all 5 requested frames and produced the normal `AAARAAAA`
LM decision sequence. No speculative Eigen decomposition or quaternion-order
change was retained in production by this cleanup.

The f32 path does not close the remaining LM-control-flow gap to upstream:
upstream accepts iteration 3 (`AAAAAAAA`), while the Rust f32 replay rejects
it (`AAARAAAA`). This is the same control-flow signature as the prior stable
Rust run, and is separate from the catastrophic initialization artifact.

## Production-path audit

The following source contracts are the active path:

| Contract | Evidence |
| --- | --- |
| Compatibility config selects f32 | `pipelines/basalt/src/config.rs:118` sets `ScalarMode::UpstreamF32`; `ScalarMode::default()` is also `UpstreamF32` in `pipelines/basalt/src/vio/scalar.rs`. |
| IMU queue owns f32 arithmetic | `pipelines/basalt/src/vio/estimator.rs:1687-1690,1922` dispatches to `integrate_queue_f32`; public deltas cross back to f64 only at `F32ImuState::into_delta`. |
| Gravity initialization owns f32 arithmetic | `estimator.rs:1778-1803` casts the selected packet to f32 and uses `from_two_vectors_eigen_f32`; the conversion at `1803` is `Quaternion::new(q.w, q.i, q.j, q.k)`. |
| Eigen two-vector helper is correctly ordered | `estimator.rs:1835-1859` constructs `Quaternion::new(w, x, y, z)`, matching nalgebra's constructor and Eigen's coefficient semantics. |
| Visual and LM compatibility path owns f32 arithmetic | `pipelines/basalt/src/vio/window.rs:1446-1507` selects f32 landmark back-substitution/factors; `pipelines/basalt/src/vio/aom.rs:691-907` selects f32 normal-system, damping, cost, and LM arithmetic. |
| `SymmetricEigen` is not an initialization/solve route | `window.rs:1841-1847` computes only covariance min/max diagnostics for `ImuLinkDiagnostics`; it does not feed whitening, initialization, H/b, or the LM solve. |

The f32 cross-product helper was checked against the f64 reference: its matrix
is `[0,-z,y; z,0,-x; -y,x,0]`. No source edit was needed for this audit.

## Required verification

Library test command:

```text
cargo test -p visloc-basalt --lib
```

Result: **113 passed, 0 failed**.

Fresh release replay (default compatibility configuration):

```text
cargo run --release --example basalt_euroc_vio_demo -- --euroc-dir E:\datasets\euroc_mav\machine_hall\MH_01_easy --calibration target\euroc_ds_calib.json --config target\euroc_config.json --out-dir target\m7an_f32_boundary_run5 --max-frames 5
```

Result: `frames_processed=5`, `observations_emitted=1114`,
`imu_samples_delivered=42`. The output summary is
`target/m7an_f32_boundary_run5/summary.txt`; the frame trace is
`target/m7an_f32_boundary_run5/trace.jsonl`.

Fresh frame-4 result:

```text
initial cost = 4216.1083984375
final cost   = 250.6915740966797
LM           = AAARAAAA (8 iterations, 7 accepted, 1 rejected)
```

The identical prior final-f32 replay is retained at
`target/m7al_rust_final_f32_run5`; its trace SHA-256 equals the fresh run's
trace SHA-256.

## Frame-4 oracle comparison

All costs below are the frame-4 initial LM cost. Deltas are against the
verified upstream rerun at `4215.9326171875`.

| Run | Initial cost | Delta vs upstream | LM decisions | Evidence |
| --- | ---: | ---: | --- | --- |
| Upstream M7al | `4215.9326171875` | `0` | `AAAAAAAA` | `target/m7al_upstream_f4_20260821T000200Z/iteration.jsonl` |
| Prior stable Rust | `4215.972884831595` | `+0.04026764409536554` | `AAARAAAA` | `target/m7al_rust_f4_20260821T000300Z.jsonl` |
| Fresh Rust default f32 | `4216.1083984375` | `+0.17578125` | `AAARAAAA` | `target/m7an_f32_boundary_run5/trace.jsonl` |
| Existing f32 Eigen/SO3 normal trial | `4216.09716796875` | `+0.16455078125` | `AAARAAAA` | `target/m7al_rust_current_f32_eigen_so3_run5/trace.jsonl` |
| Existing f32 Eigen-init-2 normal trial | `4216.0966796875` | `+0.1640625` | `AAARAAAA` | `target/m7al_rust_current_f32_eigeninit2_run5/trace.jsonl` |
| **Rejected catastrophic Eigen-init trial** | **`163357456`** | **`+163353240.0673828`** | `AARARARA` | `target/m7al_rust_current_f32_eigeninit_run5/trace.jsonl` |

The fresh f32 initial-cost delta is `+0.13551360590463446` relative to the
prior stable Rust run. This is a numerical consequence of the f32-owned
factor/prediction boundary, not evidence that the catastrophic initializer is
active. The normal f32 frame-0 quaternion is
`(qw,qx,qy,qz) = (0.5944822446754475, -0.05277849375633049,
-0.802374782357007, 0)`. In contrast, the rejected artifact reports
`(-0.05277849375633049, -0.802374782357007, 0, 0.5944822446754475)`, the
Eigen `(x,y,z,w)` coefficients incorrectly placed into the public `(w,x,y,z)`
fields; that artifact is not used by the current source.

## Artifact hashes

| Artifact | Bytes | SHA-256 |
| --- | ---: | --- |
| `target/m7an_f32_boundary_run5/summary.txt` | 415 | `B44A091BA95B85D693FEEFB749FD427F01B110B58E5F9C36F428BB5C80803D51` |
| `target/m7an_f32_boundary_run5/trace.jsonl` | 30387 | `C8BA3F11D4CA040F3353C422595D5CAAB6416AA1409D3D67652AC17F8E50FBEA` |
| `target/m7al_upstream_f4_20260821T000200Z/iteration.jsonl` | 32541464 | `18E87CFF493F758729F91E8270C9968E55BC8FB33CDCF1C3307FCE9119711F7A` |
| `target/m7al_rust_f4_20260821T000300Z.jsonl` | 73225345 | `9A9195D36B8FD1FF7E6DE48B74486D79516131AA03D688093DF3FF60D5B3E152` |
| `target/m7al_rust_current_f32_eigeninit_run5/trace.jsonl` | 30324 | `D376FEE16955C870DD0A3D002E8643FB66D35A9D0160660AC68B8C2671A135F5` |
| `target/m7al_rust_current_f32_eigeninit2_run5/trace.jsonl` | 30374 | `3948EB98EE5262E360ED04D83D66F59BEB45BD120D9DD72EA3648BEF5109C0F7` |
| `target/m7al_rust_current_f32_eigen_so3_run5/trace.jsonl` | 30378 | `16E88E45443615CEB293F9D85A6722B891CC4A7ABF2782A2F4AFF49A510149D7` |

Source hashes at audit time: `estimator.rs` 110807 bytes,
`A5D957899977641BD969110D55E4F90955B7AE88F0CA8CEA56B1A3B3C811AEF4`;
`window.rs` 117912 bytes,
`B7810F315F5EE7847F7AEFAE5C68D22FACB8406C77294C1A779A8411EE961D72`;
`aom.rs` 84834 bytes,
`FC844DE09958BFE797447D7AAA1546669E9287FC705608918FA4D4B2E132BF3D`.

## Conclusion

Keep the current source-faithful f32 compatibility boundary and the corrected
`w,x,y,z` Eigen-init conversion. Do not activate or copy the catastrophic
`eigeninit` experiment. The remaining `AAARAAAA` versus upstream `AAAAAAAA`
is an unresolved LM numerical parity difference; this cleanup does not claim
to fix it.
