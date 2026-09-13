# M7bl Basalt hygiene cleanup report

Date: 2026-08-23 JST

## Scope

This cleanup addresses H3 and H5 from `m7bk_hygiene_audit.md`. It does not
change the triangulation, landmark, AOM, camera, patch, pyramid, provenance,
or handoff sources.

## H3: estimator trace removal

The temporary `VISLOC_BASALT_TRIANG_TRACE` branch and its `M7_TRI` and
`M7_TRI_BITS` `eprintln!` payloads were removed from
`pipelines/basalt/src/vio/estimator.rs`. The branch was after the existing
triangulation result and before the existing acceptance checks; no estimator
control flow or triangulation arithmetic was changed. A source scan confirms
that the production estimator no longer contains the temporary trace labels
or environment variable.

## H5: explicit ignored-test inputs

The ignored external probes now require every external input path at the test
boundary and have no machine-specific or `target` fallbacks:

| test | required variables |
| --- | --- |
| `m8c_feature_parity.rs` | `VISLOC_BASALT_MH01_ROOT`, `VISLOC_BASALT_CALIBRATION`, `VISLOC_BASALT_CONFIG` |
| `m8c_exact_bits_probe.rs` | `M8C_FEATURE_RAW_JSON`, `M8C_FEATURE_ORACLE_JSON` |
| `m8d_setup_opt_oracle.rs` | `VISLOC_BASALT_M8D_SETUP_ORACLE`, `VISLOC_BASALT_CALIBRATION`, `VISLOC_BASALT_CONFIG`, `VISLOC_BASALT_M8D_SETUP_INPUT` |

Missing variables fail before any external file is opened with an error that
names the required variable and its purpose. The pinned JSON included in the
repository remains an intentional compile-time fixture in `m8c_feature_parity`.

## Report corrections

`m7at_f32_window_exact_report.md` now uses the estimator's actual
`M7_TRI_BITS` label. `m7ay_camera_unproject_fma_report.md` now records that
the temporary estimator trace was removed from `estimator.rs` and that no
`M7_TRI`/`M7_TRI_BITS` production dump remains.

## Verification

The release integration-test compile passed:

```text
cargo test --release -p visloc-basalt --tests --no-run
Finished `release` profile
```

The release library test run built successfully and reached 161 tests, with
158 passing, 2 ignored, and the pre-existing
`vio::aom::tests::m7bf_source_pose_chain_matches_authoritative_lanes` failure
in `aom.rs:2581`. That AOM test is outside this cleanup scope; no AOM source
was changed. The run also showed the pre-existing dead-code warnings for the
M7bf source-pose probe helpers.

