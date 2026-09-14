# M7 row-product v11 replay regression (intermediate failure)

Date: 2026-08-24 JST  
Scope: pinned Basalt `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`, MH_01_easy,
plain production replay with `--no-marg-data`.  This is an intermediate
failure record, not a parity claim.  The replay was performed with the Luna
Max production tree; this report does not change production Rust source.

## Mechanical v9 versus v11 comparison

Both artifacts were compared with the same generic comparator,
`benchmarks/basalt/m7he_lm_branch_compare.py`, against the pinned one-thread
native logger `target/m7he_native_lm_debug80_1t.txt`:

```text
python benchmarks/basalt/m7he_lm_branch_compare.py --rust-trace target/m7he_normal_v9_80_plain/trace.jsonl --native-stdout target/m7he_native_lm_debug80_1t.txt --out target/m7he_lm_branch_compare_normal_v9_80.json
python benchmarks/basalt/m7he_lm_branch_compare.py --rust-trace target/m7he_rowprod_v11_80/trace.jsonl --native-stdout target/m7he_native_lm_debug80_1t.txt --out target/m7he_lm_branch_compare_rowprod_v11_80.json
```

| metric | v9 | v11 | delta (v11 - v9) |
|---|---:|---:|---:|
| common LM runs | 76 | 76 | 0 |
| branch-equal frames | 11 | 9 | -2 |
| branch-divergent frames | 65 | 67 | +2 |
| displayed-lambda mismatches | 6 | 8 | +2 |
| first divergence | frame 8 / iter 4: native `R`, Rust `A` | frame 8 / iter 4: native `R`, Rust `A` | unchanged |

The equal-frame IDs changed from
`[4, 5, 6, 7, 10, 14, 19, 43, 71, 75, 76]` (v9) to
`[4, 5, 6, 7, 10, 14, 43, 70, 76]` (v11).  The common first boundary is
therefore unchanged, while the current row-product build is not a
regression-free replay of the old v9 branch sequence.

The trace files each contain 80 frame records.  Only 4 complete JSONL lines
are byte-identical between v9 and v11; the first differing line is line 5
(frame 4).  This is a trace-level observation, not an attribution of the
later LM differences to one particular factor.

## Runtime, RSS, and hashes

The v9 capture was made without a durable `/usr/bin/time` sidecar, so its
runtime and peak RSS are **not available** from the retained artifacts.  They
must not be reconstructed from directory mtimes.  v11 was run with
`/usr/bin/time -f 'elapsed=%e rss_kb=%M'`:

| run | elapsed | peak RSS | trace SHA-256 | comparator SHA-256 |
|---|---:|---:|---|---|
| v9 `m7he_normal_v9_80_plain` | unavailable | unavailable | `A66299518B71606BA5C72ED2141C5E5674F459E725A255821ED119CD4E56B383` | `9AF7F69B2D1D79B53A816F5ED95E8ADA2FDDB9E4988B0E4E2AC636F6FB3BF06D` |
| v11 `m7he_rowprod_v11_80` | 52.51 s | 90,464 KB | `F0548F98B288391AA69AC89AF2E71C0A53D8146F448C57DCA9846A1177BFE219` | `EA0CEAC45D91D622271789A291C60B45F1445B4486A31585D3BF8EEAA6AA29FB` |

The fresh v11 WSL ELF was
`target/release/examples/basalt_euroc_vio_demo`, SHA-256
`D25A745E61D1126687037330B47E556FB33052E6F6B460D27CDAEA1197F1DB83`,
size 2,500,456 bytes, mtime `2026-08-24T22:32:04.2806037+09:00`.  The v9
ELF was not retained after the subsequent build, so an ELF SHA for v9 is not
claimed.  The retained v9 trace/comparator hashes above are the authoritative
replay evidence.

The same v11 binary also passed the five-frame smoke comparison: 1/1 branch
equal, 0 lambda mismatches, 0.92 s elapsed, and 23,144 KB peak RSS.  The
80-frame result above is the required regression record.

## Audit boundary (no causal conclusion)

The result must currently be recorded as an intermediate failure.  The audit
has two concrete schedule-shape concerns; neither is asserted to be the sole
cause of the v9→v11 changes or of the frame-8 boundary:

1. The reduction-side implementation maintains separate IMU and bias phase
   accumulators (`imu_h`/`imu_b`) and dispatches the special product by a
   nine-row shape test.  An adjacent `Imu` + `Bias` path can append 9 + 6 rows,
   but that conditional assembly is not the same representation as an
   upstream `ImuBlock` whose source object owns one 15-row stack
   `[preintegration 9 | gyro-bias 3 | accel-bias 3]` and one packetized
   product/reduction schedule.
2. Consequently, a 9-row shape classification can select a different Eigen
   product/reduction schedule from the native 15-row `ImuBlock`, even when
   the numerical row values are individually equal.  This is an audit
   hypothesis supported by the source shape/phase boundary and the replay
   deltas, not a root-cause determination.

No fixture-specific branch rule is introduced.  The first divergence remains
the generic frame-8 LM boundary and the v11 replay is not promoted to a
parity gate.

## Re-run recipe after the 15-row implementation

Once the native-shaped 15-row `ImuBlock` product is implemented, rebuild and
rerun under a new tag (for example `row15_v12`) with every detail and
diagnostic variable unset.  Keep the command conditions identical:

```text
wsl.exe -d Ubuntu-22.04 -- bash -lc 'cd /mnt/c/Users/rsasa/Workspace/visloc-rs && cargo build --release --example basalt_euroc_vio_demo && env -u VISLOC_BASALT_DETAIL_TRACE -u VISLOC_BASALT_DETAIL_FRAME -u VISLOC_BASALT_DETAIL_ITERATIONS -u VISLOC_BASALT_DIAGNOSTIC_IMU_HB -u VISLOC_BASALT_DIAGNOSTIC_FRAME -u VISLOC_BASALT_DIAGNOSTIC_ITERATION -u VISLOC_BASALT_DIAGNOSTIC_RAW_RESIDUAL -u VISLOC_BASALT_DIAGNOSTIC_FACTOR -u VISLOC_BASALT_DIAGNOSTIC_JACOBIAN -u VISLOC_BASALT_DIAGNOSTIC_COVARIANCE /usr/bin/time -f "elapsed=%e rss_kb=%M" ./target/release/examples/basalt_euroc_vio_demo --euroc-dir /mnt/e/datasets/euroc_mav/machine_hall/MH_01_easy --calibration target/euroc_ds_calib.json --config target/euroc_config.json --out-dir target/m7he_row15_v12_5 --max-frames 5 --no-marg-data'

wsl.exe -d Ubuntu-22.04 -- bash -lc 'cd /mnt/c/Users/rsasa/Workspace/visloc-rs && env -u VISLOC_BASALT_DETAIL_TRACE -u VISLOC_BASALT_DETAIL_FRAME -u VISLOC_BASALT_DETAIL_ITERATIONS -u VISLOC_BASALT_DIAGNOSTIC_IMU_HB -u VISLOC_BASALT_DIAGNOSTIC_FRAME -u VISLOC_BASALT_DIAGNOSTIC_ITERATION -u VISLOC_BASALT_DIAGNOSTIC_RAW_RESIDUAL -u VISLOC_BASALT_DIAGNOSTIC_FACTOR -u VISLOC_BASALT_DIAGNOSTIC_JACOBIAN -u VISLOC_BASALT_DIAGNOSTIC_COVARIANCE /usr/bin/time -f "elapsed=%e rss_kb=%M" ./target/release/examples/basalt_euroc_vio_demo --euroc-dir /mnt/e/datasets/euroc_mav/machine_hall/MH_01_easy --calibration target/euroc_ds_calib.json --config target/euroc_config.json --out-dir target/m7he_row15_v12_80 --max-frames 80 --no-marg-data'

python benchmarks/basalt/m7he_lm_branch_compare.py --rust-trace target/m7he_row15_v12_5/trace.jsonl --native-stdout target/m7he_native_lm_debug_1t.txt --out target/m7he_lm_branch_compare_row15_v12_5.json
python benchmarks/basalt/m7he_lm_branch_compare.py --rust-trace target/m7he_row15_v12_80/trace.jsonl --native-stdout target/m7he_native_lm_debug80_1t.txt --out target/m7he_lm_branch_compare_row15_v12_80.json
```

Record the fresh ELF/trace/comparator SHA-256, elapsed time, peak RSS, and
the complete comparator counts before updating any parity claim.
