# M7 read-only performance profile: MH01

Date: 2026-08-22 (JST). This is a GT-free timing/RSS audit; it does not change
the VIO, mapper, or core implementation. The run is intended to identify the
next behavior-preserving optimization targets, not to establish a parity gate.

## Identity and protocol

- Dataset: `E:\datasets\euroc_mav\machine_hall\MH_01_easy` (`/mnt/e/...` in
  WSL), with `--max-frames 20` or `80`.
- Config SHA-256: `82937bd6493e592ef89572d31260c10f7437b4fb3ff1fda179375713966e34fa`.
- Calibration SHA-256: `ad8c5a18c48c55dacf61d18ebbc18cd7d4f3acccd5840bd5a5cd8646adb6271c`.
- Rust release executable used for the final 20f and 80f r1 run:
  `136289a083065a5dd34e524e8a2dcc1ce404d277e71047e62265477e2d3ff1d3`
  (1,857,536 bytes). This is the current workspace executable at the time of
  measurement.
- Upstream: pinned commit `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`,
  `basalt_vio` SHA-256
  `59cd32918dd170003df2e2e3ce6340e7726d70bd067a468861ce4c80048da6e6`.
- Rust command (PowerShell, direct executable; no Cargo startup/compile):

  ```text
  target/release/examples/basalt_euroc_vio_demo.exe --euroc-dir E:\datasets\euroc_mav\machine_hall\MH_01_easy --calibration target/euroc_ds_calib.json --config target/euroc_config.json --out-dir target/m7perf_final_rust_<n> --max-frames <20|80>
  ```

- Upstream command (WSL, no GT path and no `--marg-data` for the baseline):

  ```text
  /root/visloc-basalt-oracle-0f3b2b52/build/core-relwithdebinfo/basalt_vio --dataset-path /mnt/e/datasets/euroc_mav/machine_hall/MH_01_easy --cam-calib /root/visloc-basalt-oracle-0f3b2b52/data/euroc_ds_calib.json --dataset-type euroc --config-path /root/visloc-basalt-oracle-0f3b2b52/data/euroc_config.json --show-gui 0 --save-trajectory euroc --max-frames <20|80> --num-threads 4
  ```

Rust timing/RSS came from `scripts.benchmark_process_metrics.run_monitored`
(`poll_seconds=0.05`, effective 0.1 s), which samples the Windows process
tree working set. Upstream timing/RSS came from `/usr/bin/time -f
'ELAPSED_SECONDS=%e\\nMAX_RSS_KB=%M\\nEXIT_STATUS=%x'` in WSL. All final upstream
repetitions were started with no Windows `cargo`/`rustc` processes present.

## Clean timing and RSS

Rust always emits its trace and mapper diagnostic output in this executable;
the CLI has no output-disable switch. Therefore the Rust rows below are
**full-output** rows, while upstream rows are **no-marg-data** rows. This is
the closest available GT-free control, not a byte-for-byte I/O parity pair.

| engine / variant | frames | n | wall seconds (min / median / max) | peak process-tree RSS (min / median / max) |
|---|---:|---:|---:|---:|
| Rust full output, hash above | 20 | 3 | 2.7565 / **3.0239** / 3.1732 | 28,258,304 / **31,473,664** / 32,563,200 B |
| upstream, no `--marg-data` | 20 | 3 | 2.47 / **2.81** / 3.02 | 68,780,032 / **69,083,136** / 69,230,592 B |
| Rust full output, hash above | 80 | 1* | **24.3505** | **91,934,720 B** |
| upstream, no `--marg-data` | 80 | 3 | 7.67 / **8.36** / 9.08 | 68,567,040 / **69,066,752** / 70,348,800 B |

The directly observed 20f ratios (Rust / upstream median) are **1.0761x
wall** and **0.4556x RSS**. The 80f single clean Rust observation is
**2.9127x wall** and **1.3311x RSS** against the upstream 80f median; treat
this as provisional because two planned Rust repetitions were interrupted by
another workspace starting a Cargo/rustc build. The 20f ratio is the reliable
comparison in this audit. The 80f Rust run itself finished before the later
07:28:49 Cargo process appeared; the later build was not included in that
process tree.

The upstream `%M` value is only about 66--69 MiB, while Basalt's own
`resident_memory_peak` statistic in the same logs is about 0.52--0.55 GiB.
This WSL/process-accounting discrepancy is material: use the explicitly
recorded process-tree metric for the ratio, but do not interpret the upstream
RSS as allocator-wide memory usage.

## I/O and phase evidence

The final Rust 80f output tree is 90,976,291 bytes. The five mapper packets
alone are 89,665,787 bytes (17.47--18.40 MB each), and `trace.jsonl` is
1,287,966 bytes. Packet mtimes are:

```text
frame_000051.json  07:28:34.3359  17,472,219 B
frame_000058.json  07:28:36.9285  17,697,443 B
frame_000065.json  07:28:39.2089  17,907,179 B
frame_000072.json  07:28:42.1049  18,401,494 B
frame_000079.json  07:28:45.4345  18,187,452 B
trace.jsonl        07:28:45.1527   1,287,966 B
```

The first packet appears about 13 s after the clean 80f process start, and
the five packet writes span about 11.1 s; trajectory/trace finalization is at
the final timestamps. This
strongly identifies MargData JSON construction/write as a first output-side
target. The last packet mtime is a few hundred milliseconds after the final
trace mtime because the files are created by separate writes; this ordering is
not used to claim a trace bottleneck. The 20f Rust output has no mapper packets: total tree 217,415 bytes,
of which `trace.jsonl` is 210,820 bytes.

The Rust summary also shows that an 80f run loads 3,682 cam0 manifest rows,
3,682 cam1 timestamps, and 36,820 IMU samples although it delivers only 792
IMU samples to the first 80 frames. This is a plausible startup/I/O target,
but lazy indexing must preserve timestamp selection and error behavior.

Upstream's internal 80f timing table (three no-marg runs) is stable enough to
locate compute cost: median per-frame values are `measure` 25.71 ms,
`optimize` 26.04 ms, `iteration` 19.35 ms, `solve` 6.64 ms,
`get_dense_H_b` 2.78 ms, `linearizeProblem` 1.56 ms, `performQR` 0.66 ms,
and `marginalize` 0.51 ms. These are upstream's internal timings, not Rust
timings. `--marg-data` controls saving packets; it does not remove the
upstream marginalization computation. At 20f, optimizer timings vary widely
(startup/cache effects and a few outliers), so they are not a useful hotspot
ranking.

The Rust trace currently records stage/phase names and ordering but no
durations, so it cannot safely distinguish Rust frontend from backend CPU
time. A CPU profiler run with Windows WPR/xperf was intentionally discarded:
the ETL overlapped unrelated Cargo/rustc builds and had no usable Rust
function symbols. It is not evidence for a Rust function hotspot.

## Prioritized behavior-preserving targets

1. **MargData output path.** Stream the same JSON bytes directly through a
   buffered writer (for example, `serde_json::to_writer`) instead of first
   materializing the entire packet string. This should preserve the schema and
   byte-level fields while reducing transient allocation and write latency.
2. **Repeated raw image payloads.** Each of the five packets embeds 16 raw
   images as JSON/base64-sized data. Measure a content-addressed or binary
   sidecar representation only if the mapper/reader contract can be extended
   without changing semantics; do not silently remove image data.
3. **Separate diagnostics from the hot path.** Make trace/MargData emission
   explicitly opt-in only where the benchmark/output contract permits it.
   Trace final write is small here, but per-frame trace construction can then
   be measured independently.
4. **Dataset prefix loading.** Avoid reading all image manifests/IMU rows for
   a bounded smoke run, while preserving exact half-open IMU intervals and
   duplicate/timestamp validation. This is a startup/I/O optimization, not a
   frontend/backend algorithm change.
5. Add Rust phase-duration instrumentation, then profile frontend image
   decode/feature tracking versus estimator/backend separately. Do not infer a
   Rust solver hotspot from the upstream table.

## Limitations and contamination record

- Rust ran as a Windows executable; upstream ran under WSL. Filesystem cache,
  allocator, compiler, and thread/runtime behavior are not identical.
- The upstream baseline uses `--num-threads 4`; the Rust demo has no equivalent
  thread-count flag in this command.
- Rust has no literal no-trace/no-MargData run without changing the executable,
  so the clean ratio includes unequal output contracts.
- Earlier Rust repetitions and the WPR run were excluded where their binary
  hash was old or another workspace's `real_scan_3dgs_showcase` Cargo/rustc
  build was active. The 80f current-hash r1 above is retained only as a
  provisional single observation; no contaminated run is pooled into a
  median.
