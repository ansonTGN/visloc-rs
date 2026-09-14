# M8 end-to-end mapper report — M7 MargData, 80 frames

Run date: 2026-08-22 (JST). Status: **PASS, three repetitions; non-deterministic outputs observed**. These were fresh offline, GT-free runs. No ground-truth path or evaluation file was passed to the mapper; evaluation, where cited below, is post-exit oracle comparison only.

## Inputs and execution

- MargData: `target/m7_postm7_fma_run80_20260822_114950/marg_data`
- Input packets: 5 (`frame_000051.json`, `frame_000058.json`, `frame_000065.json`, `frame_000072.json`, `frame_000079.json`)
- Unique image records: 24 (12 stereo frames, frame indices 0, 7, 14, ..., 77); each packet contains 16 image records.
- Pinned config: `configs/basalt/euroc_config.json`, SHA-256 `82937BD6493E592EF89572D31260C10F7437B4FB3FF1FDA179375713966E34FA`
- Pinned calibration: `target/euroc_ds_calib.json`, SHA-256 `AD8C5A18C48C55DACF61D18EBBC18CD7D4F3ACCCD5840BD5A5CD8646ADB6271C`
- Basalt compatibility provenance: upstream commit `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
- Release executable: `target/release/examples/m8_mapper_e2e_80.exe`, SHA-256 `55B27038029CEE5F1465292B163318A48F184AF7E5BCF6198F0B8F916A2FA55C`.
- Exact GT-free command (one invocation per output directory): `target/release/examples/m8_mapper_e2e_80.exe --marg-dir target/m7_postm7_fma_run80_20260822_114950/marg_data --calibration target/euroc_ds_calib.json --config configs/basalt/euroc_config.json --out-dir target/m8_e2e_80_mapper_<run>`.
- The repository has no dedicated mapper CLI for this path. The release example drove the existing `visloc-basalt::mapper` API through M8a, M8b, M8c, M8d, and M8e. No production mapper/VIO algorithm was edited.
- The command was executed as a direct argv (`shell=False`) with no GT-looking argument or environment input. Stdout/stderr were captured separately. External wall time used `time.perf_counter()` around the process; peak process-tree RSS used `scripts/benchmark_process_metrics.py::process_tree_rss` sampled every 0.5 s.

The three unique artifact directories are retained in `target/m8_e2e_80_mapper_20260822_140000/` (run 1, existing), `target/m8_e2e_80_mapper_20260822_133600_rep2/` (run 2), and `target/m8_e2e_80_mapper_20260822_133700_rep3/` (run 3). Each contains `mapper_report.json`, `matches.json`, `tracks.json`, `landmarks.json`, `final_poses.json`, `factors.json`, `solver_trace.json`, and stdout/stderr logs. All three exited `0` with empty stderr.

| Run | Mapper runtime (s) | External wall (s) | Peak process-tree RSS (bytes) |
|---|---:|---:|---:|
| run 1 (existing) | 15.2688405 | 21.6510000 | 188,231,680 |
| run 2 (`...133600_rep2`) | 13.3303534 | 19.2106034 | 187,920,384 |
| run 3 (`...133700_rep3`) | 13.5408144 | 19.2254341 | 187,473,920 |

## Counts

Values are `run 1 / run 2 / run 3`; the final column is median (max−min spread).

| Stage | Run 1 / run 2 / run 3 | Median (spread) |
|---|---:|---:|
| MargData packets | 5 / 5 / 5 | 5 (0) |
| Packet-local MargData observations | 8,408 / 8,408 / 8,408 | 8,408 (0) |
| M8a AOM columns → retained columns | 72→48 / 72→48 / 72→48 (all packets) | 72→48 (0) |
| Recovered relative-pose factors | 35 / 35 / 35 | 35 (0) |
| Recovered roll/pitch factors | 5 / 5 / 5 | 5 (0) |
| Recovered BA-covisibility factors | 0 / 0 / 0 | 0 (0) |
| Pose blocks supplied to setup/BA | 17 / 17 / 17 | 17 (0) |
| Feature rows / total corners (= descriptors = rays = hashes) | 24 / 17,558; 24 / 17,558; 24 / 17,558 | 24 / 17,558 (0) |
| Stereo edges / raw matches / essential inliers | 12 / 3,020 / 1,950; 12 / 3,020 / 1,950; 12 / 3,020 / 1,950 | 12 / 3,020 / 1,950 (0) |
| BoW temporal attempts / accepted | 264 / 264; 264 / 264; 264 / 264 | 264 / 264 (0) |
| Temporal raw / RANSAC / refined inliers | 38,137 / 36,405 / 36,888; 38,137 / 36,532 / 36,921; 38,137 / 36,452 / 36,831 | 38,137 / 36,452 / 36,888 (0 / 127 / 90) |
| Stored match pairs / stored match edges | 276 / 38,355; 276 / 38,482; 276 / 38,402 | 276 / 38,402 (0 / 127) |
| Track graph nodes / components before / after filtering | 11,076 / 1,927 / 649; 11,132 / 1,930 / 655; 11,099 / 1,933 / 642 | 11,099 / 1,930 / 649 (56 / 3 / 13) |
| Setup candidate attempts / attempted / accepted tracks | 776 / 649 / 639; 778 / 655 / 645; 760 / 642 / 634 | 776 / 649 / 639 (18 / 13 / 11) |
| Setup observations | 7,585 / 7,716 / 7,564 | 7,585 (152) |
| Setup rejections: baseline-too-small / inverse-distance ≤0 / too-large | 82 / 42 / 13; 79 / 42 / 12; 79 / 38 / 9 | 79 / 42 / 12 (3 / 4 / 4) |
| Final landmarks | 639 / 645 / 634 | 639 (11) |

## Solver

Global BA status was `completed` in every run, with gauge anchor `0`, 10 iterations, and no failures. Initial and final costs, canonical state hashes, and canonical trace hashes were:

| Run | Initial cost | Final cost | Final lambda | Final state hash | Solver trace hash |
|---|---:|---:|---:|---:|---:|
| run 1 | 3,207,078.8881885265 | 1,306,749.0086239078 | 1000.0 | `12264184824382555306` | `14853296853169102138` |
| run 2 | 3,386,074.7852365230 | 1,362,936.2749768884 | 1000.0 | `1841404840941739507` | `12932970621239152685` |
| run 3 | 3,278,129.4013116676 | 1,304,287.8765481547 | 1000.0 | `11974877604938298411` | `9790860199204298682` |
| Median (spread) | 3,278,129.4013116676 (178,995.8970479965) | 1,306,749.0086239078 (58,648.3984287337) | 1000.0 (0.0) | — | — |

The accept/reject sequence was identical in all three runs: `A, A, A, R×10, R×10, A, A, A, A, R×10` for iterations 0–9. The identical lambda schedule was:

- Iterations 0–2: `A @ 1e-32`.
- Iteration 3: `R×10 @ [1e-32, 2e-32, 8e-32, 6.4e-31, 1.024e-29, 3.2768e-28, 2.097152e-26, 2.68435456e-24, 6.8719476736e-22, 3.5184372088832e-19]`.
- Iteration 4: `R×10 @ [3.602879701896397e-16, 7.378697629483821e-13, 3.022314549036573e-9, 2.4758800785707607e-5, 0.40564819207303343, 1000, 1000, 1000, 1000, 1000]`.
- Iterations 5–8: `A @ [1000, 333.3333333333333, 111.1111111111111, 37.03703703703703]`.
- Iteration 9: `R×10 @ [12.345679012345677, 24.691358024691354, 98.76543209876542, 790.1234567901233, 1000, 1000, 1000, 1000, 1000, 1000]`.

## Repetition determinism and output hashes

The release mapper is **not deterministic end-to-end** under these three identical invocations. Feature extraction and stereo matching are stable, as are the recovered factor file and LM accept/lambda schedule; temporal RANSAC inliers, track graph, setup output, landmarks, costs, state hashes, trace hashes, and their dependent output files vary. The aggregate output hash below is SHA-256 over sorted relative file names, file sizes, and per-file SHA-256 digests (including the two logs).

The setup-opt canonical hashes (run 1 / run 2 / run 3) were `15266849730434638932` / `17061353658406916701` / `788367290846595749`.

| Run | Aggregate output-tree SHA-256 | Mapper runtime (s) | Wall (s) | Peak RSS (bytes) |
|---|---|---:|---:|---:|
| run 1 | `09D9FBDCB71A950DEFBAE06667E7D64E593FB66018AC47DA60FC245BFB22D6BB` | 15.2688405 | 21.6510000 | 188,231,680 |
| run 2 | `035EACD803CA6A6347737E8E28AEB8F279CA8840FB16800CE886502B1BD2890C` | 13.3303534 | 19.2106034 | 187,920,384 |
| run 3 | `A191894307EE534CFA15234BF5F48850EA99D9776CA8C7DDD062D54CE6F1E08A` | 13.5408144 | 19.2254341 | 187,473,920 |
| Median (spread) | — | 13.5408144 (1.9384871) | 19.2254341 (2.4403966) | 187,920,384 (757,760) |

Per-file output SHA-256 values are:

| Artifact | Run 1 | Run 2 | Run 3 |
|---|---|---|---|
| `mapper_report.json` | `26C43D87ED082A7F7A86C37A25A051F206DAB03703658ADA08F2C91EA465F851` | `C7E5DD4FFFD50DE124718959BE911A994DBD67C0B7E81F5F9B0EFA94E138EE6A` | `EB976621332138C8E6CEB4CADD98E53B201C8AC61EC6361DD482F2A3D12AE090` |
| `matches.json` | `85ACB2BC5D7FC74149504E6A923D2C80F70A1D05B502C37BAF438A292BDBE5B5` | `57D861500F5A1C19F1C3878C98A5F8D0FF6917DBE81D4D8BBDCEA27806676227` | `7CD107C3C5E914FDC8862581E5F92B49B6574C333D5923EFEED31E5D452823C7` |
| `tracks.json` | `03263D86C11DC2F9A0DF9A3207B9631F766959EF8ED2A674DD18EB8CA29C20A0` | `04E88AF76C7D3B538FBB3C60EFE9F607972D091BB80CC07B6CB5F5F6E52F0C18` | `D7A9C3003AEF10C39D6533B367B1EDC5382F6D88D6B81BFEB6D4EC88E7BB5F68` |
| `landmarks.json` | `6718D888271A9FD6477D433CF0F1CC05DE6FBDC205210B17C3E028D13AD8CDEA` | `303A46C3608FF784E66581A3714D46E13D3A5320BF10222B68F6BFA9CE7148BD` | `AB06B786B9CFB7323A391006FDD396C62C104401BA50EBDCA0A4C3FE0B4E615D` |
| `final_poses.json` | `EB73EF84EC97DFB87C19200B97D070729D770B403C0F0385118DB966CF624F4A` | `7F0B9A1B1A638DBC437064FEE665A5EFD7361084BDA9F3F284EDBF73D5140F7D` | `29837CA9ECB789A231CA21E6E775CEC9C8BF340C6693EB47C7E7405E3BE3507F` |
| `factors.json` | `B2F9BAB966877AAB001CC0067A266A88EC4F74C6B699616C37AC9E951BEE4F6B` | `B2F9BAB966877AAB001CC0067A266A88EC4F74C6B699616C37AC9E951BEE4F6B` | `B2F9BAB966877AAB001CC0067A266A88EC4F74C6B699616C37AC9E951BEE4F6B` |
| `solver_trace.json` | `7B0A3D9F2527DF1D9A786784C9230EF98CF8D567A4304BE614872A8FE2EE280A4` | `5AC7E1DEF1C1C02B3C697BC24F08ED798B3149DB48EC475374E54B6A92F2CAFC` | `815E51510E5295799F3DF2E19E570FB34DB4F6B3B70A07862F5758D9FF0F29B5` |
| `mapper_stdout.log` | `111F286820D0161F942AA07EAE84EF46C34DA354043C2178980E8A05EB28E891` | `F04D630F1029CFE17A8395444635E7453334EB89B2D7E36E6A0EFEDDC5F07553` | `113A6EE1F9B6F7EFD13249CC507064A0D2088FA37E8077BCDC7C711EBF8436FD` |
| `mapper_stderr.log` | `E3B0C44298FC1C149AFBF4C8996FB92427AE41E4649B934CA495991B7852B855` | `E3B0C44298FC1C149AFBF4C8996FB92427AE41E4649B934CA495991B7852B855` | `E3B0C44298FC1C149AFBF4C8996FB92427AE41E4649B934CA495991B7852B855` |

## Nondeterminism diagnosis and seed provenance

A minimal diagnostic replay of the three retained `mapper_report.json` files found the earliest semantic divergence in temporal pair index 0, before track construction. All 264 `(left,right)` pair identities and all 264 raw mutual-match arrays are identical across the runs. The first pair is `(frame 7, camera 0) → (frame 0, camera 0)` and has 114 raw matches in every run, but its production RANSAC seeds and inlier counts are:

| Run | Pair-0 seed | Pair-0 RANSAC / refined inliers | Exact first 8 sample indices (raw-list positions) |
|---|---:|---:|---|
| run 1 | `2054395473` | 112 / 112 | `1, 19, 75, 35, 16, 50, 47, 51` |
| run 2 | `2152633361` | 109 / 111 | `7, 14, 64, 57, 101, 81, 89, 35` |
| run 3 | `2744889993` | 104 / 110 | `61, 109, 49, 1, 58, 52, 20, 2` |

The sample indices above are a no-write replay of the Rust implementation's first draw sequence: the pinned libstdc++-compatible MT19937, `uniform_int_distribution<int>(0, INT_MAX)` represented as `next_u32() >> 1`, and the OpenGV swap/modulo sample-of-8 operation. Thus the first divergence is the time-derived seed/sample selection, followed by RANSAC inliers and then the dependent stored matches, tracks, setup, landmarks, and solver state. The per-run 264-seed provenance digests (SHA-256 of the compact JSON seed array) are:

| Run | 264-seed sequence SHA-256 |
|---|---|
| run 1 | `11B17C095E502B81ACB74672BC58507DBF0919A4CF8DB9CE801D7D1CC3CA550B` |
| run 2 | `75227337717E49740BA8BFE23327C827CA8FB2266ECC4FBBD07A8CA7CDC89BB1` |
| run 3 | `0940EDECAA8D15DC623E7CBC098C972091751289D8593184E7C9EA29512A5543` |

The source trace is `pipelines/basalt/src/mapper/features.rs`: production `match_temporal_ransac` delegates to the explicit-seed helper with `opengv_time_seed()`, whose seed is the wrapping `u32(epoch_seconds) + u32(subsecond_nanoseconds)`. The same file implements the OpenGV MT19937 and sample semantics. Pair and track traversal in `pipelines/basalt/src/mapper/mod.rs` uses ordered `BTreeMap`/`BTreeSet`; the identical pair order and raw arrays in this replay provide no evidence of a Rust container or parallel-order defect.

This is faithful production behavior, not a Rust-only defect. In the pinned Basalt/OpenGV path, the default `CentralRelativePoseSacProblem` is constructed with its upstream randomized mode and uses `time(0)+clock()` inside the `NfrMapper::match_all` TBB loop; the existing M8c reports document those default inlier summaries as observational. The Rust port has no portable standard-library process CPU clock, so its production adapter uses the nanosecond component to preserve per-invocation time variability; seed numbers are consequently not expected to be cross-language bit-identical. The pinned diagnostic oracle instead constructs the upstream problem with the fixed-seed mode and explicitly replaces its RNG stream, which corresponds to Rust's `match_temporal_ransac_seeded` test/fixture API. No production source change or forced fixed seed is warranted.

The resulting protocol is three identical GT-free release invocations on the same packet/config/calibration/executable, retaining each output tree and its per-pair seed sequence. Runtime, RSS, counts, and costs are summarized by median and max−min spread; output/state/trace hashes and seed digests remain run-specific observations. Deterministic OpenGV parity continues to use the explicit seeded API only in focused tests/fixtures. The focused Rust test command passed (`2 passed, 1 ignored`); no temporary diagnostic files were retained.

## Oracle comparison

- M8a/M8b: every latest packet reduced 72→48 and produced 7 relative-pose, 1 roll/pitch, and 0 BA-covisibility factors, matching the authoritative first-packet M8a/M8b fixture shape and factor counts.
- M8c: all 20 overlapping `TimeCamId` feature rows matched `benchmarks/basalt/m8c_feature_oracle20.json` exactly for corner, descriptor, ray, and hash counts. The first 10 overlapping stereo pairs also matched raw/essential counts exactly: `245/157, 222/154, 138/101, 187/118, 305/189, 337/208, 270/172, 124/87, 243/153, 350/219`; all were stored. The latest run has two additional stereo frames beyond that 20-TimeCamId fixture.
- M8d/M8e: the authoritative fixtures use a different graph (20 images, 80 poses, 554 tracks, 549 landmarks, 5,687 observations for setup; 83 poses/549 landmarks/5,687 observations for BA). These runs produce 24 images, 17 pose blocks, 642–655 tracks, 634–645 landmarks, and 7,564–7,716 setup observations, so their counts/costs/hashes are not interchangeable. No parity claim is made beyond the timestamp-overlap checks above.

## Mapper-off VIO integrity

The mapper wrote only the three unique output directories above. The mapper-off source directory remained separate and its root artifacts were unchanged after all three runs; final SHA-256 values are:

| Artifact | SHA-256 |
|---|---|
| `evaluation.json` | `096056629BD4C396A16010A818B38F5D6AC9D40CB2C343C9D42D05BA2D7B38B7` |
| `summary.txt` | `B4F7DCDF3B250327714D717FEF3E4DE72057F5E58EC25C27E73321ADECEF4B1E` |
| `trace.jsonl` | `21AAF512A60CEB68804D745F4C95A651FD801F8C3C40CA5CC6AEDFAF4CB76859` |
| `trajectory.csv` | `AE683D790E635A7768246DD616FFCAA8C1981601AE06EFAD8D3C739206B2D327` |
| `trajectory.tum` | `134820620D7DA4D6BCEB2F109FA89D1EACF2BF55A679ED360904380754A8EB2D` |

All five input MargData packet timestamps remained at their pre-run 11:50:45–11:50:56 JST write times after all three runs. Their final hashes are:

| Packet | SHA-256 |
|---|---|
| `frame_000051.json` | `ADD48B3DF77B69C09D4946C77B423AE920C6072AF38D83004D4319EC53BBCDE5` |
| `frame_000058.json` | `AF71644CF66EE02AEAD41C5F79A6755C0F190D9FD90CE9FF2836728809894D57` |
| `frame_000065.json` | `C8CA7E3D98B3A472DFD17E6C54FF0F64BBA6A15C4C01AE8EB3EA261AD7633505` |
| `frame_000072.json` | `B0AACD200EA112F9B15D9276749E3D6E7A8CD88D574C47DABBCA85F1D326F05B` |
| `frame_000079.json` | `D621C69900888C23A57B1FE4D0969BAF320881E2F62D24189973E18BFB633271` |

`HANDOFF_codex.md`, `work/`, and M7 production sources were not touched.
