# EuRoC all11 dataset freeze report

Status: **frozen**  
Manifest: `benchmarks/basalt/euroc_dataset_manifest.json`  
Manifest SHA-256: `a803db9e01f7a39f587cb9f87fd8601c8800455135ae7a2dc67f9e108003dd26`  
Manifest size: 920,070 bytes

## Source and bindings

The source is the official ETH Zurich EuRoC MAV ASL release, DOI
[10.3929/ethz-b-000690084](https://doi.org/10.3929/ethz-b-000690084), handle
`20.500.11850/690084`. The manifest records SHA-256 snapshots of
`work/ethz_item.json` and `work/ethz_original_bs.json`.

- Dataset base: `E:\datasets\euroc_mav`
- Unified all11 runner tree: `E:\datasets\euroc_mav\all11`
- Physical Machine Hall roots: `E:\datasets\euroc_mav\machine_hall`
- Physical Vicon roots: `E:\datasets\euroc_mav\vicon_room\sequences`
- Verified Vicon sequence archives: `E:\datasets\euroc_mav\vicon_room\sequence_archives_verified`
- The five Machine Hall and six Vicon sequence ZIPs are the bound input archives.
- Official `vicon_room1.zip` is locally present with the expected MD5
  `5ce06b405827e453a82523d3ca9c2fd0` and is bound.
- Official `vicon_room2.zip` expects MD5
  `c6347f4e0476aaa9a43a919c163c49c5`; the local assembled outer has MD5
  `f332544ffdd89b742db02877ce566fd2`, so it is explicitly rejected and never
  bound as a sequence archive.
- The official Machine Hall outer is not present or bound; verified per-sequence
  archives remain the frozen inputs.

## All11 recomputed counts

The manifest recomputes every archive byte count/SHA-256, camera PNG/CSV count,
IMU and GT row count, sensor CSV/YAML hash, and exact cam0/cam1 timestamp-set
intersection. Full intersection timestamp data and hashes are in the manifest.

| Sequence | cam0/cam1 rows | IMU rows | GT rows | stereo intersection |
| --- | ---: | ---: | ---: | ---: |
| MH_01_easy | 3682 / 3682 | 36820 | 36382 | 3682 |
| MH_02_easy | 3040 / 3040 | 30400 | 29993 | 3040 |
| MH_03_medium | 2700 / 2700 | 27008 | 26302 | 2700 |
| MH_04_difficult | 2033 / 2032 | 20320 | 19753 | 2032 |
| MH_05_difficult | 2273 / 2273 | 22721 | 22212 | 2273 |
| V1_01_easy | 2912 / 2912 | 29120 | 28712 | 2912 |
| V1_02_medium | 1710 / 1711 | 17100 | 16702 | 1710 |
| V1_03_difficult | 2149 / 2149 | 21500 | 20932 | 2149 |
| V2_01_easy | 2280 / 2280 | 22800 | 22401 | 2280 |
| V2_02_medium | 2348 / 2348 | 23490 | 23091 | 2348 |
| V2_03_difficult | 1922 / 2336 | 23370 | 22970 | 1921 |

## Verification

The full CRC build used four bounded workers while preserving frozen sequence
order. Every archive CRC check passed (`unzip`, exit code 0), and every archive
SHA-256 was recomputed.

- Full build: `status=frozen`, 11 sequences, 355.4 seconds.
- Full validator: `status=valid`, `archive_crc_checked=true`,
  `ground_truth_firewall=pass`, 461.5 seconds.
- Focused fixture tests: 4 passed, including explicit diagnostic CRC skip and
  corrupted-ZIP rejection.

Reproduce from the repository root:

```powershell
python benchmarks/basalt/euroc_dataset_manifest.py build `
  --dataset-root E:\datasets\euroc_mav `
  --output benchmarks/basalt/euroc_dataset_manifest.json

python benchmarks/basalt/euroc_dataset_manifest.py validate `
  --manifest benchmarks/basalt/euroc_dataset_manifest.json `
  --dataset-root E:\datasets\euroc_mav

python -m unittest tests.test_euroc_dataset_manifest -v
```

## Ground-truth firewall

Ground truth is audit metadata only. The manifest records
`ground_truth_available_to_engine=false`, `ground_truth_path_passed_to_engine=false`,
`ground_truth_environment_keys_passed=false`, `staged_input_excludes_ground_truth=true`,
`manifest_written_after_engine_exit=true`, and
`evaluation_must_be_a_separate_process=true`.
