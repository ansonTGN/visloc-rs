# Mapper queue memory: strip already-sent images (2026-09-24)

## Diagnosis

The 11-sequence urgent-keyframe screen (`vi_slam_urgent_keyframe_20260924.md`)
recorded a peak RSS of 718 MB for MH_01 at kf5 and 1.36 GB with urgent
keyframes.

The `mapper_heap_estimate` lines in those runs show two components:

1. **Persistent mapper state grows with sequence length.** For MH_01 kf5, the
   lower-bound payload is about 128 MB:

   | Structure | Size |
   |---|---|
   | feature_corners | 67.6 MB |
   | feature_match_data | 29.5 MB |
   | feature_matches | 14.2 MB |
   | lmdb | 11.0 MB |
   | tracks | 6.2 MB |

   Each background optimize clones the whole `NfrMapper`, which roughly
   doubles this at the peak.
2. **Mapper queue backlog.** With urgent keyframes on MH_01, `max_queue_depth`
   rose from 4 to **56** and `lag_max` from 3.0 s to **50.0 s**.

The two components contribute as follows:

* Every MargData packet carries the raw images of the **whole AOM window**,
  up to 16 images of about 720 KB each.
* The mapper reports `new_keys=2` per packet, so almost every queued image
  repeats one from an earlier packet.
* Queued packets keep their pixels until the mapper reads them. A 56-packet
  backlog therefore holds hundreds of MB of duplicate pixels.

## Change

`SentImageFilter` in `mapper/online.rs` is a producer-side filter. It drops
images whose `(frame_id, timestamp_ns, camera_id)` key was already forwarded.
`examples/basalt_euroc_online_slam_demo.rs` applies it before each
`sender.send`.

Stripping the duplicates is safe for these reasons:

* MargData validation accepts any subset of window images.
* The mapper skips detection for a key it has already processed, so a
  repeated copy only re-enters `img_data` and is removed again.
* An image that is still waiting for its pose stays in `img_data` until it
  becomes eligible. `img_data` is removed only after processing, at
  `online.rs:647`.

The test `mapper_online::tests::sent_image_filter_preserves_online_mapper_state`
ingests overlapping packets both with and without the filter. The resulting
`feature_corners`, `feature_matches`, `feature_match_data`, `frame_poses` and
retained image bytes are identical, and the second packet shrinks from 4
images to 2. All 434 `visloc-basalt` tests pass.

## Measurement: MH_01, one run each

The dedup runs are in `/mnt/win/linux_data/visloc_image_dedup_probe_20260924`.
The previous runs come from the 11-sequence screen. The runs are sequential
rather than interleaved, and host load differed, so wall times are not
comparable.

| | urgent, previous | urgent, dedup | kf5, previous | kf5, dedup |
|---|---|---|---|---|
| peak RSS (KiB) | 1328392 | **885164 (−33%)** | 718196 | 745672 (+4%) |
| max_queue_depth | 56 | 47 | 4 | 4 |
| lag_max | 50.0 s | 48.9 s | 3.0 s | 3.3 s |
| SE3 ATE | 0.01545 | 0.01515 | 0.01627 | 0.01703 |

* **Urgent keyframes:** the backlog still forms, because the mapper is slower
  than VIO with more keyframes. It now holds about one copy of each image
  instead of about 8, which cuts peak RSS by 33%.
* **kf5:** the queue barely backs up, so dedup has no measurable effect. The
  +4% is within run-to-run variation.
* **Accuracy:** ATE differences are within asynchronous run-to-run variation.
  The mapper inputs are identical by construction and by test.

The remaining kf5 peak for MH_01, about 730 MB, comes from the persistent
mapper state (about 128 MB lower bound) plus the full-`NfrMapper` clone taken
for each background optimize. That is the next memory target.
