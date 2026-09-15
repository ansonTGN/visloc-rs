# Basalt online mapper: design (Phase A)

Status: 2026-09-15, Phase A of the online-mapper build approved by the owner
(`docs/vi_slam_global_consistency_plan.md` §1.4/§3/§4 calls this "Stage 3" in
its original numbering; the current revision of that document tracks the same
goal as its Stage 7, sequenced after further VIO accuracy work — this task is
an explicit owner override: online-ize the *existing* 8/11-vs-ORB-SLAM3 result
(§1.4) now, rather than wait for Stage 5/6). This document is the design and
measurement report requested before any implementation (Phase B) starts.

## 1. What was measured

Tool: `examples/basalt_mapper_stage_profile.rs` (new, throwaway diagnostic —
not part of any parity path). It loads the first *N* MargData packets from a
`--marg-dir`, builds a fresh `NfrMapper`, ingests them via `add_marg_data`,
then times each `run_headless`-equivalent stage individually:
`detect_keypoints`, `match_stereo`, `match_all`, `build_tracks`, `setup_opt`,
`optimize(10)` ×2, `filter_outliers`, reading Windows peak working-set bytes
(`GetProcessMemoryInfo`) after each *N*.

Input: `E:\visloc-rs-runs\basalt_official_calib_20260915\vio_marg\MH_01_easy\marg_data`
(451 packets, official-calibration VIO run that produced the §1.4 8/11
result). Raw output: `E:\visloc-rs-runs\basalt_online_mapper_design_20260915\stage_profile_mh01.{json,log}`.

### 1.1 Per-stage timing (MH_01, official calibration)

| N (packets) | detect | stereo | match_all | build_tracks | setup_opt | optimize×2 | filter | peak RSS |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 50 | 5.58 s | 0.20 s | **47.17 s** | 0.09 s | 0.01 s | 1.26 s + 0.64 s | 1.01 s | 726 MB |
| 100/200/454 | *(running — see §1.1a; this section will be filled from the completed sweep before Phase B starts, but the N=50 result already settles the priority ordering below)* | | | | | | | |

**`match_all` is 84 % of total wall time already at N = 50** (47.17 s of
56.0 s), an order of magnitude more than every other stage combined. This is
the dominant lever for online-izing the mapper, full stop — not a secondary
optimization.

### 1.1a Why `match_all` is quadratic-shaped and what bounds it

`NfrMapper::match_all_impl` (`pipelines/basalt/src/mapper/session.rs:1759`)
does, on every call:

1. For **every** key in `feature_corners` (i.e. every image ever detected,
   not just new ones), call `query_bow_candidates`
   (`pipelines/basalt/src/mapper/features.rs:490`), which linearly scans the
   **entire** database of earlier images (`image.frame_id < query_id.frame_id`)
   computing an L1 BoW score against each, then truncates to the top
   `match_window` (30 by default) candidates.
2. For each candidate surviving the score gate
   (`frames_to_match_threshold = 0.04`), run mutual descriptor matching and
   (if the raw-match-count gate passes) full RANSAC (`match_temporal_ransac`,
   OpenGV-style, up to 100 iterations).

Step 1 is `O(N)` work per query → `O(N²)` total, but each comparison is a
cheap float loop over sparse BoW vectors, so it is not expected to dominate
wall time even at EuRoC's largest sequence (~2700 frames / a few hundred
keyframes). Step 2 is `O(min(N, match_window))` RANSAC-eligible candidates
per query once the database exceeds the window, so **step 2's cost is
`O(N · match_window)` — linear in N — once N > 30**; below N = 30 every query
sees its whole (smaller) history and the cost is `O(N²)` in the expensive
RANSAC step too, which is exactly the regime the N=50 measurement sits in
(most of MH_01's early images still see close to their entire — sub-30 —
history). **This predicts `match_all`'s absolute cost keeps growing with N
past 50 but its *marginal* cost per new keyframe should flatten out once the
30-frame window is the effective candidate bound**, i.e. the *incremental*
per-keyframe form of this stage is the fix, not a smaller `match_window`.

The critical design consequence: **`match_all` recomputes candidates for
every already-processed image on every call** (there is no "only images
without a match_all pass yet" tracking). Called once per `run_headless` in
the offline path this is merely wasteful; called incrementally as new
keyframes arrive online it would be `O(N)` extra work *per new keyframe*,
i.e. `O(N²)` over a sequence — unacceptable for online use. §3.2 below is the
fix: track a "last matched" watermark so a new keyframe queries only the
existing BoW database once (upstream's own `image.frame_id < query_id`
exclusion already makes this correct — old images' candidate sets never
change as new images arrive, they just were never cached).

### 1.2 What dominates the offline mapper's 6 GB RSS

Direct inspection of one MargData packet
(`vio_marg/MH_01_easy/marg_data/frame_000072.json`, 16.05 MB) via
`json.load`: **16 `of_images` entries, each a base64 string of the full
752×480 u16 frame (962,560 base64 chars ≈ 721,920 raw bytes = 752·480·2)**.
That is ~11.5 MB of raw pixel data per packet (16 images × 721,920 bytes),
essentially the *entire* packet size — the AOM matrices, poses, and
observations are comparably tiny next to the image payload.

`NfrMapper::retain_images` (`session.rs:735`) stores every packet's
`of_images` into `self.img_data: BTreeMap<i64, Vec<OfImageData>>`
(`OfImageData::data: Vec<u16>`, i.e. decoded, not base64) **forever** — it is
never cleared, not even across `run_headless`'s stage resets (only
`feature_corners`/`feature_matches`/`feature_match_data` are cleared there).
At 451 packets × ~11.5 MB ≈ **5.2 GB**, which matches the observed 3–6 GB RSS
range for the offline mapper almost exactly. By contrast:

- The optimize stage's dense pose-pose Hessian
  (`DMatrix::zeros(pose_count*6, pose_count*6)` in `linearize_vision`,
  `mod.rs:1300`) is only ~59 MB at MH_01's 454 poses (2724×2724 f64) — not a
  memory problem at EuRoC scale, though its `O(pose_count³)` solve cost is a
  latency concern addressed in §3.4.
- `feature_matches`/`feature_match_data` store inlier index pairs and one
  `SE3` per accepted pair — tens of MB at most for EuRoC-sized sequences.

**Conclusion: `img_data` (retained raw pixel buffers) is the memory story,
not matches or the BA workspace.** The online mapper's single most important
memory rule is in §4.1: never retain `OfImageData`/raw pixels past the
`detect_keypoints` call that consumes them.

## 2. Which stages are incremental-capable as-is vs. need new code

| Stage | Current behavior | Incremental-capable as-is? |
| --- | --- | --- |
| `add_marg_data` | Already per-packet; O(1) amortized state merge | **Yes** |
| `detect_keypoints` | Iterates **all** of `img_data` every call, `extend`s `feature_corners` (idempotent but wasteful — O(N) per call, not O(new)) | No — needs a "detected up to" watermark so it only processes images from packets since the last call |
| `match_stereo` | Iterates **all** of `img_data` every call; same-timestamp pairs only, so each pair's result never changes once computed, but it's still recomputed | No — same watermark fix as detect |
| `match_all` | Recomputes candidates for **every** key each call (§1.1a) | No — needs `match_new_keyframe`: query only the new key(s) against the existing BoW database (upstream's own `frame_id <` ordering already makes old results stable, so this is a correctness-preserving change, not an approximation) |
| `build_tracks` | Full `TrackBuilder::Build` over the whole `feature_matches` graph from scratch every call (union-find has no incremental variant in this port) | No — cheapest to keep as a periodic full rebuild (0.09 s at N=50; even at N=454 this is expected to stay well under a second, see caveat in §3.3) rather than porting incremental union-find right now |
| `setup_opt` | Re-triangulates from `feature_tracks` from scratch every call | Same as `build_tracks` — periodic full rebuild, not per-KF |
| `optimize` | `GlobalBaOptimizerState` (`lambda`, `lambda_vee`) already persists **across separate `optimize()` calls** on the same `NfrMapper` (`session.rs:580`) | **Yes for warm-starting** — this is exactly what an online periodic-trigger optimizer needs; no new state-carrying code required, just a policy for when to call it |
| `filter_outliers` | O(landmarks); cheap (1.01 s at N=50, dominated by `reprojection_diagnostics` which is already a single pass) | Effectively yes — periodic, same cadence as optimize |

## 3. Architecture

### 3.1 Threading model

Two threads, one process, `std::thread` + `std::sync::mpsc` (the repository
has no existing async runtime or crossbeam dependency — `pipelines/basalt/src/vio/aom.rs`
uses plain `std::thread::spawn` for its parallel BA workers, so this matches
existing convention and adds no new dependency):

- **VIO thread** (existing `BasaltVioEstimator`/adapter, unchanged): consumes
  the dataset at real-time or as-fast-as-possible rate. After each
  `adapter.process(frame)` call, if `output.estimator.marg_data.is_mapper_packet()`
  (`MargData::is_mapper_packet`, `margdata.rs:671`), the packet is **moved**
  (not cloned, not serialized) into an `mpsc::Sender<MargData>` bounded
  channel to the mapper thread. `EstimatorOutput::marg_data: MargData` is
  already an owned field the demo currently only serializes to JSON for the
  offline path (`examples/basalt_euroc_vio_demo.rs:158-179`) — the online
  demo skips that entirely and hands the value straight to the channel. This
  satisfies "no JSON/base64 round trip" for free: the JSON+base64 path only
  exists today because the offline demo chose to persist packets to disk for
  a separate process to read later; in-process there was never a need for
  it.
- **Mapper thread** (`OnlineNfrMapper`, new, wraps `NfrMapper`): receives
  packets from the channel in a loop. For each packet: `add_marg_data`,
  `detect_new_keypoints` (incremental, §3.2), `match_stereo_new` (incremental,
  §3.2), `match_new_keyframe` (incremental BoW query, §3.2). Every K accepted
  keyframes *or* whenever `match_new_keyframe` accepts a loop pair (a
  temporal match whose two frames are more than `match_window` apart in
  keyframe order — the mapper's own definition of "not just adjacent
  temporal continuation"), it triggers a background optimization pass:
  `build_tracks` → `setup_opt` → `optimize(few iters)` → `filter_outliers` →
  `optimize(few iters)`, reusing the persistent `GlobalBaOptimizerState`
  warm-start (§2). The VIO thread is never blocked: the channel is the only
  synchronization point, and it is a producer/consumer queue, not a
  request/response call.
- A bounded channel (capacity ~8 packets) applies natural backpressure: if
  the mapper falls behind, the VIO thread's `send` blocks momentarily rather
  than growing memory unboundedly, and the online demo reports this as
  "mapper queue lag" (frames the VIO produced before the mapper drained the
  previous one) — an honest signal that the run was not real-time-mapped
  even if the VIO itself was real-time.
- Final optimization: when the VIO thread finishes the sequence, it closes
  the channel (drops the `Sender`); the mapper thread's receive loop exits on
  channel closure, runs one last full `build_tracks`/`setup_opt`/`optimize`/
  `filter`/`optimize` pass (matching the offline `run_headless` order
  exactly, so the final trajectory is produced by the *same* optimization
  code path as the offline mapper — see §5), and writes `trajectory_online.tum`
  from the propagated final keyframe poses.

### 3.2 Incremental frontend methods (new, `pipelines/basalt/src/mapper/online.rs`)

None of these change the existing `NfrMapper` methods (bit-identity
requirement, §5) — they are new methods on a wrapper that call the
lower-level pure functions (`extract_mapper_features`, `match_stereo_features`,
`query_bow_candidates`, `match_temporal_stage`, `match_temporal_ransac`)
directly, the same functions `session.rs` already calls, just scoped to only
the new image(s)/key(s) instead of the full map:

- `detect_new_keypoints(&mut self)`: like `detect_keypoints` but iterates
  only `img_data` entries inserted since the last call (a `BTreeSet<i64>` of
  already-detected timestamps, or simply the tail of `img_data` since it's a
  `BTreeMap<i64, _>` — track the last-processed key).
- `match_stereo_new(&mut self)`: same watermark restricted to the new
  timestamp(s).
- `match_new_keyframe(&mut self, new_key: TimeCamId)`: calls
  `query_bow_candidates(new_key, ..., &existing_database, match_window)`
  exactly as `match_all_impl` does per-query today, against the database of
  **already-inserted** keys only (this is already what upstream's `frame_id <`
  filter does — no semantic change), then runs the same
  descriptor-match/RANSAC/accept logic as the existing loop body. Called once
  per new keyframe, this reduces §1.1a's `O(N²)` BoW-scoring re-scan to
  `O(N)` total over the sequence (`O(1)` amortized per keyframe, same
  algorithm, just not repeated for already-processed keys), and keeps the
  RANSAC step at its already-linear `O(match_window)` per keyframe.
- These three are covered by Phase B unit tests asserting **identical
  accepted pairs** to a batch `match_stereo`/`match_all` call on the same
  packet set (same candidate order, same RANSAC seed policy) — an exactness
  requirement, not just a performance one, because §5 requires the final
  optimized trajectory to be reproducible from the same inputs.

### 3.3 Trigger policy

- Optimize every **K = 20 accepted keyframes** by default (tunable via
  `--optimize-every-k`), or immediately when `match_new_keyframe` accepts a
  non-adjacent (loop) pair — matching the task brief's "every K keyframes and
  on every accepted loop."
- `build_tracks`/`setup_opt` are full rebuilds (§2), run only at the same
  cadence as `optimize`, not per-keyframe. Risk: `build_tracks`'s
  union-find rebuild is `O(matches)` and was 0.09 s at N=50; if it grows
  worse than linearly by N=454 (not yet measured — §1.1 table pending) it
  could become the new bottleneck once `match_all`'s cost is fixed. Phase B's
  N=100/200/454 sweep (already running as of this report, results to be
  folded in before implementation starts) settles this; if it does exceed
  ~1 s at 454, the kill/pivot is to keep periodic-K full rebuilds anyway
  (still far cheaper than today's `match_all` cost) rather than porting
  incremental union-find.
- `optimize`'s warm start (§2) means each periodic call only needs a handful
  of LM iterations (the offline path already only requests 10), not a
  from-scratch solve — this is what keeps the `O(pose_count³)` dense solve
  affordable at the every-K cadence instead of every keyframe.

### 3.4 Memory plan (target: peak RSS ≤ ~1 GB)

1. **Never retain `OfImageData`/raw pixels past `detect_new_keypoints`**
   (§1.2's 5.2 GB finding). `OnlineNfrMapper` must not keep an `img_data`-like
   map at all: the mapper thread receives one `MargData` packet at a time
   from the channel, extracts features immediately, and drops the packet
   (including its `of_images`) before receiving the next one. This alone
   removes the dominant memory cost.
2. Keep `feature_matches`/`feature_match_data` only for **accepted** pairs
   (already true today — `session.rs` only inserts on RANSAC acceptance).
3. `optimize`'s dense Hessian (~59 MB at MH_01 scale, §1.2) is bounded by
   `pose_count`, which is bounded by keyframe count — fine for EuRoC-length
   sequences (≤ ~500 keyframes); flagged as a scaling risk for much longer
   trajectories, not an EuRoC-scope problem.
4. Estimated total: VIO thread's existing 29 MB (§1.1 of the plan doc) +
   mapper thread's compact state (BoW vectors + descriptors for every
   keyframe, no raw pixels, no duplicated packets) + the ≤~60 MB BA
   workspace. This is expected to land near or under the ~1 GB target but
   is an estimate pending the Phase C measured run — reported honestly, not
   assumed.

## 4. What must stay bit-identical

- `pipelines/basalt/src/mapper/{mod.rs,session.rs,features.rs,triangulation.rs}`
  are **not modified**. `OnlineNfrMapper` (new file, `online.rs`) wraps
  `NfrMapper` and calls its existing public methods plus the same
  lower-level pure functions the existing methods call — it adds new
  incremental entry points, it does not change any existing one's behavior
  or signature.
- `examples/basalt_mapper_offline_demo.rs` and its JSON/base64 MargData
  on-disk path are untouched — the online demo is a new binary
  (`examples/basalt_euroc_online_slam_demo.rs`), not a replacement.
- The existing parity test suite (`cargo test -p visloc-basalt --lib`,
  374 passed / 0 failed / 59 ignored) must stay green; Phase B adds new
  tests for the incremental matcher and channel, it does not modify existing
  ones.
- The final online trajectory comes from the *same* `optimize`/
  `filter_outliers`/`optimize` sequence the offline path uses at the end
  (§3.1's "final optimization" step) — the online run's last pass is
  literally `run_headless`'s tail called on the fully-ingested state, so a
  full-history online run and an offline run over the same packets are
  expected to produce numerically close (not necessarily bit-identical,
  because the incremental frontend may process pairs in a different order
  than the batch path, which can perturb floating-point summation order in
  BoW/RANSAC — Phase B's unit tests check *identical accepted pairs*, not
  bit-identical intermediate floats) trajectories. This is the basis for the
  Phase C "online ATE within ~10% of offline" gate.

## 5. Effort estimate

- Phase A (this document): ~1 measurement run + design, matches the
  "≤ half a day" budget.
- Phase B (implementation): `online.rs` (incremental methods + trigger
  policy + thread plumbing), channel wiring in the adapter/demo, new example,
  unit tests for exactness. Estimated 1.5–2.5 days given the frontend
  primitives already exist as reusable pure functions (§3.2) and the
  optimizer warm-start needs no new code (§2) — the actual new logic is the
  watermarking/dispatch, not new numerical code.
- Phase C (evaluation): MH_01 first (~15–30 min including the offline/
  ORB-SLAM3 comparison), then all 11 detached (each offline run was 2–18 min
  per the official-calibration reference; online should be comparable or
  faster per-sequence since `match_all`'s fix removes the dominant cost, but
  budget similar wall time for the full sweep with real-time-factor
  reporting). Estimated 0.5–1 day including any re-tuning if the 10% ATE
  gate is missed on MH_01.
- Phase D (README/PR): 0.5 day.
- **Total estimate: 3–4.5 days**, dominated by Phase B/C, not this design
  phase.

## 6. Artifacts

- Stage-profile tool: `examples/basalt_mapper_stage_profile.rs`
- Raw measurements: `E:\visloc-rs-runs\basalt_online_mapper_design_20260915\stage_profile_mh01.{json,log}`
- Worktree: `E:\visloc-rs-runs\online_wt`, branch `feat/basalt-online-mapper`
