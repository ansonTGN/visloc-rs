# Rust 3D Gaussian Splatting: Renderer + Trainer Plan

Goal: take a visloc-rs SfM reconstruction (COLMAP model) and produce a 3D
Gaussian Splatting scene that a **pure-Rust real-time renderer** can display,
with the **training implemented in Rust** rather than the current
Python/torch/gsplat path. Browser (WebGPU/wasm) is a desired secondary target,
but must not impose constraints that slow the native path.

This document records the decision, the concrete architecture, and the staged
plan. It is a plan, not a claim.

## Where we start from

visloc-rs already produces the *input* to a 3DGS optimizer:

- `write_colmap_reconstruction_for_3dgs` (`crates/io/src/colmap/mod.rs:523`)
  writes a COLMAP model with genuine multi-view `TRACK[]` tails.
- EuRoC V2_03 (Vicon room orbit, frames 0-150) already renders **crisp** from
  visloc-rs's own poses: reprojection 0.53 px, downstream l1 ~= 0.006
  (`docs/euroc_sfm_benchmark.md:53`).
- A real `.splat` (antimatter15 32-byte) and a WebGL viewer already exist under
  `docs/euroc_splat/`.

What does not exist is a Rust trainer or a Rust renderer. Today those are
`scripts/gsplat_mcmc_train.py` (Python + torch + gsplat, CUDA) and
`scripts/render_splat_flythrough.py` / the WebGL viewer.

## Survey of the ecosystem (2026)

| Project | Lang | Render | Train | GPU backend | Browser | License | Status |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [brush](https://github.com/ArthurBrussee/brush) | Rust | yes | **yes** | wgpu + CUDA (Burn/CubeCL) | yes (wasm) | Apache-2.0 | 5.0k star, v0.3.0, active |
| bevy_gaussian_splatting | Rust | yes | no | wgpu | yes | MIT/Apache | 276 star, active |
| web-splat | Rust | yes | no | wgpu | yes | Apache-2.0 | 297 star |
| wgpu-3dgs-viewer | Rust | yes | no | wgpu 30 | yes | see repo | v0.8.0 |
| gsplat (nerfstudio) | CUDA/Python | yes | yes | CUDA | no | Apache-2.0 | reference |
| candle | Rust | - | custom op | CUDA only | CPU wasm only | MIT/Apache | maintenance-only |
| burn + cubecl | Rust | - | yes (custom op) | wgpu / CUDA | yes | MIT/Apache | active |

Key findings that shape the decision:

1. **brush is the only mature pure-Rust 3DGS trainer**, and it uses **Burn +
   CubeCL custom kernels** — one kernel language that compiles to both wgpu and
   CUDA. This is the proven architecture, not speculative.
2. **brush pins Burn to git `main`** (not a released version) and uses a
   **forked wgpu** for WebGPU subgroup support. Depending on released
   crates.io Burn would not reproduce the browser path.
3. **WebGPU reached Baseline in January 2026** (Chrome 113+ since 2023, Safari
   26, Firefox 141+ on Windows). But core WebGPU has **no subgroup ops and no
   reliable global atomics**, so any GPU sort must be a hierarchical radix sort.
   In-browser *training* is proven but niche and Chrome-only.
4. A full forward + **analytic backward** differentiable rasterizer is the hard
   part. brush's renderer alone is ~8k lines (forward + backward kernels, sort,
   scan, camera models), plus ~1.7k lines of training loop.

## Decision

**Self-implement on wgpu + Burn + CubeCL, in a separate workspace crate.**
Do not depend on brush (it would forfeit control of the training loop, the
visloc-rs integration, and the benchmark surface, and it drags in a forked
wgpu). Read brush as the reference implementation.

Constraints inherited from visloc-rs architecture (`docs/decisions.md`):

- **`visloc-core` must not depend on CUDA, torch, wgpu, or Burn.** The 3DGS
  crates are separate workspace members, exactly like `visloc-basalt`.
- The SfM side stays the existing `write_colmap_reconstruction_for_3dgs`; the
  new crate consumes a COLMAP model directory. No SfM changes are required for
  stage 0.
- Default builds stay lightweight; the 3DGS crates are opt-in workspace members
  and are not pulled in by `visloc-rs`'s default features.

### Library vs. brush vs. our own (why not just use brush)

- Control: we want to drive training from visloc-rs SfM output and measure it
  in our own benchmark registry.
- Dependency weight: brush's fork of wgpu is a hard fork; adopting it couples us
  to their WebGPU workarounds.
- Learning: reimplementing the backward pass is the point of "学習もRust移植".

## Architecture

```
visloc SfM  --(write_colmap_reconstruction_for_3dgs)-->  COLMAP model dir
                                                              |
                        +-------------------------------------+
                        v
   crates/gsplat-core   (plain Rust: COLMAP model parse, SH math, camera,
                         gaussian parameter types, .ply/.splat IO, losses)
                        |
        +---------------+----------------+
        v                                v
 crates/gsplat-render              crates/gsplat-train
 (wgpu compute rasterizer,         (Burn/CubeCL autodiff trainer:
  forward; then backward)           init from SfM points, optimize,
                                    densify/prune, export)
        |                                |
        +---------------+----------------+
                        v
                apps/gsplat-cli / examples/*
                (train, render flythrough, export .ply/.splat)
```

Staged so each stage is useful on its own:

### Stage 0 — IO + preview (no GPU training yet) — IMPLEMENTED
- `crates/gsplat-core`: COLMAP model → scene seed (via `visloc-io`), gaussian
  parameter types, SH degree-3 evaluation, `.splat` and Inria `.ply` read/write,
  camera projection.
- A **CPU reference renderer** (tiny, slow, plane-scan rasterizer) used as the
  ground truth for the GPU kernels.
- Deliverable: `gsplat_cpu_render` example loads a real `.splat` (422,860
  gaussians from `docs/euroc_splat/`) and renders a frame; a COLMAP model
  (`--colmap`) renders its sparse cloud from a registered camera. Round-trip
  `.splat`/`.ply` pinned by tests.

### Stage 1 — wgpu forward renderer — IMPLEMENTED
- `crates/gsplat-render`: wgpu 30 compute rasterizer. Five dispatches run the
  whole forward pass on the GPU — `project_forward` (project + compact visible
  gaussians + tile counts), `project_visible` (SH colour + projected splats),
  `map_gaussians` (tile expansion), `get_tile_offsets` (per-tile ranges), and
  `rasterize` (one workgroup per 16x16 tile, front-to-back transmittance).
- All shaders use **only baseline WebGPU**: `u32` storage atomics, workgroup
  barriers, no subgroups. This keeps the wasm path open.
- The CPU reference in `gsplat-core` was corrected to the same physically
  correct front-to-back transmittance compositing (it previously applied an
  "over" operator in scene order), so GPU and CPU agree by construction.
- Deliverable met: `gsplat_gpu_render` renders a real 422,860-gaussian
  `.splat` on the GPU and writes a PNG.

**Measured.** Synthetic 3-gaussian scene at 64x64: GPU vs CPU **max abs error
0.006, mean 0.000014, no pixel over 0.1** (pinned by
`gpu_matches_cpu_reference`). Real `euroc_v101.splat` (418k gaussians after
outlier prune) at 640x480: mean abs error 0.003 on a 1/104 subsample; the
residual is faint boundary floaters. Native forward timing on a GTX 1660 Ti:
~79 ms/frame at 640x480, ~119 ms/frame at 1920x1080 for 419k gaussians.

**Device-side sort + scan (implemented).** The two per-frame sorts and the
compact-order tile-count prefix sum now run on the GPU:

- `shaders/radix.wgsl` — subgroup-free LSD radix sort (4 bits/pass, 8 passes),
  with a **stable** in-block per-digit exclusive scan so LSD correctness holds.
  Per-pass: `radix_histogram` → `radix_scan` (O(BINS·blocks)) → `radix_scatter`.
  A dedicated test (`gpu_radix_sort_is_correct_and_stable`) checks 5,000 keys
  with duplicates against a stable host sort.
- `shaders/scan.wgsl` — subgroup-free blocked inclusive scan, used for the
  compact-order tile counts that `map_gaussians` consumes.

This removes every large GPU→CPU readback from the frame (depth array, id
array, counts array, both isect arrays); only the 8-byte `num_visible` /
`num_intersections` counters and the output image still come back.
`gpu_matches_cpu_reference` still passes on the fully device-side pipeline.

**Honest performance.** The end-to-end PNG path is ~80 ms at 640x480 and
~114 ms at 1920x1080 for 419k gaussians — but almost all of that is the
**output-image readback** (~90 ms at 1920x1080), not the render. Timing the
GPU-only path (no image readback, which is what a viewer pays) gives:

| resolution | GPU-only | full PNG path |
| --- | --- | --- |
| 640x480 | 49 ms | 80 ms |
| 1920x1080 | 26 ms | 114 ms |

So the renderer is already near-real-time (~40 fps at 1080p for 419k
gaussians); the counter readback is only ~0.4 ms. Two costs remain:

1. The **output-image readback** (~90 ms at 1080p), which a viewer avoids by
   presenting the output buffer directly.
2. ~~CPU submit overhead~~ — **falsified**, see below.

`Renderer::set_skip_readback(true)` exposes the GPU-only path for measurement.

**Per-stage profile and two correctness fixes.** `GSPLAT_PROFILE=1` now makes
`render` block after every stage and print per-stage wall time, the raw
counters, and per-tile list-length statistics. What it showed (GTX 1660 Ti,
419k gaussians):

- *Submit overhead is not the cost.* Folding each radix sort into one submit
  (24 → 2 submits/frame) left GPU-only time unchanged (640x480: 48.6 → 47.7
  ms; 1080p within noise).
- *The auto-framed benchmark view is unrepresentative.* It puts the camera far
  outside the scene, so ~57% of visible gaussians land in a single tile and
  `rasterize` (one workgroup per tile) is bound by that one tile. The example
  now takes `--pose-cw qw,qx,qy,qz,tx,ty,tz` (world-to-camera); a V1_01
  ground-truth cam0 pose (`T_WC = T_WB · T_BS`) renders the room correctly,
  confirming `euroc_v101.splat` is in the EuRoC GT frame.
- *Bug: prefix-scan race.* `scan_apply` had every workgroup rewrite
  `block_sums` in place, racing across workgroups, so `cum_tiles_hit` was wrong
  whenever the scan spanned more than one 2048-element block and isects landed
  in other gaussians' slots (blocky tiles). The block-sum scan is now its own
  single-workgroup kernel; `gpu_prefix_scan_matches_host_across_many_blocks`
  fails on the old code at element 2048.
- *Bug: footprint cap.* CPU and GPU both capped the projected radius at
  `2 * max(w, h)`, cutting large near-camera gaussians off at a rectangle (hard
  tile-aligned edges). Both now use the Inria 1.3x-frustum Jacobian clamp and
  no cap. Real-pose parity (1/104 subsample): max error 0.63 → **0.0077**,
  pixels over 0.1: 5798 → **0**.

From the real pose (640x480, 5.3M intersections) the frame is sort-bound:

| stage | ms |
| --- | --- |
| tile_sort | 17.2 |
| tile_offsets | 6.4 |
| map_gaussians | 4.3 |
| rasterize | 3.6 |
| depth_sort | 2.8 |
| rest | 2.5 |
| **total** | **36.8** |

The tile sort now runs only `ceil(log2(num_tiles) / 4)` radix passes (tile ids
are small; the LSD sort is stable so depth order survives): 3 passes at
640x480, 4 at 1080p, versus 8. `rasterize` also stops loading batches once
every pixel in its tile is saturated. Next targets: the tile sort and
`tile_offsets` on real views.

**Sort, offsets and capacity (after the DXC switch).** Re-profiled with DXC
(which made the old tile sort *slower*, 17 → 24.6 ms), then:

- `radix_scan` was 16 threads each walking every block serially; it is now a
  chunked parallel scan of the digit-major histogram (tile_sort 24.6 → 17.7).
- 16 keys per radix thread instead of 4 amortises the in-block digit scan
  (17.7 → 10.8; 32 spills registers and doubles it).
- `get_tile_offsets` did an atomicMin/atomicMax per isect on the tile's two
  counters; the list is sorted, so only run boundaries now write, with plain
  stores (6.4 → 0.6 ms), dispatched in 2D past the 65535-workgroup limit.
- Isect buffers were sized once at `64 * n` (26.8M for this scene) and
  silently truncated beyond it; a 1080p view from inside the room needs 31M,
  and its `tile_offsets` dispatch then exceeded 65535 workgroups and panicked
  (also before this branch). The renderer now grows the isect buffers, the tile
  sort scratch and the affected bind groups on demand after reading the
  counters. `projected_splats` is per visible gaussian and now sized by `n`.

V1_01 GT pose, GTX 1660 Ti, GPU-only per frame:

| resolution | isects | before (FXC build) | now |
| --- | --- | --- | --- |
| 640x480 | 5.3M | 41 ms | **21 ms** |
| 1920x1080 | 31M | crash | 117 ms |

At 1080p the frame is dominated by the tile sort (72 ms), `map_gaussians`
(22 ms) and `rasterize` (19 ms) over 31M intersections: large near-camera
gaussians each cover hundreds of tiles. Tighter per-gaussian tile culling
(e.g. an opacity-aware extent instead of 3 sigma) is the next lever.

**DX12 startup.** `Renderer::new` used to take ~10 minutes on Windows: wgpu's
`Auto` shader-compiler choice falls back to FXC when `dxcompiler.dll` is not
on the DLL search path, and FXC is extremely slow on these compute shaders.
`GpuContext` now finds `dxcompiler.dll` on `PATH` or in the newest installed
Windows SDK and loads DXC explicitly (`WGPU_DX12_COMPILER` still overrides).
GPU test suite 145 s → 3.7 s; `gsplat_gpu_render` end to end ~590 s → 11 s.

### Stage 2 — Burn + CubeCL differentiable rasterizer
- Port the forward rasterizer to CubeCL kernels; add the **analytic backward**
  (projection backward + tile rasterize backward) as Burn custom ops.
- Validate every gradient against finite differences and against the CPU
  reference. This is the highest-risk stage; brush and diffsplat are the
  references.
- Deliverable: a differentiable render step whose `backward` matches FD.

### Stage 3 — Trainer
- Init gaussians from the SfM sparse points (on-surface init, the fix
  `euroc_colmap_splat.sh` documents), optimize with L1 + 0.2*(1-SSIM), RNG-free
  densify/prune or MCMC strategy.
- Deliverable: `gsplat-cli train <colmap_model> <out.ply>` runs in Rust on
  EuRoC V2_03 and produces a `.splat` comparable to the Python baseline
  (l1 ~= 0.006 on the orbit capture).

### Stage 4 — Browser (optional, non-blocking)
- wasm + WebGPU build of the renderer; training only if stage 2/3 land cleanly.
- Explicitly deferred so it cannot slow the native path.

## Benchmarking

Every claim recorded in `benchmarks/registry` with: commit, cargo lock hash,
cargo/rustc, model hash, dataset identity, command, and metric implementation.
Report against the existing Python gsplat baseline on EuRoC V2_03:
train wall-clock, PSNR/l1, gaussian count, render ms/frame at 752x480, and
peak GPU memory.

## Risks

- **Analytic backward correctness** — highest risk. Mitigation: CPU reference +
  finite-difference gradient checks before any training run.
- **WebGPU sort without global atomics** — mitigation: hierarchical radix sort
  (brush-sort reference), native-only for v1.
- **Burn is pre-1.0 and brush tracks git main** — pin an exact Burn/CubeCL
  revision; isolate behind our own crate so a Burn bump is contained.
- **Scope** — ~8k lines of renderer + ~1.7k of training in the reference. Staged
  so stage 0/1 deliver value even if stage 2 stalls.

## Non-goals for v1

- Feed-forward (inference) splatting.
- 2DGS / 4DGS.
- Real-time performance on wasm.
- Replacing the Python trainer in CI.
