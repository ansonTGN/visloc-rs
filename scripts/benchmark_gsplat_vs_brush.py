"""Reproduce the 3DGS training table (ours vs brush 0.3) in
docs/rust_3dgs_plan.md and the README.

Protocol (docs/rust_3dgs_plan.md, "Evaluation protocol"):
- Every 8th view by image name is held out: brush's and the Inria split.
- 30k steps per trainer. Ours runs with `--strategy brush` and a 2.5M
  gaussian cap.
- Both trainers' final PLYs are scored by the same `gsplat_eval` (one
  renderer, black background).

Datasets: each scene directory holds `images/` and `sparse/0/` (COLMAP).
- Mip-NeRF 360 (bonsai, room, garden) uses `images_4` renamed to `images`,
  with the original `sparse/0`.
- The COLMAP datasets (south-building, gerrard-hall) are undistorted with
  `colmap image_undistorter --max_image_size 1024`.

usage:
  cargo build --release -p visloc-gsplat-train --features gpu \\
      --example gsplat_train --example gsplat_eval
  python scripts/benchmark_gsplat_vs_brush.py --out <dir> \\
      --scene bonsai=<mipnerf360>/bonsai --scene room=<mipnerf360>/room \\
      --scene garden=<mipnerf360>/garden \\
      --scene south-building=<undistorted>/south-building \\
      --scene gerrard-hall=<undistorted>/gerrard-hall \\
      [--brush <brush_app>] [--ours-args "--normal-weight 0.005"]

Writes <out>/results.jsonl (one row per scene and trainer) and prints a
Markdown table. Times are wall-clock training time on this machine. The GPU
must be otherwise idle: concurrent jobs distort both the timings and brush's
refine schedule.
"""
import argparse
import json
import os
import re
import shlex
import subprocess
import sys
import time

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--out', required=True)
ap.add_argument('--scene', action='append', required=True, help='name=path (repeatable)')
ap.add_argument('--train-exe', default=os.path.join('target', 'release', 'examples', 'gsplat_train'))
ap.add_argument('--eval-exe', default=os.path.join('target', 'release', 'examples', 'gsplat_eval'))
ap.add_argument('--brush', help='brush_app executable (brush 0.3); omit to skip brush')
ap.add_argument('--steps', type=int, default=30000)
ap.add_argument('--ours-args', default='', help='extra gsplat_train arguments')
args = ap.parse_args()
os.makedirs(args.out, exist_ok=True)


def evaluate(ply, data):
    out = subprocess.run([args.eval_exe, '--ply', ply, '--data', data, '--eval-every', '8'],
                         capture_output=True, text=True).stdout
    m = re.search(r'mean psnr ([\d.]+)\s+ssim ([\d.]+)', out)
    return (float(m.group(1)), float(m.group(2))) if m else (None, None)


def timed(cmd, log):
    t = time.time()
    with open(log, 'w') as f:
        rc = subprocess.call(cmd, stdout=f, stderr=subprocess.STDOUT)
    return rc, time.time() - t


rows = []
for spec in args.scene:
    name, data = spec.split('=', 1)
    # The COLMAP datasets are already 1024 px; brush must not downscale Mip-NeRF 360's images_4.
    max_res = '1024' if 'building' in name or 'hall' in name else '1600'
    ours_dir = os.path.join(args.out, 'ours', name)
    os.makedirs(ours_dir, exist_ok=True)
    ply = os.path.join(ours_dir, 'ours.ply')
    rc, secs = timed([args.train_exe, '--data', data, '--strategy', 'brush', '--steps', str(args.steps),
                      '--eval-every', str(args.steps), '--max-gaussians', '2500000', '--out', ply,
                      *shlex.split(args.ours_args)], os.path.join(ours_dir, 'train.log'))
    psnr, ssim = evaluate(ply, data) if rc == 0 else (None, None)
    rows.append({'scene': name, 'trainer': 'ours', 'rc': rc, 'seconds': round(secs), 'psnr': psnr, 'ssim': ssim})
    print(rows[-1], flush=True)
    if args.brush:
        brush_dir = os.path.join(args.out, 'brush', name)
        os.makedirs(brush_dir, exist_ok=True)
        rc, secs = timed([args.brush, data, '--total-steps', str(args.steps), '--max-resolution', max_res,
                          '--eval-split-every', '8', '--export-every', str(args.steps), '--export-path', brush_dir],
                         os.path.join(brush_dir, 'brush.log'))
        bply = os.path.join(brush_dir, f'export_{args.steps}.ply')
        psnr, ssim = evaluate(bply, data) if os.path.exists(bply) else (None, None)
        rows.append({'scene': name, 'trainer': 'brush', 'rc': rc, 'seconds': round(secs), 'psnr': psnr, 'ssim': ssim})
        print(rows[-1], flush=True)
    with open(os.path.join(args.out, 'results.jsonl'), 'w') as f:
        f.writelines(json.dumps(r) + '\n' for r in rows)

print('\n| scene | trainer | PSNR / SSIM | time |\n| --- | --- | --- | ---: |')
for r in rows:
    q = f"{r['psnr']:.2f} / {r['ssim']:.3f}" if r['psnr'] is not None else 'failed'
    print(f"| {r['scene']} | {r['trainer']} | {q} | {r['seconds']} s |")
sys.exit(0)
