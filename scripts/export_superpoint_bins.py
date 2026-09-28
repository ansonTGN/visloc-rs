#!/usr/bin/env python3
"""SuperPoint features for gsplat_euroc --import-features.

usage: export_superpoint_bins.py --images <work>/images --onnx superpoint.onnx --out <dir>

Writes <out>/frame_XXXXX.bin per frame_XXXXX.png: u32 count, then count x
(x, y) f32, then count x 256 f32 descriptors (little endian).
"""
import argparse
from pathlib import Path

import numpy as np
import onnxruntime as ort
from PIL import Image

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument('--images', type=Path, required=True)
ap.add_argument('--onnx', type=Path, required=True)
ap.add_argument('--out', type=Path, required=True)
args = ap.parse_args()
sess = ort.InferenceSession(str(args.onnx), providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
args.out.mkdir(parents=True, exist_ok=True)
frames = sorted(args.images.glob('frame_*.png'))
for p in frames:
    img = np.asarray(Image.open(p).convert('L'), dtype=np.float32) / 255.0
    kps, scores, desc = sess.run(None, {'image': img[None, None]})
    keep = scores > 0
    kps, desc = kps[keep].astype(np.float32), desc[keep].astype(np.float32)
    with open(args.out / (p.stem + '.bin'), 'wb') as f:
        f.write(np.uint32(len(kps)).tobytes())
        f.write(kps.tobytes())
        f.write(desc.tobytes())
print(f'{len(frames)} frames')
