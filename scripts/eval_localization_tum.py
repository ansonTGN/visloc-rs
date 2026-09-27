"""Score a localization.tum (map frame) against a ground-truth TUM trajectory.

usage: python scripts/eval_localization_tum.py <localization.tum> <gt.tum> [per_frame.csv]

Timestamps are matched within 20 ms; either file may store seconds or
nanoseconds (values above 1e6 with no fractional part are taken as ns). The
estimate is aligned with a Sim(3) Umeyama fit (the map is up to scale) and
the camera-centre RMSE is reported, together with the success rate and
latency from per_frame.csv when given.
"""
import csv
import sys

import numpy as np


def load(path):
    rows = []
    for line in open(path):
        t = line.split()
        if len(t) < 8 or line.startswith('#'):
            continue
        ts = float(t[0])
        if ts > 1e6 and '.' not in t[0]:
            ts *= 1e-9
        rows.append((ts, float(t[1]), float(t[2]), float(t[3])))
    return np.array(rows)


def umeyama(src, dst):
    ms, md = src.mean(0), dst.mean(0)
    s, d = src - ms, dst - md
    u, sig, vt = np.linalg.svd(d.T @ s / len(src))
    sg = np.eye(3)
    if np.linalg.det(u @ vt) < 0:
        sg[2, 2] = -1
    r = u @ sg @ vt
    c = np.trace(np.diag(sig) @ sg) / (s ** 2).sum(1).mean()
    return c * (r @ src.T).T + md - c * r @ ms


est, gt = load(sys.argv[1]), load(sys.argv[2])
pairs = []
for e in est:
    k = np.argmin(np.abs(gt[:, 0] - e[0]))
    if abs(gt[k, 0] - e[0]) <= 0.02:
        pairs.append((e[1:], gt[k, 1:]))
if len(pairs) < 3:
    sys.exit(f'only {len(pairs)} matched timestamps')
src = np.array([p[0] for p in pairs])
dst = np.array([p[1] for p in pairs])
err = np.linalg.norm(umeyama(src, dst) - dst, axis=1)
line = f'{len(pairs)} poses scored, Sim(3) ATE RMSE {np.sqrt((err ** 2).mean()):.3f} m, median {np.median(err):.3f} m, max {err.max():.3f} m'
if len(sys.argv) > 3:
    rows = list(csv.DictReader(open(sys.argv[3])))
    ok = sum(1 for r in rows if r['success'] == 'true')
    lat = sorted(float(r['latency_ms']) for r in rows)
    line += f'; localized {ok}/{len(rows)}, latency p50 {lat[len(lat) // 2]:.0f} ms, p90 {lat[int(len(lat) * 0.9)]:.0f} ms'
print(line)
