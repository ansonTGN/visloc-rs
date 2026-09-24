#!/usr/bin/env python3
"""Paired full-sequence screen for the opt-in urgent keyframe policy.

Both arms use one frozen binary; only the config JSON differs. The reference
config is the keyframe-sweep kf5 config; the candidate adds
`vio_urgent_kf_keypoints_thresh` / `vio_urgent_min_frames_after_kf`.
Stops at the first failure or timeout; never retries or overwrites a run.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

REPO = Path(__file__).resolve().parents[1]
SCRIPTS = REPO / 'scripts'
SWEEP = REPO / 'target/keyframe_sweep_full_20260921'
MH = '/mnt/win/linux_data/euroc_mh03_official_20260830/extracted'
VICON = '/mnt/win/linux_data/euroc_vicon_missing_20260921/extracted'
DATASETS = {
    'MH_03_medium': '/mnt/win/linux_data/euroc_mh03_official_20260830/MH_03_medium',
    'MH_04_difficult': f'{MH}/MH_04_difficult',
    'V2_03_difficult': f'{MH}/V2_03_difficult',
    'V2_02_medium': f'{VICON}/V2_02_medium',
}
# Sequences not used while designing the urgent policy (held-out check).
HELD_OUT = {
    'MH_01_easy': f'{MH}/MH_01_easy',
    'MH_02_easy': f'{MH}/MH_02_easy',
    'MH_05_difficult': f'{MH}/MH_05_difficult',
    'V1_01_easy': f'{VICON}/V1_01_easy',
    'V1_02_medium': f'{VICON}/V1_02_medium',
    'V1_03_difficult': f'{VICON}/V1_03_difficult',
    'V2_01_easy': f'{VICON}/V2_01_easy',
}
DATASETS.update(HELD_OUT)
FLAGS = ['--optimize-every-k', '100', '--periodic-iterations', '4', '--threads', '4',
         '--loop-match-max-rot-error', '30']


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def save(path, value):
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    tmp.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out-root', type=Path, required=True)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--threshold', type=float, default=0.5)
    parser.add_argument('--urgent-spacing', type=int, default=2)
    parser.add_argument('--sequences', nargs='+', default=list(DATASETS)[:4])
    parser.add_argument('--per-run-seconds', type=int, default=1500)
    args = parser.parse_args()

    root = args.out_root
    root.mkdir(parents=True)
    binary = root / 'slam_demo'
    shutil.copy2(args.binary, binary)
    base = json.loads((SWEEP / 'runs/MH_03_medium/kf5/rep1/config.json').read_text())
    assert base['value0']['config.vio_min_frames_after_kf'] == 5
    candidate = json.loads(json.dumps(base))
    candidate['value0']['config.vio_urgent_kf_keypoints_thresh'] = args.threshold
    candidate['value0']['config.vio_urgent_min_frames_after_kf'] = args.urgent_spacing
    save(root / 'baseline_config.json', base)
    save(root / 'candidate_config.json', candidate)
    shutil.copy2(SWEEP / 'calibration.json', root / 'calibration.json')

    cases = []
    for index, seq in enumerate(args.sequences):
        order = ['baseline', 'candidate'] if index % 2 == 0 else ['candidate', 'baseline']
        cases += [(seq, label) for label in order]
    report = dict(state='running', binary_sha256=digest(binary), threshold=args.threshold,
                  urgent_spacing=args.urgent_spacing, flags=FLAGS,
                  config_sha256={l: digest(root / f'{l}_config.json') for l in ('baseline', 'candidate')},
                  order=cases, runs=[])
    summary = root / 'summary.json'
    save(summary, report)
    for seq, label in cases:
        run = root / f'{seq}_{label}'
        run.mkdir()
        dataset = Path(DATASETS[seq])
        command = [str(binary), '--euroc-dir', str(dataset), '--calibration', str(root / 'calibration.json'),
                   '--config', str(root / f'{label}_config.json'), '--out-dir', str(run), *FLAGS]
        row = dict(sequence=seq, label=label, command=command, state='running')
        report['runs'].append(row)
        save(summary, report)
        print('START', run.name, flush=True)
        with (run / 'stdout.log').open('x') as out, (run / 'stderr.log').open('x') as err:
            result = subprocess.run([sys.executable, str(SCRIPTS / 'run_bounded_process.py'), '--seconds',
                                     str(args.per_run_seconds), '--', '/usr/bin/time', '-f', '%e %U %S %M %x',
                                     '-o', str(run / 'resources.txt'), *command], stdout=out, stderr=err)
        row['exit_code'] = result.returncode
        if result.returncode:
            row['state'] = 'timeout' if result.returncode == 124 else 'failed'
            report['state'] = 'stopped'
            save(summary, report)
            print('FAIL', run.name, row['state'], flush=True)
            return 1
        subprocess.run([sys.executable, str(SCRIPTS / 'evaluate_euroc_trajectory.py'),
                        '--ground-truth-csv', str(dataset / 'mav0/state_groundtruth_estimate0/data.csv'),
                        '--trajectory', str(run / 'trajectory_online.tum'), '--tum-time-unit', 's',
                        '--out-json', str(run / 'evaluation.json')], check=True, timeout=120,
                       stdout=subprocess.DEVNULL)
        evaluation = json.loads((run / 'evaluation.json').read_text())['runs'][0]
        wall, user, system, rss, _ = (run / 'resources.txt').read_text().split()
        keyframes = sum(1 for line in (run / 'trajectory_online_kf.tum').read_text().splitlines()
                        if line.strip() and not line.startswith('#'))
        row.update(state='ok', ate_se3_m=evaluation['ate_translation_se3_m']['rmse'],
                   rpe_consecutive_m=evaluation['rpe_translation_consecutive_m'],
                   estimate_poses=evaluation['estimate_poses'],
                   associated_poses=evaluation['associated_poses'],
                   wall_seconds=float(wall), cpu_seconds=round(float(user) + float(system), 2),
                   peak_rss_kib=int(rss), mapper_keyframes=keyframes)
        save(summary, report)
        print('PASS', run.name, 'ATE=%.5f wall=%.1f rss=%d kf=%d' % (
            row['ate_se3_m'], row['wall_seconds'], row['peak_rss_kib'], keyframes), flush=True)
    report['state'] = 'finished'
    report['results'] = []
    for seq in args.sequences:
        a, b = (next(r for r in report['runs'] if r['sequence'] == seq and r['label'] == l)
                for l in ('baseline', 'candidate'))
        report['results'].append(dict(sequence=seq, metrics={
            k: dict(baseline=a[k], candidate=b[k], ratio=b[k] / a[k])
            for k in ('ate_se3_m', 'wall_seconds', 'cpu_seconds', 'peak_rss_kib', 'mapper_keyframes')}))
    save(summary, report)
    print('FINISHED', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
