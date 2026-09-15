#!/usr/bin/env python3
"""Build the portable M8e Rust BA input from the pinned oracle artifacts.

The C++ oracle consumes cereal and the binary M8d setup stream.  This export
keeps the same setup-selected 549 landmarks, all 5687 feature observations,
the merged 83-pose set, and the M8a factor measurements in JSON so the Rust
solver can be exercised without a cereal reader.
"""
import argparse, json
from pathlib import Path

COMMIT = "0f3b2b52c807f70ff4e2973ce253c73329eea7bc"

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--features", type=Path, default=Path("target/m8c_feature_raw20_final.json"))
    ap.add_argument("--tracks", type=Path, default=Path("benchmarks/basalt/m8d_track_oracle20.json"))
    ap.add_argument("--setup", type=Path, default=Path("target/m8d_setup_m7v80.oracle4.json"))
    ap.add_argument("--poses", type=Path, default=Path("target/m8d_setup_poses_m7v80.json"))
    ap.add_argument("--factors", type=Path, default=Path("benchmarks/basalt/m8a_m8b_mh01_1403636579763555584.json"))
    ap.add_argument("--factor-oracle", type=Path, help="JSON emitted by upstream_m8e_global_ba_oracle; supplies weighted cov_inv")
    ap.add_argument("--output", type=Path, required=True)
    a = ap.parse_args()
    features = json.loads(a.features.read_text())['features']
    pixels = {(int(x['time_cam_id']['frame_id']), int(x['time_cam_id']['cam_id'])):
              x['corners_xy'] for x in features}
    tracks = json.loads(a.tracks.read_text())['oracle']['canonical_observations']
    setup = {int(x['track_id']): x for x in json.loads(a.setup.read_text())['tracks'] if x.get('accepted')}
    landmarks = []
    for t in tracks:
        tid = int(t['id'])
        if tid not in setup: continue
        s = setup[tid]
        obs = []
        for o in t['observations']:
            image = {'frame_id': int(o['image']['frame_id']), 'cam_id': int(o['image']['cam_id'])}
            xy = pixels[(image['frame_id'], image['cam_id'])][int(o['feature_id'])]
            obs.append({'image': image, 'feature_id': int(o['feature_id']), 'pixel': [float(xy[0]), float(xy[1])]})
        landmarks.append({'track_id': tid, 'host': s['host'], 'second': s['second'],
                         'direction': s['direction'], 'inverse_distance': s['inverse_distance'],
                         'observations': obs})
    poses = json.loads(a.poses.read_text())['poses']
    # The setup stream contributes 80 poses; the MargData packet contributes
    # three endpoint-only poses.  Preserve the oracle's merged pose set.
    by_id = {int(p['frame_id']): p for p in poses}
    m = json.loads(a.factors.read_text())['output']
    oracle = json.loads(a.factor_oracle.read_text()) if a.factor_oracle else None
    weighted = oracle['factor_export'] if oracle else None
    installed_poses = {
        int(p['frame_id']): p
        for p in (oracle or {}).get('installed_poses', [])
    }
    setup_pose_ids = set(by_id)
    if installed_poses:
        # The setup stream is authoritative for its 80 IDs.  Only the
        # MargData-only factor endpoints are taken from the upstream
        # full-double export; the M8a JSON frame_poses are float-rounded.
        for frame_id, pose in installed_poses.items():
            if frame_id not in setup_pose_ids:
                by_id[frame_id] = {
                    'frame_id': frame_id,
                    'translation': pose['translation'],
                    'quaternion_xyzw': pose['quaternion_xyzw'],
                }
    weighted_rel = {(int(x['from']), int(x['to'])): x for x in weighted['relative_pose']} if weighted else {}
    weighted_rp = {int(x['frame_id']): x for x in weighted['roll_pitch']} if weighted else {}
    for p in m['frame_poses']:
        q = p['pose']; by_id.setdefault(int(p['id']), {'frame_id': int(p['id']), 'translation': q['translation'], 'quaternion_xyzw': q['quaternion_xyzw']})
    relative = []
    for f in m['factors']['relative_pose']:
        key=(int(f['t_i_ns']), int(f['t_j_ns']))
        wf=weighted_rel.get(key, {})
        relative.append({'from': key[0], 'to': key[1], 'translation': wf.get('translation', f['measurement_translation']), 'rotation': wf.get('rotation', f['measurement_quaternion_xyzw']), 'information': wf.get('information', f['information']), 'weight': 1.0})
    roll_pitch = []
    for f in m['factors']['roll_pitch']:
        r = f['measurement_rotation']
        fid=int(f['t_ns']); wr=weighted_rp.get(fid, {}); roll_pitch.append({'frame_id': fid, 'roll': 0.0, 'pitch': 0.0, 'information': wr.get('information', [f['information'][0][0], f['information'][0][1], f['information'][1][0], f['information'][1][1]]), 'weight': 1.0, 'measured_rotation': wr.get('measured_rotation', sum(r, []))})
    out = {'schema': 'basalt-m8e-rust-fixture-v2', 'upstream_commit': COMMIT,
           'poses': [by_id[k] for k in sorted(by_id)], 'landmarks': landmarks,
           'factors': {'provenance_version': 'm8a-m8e-pinned', 'relative_pose': relative, 'roll_pitch': roll_pitch, 'ba_covisibility': []},
           'counts': {'poses': len(by_id), 'landmarks': len(landmarks), 'observations': sum(len(x['observations']) for x in landmarks)}}
    a.output.write_text(json.dumps(out, separators=(',', ':')) + '\n')
    print(json.dumps(out['counts'], sort_keys=True))
if __name__ == '__main__': main()
