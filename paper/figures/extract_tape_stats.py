#!/usr/bin/env python3
"""Per-tape numeric statistics for the four matched demo sets (world x source).
Writes tape_stats.csv (one row per tape) and tape_paths.npz (20-point resampled
EEF path per tape) for local plotting. No images are read (npz members are lazy).
"""
import csv, sys
import numpy as np
import pathlib as pl

ROOT = pl.Path('/cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines')
SETS = [
    ('old', 'dH',  ROOT / 'matched_v2' / 'dH'),
    ('old', 'dDP', ROOT / 'matched_v2' / 'dDP'),
    ('w3',  'dH',  ROOT / 'matched_w3' / 'dH'),
    ('w3',  'dDP', ROOT / 'matched_w3' / 'dDP'),
]
K = 20  # resample points

rows, paths, keys = [], [], []
for world, arm, d in SETS:
    files = sorted(d.glob('*.npz'))
    print(f'{world}/{arm}: {len(files)} tapes', flush=True)
    for f in files:
        z = np.load(f, allow_pickle=True)
        eef = z['eef_pos'].astype(np.float64)          # (T+1, 3)
        act = z['actions_delta'].astype(np.float64)    # (T, 7)
        steps = np.linalg.norm(np.diff(eef, axis=0), axis=1)
        path_len = float(steps.sum())
        net = float(np.linalg.norm(eef[-1] - eef[0]))
        tort = path_len / net if net > 1e-6 else np.nan
        idle = float((np.abs(act).max(axis=1) < 1e-3).mean())
        # wander: mean distance from the straight chord start->end
        a, b = eef[0], eef[-1]; ab = b - a; L2 = (ab @ ab) or 1e-12
        t = np.clip(((eef - a) @ ab) / L2, 0, 1)[:, None]
        wander = float(np.linalg.norm(eef - (a + t * ab), axis=1).mean())
        rows.append(dict(world=world, arm=arm, uid=int(z['uid']), ic_uid=int(z['ic_uid']),
                         stage=str(z['stage']), n_rows=int(z['n']), path_len=round(path_len, 4),
                         net_disp=round(net, 4), tortuosity=round(tort, 3), idle_frac=round(idle, 4),
                         wander=round(wander, 4), mean_step=round(float(steps.mean()), 5),
                         eef_std_x=round(float(eef[:, 0].std()), 4), eef_std_y=round(float(eef[:, 1].std()), 4),
                         eef_std_z=round(float(eef[:, 2].std()), 4), reward_sum=round(float(z['rewards'].sum()), 2)))
        # 20-point arc-length-uniform resample of the EEF path
        s = np.concatenate([[0], np.cumsum(steps)]); s /= (s[-1] or 1)
        tq = np.linspace(0, 1, K)
        paths.append(np.stack([np.interp(tq, s, eef[:, i]) for i in range(3)], axis=1))
        keys.append(f'{world}|{arm}|{int(z["ic_uid"])}|{int(z["uid"])}')

out = pl.Path(sys.argv[1] if len(sys.argv) > 1 else '/tmp')
with open(out / 'tape_stats.csv', 'w', newline='') as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
np.savez_compressed(out / 'tape_paths.npz', paths=np.asarray(paths), keys=np.asarray(keys))
print(f'wrote {len(rows)} rows -> {out}/tape_stats.csv, tape_paths.npz')
