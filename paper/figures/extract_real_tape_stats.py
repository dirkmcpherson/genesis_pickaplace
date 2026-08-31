#!/usr/bin/env python3
"""Same per-tape statistics as extract_tape_stats.py, computed on the REAL in-the-wild
command tapes (inthewild_trials/<uid>_episodes.npy: 60 Hz joint-position targets).
EEF path = FK of the commanded joints through the same URDF the sim uses (kinematic,
commanded — no dynamics; the real arm's executed path lives only in the bags).
Writes real_tape_stats.csv + real_tape_paths.npz (20-pt arc-length resample).
"""
import os, csv, sys
os.environ.setdefault('PYOPENGL_PLATFORM', 'egl')
import pathlib as pl
import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              '/cluster/tufts/shortlab/jstale02/genesis_pickaplace'))
sys.path.insert(0, str(REPO))
import genesis as gs
import torch
from kinova import JOINT_NAMES, EEF_NAME
from kinova import (trials_position_0_successful as p0s, trials_position_0_failed as p0f,
                    trials_position_1_successful as p1s, trials_position_1_failed as p1f,
                    trials_position_2_successful as p2s, trials_position_2_failed as p2f)


def np_(x):
    return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)


def label_of(uid):
    for p, lst in ((0, p0s), (1, p1s), (2, p2s)):
        if uid in lst:
            return p, 'success'
    for p, lst in ((0, p0f), (1, p1f), (2, p2f)):
        if uid in lst:
            return p, 'fail'
    return None, 'unlisted'


gs.init(backend=gs.cpu, seed=0, precision="32", logging_level="warning")
scene = gs.Scene(show_viewer=False)
kinova = scene.add_entity(gs.morphs.URDF(file=str(REPO / 'gen3_lite_2f_robotiq_85.urdf'),
                                         fixed=True, pos=(0.0, 0.0, 0.05)))
kdofs = [kinova.get_joint(n).dof_idx_local for n in JOINT_NAMES]
eef = kinova.get_link(EEF_NAME)
scene.build()

K, HZ, STRIDE = 20, 60.0, 2   # stride-2 FK (30 Hz effective) — plenty for path geometry
rows, paths, keys = [], [], []
uids = sorted(int(p.name.split('_')[0]) for p in (REPO / 'inthewild_trials').glob('*_episodes.npy'))
for uid in uids:
    pos, lab = label_of(uid)
    if lab == 'unlisted':
        continue
    d = np.load(REPO / 'inthewild_trials' / f'{uid}_episodes.npy', allow_pickle=True).item()
    vel, gp = np.asarray(d['vel_cmd']), np.asarray(d['gripper_pos']).reshape(-1)
    if len(vel) < 10:
        rows.append(dict(uid=uid, pos=pos, label=lab, n_cmds=len(vel), stub=1)); continue
    idx = np.arange(0, len(vel), STRIDE)
    ez = np.zeros((len(idx), 3))
    for j, i in enumerate(idx):
        motor = (100 - gp[i]) / 100
        kinova.set_dofs_position(np.concatenate([vel[i], [-motor, motor, -0.5, -0.5]]), kdofs)
        ez[j] = np_(eef.get_pos())
    steps = np.linalg.norm(np.diff(ez, axis=0), axis=1)
    path_len = float(steps.sum()); net = float(np.linalg.norm(ez[-1] - ez[0]))
    a, b = ez[0], ez[-1]; ab = b - a; L2 = (ab @ ab) or 1e-12
    t = np.clip(((ez - a) @ ab) / L2, 0, 1)[:, None]
    wander = float(np.linalg.norm(ez - (a + t * ab), axis=1).mean())
    # idle: commanded joint targets unchanged between consecutive 60 Hz frames
    dj = np.abs(np.diff(np.asarray(d['vel_cmd']), axis=0)).max(axis=1)
    idle = float((dj < 1e-3).mean())
    rows.append(dict(uid=uid, pos=pos, label=lab, n_cmds=len(vel), stub=0,
                     dur_s=round(len(vel) / HZ, 1), path_len=round(path_len, 4),
                     net_disp=round(net, 4),
                     tortuosity=round(path_len / net if net > 1e-6 else np.nan, 3),
                     idle_frac=round(idle, 4), wander=round(wander, 4),
                     eef_std_x=round(float(ez[:, 0].std()), 4),
                     eef_std_y=round(float(ez[:, 1].std()), 4),
                     eef_std_z=round(float(ez[:, 2].std()), 4)))
    s = np.concatenate([[0], np.cumsum(steps)]); s /= (s[-1] or 1)
    tq = np.linspace(0, 1, K)
    paths.append(np.stack([np.interp(tq, s, ez[:, i]) for i in range(3)], axis=1))
    keys.append(f'real|{lab}|{pos}|{uid}')
    print(uid, lab, len(vel), round(path_len, 3), flush=True)

out = pl.Path(sys.argv[1] if len(sys.argv) > 1 else '/tmp')
fields = sorted({k for r in rows for k in r})
with open(out / 'real_tape_stats.csv', 'w', newline='') as fh:
    w = csv.DictWriter(fh, fieldnames=fields); w.writeheader(); w.writerows(rows)
np.savez_compressed(out / 'real_tape_paths.npz', paths=np.asarray(paths), keys=np.asarray(keys))
print(f'wrote {len(rows)} rows ({len(paths)} with paths)')
