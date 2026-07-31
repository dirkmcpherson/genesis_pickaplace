"""Build the {observation} x {action} 2x2 that should settle why cartesian BC fails.

Joint DP reaches 0.67 in-distribution; every end-effector encoding reaches 0.00 --
but the cartesian conditions changed the OBSERVATION (18-dim ee-centric) at the same
time as the ACTION, and the two have never been varied independently.

All four cells are emitted from ONE source (episodes_cartesian_dual, which records
both representations for the same frames) with identical pruning, so the only
difference between cells is the representation itself:

    cell            observation            action
    jobs_jact       joint  (17)            joint targets (7)
    jobs_eact       joint  (17)            abs6 ee pose  (7)
    eobs_jact       ee     (18)            joint targets (7)
    eobs_eact       ee     (18)            abs6 ee pose  (7)

abs6 actions are derived here (tool pose + wrist rotvec relative to reset) rather
than reused from another directory, so every cell shares the same frames.

Usage: build_obs_action_2x2.py [--src baselines/episodes_cartesian_dual]
                               [--outroot baselines/x2x2] [--picked-only]
"""
# TIP-TRUNCATION DECISION (audit, 2026-07-31): BC datasets deliberately KEEP
# post-tip frames. The RL relabelers truncate at the first tipped-free frame
# (mirroring env termination) because TD bootstraps through those states; BC only
# imitates state->action pairs it is fed, never visits post-tip states at eval
# (the env terminates), and 67-demo BC is data-starved enough that dropping frames
# costs more than the off-distribution tail risks. Revisit if BC evals ever show
# can-down behaviors.
import os
import argparse
import glob
import pathlib as pl

import numpy as np
from scipy.spatial.transform import Rotation as R

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[1]))

ap = argparse.ArgumentParser()
ap.add_argument('--src', default='baselines/episodes_cartesian_dual')
ap.add_argument('--outroot', default='baselines/x2x2')
ap.add_argument('--picked-only', action='store_true', default=True)
args = ap.parse_args()

SRC = REPO / args.src
OUT = REPO / args.outroot
REF_TOOL = np.array([0.367, 0.011, 0.09])
CELLS = ('jobs_jact', 'jobs_eact', 'eobs_jact', 'eobs_eact')
for c in CELLS:
    (OUT / c).mkdir(parents=True, exist_ok=True)


def gs2xyzw(q):
    q = np.asarray(q, float)
    return np.stack([q[..., 1], q[..., 2], q[..., 3], q[..., 0]], axis=-1)


offset_local = None
n_out = 0
for p in sorted(glob.glob(str(SRC / '*.npz'))):
    d = np.load(p, allow_pickle=True)
    if args.picked_only and not bool(d['picked']):
        continue
    s_ee = d['states'].astype(np.float32)          # (n,18) ee-centric
    s_j = d['states_joint'].astype(np.float32)     # (n,17) joint
    a_j = d['actions_joint'].astype(np.float32)    # (n,7)  joint targets + grip
    grip = d['actions'][:, 4].astype(np.float64)   # recorded 0..1

    # --- derive abs6 actions from the SAME frames ---------------------------------
    ee, eq = s_ee[:, 0:3].astype(np.float64), s_ee[:, 3:7].astype(np.float64)
    rot = R.from_quat(gs2xyzw(eq))
    if offset_local is None:
        offset_local = rot[0].inv().apply(REF_TOOL - ee[0])
    tool = ee + rot.apply(np.broadcast_to(offset_local, (len(s_ee), 3)))
    rotvec = (rot * rot[0].inv()).as_rotvec()
    # action_i = the pose reached at i+1 (absolute target), so drop the last frame
    m = len(tool) - 1
    a_e = np.concatenate([tool[1:m + 1], rotvec[1:m + 1], grip[:m, None]],
                         axis=1).astype(np.float32)

    common = dict(n=m, uid=d['uid'], picked=d['picked'], placed=d['placed'],
                  contact=d['contact'],
                  label='success', stage=('contact' if bool(d['contact'])
                                          else 'placed' if bool(d['placed'])
                                          else 'picked'))
    name = pl.Path(p).name
    np.savez_compressed(OUT / 'jobs_jact' / name,
                        states=s_j[:m], actions=a_j[:m], **common)
    np.savez_compressed(OUT / 'jobs_eact' / name,
                        states=s_j[:m], actions=a_e[:m], **common)
    np.savez_compressed(OUT / 'eobs_jact' / name,
                        states=s_ee[:m], actions=a_j[:m], **common)
    np.savez_compressed(OUT / 'eobs_eact' / name,
                        states=s_ee[:m], actions=a_e[:m], **common)
    n_out += 1

print(f'{n_out} episodes -> 4 cells under {OUT}')
for c in CELLS:
    f = sorted(glob.glob(str(OUT / c / '*.npz')))
    if f:
        z = np.load(f[0])
        print(f'  {c:10s} states {z["states"].shape} actions {z["actions"].shape} '
              f'({len(f)} eps)')
print('PROPRIO for convert_to_lerobot: joint obs -> 8, ee obs -> 9')
