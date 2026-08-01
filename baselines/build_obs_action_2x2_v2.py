"""Rebuild the {observation} x {action} 2x2 from the PROVEN replay source.

v1 (build_obs_action_2x2.py) drew from episodes_cartesian_dual, which replays the
bags' joint_pos streams -- and that source ALONE drops joint DP 0.73 -> 0.07 (its
control cell failed on cluster AND in a local replication; July30th_Fable.md §2).

v2 draws from episodes_all_ee: the exact vel-cmd replay that produced the 0.67-0.80
positive control, re-collected with `--ee` so each frame also carries the 18-dim
ee-centric state (verified 91/91 bitwise-identical states/actions/stage to
episodes_all). Consequences, by construction:
  - jobs_jact == the positive control's data content (same frames, same actions)
  - the ee cells differ from it ONLY in representation
so cell differences are attributable to representation, which is the question.

    cell            observation                 action
    jobs_jact       states       joint (17)     actions        joint targets (7)
    jobs_eact       states       joint (17)     derived abs6   ee pose      (7)
    eobs_jact       states_ee    ee    (18)     actions        joint targets (7)
    eobs_eact       states_ee    ee    (18)     derived abs6   ee pose      (7)

abs6 derivation is identical to v1's audit-verified block (tool pose + wrist rotvec
relative to reset; action_i = pose reached at i+1); grip comes from actions[:,6].
Filter: label==success and stage>=picked (the 66-demo IL set).

Usage: build_obs_action_2x2_v2.py [--src baselines/episodes_all_ee]
                                  [--outroot baselines/x2x2v2]
"""
import os
import argparse
import glob
import pathlib as pl

import numpy as np
from scipy.spatial.transform import Rotation as R

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[1]))

ap = argparse.ArgumentParser()
ap.add_argument('--src', default='baselines/episodes_all_ee')
ap.add_argument('--outroot', default='baselines/x2x2v2')
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
    if str(d['label']) != 'success' or str(d['stage']) == 'no-pick':
        continue
    s_j = d['states'].astype(np.float32)           # (n,17) joint -- proven content
    s_ee = d['states_ee'].astype(np.float32)       # (n,18) ee-centric, same frames
    a_j = d['actions'].astype(np.float32)          # (n,7)  joint targets + grip
    grip = a_j[:, 6].astype(np.float64)

    # --- derive abs6 actions from the SAME frames (v1's audit-verified block) -----
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

    stage = str(d['stage'])
    common = dict(n=m, uid=int(d['uid']), label='success', stage=stage,
                  picked=stage != 'no-pick',
                  placed=stage in ('placed', 'contact', 'nested'),
                  contact=stage in ('contact', 'nested'))
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
