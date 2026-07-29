"""Derive REALIZED-velocity cartesian demos from episodes_cartesian (offline).

episodes_cartesian pairs joint-replay states with the human's COMMANDED velocities
-- BC-faithful, but dynamics-INCONSISTENT with CartesianCanEnv (open-loop census:
commanded execution picks in only 20/96 vs 61 joint replay). For RL the action
must be what MOVES the state: realized tool velocity, finite-differenced from the
recorded wrist pose (tool = wrist + R@offset, the env's own calibrated offset),
pitch rate from consecutive quats. Integrating these through the env's setpoint
reproduces the demonstrated trajectory by construction.

Caps: |v| clipped to VCAP (0.11), pitch to PITCH_CAP (1.0) -- realized motion obeys
the plugin caps up to replay noise, clip guards numeric spikes. Grip: recorded 0..1
command (identical realized/commanded).

Output: --outdir (default episodes_cartesian_realized), npz mirroring the source
(states/actions/n/uid/picked/placed/contact[/images passthrough]).
"""
import os
import argparse, glob, pathlib as pl
import numpy as np
from scipy.spatial.transform import Rotation as R

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', '/home/j/workspace/genesis_pickaplace'))

ap = argparse.ArgumentParser()
ap.add_argument('--src', default='baselines/episodes_cartesian')
ap.add_argument('--outdir', default='baselines/episodes_cartesian_realized')
ap.add_argument('--mode', choices=['velocity', 'delta', 'abs', 'abs6'], default='velocity',
                help='delta: actions are per-step tool-pose deltas (m, rad) for the '
                     'delta control mode -- self-correcting reference, BC-viable')
args = ap.parse_args()
OUT = REPO / args.outdir; OUT.mkdir(parents=True, exist_ok=True)

DT = 0.025
VCAP, PITCH_CAP = 0.11, 1.0
# wrist->tool offset in the wrist frame: the env calibrates it at reset from
# REF_TOOL_AT_START; reproduce that calibration from the first frame of any demo
# (constant geometry -- same for all).
REF_TOOL = np.array([0.367, 0.011, 0.09])


def gs_to_xyzw(q):
    q = np.asarray(q, float)
    return np.stack([q[..., 1], q[..., 2], q[..., 3], q[..., 0]], axis=-1)


offset_local = None
n_out = 0
for p in sorted(glob.glob(str(REPO / args.src / '*.npz'))):
    d = np.load(p, allow_pickle=True)
    s = d['states'].astype(np.float64)
    n = int(d['n'])
    ee, eq = s[:, 0:3], s[:, 3:7]
    rot = R.from_quat(gs_to_xyzw(eq))
    if offset_local is None:
        offset_local = rot[0].inv().apply(REF_TOOL - ee[0])
    tool = ee + rot.apply(np.broadcast_to(offset_local, (len(s), 3)))
    if args.mode == 'abs6':
        rel_all = (rot * rot[0].inv()).as_rotvec()      # (n,3) full wrist rotation
        act = np.concatenate([tool[1:], rel_all[1:],
                              np.clip(d['actions'][:, 4:5].astype(np.float64)[:len(tool)-1], 0, 1)],
                             axis=1).astype(np.float32)
        m = len(act)
        payload = {k: d[k] for k in d.files if k != 'actions'}
        payload['states'] = d['states'][:m + 1]
        payload['actions'] = act
        payload['n'] = m
        if 'images' in d.files:
            payload['images'] = d['images'][:m + 1]
        np.savez_compressed(OUT / pl.Path(p).name, **payload)
        n_out += 1
        continue
    if args.mode == 'abs':
        # absolute target = the pose the demo REACHES next step
        v = tool[1:]
        rel_all = (rot * rot[0].inv()).as_rotvec()[:, 1]
        wy = rel_all[1:]
        VC, PC = None, None
    elif args.mode == 'delta':
        v = np.diff(tool, axis=0)                                    # per-step deltas (m)
        rel = (rot[1:] * rot[:-1].inv()).as_rotvec()
        wy = rel[:, 1]                                               # per-step pitch delta
        VC, PC = 0.01, 0.075                                      # DCAP / DPITCH_CAP
    else:
        v = np.diff(tool, axis=0) / DT                               # (n-1, 3)
        # pitch rate about base Y from consecutive wrist quats
        rel = (rot[1:] * rot[:-1].inv()).as_rotvec()
        wy = rel[:, 1] / DT
        VC, PC = VCAP, PITCH_CAP
    grip = d['actions'][:, 4].astype(np.float64)                     # recorded 0..1
    m = len(v)                                                       # n-1 transitions
    if args.mode == 'abs6':
        rel_all = (rot * rot[0].inv()).as_rotvec()      # (n,3) full wrist rotation
        act = np.concatenate([tool[1:], rel_all[1:],
                              np.clip(d['actions'][:, 4:5].astype(np.float64)[:len(tool)-1], 0, 1)],
                             axis=1).astype(np.float32)
        m = len(act)
        payload = {k: d[k] for k in d.files if k != 'actions'}
        payload['states'] = d['states'][:m + 1]
        payload['actions'] = act
        payload['n'] = m
        if 'images' in d.files:
            payload['images'] = d['images'][:m + 1]
        np.savez_compressed(OUT / pl.Path(p).name, **payload)
        n_out += 1
        continue
    if args.mode == 'abs':
        act = np.stack([v[:, 0], v[:, 1], v[:, 2], wy,
                        np.clip(grip[:m], 0, 1)], axis=1).astype(np.float32)
    else:
        act = np.stack([np.clip(v[:, 0], -VC, VC),
                        np.clip(v[:, 1], -VC, VC),
                        np.clip(v[:, 2], -VC, VC),
                        np.clip(wy, -PC, PC),
                        np.clip(grip[:m], 0, 1)], axis=1).astype(np.float32)
    payload = {k: d[k] for k in d.files if k not in ('actions',)}
    payload['states'] = d['states'][:m + 1]
    payload['actions'] = act
    payload['n'] = m
    if 'images' in d.files:
        payload['images'] = d['images'][:m + 1]
    np.savez_compressed(OUT / pl.Path(p).name, **payload)
    n_out += 1
print(f'{n_out} realized-velocity episodes -> {OUT}')
