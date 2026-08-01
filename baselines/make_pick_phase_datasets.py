"""PICK-PHASE core datasets + the place-phase entry-state bank (2026-08-01).

The paper's controlled comparison is phase 1 (pick). Fairness rule: BOTH demo
sources truncate at the pick grant with the SAME rule -- model demos already stop
there (harvest --scope pick); this builds the human equivalent:

  baselines/episodes_pick_phase/<uid>.npz   states/actions cut at the grant frame
  baselines/pick_entry_states.json          the world state AT each grant -- arm
                                            qpos, grip, can pose, goal -- so place
                                            specialists can later reset INTO a
                                            picked state (user: "new models that
                                            start from a picked state")

Grant rule (identical to to_dreamer_demos/relabel_full): first frame with
can_z > PICK_Z and grip command > GRIP_CLOSED_FRAC.

Usage: make_pick_phase_datasets.py [--src baselines/episodes_all]
"""
import os
import argparse
import glob
import json
import pathlib as pl
import sys

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))
import pick_env  # noqa: E402

PICK_Z = 0.1505                       # FullTaskEnv.pick_z under the current world

ap = argparse.ArgumentParser()
ap.add_argument('--src', default='baselines/episodes_all')
ap.add_argument('--outdir', default='baselines/episodes_pick_phase')
ap.add_argument('--bank', default='baselines/pick_entry_states.json')
args = ap.parse_args()

OUT = REPO / args.outdir
OUT.mkdir(parents=True, exist_ok=True)
bank = {}
n_out = n_skip = 0
for p in sorted(glob.glob(str(REPO / args.src / '*.npz'))):
    d = np.load(p, allow_pickle=True)
    if str(d['label']) != 'success' or str(d['stage']) == 'no-pick':
        continue
    s, a = d['states'].astype(np.float32), d['actions'].astype(np.float32)
    grant = np.flatnonzero((s[:, 10] > PICK_Z) & (a[:, 6] > pick_env.GRIP_CLOSED_FRAC))
    if not len(grant):
        n_skip += 1
        print(f'{d["uid"]}: SKIP -- proxy never fires (stage {d["stage"]})')
        continue
    k = int(grant[0])
    # keep ONE frame past the grant (matches harvest episodes): relabel_full scans
    # frames [:-1], so an episode ending AT the grant loses it (0 rewarded -- caught
    # by the SACfD smoke 2026-08-01)
    end = min(k + 2, len(s))
    np.savez_compressed(OUT / pl.Path(p).name,
                        states=s[:end], actions=a[:end], uid=int(d['uid']),
                        n=end, label='success', stage='picked')
    bank[int(d['uid'])] = dict(
        frame=k,
        qpos=[float(x) for x in s[k, :6]],
        grip_cmd=float(a[k, 6]),
        grip_obs=float(s[k, 6]),
        can_pos=[float(x) for x in s[k, 8:11]],
        can_quat=[float(x) for x in s[k, 11:15]],
        goal_xy=[float(x) for x in s[k, 15:17]])
    n_out += 1

(REPO / args.bank).write_text(json.dumps(bank, indent=1))
lens = [np.load(f)['n'] for f in glob.glob(str(OUT / '*.npz'))]
print(f'{n_out} pick-phase demos -> {OUT} ({n_skip} skipped); '
      f'len min/med/max {int(min(lens))}/{int(np.median(lens))}/{int(max(lens))}')
print(f'{len(bank)} entry states -> {args.bank}')
