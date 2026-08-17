"""Carve PLACE-PHASE demo segments + a matched post-pick IC bank.

Mirrors make_pick_phase_datasets.py one stage later. Each output episode runs
from the demo's PICK GRANT to its first PLACED_V2 grant (+1 frame, the same
off-by-one the pick carver documents: relabel scans frames[:-1], so an episode
ending AT the grant loses it).

  baselines/episodes_place_phase/<uid>.npz       success segments (BC-safe)
  baselines/episodes_place_phase_all/<uid>.npz   + zero-reward negatives (RLfD)
  baselines/place_entry_states.json              world state AT each pick grant,
                                                 one entry per carved uid

SOURCE MUST BE episodes_delta_rerecord (default), NOT episodes_all.
P1 (paper/p1_delta_divergence_2026-08-13.md): re-ENCODING the recorded tape into
the delta action space loses the downstream phases -- open-loop delta replay
scored contact 5 / nested 1 against labels 25/16, because one clipped frame
leaves a permanent offset in a target-referenced integrator. The closed-loop
RE-RECORD recovered them (28/20). Place lives entirely downstream of the pick,
so a place dataset built from a re-encoded tape would be exactly the corrupted
signal P1 documents. The re-record tapes are measured-referenced and carry their
own encoding stamp (delta_ref/delta_scale) -- verified and propagated here.

PLACED_V2 is evaluated OFFLINE with the env's OWN predicate parts, imported not
re-implemented (tilt_deg / in_shelf_footprint / BOX_TOP_Z from replay_harness,
constants read off FullTaskEnv). The grip-column bug family lived in
re-implemented predicate math; importing closes that class by construction.

The pick-grant frame comes from the demo itself (same proxy as the pick carver:
can above PICK_Z with the grip commanded closed), NOT from the 54-uid
pick_entry_states.json -- that bank predates the re-record set and misses uids,
which cost ~5 segments in the feasibility scan.

Usage:
  make_place_phase_datasets.py                     # defaults, writes everything
  make_place_phase_datasets.py --dry-run           # census only, no writes
"""
import os
import argparse
import glob
import json
import pathlib as pl
import shutil
import sys

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))
sys.path.insert(0, str(REPO / 'can_pos_recovery'))

from replay_harness import tilt_deg, in_shelf_footprint, BOX_TOP_Z  # noqa: E402
import pick_env  # noqa: E402
from full_env import FullTaskEnv as F  # noqa: E402  (constants only, no env built)

PICK_Z = float(pick_env.PICK_Z) if hasattr(pick_env, 'PICK_Z') else 0.16
RELEASE = F.PLACE_RELEASE          # 0.45
TILT = F.PLACE_TILT_DEG            # 20.0
SUSTAIN = F.PLACE_SUSTAIN          # 10

ap = argparse.ArgumentParser()
ap.add_argument('--src', default='baselines/episodes_delta_rerecord')
ap.add_argument('--outdir', default='baselines/episodes_place_phase')
ap.add_argument('--bank', default='baselines/place_entry_states.json')
ap.add_argument('--dry-run', action='store_true')
args = ap.parse_args()


def pick_grant(s, a):
    """First frame where the can is lifted AND the grip is commanded closed."""
    g = np.flatnonzero((s[:, 10] > PICK_Z) & (a[:, 6] > pick_env.GRIP_CLOSED_FRAC))
    return int(g[0]) if len(g) else None


def placed_v2_grant(s, a, after):
    """First frame at which the release predicate has held SUSTAIN frames."""
    run = 0
    for i in range(min(len(s), len(a))):
        ok = (float(a[i, 6]) < RELEASE
              and in_shelf_footprint(s[i, 8:11])
              and BOX_TOP_Z + 0.01 < float(s[i, 10]) < BOX_TOP_Z + 0.07
              and tilt_deg(s[i, 11:15]) < TILT)
        run = run + 1 if ok else 0
        if run >= SUSTAIN and i > after:
            return i
    return None


OUT = REPO / args.outdir
ALL = REPO / (args.outdir + '_all')
bank, recs, fails = {}, [], []
for p in sorted(glob.glob(str(REPO / args.src / '*.npz')),
                key=lambda q: int(pl.Path(q).stem)):
    d = np.load(p, allow_pickle=True)
    uid = int(d['uid'])
    s = d['states'].astype(np.float32)
    a = d['actions'].astype(np.float32)
    stage = str(d['stage']) if 'stage' in d.files else '?'
    label = str(d['label']) if 'label' in d.files else '?'
    k = pick_grant(s, a)
    if k is None:
        fails.append((uid, 'no-pick-grant', stage))
        continue
    j = placed_v2_grant(s, a, k)
    if j is None:
        fails.append((uid, 'no-placed_v2', stage))
        continue
    end = min(j + 2, len(s))                      # keep one frame past the grant
    recs.append(dict(uid=uid, start=k, end=end, n=end - k, src_stage=stage,
                     label=label))
    bank[uid] = dict(
        frame=k,
        qpos=[float(x) for x in s[k, :6]],
        grip_cmd=float(a[k, 6]),
        grip_obs=float(s[k, 6]),
        can_pos=[float(x) for x in s[k, 8:11]],
        can_quat=[float(x) for x in s[k, 11:15]],
        goal_xy=[float(x) for x in s[k, 15:17]])

L = np.array([r['n'] for r in recs]) if recs else np.array([0])
print(f'src={args.src}')
print(f'carved: {len(recs)} place segments | skipped {len(fails)}')
print(f'length: median {np.median(L):.0f} min {L.min()} max {L.max()} total {L.sum()}')
print(f'reward density (1 terminal/segment): {100*len(recs)/max(L.sum(),1):.3f}%')
from collections import Counter
print('skips:', Counter(f[1] for f in fails).most_common())
print('src stages carved:', Counter(r['src_stage'] for r in recs).most_common())
if args.dry_run:
    print('DRY RUN -- nothing written')
    raise SystemExit(0)

OUT.mkdir(parents=True, exist_ok=True)
ALL.mkdir(parents=True, exist_ok=True)
stamp_keys = ('delta_ref', 'delta_scale', 'action_repeat')
for r in recs:
    p = REPO / args.src / f"{r['uid']}.npz"
    d = np.load(p, allow_pickle=True)
    s = d['states'].astype(np.float32)[r['start']:r['end']]
    a = d['actions'].astype(np.float32)[r['start']:r['end']]
    pay = dict(states=s, actions=a, uid=r['uid'], n=len(s), label='success',
               stage='placed_v2', src_start=r['start'], src_end=r['end'])
    for k_ in stamp_keys:                          # propagate encoding semantics
        if k_ in d.files:
            pay[k_] = d[k_]
    if 'actions_delta' in d.files:
        pay['actions_delta'] = d['actions_delta'].astype(np.float32)[r['start']:r['end']]
    np.savez_compressed(OUT / f"{r['uid']}.npz", **pay)

# ALL set: successes + zero-reward negatives. Negatives = demos WITH a pick grant
# but no placed_v2 (they carry honest failed-place dynamics from the same IC
# distribution). Demos with no pick grant are excluded -- they never enter the
# place scope's state distribution at all, so they are not place negatives.
for f in glob.glob(str(OUT / '*.npz')):
    shutil.copy(f, ALL / pl.Path(f).name)
n_neg = 0
for uid, why, stage in fails:
    if why != 'no-placed_v2':
        continue
    d = np.load(REPO / args.src / f'{uid}.npz', allow_pickle=True)
    s = d['states'].astype(np.float32)
    a = d['actions'].astype(np.float32)
    k = pick_grant(s, a)
    end = min(k + 1200, len(s))
    pay = dict(states=s[k:end], actions=a[k:end], uid=uid, n=end - k,
               label='fail', stage='no-place', src_start=k, src_end=end)
    for k_ in stamp_keys:
        if k_ in d.files:
            pay[k_] = d[k_]
    np.savez_compressed(ALL / f'{uid}.npz', **pay)
    n_neg += 1

(REPO / args.bank).write_text(json.dumps(bank, indent=1))
print(f'\nwrote {len(recs)} -> {OUT}')
print(f'wrote {len(recs)} success + {n_neg} negatives -> {ALL}')
print(f'wrote {len(bank)}-uid post-pick IC bank -> {args.bank}')
