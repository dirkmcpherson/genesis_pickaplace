"""Standalone verification for FullTaskEnv scope='place' (2026-08-02).

Two checks, one process (one Genesis world):
  (a) reset N random banked entries -> report the entry-restore survival rate
      (can still held after the 20-step settle).
  (b) open-loop replay: reset to a specific uid's entry state, then feed the
      ORIGINAL demo's actions from the entry frame onward (episodes_all/<uid>.npz,
      normalized to the env's [-1,1]^7 convention) and report whether PLACED_V2
      fires. Validates that the banked entry states are ACTIONABLE, not just
      restorable. Replay is open-loop through the policy-facing float32/clip path,
      not the bit-exact raw-command path, so some divergence is expected --
      per-uid results are reported honestly.

Usage (r2dreamer venv works):
  MUJOCO_GL=egl GENESIS_PICKAPLACE_ROOT=... python verify_place_scope.py \
      [--resets 10] [--replay-uids 233 242 259] [--seed 0]
"""
import os
import argparse
import pathlib as pl
import sys

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))

import pick_env  # noqa: E402
from full_env import FullTaskEnv  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--resets', type=int, default=10)
ap.add_argument('--replay-uids', type=int, nargs='*', default=[233, 242, 262],
                help='defaults are demos whose RECORDED frames reach placed_v2 '
                     '(233/242 human-validated, 262 shortest tail)')
ap.add_argument('--src', default=str(REPO / 'baselines' / 'episodes_all'))
ap.add_argument('--seed', type=int, default=0)
args = ap.parse_args()

env = FullTaskEnv(backend='cpu', scope='place', max_steps=1150)
env.reset(seed=args.seed)

# --- (a) survival over N random entry restores --------------------------------
print(f'\n=== (a) survival over {args.resets} random entry restores ===')
for i in range(args.resets):
    _obs, info = env.reset()
    print(f'  reset {i}: uid {info["uid"]} entry_frame {info["entry_frame"]}')
print(f'SURVIVAL: {env.place_survived}/{env.place_attempts} '
      f'({env.place_survived / max(env.place_attempts, 1):.2f})')

# --- (b) open-loop replay of the original post-entry actions ------------------
print(f'\n=== (b) open-loop post-entry replay, uids {args.replay_uids} ===')
results = {}
for uid in args.replay_uids:
    d = np.load(pl.Path(args.src) / f'{uid}.npz', allow_pickle=True)
    a_raw = d['actions'].astype(np.float32)
    k = env._entry_bank[uid]['frame']
    try:
        _obs, info = env.reset(options={'uid': uid})
    except RuntimeError as ex:
        results[uid] = f'entry did not survive restore ({ex})'
        print(f'  uid {uid}: RESTORE FAILED')
        continue
    outcome = 'actions exhausted (no placed_v2)'
    for t, a in enumerate(a_raw[k:]):
        _s, r, term, trunc, sinfo = env.step(pick_env.normalize_action(a))
        if sinfo.get('placed_v2'):
            outcome = f'PLACED_V2 at step {t} (reward {r})'
            break
        if term:
            which = 'tipped' if sinfo.get('tipped') else 'terminated'
            outcome = f'{which} at step {t}'
            break
        if trunc:
            outcome = f'truncated at step {t}'
            break
    results[uid] = outcome
    print(f'  uid {uid}: entry frame {k}, tail {len(a_raw) - k} actions -> {outcome}')

ok = sum('PLACED_V2' in v for v in results.values())
print(f'\nREPLAY CHECK: {ok}/{len(args.replay_uids)} reached placed_v2')
for u, v in results.items():
    print(f'  {u}: {v}')
