"""Collect sim rollouts of ALL demos for the SACfD replay buffer -- KEEP EVERYTHING,
no success filter. Sparse-reward off-policy RL learns from failures too: they broaden
the pick-state distribution and give the critic negative examples. Includes success-
AND fail-labeled demos; failed sim rollouts contribute r=0 coverage, and fail-labeled
demos that DO pick become positive pick examples under reward relabeling.

Saves per trial (baselines/episodes_raw_rl/<uid>.npz): states (n,17), actions (n,7),
picked_at (int, -1 if never), uid, n. demo_buffer reward-relabels for pick.

Usage: python baselines/rl/collect_all_rl.py [--uids ...]
"""
import argparse, json, sys, pathlib as pl
import numpy as np
REPO = pl.Path('/home/j/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'can_pos_recovery'))
from genesis_can_env import GenesisCanEnv
from replay_harness import load_episode
import torch
def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)

ap = argparse.ArgumentParser()
ap.add_argument('--uids', type=int, nargs='*', default=None)
args = ap.parse_args()
OUT = REPO / 'baselines/episodes_raw_rl'; OUT.mkdir(exist_ok=True, parents=True)

tbl = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())
placements = {int(u): r for u, r in tbl['trials'].items()}
fk = {int(k): v for k, v in json.loads((REPO / 'can_pos_recovery/fk_recovered.json').read_text()).items()}
env = GenesisCanEnv(backend='cpu')
CANZ = env.w['can_start_z']; GOALZ = env.w['goal_start_z']; STATIC_GOAL = (0.6, -0.2, GOALZ)

# ALL demos with a real trajectory: every labeled trial except the two stubs (no grasp)
uids = args.uids or sorted(int(u) for u in placements if u not in ('290', '322'))

n_ok = n_pick = 0
for uid in uids:
    f = OUT / f'{uid}.npz'
    if f.exists(): n_ok += 1; continue
    r = placements[uid]
    if r['status'] in ('ok', 'ok_batch'):
        can_pos = tuple(r['can_pos']); can_quat = r.get('can_quat') or [1, 0, 0, 0]
    elif uid in fk and fk[uid].get('can_xy'):
        cx, cy = fk[uid]['can_xy']; can_pos = (cx, cy, CANZ); can_quat = [1, 0, 0, 0]
    else:
        b = {0: (0.4381, 0.1), 1: (0.4381, -0.05), 2: (0.4381, -0.2)}.get(fk.get(uid, {}).get('pos'), (0.4381, -0.05))
        can_pos = (b[0], b[1], CANZ); can_quat = [1, 0, 0, 0]
    vel, gp = load_episode(uid)
    obs = env.reset(can_pos=can_pos, can_quat=can_quat, goal_pos=STATIC_GOAL)
    states, actions = [], []; picked_at = -1
    for i in range(len(vel) - 1):
        a = np.concatenate([vel[i], [np.clip(gp[i] / 100.0, 0, 1)]])
        states.append(obs['state']); actions.append(a.astype(np.float32))
        obs, done, info = env.step(a)
        if picked_at < 0 and info['picked']:
            picked_at = len(states)
    np.savez_compressed(f, states=np.array(states), actions=np.array(actions),
                        picked_at=picked_at, uid=uid, n=len(states),
                        label=r.get('label', '?'))
    n_ok += 1; n_pick += (picked_at >= 0)
    print(f"{uid}({r.get('label','?')[:4]}): {len(states)} frames, "
          f"{'PICK@'+str(picked_at) if picked_at>=0 else 'no-pick'}", flush=True)
print(f"\ncollected {n_ok} trials, {n_pick} achieved pick in sim -> {OUT}")
