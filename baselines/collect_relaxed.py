"""RELAXED-keep collection (v5): keep any rollout that PLACES the can on the shelf
UPRIGHT (tilt < 20 deg), regardless of the final slide-to-goal. Runs over ALL 75 legit
success demos -- the 34 already-collected PLUS the 16 flaky + 25 replay-unsolved that
full-task collection dropped. The criterion doesn't involve the goal, so this sidesteps
goal relocation entirely: spawn can at its recovered position, goal at the true static
spot, keep on place+upright, truncate there.

Rationale: pick is the DP bottleneck and downstream is nearly solved; we want lots of
pick+carry+place demonstrations, not only full-task successes. This recovers sim-
consistent data (real sim rollouts, no train/eval domain gap) for the dropped demos.

17-dim obs (matches current env incl grip_effort). Usage:
  python baselines/collect_relaxed.py --outdir baselines/episodes_raw_v5 [--uids ...]
"""
import argparse, json, sys, pathlib as pl
import numpy as np
REPO = pl.Path('/home/j/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'can_pos_recovery'))
from genesis_can_env import GenesisCanEnv
from replay_harness import load_episode, tilt_deg
import torch
def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)

ap = argparse.ArgumentParser()
ap.add_argument('--outdir', default='baselines/episodes_raw_v5')
ap.add_argument('--uids', type=int, nargs='*', default=None)
ap.add_argument('--attempts', type=int, default=8)
args = ap.parse_args()
OUT = REPO / args.outdir; OUT.mkdir(exist_ok=True, parents=True)

tbl = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())
placements = {int(u): r for u, r in tbl['trials'].items()}
fk = {int(k): v for k, v in json.loads((REPO / 'can_pos_recovery/fk_recovered.json').read_text()).items()}

env = GenesisCanEnv(backend='cpu')
CANZ = env.w['can_start_z']; GOALZ = env.w['goal_start_z']
STATIC_GOAL = (0.6, -0.2, GOALZ)   # true fixed goal for the obs (criterion ignores it)

# all 75 legit success demos (drop stubs 290/322)
uids = args.uids or sorted(int(u) for u, r in placements.items()
                           if r.get('label') == 'success' and u not in ('290', '322'))
UPRIGHT = 20.0        # deg tilt max ("without falling over")
HOLD = 15             # frames the can must stay placed+upright after first place

kept = skipped = 0
for uid in uids:
    f = OUT / f'{uid}.npz'
    if f.exists(): kept += 1; continue
    # spawn: solved -> its recovered can_pos; unsolved -> FK can_xy
    r = placements[uid]
    if r['status'] in ('ok', 'ok_batch'):
        can_pos = tuple(r['can_pos']); can_quat = r.get('can_quat') or [1, 0, 0, 0]
    else:
        cx, cy = fk[uid]['can_xy']; can_pos = (cx, cy, CANZ); can_quat = [1, 0, 0, 0]
    vel, gp = load_episode(uid)
    good = False
    for attempt in range(args.attempts):
        obs = env.reset(can_pos=can_pos, can_quat=can_quat, goal_pos=STATIC_GOAL)
        states, actions = [], []
        placed_upright_run = 0; keep_upto = -1
        for i in range(len(vel) - 1):
            a = np.concatenate([vel[i], [np.clip(gp[i] / 100.0, 0, 1)]])  # executed cmd
            states.append(obs['state']); actions.append(a.astype(np.float32))
            obs, done, info = env.step(a)
            tilt = tilt_deg(np_(env.w['bottle'].get_quat()))
            if info['placed'] and tilt < UPRIGHT:
                placed_upright_run += 1
                if placed_upright_run == 1: keep_upto = len(states)   # truncate at place
                if placed_upright_run >= HOLD:
                    good = True; break
            else:
                placed_upright_run = 0
        if good:
            # keep the trajectory UP TO the place (+ the short upright hold)
            end = min(keep_upto + HOLD, len(states))
            np.savez_compressed(f, states=np.array(states[:end]),
                                actions=np.array(actions[:end]), uid=uid, n=end)
            print(f"{uid}: kept ({end} frames, {'solved' if r['status'] in ('ok','ok_batch') else 'RESCUED'})", flush=True)
            kept += 1
            break
    if not good:
        print(f"{uid}: no place-upright in {args.attempts} attempts, SKIPPED", flush=True)
        skipped += 1
print(f"\nrelaxed collection: kept {kept}, skipped {skipped} -> {OUT}")
