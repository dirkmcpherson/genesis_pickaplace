"""KEEP-EVERYTHING collection + authoritative classification, for the RL-on-everything vs
DP-on-success experiment. Replays EVERY non-stub demo in the training env at the corrected
goal, keeps the full trajectory regardless of outcome, and records the highest funnel stage
reached (no-pick < picked < placed < contact < nested) via the env's own predicates.

  solved demos (status ok/ok_batch): reset(uid=uid) -> validated can start pose.
  fk-seed demos (unsolved):          reset(can_pos=FK-guess, goal=corrected).

Output: <outdir>/<uid>.npz  (states, actions, uid, n, label, stage) for every demo, plus
appends to <outdir>/_stage.json a {uid: {label, source, stage, n}} record (merged across the
parallel workers by merge step). This dir is the RL "everything" dataset; the DP "successful
picks" subset is just the demos with stage in {picked,placed,contact,nested}.

Usage: collect_all_classified.py --outdir baselines/episodes_all [--uids ...]
"""
import argparse, json, sys, pathlib as pl
import numpy as np

REPO = pl.Path('/home/james/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'can_pos_recovery'))
from genesis_can_env import GenesisCanEnv
from replay_harness import load_episode, STATIC_BOTTLE_POSITION

ap = argparse.ArgumentParser()
ap.add_argument('--outdir', default='baselines/episodes_all')
ap.add_argument('--uids', type=int, nargs='*', default=None)
args = ap.parse_args()

OUT = REPO / args.outdir
OUT.mkdir(exist_ok=True, parents=True)
BUCKET = {0: (0.4381, 0.1), 1: (0.4381, -0.05), 2: (0.4381, -0.2), None: (0.4381, -0.05)}
STUB = {'290', '322'}

tbl = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())['trials']
fk = {int(k): v for k, v in json.loads((REPO / 'can_pos_recovery/fk_recovered.json').read_text()).items()}

env = GenesisCanEnv(backend='cpu')
CANZ = env.w['can_start_z']
GOAL = (STATIC_BOTTLE_POSITION[0], STATIC_BOTTLE_POSITION[1], env.w['goal_start_z'])

uids = args.uids or sorted(int(u) for u in tbl if u not in STUB)
recs = []
for uid in uids:
    try:
        r = tbl[str(uid)]
        label = r.get('label', '?'); status = r.get('status', '?')
        solved = status in ('ok', 'ok_batch')
        if solved:
            obs = env.reset(uid=uid)
            source = 'solved'
        else:
            f = fk.get(uid, {})
            seed = f.get('close_xy') or (f.get('can_xy') if f.get('conf') in ('HIGH', 'MED')
                                         else BUCKET[f.get('pos')])
            obs = env.reset(can_pos=(seed[0], seed[1], CANZ), can_quat=[1, 0, 0, 0], goal_pos=GOAL)
            source = 'fk-seed'
        vel, gp = load_episode(uid)
        states, actions = [], []
        picked = placed = contact = False
        for i in range(len(vel) - 1):
            a = np.concatenate([vel[i], [np.clip(gp[i] / 100.0, 0, 1)]]).astype(np.float32)
            states.append(obs['state'])
            actions.append(a)
            obs, done, info = env.step(a)
            picked |= bool(info['picked']); placed |= bool(info['placed']); contact |= bool(info['contact'])
        # settle in place (hold last action) to score nested at rest
        hold = np.concatenate([vel[-1], [np.clip(gp[-1] / 100.0, 0, 1)]]).astype(np.float32)
        for _ in range(100):
            obs, done, info = env.step(hold)
        # score nested directly (env only computes it at its own 1200-step horizon):
        # picked + proximity-touch + both upright (see replay_harness.NESTED_TOUCH_DIST)
        import torch
        from replay_harness import tilt_deg, NESTED_TOUCH_DIST
        def _np(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)
        w = env.w
        bp_, gp_ = _np(w['bottle'].get_pos()), _np(w['goal'].get_pos())
        touch = float(np.hypot(bp_[0] - gp_[0], bp_[1] - gp_[1])) <= NESTED_TOUCH_DIST
        nested = bool(picked and touch and tilt_deg(_np(w['bottle'].get_quat())) < 20
                      and tilt_deg(_np(w['goal'].get_quat())) < 20)
        stage = ('nested' if nested else 'contact' if contact else 'placed' if placed
                 else 'picked' if picked else 'no-pick')
        np.savez_compressed(OUT / f'{uid}.npz', states=np.array(states), actions=np.array(actions),
                            uid=uid, n=len(states), label=label, stage=stage)
        recs.append(dict(uid=uid, label=label, source=source, stage=stage, n=len(states)))
        print(f'{uid}({label[:4]},{source}): stage={stage} ({len(states)} frames)', flush=True)
    except Exception as e:
        print(f'{uid}: ERROR {type(e).__name__}: {e}', flush=True)

# each worker writes its own shard of the stage map; a merge step unions them
shard = OUT / f'_stage_{uids[0]}_{uids[-1]}.json'
shard.write_text(json.dumps({r['uid']: r for r in recs}, indent=2))
print(f'\nwrote {len(recs)} episodes + {shard.name}')
