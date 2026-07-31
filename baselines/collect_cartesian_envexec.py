"""ENV-EXECUTED cartesian demos: step CartesianCanEnv with the human's commanded
velocities and record the REALIZED transitions (s_i, a_i, s_{i+1} actually produced
by the env).

Why: episodes_cartesian pairs commanded actions with JOINT-REPLAY states -- at
coherence ~0.55 those transitions are dynamics-inconsistent with the training env,
which poisons TD backups (SACfD/RLPD flat at 0.00 while dynamics-free BC works).
Open-loop commanded replay does reproduce the task where the commands are robust
(uid 232 picks+contacts); this collector keeps whatever stages each episode
actually reaches under env execution and records honest per-episode outcomes.

Output npz mirrors episodes_cartesian (+ realized flags): states(n,18) actions(n,5)
n uid picked placed contact stage label [images].
Usage: collect_cartesian_envexec.py [--uids ...] [--images] [--outdir ...]
"""
import os
import argparse, glob, json, pathlib as pl
import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[1]))
import sys
sys.path.insert(0, str(REPO / 'baselines')); sys.path.insert(0, str(REPO / 'can_pos_recovery'))
from cartesian_env import CartesianCanEnv

ap = argparse.ArgumentParser()
ap.add_argument('--uids', type=int, nargs='*', default=None)
ap.add_argument('--outdir', default='baselines/episodes_cartesian_envexec')
ap.add_argument('--images', action='store_true')
args = ap.parse_args()
OUT = REPO / args.outdir; OUT.mkdir(parents=True, exist_ok=True)

tbl = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())['trials']
fk = {int(k): v for k, v in json.loads((REPO / 'can_pos_recovery/fk_recovered.json').read_text()).items()}
STATIC_GOAL = (0.6, -0.2)
BUCKET = {0: (0.4381, 0.1), 1: (0.4381, -0.05), 2: (0.4381, -0.2), None: (0.4381, -0.05)}

# commanded-velocity tapes -> velocity semantics by construction (explicit:
# the control default must never be load-bearing)
env = CartesianCanEnv(backend='cpu', max_steps=10 ** 9, camera_rig=args.images,
                      control='vel')
CANZ, GOALZ = env.env.w['can_start_z'], env.env.w['goal_start_z']


def place(uid):
    r = tbl.get(str(uid), {})
    solved = r.get('status') in ('ok', 'ok_batch')
    cq = list(r.get('can_quat') or [1, 0, 0, 0]) if solved else [1, 0, 0, 0]
    if solved:
        cx, cy = r['can_pos'][0], r['can_pos'][1]
        gp = r.get('goal_pos')
        gx, gy = ((gp[0], gp[1]) if (gp and r.get('label') == 'success'
                                     and not r.get('goal_moved')) else STATIC_GOAL)
    else:
        f = fk.get(uid, {})
        seed = f.get('close_xy') or (f.get('can_xy') if f.get('conf') in ('HIGH', 'MED')
                                     else BUCKET.get(f.get('pos')))
        if seed is None:
            raise KeyError('no seed')
        cx, cy = seed[0], seed[1]; gx, gy = STATIC_GOAL
    return (cx, cy, CANZ), cq, (gx, gy, GOALZ)


avail = sorted(int(pl.Path(p).stem.split('_')[0])
               for p in glob.glob(str(REPO / 'inthewild_trials/*_cartesian.npy')))
uids = args.uids or avail

n_ok = n_pick = 0
for uid in uids:
    try:
        can_pos, can_quat, goal_pos = place(uid)
    except (TypeError, KeyError) as e:
        print(f'{uid}: SKIP (placement: {e})', flush=True)
        continue
    d = np.load(REPO / f'inthewild_trials/{uid}_cartesian.npy', allow_pickle=True).item()
    cv = np.asarray(d['cartesian_velocity'], float)
    gp = np.asarray(d['gripper_pos'], float)[:, 0]
    n = len(cv)
    obs = env.reset(can_pos=can_pos, can_quat=can_quat, goal_pos=goal_pos)
    states, actions, images = [obs['state']], [], ([] if args.images else None)
    picked = placed = contact = False
    for i in range(n):
        a = np.array([cv[i, 0], cv[i, 1], cv[i, 2], cv[i, 4],
                      np.clip(gp[i] / 100.0, 0, 1)], np.float32)
        obs, done, info = env.step(a)
        actions.append(a)
        states.append(obs['state'])
        if args.images:
            images.append(env.rig_obs())
        picked |= bool(info['picked']); placed |= bool(info['placed'])
        contact |= bool(info['contact'])
    # states has n+1 entries (s_0..s_n): transition-complete for RL relabel
    stage = ('contact' if contact else 'placed' if placed else
             'picked' if picked else 'no-pick')
    payload = dict(states=np.array(states, np.float32),
                   actions=np.array(actions, np.float32),
                   n=n, uid=uid, picked=picked, placed=placed, contact=contact,
                   stage=stage, label='envexec')
    if args.images:
        payload['images'] = np.array(images, np.uint8)
    np.savez_compressed(OUT / f'{uid}.npz', **payload)
    n_ok += 1; n_pick += picked
    print(f'{uid}: n={n} picked={picked} placed={placed} contact={contact}', flush=True)
print(f'\nENVEXEC collected {n_ok} ({n_pick} pick) -> {OUT}')
