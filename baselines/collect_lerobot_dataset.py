"""Collect a LeRobotDataset of sim-verified demonstrations for BC baselines.

For every solved trial in trial_placements.json: replay the recorded demo open-loop in
the corrected world, record (obs, action) at 30 Hz, and KEEP the episode only if the
replay reaches contact-success (so the dataset contains only working demonstrations
under sim physics). Action convention: next-waypoint joint targets + gripper 0..1.

Run with the LEROBOT venv (needs lerobot + this repo's genesis venv packages on path is
NOT required -- we run physics via the genesis venv? No: run everything in the genesis
venv and pip-install lerobot there? Neither: this script runs under the genesis venv and
writes episodes to disk as npz; convert_to_lerobot.py (run under the lerobot venv) packs
them into a LeRobotDataset. Two stages so neither venv needs the other's heavyweights.

Stage 1 (genesis venv):  python baselines/collect_lerobot_dataset.py [--render]
    -> baselines/episodes_raw/<uid>.npz
"""
import argparse, json, sys, pathlib as pl
import numpy as np

REPO = pl.Path('/home/james/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'can_pos_recovery'))
from genesis_can_env import GenesisCanEnv
from replay_harness import load_episode

ap = argparse.ArgumentParser()
ap.add_argument('--render', action='store_true', help='record 96x96 camera frames too')
ap.add_argument('--uids', type=int, nargs='*', default=None)
ap.add_argument('--outdir', default='baselines/episodes_raw')
args = ap.parse_args()

OUT = REPO / args.outdir
OUT.mkdir(exist_ok=True, parents=True)

env = GenesisCanEnv(backend='cpu', render_size=(96, 96) if args.render else None)
uids = args.uids or [u for u in env.solved_uids
                     if env.placements[u].get('label') == 'success']
kept = skipped = 0
for uid in uids:
    f = OUT / f'{uid}.npz'
    if f.exists():
        kept += 1; continue
    vel, gp = load_episode(uid)
    contact = False
    SETTLE_FRAMES = 15   # ~0.5s after first contact, then stop -- keeps the demo ending
                         # at the task completion instead of the arm's trailing retreat
                         # (which replayed open-loop tends to knock the can over)
    for attempt in range(8):    # borderline placements are stochastic; try several times
        obs = env.reset(uid=uid)
        states, images, actions = [], [], []
        post_contact = 0
        for i in range(len(vel) - 1):
            a = np.concatenate([vel[i + 1], [np.clip(gp[i + 1] / 100.0, 0, 1)]])  # next waypoint
            states.append(obs['state'])
            if args.render: images.append(obs['image'])
            actions.append(a.astype(np.float32))
            obs, done, info = env.step(np.concatenate([vel[i], [np.clip(gp[i] / 100.0, 0, 1)]]))
            # env's contact now requires the can to have been picked first, so a
            # table-shove into the goal no longer counts as success (trial 284)
            if info['contact']:
                contact = True
            if contact:
                post_contact += 1
                if post_contact >= SETTLE_FRAMES:
                    break   # truncate: demo ends at completion, not the retreat
        if contact:
            break
    if not contact:
        print(f'{uid}: no contact-success in 8 attempts, SKIPPED', flush=True)
        skipped += 1
        continue
    data = dict(states=np.array(states), actions=np.array(actions),
                uid=uid, n=len(states))
    if args.render:
        data['images'] = np.array(images, dtype=np.uint8)
    np.savez_compressed(f, **data)
    kept += 1
    print(f'{uid}: kept ({len(states)} frames)', flush=True)
print(f'\nkept {kept}, skipped {skipped} -> {OUT}')
