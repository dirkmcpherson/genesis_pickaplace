"""Evaluate a trained lerobot policy (e.g. Diffusion Policy) in the Genesis env.

Runs each solved trial's initial conditions N times under the policy (closed loop,
30 Hz) and reports pick/place/contact/nested rates -- directly comparable to the
open-loop replay numbers in can_pos_recovery/validate_v2.log.

Needs BOTH stacks importable, so run with the lerobot venv and let it import genesis
from the sim venv's site-packages... in practice: install lerobot into the genesis venv
(pip install -e ~/workspace/lerobot --no-deps + missing deps) OR export the policy to
TorchScript. Simplest supported path here: run under the genesis venv after
    pip install -e ~/workspace/lerobot  (into the genesis venv)

Usage:
    python baselines/eval_policy.py <checkpoint_dir> [--reps 3] [--uids ...]
"""
import argparse, sys, pathlib as pl
import numpy as np
import torch

REPO = pl.Path('/home/james/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO / 'baselines'))
from genesis_can_env import GenesisCanEnv

ap = argparse.ArgumentParser()
ap.add_argument('checkpoint')
ap.add_argument('--reps', type=int, default=3)
ap.add_argument('--uids', type=int, nargs='*', default=None)
ap.add_argument('--image', action='store_true', help='policy consumes camera obs')
ap.add_argument('--random', type=int, default=0, metavar='N',
                help='instead of demo ICs, run N episodes from randomized ICs: can xy '
                     'uniform over the demo support, upright, goal at the standard spot')
ap.add_argument('--seed', type=int, default=0)
args = ap.parse_args()

from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
from lerobot.policies.factory import make_pre_post_processors
policy = DiffusionPolicy.from_pretrained(args.checkpoint)
policy.eval()
device = 'cuda' if torch.cuda.is_available() else 'cpu'
policy.to(device)
# lerobot 0.4.x moved (un)normalization OUT of the policy into processor pipelines.
# select_action consumes NORMALIZED obs and returns NORMALIZED actions, so we must
# wrap it with the saved pre/post processors or the arm gets [-1,1]-scale joint targets.
preprocessor, postprocessor = make_pre_post_processors(
    policy_cfg=policy.config, pretrained_path=args.checkpoint)


def policy_action(obs):
    batch = {'observation.state':
             torch.from_numpy(obs['state'][:7]).float().unsqueeze(0).to(device),
             'observation.environment_state':
             torch.from_numpy(obs['state'][7:]).float().unsqueeze(0).to(device),
             'task': [TASK]}
    if args.image:
        img = torch.from_numpy(obs['image']).permute(2, 0, 1).float() / 255.0
        batch['observation.images.cam'] = img.unsqueeze(0).to(device)
    batch = preprocessor(batch)
    with torch.no_grad():
        action = policy.select_action(batch)
    action = postprocessor(action)
    return action.squeeze(0).cpu().numpy()


TASK = 'pick the can and slide it against the can on the shelf'

env = GenesisCanEnv(backend='cpu', render_size=(96, 96) if args.image else None)
uids = args.uids or [u for u in env.solved_uids
                     if env.placements[u].get('label') == 'success']

if args.random:
    # randomized ICs: sample can xy uniformly over the demo-support box (with margin),
    # upright can, goal can at its standard shelf spot -- tests generalization beyond
    # the demos' exact starts
    rng = np.random.default_rng(args.seed)
    cxy = np.array([env.placements[u]['can_pos'][:2] for u in env.solved_uids])
    lo, hi = cxy.min(0) - 0.01, cxy.max(0) + 0.01
    can_z = 0.05 + env.world_cfg['can_height'] / 2 + 0.0125
    goal = (0.6, -0.2, 0.11 + env.world_cfg['can_height'] / 2 + 0.0425)
    episodes = [dict(can_pos=(float(rng.uniform(lo[0], hi[0])),
                              float(rng.uniform(lo[1], hi[1])), can_z),
                     goal_pos=goal, uid=None) for _ in range(args.random)]
else:
    episodes = [dict(uid=uid) for uid in uids for _ in range(args.reps)]

agg = dict(picked=0, placed=0, contact=0, nested=0, n=0)
for ep in episodes:
    for rep in range(1):
        uid = ep.get('uid')
        obs = env.reset(**ep)
        policy.reset()
        done = False
        while not done:
            action = policy_action(obs)
            obs, done, info = env.step(action)
        for k in ('picked', 'placed', 'contact', 'nested'):
            agg[k] += bool(info.get(k))
        agg['n'] += 1
        print(f"{uid} rep{rep}: picked={info['picked']} placed={info['placed']} "
              f"contact={info['contact']} nested={info.get('nested')}", flush=True)
n = max(agg['n'], 1)
print(f"\npolicy eval over {n} rollouts: picked {agg['picked']/n:.2f} "
      f"placed {agg['placed']/n:.2f} contact {agg['contact']/n:.2f} "
      f"nested {agg['nested']/n:.2f}")
# stage-conditional funnel: WHERE does it fail?
p, pl, c = max(agg['picked'], 1), max(agg['placed'], 1), max(agg['contact'], 1)
print(f"funnel: P(pick)={agg['picked']/n:.2f} -> P(place|pick)={agg['placed']/p:.2f} "
      f"-> P(slide|place)={agg['contact']/pl:.2f} -> P(nested|slide)={agg['nested']/c:.2f}")
