"""Evaluate a trained SB3 SAC policy in the FULL GenesisCanEnv via the shared eval_core,
so SAC is scored by the SAME loop/metrics/ICs as DP and BC (the honesty protocol needs
all three students on one ruler). SAC's own train_sacfd.evaluate() only measures pick on
PickOnlyEnv; this measures the full picked/placed/contact/nested funnel on the same env
and random ICs the other students use.

Usage:
  python baselines/rl/eval_sac.py <sacfd_ckpt.zip> [--reps 3] [--random N] [--seed 0]
                                  [--record-dir DIR]
"""
import os
import argparse
import pathlib as pl
import sys

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', '/home/j/workspace/genesis_pickaplace'))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))
from genesis_can_env import GenesisCanEnv  # noqa: E402
from pick_env import denormalize_action  # noqa: E402
import ic_sampling  # noqa: E402
import eval_core  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('checkpoint')
ap.add_argument('--reps', type=int, default=3)
ap.add_argument('--uids', type=int, nargs='*', default=None)
ap.add_argument('--random', type=int, default=0, metavar='N')
ap.add_argument('--seed', type=int, default=0)
ap.add_argument('--record-dir', default=None)
ap.add_argument('--max-steps', type=int, default=1200,
                help='rollout horizon (#21 lever). Use short (e.g. 400) for pick-only '
                     'checkpoint curves; keep 1200 for numbers compared across students.')
args = ap.parse_args()

from stable_baselines3 import SAC  # noqa: E402
model = SAC.load(args.checkpoint)


def policy_action(obs):
    a_norm, _ = model.predict(obs['state'], deterministic=True)
    return denormalize_action(a_norm)


render_size = (480, 640) if args.record_dir else None
env = GenesisCanEnv(backend='cpu', render_size=render_size, max_steps=args.max_steps)
if args.random:
    episodes = ic_sampling.sample_support_ics(env, args.random, seed=args.seed)
else:
    episodes = ic_sampling.demo_ics(env, uids=args.uids, reps=args.reps)

eval_core.run_eval(env, policy_action, episodes, record_dir=args.record_dir, tag='sac')
