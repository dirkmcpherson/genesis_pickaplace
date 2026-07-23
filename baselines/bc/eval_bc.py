"""Evaluate a trained BC MLP in the Genesis env -- same loop/metrics/ICs as eval_policy.py
(DP) via the shared eval_core + ic_sampling modules, so BC and DP numbers are comparable.

Usage:
  python baselines/bc/eval_bc.py <bc.pt> [--reps 3] [--random N] [--seed 0] [--record-dir DIR]
"""
import argparse
import pathlib as pl
import sys

REPO = pl.Path('/home/j/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'bc'))
from genesis_can_env import GenesisCanEnv  # noqa: E402
import ic_sampling  # noqa: E402
import eval_core  # noqa: E402
from train_bc import load_bc_runner  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('checkpoint')
ap.add_argument('--reps', type=int, default=3)
ap.add_argument('--uids', type=int, nargs='*', default=None)
ap.add_argument('--random', type=int, default=0, metavar='N')
ap.add_argument('--seed', type=int, default=0)
ap.add_argument('--record-dir', default=None)
args = ap.parse_args()

render_size = (480, 640) if args.record_dir else None
env = GenesisCanEnv(backend='cpu', render_size=render_size)
policy_action = load_bc_runner(args.checkpoint)

if args.random:
    episodes = ic_sampling.sample_support_ics(env, args.random, seed=args.seed)
else:
    episodes = ic_sampling.demo_ics(env, uids=args.uids, reps=args.reps)

eval_core.run_eval(env, policy_action, episodes, record_dir=args.record_dir, tag='bc')
