"""Closed-loop eval of a 4-DOF Cartesian policy in CartesianCanEnv.

The policy emits [vx, vy, vz, v_pitch, gripper]; CartesianCanEnv integrates it (IK velocity
control) and reports the same picked/placed/contact/nested funnel as eval_policy.py, via the
shared eval_core -- so Cartesian numbers are directly comparable to the joint-space DP/BC.

Handles both policy types:
  * DP  -> checkpoint is a directory (pretrained_model); loaded via dp_runner (obs split from
           the checkpoint config: state[:9] + environment_state[9:]).
  * BC  -> checkpoint is a .pt file; loaded via bc.train_bc.load_bc_runner.

Usage:
  python baselines/eval_cartesian.py <checkpoint> [--reps 3] [--uids ...]
  python baselines/eval_cartesian.py <checkpoint> --random 30 --seed 0 [--record-dir DIR]
"""
import argparse
import pathlib as pl
import sys

REPO = pl.Path('/home/j/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'bc'))
from cartesian_env import CartesianCanEnv  # noqa: E402
import ic_sampling  # noqa: E402
import eval_core  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('checkpoint')
ap.add_argument('--reps', type=int, default=3)
ap.add_argument('--uids', type=int, nargs='*', default=None)
ap.add_argument('--random', type=int, default=0, metavar='N')
ap.add_argument('--seed', type=int, default=0)
ap.add_argument('--record-dir', default=None)
args = ap.parse_args()

ck = pl.Path(args.checkpoint)
is_bc = ck.is_file() and ck.suffix == '.pt'

render_size = (480, 640) if args.record_dir else None
env = CartesianCanEnv(backend='cpu', render_size=render_size)

policy_reset = None
if is_bc:
    from train_bc import load_bc_runner
    policy_action = load_bc_runner(str(ck))
    print(f'[eval_cartesian] BC policy from {ck.name}', flush=True)
else:
    from dp_runner import load_dp_runner
    policy_action, policy_reset, proprio = load_dp_runner(str(ck))
    print(f'[eval_cartesian] DP policy, PROPRIO={proprio}', flush=True)

if args.random:
    episodes = ic_sampling.sample_support_ics(env, args.random, seed=args.seed)
else:
    episodes = ic_sampling.demo_ics(env, uids=args.uids, reps=args.reps)

eval_core.run_eval(env, policy_action, episodes, policy_reset=policy_reset,
                   record_dir=args.record_dir, tag='cart')
