"""Evaluate a trained lerobot Diffusion Policy in the Genesis env.

Thin CLI over the shared eval modules (ic_sampling + eval_core + dp_runner) so DP is
scored by the SAME loop/metrics/IC distribution as the BC and SAC students. Runs each
solved trial's ICs N times (closed loop, 30 Hz) OR --random N randomized ICs, and reports
pick/place/contact/nested rates + a failure funnel -- directly comparable to the open-loop
replay numbers in can_pos_recovery/validate_v2.log.

Run under the genesis venv after `pip install -e ~/workspace/lerobot` into it (or under
.venv-eval, which has genesis + lerobot together).

Usage:
    python baselines/eval_policy.py <checkpoint_dir> [--reps 3] [--uids ...]
    python baselines/eval_policy.py <checkpoint_dir> --random 60 --seed 0 [--record-dir DIR]
"""
import argparse
import pathlib as pl
import sys

REPO = pl.Path('/home/j/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO / 'baselines'))
from genesis_can_env import GenesisCanEnv  # noqa: E402
import ic_sampling  # noqa: E402
import eval_core  # noqa: E402
from dp_runner import load_dp_runner  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('checkpoint')
ap.add_argument('--reps', type=int, default=3)
ap.add_argument('--uids', type=int, nargs='*', default=None)
ap.add_argument('--image', action='store_true', help='policy consumes camera obs')
ap.add_argument('--random', type=int, default=0, metavar='N',
                help='instead of demo ICs, run N episodes from randomized ICs sampled '
                     'over the demo support (ic_sampling.sample_support_ics)')
ap.add_argument('--seed', type=int, default=0)
ap.add_argument('--record-dir', default=None,
                help='if set, save a per-episode mp4 (uses the env camera)')
ap.add_argument('--max-steps', type=int, default=1200,
                help='rollout horizon (#21 throughput lever; keep equal across policies '
                     'for a fair comparison)')
args = ap.parse_args()

render_size = (96, 96) if args.image else ((480, 640) if args.record_dir else None)
env = GenesisCanEnv(backend='cpu', render_size=render_size, max_steps=args.max_steps)
policy_action, policy_reset, proprio = load_dp_runner(args.checkpoint, image=args.image)
print(f'[eval_policy] checkpoint PROPRIO={proprio}', flush=True)

if args.random:
    episodes = ic_sampling.sample_support_ics(env, args.random, seed=args.seed)
else:
    episodes = ic_sampling.demo_ics(env, uids=args.uids, reps=args.reps)

eval_core.run_eval(env, policy_action, episodes, policy_reset=policy_reset,
                   record_dir=args.record_dir, tag='dp')
