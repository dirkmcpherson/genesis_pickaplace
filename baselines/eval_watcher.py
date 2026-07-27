"""Live eval curves for lerobot training runs (DP/ACT).

Watches <output_dir>/checkpoints/ while lerobot-train writes it; evals each NEW
checkpoint (and backfills existing ones) in THIS single process -- one genesis
world reused across checkpoints, one wandb run whose x-axis is the checkpoint
step. Gives lerobot legs the same inline-eval visibility the SB3 callback and the
dreamer results-file provide.

Usage:
  eval_watcher.py --output-dir baselines/outputs/dp_cart_delta \
      --cartesian --control delta --episodes 8 --until-steps 100000 \
      --group dp_cart --name dp_cart_delta-eval-live
"""
import os
import argparse
import pathlib as pl
import sys
import time

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', '/home/j/workspace/genesis_pickaplace'))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'can_pos_recovery'))

ap = argparse.ArgumentParser()
ap.add_argument('--output-dir', required=True)
ap.add_argument('--episodes', type=int, default=8)
ap.add_argument('--cartesian', action='store_true')
ap.add_argument('--control', choices=['vel', 'delta'], default='vel')
ap.add_argument('--until-steps', type=int, required=True,
                help='exit after evaluating a checkpoint >= this step')
ap.add_argument('--seed', type=int, default=0)
ap.add_argument('--group', default='eval_watcher')
ap.add_argument('--name', default=None)
ap.add_argument('--poll', type=int, default=120)
args = ap.parse_args()

import wandb  # noqa: E402
import ic_sampling  # noqa: E402
from eval_core import run_eval  # noqa: E402
from dp_runner import load_dp_runner  # noqa: E402

if args.cartesian:
    from cartesian_env import CartesianCanEnv
    env = CartesianCanEnv(backend='cpu', max_steps=1200, control=args.control)
else:
    from genesis_can_env import GenesisCanEnv
    env = GenesisCanEnv(backend='cpu', max_steps=1200)

run = wandb.init(project='genesis_pickaplace', group=args.group,
                 name=args.name or f'{pl.Path(args.output_dir).name}-eval-live',
                 config=vars(args))
episodes = ic_sampling.sample_support_ics(env, args.episodes, seed=args.seed)
ckdir = pl.Path(args.output_dir) / 'checkpoints'
seen = set()

while True:
    fresh = []
    for d in sorted(ckdir.glob('*/pretrained_model')) if ckdir.exists() else []:
        step_name = d.parent.name
        if step_name == 'last' or step_name in seen:
            continue
        try:
            step = int(step_name)
        except ValueError:
            continue
        fresh.append((step, d))
    for step, d in sorted(fresh):
        seen.add(d.parent.name)
        policy_action, policy_reset, _ = load_dp_runner(str(d))
        agg = run_eval(env, policy_action, episodes, policy_reset=policy_reset,
                       verbose=False)
        n = max(agg['n'], 1)
        metrics = {f'eval/{k}': agg[k] / n for k in ('picked', 'placed', 'contact', 'nested')}
        run.log(metrics, step=step)
        print(f'[watcher] ckpt {step}: ' +
              ' '.join(f'{k.split("/")[1]}={v:.2f}' for k, v in metrics.items()), flush=True)
        if step >= args.until_steps:
            run.finish()
            print('[watcher] done', flush=True)
            sys.exit(0)
    time.sleep(args.poll)
