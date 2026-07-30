"""Eval a checkpoint (SAC .zip or DP dir) on random ICs, record videos + metrics json,
optionally log to wandb. One implementation for three uses:

  1. SACfD in-train periodic eval: spawned by the trainer with --no-wandb --json;
     the TRAINING process reads the json and logs to ITS run (single wandb writer).
  2. DP post-train eval: run with wandb on -> its own run in the same project/group.
  3. Ad-hoc: eval any checkpoint with videos, wandb optional.

Usage:
  wandb_eval.py --kind sac --checkpoint ck.zip --random 10 --max-steps 400 \
      --record-dir /tmp/ev --json /tmp/ev/metrics.json --no-wandb
  wandb_eval.py --kind dp --checkpoint baselines/outputs/dp_pick_v2/checkpoints/last/pretrained_model \
      --random 15 --group dp_pick_v3 --name dp_pick_v3-eval
"""
import os
import argparse, json, pathlib as pl, sys

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', '/home/j/workspace/genesis_pickaplace'))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))

ap = argparse.ArgumentParser()
ap.add_argument('--kind', choices=('sac', 'dp'), required=True)
ap.add_argument('--checkpoint', required=True)
ap.add_argument('--random', type=int, default=10)
ap.add_argument('--uids', type=int, nargs='*', default=None)
ap.add_argument('--reps', type=int, default=1)
ap.add_argument('--seed', type=int, default=0)
ap.add_argument('--max-steps', type=int, default=1200)
ap.add_argument('--cartesian', action='store_true',
                help='eval a cartesian-action policy through CartesianCanEnv')
ap.add_argument('--ic-mode', choices=['random', 'demo', 'both'], default='random',
                help="demo = the demos' OWN starting placements (in-distribution). "
                     "A policy that scores well on demo ICs but zero on random ICs "
                     "is a generalization failure, not a broken pipeline -- the "
                     "positive control this project has never had.")
ap.add_argument('--control', choices=['vel', 'delta', 'abs', 'abs6', 'delta6'], default='vel',
                help='cartesian env control mode (must match the policy training data)')
ap.add_argument('--n-action-steps', type=int, default=None,
                help='override the chunk-execution length (ACT default 100 = 2.5s '
                     'open-loop; velocity-integration drift compounds within chunks)')
ap.add_argument('--record-dir', default=None, help='default: <checkpoint>_eval_videos')
ap.add_argument('--json', dest='json_out', default=None, help='write metrics dict here')
ap.add_argument('--videos', type=int, default=3, help='max videos uploaded to wandb')
ap.add_argument('--no-wandb', action='store_true')
ap.add_argument('--project', default='genesis_pickaplace')
ap.add_argument('--group', default=None)
ap.add_argument('--name', default=None)
ap.add_argument('--step', type=int, default=None, help='training step tag for the metrics')
args = ap.parse_args()

import numpy as np  # noqa: E402
import ic_sampling, eval_core  # noqa: E402
from genesis_can_env import GenesisCanEnv  # noqa: E402

rec = args.record_dir or (str(pl.Path(args.checkpoint).with_suffix('')) + '_eval_videos')
# Decide camera needs BEFORE building the env: genesis allows ONE world per process,
# so probing the checkpoint after construction and rebuilding would double-gs.init.
_needs_rig = False
if args.kind == 'dp':
    import json as _json
    _cfgd = _json.loads((pl.Path(args.checkpoint) / 'config.json').read_text())
    _needs_rig = any(k.startswith('observation.images.')
                     for k in _cfgd.get('input_features', {}))
if args.cartesian:
    from cartesian_env import CartesianCanEnv
    env = CartesianCanEnv(backend='cpu', render_size=(480, 640),
                          max_steps=args.max_steps, camera_rig=_needs_rig,
                          control=args.control)
else:
    env = GenesisCanEnv(backend='cpu', render_size=(480, 640), max_steps=args.max_steps,
                        camera_rig=_needs_rig)

if args.kind == 'sac':
    from stable_baselines3 import SAC
    if args.cartesian:
        from cartesian_env import CartesianCanEnv as _C
        # MUST match the env's control mode: delta actions denormalized with the
        # velocity scale (or vice versa) are a different command type entirely.
        denormalize_action = ({'delta': _C.denormalize_delta,
                               'abs': _C.denormalize_abs,
                               'abs6': _C.denormalize_abs6,
                               'delta6': _C.denormalize_delta6}.get(args.control)
                              or _C.denormalize_action)
    else:
        from pick_env import denormalize_action
    model = SAC.load(args.checkpoint, device='cpu')

    def policy_action(obs):
        a, _ = model.predict(obs['state'], deterministic=True)
        return denormalize_action(a)
    policy_reset = None
else:
    from dp_runner import load_dp_runner
    policy_action, policy_reset, _proprio = load_dp_runner(
        args.checkpoint, rig_provider=(env.rig_obs if _needs_rig else None),
        n_action_steps=args.n_action_steps)

_ic_sets = {}
if args.random:
    if args.ic_mode in ('demo', 'both'):
        _ic_sets['indist'] = ic_sampling.demo_ics(env, reps=1)[:args.random]
    if args.ic_mode in ('random', 'both'):
        _ic_sets['random'] = ic_sampling.sample_support_ics(
            env, args.random, seed=args.seed)
else:
    _ic_sets['indist'] = ic_sampling.demo_ics(env, uids=args.uids, reps=args.reps)
episodes = list(_ic_sets.values())[0]

# Evaluate every prepared IC set in ONE env (genesis allows one world per process).
# Prefix stays 'eval/' for the single-set case so existing dashboards keep working;
# 'both' adds eval_indist/* and eval_random/* so the generalization gap is visible
# on one chart instead of across two runs.
metrics = {}
_aggs = {}
_single = len(_ic_sets) == 1
for _name, _eps in _ic_sets.items():
    _rec = rec if _single else str(pl.Path(rec) / _name)
    _a = eval_core.run_eval(env, policy_action, _eps, policy_reset=policy_reset,
                            record_dir=_rec, tag=f'{args.kind}:{_name}')
    _aggs[_name] = _a
    _n = max(_a['n'], 1)
    _pref = 'eval' if _single else f'eval_{_name}'
    metrics.update({f'{_pref}/{k}': _a[k] / _n for k in eval_core.STAGES})
    metrics[f'{_pref}/n'] = _a['n']
if not _single:                      # keep a canonical eval/* = the random (honest) set
    _a = _aggs.get('random') or list(_aggs.values())[0]
    _n = max(_a['n'], 1)
    metrics.update({f'eval/{k}': _a[k] / _n for k in eval_core.STAGES})
    metrics['eval/gen_gap_picked'] = (
        (_aggs['indist']['picked'] / max(_aggs['indist']['n'], 1))
        - (_aggs['random']['picked'] / max(_aggs['random']['n'], 1))
    ) if 'indist' in _aggs and 'random' in _aggs else 0.0
agg = _aggs.get('random') or list(_aggs.values())[0]
n = max(agg['n'], 1)
if args.step is not None:
    metrics['eval/train_step'] = args.step
vids = sorted(str(p) for p in pl.Path(rec).rglob('*.mp4'))
tiled = None
if len(vids) > 1:
    import subprocess
    tiled = str(pl.Path(rec) / 'tiled.mp4')
    tc = subprocess.run([sys.executable, str(REPO / 'baselines/tile_videos.py'),
                         rec, '--out', tiled], capture_output=True, text=True)
    if tc.returncode != 0:
        print('tiling failed:', tc.stderr[-200:], flush=True)
        tiled = None
    vids = [v for v in vids if not v.endswith('tiled.mp4')]
result = dict(metrics=metrics, videos=vids, tiled=tiled, checkpoint=args.checkpoint,
              seed=args.seed, max_steps=args.max_steps)

if args.json_out:
    pl.Path(args.json_out).write_text(json.dumps(result, indent=1))
    print(f'metrics -> {args.json_out}', flush=True)

if not args.no_wandb:
    import wandb
    run = wandb.init(project=args.project, group=args.group, name=args.name,
                     job_type='eval', config=dict(checkpoint=args.checkpoint,
                                                  seed=args.seed, n=agg['n'],
                                                  max_steps=args.max_steps))
    log = dict(metrics)
    if tiled:
        log['eval/rollouts_tiled'] = wandb.Video(tiled, format='mp4',
                                                 caption='all episodes tiled')
    for i, v in enumerate(vids[:args.videos]):
        log[f'eval/video_{i}'] = wandb.Video(v, format='mp4',
                                             caption=pl.Path(v).stem)
    run.log(log, step=args.step or 0)
    run.finish()
    print(f'wandb run {run.name} logged', flush=True)
