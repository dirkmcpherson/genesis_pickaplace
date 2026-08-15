"""RLPD POSITIVE CONTROL: OUR RLPD machinery on ManiSkill PickCube-v1.

The claim under test is "our RLPD implementation is correct".  So the machinery
is held FIXED and imported unmodified from baselines/rl/rlpd_sac.py:

    E=10 LayerNorm critic ensemble | Z=2 target-min subset | UTD=10 (critic-only,
    one actor+alpha update per env step) | 50/50 online/demo batches 128+128 |
    backup_entropy OFF | auto-alpha | target_entropy = -dim/2 | lr 3e-4 |
    tau 0.005 | net (256,256) | actor trained on the ensemble MEAN Q.

Only HORIZON-SCALE parameters are re-tuned for the new task, each documented in
paper/ms_positive_control_2026-08-15.md.  The only substantive one is gamma
0.998 -> 0.99 (the genesis pick needs a ~500-step credit window; PickCube's is
~100).  0.99 is also RLPD's own published discount for its non-AntMaze domains.

Run with the dreamerv3-torch venv (the one that has mani_skill AND sb3 2.8):

  cd /home/j/workspace/genesis_pickaplace
  /home/j/workspace/dreamerv3-torch/venv/bin/python baselines/rl/train_rlpd_ms.py \
      --steps 300000 --seed 0 --run-name MS_RLPD-ctl_s0 \
      --out-dir baselines/rl/checkpoints/rlpd_msctl_s0 --project genesis_paper

This file is ADDITIVE: it imports rlpd_sac but changes nothing in it, and it
never imports full_env / train_sacfd_full / pick_env, so no genesis default can
be perturbed from here.
"""
import argparse
import json
import os
import pathlib as pl
import subprocess
import sys
import time

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))


def evaluate(model, success_mode, horizon, n_episodes, seed_offset, deterministic=True):
    """Deterministic-policy eval in a FRESH env instance with unseen seeds.

    Returns (success_rate, mean_len, native_rate, grasp_rate).  Protocol caveats
    live in the report: this is an in-train eval (same process, same interpreter),
    ManiSkill's own env, and the primary predicate is the RELAXED one the policy
    was trained on.
    """
    from ms_env import MSPickCubeEnv
    env = MSPickCubeEnv(success_mode=success_mode, horizon=horizon)
    n_ok = n_native = n_grasp = 0
    lens = []
    for i in range(n_episodes):
        obs, _ = env.reset(seed=int(seed_offset + i))
        grasped = False
        for t in range(horizon):
            act, _ = model.predict(obs, deterministic=deterministic)
            obs, r, term, trunc, info = env.step(act)
            grasped |= bool(info['is_grasped'])
            if term or trunc:
                n_ok += int(term)
                n_native += int(info['success_native'])
                lens.append(t + 1)
                break
        n_grasp += int(grasped)
    env.close()
    n = float(n_episodes)
    return n_ok / n, float(np.mean(lens)) if lens else float(horizon), n_native / n, n_grasp / n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--steps', type=int, default=300_000,
                    help='env steps. 300k is the pre-registered budget: published '
                         'RLPD-class methods solve sparse tasks of this difficulty '
                         'well inside it, and MS PPO solves PickCube far sooner.')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--device', default='cuda')
    ap.add_argument('--out-dir', default='baselines/rl/checkpoints/rlpd_msctl')
    ap.add_argument('--run-name', default=None)
    ap.add_argument('--project', default='genesis_paper')
    ap.add_argument('--no-wandb', action='store_true')
    # --- demos ---
    ap.add_argument('--demo-source', choices=['motionplanning', 'teleop'],
                    default='motionplanning',
                    help="motionplanning: ManiSkill's own solver demos (1000 "
                         'available, 100%% success, 49-98 steps, all inside the '
                         '100-step horizon). teleop: the 10 human episodes the '
                         'working dv3 ManiSkill runs used (83-133 steps).')
    ap.add_argument('--demo-episodes', type=int, default=50)
    # --- env ---
    ap.add_argument('--success-mode', choices=['relaxed', 'native'], default='relaxed',
                    help="relaxed (default) = grasped AND placed: the predicate the "
                         'dv3 ManiSkill runs used. native = ManiSkill\'s own '
                         '(placed AND robot static). Drives reward, termination AND '
                         'the demo relabel together -- never mixed.')
    ap.add_argument('--horizon', type=int, default=100,
                    help='decision steps per episode (dv3 MS runs used 100; stock '
                         'ManiSkill registers PickCube at 50)')
    # --- RLPD machinery: defaults are OUR genesis values, unchanged ---
    ap.add_argument('--utd', type=int, default=10)
    ap.add_argument('--ensemble-size', type=int, default=10)
    ap.add_argument('--subset-size', type=int, default=2)
    ap.add_argument('--demo-batch', type=int, default=128)
    ap.add_argument('--backup-entropy', choices=['on', 'off'], default='off')
    ap.add_argument('--per-member-ln', choices=['on', 'off'], default='off')
    ap.add_argument('--ent-coef', default='auto')
    ap.add_argument('--target-entropy', type=float, default=None,
                    help='default -dim/2 = -2.0 for the 4-dim pd_ee_delta_pos action')
    ap.add_argument('--learning-starts', type=int, default=None,
                    help='override make_rlpd\'s 1000 random warmup steps. Only used '
                         'by the training smoke (which is shorter than the warmup and '
                         'would otherwise never reach train()); leave unset for real '
                         'runs so the machinery keeps its genesis default.')
    ap.add_argument('--gamma', type=float, default=0.99,
                    help='TASK-SCALE adaptation: genesis uses 0.998 for a ~500-step '
                         'credit window; PickCube solves in 34-86 steps, and 0.99 is '
                         "RLPD's own published discount outside AntMaze.")
    # --- eval ---
    ap.add_argument('--eval-freq', type=int, default=50_000)
    ap.add_argument('--eval-episodes', type=int, default=10)
    ap.add_argument('--eval-seed-base', type=int, default=1_000_000,
                    help='eval ICs come from seeds far outside anything training or '
                         'the demos touched')
    args = ap.parse_args()

    t0 = time.time()
    import torch as th
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.callbacks import BaseCallback, CallbackList

    from ms_env import (MSPickCubeEnv, load_ms_demos, ENV_ID, CONTROL_MODE,
                        OBS_MODE, ROBOT_UIDS, REWARD_MODE)
    from rlpd_sac import make_rlpd, DemoData

    # ---- env ----
    env = MSPickCubeEnv(success_mode=args.success_mode, horizon=args.horizon)
    assert env.success_mode == args.success_mode
    assert env.horizon == args.horizon
    env = Monitor(env, info_keywords=('success', 'success_native', 'is_grasped'))
    print(f'[env] {ENV_ID} obs={OBS_MODE}{env.observation_space.shape} '
          f'act={CONTROL_MODE}{env.action_space.shape} robot={ROBOT_UIDS} '
          f'reward={REWARD_MODE}/{args.success_mode} horizon={args.horizon} '
          f'(built in {time.time() - t0:.1f}s)', flush=True)

    # ---- model: OUR machinery, imported unmodified ----
    # q_watchdog stays at 2.0: max task return is exactly 1.0 here (one +1, then
    # terminate), the same shape as the genesis terminal-only pick reward.
    model = make_rlpd(env, args.seed, args.device, q_watchdog=2.0,
                      backup_entropy=(args.backup_entropy == 'on'),
                      per_member_ln=(args.per_member_ln == 'on'),
                      ensemble_size=args.ensemble_size, subset_size=args.subset_size,
                      utd=args.utd, gamma=args.gamma, ent_coef=args.ent_coef,
                      target_entropy=args.target_entropy, demo_batch=args.demo_batch)
    if args.learning_starts is not None:
        model.learning_starts = int(args.learning_starts)
    print(f'[cfg] RLPD | E={args.ensemble_size} Z={args.subset_size} UTD={args.utd} '
          f'gamma={args.gamma} ent_coef={args.ent_coef} '
          f'target_entropy={model.target_entropy} demo_batch={args.demo_batch}/256 '
          f'backup_entropy={args.backup_entropy} per_member_ln={args.per_member_ln} '
          f'buffer={model.buffer_size} learning_starts={model.learning_starts} '
          f'q_watchdog=2.0 steps={args.steps}', flush=True)

    # ---- demos: same reward semantics as the env, by construction ----
    transitions, census = load_ms_demos(source=args.demo_source,
                                        n_episodes=args.demo_episodes,
                                        success_mode=args.success_mode)
    # actions are already native pd_ee_delta_pos in [-1,1]: NO action transform.
    demo = DemoData(transitions, None, th.device(args.device))
    assert demo.n_rewarded == census['n_episodes_used'], (demo.n_rewarded, census)
    model.set_demo_data(demo)
    print(f'[demos] {census["n_episodes_used"]} eps -> {demo.n} transitions in the '
          f'IMMUTABLE demo buffer (50% of every batch), {demo.n_rewarded} rewarded '
          f'({census["density_pct"]:.3f}%)', flush=True)

    # ---- sidecar (env=ms travels WITH the artifact; the silent-default rule) ----
    out = pl.Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    try:
        git = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=str(REPO),
                             capture_output=True, text=True, timeout=5).stdout.strip()
    except Exception:
        git = 'unknown'
    sidecar = {
        'env': 'ms', 'env_id': ENV_ID, 'robot_uids': ROBOT_UIDS,
        'obs_mode': OBS_MODE, 'control_mode': CONTROL_MODE,
        'reward_mode': REWARD_MODE, 'success_mode': args.success_mode,
        'horizon': args.horizon, 'terminate_on_success': True,
        'action_mode': 'ms_pd_ee_delta_pos', 'action_repeat': 1,
        'demo_source': args.demo_source, 'demo_file': census['path'],
        'demo_episodes': census['n_episodes_used'],
        'demo_transitions': census['n_transitions'],
        'demo_rewarded': census['n_rewarded'],
        'demo_density_pct': round(census['density_pct'], 4),
        'gamma': args.gamma, 'utd': args.utd, 'ensemble_size': args.ensemble_size,
        'subset_size': args.subset_size, 'demo_batch': args.demo_batch,
        'backup_entropy': args.backup_entropy, 'per_member_ln': args.per_member_ln,
        'ent_coef': args.ent_coef, 'seed': args.seed, 'steps': args.steps,
        'git': git or 'unknown', 'interpreter': sys.executable,
        # loud marker: wandb_eval / the genesis eval tooling CANNOT evaluate this
        # checkpoint -- it is a different MDP with a 42-dim obs and a 4-dim action.
        'genesis_eval_compatible': False,
    }
    sidecar_json = json.dumps(sidecar)
    (out / 'rlpd_ms.sidecar.json').write_text(sidecar_json)
    (out / 'demo_census.json').write_text(json.dumps(census))

    # ---- logging + eval callbacks ----
    run = None
    if not args.no_wandb:
        from wandb_utils import init_wandb, WandbScalarCallback   # generic, sb3-only
        run = init_wandb(args, name=args.run_name or out.name,
                         tags=('rlpd', 'maniskill', 'positive-control'),
                         project=args.project)
        if run is not None:
            run.config.update({'sidecar': sidecar}, allow_val_change=True)

    class MSEvalCallback(BaseCallback):
        """Deterministic-policy eval every eval_freq steps, plus one at step 0
        (a negative control: an untrained policy must score ~0) and one at the end.
        Cheap enough to run in-process: 10 eps x <=100 steps ~ 1000 env steps."""

        def __init__(self, run, out_dir, sidecar_json):
            super().__init__()
            self.run, self.out = run, pl.Path(out_dir)
            self.sidecar_json = sidecar_json
            self.best = -1.0

        def _do_eval(self, tag):
            t = time.time()
            sr, ml, nat, gr = evaluate(self.model, args.success_mode, args.horizon,
                                       args.eval_episodes, args.eval_seed_base)
            print(f'[eval @{self.num_timesteps} ({tag})] success={sr:.2f} '
                  f'(n={args.eval_episodes}) native={nat:.2f} grasp={gr:.2f} '
                  f'mean_len={ml:.0f} in {time.time() - t:.0f}s', flush=True)
            if self.run is not None:
                self.run.log({'eval/success': sr, 'eval/success_native': nat,
                              'eval/grasp_rate': gr, 'eval/mean_len': ml,
                              'eval/train_step': self.num_timesteps})
            if sr > self.best:
                self.best = sr
                self.model.save(str(self.out / 'rlpd_ms_best'))
                (self.out / 'rlpd_ms_best.sidecar.json').write_text(self.sidecar_json)
            return sr

        def _on_training_start(self):
            self._do_eval('pre-train negative control')

        def _on_step(self):
            if args.eval_freq and self.n_calls % args.eval_freq == 0:
                p = self.out / f'rlpd_ms_{self.num_timesteps}_steps'
                self.model.save(str(p))
                p.with_suffix('.sidecar.json').write_text(self.sidecar_json)
                self._do_eval('checkpoint')
            return True

    cbs = [MSEvalCallback(run, out, sidecar_json)]
    if run is not None:
        from wandb_utils import WandbScalarCallback
        cbs.append(WandbScalarCallback(run))

    model.learn(total_timesteps=args.steps, log_interval=10,
                callback=CallbackList(cbs))
    model.save(str(out / 'rlpd_ms_final'))
    (out / 'rlpd_ms_final.sidecar.json').write_text(sidecar_json)
    sr, ml, nat, gr = evaluate(model, args.success_mode, args.horizon,
                               max(args.eval_episodes, 20), args.eval_seed_base)
    print(f'[final eval] success={sr:.2f} native={nat:.2f} grasp={gr:.2f} '
          f'mean_len={ml:.0f}', flush=True)
    if run is not None:
        run.log({'eval/success': sr, 'eval/success_native': nat,
                 'eval/grasp_rate': gr, 'eval/train_step': args.steps})
        run.finish()
    print(f'[rlpd-ms] done in {(time.time() - t0) / 3600:.2f}h -> '
          f'{out}/rlpd_ms_final.zip', flush=True)


if __name__ == '__main__':
    main()
