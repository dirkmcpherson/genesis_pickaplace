"""RLPD trainer for the JOINT pick task -- SAC + 50/50 online/demo batches +
LayerNorm ensemble critics + high UTD (Ball et al. 2023), via RLPDSAC in
baselines/rl/rlpd_sac.py.

This is an EXTENSION of the SACfD path, not a fork:
  * the env is the same FullTaskEnv (delta_joint or absolute), and
  * the demo transitions come from train_sacfd_full's OWN encoder
    (relabel_full / delta_encode_transitions) so the RLPD demo tensors are
    BIT-IDENTICAL to SACfD's -- the human-vs-model demo-source comparison then
    differs only in the algorithm, never in the data pipeline (guarded by
    sacfd_delta_gate.py).

Why RLPD over seed-once SACfD (measured pathologies, see RLPD_PLAN.md):
  * demo dilution: injecting ~O(10^5) demo transitions into a 300k FIFO buffer
    that online data overwrites makes the rewarded demo frames a vanishing
    minority late in training. Here demos are an IMMUTABLE buffer that is
    permanently HALF of every batch.
  * sparse-reward value pessimism: LayerNorm critics + high UTD (RLPD's recipe).

The stale --cartesian branch of the previous train_rlpd was removed per
RLPD_PLAN.md (the EEF arm rides train_sacfd_full --cartesian; RLPD is the joint
arm). Recover it from git history if ever needed.

Usage (200k local pilot, the plan's primary arm):
  .venv-eval/bin/python baselines/rl/train_rlpd.py \
      --steps 200000 --scope pick --action-mode delta_joint --gamma 0.998 \
      --utd 10 --demo-dir baselines/episodes_pick_phase_all \
      --out-dir baselines/rl/checkpoints/rlpd_dH --run-name dH_RLPD_s0 \
      --project genesis_paper --seed 0 --device cuda
"""
import os
import argparse
import glob
import json
import pathlib as pl
import sys
import time

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--steps', type=int, default=200_000)
    ap.add_argument('--demo-dir', default='baselines/episodes_pick_phase_all',
                    help='human demo set (91 eps, 17/7-dim). The AI-demo arm points '
                         'this at a harvested set with the SAME layout.')
    ap.add_argument('--out-dir', default='baselines/rl/checkpoints/rlpd')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--device', default='cuda')
    # --- RLPD hypers (RLPD_PLAN.md "Pinned hypers") ---
    ap.add_argument('--utd', type=int, default=10,
                    help='critic gradient steps per env step (actor/alpha still x1). '
                         'Primary arm UTD-10 @200k ~7h; UTD-20 @100k is the alt arm.')
    ap.add_argument('--ensemble-size', type=int, default=10, help='E critic ensemble')
    ap.add_argument('--subset-size', type=int, default=2,
                    help='Z: target = min over a random Z-of-E TARGET critics')
    ap.add_argument('--demo-batch', type=int, default=128,
                    help='demo half of the 256 batch (128 demo + 128 online = 50/50)')
    ap.add_argument('--backup-entropy', choices=['on', 'off'], default='off',
                    help="entropy term in the CRITIC TARGET. 'off' = RLPD's setting "
                         "for every sparse domain (audit bug 1: 'on' at gamma=0.998 "
                         "gives a 500*alpha*H zero-reward fixed point that buries "
                         "the +1 and pays 400:1 AGAINST terminating). 'on' restores "
                         "the pre-audit behavior for comparison only.")
    ap.add_argument('--gamma', type=float, default=0.998,
                    help='0.998 ~ 500-step credit window. 0.98 made the demo pick '
                         'terminal (median frame 662) worth ~1.6e-6 from start '
                         'states -- a silent cause of the uniform SACfD zeros.')
    ap.add_argument('--ent-coef', default='auto',
                    help="SAC entropy coefficient. If the Q watchdog trips (mean "
                         "actor Q > 2), the pre-registered fix is a small FIXED "
                         "value here, e.g. 0.005.")
    ap.add_argument('--target-entropy', type=float, default=None,
                    help='default -dim/2 (= -3.5 for the 7-dim joint action)')
    ap.add_argument('--train-max-steps', type=int, default=900,
                    help='training episode horizon in SIM steps (joint; position-'
                         'target SAC can outrun the demonstrator so 900 suffices for '
                         'pick). With --action-repeat N this is ceil(900/N) decisions.')
    ap.add_argument('--action-repeat', type=int, default=1,
                    help='hold each policy decision for N consecutive sim steps '
                         '(delta_joint: same delta N times => N*a*cap target advance). '
                         'N=4 shrinks the 900-step episode to ~225 decisions, inside '
                         'the gamma=0.998 credit horizon (~500). MUST be passed '
                         'explicitly: it travels in the checkpoint sidecar and is '
                         'mirrored by the demo encoder, the in-train eval, and '
                         'wandb_eval --action-mode auto (the silent-default rule).')
    # --- shared-with-SACfD flags (mirror train_sacfd_full) ---
    ap.add_argument('--scope', choices=['full', 'pick'], default='pick',
                    help='pick: +1 and terminate on the pick (phase-1 paper core)')
    ap.add_argument('--action-mode', choices=['absolute', 'delta_joint'],
                    default='delta_joint',
                    help='delta_joint: env actions are per-step joint-target deltas '
                         '(cap 0.025, leash 5x) and demo actions are delta-encoded '
                         'to match, IMPORTED from train_sacfd_full so tensors are '
                         'bit-identical to SACfD. A sidecar records the mode for '
                         'wandb_eval --action-mode auto.')
    ap.add_argument('--no-wandb', action='store_true')
    ap.add_argument('--run-name', default=None)
    ap.add_argument('--project', default='genesis_paper', help='wandb project')
    ap.add_argument('--eval-freq', type=int, default=25_000,
                    help='video-eval subprocess cadence (0 disables)')
    ap.add_argument('--eval-max-steps', type=int, default=400,
                    help='eval rollout horizon (400 for pick-only curves, #21 lever)')
    args = ap.parse_args()

    # ---- env (joint only; the cartesian arm rides train_sacfd_full) ----
    t0 = time.time()
    from full_env import FullTaskEnv, STAGE_REWARD
    import pick_env
    from train_sacfd_full import (relabel_full, delta_encode_transitions,
                                  delta_encode_transitions_repeat)
    assert args.action_repeat >= 1, args.action_repeat
    if args.action_repeat > 1:
        assert args.action_mode == 'delta_joint', (
            'action_repeat is only wired for delta_joint (the repeat-encoder + eval '
            f'repeat assume it); got action_mode={args.action_mode}')
    env = FullTaskEnv(backend='cpu', max_steps=args.train_max_steps,
                      scope=args.scope, action_mode=args.action_mode,
                      action_repeat=args.action_repeat)
    # asserts: no silent defaults -- the env must be running the mode we asked for
    assert env.scope == args.scope, (env.scope, args.scope)
    assert env.action_mode == args.action_mode, (env.action_mode, args.action_mode)
    assert env.action_repeat == args.action_repeat, (env.action_repeat, args.action_repeat)
    print(f'[env] {type(env).__name__} built in {time.time() - t0:.1f}s | '
          f'pick_z={env.pick_z:.4f} scope={env.scope} action_mode={env.action_mode} '
          f'delta_cap={env.delta_cap} action_repeat={env.action_repeat} '
          f'(~{-(-args.train_max_steps // args.action_repeat)} decisions/ep)', flush=True)

    # ---- model (RLPDSAC: LN ensemble critics built at construction) ----
    from rlpd_sac import make_rlpd, DemoData
    model = make_rlpd(env, args.seed, args.device,
                      backup_entropy=(args.backup_entropy == 'on'),
                      ensemble_size=args.ensemble_size, subset_size=args.subset_size,
                      utd=args.utd, gamma=args.gamma, ent_coef=args.ent_coef,
                      target_entropy=args.target_entropy, demo_batch=args.demo_batch)
    print(f'[cfg] RLPD | E={args.ensemble_size} Z={args.subset_size} UTD={args.utd} '
          f'gamma={args.gamma} ent_coef={args.ent_coef} '
          f'target_entropy={model.target_entropy} demo_batch={args.demo_batch}/256 '
          f'scope={args.scope} action_mode={args.action_mode} '
          f'action_repeat={args.action_repeat}', flush=True)
    print(model.critic, flush=True)

    # ---- demos: SAME encoder as train_sacfd_full (bit-identical tensors) ----
    paths = sorted(glob.glob(str(REPO / args.demo_dir / '*.npz')))
    assert paths, f'no npz in {args.demo_dir}'
    if args.action_mode == 'delta_joint':
        # EXPLICIT encoder selection (no silent stride-1 fallback): action_repeat>1
        # decision-level demos vs the stride-1 encoder. repeat==1 keeps the exact
        # stride-1 tensors SACfD/the gate assert bit-equality against.
        if args.action_repeat > 1:
            transitions = delta_encode_transitions_repeat(
                paths, env.pick_z, args.scope, env.delta_cap, args.action_repeat)
        else:
            transitions = delta_encode_transitions(paths, env.pick_z, args.scope,
                                                   env.delta_cap)
        norm = None                    # actions already in normalized delta space
    else:
        transitions, _ = relabel_full(paths, env.pick_z)
        if args.scope == 'pick':
            transitions = [(o, a, r, o2, True) if r >= STAGE_REWARD['picked'] else
                           (o, a, r, o2, d) for (o, a, r, o2, d) in transitions]
        norm = pick_env.normalize_action
    import torch as th
    demo = DemoData(transitions, norm, th.device(args.device))
    model.set_demo_data(demo)
    print(f'[demos] {len(paths)} eps -> {demo.n} transitions in the IMMUTABLE demo '
          f'buffer (50% of every batch), {demo.n_rewarded} rewarded', flush=True)

    # ---- output + action_mode sidecars (the silent-default-bug rule: control mode
    # travels WITH the artifact so wandb_eval --action-mode auto reads it) ----
    out = pl.Path(args.out_dir); out.mkdir(parents=True, exist_ok=True)
    # action_repeat travels WITH the artifact so wandb_eval --action-mode auto applies
    # the SAME repeat at eval time (stateful: one policy query per N env steps). Evaling
    # a repeat-N policy at stride 1 is the exact silent-default bug family this repo keeps
    # hitting -- the sidecar closes it.
    sidecar = {'action_mode': args.action_mode, 'action_repeat': args.action_repeat,
               'backup_entropy': args.backup_entropy}
    # STARTUP sidecar next to the VideoEvalCallback snapshot dir, so even the first
    # in-train eval snapshot (which the callback ALSO passes --action-mode for) has a
    # readable record; and the final one next to rlpd_final.
    (out / 'wandb_eval').mkdir(parents=True, exist_ok=True)
    (out / 'wandb_eval' / 'snapshot.action_mode.json').write_text(json.dumps(sidecar))

    from stable_baselines3.common.callbacks import CheckpointCallback, CallbackList
    from wandb_utils import init_wandb, WandbScalarCallback, VideoEvalCallback

    class SidecarCheckpointCallback(CheckpointCallback):
        """Write the action-mode sidecar NEXT TO EVERY snapshot checkpoint.

        The stock callback saves rlpd_<N>_steps.zip with no sidecar, so wandb_eval
        --action-mode auto resolves <stem>.action_mode.json, finds nothing, and
        falls back to absolute@1 -- a delta policy then evals ~0.00 and the zero
        looks like a result. Third member of the silent-default family (grip
        column, control mode; caught by newbox_supp 2026-08-13 before it produced
        13 artifact zeros in the 100k post-hoc sweep)."""

        def __init__(self, sidecar_json, **kw):
            super().__init__(**kw)
            self._sidecar_json = sidecar_json

        def _on_step(self):
            ok = super()._on_step()
            if self.n_calls % self.save_freq == 0:
                zip_path = pl.Path(self._checkpoint_path(extension='zip'))
                zip_path.with_name(zip_path.stem + '.action_mode.json').write_text(
                    self._sidecar_json)
            return ok

    run = init_wandb(args, name=args.run_name or out.name, tags=('rlpd',),
                     project=args.project)
    cbs = [SidecarCheckpointCallback(json.dumps(sidecar), save_freq=50_000,
                                     save_path=str(out), name_prefix='rlpd'),
           WandbScalarCallback(run)]
    if args.eval_freq:
        cbs.append(VideoEvalCallback(
            run, out, eval_freq=args.eval_freq, max_steps=args.eval_max_steps,
            seed=args.seed, cartesian=False,
            action_mode=(args.action_mode if args.action_mode != 'absolute' else None),
            action_repeat=args.action_repeat))
    model.learn(total_timesteps=args.steps, log_interval=10, callback=CallbackList(cbs))
    model.save(str(out / 'rlpd_final'))
    (out / 'rlpd_final.action_mode.json').write_text(json.dumps(sidecar))
    if run is not None:
        run.finish()
    print(f'[rlpd] done in {(time.time() - t0)/3600:.1f}h -> {out}/rlpd_final.zip',
          flush=True)


if __name__ == '__main__':
    main()
