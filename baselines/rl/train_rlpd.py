"""RLPD-style SACfD: demos in a SEPARATE permanent buffer, sampled 50/50 with
online data at every gradient step (Ball et al. 2023 recipe, adapted to SB3).

Fixes two measured pathologies of the seed-once SACfD:
  * demo dilution/overwrite: 520k demo transitions into a 300k buffer wrapped at
    injection, and online data then overwrites more -- late in training demos are
    a shrinking minority. Here the demo buffer is immutable and always half of
    every batch.
  * value pessimism on sparse reward: LayerNorm on critic MLPs (the RLPD trick
    that stabilizes high UTD) + utd gradient steps per env step.

Action-space agnostic: --cartesian uses CartesianFullTaskEnv + relabel_cartesian
(18/5-dim), else FullTaskEnv + relabel_full (17/7-dim). Same staged rewards, same
eval protocol (VideoEvalCallback -> wandb_eval).

Usage:
  train_rlpd.py --cartesian --train-max-steps 1800 --steps 200000 \
      --out-dir baselines/rl/checkpoints/rlpd_cart --run-name rlpd_cart
"""
import os
import argparse
import glob
import pathlib as pl
import sys
import time

import numpy as np
import torch

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', '/home/j/workspace/genesis_pickaplace'))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))
sys.path.insert(0, str(REPO / 'can_pos_recovery'))


def build_rlpd_model(env, seed, device, utd=4, gamma=0.98, ent_coef='auto'):
    """SAC + LayerNorm critics + UTD>1. Demo mixing is done by DemoMixSAC."""
    from stable_baselines3 import SAC
    policy_kwargs = dict(
        net_arch=dict(pi=[256, 256], qf=[256, 256]),
    )
    model = SAC(
        'MlpPolicy', env,
        learning_rate=3e-4,
        buffer_size=300_000,          # ONLINE data only; demos live elsewhere
        learning_starts=1_000,
        batch_size=256,
        tau=0.005,
        gamma=gamma,
        train_freq=1,
        gradient_steps=utd,           # UTD: gradient steps per env step
        ent_coef=ent_coef,
        seed=seed,
        device=device,
        verbose=1,
        policy_kwargs=policy_kwargs,
    )
    # LayerNorm after each critic Linear (RLPD's stabilizer for high UTD).
    # SB3 builds critic MLPs as Sequential(Linear, ReLU, Linear, ReLU, Linear);
    # rebuild with LayerNorm between Linear and ReLU.
    import torch.nn as nn

    def add_layernorm(qnet):
        layers = []
        for m in qnet:
            layers.append(m)
            if isinstance(m, nn.Linear) and m.out_features != 1:
                layers.append(nn.LayerNorm(m.out_features))
        # drop the ReLUs that directly follow a Linear (they now follow the LN)
        return nn.Sequential(*layers)

    for critic in (model.critic, model.critic_target):
        for i, qnet in enumerate(critic.q_networks):
            critic.q_networks[i] = add_layernorm(qnet).to(device)
    # re-point the optimizer at the new critic parameters
    model.critic.optimizer = torch.optim.Adam(model.critic.parameters(), lr=3e-4)
    return model


class DemoMix:
    """Wraps model.replay_buffer.sample to return 50/50 online/demo batches.

    The demo half comes from immutable tensors built once at startup; the online
    half from SB3's normal buffer. SAC's train() only touches the buffer through
    .sample(), so this transparently implements RLPD's symmetric sampling.
    """

    def __init__(self, model, transitions, action_transform):
        obs = torch.as_tensor(np.stack([t[0] for t in transitions]), dtype=torch.float32)
        nobs = torch.as_tensor(np.stack([t[3] for t in transitions]), dtype=torch.float32)
        act = np.stack([t[1] for t in transitions]).astype(np.float32)
        if action_transform is not None:
            act = np.asarray(action_transform(act), dtype=np.float32)
        act = torch.as_tensor(act)
        rew = torch.as_tensor(np.array([t[2] for t in transitions], dtype=np.float32))
        done = torch.as_tensor(np.array([t[4] for t in transitions], dtype=np.float32))
        self.data = (obs, act, nobs, done, rew)
        self.n = len(transitions)
        self.device = model.device
        self.buffer = model.replay_buffer
        self._orig_sample = self.buffer.sample
        self.buffer.sample = self.sample   # monkey-patch: SAC.train calls this
        self.rng = np.random.default_rng(0)

    def sample(self, batch_size, env=None):
        from stable_baselines3.common.buffers import ReplayBufferSamples
        half = batch_size // 2
        online = self._orig_sample(batch_size - half, env)
        idx = self.rng.integers(0, self.n, half)
        obs, act, nobs, done, rew = (x[idx].to(self.device) for x in self.data)
        return ReplayBufferSamples(
            observations=torch.cat([online.observations, obs]),
            actions=torch.cat([online.actions, act]),
            next_observations=torch.cat([online.next_observations, nobs]),
            dones=torch.cat([online.dones, done.unsqueeze(1)]),
            rewards=torch.cat([online.rewards, rew.unsqueeze(1)]),
        )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--steps', type=int, default=200_000)
    ap.add_argument('--demo-dir', default=None,
                    help='default: episodes_cartesian if --cartesian else episodes_all')
    ap.add_argument('--out-dir', default='baselines/rl/checkpoints/rlpd')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--device', default='cuda')
    ap.add_argument('--utd', type=int, default=4)
    ap.add_argument('--gamma', type=float, default=0.98,
                    help='0.98 = ~50-step credit window. Cartesian VCAP physics puts '
                         'the first pick ~600 steps out (0.98^600~6e-6: reward '
                         'mathematically invisible) -> use 0.999 (~1000-step window). '
                         'Joint tolerated 0.98 because position-target SAC moves '
                         'faster than the demonstrator.')
    ap.add_argument('--ent-coef', default='auto',
                    help="SAC entropy coefficient. MUST be a small fixed value at "
                         "high gamma: Q accumulates gamma^t*alpha*H ~ alpha*H/(1-gamma) "
                         "-- at gamma .999 the probe measured start-state Q=+536 "
                         "(task max ~8): the critic valued staying-random 60x above "
                         "the task. e.g. 0.005 with --gamma 0.995.")
    ap.add_argument('--train-max-steps', type=int, default=900)
    ap.add_argument('--cartesian', action='store_true')
    ap.add_argument('--control', choices=['vel', 'delta'], default='vel',
                    help="cartesian control mode. MUST match --demo-dir: 'vel' with "
                         "episodes_cartesian[_realized], 'delta' with "
                         "episodes_cartesian_delta. (Every cartesian RL run before "
                         "2026-07-29 silently used 'vel' -- the constructor default.)")
    ap.add_argument('--no-wandb', action='store_true')
    ap.add_argument('--run-name', default=None)
    ap.add_argument('--eval-freq', type=int, default=25_000)
    ap.add_argument('--eval-max-steps', type=int, default=1200)
    args = ap.parse_args()

    t0 = time.time()
    if args.cartesian:
        from full_env import CartesianFullTaskEnv
        from relabel_cartesian import relabel_cartesian as relabel
        from cartesian_env import CartesianCanEnv
        env = CartesianFullTaskEnv(backend='cpu', max_steps=args.train_max_steps,
                                   control=args.control)
        norm = (CartesianCanEnv.normalize_delta if args.control == 'delta'
                else CartesianCanEnv.normalize_action)
        demo_dir = args.demo_dir or ('baselines/episodes_cartesian_delta'
                                     if args.control == 'delta'
                                     else 'baselines/episodes_cartesian')
    else:
        from full_env import FullTaskEnv
        from train_sacfd_full import relabel_full
        import pick_env
        env = FullTaskEnv(backend='cpu', max_steps=args.train_max_steps)
        relabel = lambda p, z: relabel_full(p, z)  # noqa: E731
        norm = pick_env.normalize_action
        demo_dir = args.demo_dir or 'baselines/episodes_all'
    print(f'[env] {type(env).__name__} (max_steps={args.train_max_steps}) '
          f'in {time.time() - t0:.1f}s', flush=True)

    ec = args.ent_coef if args.ent_coef == 'auto' else float(args.ent_coef)
    model = build_rlpd_model(env, args.seed, args.device, utd=args.utd,
                             gamma=args.gamma, ent_coef=ec)
    paths = sorted(glob.glob(str(REPO / demo_dir / '*.npz')))
    assert paths, f'no npz in {demo_dir}'
    transitions, stats = relabel(paths, env.pick_z)
    mix = DemoMix(model, transitions, norm)
    print(f'[demos] {len(paths)} eps -> {mix.n} transitions in the IMMUTABLE demo '
          f'buffer (50% of every batch) | grants {stats}', flush=True)

    from stable_baselines3.common.callbacks import CheckpointCallback, CallbackList
    from wandb_utils import init_wandb, WandbScalarCallback, VideoEvalCallback
    out = pl.Path(args.out_dir); out.mkdir(parents=True, exist_ok=True)
    run = init_wandb(args, name=args.run_name or out.name, tags=('rlpd',))
    cbs = [CheckpointCallback(save_freq=50_000, save_path=str(out), name_prefix='rlpd'),
           WandbScalarCallback(run)]
    if args.eval_freq:
        cbs.append(VideoEvalCallback(run, out, eval_freq=args.eval_freq,
                                     max_steps=args.eval_max_steps, seed=args.seed,
                                     cartesian=args.cartesian, control=args.control))
    model.learn(total_timesteps=args.steps, log_interval=10, callback=CallbackList(cbs))
    model.save(str(out / 'rlpd_final'))
    if run is not None:
        run.finish()
    print(f'[rlpd] done in {(time.time() - t0)/3600:.1f}h -> {out}/rlpd_final.zip', flush=True)


if __name__ == '__main__':
    main()
