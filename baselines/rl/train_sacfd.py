"""SACfD (SAC from Demonstrations) for the PICK subtask.

SACfD fidelity notes:
  * NO behavior-cloning / imitation loss anywhere -- that is the point of the
    method. Demonstrations enter ONLY through the replay buffer: they are
    injected as ordinary (s, a, r, s', done) transitions BEFORE learning
    starts, with the pick reward relabeled offline (see demo_buffer.py), and
    SAC's usual off-policy critic/actor updates consume them exactly like
    agent experience.
  * Optional demo oversampling: --duplicate N (default 3, i.e. ON) injects
    each demo transition N times, a cheap stand-in for prioritized demo
    sampling early in training; fresh agent experience progressively dilutes
    them in the 300k FIFO buffer. --duplicate 1 disables it.
  * learning_starts is small (100) because the buffer is pre-seeded with
    thousands of demo transitions -- there is no need for a long random
    warmup, and SAC starts fitting the demo reward signal almost immediately.

Env/action conventions: PickOnlyEnv (baselines/rl/pick_env.py) exposes the
17-dim state and a [-1,1]-normalized 7-dim action space; demo actions are
mapped with pick_env.normalize_action at injection so the buffer, actor, and
critic all share one action space.

Usage (one gs.init per process -- run each mode in a fresh process):
  .venv-eval/bin/python baselines/rl/train_sacfd.py --smoke   # ~1k steps + eval
  .venv-eval/bin/python baselines/rl/train_sacfd.py --full    # 200k steps + ckpts
"""
import argparse
import pathlib as pl
import sys
import time

import numpy as np

RL_DIR = pl.Path(__file__).resolve().parent
sys.path.insert(0, str(RL_DIR))

import demo_buffer  # noqa: E402
import pick_env  # noqa: E402

CKPT_DIR = RL_DIR / 'checkpoints'


def build_model(env, seed, device):
    from stable_baselines3 import SAC
    return SAC(
        'MlpPolicy', env,
        learning_rate=3e-4,
        buffer_size=300_000,
        learning_starts=100,      # small: buffer is pre-seeded with demos
        batch_size=256,
        tau=0.005,
        gamma=0.98,               # short-horizon sparse task
        train_freq=1,
        gradient_steps=1,
        ent_coef='auto',
        seed=seed,
        device=device,
        verbose=1,
    )


def evaluate(model, env, n_episodes=3, fixed_uid=None):
    """Greedy (deterministic) rollouts on the training env instance."""
    results = []
    for ep in range(n_episodes):
        options = {'uid': fixed_uid} if fixed_uid is not None else None
        obs, info = env.reset(seed=1000 + ep, options=options)
        uid = info['uid']
        picked, steps = False, 0
        for _ in range(env.max_steps):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            steps += 1
            if terminated:
                picked = True
                break
            if truncated:
                break
        results.append((uid, picked, steps))
        print(f'  eval ep {ep}: uid={uid} picked={picked} steps={steps}')
    n_pick = sum(1 for _, p, _ in results if p)
    print(f'[eval] pick success {n_pick}/{n_episodes}')
    return results


def main():
    ap = argparse.ArgumentParser()
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument('--smoke', action='store_true',
                      help='1k-step smoke run + buffer stats + 3-episode eval')
    mode.add_argument('--full', action='store_true',
                      help='200k steps, checkpoints every 25k')
    ap.add_argument('--duplicate', type=int, default=3,
                    help='inject each demo transition N times (1 = off; default 3)')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--device', default='cpu',
                    help="'cpu' default: MLP nets are small and the GPU is "
                         'usually busy with the dataset/training chain')
    ap.add_argument('--eval-uid', type=int, default=None,
                    help='fixed trial uid for eval episodes (default: random success uids)')
    ap.add_argument('--demo-dir', default=None,
                    help='directory of demo *.npz to seed the replay buffer from. When '
                         'set, overrides demo_buffer.find_demo_paths() -- used to inject '
                         'HUMAN vs harvested-AI demos for the demonstration-source study.')
    ap.add_argument('--steps', type=int, default=200_000,
                    help='total timesteps for --full (default 200k)')
    ap.add_argument('--out-dir', default=None,
                    help='checkpoint output dir (default baselines/rl/checkpoints). Set a '
                         'per-run dir when sweeping seeds/conditions so runs do not clobber.')
    args = ap.parse_args()
    ckpt_dir = pl.Path(args.out_dir) if args.out_dir else CKPT_DIR

    t0 = time.time()
    # Build env ONCE per process (gs.init constraint), CPU backend.
    env = pick_env.PickOnlyEnv(backend='cpu', max_steps=300)
    print(f'[env] built in {time.time() - t0:.1f}s | pick_z={env.pick_z:.4f} | '
          f'{len(env.success_uids)} success-labeled trials')

    model = build_model(env, args.seed, args.device)

    # ---- demo injection BEFORE learning starts (the SACfD step) ----
    if args.demo_dir:
        import glob
        paths = sorted(glob.glob(str(pl.Path(args.demo_dir) / '*.npz')))
        assert paths, f'no *.npz in --demo-dir {args.demo_dir}'
        print(f'[demos] --demo-dir override: {len(paths)} episodes from {args.demo_dir}')
    else:
        paths = demo_buffer.find_demo_paths()
    transitions = demo_buffer.load_demo_transitions(paths, pick_z=env.pick_z)
    n_r1 = sum(1 for t in transitions if t[2] > 0)
    n_added = demo_buffer.inject_into_replay_buffer(
        model, transitions,
        action_transform=pick_env.normalize_action,
        duplicate=args.duplicate)
    print(f'[demos] {len(transitions)} unique transitions ({n_r1} with r=1), '
          f'x{args.duplicate} -> {n_added} added; '
          f'buffer pos={model.replay_buffer.pos} full={model.replay_buffer.full}')

    if args.smoke:
        t1 = time.time()
        model.learn(total_timesteps=1000, log_interval=4)
        t_learn = time.time() - t1
        buf = model.replay_buffer
        stored = buf.buffer_size if buf.full else buf.pos
        print(f'[smoke] learn(1000) done in {t_learn:.1f}s | '
              f'buffer stored={stored} full={buf.full} | '
              f'stored r=1 count={int((buf.rewards[:stored] > 0).sum())}')
        evaluate(model, env, n_episodes=3, fixed_uid=args.eval_uid)
        print(f'[smoke] total wall time {time.time() - t0:.1f}s')
    else:
        from stable_baselines3.common.callbacks import CheckpointCallback
        ckpt_dir.mkdir(parents=True, exist_ok=True)
        cb = CheckpointCallback(save_freq=25_000, save_path=str(ckpt_dir),
                                name_prefix='sacfd')
        model.learn(total_timesteps=args.steps, log_interval=10, callback=cb)
        model.save(str(ckpt_dir / 'sacfd_final'))
        evaluate(model, env, n_episodes=10, fixed_uid=args.eval_uid)


if __name__ == '__main__':
    main()
