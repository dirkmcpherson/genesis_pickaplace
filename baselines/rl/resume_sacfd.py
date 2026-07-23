"""Resume SACfD pick training from a checkpoint (+N more steps).

SB3 .zip carries weights/optimizer but NOT the replay buffer, so we re-inject the demo
transitions into a fresh buffer (online experience from the prior run is lost -- demos
re-seed it; standard SACfD resume). reset_num_timesteps=False so schedules/logging continue.

Usage: resume_sacfd.py --ckpt <sacfd_final.zip> --steps 150000 --demo-dir <dir> --out-dir <dir>
"""
import os
import argparse, glob, pathlib as pl, sys, time

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', '/home/j/workspace/genesis_pickaplace'))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))
import pick_env  # noqa: E402
import demo_buffer  # noqa: E402
from stable_baselines3 import SAC  # noqa: E402
from stable_baselines3.common.callbacks import CheckpointCallback  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--ckpt', required=True)
ap.add_argument('--steps', type=int, default=150_000, help='ADDITIONAL steps to train')
ap.add_argument('--demo-dir', required=True)
ap.add_argument('--out-dir', required=True)
ap.add_argument('--duplicate', type=int, default=3)
ap.add_argument('--device', default='cuda')
args = ap.parse_args()

t0 = time.time()
env = pick_env.PickOnlyEnv(backend='cpu', max_steps=300)
print(f'[env] built in {time.time() - t0:.1f}s', flush=True)
model = SAC.load(args.ckpt, env=env, device=args.device)
print(f'[resume] loaded {args.ckpt} at num_timesteps={model.num_timesteps}', flush=True)

paths = sorted(glob.glob(str(pl.Path(args.demo_dir) / '*.npz')))
assert paths, f'no npz in {args.demo_dir}'
transitions = demo_buffer.load_demo_transitions(paths, pick_z=env.pick_z)
n_added = demo_buffer.inject_into_replay_buffer(
    model, transitions, action_transform=pick_env.normalize_action, duplicate=args.duplicate)
print(f'[resume] re-injected {n_added} demo transitions '
      f'(buffer pos={model.replay_buffer.pos} full={model.replay_buffer.full})', flush=True)

out = pl.Path(args.out_dir); out.mkdir(parents=True, exist_ok=True)
cb = CheckpointCallback(save_freq=25_000, save_path=str(out), name_prefix='sacfd')
target = model.num_timesteps + args.steps
model.learn(total_timesteps=target, reset_num_timesteps=False, log_interval=10, callback=cb)
model.save(str(out / 'sacfd_final'))
print(f'[resume] done at num_timesteps={model.num_timesteps} -> {out}/sacfd_final.zip', flush=True)
