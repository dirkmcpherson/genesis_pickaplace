"""Record one r2dreamer GenesisPick adapter trace (one process per env).

Usage: record_r2d_trace.py <out.npz> plain|shaped
Champion-recipe knobs: delta_joint, cap 0.025, leash_mult 5, repeat 4,
reward_scale 100. Shaped adds pick_shaping (adapter-boundary, gamma 0.999).
"""
import sys, os
import numpy as np

sys.path.insert(0, os.path.expanduser('~/workspace/r2dreamer'))
from envs.genesis import GenesisPick  # noqa: E402

out, mode = sys.argv[1], sys.argv[2]
N_STEPS = 40

env = GenesisPick(task='pick', seed=0, scope='pick', action_repeat=4,
                  reward_scale=100.0, action_mode='delta_joint',
                  delta_cap=0.025, delta_leash_mult=5,
                  pick_shaping=(mode == 'shaped'))
env.reset()
acts = np.random.RandomState(11).uniform(-1, 1, size=(N_STEPS, 7)).astype(np.float32)

rews, phis_before, phis_after, imgs, dones = [], [], [], [], []
for t in range(N_STEPS):
    phis_before.append(env._env._pick_phi())
    obs, r, done, info = env.step(acts[t])
    phis_after.append(env._env._pick_phi())
    rews.append(float(r))
    imgs.append(np.asarray(obs['image'], dtype=np.uint8).sum())
    dones.append(bool(done))
    if done:
        break

np.savez(out, rews=np.array(rews), phis_before=np.array(phis_before),
         phis_after=np.array(phis_after), img_sums=np.array(imgs),
         dones=np.array(dones))
print(f'{mode}: {len(rews)} agent steps -> {out}')
