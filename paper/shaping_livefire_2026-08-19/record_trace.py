"""Record one FullTaskEnv trace (one process per env: Genesis single-init).

Usage: record_trace.py <out.npz> plain | shaped <gamma>
"""
import sys, os
import numpy as np

sys.path.insert(0, os.path.expanduser('~/workspace/genesis_pickaplace/baselines/rl'))
from full_env import FullTaskEnv  # noqa: E402

out = sys.argv[1]
mode = sys.argv[2]
N_STEPS = 60

kw = dict(backend='cpu', max_steps=10 ** 9, scope='pick',
          action_mode='delta_joint', delta_cap=0.025,
          delta_leash_mult=5.0, action_repeat=4)
if mode == 'shaped':
    kw.update(pick_shaping=True, pick_shaping_gamma=float(sys.argv[3]))

env = FullTaskEnv(**kw)
env.reset(seed=123)
acts = np.random.RandomState(7).uniform(
    -1, 1, size=(N_STEPS, env.action_space.shape[0])).astype(np.float32)

rews, phis_before, phis_after, states, dones = [], [], [], [], []
for t in range(N_STEPS):
    phis_before.append(env._pick_phi())
    obs, r, terminated, truncated, info = env.step(acts[t])
    done = bool(terminated or truncated)
    phis_after.append(env._pick_phi())
    rews.append(float(r))
    states.append(np.asarray(obs, dtype=np.float64).ravel())
    dones.append(bool(done))
    if done:
        break

np.savez(out, rews=np.array(rews), phis_before=np.array(phis_before),
         phis_after=np.array(phis_after), states=np.array(states),
         dones=np.array(dones))
print(f'{mode} {sys.argv[3] if mode == "shaped" else ""}: {len(rews)} steps -> {out}')
