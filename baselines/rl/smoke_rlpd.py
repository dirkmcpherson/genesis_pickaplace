"""RLPD smoke on the REAL FullTaskEnv(delta_joint) + human demos. Runs a short train,
reads the SB3 logger directly (pick-scope episodes are ~900 steps so the periodic
table rarely dumps), asserts the six checks, and saves a checkpoint for the
fresh-process eval round-trip. Not a training entry point -- a CI-style gate."""
import os, glob, pathlib as pl, sys
import numpy as np, torch as th

REPO = pl.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))

from full_env import FullTaskEnv
from train_sacfd_full import delta_encode_transitions
from rlpd_sac import make_rlpd, DemoData, EnsembleCritic
import torch.nn as nn

OUT = REPO / 'baselines/rl/checkpoints/rlpd_smoke'
OUT.mkdir(parents=True, exist_ok=True)
UTD = 10

env = FullTaskEnv(backend='cpu', max_steps=900, scope='pick', action_mode='delta_joint')
model = make_rlpd(env, seed=0, device='cuda', ensemble_size=10, subset_size=2,
                  utd=UTD, gamma=0.998, ent_coef='auto', demo_batch=128)
model.learning_starts = 200            # smoke: engage training quickly
paths = sorted(glob.glob(str(REPO / 'baselines/episodes_pick_phase_all/*.npz')))
trans = delta_encode_transitions(paths, env.pick_z, 'pick', env.delta_cap)
model.set_demo_data(DemoData(trans, None, th.device('cuda')))

# (c) critic LayerNorm + ensemble active
crit = model.critic
n_ln = sum(isinstance(m, nn.LayerNorm) for m in crit.qnet)
n_ens = sum(1 for m in crit.qnet if type(m).__name__ == 'EnsembleLinear')
print(f'[C] critic={type(crit).__name__} n_critics={crit.n_critics} '
      f'LayerNorm_layers={n_ln} EnsembleLinear_layers={n_ens}')
assert isinstance(crit, EnsembleCritic) and crit.n_critics == 10 and n_ln == 2

model.learn(total_timesteps=900, log_interval=10000)
lv = model.logger.name_to_value
g = lambda k: lv.get(k, float('nan'))
print(f'[A] demo_frac={g("train/demo_frac"):.3f} online_bs={int(g("train/online_bs"))} '
      f'demo_bs={int(g("train/demo_bs"))} (sum={int(g("train/online_bs"))+int(g("train/demo_bs"))})')
print(f'[B] demo_rew_per_batch={g("train/demo_rew_per_batch"):.2f}')
print(f'[D] utd(grad steps/env step)={int(g("train/utd"))}')
print(f'[F] critic_loss={g("train/critic_loss"):.4f} actor_loss={g("train/actor_loss"):.4f} '
      f'ent_coef={g("train/ent_coef"):.4f} actor_q_mean={g("train/actor_q_mean"):.3f} '
      f'n_updates={int(g("train/n_updates"))}')

assert abs(g('train/demo_frac') - 0.5) < 1e-9
assert int(g('train/online_bs')) == 128 and int(g('train/demo_bs')) == 128
assert g('train/demo_rew_per_batch') > 0
assert int(g('train/utd')) == UTD
for k in ('train/critic_loss', 'train/actor_loss', 'train/ent_coef'):
    assert np.isfinite(g(k)), f'{k} not finite'
# UTD ratio: n_updates == UTD * (num_timesteps - learning_starts)
exp_updates = UTD * (model.num_timesteps - model.learning_starts)
print(f'[D] n_updates={int(g("train/n_updates"))} == UTD*(steps-starts)={exp_updates}? '
      f'{int(g("train/n_updates")) == exp_updates}')

# (e) checkpoint save; load round-trip is verified in a FRESH process via wandb_eval
model.save(str(OUT / 'rlpd_final'))
(OUT / 'rlpd_final.action_mode.json').write_text('{"action_mode": "delta_joint"}')
print(f'[E] saved {OUT}/rlpd_final.zip (+ action_mode sidecar)')
print('SMOKE-CORE PASS')
