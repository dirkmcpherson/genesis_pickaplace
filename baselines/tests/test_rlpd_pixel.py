"""Offline checks of the {RLPD} PIXEL path (baselines/rl/rlpd_pixel.py) -- NO Genesis, CPU only.

What is pinned here (lane PXR-1, 2026-09-13):
  1. the DrQ-v2 wiring: the actor SEES the critic's encoder (same module), the encoder is in the
     CRITIC optimizer and not in the actor's, the target critic has its own copy;
  2. random_shift: per-sample integer offsets, replicate padding (a constant frame is invariant,
     a ramp moves by whole pixels <= 4), range preserved;
  3. PixelDemoData: channel-first frames, next_obs = the following row of the same tape;
  4. two real RLPDSAC.train() calls on a dict-obs env: 50/50 batches, UTD honoured, the first-batch
     stamps fire, the augmented batch differs from the raw one, the encoder moves under the critic loss;
  5. save -> SAC.load round trip rebuilds PixelRLPDPolicy with the same wiring and the same action.

Run (stable-baselines3 lives in .venv-eval, not in the sim2real venv -- the test SKIPS where it is absent):
  .venv-eval/bin/python baselines/tests/test_rlpd_pixel.py
  .venv-eval/bin/python -m pytest baselines/tests/test_rlpd_pixel.py -q
"""
import os
import sys
import tempfile
import pathlib as pl

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', pl.Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(REPO / 'baselines')); sys.path.insert(0, str(REPO / 'baselines' / 'rl'))

try:
    import pytest
    pytest.importorskip('stable_baselines3')
    pytest.importorskip('gymnasium')
except ImportError:            # run as a plain script: no pytest, but sb3 may still be present
    import stable_baselines3  # noqa: F401

import torch as th
import gymnasium as gym
from gymnasium import spaces

from rlpd_pixel import (make_rlpd_pixel, random_shift, PixelDemoData, PixelRLPDPolicy, PixelRLPDSAC,
                        IMG_SHAPE, PROPRIO_DIM)

DEV = 'cpu'


class FakePixEnv(gym.Env):
    """Dict-obs env with the pixel path's spaces and random non-blank frames; no physics."""

    def __init__(self):
        self.observation_space = spaces.Dict({'image': spaces.Box(0, 255, IMG_SHAPE, np.uint8),
                                              'proprio': spaces.Box(-np.inf, np.inf, (PROPRIO_DIM,), np.float32)})
        self.action_space = spaces.Box(-1, 1, (7,), np.float32)
        self.t = 0
        self._rng = np.random.default_rng(0)

    def _o(self):
        return {'image': self._rng.integers(1, 255, IMG_SHAPE, dtype=np.uint8),
                'proprio': self._rng.standard_normal(PROPRIO_DIM).astype(np.float32)}

    def reset(self, seed=None, options=None):
        self.t = 0
        return self._o(), {}

    def step(self, a):
        self.t += 1
        return self._o(), 0.0, False, self.t >= 20, {}


def _model(buffer_size=500):
    th.manual_seed(0)
    m = make_rlpd_pixel(FakePixEnv(), seed=0, device=DEV, gamma=0.99, buffer_size=buffer_size)
    m.learning_starts = 40
    return m


def _demo(seed=0):
    rng = np.random.default_rng(seed)
    frames = rng.integers(1, 255, (30,) + IMG_SHAPE, dtype=np.uint8)
    prop = rng.standard_normal((30, PROPRIO_DIM)).astype(np.float32)
    obs_idx = np.concatenate([np.arange(9), 10 + np.arange(9), 20 + np.arange(9)])   # 3 tapes of T=10
    return frames, PixelDemoData(frames, prop, obs_idx, rng.uniform(-1, 1, (27, 7)),
                                 np.array([0.0] * 26 + [10.0]), np.zeros(27), th.device(DEV), seed=0)


def test_1_drqv2_wiring():
    m = _model()
    w = m.policy.encoder_wiring()
    assert isinstance(m.policy, PixelRLPDPolicy) and isinstance(m, PixelRLPDSAC)
    assert w['actor_sees_critic_encoder'] and w['encoder_in_critic_optimizer']
    assert not w['encoder_in_actor_optimizer'] and w['target_encoder_separate']
    assert m.policy.critic.features_extractor.features_dim == 50 + PROPRIO_DIM
    # SB3 transposed the image space: the policy is channel-first
    assert tuple(m.observation_space['image'].shape) == (IMG_SHAPE[2],) + IMG_SHAPE[:2]


def test_2_random_shift():
    th.manual_seed(1)
    x = th.randint(0, 256, (16, 6, 64, 64), dtype=th.uint8)
    y = random_shift(x)
    assert y.shape == x.shape and y.dtype == th.float32 and float(y.min()) >= 0 and float(y.max()) <= 255
    d = (y - x.float()).abs().reshape(16, -1).max(dim=1).values
    assert float((d > 0).float().mean()) > 0.5
    c = th.full((2, 6, 64, 64), 77, dtype=th.uint8)
    assert th.equal(random_shift(c), c.float()), 'a constant frame must be invariant under replicate pad + crop'
    ramp = th.arange(64).float()[None, None, :, None].expand(1, 1, 64, 64).clone()
    r = random_shift(ramp)
    diff = (r[0, 0, 8:-8, 32] - ramp[0, 0, 8:-8, 32]).unique()
    assert diff.numel() == 1 and abs(float(diff.item())) <= 4, 'a ramp must move by one whole-pixel offset <= pad'


def test_3_pixel_demo_data():
    frames, demo = _demo()
    s = demo.sample(32)
    assert s.observations['image'].shape == (32, 6, 64, 64) and s.observations['image'].dtype == th.uint8
    assert s.next_observations['proprio'].shape == (32, PROPRIO_DIM) and s.rewards.shape == (32, 1)
    i = 5; fi = int(demo.obs_idx[i])
    assert th.equal(demo.frames[fi + 1], th.as_tensor(frames[fi + 1].transpose(2, 0, 1)))
    assert demo.n == 27 and demo.n_frames == 30 and demo.n_rewarded == 1


def test_4_train_updates_encoder_through_critic_and_augments(capsys=None):
    m = _model()
    _, demo = _demo()
    m.set_demo_data(demo)
    m.learn(total_timesteps=48, log_interval=10 ** 6)
    lv = m.logger.name_to_value
    assert int(lv['train/utd']) == 10 and abs(lv['train/demo_frac'] - 0.5) < 1e-9
    assert int(lv['train/n_updates']) == 10 * (m.num_timesteps - m.learning_starts)
    assert m._aug_stamped, 'the first-batch stamps (image non-zero, aug differs) did not run'
    enc0 = [p.detach().clone() for p in m.policy.critic.features_extractor.parameters()]
    m.train(gradient_steps=3, batch_size=256)
    moved = sum(float((a - b).abs().max()) > 0 for a, b in zip(enc0, m.policy.critic.features_extractor.parameters()))
    assert moved == len(enc0), f'critic loss moved {moved}/{len(enc0)} encoder tensors'
    assert m.policy.actor.features_extractor is m.policy.critic.features_extractor


def test_5_save_load_round_trip():
    m = _model()
    _, demo = _demo()
    m.set_demo_data(demo)
    m.learn(total_timesteps=44, log_interval=10 ** 6)
    from stable_baselines3 import SAC
    o = FakePixEnv()._o()
    with tempfile.TemporaryDirectory() as td:
        m.save(f'{td}/m')
        m2 = SAC.load(f'{td}/m', device=DEV)
    assert isinstance(m2.policy, PixelRLPDPolicy) and m2.policy.encoder_wiring()['actor_sees_critic_encoder']
    for (k1, v1), (k2, v2) in zip(m.policy.state_dict().items(), m2.policy.state_dict().items()):
        assert k1 == k2 and th.allclose(v1, v2), k1
    a1, _ = m2.predict(o, deterministic=True); a2, _ = m.predict(o, deterministic=True)
    assert a1.shape == (7,) and float(np.abs(a1 - a2).max()) < 1e-5


if __name__ == '__main__':
    import inspect
    fns = [f for n, f in sorted(globals().items()) if n.startswith('test_') and inspect.isfunction(f)]
    for f in fns:
        f(); print('PASS', f.__name__)
    print(f'{len(fns)} passed')
