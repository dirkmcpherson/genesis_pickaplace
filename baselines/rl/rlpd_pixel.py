"""PIXEL observation path for {RLPD} (lane PXR-1, 2026-09-13).

Gives train_rlpd.py `--obs pixels`: the SAME recipe of record (rlpd_sac: UTD 10, E10/Z2,
LayerNorm ensemble critics, 50/50 demo batches, gamma from the launcher, delta_joint /
repeat 4, the FullTaskEnv ladder and tip guard) with the 17-dim state replaced by

    obs = {'image':   uint8 (64, 64, 6)  = GenesisCanEnv.rig_obs(): top RGB ++ wrist RGB,
           'proprio': float32 (8,)       = state[:8] = q[:6], gripper motor, grip effort}

i.e. NO can pose and NO goal pose -- exactly what the pixel world model
(r2dreamer `env=genesis_full_pixel`, `state_slice: 8`, `image_aug: shift4`) sees.

Pieces (all new, nothing here is imported by a state run):
  PixelObsWrapper        gym.Wrapper over a FullTaskEnv built with camera_rig=True; renders
                         rig_obs() ONCE after reset and ONCE after every decision -- the same
                         call at the same point in the loop as the world-model adapter
                         (~/workspace/r2dreamer/envs/genesis.py `_image`) and the `_img` demo
                         sets (relabel_reward.py --images), so demo and online frames are one
                         observation function.
  random_shift           DrQ random shift: replicate-pad 4 px, random integer crop back to
                         64x64, one offset per SAMPLE (obs and next_obs shifted independently).
  DrQEncoder             DrQ-v2 conv stack (4 x Conv3x3, 32 ch, stride 2/1/1/1) + Linear ->
                         LayerNorm -> Tanh trunk (feature_dim 50).
  PixelProprioExtractor  SB3 features extractor: cat(DrQEncoder(image), proprio) -> 58 dims.
  DetachedFeaturesActor  SB3 SAC actor whose features are computed under no_grad.
  PixelRLPDPolicy        RLPDPolicy wired DrQ-v2 style: the CRITIC owns the encoder and trains
                         it through the critic loss; the actor SEES the same encoder module
                         but detached, and its optimizer excludes the encoder. The target
                         critic carries its own polyak-averaged copy (DrQ-v1 / SAC-AE target
                         encoder, tau = the critic tau 0.005).
  PixelDemoData          the immutable demo half, frames stored ONCE per tape row (uint8) and
                         gathered by index (obs = frame[t], next_obs = frame[t+1]).
  PixelRLPDSAC           RLPDSAC whose _prep_batch hook applies random_shift to the image of
                         the concatenated online++demo batch (obs and next_obs) and stamps /
                         asserts, on its first call, that the augmented batch differs from the
                         raw one and that the ONLINE half holds real (non-zero) frames.
  make_rlpd_pixel        make_rlpd with the pixel classes (one list of pinned hypers).

SB3 plumbing this relies on (stable-baselines3 2.8, asserted by rlpd_sac):
  * a Dict observation space with a (64,64,6) uint8 Box is an "image space" (channels are not
    checked), so BaseAlgorithm wraps the env in VecTransposeImage -> the replay buffer, the
    policy and the saved observation_space are CHANNEL-FIRST (6,64,64); `predict()` on a raw
    (64,64,6) env observation transposes it back (`maybe_transpose`). PixelDemoData therefore
    stores frames channel-first too.
  * preprocess_obs divides an image Box by 255 inside extract_features; the encoder receives
    floats in [0,1] and subtracts 0.5 (DrQ-v2).
  * SAC.load rebuilds the policy from policy_class + policy_kwargs, so every class here lives
    in this MODULE (never __main__) and baselines/rl must be on sys.path in the evaluator.
"""
import numpy as np
import torch as th
import torch.nn as nn
import torch.nn.functional as F

import gymnasium as gym
from gymnasium import spaces
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.type_aliases import DictReplayBufferSamples
from stable_baselines3.sac.policies import Actor

from rlpd_sac import RLPDPolicy, RLPDSAC, make_rlpd

IMG_SHAPE = (64, 64, 6)      # (H, W, C) as the env emits it: top RGB ++ wrist RGB
PROPRIO_DIM = 8              # state[:8] = q[:6], gripper motor (0..1), grip effort
SHIFT_PAD = 4                # DrQ random-shift padding (pixels)
CNN_FEATURE_DIM = 50         # DrQ-v2 trunk width


# ------------------------------------------------------------------------------------ env side
class PixelObsWrapper(gym.Wrapper):
    """{'image': rig_obs() uint8 (64,64,6), 'proprio': state[:8] float32} over a FullTaskEnv.

    The base env must have been built with camera_rig=True (asserted) and, like every state
    run, with sim_variant_hook.apply_pre/apply_post around it -- this wrapper renders, it does
    not build. Everything the trainer/evaluator read off the env (scope, ladder, action_repeat,
    _granted, terminal_stages, provenance, ...) is read off `.base`; no attribute magic
    (gymnasium 1.x wrappers forward nothing)."""

    def __init__(self, env):
        super().__init__(env)
        assert getattr(env.genv, 'camera_rig', False) is True, (
            'PixelObsWrapper needs FullTaskEnv(camera_rig=True): rig_obs() renders cam_top/cam_wrist')
        assert env.scope == 'full', f'the pixel path is scope=full only (got {env.scope!r})'
        assert env.observation_space.shape[0] >= PROPRIO_DIM, env.observation_space
        self.observation_space = spaces.Dict({
            'image': spaces.Box(0, 255, IMG_SHAPE, dtype=np.uint8),
            'proprio': spaces.Box(-np.inf, np.inf, (PROPRIO_DIM,), dtype=np.float32),
        })
        self.action_space = env.action_space
        self.n_renders = 0

    @property
    def base(self):
        return self.env

    @property
    def genv(self):
        return self.env.genv

    def observe(self, state):
        """The ONE observation function: render the rig now, take the first 8 state dims."""
        img = np.asarray(self.env.genv.rig_obs(), dtype=np.uint8)
        assert img.shape == IMG_SHAPE, img.shape
        self.n_renders += 1
        return {'image': img, 'proprio': np.asarray(state, np.float32)[:PROPRIO_DIM].copy()}

    def reset(self, seed=None, options=None):
        state, info = self.env.reset(seed=seed, options=options)
        return self.observe(state), info

    def reset_to(self, ic):
        state, info = self.env.reset_to(ic)
        return self.observe(state), info

    def step(self, action):
        state, r, term, trunc, info = self.env.step(action)
        return self.observe(state), r, term, trunc, info


# ------------------------------------------------------------------------------ augmentation
def random_shift(x, pad=SHIFT_PAD):
    """DrQ random shift on a (B, C, H, W) batch: replicate-pad `pad` px, then crop back to
    (H, W) at a random integer offset drawn independently PER SAMPLE. Returns float32 in the
    input's value range (uint8 in -> [0, 255] float out; preprocess_obs divides by 255 later).
    Offsets come from torch's global RNG (seeded by SB3 set_random_seed)."""
    assert x.dim() == 4, x.shape
    B, C, H, W = x.shape
    xp = F.pad(x.float(), (pad, pad, pad, pad), mode='replicate')
    dy = th.randint(0, 2 * pad + 1, (B,), device=x.device)
    dx = th.randint(0, 2 * pad + 1, (B,), device=x.device)
    rows = th.arange(H, device=x.device)[None, :] + dy[:, None]          # (B, H)
    cols = th.arange(W, device=x.device)[None, :] + dx[:, None]          # (B, W)
    b = th.arange(B, device=x.device)[:, None, None, None]
    c = th.arange(C, device=x.device)[None, :, None, None]
    return xp[b, c, rows[:, None, :, None], cols[:, None, None, :]]


# ---------------------------------------------------------------------------------- networks
class DrQEncoder(nn.Module):
    """DrQ-v2 encoder + trunk for a (C, 64, 64) input in [0, 1]:
    Conv(C->32, 3, s2) ReLU, 3 x [Conv(32->32, 3, s1) ReLU], flatten (32*25*25 = 20000),
    Linear -> feature_dim, LayerNorm, Tanh. The trunk is SHARED by actor and critic here
    (DrQ-v2 gives each its own trunk over the shared convs; see the doc, §deviations)."""

    def __init__(self, in_channels, feature_dim=CNN_FEATURE_DIM, hw=64):
        super().__init__()
        self.convs = nn.Sequential(
            nn.Conv2d(in_channels, 32, 3, stride=2), nn.ReLU(),
            nn.Conv2d(32, 32, 3, stride=1), nn.ReLU(),
            nn.Conv2d(32, 32, 3, stride=1), nn.ReLU(),
            nn.Conv2d(32, 32, 3, stride=1), nn.ReLU())
        with th.no_grad():
            n_flat = int(self.convs(th.zeros(1, in_channels, hw, hw)).numel())
        self.n_flat = n_flat
        self.trunk = nn.Sequential(nn.Linear(n_flat, feature_dim), nn.LayerNorm(feature_dim), nn.Tanh())
        self.feature_dim = int(feature_dim)

    def forward(self, img01):                       # (B, C, H, W) in [0, 1]
        h = self.convs(img01 - 0.5)
        return self.trunk(h.reshape(h.shape[0], -1))


class PixelProprioExtractor(BaseFeaturesExtractor):
    """SB3 features extractor for the {'image', 'proprio'} Dict space (CHANNEL-FIRST image, as
    VecTransposeImage hands it over): cat(DrQEncoder(image), proprio). The raw proprio bypasses
    the encoder so the actor's gradient reaches its own first layer on it."""

    def __init__(self, observation_space, cnn_feature_dim=CNN_FEATURE_DIM):
        assert isinstance(observation_space, spaces.Dict) and set(observation_space.spaces) == {'image', 'proprio'}, \
            observation_space
        img = observation_space['image']; pro = observation_space['proprio']
        assert img.dtype == np.uint8 and len(img.shape) == 3, img
        # channel-first: SB3 transposes (64,64,6) -> (6,64,64) before the policy ever sees it
        assert img.shape[0] == IMG_SHAPE[2] and img.shape[1:] == IMG_SHAPE[:2], (
            f'expected channel-first {(IMG_SHAPE[2],) + IMG_SHAPE[:2]}, got {img.shape}')
        assert pro.shape == (PROPRIO_DIM,), pro.shape
        super().__init__(observation_space, features_dim=int(cnn_feature_dim) + PROPRIO_DIM)
        self.cnn = DrQEncoder(img.shape[0], feature_dim=int(cnn_feature_dim), hw=img.shape[1])

    def forward(self, obs):
        return th.cat([self.cnn(obs['image']), obs['proprio']], dim=1)


class DetachedFeaturesActor(Actor):
    """SAC actor whose features come from the (critic-owned) encoder under no_grad, so the
    actor loss never trains the encoder (DrQ-v2)."""

    def extract_features(self, obs, features_extractor):
        with th.no_grad():
            return super().extract_features(obs, features_extractor)


class PixelRLPDPolicy(RLPDPolicy):
    """RLPDPolicy with the DrQ-v2 encoder wiring. SB3's own `share_features_extractor=True`
    does the OPPOSITE of DrQ-v2 (the actor optimizer owns the shared extractor and the critic
    runs it without grad), so it must stay False here and the wiring is done in _build."""

    def make_actor(self, features_extractor=None):
        actor_kwargs = self._update_features_extractor(self.actor_kwargs, features_extractor)
        return DetachedFeaturesActor(**actor_kwargs).to(self.device)

    def _build(self, lr_schedule):
        assert not self.share_features_extractor, (
            'PixelRLPDPolicy wires the shared encoder itself; SB3 share_features_extractor must be False')
        # the critic OWNS the encoder: its optimizer covers every critic parameter incl. the
        # extractor, and EnsembleCritic.forward runs the extractor with grad (share=False)
        self.critic = self.make_critic(features_extractor=None)
        # the actor SEES the critic's encoder (same module object), detached by the actor class
        self.actor = self.make_actor(features_extractor=self.critic.features_extractor)
        actor_params = [p for n, p in self.actor.named_parameters() if not n.startswith('features_extractor.')]
        assert actor_params and len(actor_params) < len(list(self.actor.named_parameters())), 'actor param split failed'
        self.actor.optimizer = self.optimizer_class(actor_params, lr=lr_schedule(1), **self.optimizer_kwargs)
        # target critic: its own encoder copy, polyak-averaged with the rest of the critic
        self.critic_target = self.make_critic(features_extractor=None)
        self.critic_target.load_state_dict(self.critic.state_dict())
        self.critic.optimizer = self.optimizer_class(self.critic.parameters(), lr=lr_schedule(1),
                                                     **self.optimizer_kwargs)
        self.critic_target.set_training_mode(False)

    def encoder_wiring(self):
        """Stamp: proves the wiring at runtime (printed by the trainer)."""
        shared = self.actor.features_extractor is self.critic.features_extractor
        actor_ids = {id(p) for g in self.actor.optimizer.param_groups for p in g['params']}
        critic_ids = {id(p) for g in self.critic.optimizer.param_groups for p in g['params']}
        enc_ids = {id(p) for p in self.critic.features_extractor.parameters()}
        return dict(actor_sees_critic_encoder=shared,
                    encoder_in_actor_optimizer=bool(enc_ids & actor_ids),
                    encoder_in_critic_optimizer=enc_ids <= critic_ids,
                    target_encoder_separate=(self.critic_target.features_extractor is not self.critic.features_extractor),
                    n_encoder_params=int(sum(p.numel() for p in self.critic.features_extractor.parameters())))


# --------------------------------------------------------------------------------- demo half
class PixelDemoData:
    """Immutable pixel demo buffer, permanently half of every batch (rlpd_sac.DemoData with the
    frames stored once). frames: (F, 64, 64, 6) uint8 = every tape's rows concatenated in the
    loader's order; proprio: (F, 8); obs_idx[i] = the frame row of transition i, whose
    next_obs is row obs_idx[i] + 1 (tapes are contiguous, transitions are t -> t+1)."""

    def __init__(self, frames, proprio, obs_idx, actions, rewards, dones, device, seed):
        frames = np.asarray(frames, np.uint8); proprio = np.asarray(proprio, np.float32)
        obs_idx = np.asarray(obs_idx, np.int64)
        assert frames.shape[1:] == IMG_SHAPE and proprio.shape == (frames.shape[0], PROPRIO_DIM), (frames.shape, proprio.shape)
        assert obs_idx.max() + 1 < frames.shape[0] and obs_idx.min() >= 0, (obs_idx.min(), obs_idx.max(), frames.shape)
        self.device = device
        pin = device.type == 'cuda'

        def _t(a):
            x = th.as_tensor(a)
            return x.pin_memory() if pin else x
        # channel-first, as VecTransposeImage stores the online frames
        self.frames = _t(np.ascontiguousarray(frames.transpose(0, 3, 1, 2)))
        self.proprio = _t(proprio)
        self.obs_idx = th.as_tensor(obs_idx)
        self.actions = _t(np.asarray(actions, np.float32))
        self.rewards = _t(np.asarray(rewards, np.float32).reshape(-1, 1))
        self.dones = _t(np.asarray(dones, np.float32).reshape(-1, 1))
        self.n = int(obs_idx.shape[0])
        self.n_frames = int(frames.shape[0])
        self.n_rewarded = int((np.asarray(rewards) > 0).sum())
        self.bytes = int(self.frames.numel() + self.proprio.numel() * 4)
        self._g = th.Generator().manual_seed(int(seed))
        self._pin = pin
        self._bufs = {}

    def _gather_frames(self, fi, key):
        if not self._pin:
            return self.frames[fi]
        buf = self._bufs.get((key, int(fi.shape[0])))
        if buf is None:
            buf = th.empty((int(fi.shape[0]),) + tuple(self.frames.shape[1:]), dtype=th.uint8).pin_memory()
            self._bufs[(key, int(fi.shape[0]))] = buf
        th.index_select(self.frames, 0, fi, out=buf)
        return buf

    def sample(self, batch_size):
        idx = th.randint(0, self.n, (batch_size,), generator=self._g)
        fi = self.obs_idx[idx]
        to = dict(device=self.device, non_blocking=True)
        obs = {'image': self._gather_frames(fi, 'o').to(**to), 'proprio': self.proprio[fi].to(**to)}
        nobs = {'image': self._gather_frames(fi + 1, 'n').to(**to), 'proprio': self.proprio[fi + 1].to(**to)}
        return DictReplayBufferSamples(
            observations=obs, actions=self.actions[idx].to(**to), next_observations=nobs,
            dones=self.dones[idx].to(**to), rewards=self.rewards[idx].to(**to))


# --------------------------------------------------------------------------------- algorithm
class PixelRLPDSAC(RLPDSAC):
    """RLPDSAC + DrQ random shift on the image half of every update batch (obs and next_obs,
    online and demo alike -- the hook sees the concatenated batch). `image_aug` is a
    constructor argument recorded in the sidecar, never an env var."""

    def __init__(self, *args, image_aug='shift4', **kwargs):
        assert image_aug in ('shift4', 'none'), image_aug
        self.image_aug = str(image_aug)
        self._aug_stamped = False
        super().__init__(*args, **kwargs)

    def _prep_batch(self, obs, nobs):
        assert isinstance(obs, dict) and 'image' in obs, 'PixelRLPDSAC needs the {image, proprio} observation'
        if not self._aug_stamped:
            self._stamp_batch(obs)
        if self.image_aug == 'none':
            return obs, nobs
        obs = dict(obs); nobs = dict(nobs)
        raw = obs['image']
        obs['image'] = random_shift(raw, SHIFT_PAD)
        nobs['image'] = random_shift(nobs['image'], SHIFT_PAD)
        if not self._aug_stamped:
            self._stamp_aug(raw, obs['image'])
        self._aug_stamped = True
        return obs, nobs

    def _stamp_batch(self, obs):
        online_bs = int(self.batch_size - self.demo_batch)
        img = obs['image']
        on, de = img[:online_bs], img[online_bs:]
        stat = lambda x: (int(x.min()), int(x.max()), float(x.float().mean()),
                          float((x.reshape(x.shape[0], -1).max(dim=1).values > 0).float().mean()))
        so, sd = stat(on), stat(de)
        print(f'[image] first update batch: shape={tuple(img.shape)} dtype={img.dtype} '
              f'ONLINE({online_bs}) min={so[0]} max={so[1]} mean={so[2]:.2f} nonzero_frames={so[3]:.3f} | '
              f'DEMO({de.shape[0]}) min={sd[0]} max={sd[1]} mean={sd[2]:.2f} nonzero_frames={sd[3]:.3f}', flush=True)
        assert so[3] == 1.0 and so[1] > 0, 'ONLINE replay frames are blank -- the rig did not render into the buffer'
        assert sd[3] == 1.0 and sd[1] > 0, 'DEMO frames are blank -- a placeholder (state-only) set was loaded'

    def _stamp_aug(self, raw, aug):
        rawf = raw.float()
        d = (aug - rawf).abs()
        changed = float((d.reshape(d.shape[0], -1).max(dim=1).values > 0).float().mean())
        print(f'[image_aug] {self.image_aug} pad={SHIFT_PAD}: max|aug-raw|={float(d.max()):.1f}/255 '
              f'frames_changed={changed:.3f} aug_mean={float(aug.mean()):.2f} raw_mean={float(rawf.mean()):.2f} '
              f'shape={tuple(aug.shape)}  [the augmented batch differs from the raw batch]', flush=True)
        # one zero-offset draw per sample has probability 1/81; a whole batch of them is impossible
        assert changed > 0.5 and float(d.max()) > 0, 'random_shift left the batch unchanged'
        self._aug_stamped = True


def make_rlpd_pixel(env, seed, device, *, image_aug='shift4', buffer_size=300_000,
                    cnn_feature_dim=CNN_FEATURE_DIM, **kw):
    """make_rlpd with the pixel policy/algorithm; every other pinned hyper is make_rlpd's."""
    return make_rlpd(env, seed, device,
                     policy_class=PixelRLPDPolicy, algo_class=PixelRLPDSAC, buffer_size=buffer_size,
                     policy_kwargs_extra=dict(features_extractor_class=PixelProprioExtractor,
                                              features_extractor_kwargs=dict(cnn_feature_dim=int(cnn_feature_dim)),
                                              share_features_extractor=False),
                     algo_kwargs_extra=dict(image_aug=image_aug), **kw)
