"""RLPD (Ball et al. 2023) on top of SB3 2.8 SAC, for the sparse-reward + prior-data
regime of this repo.

The published recipe, adapted to SB3 (see baselines/rl/RLPD_PLAN.md):
  * symmetric 50/50 online/demo batches (NO behavior-cloning loss -- demos are just
    off-policy data; that is the whole point of the method),
  * an ENSEMBLE of E LayerNorm critics (LN is RLPD's stabilizer for high UTD),
  * per-update Bellman target = min over a random Z-of-E subset of the TARGET critics,
  * high update-to-data ratio (UTD gradient steps per env step) on the CRITIC only,
    with a SINGLE actor+alpha update per env step (last grad step) -- prevents the
    actor/entropy collapse that high-UTD actor updates cause,
  * actor trained against the ENSEMBLE MEAN Q (not the min).

Why NOT the old train_rlpd DemoMix monkey-patch: it rebuilt critics AFTER
construction (LayerNorm inserted post-hoc) and monkey-patched buffer.sample, both of
which break SAC.save/SAC.load round-trips (the eval subprocess reconstructs the policy
in a FRESH process and would get a plain 2-critic MLP with no LayerNorm -> state-dict
mismatch). Here the ensemble+LN critic is built AT CONSTRUCTION by RLPDPolicy, and the
50/50 sampling lives inside RLPDSAC.train() (an explicit two-buffer draw), so the saved
zip round-trips through the stock SAC.load used by wandb_eval.

These classes live in a MODULE (never __main__) so the eval subprocess can unpickle
the policy_class stored in the checkpoint.
"""
import numpy as np
import torch as th
import torch.nn as nn
import torch.nn.functional as F

import stable_baselines3 as sb3
from stable_baselines3 import SAC
from stable_baselines3.common.policies import BaseModel
from stable_baselines3.common.preprocessing import get_action_dim
from stable_baselines3.common.buffers import ReplayBufferSamples
from stable_baselines3.sac.policies import SACPolicy
from stable_baselines3.common.utils import polyak_update

assert sb3.__version__.startswith('2.8'), (
    f'RLPD train() mirrors SB3 2.8.x SAC internals (ent_coef/target/log_prob '
    f'handling); found sb3 {sb3.__version__}. Re-audit before running.')


class EnsembleLinear(nn.Module):
    """E independent Linear layers evaluated in one batched matmul (baddbmm), so an
    E=10 critic ensemble costs ~one Linear's worth of kernel launches instead of ten.

    weight: (E, in, out), bias: (E, 1, out).  forward(x: (E, B, in)) -> (E, B, out).
    Each member is initialised EXACTLY like nn.Linear (kaiming_uniform a=sqrt(5),
    bias U(-1/sqrt(fan_in), ...)), so the ensemble is not degenerate at init.
    """

    def __init__(self, in_features, out_features, ensemble_size):
        super().__init__()
        self.in_features = in_features
        self.out_features = out_features
        self.ensemble_size = ensemble_size
        self.weight = nn.Parameter(th.empty(ensemble_size, in_features, out_features))
        self.bias = nn.Parameter(th.empty(ensemble_size, 1, out_features))
        self.reset_parameters()

    def reset_parameters(self):
        for e in range(self.ensemble_size):
            ref = nn.Linear(self.in_features, self.out_features)  # nn.Linear's init
            self.weight.data[e] = ref.weight.data.t()            # (out,in) -> (in,out)
            self.bias.data[e, 0] = ref.bias.data

    def forward(self, x):                       # x: (E, B, in)
        return th.baddbmm(self.bias, x, self.weight)  # (E, B, out)


class EnsembleLayerNorm(nn.Module):
    """Per-member LayerNorm for an (E, B, h) ensemble stream. nn.LayerNorm(h) in this
    position normalizes each row correctly but SHARES its affine weight/bias across
    all E members (audit bug 2) -- the members' post-norm scales are tied, degrading
    ensemble diversity and with it the min-of-Z pessimism. This module gives each
    member its own (h,) weight/bias. Opt-in via per_member_ln; default off preserves
    byte-identical construction so every pre-existing checkpoint still loads."""

    def __init__(self, ensemble_size, h, eps=1e-5):
        super().__init__()
        self.eps = eps
        self.weight = nn.Parameter(th.ones(ensemble_size, 1, h))
        self.bias = nn.Parameter(th.zeros(ensemble_size, 1, h))

    def forward(self, x):                      # (E, B, h)
        mu = x.mean(dim=-1, keepdim=True)
        var = x.var(dim=-1, keepdim=True, unbiased=False)
        return (x - mu) / th.sqrt(var + self.eps) * self.weight + self.bias


class EnsembleCritic(BaseModel):
    """Drop-in for SB3's ContinuousCritic: same constructor signature (so
    SACPolicy.make_critic can pass critic_kwargs unchanged) but builds ONE vectorized
    ensemble of n_critics LayerNorm Q-nets. LayerNorm follows each HIDDEN Linear only
    (never the scalar output). forward returns a (E, B, 1) tensor -- RLPDSAC.train
    consumes the ensemble axis directly.
    """

    def __init__(self, observation_space, action_space, net_arch, features_extractor,
                 features_dim, activation_fn=nn.ReLU, normalize_images=True,
                 n_critics=10, share_features_extractor=True, per_member_ln=False):
        super().__init__(observation_space, action_space,
                         features_extractor=features_extractor,
                         normalize_images=normalize_images)
        action_dim = get_action_dim(self.action_space)
        self.share_features_extractor = share_features_extractor
        self.n_critics = n_critics
        E = n_critics
        self.per_member_ln = bool(per_member_ln)
        layers, last = [], features_dim + action_dim
        for h in net_arch:
            layers.append(EnsembleLinear(last, h, E))
            # default (False) = original shared-affine nn.LayerNorm -> old ckpts load
            layers.append(EnsembleLayerNorm(E, h) if self.per_member_ln
                          else nn.LayerNorm(h))
            layers.append(activation_fn())
            last = h
        layers.append(EnsembleLinear(last, 1, E))
        self.qnet = nn.Sequential(*layers)

    def forward(self, obs, actions):               # -> (E, B, 1)
        with th.set_grad_enabled(not self.share_features_extractor):
            features = self.extract_features(obs, self.features_extractor)
        qin = th.cat([features, actions], dim=1)   # (B, feat+act)
        qin = qin.unsqueeze(0).expand(self.n_critics, -1, -1)  # (E, B, in)
        return self.qnet(qin)                      # (E, B, 1)


class RLPDPolicy(SACPolicy):
    """SACPolicy whose critic/critic_target are EnsembleCritics. Everything else
    (actor, features extractor, save/load constructor params incl. n_critics) is the
    stock SACPolicy machinery, so `SAC.load(<rlpd checkpoint>)` in a fresh process
    rebuilds the identical LayerNorm ensemble and loads its state dict cleanly.
    per_member_ln travels in the constructor parameters so save/load round-trips
    it; default False keeps every pre-flag checkpoint loadable byte-identically."""

    def __init__(self, *args, per_member_ln=False, **kwargs):
        self.per_member_ln = bool(per_member_ln)
        super().__init__(*args, **kwargs)

    def make_critic(self, features_extractor=None):
        critic_kwargs = self._update_features_extractor(self.critic_kwargs,
                                                        features_extractor)
        return EnsembleCritic(per_member_ln=self.per_member_ln,
                              **critic_kwargs).to(self.device)

    def _get_constructor_parameters(self):
        data = super()._get_constructor_parameters()
        data.update(per_member_ln=self.per_member_ln)
        return data


class DemoData:
    """Immutable, permanently-half-of-every-batch demo buffer (RLPD's core departure
    from seed-once SACfD, where demos dilute/overwrite in the FIFO buffer). Tensors are
    built once, pinned when training on CUDA for a fast async host->device copy."""

    def __init__(self, transitions, action_transform, device, seed):
        obs = np.stack([t[0] for t in transitions]).astype(np.float32)
        act = np.stack([t[1] for t in transitions]).astype(np.float32)
        if action_transform is not None:
            act = np.asarray(action_transform(act), dtype=np.float32)
        nobs = np.stack([t[3] for t in transitions]).astype(np.float32)
        rew = np.array([t[2] for t in transitions], dtype=np.float32)[:, None]
        done = np.array([t[4] for t in transitions], dtype=np.float32)[:, None]
        self.device = device
        pin = device.type == 'cuda'

        def _t(a):
            x = th.as_tensor(a)
            return x.pin_memory() if pin else x
        self.observations = _t(obs)
        self.actions = _t(act)
        self.next_observations = _t(nobs)
        self.rewards = _t(rew)
        self.dones = _t(done)
        self.n = len(transitions)
        self.n_rewarded = int((rew > 0).sum())
        # Per-run demo-sampling stream. Hard-coded 0 until 2026-08-17: every seed
        # consumed the IDENTICAL demo-batch sequence, correlating "independent"
        # seeds through the demo curriculum. seed=0 reproduces the old stream.
        self._g = th.Generator().manual_seed(int(seed))

    def sample(self, batch_size):
        idx = th.randint(0, self.n, (batch_size,), generator=self._g)
        to = dict(device=self.device, non_blocking=True)
        return ReplayBufferSamples(
            observations=self.observations[idx].to(**to),
            actions=self.actions[idx].to(**to),
            next_observations=self.next_observations[idx].to(**to),
            dones=self.dones[idx].to(**to),
            rewards=self.rewards[idx].to(**to),
        )


class RLPDSAC(SAC):
    """SAC that overrides ONLY train(): 50/50 two-buffer batch, Z-of-E min target,
    sum-of-E critic loss, ensemble-mean actor loss, and actor+alpha updated exactly
    once per env step (on the final UTD grad step). Everything else -- collection,
    save/load, logging, ent-coef auto/fixed -- is stock SB3 2.8 SAC."""

    def __init__(self, *args, ensemble_size=10, subset_size=2, demo_batch=128,
                 q_watchdog=2.0, backup_entropy=False, **kwargs):
        self.ensemble_size = int(ensemble_size)
        self.subset_size = int(subset_size)
        self.demo_batch = int(demo_batch)
        self.q_watchdog = float(q_watchdog)
        # False = RLPD's setting for every sparse domain (audit bug 1). True
        # restores the pre-audit (SB3-SAC-inherited) behavior for comparison runs.
        self.backup_entropy = bool(backup_entropy)
        self.demo_data = None
        self._watchdog_tripped = False
        super().__init__(*args, **kwargs)

    def set_demo_data(self, demo_data):
        assert demo_data.n > 0, 'empty demo buffer'
        self.demo_data = demo_data

    def _excluded_save_params(self):
        # demo_data holds large tensors and is rebuilt from the demo dir at launch;
        # never pickle it into the checkpoint.
        return super()._excluded_save_params() + ['demo_data']

    def train(self, gradient_steps, batch_size=256):
        assert self.demo_data is not None, 'call set_demo_data() before learn()'
        self.policy.set_training_mode(True)
        opt = [self.actor.optimizer, self.critic.optimizer]
        if self.ent_coef_optimizer is not None:
            opt += [self.ent_coef_optimizer]
        self._update_learning_rate(opt)

        online_bs = batch_size - self.demo_batch
        assert online_bs > 0, f'demo_batch {self.demo_batch} >= batch_size {batch_size}'
        auto_alpha = (self.ent_coef_optimizer is not None
                      and self.log_ent_coef is not None)

        critic_losses, actor_losses, ent_coefs, ent_coef_losses = [], [], [], []
        q_means, demo_rew_counts = [], []

        for gstep in range(gradient_steps):
            last = gstep == gradient_steps - 1
            # ---- 50/50 two-buffer batch (explicit, not a monkey-patched sample) ----
            online = self.replay_buffer.sample(online_bs, env=self._vec_normalize_env)
            demo = self.demo_data.sample(self.demo_batch)
            obs = th.cat([online.observations, demo.observations])
            act = th.cat([online.actions, demo.actions])
            nobs = th.cat([online.next_observations, demo.next_observations])
            dones = th.cat([online.dones, demo.dones])
            rewards = th.cat([online.rewards, demo.rewards])
            demo_rew_counts.append(float((demo.rewards > 0).sum().item()))

            ent_coef = (th.exp(self.log_ent_coef.detach()) if auto_alpha
                        else self.ent_coef_tensor)

            # ---- critic update (every grad step): Z-of-E min Bellman target ----
            with th.no_grad():
                next_act, next_logp = self.actor.action_log_prob(nobs)
                next_q_all = self.critic_target(nobs, next_act)          # (E, B, 1)
                subset = th.randperm(self.ensemble_size, device=self.device)[:self.subset_size]
                next_q, _ = th.min(next_q_all[subset], dim=0)            # (B, 1)
                # ENTROPY BACKUP (audit 2026-08-14, paper/rlpd_audit_2026-08-14.md
                # bug 1): RLPD sets backup_entropy=False for EVERY sparse domain
                # (Table 2: AntMaze, Adroit, pixel-DMC). With it ON at gamma=0.998
                # the critic's zero-reward fixed point is 500*alpha*H — measured
                # Q 269..2400 vs max task return 1.0, and terminals (the PICKS)
                # get target 1.0 vs ~400 for non-terminals: a 400:1 incentive
                # AGAINST completing the task. Off by default for sparse scopes;
                # callers set it EXPLICITLY (train_rlpd --backup-entropy).
                if self.backup_entropy:
                    next_q = next_q - ent_coef * next_logp.reshape(-1, 1)
                target_q = rewards + (1.0 - dones) * self.gamma * next_q
            current_q_all = self.critic(obs, act)                       # (E, B, 1)
            target_exp = target_q.unsqueeze(0).expand_as(current_q_all)
            # sum of the per-critic MSE over all E members (RLPD_PLAN: "sum MSE all 10")
            critic_loss = F.mse_loss(current_q_all, target_exp,
                                     reduction='none').mean(dim=(1, 2)).sum()
            self.critic.optimizer.zero_grad()
            critic_loss.backward()
            self.critic.optimizer.step()
            critic_losses.append(critic_loss.item())

            # ---- actor + alpha: ONCE per env step (final grad step only) ----
            if last:
                actions_pi, log_prob = self.actor.action_log_prob(obs)
                log_prob = log_prob.reshape(-1, 1)
                if auto_alpha:
                    ent_coef_loss = -(self.log_ent_coef
                                      * (log_prob + self.target_entropy).detach()).mean()
                    self.ent_coef_optimizer.zero_grad()
                    ent_coef_loss.backward()
                    self.ent_coef_optimizer.step()
                    ent_coef_losses.append(ent_coef_loss.item())
                    ent_coef = th.exp(self.log_ent_coef.detach())
                q_pi = self.critic(obs, actions_pi).mean(dim=0)         # ensemble MEAN
                actor_loss = (ent_coef * log_prob - q_pi).mean()
                self.actor.optimizer.zero_grad()
                actor_loss.backward()
                self.actor.optimizer.step()
                actor_losses.append(actor_loss.item())
                ent_coefs.append(ent_coef.item())
                q_means.append(float(q_pi.mean().item()))

            polyak_update(self.critic.parameters(),
                          self.critic_target.parameters(), self.tau)

        self._n_updates += gradient_steps

        # ---- Q watchdog: max task return is 1 (pick) + entropy term; a mean actor Q
        # far above that signals the gamma*alpha*H/(1-gamma) explosion (the +536 seen
        # at gamma .999 auto-alpha). Warn loudly; the pre-registered fix is a fixed
        # small ent_coef (--ent-coef 0.005) restart. ----
        if q_means:
            qm = float(np.mean(q_means))
            # RE-ARMING (audit: the one-shot warn fired once at step 1001/Q=2.82
            # and the later ride to Q 269-2400 was never re-flagged). Warns at most
            # once per 10k steps. The old 'fixed --ent-coef 0.005' prescription is
            # RETIRED (audit: it explodes worse, Q->1.6e5).
            if qm > self.q_watchdog and \
                    self.num_timesteps - getattr(self, '_watchdog_last', -10**9) >= 10_000:
                self._watchdog_last = self.num_timesteps
                print(f'[Q-WATCHDOG] mean actor-state Q={qm:.2f} > {self.q_watchdog} '
                      f'at step {self.num_timesteps} -- value scale far above max '
                      f'task return; check backup_entropy and critic health',
                      flush=True)
            self.logger.record('train/actor_q_mean', qm)

        self.logger.record('train/n_updates', self._n_updates, exclude='tensorboard')
        self.logger.record('train/critic_loss', float(np.mean(critic_losses)))
        if actor_losses:
            self.logger.record('train/actor_loss', float(np.mean(actor_losses)))
        if ent_coefs:
            self.logger.record('train/ent_coef', float(np.mean(ent_coefs)))
        if ent_coef_losses:
            self.logger.record('train/ent_coef_loss', float(np.mean(ent_coef_losses)))
        # smoke/verification signals: batch composition + rewarded-demo density + UTD
        self.logger.record('train/utd', gradient_steps)
        self.logger.record('train/online_bs', online_bs)
        self.logger.record('train/demo_bs', self.demo_batch)
        self.logger.record('train/demo_frac', self.demo_batch / batch_size)
        self.logger.record('train/demo_rew_per_batch', float(np.mean(demo_rew_counts)))


def make_rlpd(env, seed, device, *, ensemble_size=10, subset_size=2, utd=10,
              gamma=0.998, ent_coef='auto', target_entropy=None, demo_batch=128,
              net_arch=(256, 256), q_watchdog=2.0, backup_entropy=False,
              per_member_ln=False):
    """Construct an RLPDSAC with the pinned RLPD hypers. target_entropy defaults to
    -dim/2 (RLPD_PLAN: -3.5 for the 7-dim joint action)."""
    dev = th.device(device)
    act_dim = int(np.prod(env.action_space.shape))
    te = target_entropy if target_entropy is not None else -act_dim / 2.0
    ec = ent_coef if ent_coef == 'auto' else float(ent_coef)
    model = RLPDSAC(
        RLPDPolicy, env,
        ensemble_size=ensemble_size, subset_size=subset_size, demo_batch=demo_batch,
        q_watchdog=q_watchdog, backup_entropy=backup_entropy,
        learning_rate=3e-4,
        buffer_size=300_000,          # ONLINE data only; demos live in DemoData
        learning_starts=1_000,
        batch_size=256,
        tau=0.005,
        gamma=gamma,
        train_freq=1,
        gradient_steps=utd,           # UTD: critic grad steps per env step
        ent_coef=ec,
        target_entropy=te,
        seed=seed,
        device=device,
        verbose=1,
        policy_kwargs=dict(
            net_arch=dict(pi=list(net_arch), qf=list(net_arch)),
            n_critics=ensemble_size,
            per_member_ln=per_member_ln,
        ),
    )
    return model
