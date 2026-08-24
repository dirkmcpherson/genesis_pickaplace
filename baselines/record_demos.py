#!/usr/bin/env python
"""ONE recorder for EVERY demonstration set -- contract v1 (PREREG §4).

WHY (paper/PREREG_final_round_robin_2026-08-23.md §0/§4, CRITIQUE_design R1-R5):
until 08-23 the three demo sources reached the learners through three different
pipelines (human collector → stride-1 tapes; DP teacher harvested in GenesisCanEnv with
NO tip rule and a 1200-frame cap; r2dreamer champion harvested through FullTaskEnv at
repeat 4 but recorded per sim step), so "demo source" was confounded with clock, action
representability, truncation, terminal labelling and even the env the teacher acted in.
This script records every source THROUGH THE LEARNERS' OWN MDP:

    FullTaskEnv(scope='pick', action_mode='delta_joint', delta_ref='target',
                delta_cap=0.025, delta_leash_mult=5.0 (leash 0.125), action_repeat=4,
                max_steps=1200 sim steps (= 300 decisions), camera_rig=True, cpu)

and writes one npz per rollout in the CONTRACT-V1 layout (one row = one DECISION):

    states        (n,17) f32  obs BEFORE each decision
    final_state   (17,)  f32  obs after the last decision
    actions_delta (n,7)  f32  the normalized [-1,1]^7 decision the env EXECUTED (clipped)
    actions       (n,7)  f32  absolute command at the window END [6 joint targets rad, grip 0..1]
    rewards       (n,)   f32  env reward per decision (sparse; NO shaping baked in)
    terminated    (n,)   bool env terminated flag per decision (last row only)
    truncated     (n,)   bool env/recorder truncation flag (last row only)
    picked        (n,)   bool env hardened-pick flag per decision
    tipped        (n,)   bool env tip flag per decision
    eef_pos       (n+1,3) f32 tool position before each decision + final
    images        (n+1,64,64,6) u8  rig (top ++ wrist) before each decision + final  [--no-images drops]
    sim_states    (m,17) f32  per-SIM-step sub-tape: obs before each sim step (m <= 4n)
    sim_actions   (m,7)  f32  absolute command executed at each sim step  [--no-sim-tape drops]
    scalars: uid (rollout idx >= 100000), ic_uid, label, stage, teacher, teacher_ckpt,
             act_mode, action_repeat, delta_cap, delta_leash, delta_ref, pick_z, n,
             git_sha, env_class, recorder, contract, end_reason

The tape ENDS where the env ended (terminated on the last row) or at the cap (truncated
on the last row) -- never later. label='success' iff the last row has picked=True.
Successes go to --outdir, fails to --fails-outdir (default <outdir>_fails), each with a
manifest.json (teacher, ICs, attempts, kept/dropped, negctl, per-rollout records,
teacher success rate on the harvest ICs, content sha256, git sha, contract).

ADAPTERS (the only thing that differs between sources):
  human   closed-loop waypoint follower on a human demo's recorded ABSOLUTE command
          stream (stride-1 npz: states (n,17), actions (n,7), uid), generalizing
          baselines/rl/rerecord_delta_demos.py to delta_ref=target and repeat 4:
            a_arm = clip((cmd_j - target_now) / (4*cap), -1, 1),  a_grip = grip_j*2-1
          advance by arrival ||q_meas - ref_j||_inf < tol, up to 4 waypoints per
          decision, dwell cap, dilation cap; after the waypoints are exhausted the last
          one is held for --settle decisions (the hardened pick needs 10 sustained sim
          frames past the geometric grant) -- the env's own terminal ends the tape.
  dp      lerobot policy (baselines/dp_runner.py) queried ONCE per decision on the
          current 17-dim obs; its absolute target q* becomes
            a_arm = clip((q* - target_now) / (4*cap), -1, 1)
          so the DP teacher is embedded in the delta MDP (hold-4) and its tape is in
          the learners' action space by construction.
  r2d     r2dreamer champion (loader lifted from harvest_champion_demos.py); native
          delta actions.
  random  uniform [-1,1]^7 per decision -- the NEGATIVE CONTROL; must keep ~0.

Per-sim-step sub-tape: FullTaskEnv.step() is exactly `for _ in range(action_repeat):
_step_once(a); break on terminated|truncated` (+ a pick_shaping block we assert OFF), so
the recorder runs that loop itself to see the intermediate sim states; it adds nothing
and changes nothing (asserted: env._t advances by the number of sub-steps taken).

USAGE (devbox/cluster; the CPU Genesis world takes ~20-35 s to build):
  # human re-record at repeat 4 (pruned human pick-phase tapes; ic_uid = the demo uid)
  VENV/python baselines/record_demos.py --teacher human \
      --src baselines/episodes_pick_phase_dppruned --outdir baselines/demos_v1/dH
  # dp teacher (lerobot venv), demo ICs cycled, sampled, <=3 attempts per IC
  LEROBOT_VENV/python baselines/record_demos.py --teacher dp --checkpoint <dp ckpt dir> \
      --ic-mode demo --attempts 3 --mode sample --outdir baselines/demos_v1/dDP --verify
  # r2dreamer champion (r2dreamer venv: tensordict/omegaconf)
  R2D_VENV/python baselines/record_demos.py --teacher r2d \
      --checkpoint ~/workspace/r2dreamer/runs/pick_delta25d4_s0/CHAMPION_1576820.pt \
      --ic-mode demo --attempts 3 --mode sample --outdir baselines/demos_v1/dR2D --verify
  # negative control (MUST keep ~0; otherwise the keep predicate is broken, not the teacher)
  VENV/python baselines/record_demos.py --teacher random --ic-mode demo --n 30 \
      --outdir baselines/demos_v1/negctl
  # arg check only (no env): add --dry-run.  Shards: --shard-idx i --shard-n k; then --merge.

Stdlib + numpy at import; genesis / lerobot / r2dreamer are imported lazily.
"""
import argparse
import glob
import hashlib
import json
import os
import pathlib as pl
import subprocess
import sys
import time

import numpy as np

CONTRACT = 'v1'
RECORDER = 'record_demos.py v1'
REPEAT = 4
DELTA_CAP = 0.025
LEASH_MULT = 5.0
MAX_SIM_STEPS = 1200
ROLLOUT_BASE = 100000
IMG_SHAPE = (64, 64, 6)
STATE_DIM, ACT_DIM = 17, 7

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[1]))
def _r2d_root():
    # R2DREAMER_ROOT, else the repo's sibling checkout (cluster: $LAB/r2dreamer), else the devbox path
    for c in (os.environ.get('R2DREAMER_ROOT'), str(REPO.parent / 'r2dreamer'), os.path.expanduser('~/workspace/r2dreamer')):
        if c and (pl.Path(c) / 'tools.py').exists():
            return pl.Path(c)
    return pl.Path(os.environ.get('R2DREAMER_ROOT', os.path.expanduser('~/workspace/r2dreamer')))
R2D_ROOT = _r2d_root()

SCALARS = ('uid', 'ic_uid', 'label', 'stage', 'teacher', 'teacher_ckpt', 'act_mode', 'sim_variant',
           'action_repeat', 'delta_cap', 'delta_leash', 'delta_ref', 'pick_z', 'n',
           'git_sha', 'env_class', 'recorder', 'contract', 'end_reason')


# ============================================================================ contract
def validate_tape(d, require_images=False, require_sim=False):
    """Pure-numpy contract-v1 checker. `d` = dict-like (np.load result or dict).
    Returns a list of violation strings (empty = valid)."""
    errs = []
    keys = set(d.keys() if hasattr(d, 'keys') else d.files)

    def g(k):
        v = d[k]
        return v.item() if (hasattr(v, 'ndim') and v.ndim == 0) else v
    for k in ('states', 'final_state', 'actions_delta', 'actions', 'rewards', 'terminated',
              'truncated', 'picked', 'tipped', 'eef_pos') + tuple(k for k in SCALARS if k != 'sim_variant'):
        if k not in keys:
            errs.append(f'missing key {k}')
    if errs:
        return errs
    n = int(g('n'))
    S = np.asarray(d['states']); A = np.asarray(d['actions_delta']); C = np.asarray(d['actions'])
    if S.shape != (n, STATE_DIM): errs.append(f'states shape {S.shape} != ({n},{STATE_DIM})')
    if np.asarray(d['final_state']).shape != (STATE_DIM,): errs.append('final_state shape')
    if A.shape != (n, ACT_DIM): errs.append(f'actions_delta shape {A.shape}')
    if C.shape != (n, ACT_DIM): errs.append(f'actions shape {C.shape}')
    for k in ('rewards', 'terminated', 'truncated', 'picked', 'tipped'):
        if np.asarray(d[k]).shape != (n,): errs.append(f'{k} shape {np.asarray(d[k]).shape} != ({n},)')
    if np.asarray(d['eef_pos']).shape != (n + 1, 3): errs.append(f'eef_pos shape {np.asarray(d["eef_pos"]).shape} != ({n+1},3)')
    if n < 1:
        errs.append('empty tape'); return errs
    if A.size and (A.min() < -1.0 - 1e-6 or A.max() > 1.0 + 1e-6): errs.append('actions_delta outside [-1,1]')
    if C.size and (C[:, 6].min() < -1e-6 or C[:, 6].max() > 1.0 + 1e-6): errs.append('actions grip outside [0,1]')
    term = np.asarray(d['terminated'], bool); trunc = np.asarray(d['truncated'], bool)
    if term[:-1].any(): errs.append('terminated=True before the last row')
    if trunc[:-1].any(): errs.append('truncated=True before the last row')
    if not (term[-1] ^ trunc[-1]): errs.append('last row must be exactly one of terminated/truncated')
    picked = np.asarray(d['picked'], bool); tipped = np.asarray(d['tipped'], bool)
    lab = str(g('label'))
    if lab not in ('success', 'fail'): errs.append(f'label {lab!r}')
    if (lab == 'success') != bool(picked[-1]): errs.append('label/picked[-1] mismatch')
    if picked[-1] and not term[-1]: errs.append('picked on last row but not terminated')
    if tipped[-1] and not term[-1]: errs.append('tipped on last row but not terminated')
    if picked[:-1].any(): errs.append('picked=True before the last row (env terminates on the pick)')
    if str(g('contract')) != CONTRACT: errs.append(f'contract {g("contract")!r} != {CONTRACT!r}')
    if int(g('action_repeat')) != REPEAT: errs.append(f'action_repeat {g("action_repeat")} != {REPEAT}')
    if abs(float(g('delta_cap')) - DELTA_CAP) > 1e-9: errs.append('delta_cap stamp')
    if str(g('delta_ref')) != 'target': errs.append('delta_ref stamp != target')
    if int(g('uid')) < ROLLOUT_BASE: errs.append(f'uid {g("uid")} < {ROLLOUT_BASE} (rollout index convention)')
    if 'images' in keys:
        I = np.asarray(d['images'])
        if I.shape != (n + 1,) + IMG_SHAPE or I.dtype != np.uint8: errs.append(f'images shape/dtype {I.shape} {I.dtype}')
    elif require_images:
        errs.append('images required but absent')
    if 'sim_states' in keys:
        SS = np.asarray(d['sim_states']); SA = np.asarray(d['sim_actions'])
        if SS.ndim != 2 or SS.shape[1] != STATE_DIM or SA.shape != (SS.shape[0], ACT_DIM): errs.append('sim_* shapes')
        if not (n <= SS.shape[0] <= REPEAT * n): errs.append(f'sim tape length {SS.shape[0]} not in [{n},{REPEAT*n}]')
    elif require_sim:
        errs.append('sim_* required but absent')
    return errs


def git_sha(path):
    try:
        return subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=str(path),
                              capture_output=True, text=True, timeout=5).stdout.strip() or 'unknown'
    except Exception:
        return 'unknown'


# ================================================================================ env
def build_env(args):
    """FullTaskEnv in the contract MDP; every knob asserted (silent-default rule)."""
    sys.path.insert(0, str(REPO / 'baselines')); sys.path.insert(0, str(REPO / 'baselines' / 'rl'))
    sys.path.insert(0, str(REPO / 'can_pos_recovery'))
    from full_env import FullTaskEnv
    from sim_variant_hook import apply_pre, apply_post
    t0 = time.time()
    apply_pre(getattr(args, 'sim_variant', 'base'))
    env = FullTaskEnv(backend='cpu', max_steps=MAX_SIM_STEPS, scope='pick',
                      action_mode='delta_joint', delta_cap=DELTA_CAP,
                      delta_leash_mult=LEASH_MULT, action_repeat=REPEAT, delta_ref='target',
                      camera_rig=not args.no_images, pick_shaping=False)
    assert env.scope == 'pick' and env.action_mode == 'delta_joint' and env.delta_ref == 'target'
    assert env.action_repeat == REPEAT and abs(env.delta_cap - DELTA_CAP) < 1e-12
    assert abs(env.delta_leash - LEASH_MULT * DELTA_CAP) < 1e-12, env.delta_leash
    assert env.max_steps == MAX_SIM_STEPS and not env.pick_shaping and not env.pick_hold_reward
    assert env.genv.max_steps >= 10 ** 8, 'inner env must never truncate (#26)'
    apply_post(env, getattr(args, 'sim_variant', 'base'))
    print(f'[env] FullTaskEnv built in {time.time()-t0:.1f}s | pick_z={env.pick_z:.4f} '
          f'repeat={env.action_repeat} cap={env.delta_cap} leash={env.delta_leash} '
          f'max_sim={env.max_steps} rig={not args.no_images}', flush=True)
    return env


class Recorder:
    """Runs one adapter through the env and returns a contract-v1 tape."""

    def __init__(self, env, images=True, sim_tape=True):
        self.env = env; self.images = images; self.sim_tape = sim_tape
        self.pick_z = float(env.pick_z)

    def _reset(self, ic):
        env = self.env
        if ic.get('uid') is not None:
            obs, info = env.reset(options={'uid': int(ic['uid'])})
        else:
            kw = {k: ic[k] for k in ('can_pos', 'can_quat', 'goal_pos') if ic.get(k) is not None}
            if kw.get('goal_pos') is None:
                # the static goal every uid reset uses (genesis_can_env.reset, #27)
                from replay_harness import STATIC_BOTTLE_POSITION
                kw['goal_pos'] = (float(STATIC_BOTTLE_POSITION[0]), float(STATIC_BOTTLE_POSITION[1]),
                                  float(env.genv.w['goal_start_z']))
            obs, info = env.reset_to(kw)
        # tool offset is calibrated at the reset pose (HARDCODED_START) -- the pose
        # REF_TOOL_AT_START was measured at (genesis_can_env.reset does this only under
        # workspace_limit; FullTaskEnv._pick_phi relies on the lazy first call instead).
        env.genv._calib_tool_offset()
        return np.asarray(obs, np.float32)

    def _eef(self):
        return np.asarray(self.env.genv.tool_pos(), np.float32)[:3]

    def _img(self):
        im = np.asarray(self.env.genv.rig_obs(), np.uint8)
        assert im.shape == IMG_SHAPE, im.shape
        return im

    def _abs_cmd(self, a):
        return np.concatenate([np.asarray(self.env._dj_target, np.float64),
                               [(float(np.clip(a[6], -1.0, 1.0)) + 1.0) / 2.0]]).astype(np.float32)

    def run(self, adapter, ic, max_decisions=None):
        """-> (tape dict or None, record dict)."""
        env = self.env
        obs = self._reset(ic)
        adapter.reset(obs, env)
        T = dict(states=[], actions_delta=[], actions=[], rewards=[], terminated=[], truncated=[],
                 picked=[], tipped=[], eef_pos=[], images=[], sim_states=[], sim_actions=[])
        t0 = time.time(); t = 0; info = {}; end_reason = None
        max_dec = int(max_decisions or (MAX_SIM_STEPS // REPEAT + 1))
        while True:
            if self.images: T['images'].append(self._img())
            T['eef_pos'].append(self._eef())
            a = adapter.act(obs, env, t)
            if a is None:                     # adapter exhausted (human follower): no decision
                if self.images: T['images'].pop()   # this obs IS the final observation -> appended once below
                T['eef_pos'].pop()
                end_reason = 'adapter_exhausted'; break
            a = np.clip(np.asarray(a, np.float32), -1.0, 1.0)
            assert a.shape == (ACT_DIM,), a.shape
            T['states'].append(obs)
            T['actions_delta'].append(a)
            # --- the window: mirrors FullTaskEnv.step() exactly (pick_shaping asserted off)
            total_r = 0.0; terminated = truncated = False; t_before = env._t; o = obs
            for k in range(REPEAT):
                if self.sim_tape:
                    T['sim_states'].append(np.asarray(o, np.float32))   # obs before this sim step
                o, r, terminated, truncated, info = env._step_once(a)
                if self.sim_tape:
                    T['sim_actions'].append(self._abs_cmd(a))
                total_r += float(r)
                if terminated or truncated:
                    break
            assert env._t - t_before == (k + 1), (env._t, t_before, k)
            obs = np.asarray(o, np.float32)
            T['actions'].append(self._abs_cmd(a))
            T['rewards'].append(total_r)
            T['terminated'].append(bool(terminated)); T['truncated'].append(bool(truncated))
            T['picked'].append(bool(info.get('picked')) or ('picked' in env._granted))
            T['tipped'].append(bool(info.get('tipped')))
            adapter.observe(obs, env, float(total_r), bool(terminated or truncated))
            t += 1
            if terminated: end_reason = 'terminated'; break
            if truncated: end_reason = 'env_truncated'; break
            if t >= max_dec: end_reason = 'recorder_cap'; break
        if not T['states']:
            return None, dict(ic_uid=int(ic.get('ic_uid') or ic.get('uid') or -1), outcome='empty', decisions=0, kept=False,
                              seconds=round(time.time() - t0, 1))
        if end_reason != 'terminated':
            T['truncated'][-1] = True          # recorder-side end counts as truncation (bootstrap)
        # final observation
        if self.images: T['images'].append(self._img())
        T['eef_pos'].append(self._eef())
        tape = dict(states=np.stack(T['states']), final_state=obs,
                    actions_delta=np.stack(T['actions_delta']), actions=np.stack(T['actions']),
                    rewards=np.asarray(T['rewards'], np.float32),
                    terminated=np.asarray(T['terminated'], bool), truncated=np.asarray(T['truncated'], bool),
                    picked=np.asarray(T['picked'], bool), tipped=np.asarray(T['tipped'], bool),
                    eef_pos=np.stack(T['eef_pos']).astype(np.float32), end_reason=end_reason,
                    n=len(T['states']))
        if self.images: tape['images'] = np.stack(T['images']).astype(np.uint8)
        if self.sim_tape:
            tape['sim_states'] = np.stack(T['sim_states']).astype(np.float32)
            tape['sim_actions'] = np.stack(T['sim_actions']).astype(np.float32)
        picked = bool(tape['picked'][-1]); tipped = bool(tape['tipped'][-1])
        tape['label'] = 'success' if picked else 'fail'
        tape['stage'] = 'picked' if picked else 'none'
        outcome = 'picked' if picked else ('tipped' if tipped else end_reason)
        rec = dict(ic_uid=int(ic['ic_uid'] if ic.get('ic_uid') is not None else (ic.get('uid') if ic.get('uid') is not None else -1)),
                   ic_can_xy=[round(float(x), 4) for x in tape['states'][0, 8:10]],
                   outcome=outcome, decisions=int(tape['n']), sim_steps=int(env._t),
                   seconds=round(time.time() - t0, 1), kept=False, end_reason=end_reason)
        rec.update(adapter.stats())
        return tape, rec

    def verify(self, tape, ic):
        """Open-loop replay of actions_delta from a fresh reset of the same IC; the hardened
        pick must re-occur (guards manufactured success; same-machine deterministic sim)."""
        env = self.env
        self._reset(ic)
        for a in tape['actions_delta']:
            _, _, term, trunc, info = env.step(a)
            if term or trunc:
                break
        return bool(info.get('picked')) or ('picked' in env._granted)


# ============================================================================ adapters
class Adapter:
    name = 'base'
    ckpt = ''

    def reset(self, obs, env): pass
    def act(self, obs, env, t): raise NotImplementedError
    def observe(self, obs, env, reward, done): pass
    def stats(self): return {}


class RandomTeacher(Adapter):
    name = 'random'

    def __init__(self, seed):
        self.rng = np.random.default_rng(seed + 7919)

    def act(self, obs, env, t):
        return self.rng.uniform(-1.0, 1.0, ACT_DIM).astype(np.float32)


class HumanFollower(Adapter):
    """Closed-loop waypoint follower on a stride-1 human tape (absolute commands),
    generalized to delta_ref=target / repeat 4 (see module docstring)."""
    name = 'human'

    def __init__(self, tol=0.025, max_dwell=8, dilation_cap=3.0, settle=25, arrival='meas'):
        self.tol = float(tol); self.max_dwell = int(max_dwell)
        self.dilation_cap = float(dilation_cap); self.settle = int(settle)
        # arrival test for advancing waypoint k (paper/real2sim_follower_lab_2026-08-23.md §3):
        #   'meas'   : ||q_meas - ref_k||_inf < tol            (the original test; ref_k = the pose the
        #              human's tape ARRIVED at after cmd_k -- a sim arm that lags its command waits
        #              for a pose it has already effectively passed -> dwell stall -> jump)
        #   'either' : 'meas' OR ||target_env - cmd_k||_inf < tol (the env's integrated target has
        #              reached the human's COMMAND) -- local lab: 49 -> 55/66 kept, zero dwell
        #              stalls, same fidelity to the human path, lower dilation
        assert arrival in ('meas', 'either'), arrival
        self.arrival = arrival
        self.src = None

    def load(self, path):
        d = np.load(path, allow_pickle=True)
        S = np.asarray(d['states'], np.float64); A = np.asarray(d['actions'], np.float64)
        assert S.ndim == 2 and S.shape[1] == STATE_DIM and A.shape == (len(S), ACT_DIM), (S.shape, A.shape)
        self.src = dict(uid=int(d['uid']), cmd=A[:, :6], grip=np.clip(A[:, 6], 0.0, 1.0),
                        ref=np.concatenate([S[1:, :6], S[-1:, :6]], axis=0), n=len(S),
                        label=str(d['label']) if 'label' in d.files else '?',
                        stage=str(d['stage']) if 'stage' in d.files else '?')
        return self.src

    def reset(self, obs, env):
        assert self.src is not None, 'call load(path) first'
        self.j = 0; self.dwell = 0; self.stalls = 0; self.settling = 0; self.decisions = 0
        self.step_cap = int(np.ceil(self.dilation_cap * self.src['n'] / REPEAT))

    def act(self, obs, env, t):
        s = self.src; n = s['n']
        if self.j >= n:                                  # waypoints exhausted: hold the last one
            if self.settling >= self.settle:
                return None
            self.settling += 1
            jt = n - 1
        else:
            if self.decisions >= self.step_cap:            # time-dilation cap
                return None
            jt = min(self.j + REPEAT - 1, n - 1)
        self.decisions += 1
        self._jt = jt
        tgt = np.asarray(env._dj_target, np.float64)
        a_arm = np.clip((s['cmd'][jt] - tgt) / (REPEAT * DELTA_CAP), -1.0, 1.0)
        return np.concatenate([a_arm, [s['grip'][jt] * 2.0 - 1.0]]).astype(np.float32)

    def observe(self, obs, env, reward, done):
        s = self.src
        if self.j >= s['n']:
            return
        qn = np.asarray(obs[:6], np.float64)
        tgt = np.asarray(env._dj_target, np.float64)
        adv = -1
        for k in range(self._jt, self.j - 1, -1):
            if float(np.max(np.abs(qn - s['ref'][k]))) < self.tol:
                adv = k; break
            if self.arrival == 'either' and float(np.max(np.abs(tgt - s['cmd'][k]))) < self.tol:
                adv = k; break
        if adv >= 0:
            self.j = adv + 1; self.dwell = 0
        else:
            self.dwell += 1
            if self.dwell >= self.max_dwell:
                self.j = self._jt + 1; self.dwell = 0; self.stalls += 1

    def stats(self):
        s = self.src
        return dict(src_uid=s['uid'], src_label=s['label'], src_stage=s['stage'], n_src_frames=int(s['n']),
                    waypoints_reached=int(min(self.j, s['n'])), dwell_stalls=int(self.stalls),
                    dilation=round(self.decisions * REPEAT / max(1, s['n']), 3), settle_decisions=int(self.settling),
                    arrival=self.arrival)


class DPTeacher(Adapter):
    """lerobot DP/ACT via dp_runner, hold-4: one query per decision, absolute target ->
    delta against the env's running target."""
    name = 'dp'

    def __init__(self, checkpoint, mode, seed, n_action_steps=None, rig_provider=None):
        import torch
        from dp_runner import load_dp_runner
        self.torch = torch; self.mode = mode; self.seed = seed; self.ep = 0
        self.ckpt = str(checkpoint)
        # POLICY device: GPU if visible (a 100-step DDPM query is ~6 s on CPU vs ~0.1 s on GPU);
        # the SIM is CPU regardless (FullTaskEnv backend='cpu'). RECORD_POLICY_DEVICE=cpu forces CPU.
        import torch
        dev = os.environ.get('RECORD_POLICY_DEVICE') or ('cuda' if torch.cuda.is_available() else 'cpu')
        self.device = dev
        self.policy_action, self.policy_reset, self.proprio = load_dp_runner(
            str(checkpoint), rig_provider=rig_provider, n_action_steps=n_action_steps, device=dev)
        print(f'[dp] policy device={dev} n_action_steps={n_action_steps}', flush=True)

    def reset(self, obs, env):
        # 'mode' = reproducible diffusion draw per episode; 'sample' = continuing RNG
        if self.mode == 'mode':
            self.torch.manual_seed(self.seed * 100003 + self.ep)
        self.ep += 1
        self.policy_reset()

    def act(self, obs, env, t):
        phys = np.asarray(self.policy_action({'state': np.asarray(obs, np.float32)}), np.float64)
        assert phys.shape == (ACT_DIM,), phys.shape
        tgt = np.asarray(env._dj_target, np.float64)
        a_arm = np.clip((phys[:6] - tgt) / (REPEAT * DELTA_CAP), -1.0, 1.0)
        return np.concatenate([a_arm, [float(np.clip(phys[6], 0.0, 1.0)) * 2.0 - 1.0]]).astype(np.float32)


class R2DTeacher(Adapter):
    """r2dreamer champion; loader + pack() lifted from harvest_champion_demos.py
    (itself lifted from r2dreamer/eval_genesis.py). Observation fed to the actor mirrors
    r2dreamer's GenesisPick obs dict: image (64,64,6) + is_first/is_last/is_terminal +
    log_<stage> zeros + reward."""
    name = 'r2d'

    def __init__(self, checkpoint, config, mode, seed, device='cpu', torch_threads=2):
        import torch
        torch.set_num_threads(torch_threads)
        from omegaconf import OmegaConf
        sys.path.insert(0, str(R2D_ROOT))
        import tools                                     # r2dreamer
        from tensordict import TensorDict
        from dreamer import Dreamer
        from envs.genesis import STAGE_KEYS                 # r2dreamer adapter
        ckpt = pl.Path(checkpoint).expanduser().resolve()
        cfg_path = pl.Path(config) if config else ckpt.parent / '.hydra' / 'config.yaml'
        assert ckpt.exists(), ckpt
        assert cfg_path.exists(), cfg_path
        tools.set_seed_everywhere(seed)
        cfg = OmegaConf.load(cfg_path); cfg.device = device; cfg.model.compile = False
        # action semantics MUST be the contract's (else the champion acts in another MDP)
        assert str(cfg.env.get('action_mode', 'absolute')) == 'delta_joint', cfg.env.get('action_mode')
        assert int(cfg.env.get('action_repeat', 1) or 1) == REPEAT, cfg.env.get('action_repeat')
        assert abs(float(cfg.env.get('delta_cap', 0.04)) - DELTA_CAP) < 1e-9, cfg.env.get('delta_cap')
        assert abs(float(cfg.env.get('delta_leash_mult', 3.0)) - LEASH_MULT) < 1e-9, cfg.env.get('delta_leash_mult')
        assert tuple(cfg.env.size) == IMG_SHAPE[:2], tuple(cfg.env.size)
        # observation/action spaces: harvest_champion_demos.py takes them from
        # r2dreamer's GenesisPick (constructed WITHOUT building its world -- _build() is
        # lazy there; Genesis allows one world per process and ours is FullTaskEnv).
        # Fallback: minimal gym spaces (image + action) -- flagged as an API guess.
        try:
            from envs.genesis import GenesisPick
            gp = GenesisPick('pick', size=tuple(cfg.env.size), seed=seed, scope='pick',
                             action_repeat=REPEAT, action_mode='delta_joint', delta_cap=DELTA_CAP,
                             delta_leash_mult=LEASH_MULT, reward_scale=float(cfg.env.get('reward_scale', 1.0)))
            assert getattr(gp, '_env', None) is None, 'GenesisPick built a world on construction'
            obs_space, act_space = gp.observation_space, gp.action_space
        except Exception as e:   # pragma: no cover
            print(f'[r2d] WARNING: GenesisPick spaces unavailable ({type(e).__name__}: {e}); using minimal gym spaces', flush=True)
            import gymnasium as gym
            obs_space = gym.spaces.Dict({'image': gym.spaces.Box(0, 255, IMG_SHAPE, np.uint8)})
            act_space = gym.spaces.Box(-1.0, 1.0, (ACT_DIM,), np.float32)
        self.agent = Dreamer(cfg.model, obs_space, act_space).to(device)
        ck = torch.load(str(ckpt), map_location=device, weights_only=False)
        missing, unexpected = self.agent.load_state_dict(ck['agent_state_dict'], strict=False)
        assert not any(k.startswith(('actor.', 'rssm.', 'encoder.')) for k in missing), missing
        self.agent.clone_and_freeze(); self.agent.requires_grad_(False); self.agent.eval()
        self.ckpt = str(ckpt); self.ckpt_step = ck.get('step'); self.mode = mode; self.device = device
        self.torch = torch; self.TensorDict = TensorDict; self.STAGE_KEYS = tuple(STAGE_KEYS)
        print(f'[r2d] champion loaded step={self.ckpt_step} (missing {len(missing)}, unexpected {len(unexpected)})', flush=True)

    def _obs(self, env, first, reward):
        o = {'image': np.asarray(env.genv.rig_obs(), np.uint8), 'is_first': first,
             'is_last': False, 'is_terminal': False}
        for k in self.STAGE_KEYS + ('task_success',):
            o[f'log_{k}'] = np.float32(0.0)
        d = {k: self.torch.as_tensor(np.asarray(v)[None]) for k, v in o.items()}
        d['reward'] = self.torch.tensor([reward], dtype=self.torch.float32)
        td = self.TensorDict(d, batch_size=(1,), device='cpu')
        for k in td.keys():
            if td[k].ndim == 1:
                td[k] = td[k].unsqueeze(-1)
        return td.to(self.device)

    def reset(self, obs, env):
        self.state = self.agent.get_initial_state(1)
        self.trans = self._obs(env, True, 0.0)

    def act(self, obs, env, t):
        act, self.state = self.agent.act(self.trans, self.state, eval=(self.mode == 'mode'))
        return act[0].detach().cpu().numpy().astype(np.float32)

    def observe(self, obs, env, reward, done):
        self.trans = self._obs(env, False, reward)


# ================================================================================ main
def uid_pool_from(args, env):
    if args.uids:
        pool = [int(u) for u in args.uids]; src = 'explicit --uids'
    else:
        pool = sorted(int(pl.Path(p).stem) for p in glob.glob(str(REPO / args.uids_from / '*.npz'))
                      if pl.Path(p).stem.isdigit()); src = args.uids_from
    ok = set(env.success_uids)
    skipped = [u for u in pool if u not in ok]
    pool = [u for u in pool if u in ok]
    print(f'[ic] demo uids from {src}: {len(pool)} resettable ({len(skipped)} skipped: {skipped})', flush=True)
    return pool


def merge(outdir):
    recs, cfgs = {}, []
    for m in sorted(glob.glob(str(outdir / 'manifest_shard*.json'))):
        b = json.loads(pl.Path(m).read_text()); cfgs.append(b['config'])
        for r in b['records']:
            recs[(r.get('rollout'), r['ic_uid'], r.get('attempt', 0))] = r
    kept = [r for r in recs.values() if r['kept']]
    files = sorted(glob.glob(str(outdir / '*.npz')))
    # n_kept is AUTHORITATIVE from disk (every npz in a success outdir is a kept tape; the
    # sbatch manifest gate asserts n_kept == npz count); records may be incomplete if a
    # shard manifest was lost -- say so rather than under-count
    n_files = len(files)
    records_complete = (len(kept) == n_files)
    if not records_complete:
        print(f'[merge] WARNING: {len(kept)} kept records in shard manifests vs {n_files} npz on disk '
              f'-- records incomplete (lost shard manifest?); n_kept taken from disk')
    # per-tape stamps from the npz themselves (always complete)
    tapes = []
    for f in files:
        z = np.load(f, allow_pickle=True)
        tapes.append(dict(name=pl.Path(f).name, ic_uid=int(z['ic_uid']), n=int(z['n']), label=str(z['label']),
                          end_reason=str(z['end_reason']) if 'end_reason' in z.files else None))
    h = hashlib.sha256()
    for f in files:
        h.update(pl.Path(f).name.encode()); h.update(pl.Path(f).read_bytes())
    first = [r for r in recs.values() if int(r.get('attempt', 0)) == 0]
    stats = dict(teacher_success_rate_first_attempt=(sum(bool(r.get('kept')) for r in first) / len(first)) if first else None,
                 rejected_by_verify=int(sum(str(r.get('outcome')) == 'verify_failed' for r in recs.values())),
                 yield_frac=(len(kept) / len(recs)) if recs else None)
    (outdir / 'manifest.json').write_text(json.dumps(dict(
        configs=cfgs, n_rollouts=len(recs), n_kept=n_files, n_kept_records=len(kept), files=n_files, **stats,
        records_complete=records_complete, tapes=tapes,
        content_sha256=h.hexdigest(), contract=CONTRACT, recorder=RECORDER,
        records=sorted(recs.values(), key=lambda r: (r['ic_uid'], r.get('attempt', 0)))), indent=1))
    print(f'[merge] rollouts={len(recs)} kept={len(kept)} files={len(files)} sha={h.hexdigest()[:16]}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--teacher', choices=['human', 'dp', 'r2d', 'random'], required=True)
    ap.add_argument('--outdir', required=True, help='success tapes + manifest')
    ap.add_argument('--fails-outdir', default=None, help='default <outdir>_fails')
    ap.add_argument('--checkpoint', default=None, help='dp: lerobot ckpt dir; r2d: CHAMPION .pt')
    ap.add_argument('--config', default=None, help='r2d: hydra config.yaml (default <ckpt>/.hydra/config.yaml)')
    ap.add_argument('--mode', choices=['mode', 'sample'], default='sample',
                    help='r2d: actor mode vs rsample; dp: "mode" = reseeded diffusion draw per episode')
    ap.add_argument('--src', default='baselines/episodes_pick_phase_dppruned',
                    help='human: stride-1 human tapes (states/actions/uid) to follow')
    ap.add_argument('--ic-mode', choices=['demo', 'random'], default='demo')
    ap.add_argument('--uids-from', default='baselines/episodes_pick_phase_dppruned',
                    help='demo ICs: uid list from this dir (intersected with env.success_uids)')
    ap.add_argument('--uids', type=int, nargs='*', default=None)
    ap.add_argument('--ic-from-tape', action='store_true', help='human: demos without a recovered placement are reset to the can pose recorded at frame 0 of their own tape (FK-seeded ICs); default = skip them')
    ap.add_argument('--attempts', type=int, default=1, help='rollouts per IC until a keep (>1 needs --mode sample)')
    ap.add_argument('--n', type=int, default=None, help='max rollouts total (default len(pool)*attempts)')
    ap.add_argument('--target-kept', type=int, default=None, help='stop after this many successes')
    ap.add_argument('--verify', action='store_true', help='open-loop replay guard on every success')
    ap.add_argument('--no-images', action='store_true'); ap.add_argument('--no-sim-tape', action='store_true')
    ap.add_argument('--tol', type=float, default=0.025); ap.add_argument('--max-dwell', type=int, default=8)
    ap.add_argument('--sim-variant', default='base', help='Genesis world variant (baselines/sim_variants.py); base = unpatched. Stamped into every tape/manifest; consumers assert it.')
    ap.add_argument('--arrival', choices=['meas', 'either'], default='meas', help='human: waypoint arrival test (see HumanFollower); either = lab-recommended')
    ap.add_argument('--dilation-cap', type=float, default=3.0); ap.add_argument('--settle', type=int, default=25,
                    help='human: decisions to hold the last waypoint after exhaustion')
    ap.add_argument('--n-action-steps', type=int, default=None, help='dp: chunk replan interval override')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--shard-idx', type=int, default=0); ap.add_argument('--shard-n', type=int, default=1)
    ap.add_argument('--rollout-base', type=int, default=ROLLOUT_BASE)
    ap.add_argument('--merge', action='store_true', help='no sim: merge shard manifests in --outdir')
    ap.add_argument('--dry-run', action='store_true', help='validate args, build nothing')
    ap.add_argument('--torch-threads', type=int, default=2)
    args = ap.parse_args()

    out = REPO / args.outdir
    fails = REPO / (args.fails_outdir or (args.outdir.rstrip('/') + '_fails'))
    if args.merge:
        merge(out); merge(fails) if fails.exists() else None; return
    if args.teacher in ('dp', 'r2d') and not args.checkpoint:
        sys.exit(f'FATAL: --teacher {args.teacher} needs --checkpoint')
    if args.attempts > 1 and args.mode != 'sample':
        sys.exit('FATAL: --attempts > 1 repeats a deterministic rollout; use --mode sample')
    if args.teacher == 'human' and args.ic_mode != 'demo':
        sys.exit('FATAL: the human follower is tied to its source demo IC (--ic-mode demo)')
    if args.teacher == 'human' and not (REPO / args.src).is_dir():
        sys.exit(f'FATAL: --src {args.src} not found')
    print(f'[args] teacher={args.teacher} mode={args.mode} ic_mode={args.ic_mode} attempts={args.attempts} '
          f'verify={args.verify} images={not args.no_images} sim_tape={not args.no_sim_tape} '
          f'out={out} fails={fails} shard={args.shard_idx}/{args.shard_n}', flush=True)
    if args.dry_run:
        print('[dry-run] ok (no env built)'); return

    env = build_env(args)
    rec = Recorder(env, images=not args.no_images, sim_tape=not args.no_sim_tape)
    sys.path.insert(0, str(REPO / 'baselines'))
    import ic_sampling

    # ---- adapter
    if args.teacher == 'random':
        adapter = RandomTeacher(args.seed)
    elif args.teacher == 'human':
        adapter = HumanFollower(args.tol, args.max_dwell, args.dilation_cap, args.settle, arrival=args.arrival)
    elif args.teacher == 'dp':
        adapter = DPTeacher(args.checkpoint, args.mode, args.seed, args.n_action_steps,
                            rig_provider=(env.genv.rig_obs if not args.no_images else None))
    else:
        adapter = R2DTeacher(args.checkpoint, args.config, args.mode, args.seed, torch_threads=args.torch_threads)

    # ---- IC plan
    if args.teacher == 'human':
        files = sorted(glob.glob(str(REPO / args.src / '*.npz')), key=lambda p: int(pl.Path(p).stem))
        if args.uids:
            want = set(int(u) for u in args.uids)
            files = [f for f in files if int(pl.Path(f).stem) in want]
        solved = set(env.success_uids)
        plan = []
        for f in files:
            u = int(pl.Path(f).stem)
            if u in solved:
                plan.append(dict(uid=u, src=f))
            elif args.ic_from_tape:
                # no recovered placement for this demo (FK-seeded at collection): reset to the
                # IC the stride-1 tape itself was recorded from (can pose at frame 0, static goal)
                s0 = np.load(f, allow_pickle=True)['states'][0]
                plan.append(dict(uid=None, ic_uid=u, src=f,
                                 can_pos=[float(v) for v in s0[8:11]], can_quat=[float(v) for v in s0[11:15]],
                                 goal_pos=None))
            else:
                print(f'[ic] skip uid {u}: no placement (pass --ic-from-tape to reset to the tape\'s own frame-0 can pose)')
        plan = plan[args.shard_idx::args.shard_n]
        attempts = 1
    elif args.ic_mode == 'demo':
        pool = uid_pool_from(args, env)
        plan = [dict(uid=u) for u in pool]
        if args.ic_from_tape:
            # demo ICs without a recovered placement: the can pose at frame 0 of the human tape
            # (same ICs the human adapter records from -> every source covers the same 66 ICs)
            have = set(pool)
            for f in sorted(glob.glob(str(REPO / args.uids_from / '*.npz')), key=lambda q: int(pl.Path(q).stem)):
                u = int(pl.Path(f).stem)
                if u in have or (args.uids and u not in set(int(x) for x in args.uids)):
                    continue
                s0 = np.load(f, allow_pickle=True)['states'][0]
                plan.append(dict(uid=None, ic_uid=u, can_pos=[float(v) for v in s0[8:11]],
                                 can_quat=[float(v) for v in s0[11:15]], goal_pos=None))
            print(f'[ic] +{len(plan) - len(pool)} tape-pose ICs (--ic-from-tape) -> {len(plan)} demo ICs total', flush=True)
        plan = plan[args.shard_idx::args.shard_n]; attempts = args.attempts
    else:
        n = args.n or 30
        ics = ic_sampling.sample_support_ics(env.genv, n, seed=args.seed + 1000 * args.shard_idx)
        plan = [dict(ic, uid=None) for ic in ics]; attempts = args.attempts
    if args.n and args.teacher != 'human' and args.ic_mode == 'demo':
        # cycle the uid pool until --n rollouts are planned
        base_plan = plan; plan = []
        while len(plan) < args.n:
            plan.extend(base_plan)
        plan = plan[:args.n]
    print(f'[plan] {len(plan)} ICs x <= {attempts} attempts', flush=True)

    out.mkdir(parents=True, exist_ok=True); fails.mkdir(parents=True, exist_ok=True)
    base = args.rollout_base + 1000 * args.shard_idx
    existing = [int(pl.Path(p).stem) for d in (out, fails) for p in glob.glob(str(d / '*.npz')) if pl.Path(p).stem.isdigit()]
    next_id = max([base - 1] + [e for e in existing if base <= e < base + 1000]) + 1
    GIT = git_sha(REPO)
    stamp = dict(teacher=args.teacher, teacher_ckpt=str(getattr(adapter, 'ckpt', '') or ''),
                 act_mode=args.mode, action_repeat=REPEAT, delta_cap=DELTA_CAP,
                 delta_leash=LEASH_MULT * DELTA_CAP, delta_ref='target', sim_variant=str(getattr(args, 'sim_variant', 'base')), pick_z=float(env.pick_z),
                 git_sha=GIT, env_class='FullTaskEnv', recorder=RECORDER, contract=CONTRACT)
    records = []; n_kept = 0; n_fail = 0; n_verify_rej = 0; n_roll = 0
    for i, ic in enumerate(plan):
        if args.target_kept and n_kept >= args.target_kept:
            break
        if args.teacher == 'human':
            adapter.load(ic['src'])
        for attempt in range(attempts):
            env_ic = {k: v for k, v in ic.items() if k != 'src'}
            tape, r = rec.run(adapter, env_ic)
            r['attempt'] = attempt; n_roll += 1
            if tape is None:
                records.append(r); continue
            ok = tape['label'] == 'success'
            verify = 'n/a'
            if ok and args.verify:
                v = rec.verify(tape, env_ic); verify = 'pass' if v else 'fail'; r['verify'] = verify
                if not v:
                    # label stays 'success' (contract: label follows picked[-1]); the tape is
                    # routed to the fails dir and never counted as kept
                    n_verify_rej += 1; ok = False; r['outcome'] = 'verify_failed'
            rid = next_id; next_id += 1
            d = dict(tape, uid=int(rid), ic_uid=int(r['ic_uid']), verify=verify, **stamp)
            errs = validate_tape(d, require_images=not args.no_images, require_sim=not args.no_sim_tape)
            if errs:
                sys.exit(f'FATAL: contract violation on rollout {rid}: {errs}')
            dest = (out if ok else fails) / f'{rid}.npz'
            np.savez_compressed(dest, **d)
            r.update(rollout=int(rid), kept=bool(ok), file=str(dest.relative_to(REPO)), label=tape['label'],
                     n=int(tape['n']))
            records.append(r)
            n_kept += int(ok); n_fail += int(not ok)
            print(f"[roll {n_roll}] ic_uid={r['ic_uid']} attempt={attempt} {r['outcome']} decisions={r['decisions']} "
                  f"sim_steps={r['sim_steps']} kept={ok} ({r['seconds']}s)" + (f" dil={r['dilation']}" if 'dilation' in r else ''), flush=True)
            if ok:
                break
    # ---- manifest (per shard; --merge folds shards)
    def _sha(d):
        h = hashlib.sha256()
        for f in sorted(glob.glob(str(d / '*.npz'))):
            h.update(pl.Path(f).name.encode()); h.update(pl.Path(f).read_bytes())
        return h.hexdigest()
    first_attempt = [r for r in records if r.get('attempt', 0) == 0 and 'outcome' in r]
    teacher_rate = (sum(r['outcome'] == 'picked' for r in first_attempt) / len(first_attempt)) if first_attempt else None
    cfg = dict(vars(args)) | dict(repeat=REPEAT, delta_cap=DELTA_CAP, leash=LEASH_MULT * DELTA_CAP,
                                  max_sim_steps=MAX_SIM_STEPS, pick_z=float(env.pick_z), git_sha=GIT,
                                  teacher_ckpt=stamp['teacher_ckpt'], contract=CONTRACT, recorder=RECORDER)
    summary = dict(config=cfg, n_rollouts=n_roll, n_kept=n_kept, n_fail=n_fail, rejected_by_verify=n_verify_rej,
                   teacher_success_rate_first_attempt=teacher_rate,
                   yield_frac=round(n_kept / max(1, n_roll), 4), records=records,
                   content_sha256_success=_sha(out), content_sha256_fails=_sha(fails),
                   built=time.strftime('%Y-%m-%dT%H:%M:%S'))
    # shard manifests are named by shard AND rollout base so a second sharded run into the
    # same outdir (e.g. the --ic-from-tape pass) cannot overwrite the first run's manifests
    name = 'manifest.json' if args.shard_n == 1 else f'manifest_shard{args.shard_idx}_base{args.rollout_base}.json'
    (out / name).write_text(json.dumps(summary, indent=1, default=str))
    (fails / name).write_text(json.dumps(dict(summary, records=[r for r in records if not r.get('kept')]), indent=1, default=str))
    print(f'[done] rollouts={n_roll} kept={n_kept} fails={n_fail} verify_rejected={n_verify_rej} '
          f'teacher_rate(first attempt)={teacher_rate} -> {out} / {fails}', flush=True)
    if args.teacher == 'random' and n_kept > max(1, n_roll // 30):
        print(f'WARNING: random teacher kept {n_kept}/{n_roll} -- the keep predicate is broken, not the teacher', flush=True)


if __name__ == '__main__':
    main()
