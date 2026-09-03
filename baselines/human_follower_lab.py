#!/usr/bin/env python
"""Human-follower LAB -- candidate translations of the human demos into the learners' MDP.

Context (paper/real2sim_follower_lab_2026-08-23.md): the contract-v1 recorder
(baselines/record_demos.py, `HumanFollower`) re-records each of the 66 pruned human pick
demos through FullTaskEnv(scope=pick, delta_joint, delta_ref=target, cap 0.025, leash 0.125,
repeat 4, 1200 sim steps, tip rule, hardened pick). The MDP is FIXED. This module only
varies the ADAPTER (how the human's recorded absolute command stream is turned into
decisions) and measures: kept count, dilation, fraction of decisions at the cap, and the
kept tapes' behavioural fidelity to the human's own measured joint path.

Every candidate is a TRANSLATION of the human's path: the waypoints are always the human's
own recorded commands (cmd_j) and measured poses (ref_j), the grip is always the human's
recorded grip command at the waypoint, no replanning, no reward, no use of the can pose.

Knobs (all default to the record_demos.HumanFollower baseline):
  tol            arrival tolerance on ||q_meas - ref_k||_inf            (0.025)
  max_dwell      decisions without arrival before a forced skip         (8)
  dilation_cap   decision budget = cap * n_src / 4                       (3.0)
  settle         decisions to hold the last waypoint after exhaustion    (25)
  arrival        'meas'   baseline: q_meas within tol of ref_k
                 'target' the env's integrated target within tol_t of cmd_k (command-space
                          progress: the follower replays the COMMAND stream at cap speed)
                 'either' meas OR target
                 'passed' meas OR (q_meas has passed ref_k along the human path direction AND
                          is within pass_tol of it) -- perpendicular deviation does not block
  tol_t          tolerance for the target-space test                      (= tol)
  pass_tol       loose radius for the 'passed' test                      (0.075 = 3 tol)
  aim            'window'     baseline: aim at cmd[j+3]
                 'lookahead'  aim at cmd[j+3+L]
                 'reach'      aim at the furthest cmd_k, k in [j+3, j+3+L], with
                              ||cmd_k - target||_inf <= 4 cap (reachable this decision)
  L              lookahead length in waypoints                            (0)
  skip_tol       idle/slow-time compression: after the normal advance, keep advancing
                 through waypoints whose cmd is within skip_tol of the current target AND
                 whose grip cmd equals the current one (|dg| < skip_grip) AND whose ref is
                 within skip_ref of q_meas, up to max_skip per decision. None = off.
                 The human's idle frames (command not moving) cost real time in the
                 baseline (4 waypoints/decision max); this compresses them.
  skip_grip      grip-cmd constancy tolerance for skipping                (0.02)
  skip_ref       q_meas-vs-ref tolerance for skipping                     (= tol)
  max_skip       extra waypoints skipped per decision                     (64)
  grip_settle    do not skip while the measured grip is still moving (|dgrip_meas| > this
                 between decisions)                                       (0.01)
  effort_settle  do not skip while the measured grip EFFORT (obs[7]) still changes by more than
                 this per decision (the fingers are still building force) (None = off)
  skip_guard     do not skip any waypoint within this many source frames after the human's
                 grip command last changed by > skip_grip (the grasp/release phase is idle in
                 command space but physically active)                      (0)
  skip_dq        do not skip while the ARM is still moving (||dq||_inf over the last decision >
                 this): the human's idle frames also let the sim arm creep to its target (soft PD
                 + gravity sag); compressing them starts the next phase with the arm short of
                 where the human's arm was                                 (None = off)
  skip_lag       do not skip unless the arm has converged to its target (||target−q||_inf <=
                 this): the sim-independent form of skip_dq               (None = off)
  stall_hold     on a dwell stall, baseline jumps j = jt+1. 'hold' keeps aiming at the
                 same window but counts the stall; 'jump' (baseline)      ('jump')
  stat_eps       stationary-arm rule: if no arrival, the arm did not move over the last
                 decision (||dq||_inf < stat_eps), the grip is settled and dwell >= 1, waiting
                 cannot help (the arm is blocked or already converged) -> advance j = jt+1 at
                 once instead of burning max_dwell decisions. None = off.  (counted: fast_stalls)

Outputs per config: baselines/demos_v1/_lab/<config>/ (kept tapes, contract v1, no images by
default) + _fails/, manifest.json with per-uid records incl. diagnostics and fidelity
metrics (computed from the per-sim-step sub-tape against the source tape):
  dil             decisions*4/n_src (as in the recorder)
  dil_active      (decisions - settle)*4/n_src
  frac_cap        decisions with max|a_arm| >= 0.999
  path_ratio      follower joint path length (L2, per sim step) / human's (to last reached wp)
  dev_max/dev_p95 max / p95 over follower sim steps of min_k ||q - ref_k||_2 (joint rad)
  cov_max         max over human frames <= reached of min ||ref_k - q_f||_2 (path coverage)
  grip_close_dev  sim step of grip-cmd close vs human's frame (in follower sim steps)
USAGE
  python baselines/human_follower_lab.py --config baseline --parallel 3
  python baselines/human_follower_lab.py --config skip_idle --uids 245 293 --trace
  python baselines/human_follower_lab.py --report            # table over all configs
  python baselines/human_follower_lab.py --list-configs
"""
import argparse
import glob
import json
import os
import pathlib as pl
import subprocess
import sys
import time

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', pl.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO / 'baselines'))
import record_demos as rd  # noqa: E402
import sim_variants  # noqa: E402
from record_demos import (HumanFollower, Recorder, validate_tape, build_env, git_sha,  # noqa: E402
                          REPEAT, DELTA_CAP, LEASH_MULT, MAX_SIM_STEPS, CONTRACT, RECORDER,
                          ROLLOUT_BASE, STATE_DIM, ACT_DIM)

LAB_ROOT = 'baselines/demos_v1/_lab'
SRC_DEFAULT = 'baselines/episodes_pick_phase_dppruned'
PICK_Z = 0.1505
GRIP_CLOSED = 0.5   # grip cmd 0..1, HIGHER = closed (tape convention)

BASE = dict(tol=0.025, max_dwell=8, dilation_cap=3.0, settle=25,
            arrival='meas', tol_t=None, pass_tol=0.075,
            aim='window', L=0,
            skip_tol=None, skip_grip=0.02, skip_ref=None, max_skip=64, grip_settle=0.01,
            effort_settle=None, skip_guard=0, skip_dq=None, skip_lag=None,
            stall_hold='jump', stat_eps=None)

# ------------------------------------------------------------------ named configs
CONFIGS = {
    'baseline': {},
    # one knob at a time
    'tol035': dict(tol=0.035),
    'tol050': dict(tol=0.05),
    'dwell4': dict(max_dwell=4),
    'dwell16': dict(max_dwell=16),
    'arr_target': dict(arrival='target'),
    'arr_either': dict(arrival='either'),
    'arr_passed': dict(arrival='passed'),
    'look2': dict(aim='lookahead', L=2),
    'look4': dict(aim='lookahead', L=4),
    'reach4': dict(aim='reach', L=4),
    'reach8': dict(aim='reach', L=8),
    'skip_idle': dict(skip_tol=1e-3),
    'skip_slow': dict(skip_tol=0.01),
    'skip_tol': dict(skip_tol=0.025),
    'dilcap6': dict(dilation_cap=6.0),
    # combinations (filled in as the lab progresses)
    'either_skip': dict(arrival='either', skip_tol=1e-3),
    'passed_skip': dict(arrival='passed', skip_tol=1e-3),
    'either_skip_dil6': dict(arrival='either', skip_tol=1e-3, dilation_cap=6.0),
    'passed_skip_dil6': dict(arrival='passed', skip_tol=1e-3, dilation_cap=6.0),
    'either_skipslow_dil6': dict(arrival='either', skip_tol=0.01, dilation_cap=6.0),
    'either_skip_reach4_dil6': dict(arrival='either', skip_tol=1e-3, aim='reach', L=4, dilation_cap=6.0),
    'stat': dict(stat_eps=1e-3),
    'skip_idle_g': dict(skip_tol=1e-3, grip_settle=0.003, effort_settle=0.1, skip_guard=45),
    'either_skipg_stat': dict(arrival='either', skip_tol=1e-3, grip_settle=0.003, effort_settle=0.1, skip_guard=45, stat_eps=1e-3),
    'either_skipg': dict(arrival='either', skip_tol=1e-3, grip_settle=0.003, effort_settle=0.1, skip_guard=45),
    'either_skipg2_stat': dict(arrival='either', skip_tol=1e-3, grip_settle=0.003, effort_settle=0.1, skip_guard=45, skip_dq=0.002, stat_eps=1e-3),
    'either_skipg2': dict(arrival='either', skip_tol=1e-3, grip_settle=0.003, effort_settle=0.1, skip_guard=45, skip_dq=0.002),
    # for a simulator that TRACKS the command (sim variants): the tape's measured refs are stale
    # (old-sim sag), so progress/skip must not wait for them -> skip_ref off, target arrival
    'either_skipg2nr_stat': dict(arrival='either', skip_tol=1e-3, grip_settle=0.003, effort_settle=0.1, skip_guard=45, skip_dq=0.002, stat_eps=1e-3, skip_ref=10.0),
    # slow-time compression: skip waypoints within 5 mrad (cap/5) of the target (path shape kept to 5 mrad)
    'either_skips5nr_stat': dict(arrival='either', skip_tol=0.005, grip_settle=0.003, effort_settle=0.1, skip_guard=45, skip_dq=0.002, stat_eps=1e-3, skip_ref=10.0),
    # slow-time compression gated on arm CONVERGENCE (lag) instead of arm velocity: in a tracking sim the
    # arm follows slow motion with lag ~0, so slow stretches compress; fast/lagging phases do not
    'either_skips5L_stat': dict(arrival='either', skip_tol=0.005, grip_settle=0.003, effort_settle=0.1, skip_guard=45, skip_lag=0.01, stat_eps=1e-3, skip_ref=10.0),
    'target_skipg2nr': dict(arrival='target', skip_tol=1e-3, grip_settle=0.003, effort_settle=0.1, skip_guard=45, skip_dq=0.002, skip_ref=10.0),
    'either_skipg_stat_dil6': dict(arrival='either', skip_tol=1e-3, grip_settle=0.003, effort_settle=0.1, skip_guard=45, stat_eps=1e-3, dilation_cap=6.0),
    'either_stat': dict(arrival='either', stat_eps=1e-3),
    'either_skip_stat': dict(arrival='either', skip_tol=1e-3, stat_eps=1e-3),
    'target_skip_stat': dict(arrival='target', skip_tol=1e-3, stat_eps=1e-3),
    'either_skip_stat_dil6': dict(arrival='either', skip_tol=1e-3, stat_eps=1e-3, dilation_cap=6.0),
}


def cfg_of(name, overrides=None):
    if name not in CONFIGS:
        sys.exit(f'unknown config {name!r}; known: {sorted(CONFIGS)}')
    c = dict(BASE); c.update(CONFIGS[name]); c.update(overrides or {})
    if c['tol_t'] is None: c['tol_t'] = c['tol']
    if c['skip_ref'] is None: c['skip_ref'] = c['tol']
    return c


# ------------------------------------------------------------------ the follower
class LabFollower(HumanFollower):
    """record_demos.HumanFollower with the knobs above. act()/observe() are re-implemented
    (the base class hard-codes the baseline rules); load()/stats() are reused."""
    name = 'human'

    def __init__(self, cfg, trace=False):
        super().__init__(tol=cfg['tol'], max_dwell=cfg['max_dwell'],
                         dilation_cap=cfg['dilation_cap'], settle=cfg['settle'])
        self.cfg = dict(cfg); self.trace_on = bool(trace)

    def load(self, path):
        s = super().load(path)
        # unit direction of the human MEASURED path at each waypoint (for 'passed')
        ref = s['ref']
        d = np.diff(ref, axis=0, append=ref[-1:])
        nrm = np.linalg.norm(d, axis=1, keepdims=True)
        s['dir'] = np.where(nrm > 1e-3, d / np.maximum(nrm, 1e-9), 0.0)   # idle -> no direction
        # frames since the grip command last changed by > skip_grip (for skip_guard)
        g = s['grip']; last = 0; since = np.zeros(len(g), int)
        for k in range(1, len(g)):
            if abs(g[k] - g[k - 1]) > self.cfg['skip_grip']: last = k
            since[k] = k - last
        s['since_grip_change'] = since
        return s

    def reset(self, obs, env):
        super().reset(obs, env)
        self.n_cap = 0; self.skipped = 0; self.trace = []; self.exhaust_t = None; self.j_hist = []
        self._prev_grip_meas = float(obs[6]); self._grip_vel = 0.0
        self._prev_eff = float(obs[7]); self._eff_vel = 0.0
        self._prev_q = np.asarray(obs[:6], np.float64).copy(); self.fast_stalls = 0
        self._sat_hist = []

    # -- aim ---------------------------------------------------------------------
    def _aim(self, env):
        s = self.src; n = s['n']; c = self.cfg
        j3 = min(self.j + REPEAT - 1, n - 1)
        if c['aim'] == 'window':
            return j3
        if c['aim'] == 'lookahead':
            return min(j3 + int(c['L']), n - 1)
        if c['aim'] == 'reach':
            tgt = np.asarray(env._dj_target, np.float64)
            k_best = j3
            for k in range(j3 + 1, min(j3 + int(c['L']), n - 1) + 1):
                if float(np.max(np.abs(s['cmd'][k] - tgt))) <= REPEAT * DELTA_CAP:
                    k_best = k
                else:
                    break
            return k_best
        raise ValueError(c['aim'])

    def act(self, obs, env, t):
        s = self.src; n = s['n']
        if self.j >= n:
            if self.settling >= self.settle:
                return None
            self.settling += 1
            jt = n - 1
        else:
            if self.decisions >= self.step_cap:
                return None
            jt = self._aim(env)
        self.decisions += 1
        self._jt = jt; self.j_hist.append(int(self.j))
        tgt = np.asarray(env._dj_target, np.float64)
        a_arm = np.clip((s['cmd'][jt] - tgt) / (REPEAT * DELTA_CAP), -1.0, 1.0)
        sat = float(np.max(np.abs(a_arm)))
        if self.j < n:
            self.n_cap += int(sat >= 0.999)
        if self.trace_on:
            qn = np.asarray(obs[:6], np.float64)
            self.trace.append(dict(t=t, j=self.j, jt=jt, dwell=self.dwell, sat=round(sat, 3),
                                   d_ref=round(float(np.max(np.abs(qn - s['ref'][min(self.j, n - 1)]))), 4),
                                   d_cmd_t=round(float(np.max(np.abs(tgt - s['cmd'][jt]))), 4),
                                   lag=round(float(np.max(np.abs(tgt - qn))), 4),
                                   g=round(float(s['grip'][jt]), 3), gm=round(float(obs[6]), 3)))
        return np.concatenate([a_arm, [s['grip'][jt] * 2.0 - 1.0]]).astype(np.float32)

    # -- advance -----------------------------------------------------------------
    def _arrived(self, k, qn, tgt):
        s = self.src; c = self.cfg
        d_meas = float(np.max(np.abs(qn - s['ref'][k])))
        if c['arrival'] in ('meas', 'either', 'passed') and d_meas < self.tol:
            return True
        if c['arrival'] in ('target', 'either'):
            if float(np.max(np.abs(tgt - s['cmd'][k]))) < c['tol_t']:
                return True
        if c['arrival'] == 'passed':
            # passed the plane through ref_k normal to the local path direction, within pass_tol
            # (idle waypoints have no direction -> the strict meas test above is the only one)
            u = s['dir'][k]
            if float(np.abs(u).sum()) > 0.0 and float(np.dot(qn - s['ref'][k], u)) >= 0.0 \
                    and float(np.linalg.norm(qn - s['ref'][k])) < c['pass_tol']:
                return True
        return False

    def observe(self, obs, env, reward, done):
        s = self.src; n = s['n']; c = self.cfg
        gmeas = float(obs[6]); self._grip_vel = abs(gmeas - self._prev_grip_meas); self._prev_grip_meas = gmeas
        eff = float(obs[7]); self._eff_vel = abs(eff - self._prev_eff); self._prev_eff = eff
        if self.j >= n:
            if self.exhaust_t is None: self.exhaust_t = int(env._t)
            return
        qn = np.asarray(obs[:6], np.float64)
        dq = float(np.max(np.abs(qn - self._prev_q))); self._prev_q = qn.copy()
        tgt = np.asarray(env._dj_target, np.float64)
        adv = -1
        for k in range(self._jt, self.j - 1, -1):
            if self._arrived(k, qn, tgt):
                adv = k; break
        if adv >= 0:
            self.j = adv + 1; self.dwell = 0
        else:
            self.dwell += 1
            if c['stat_eps'] is not None and self.dwell >= 2 and dq < c['stat_eps'] \
                    and self._grip_vel <= c['grip_settle']:
                self.j = self._jt + 1; self.dwell = 0; self.fast_stalls += 1
            elif self.dwell >= self.max_dwell:
                if c['stall_hold'] == 'jump':
                    self.j = self._jt + 1
                self.dwell = 0; self.stalls += 1
        # -- speed-aware skipping of waypoints that cost nothing (idle / slow human time)
        if c['skip_tol'] is not None and self.j < n and self.j > 0:
            settled = self._grip_vel <= c['grip_settle'] and \
                (c['effort_settle'] is None or self._eff_vel <= c['effort_settle']) and \
                (c['skip_dq'] is None or dq <= c['skip_dq']) and \
                (c['skip_lag'] is None or float(np.max(np.abs(tgt - qn))) <= c['skip_lag'])
            if settled:
                g0 = s['grip'][self.j - 1]
                k = self.j; nsk = 0
                while k < n and nsk < int(c['max_skip']):
                    if float(np.max(np.abs(s['cmd'][k] - tgt))) > c['skip_tol']: break
                    if abs(float(s['grip'][k]) - float(g0)) > c['skip_grip']: break
                    if int(s['since_grip_change'][k]) < int(c['skip_guard']): break
                    if float(np.max(np.abs(qn - s['ref'][k]))) > c['skip_ref']: break
                    k += 1; nsk += 1
                if nsk:
                    self.j = k; self.skipped += nsk; self.dwell = 0
        if self.j >= n and self.exhaust_t is None:
            self.exhaust_t = int(env._t)

    def stats(self):
        d = super().stats()
        act = max(0, self.decisions - self.settling)
        d.update(n_cap=int(self.n_cap), frac_cap=round(self.n_cap / max(1, act), 4),
                 skipped=int(self.skipped), dil_active=round(act * REPEAT / max(1, self.src['n']), 3),
                 exhaust_simstep=self.exhaust_t, fast_stalls=int(self.fast_stalls))
        return d


# ------------------------------------------------------------------ diagnostics
def src_diag(path):
    """Offline per-tape diagnostics (no sim)."""
    d = np.load(path, allow_pickle=True)
    S = d['states'].astype(np.float64); A = d['actions'].astype(np.float64); n = len(S)
    cmd = A[:, :6]; grip = np.clip(A[:, 6], 0, 1)
    dc = np.abs(np.diff(cmd, axis=0)).max(1)
    d4c = np.abs(cmd[4:] - cmd[:-4]).max(1)
    q4 = np.abs(S[4:, :6] - S[:-4, :6]).max(1)
    lead = np.abs(cmd[:-1] - S[1:, :6]).max(1)
    zl = np.where(S[:, 10] > PICK_Z)[0]
    gc = np.where(grip > GRIP_CLOSED)[0]
    tilt0 = _tilt(S[0, 11:15])
    return dict(n=int(n), idle_frac=round(float((dc < 1e-3).mean()), 3),
                T_cap=int(np.ceil(np.maximum(1, dc / DELTA_CAP).sum())),
                T_path=int(np.ceil((dc / DELTA_CAP).sum())),
                frac_win_over_cap=round(float((d4c > REPEAT * DELTA_CAP).mean()), 3),
                peak_win=round(float(d4c.max()), 3),
                frac_qwin_over_cap=round(float((q4 > REPEAT * DELTA_CAP).mean()), 3),
                lead_p99=round(float(np.percentile(lead, 99)), 3), lead_max=round(float(lead.max()), 3),
                frac_lead_over_leash=round(float((lead > LEASH_MULT * DELTA_CAP).mean()), 3),
                first_lift=int(zl[0]) if len(zl) else -1, grip_close=int(gc[0]) if len(gc) else -1,
                tilt0=round(float(tilt0), 1), can_xy0=[round(float(x), 4) for x in S[0, 8:10]])


def _tilt(q):
    w, x, y, z = [float(v) for v in q]
    # z-axis of the can in world: R[:,2]; tilt = angle from vertical
    zz = 1.0 - 2.0 * (x * x + y * y)
    return float(np.degrees(np.arccos(np.clip(zz, -1.0, 1.0))))


def fidelity(tape, src, reached, j_hist=None):
    """Follower sim-step joint path vs human measured path (joint rad, L2). Sim steps whose
    decision already aimed past the human's last frame (j >= n-1: the human tape ends at the
    lift, the follower then keeps lifting / holds cmd[-1] for the hardened pick) are excluded
    from the deviation statistics."""
    if 'sim_states' not in tape:
        return {}
    Q = np.asarray(tape['sim_states'][:, :6], np.float64)
    Qf = np.concatenate([Q, np.asarray(tape['final_state'][:6], np.float64)[None]], 0)
    if j_hist is not None:
        n_src = int(src['n'])
        # map each sim step to its decision: decisions are REPEAT sim steps except the last
        per_dec = [REPEAT] * len(j_hist)
        m = len(Q); per_dec[-1] = max(1, m - REPEAT * (len(j_hist) - 1))
        keep = np.concatenate([[j < n_src - 1] * k for j, k in zip(j_hist, per_dec)] + [[False]])
        if bool(np.asarray(tape['picked'])[-1]):
            # the human tape ends at the GEOMETRIC lift; the hardened pick needs PICK_SUSTAIN=10
            # more held frames, which the follower spends lifting further -> not in the human tape
            keep[-(10 + 3):] = False
        if keep.sum() >= 2:
            Qf = Qf[keep[:len(Qf)]]
    H = np.asarray(src['ref'][:max(2, reached)], np.float64)
    Hm = np.concatenate([np.asarray(src['q0'])[None], H], 0) if 'q0' in src else H
    # nearest-neighbour distances (small arrays: n<=~2600 x m<=1300 -> fine)
    D = np.linalg.norm(Qf[:, None, :] - Hm[None, :, :], axis=2)
    dev = D.min(1); cov = D.min(0)
    plen_f = float(np.linalg.norm(np.diff(Qf, axis=0), axis=1).sum())
    plen_h = float(np.linalg.norm(np.diff(Hm, axis=0), axis=1).sum())
    # grip-close timing: first sim step with grip cmd > GRIP_CLOSED vs human frame
    SA = np.asarray(tape['sim_actions'])
    gf = np.where(SA[:, 6] > GRIP_CLOSED)[0]
    gh = np.where(src['grip'] > GRIP_CLOSED)[0]
    # deviation at the grip-close moment
    dev_close = float(dev[int(gf[0])]) if len(gf) else None
    # can displacement before the grip closes (follower vs human)
    SS = np.asarray(tape['sim_states'])
    can_f = float(np.linalg.norm(SS[int(gf[0]) if len(gf) else -1, 8:10] - SS[0, 8:10])) if len(SS) else None
    # the same against the human's COMMAND path (== the REAL robot's measured joints, see
    # sim_fidelity_lab.py): the reference that stays valid when the simulator is changed
    Cm = np.asarray(src['cmd'][:max(2, reached)], np.float64)
    Dc = np.linalg.norm(Qf[:, None, :] - Cm[None, :, :], axis=2).min(1)
    plen_c = float(np.linalg.norm(np.diff(Cm, axis=0), axis=1).sum())
    return dict(path_ratio=round(plen_f / max(plen_h, 1e-9), 3), dev_max=round(float(dev.max()), 4),
                path_ratio_cmd=round(plen_f / max(plen_c, 1e-9), 3), dev_cmd_max=round(float(Dc.max()), 4),
                dev_cmd_p50=round(float(np.median(Dc)), 4), dev_cmd_p95=round(float(np.percentile(Dc, 95)), 4),
                dev_p95=round(float(np.percentile(dev, 95)), 4), dev_mean=round(float(dev.mean()), 4),
                cov_max=round(float(cov.max()), 4), dev_at_close=None if dev_close is None else round(dev_close, 4),
                grip_close_simstep=int(gf[0]) if len(gf) else -1, grip_close_human=int(gh[0]) if len(gh) else -1,
                can_move_before_close=None if can_f is None else round(can_f, 4))


# ------------------------------------------------------------------ runner
def plan_ics(env, src_dir, uids):
    files = sorted(glob.glob(str(REPO / src_dir / '*.npz')), key=lambda p: int(pl.Path(p).stem))
    if uids:
        want = set(int(u) for u in uids); files = [f for f in files if int(pl.Path(f).stem) in want]
    solved = set(env.success_uids); plan = []
    for f in files:
        u = int(pl.Path(f).stem)
        if u in solved:
            plan.append(dict(uid=u, src=f))
        else:   # --ic-from-tape semantics of the recorder (the cluster dH run used it for 12 uids)
            s0 = np.load(f, allow_pickle=True)['states'][0]
            plan.append(dict(uid=None, ic_uid=u, src=f, can_pos=[float(v) for v in s0[8:11]],
                             can_quat=[float(v) for v in s0[11:15]], goal_pos=None))
    return plan


def default_outdir(args):
    return f'{LAB_ROOT}/{args.config}' if args.sim == 'base' else f'{LAB_ROOT}/{args.config}@{args.sim}'


def run_shard(args):
    cfg = cfg_of(args.config, json.loads(args.override) if args.override else None)
    out = REPO / (args.outdir or default_outdir(args))
    fails = REPO / (str(out.relative_to(REPO)) + '_fails')
    out.mkdir(parents=True, exist_ok=True); fails.mkdir(parents=True, exist_ok=True)
    env_args = argparse.Namespace(no_images=not args.images)
    sim_applied = None
    if args.sim != 'base':
        sim_variants.install(args.sim)          # before the world is built (gravity comp material)
    env = build_env(env_args)
    if args.sim != 'base':
        sim_applied = sim_variants.post_build(env.genv.w, args.sim)   # arm gains / effort limits
        print(f'[lab] SIM VARIANT {sim_applied}', flush=True)
    rec = Recorder(env, images=args.images, sim_tape=True)
    fol = LabFollower(cfg, trace=args.trace)
    plan = plan_ics(env, args.src, args.uids)[args.shard_idx::args.shard_n]
    print(f'[lab] config={args.config} {cfg} | shard {args.shard_idx}/{args.shard_n} n={len(plan)}', flush=True)
    GIT = git_sha(REPO)
    stamp = dict(teacher='human', teacher_ckpt=f'lab:{args.config}|sim:{args.sim}', act_mode='mode', action_repeat=REPEAT,
                 delta_cap=DELTA_CAP, delta_leash=LEASH_MULT * DELTA_CAP, delta_ref='target',
                 pick_z=float(env.pick_z), git_sha=GIT, env_class='FullTaskEnv', recorder=RECORDER, contract=CONTRACT)
    base = args.rollout_base + 1000 * args.shard_idx
    next_id = base
    records = []
    for ic in plan:
        src = fol.load(ic['src'])
        src['q0'] = np.load(ic['src'], allow_pickle=True)['states'][0, :6].astype(np.float64)
        env_ic = {k: v for k, v in ic.items() if k != 'src'}
        tape, r = rec.run(fol, env_ic)
        r['config'] = args.config
        r['src_diag'] = src_diag(ic['src'])
        if tape is None:
            records.append(r); print(f"[roll] ic_uid={r['ic_uid']} EMPTY", flush=True); continue
        ok = tape['label'] == 'success'
        r['fid'] = fidelity(tape, src, r.get('waypoints_reached', src['n']), j_hist=fol.j_hist)
        rid = next_id; next_id += 1
        d = dict(tape, uid=int(rid), ic_uid=int(r['ic_uid']), verify='n/a', **stamp)
        errs = validate_tape(d, require_images=args.images, require_sim=True)
        if errs:
            sys.exit(f'FATAL: contract violation on rollout {rid}: {errs}')
        dest = (out if ok else fails) / f'{rid}.npz'
        np.savez_compressed(dest, **d)
        if args.trace and fol.trace:
            (fails if not ok else out).joinpath(f'{rid}_trace.json').write_text(json.dumps(fol.trace))
        r.update(rollout=int(rid), kept=bool(ok), file=str(dest.relative_to(REPO)), label=tape['label'], n=int(tape['n']))
        records.append(r)
        f = r['fid']
        print(f"[roll] ic_uid={r['ic_uid']} {r['outcome']} dec={r['decisions']} sim={r['sim_steps']} kept={ok} "
              f"dil={r['dilation']} act={r['dil_active']} stalls={r['dwell_stalls']} skip={r['skipped']} "
              f"cap={r['frac_cap']} path={f.get('path_ratio')} dev={f.get('dev_max')} ({r['seconds']}s)", flush=True)
    name = f'manifest_shard{args.shard_idx}of{args.shard_n}.json' if not args.fresh_tag else f'manifest_{args.fresh_tag}.json'
    (out / name).write_text(json.dumps(dict(config_name=args.config, config=cfg, sim=args.sim, sim_applied=sim_applied, git_sha=GIT, shard=[args.shard_idx, args.shard_n],
                                             built=time.strftime('%Y-%m-%dT%H:%M:%S'), records=records), indent=1, default=str))
    print(f'[lab] shard done kept={sum(r.get("kept", False) for r in records)}/{len(records)} -> {out / name}', flush=True)


def merge_cfg(name, outdir=None):
    out = REPO / (outdir or f'{LAB_ROOT}/{name}')
    name = name or out.name
    recs = {}; cfg = None; sim = 'base'
    for m in sorted(glob.glob(str(out / 'manifest_shard*.json')) + glob.glob(str(out / 'manifest_u*.json'))):
        b = json.loads(pl.Path(m).read_text()); cfg = b['config']; sim = b.get('sim', 'base')
        for r in b['records']:
            recs[r['ic_uid']] = r
    if not recs:
        return None
    recs = [recs[u] for u in sorted(recs)]
    kept = [r for r in recs if r.get('kept')]
    dil = np.array([r['dilation'] for r in kept]) if kept else np.array([np.nan])
    fc = np.array([r['frac_cap'] for r in kept]) if kept else np.array([np.nan])
    pr = np.array([r['fid']['path_ratio'] for r in kept]) if kept else np.array([np.nan])
    dv = np.array([r['fid']['dev_max'] for r in kept]) if kept else np.array([np.nan])
    summ = dict(config_name=name, config=cfg, sim=sim, n=len(recs), n_kept=len(kept),
                kept_uids=[r['ic_uid'] for r in kept], dropped=[(r['ic_uid'], r['outcome']) for r in recs if not r.get('kept')],
                dil_p50=round(float(np.median(dil)), 3), dil_max=round(float(dil.max()), 3),
                frac_cap_mean=round(float(fc.mean()), 4), path_ratio_p50=round(float(np.median(pr)), 3),
                path_ratio_max=round(float(pr.max()), 3), dev_max_p50=round(float(np.median(dv)), 4),
                dev_max_max=round(float(dv.max()), 4), records=recs)
    (out / 'manifest.json').write_text(json.dumps(summ, indent=1, default=str))
    return summ


def report(names=None):
    root = REPO / LAB_ROOT
    names = names or sorted(p.name for p in root.iterdir() if p.is_dir() and not p.name.endswith('_fails'))
    print(f"{'config[@sim]':30s} {'kept':>5s} {'dil50':>6s} {'dilmx':>6s} {'cap%':>6s} {'path50':>7s} {'pathmx':>7s} {'dev50':>7s} {'devmx':>7s}  dropped")
    for n in names:
        s = merge_cfg(n)
        if s is None: continue
        print(f"{n:30s} {s['n_kept']:3d}/{s['n']:<2d} {s['dil_p50']:6.3f} {s['dil_max']:6.3f} {100*s['frac_cap_mean']:6.1f} "
              f"{s['path_ratio_p50']:7.3f} {s['path_ratio_max']:7.3f} {s['dev_max_p50']:7.4f} {s['dev_max_max']:7.4f}  "
              + ' '.join(f"{u}:{o}" for u, o in s['dropped']))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config', default='baseline')
    ap.add_argument('--sim', default='base', choices=sorted(sim_variants.VARIANTS),
                    help='simulator variant (baselines/sim_variants.py); base = untouched world')
    ap.add_argument('--override', default=None, help='JSON dict of knob overrides')
    ap.add_argument('--uids', type=int, nargs='*', default=None)
    ap.add_argument('--src', default=SRC_DEFAULT)
    ap.add_argument('--outdir', default=None, help=f'default {LAB_ROOT}/<config>')
    ap.add_argument('--images', action='store_true', help='record the camera rig too (default off: state-only lab)')
    ap.add_argument('--trace', action='store_true', help='write per-decision traces next to the tapes')
    ap.add_argument('--shard-idx', type=int, default=0); ap.add_argument('--shard-n', type=int, default=1)
    ap.add_argument('--parallel', type=int, default=0, help='spawn N shard subprocesses, then merge')
    ap.add_argument('--fresh', action='store_true', help='with --parallel: ONE PROCESS PER EPISODE (fresh env each; no cross-episode solver residue), N at a time')
    ap.add_argument('--fresh-tag', default=None, help=argparse.SUPPRESS)
    ap.add_argument('--rollout-base', type=int, default=ROLLOUT_BASE)
    ap.add_argument('--merge', action='store_true'); ap.add_argument('--report', action='store_true')
    ap.add_argument('--list-configs', action='store_true')
    args = ap.parse_args()
    if args.list_configs:
        for k in CONFIGS: print(k, cfg_of(k))
        return
    if args.report:
        report(); return
    if args.merge:
        s = merge_cfg(None, args.outdir or default_outdir(args)); print(json.dumps({k: v for k, v in s.items() if k != 'records'}, indent=1)); return
    if args.parallel and args.fresh:
        out = REPO / (args.outdir or default_outdir(args))
        out.mkdir(parents=True, exist_ok=True)
        for old in glob.glob(str(out / 'manifest_*.json')):
            os.remove(old)
        uids = args.uids or sorted(int(pl.Path(p).stem) for p in glob.glob(str(REPO / args.src / '*.npz')))
        pending = list(uids); running = []
        while pending or running:
            while pending and len(running) < args.parallel:
                u = pending.pop(0)
                cmd = [sys.executable, __file__, '--config', args.config, '--sim', args.sim, '--uids', str(u), '--fresh-tag', f'u{u}',
                       '--rollout-base', str(args.rollout_base + 10 * (u - 200))]
                if args.override: cmd += ['--override', args.override]
                if args.outdir: cmd += ['--outdir', args.outdir]
                if args.images: cmd += ['--images']
                if args.trace: cmd += ['--trace']
                log = open(out / f'u{u}.log', 'w')
                running.append((subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT), u, log))
            time.sleep(2)
            still = []
            for p_, u, log in running:
                if p_.poll() is None: still.append((p_, u, log))
                else:
                    log.close()
                    if p_.returncode != 0: print(f'[lab] uid {u} process FAILED rc={p_.returncode}', flush=True)
            running = still
        s = merge_cfg(None, args.outdir or default_outdir(args))
        print(json.dumps({k: v for k, v in s.items() if k not in ('records',)}, indent=1))
        return
    if args.parallel and args.parallel > 1:
        out = REPO / (args.outdir or default_outdir(args))
        for old in glob.glob(str(out / 'manifest_*.json')):
            os.remove(old)
        procs = []
        for i in range(args.parallel):
            cmd = [sys.executable, __file__, '--config', args.config, '--sim', args.sim, '--shard-idx', str(i), '--shard-n', str(args.parallel)]
            if args.uids: cmd += ['--uids'] + [str(u) for u in args.uids]
            if args.override: cmd += ['--override', args.override]
            if args.outdir: cmd += ['--outdir', args.outdir]
            if args.images: cmd += ['--images']
            if args.trace: cmd += ['--trace']
            log = open(out.parent / f'{out.name}.shard{i}.log', 'w') if out.parent.exists() else None
            out.mkdir(parents=True, exist_ok=True)
            log = open(out / f'shard{i}.log', 'w')
            procs.append(subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT))
        for p in procs: p.wait()
        s = merge_cfg(None, args.outdir or default_outdir(args))
        print(json.dumps({k: v for k, v in s.items() if k not in ('records',)}, indent=1))
        return
    run_shard(args)


if __name__ == '__main__':
    main()
