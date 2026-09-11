#!/usr/bin/env python
"""D5 -- relabel a demonstration set's reward column BY RE-EXECUTION through the training env.

    python baselines/rl/relabel_reward.py --in <set> --out <set>_rz [--procs 8] [--limit N]

WHY THIS IS A RE-EXECUTION AND NOT A CLASSIFIER (LADDER_UNIFY_BRIEF D5)
----------------------------------------------------------------------
The previous version of this file scored tapes with its OWN offline predicates (a release
detector, a proximity test, a settle test) and wrote the resulting reward into the demo set.
That is the defect it was meant to fix, one level up: the demo buffer then pays a ladder the
environment cannot. Measured consequence -- the `_rx` prefill pays +4 on 13 of 74 human
tapes while the online env, under the ladder those runs trained on, could never pay it at
all (E2E_AUDIT_BRIEF defect 5). Buffer and env disagreed about the objective.

Re-executing the tape through `FullTaskEnv(scope='full')` -- the SAME code path training
uses, same world, same action semantics -- makes `tape reward == env reward` true BY
CONSTRUCTION rather than by two implementations agreeing. There is no predicate in this
file. It runs the env and writes down what the env paid.

WHAT CHANGES AND WHAT DOES NOT
------------------------------
Changed:   `rewards` (per decision, the unified ladder as the env paid it).
Added:     `rz_*` diagnostics (stage-grant decisions, the re-execution's end reason, the
           agreement between recorded and re-executed stage flags) and `reward_ladder`.
Unchanged: EVERYTHING else, and the action stream in particular -- the sha256 of
           `actions_delta` is asserted identical to the source before anything is written,
           per tape and over the whole set. A set whose actions moved is not a relabel.

WHERE THE REWARD CAN LEGITIMATELY DIFFER FROM THE RECORDING
-----------------------------------------------------------
The recorded tapes were produced under the OLD ladder, which terminated on the `nested`
proxy. Under the unified ladder the episode instead terminates on `slide_success` or a tip,
so a re-execution can run PAST the decision the recording stopped at, or stop before it.
Both are reported per tape (`rz_end_reason`, `rz_end_decision`) and counted in the manifest.
Decisions after the re-execution's terminal are paid 0 -- the env is in an absorbing state
and a demo row there would teach a transition that cannot occur.

ONE GENESIS WORLD PER PROCESS is a hard constraint, so `--procs N` (N <= 8) re-launches this
same script as N subprocesses with `--shard i --nshards N`; each builds one world and takes
every N-th tape. The driver merges the shard manifests.

The sim variant defaults to the tapes' own `sim_variant` stamp and is asserted, not assumed.

RECORD ONCE, SCORE MANY (user, 2026-09-11: "are you re-copying the demos every time, or
applying it as a reward layer?")
-------------------------------------------------------------------------------------------
Re-executing 146 tapes once per candidate ladder is 4x the simulation for 4x the same
trajectory: the reward column does not steer the arm, so the only thing a ladder changes about
a demonstration is WHERE THE EPISODE STOPS and WHAT IT PAYS. So:

  1. `--records-out DIR` re-executes each tape ONCE with termination suppressed
     (`env.never_terminate`, an instance knob nothing else sets) and writes a per-ENV-FRAME
     STAGE RECORD: the poses, the tool point, the commanded grip, the solver contacts and the
     env-owned stage flags -- everything StageTracker and the reward loop consume -- plus one
     end-of-episode settle (`nested_honest`).
  2. `--from-records DIR --ladder L [--far-release]` replays that record through
     `stage_predicates.StageTracker` and `full_env.LadderAccountant` in pure numpy, with no
     Genesis and no simulation, and reproduces the reward column, the grants and the terminal
     decision the direct re-execution would have produced.

That the two agree is not an assumption: `--verify-against DIR` scores a direct re-execution
and an offline replay of the same tapes and asserts the reward columns and terminal decisions
are equal element for element. It holds BY CONSTRUCTION for the reward loop (both paths call
the same `LadderAccountant`) and BY MEASUREMENT for the predicates and the sim.

Suppressing termination cannot change the trajectory BEFORE the terminal -- it only stops
`step()` from breaking out of its action-repeat loop -- so the frames the offline replay scores
are the same frames the direct run would have produced. It does mean the record holds frames
after a terminal that the direct run never saw; the offline replay discards them, exactly as
the direct run's loop does.

CAVEAT ON WHERE A RECORD MAY BE MADE. Genesis is not bit-identical across CPU classes: this
development box disagrees with the cluster's 64-core class on 6 of 74 human and 8 of 72 machine
tapes after the first contact. A record is a re-execution, so RECORDS OF RECORD MUST BE MADE ON
THE HARDWARE CLASS THE CELLS OF RECORD USE (64-core). Records made elsewhere are development
artefacts -- fine for choosing a ladder on the demonstration side, not a number for the paper.
Every record stamps its node, core count and ISA so a reader can tell which it is holding.
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

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[2]))

# The stages whose first grant is recorded per tape. `slide_success` and `home` are the paid
# terminals of the four ladders; the rest are logged so a reader can see WHY a tape paid what
# it did. Recorded under EVERY ladder, so one relabel's manifest answers "how far up any other
# ladder would this tape have got".
REPORT_STAGES = ('picked', 'placed_v2', 'contact_push', 'slide_success',
                 'nested_v2', 'released', 'pushed', 'farside', 'slide_event', 'home',
                 'settled_after_release', 'placed', 'contact', 'nested')
MAX_PROCS = 8
# --ladder -> the demo-set suffix that names it. A set carrying the wrong suffix is how a
# launcher trains one objective while believing another, so the suffix is asserted against the
# ladder (and `--far-release` appends 'f': it changes what the same ladder pays).
LADDER_SUFFIX = {'staged': '_rz', 'sparse': '_rs',
                 'nested_ramp': '_rnr', 'nested_sparse': '_rns'}
LADDER_CHOICES = tuple(LADDER_SUFFIX)
# --tip-guard -> the extra suffix letter (PHASE_PLAN amendment (aa)). The guard changes WHERE
# every tape's episode ends and what it pays after that, so a set built under it is a different
# set and must not be mistakable for one built under the rule of record. 'grip' adds nothing,
# so every set built before this amendment keeps its name.
TIP_GUARD_SUFFIX = {'grip': '', 'not_in_hand': 'h'}
TIP_GUARD_CLI = tuple(TIP_GUARD_SUFFIX)


def sha_actions(a):
    return hashlib.sha256(np.ascontiguousarray(np.asarray(a, np.float32)).tobytes()).hexdigest()


def scalar(z, k, default=None):
    if k not in z.files:
        return default
    a = np.asarray(z[k])
    return a.item() if a.shape == () else a


# ============================== STAGE RECORDS (record once, score many) =================
# The per-ENV-FRAME arrays a stage record holds. Every one is either an input StageTracker
# consumes or an input the env's own predicates consume; nothing here is derived, so the
# offline replay recomputes rather than trusts.
REC_KEYS = ('dec', 'can_pos', 'can_quat', 'goal_pos', 'goal_quat', 'tool_xy', 'grip_cmd',
            'picked', 'placed_v2', 'can_goal_contact', 'gripper_goal_contact',
            'contact', 'placed', 'nested_proxy', 'contact_push_legacy',
            'slide_success_legacy', 'tipped', 'rec_reward')


def _cpu_stamp():
    """node / cores / ISA, so a reader can tell a cluster record from a laptop one."""
    out = dict(node=os.uname().nodename, cores=os.cpu_count(), isa='unknown', cpu_model='')
    try:
        txt = open('/proc/cpuinfo').read()
        for line in txt.splitlines():
            if line.startswith('model name'):
                out['cpu_model'] = line.split(':', 1)[1].strip()
                break
        out['isa'] = 'avx512' if ' avx512f' in txt else ('avx2' if ' avx2' in txt else 'sse')
    except Exception:
        pass
    return out


class FrameRecorder:
    """Captures one row per ENV FRAME by wrapping `FullTaskEnv._step_once` on the INSTANCE.

    The wrap runs after `_step_once` returns, so `info` already carries every predicate the
    frame produced and the poses are the post-step ones (no sim step is advanced by reading
    them -- the #26 trace ablation). full_env itself is untouched: a recorder that needed a
    hook inside the env would be a lever the training path could trip over."""

    def __init__(self, env):
        from genesis_can_env import np_        # the repo's one torch->numpy helper
        self._np = np_
        self.env = env
        self.rows = []
        self._dec = 0
        self._orig = env._step_once
        env._step_once = self._wrapped

    def _wrapped(self, action):
        out = self._orig(action)
        obs, reward, term, trunc, info = out
        w, np_ = self.env.genv.w, self._np
        tool = np.asarray(self.env.genv.tool_pos(), dtype=np.float64)
        bq = np.asarray(np_(w['bottle'].get_quat()), dtype=np.float64).reshape(-1)[:4]
        gq = np.asarray(np_(w['goal'].get_quat()), dtype=np.float64).reshape(-1)[:4]
        self.rows.append(dict(
            dec=self._dec,
            can_pos=np.asarray(np_(w['bottle'].get_pos()), np.float64).reshape(-1)[:3],
            can_quat=bq,
            goal_pos=np.asarray(np_(w['goal'].get_pos()), np.float64).reshape(-1)[:3],
            goal_quat=gq,
            tool_xy=tool[:2],
            grip_cmd=float(self._grip),
            picked=bool(info.get('picked')),
            placed_v2=bool(info.get('placed_v2')),
            can_goal_contact=bool(info.get('can_goal_touch')),
            gripper_goal_contact=bool(info.get('gripper_goal_touch')),
            contact=bool(info.get('contact')),
            placed=bool(info.get('placed')),
            nested_proxy=bool(info.get('nested')),
            contact_push_legacy=bool(info.get('contact_push_legacy')),
            slide_success_legacy=bool(info.get('slide_success_legacy')),
            tipped=bool(info.get('tipped')),
            rec_reward=float(reward)))
        return out

    # the physical grip command of the decision currently being executed; set by the driver
    _grip = float('nan')

    def new_decision(self, dec, grip_phys):
        self._dec = int(dec)
        self._grip = float(grip_phys)

    def arrays(self):
        out = {}
        for k in REC_KEYS:
            vals = [r[k] for r in self.rows]
            out[k] = np.asarray(vals)
        return out

    def detach(self):
        self.env._step_once = self._orig


def grip_phys_from_action(action):
    """The physical grip command `_step_once` will apply for this decision. delta_joint maps
    a[6] in [-1,1] to (a+1)/2 -- the same affine map, read from the one place that defines it
    rather than re-derived."""
    return (float(np.clip(np.asarray(action, np.float64)[6], -1.0, 1.0)) + 1.0) / 2.0


def offline_episode(rec, ladder, far_release=False, action_repeat=4, max_steps=None,
                    tracker_kw=None, tip_guard=None):
    """Replay a stage record through StageTracker + LadderAccountant. PURE NUMPY.

    Reproduces `FullTaskEnv.step()` exactly: per ENV FRAME feed the tracker, build the same
    `info` dict `_full_scope_predicates` builds, hand it to the SAME accountant class the env
    uses, then apply the tip rule and the horizon truncation; per DECISION sum the frames and
    stop at the first terminal, as `step()`'s action-repeat loop does.

    -> dict(rewards (n_dec,), end_reason, end_decision, grants {stage: decision},
            episode = the tracker's episode dict, ramp_paid)
    """
    sys.path.insert(0, str(REPO / 'baselines'))
    sys.path.insert(0, str(REPO / 'baselines' / 'rl'))
    import full_env as FE
    from stage_predicates import StageTracker, tilt_deg

    n = int(rec['dec'].shape[0])
    n_dec = int(rec['dec'][-1]) + 1 if n else 0
    shelf_top_z = float(rec['shelf_top_z'])
    max_steps = int(rec['max_steps']) if max_steps is None else int(max_steps)
    g0 = np.asarray(rec['goal_pos'])[0]
    # tracker_kw exists for CALIBRATION SWEEPS only (baselines/diagnostics/ladder_n_*): the
    # constants of record are stage_predicates' module defaults, and any cell produced with an
    # override must say so -- `constants()` is what the provenance stamp carries.
    tr = StageTracker((float(g0[0]), float(g0[1])), shelf_top_z, far_release=far_release,
                      **(tracker_kw or {}))
    acct = FE.LadderAccountant(
        ladder, scope='full',
        pay_stages=FE.LadderAccountant.pay_stages_for('full', False))
    acct.reset()
    GRIP_OPEN = FE.FullTaskEnv.GRIP_OPEN
    TIP_DEG = FE.FullTaskEnv.TIP_DEG
    # amendment (aa): WHICH tip guard this scoring applies. None = the class default, so an
    # existing caller is unchanged. Both guards are computable EXACTLY from a stage record:
    # 'grip' from `grip_cmd`, 'not_in_hand' from the tracker's own per-frame `in_hand`, which
    # is derived from the recorded `tool_xy`/`can_pos` -- the same numbers the env used.
    tip_guard = FE.TIP_GUARD_DEFAULT if tip_guard is None else str(tip_guard)
    if tip_guard not in FE.TIP_GUARD_CHOICES:
        raise ValueError(f'unknown tip_guard {tip_guard!r}')
    TIP_SUSTAIN = int(FE.TIP_GUARD_SUSTAIN[tip_guard])
    tip_run = 0

    rewards = np.zeros(n_dec, np.float32)
    grants, end_reason, end_dec = {}, 'stream_exhausted', n_dec
    seen, stop = set(), False
    for i in range(n):
        d = int(rec['dec'][i])
        # `placed_v2` reaches the tracker STICKY (full_env: info flag OR already granted) --
        # and `_granted` at this point holds the previous frames' grants, not this one's.
        placed_granted = bool(rec['placed_v2'][i] or 'placed_v2' in acct.granted)
        flags = tr.update(
            can_pos=rec['can_pos'][i], can_quat=rec['can_quat'][i],
            goal_pos=rec['goal_pos'][i], goal_quat=rec['goal_quat'][i],
            tool_xy=rec['tool_xy'][i], grip_cmd=float(rec['grip_cmd'][i]),
            picked=bool(rec['picked'][i]),
            can_goal_contact=bool(rec['can_goal_contact'][i]),
            gripper_goal_contact=bool(rec['gripper_goal_contact'][i]),
            placed_v2=placed_granted)
        info = dict(flags)
        info['picked'] = bool(rec['picked'][i])
        info['placed_v2'] = bool(rec['placed_v2'][i])
        info['contact'] = bool(rec['contact'][i])
        info['placed'] = bool(rec['placed'][i])
        info['nested'] = bool(rec['nested_proxy'][i])
        r, terminated = acct.frame(info)
        rewards[d] += r
        for k in (set(acct.granted) | {k for k in REPORT_STAGES if info.get(k)}) - seen:
            if k in REPORT_STAGES:
                grants[k] = d
        seen |= set(acct.granted) | {k for k in REPORT_STAGES if info.get(k)}
        # the tip rule, verbatim from _step_once (TIP_PENALTY is 0.0 in full scope): the
        # guard, then the tilt, then the sustain on the CONJUNCTION. `flags['in_hand']` is
        # the tracker's own value for this frame -- exactly what the env reads out of
        # `self._track`. Under 'grip' the sustain is 1, so this branch is the old arithmetic.
        if not terminated:
            free = (float(rec['grip_cmd'][i]) < GRIP_OPEN if tip_guard == 'grip'
                    else (not bool(flags['in_hand'])))
            tip_run = tip_run + 1 if (free and tilt_deg(rec['can_quat'][i]) > TIP_DEG) else 0
        tipped = bool((not terminated) and tip_run >= TIP_SUSTAIN)
        if tipped:
            terminated = True
        truncated = (not terminated) and (i + 1) >= max_steps
        if terminated or truncated:
            end_reason = ('tipped' if tipped else 'truncated' if truncated else
                          next((s for s in acct.terminal_stages if info.get(s)), 'terminated'))
            end_dec = d + 1
            rewards[d + 1:] = 0.0
            stop = True
            break
    if not stop:
        end_dec = n_dec
    return dict(rewards=rewards, n_dec=n_dec,
                end_reason=end_reason, end_decision=end_dec, grants=grants,
                episode=tr.episode(), ramp_paid=float(acct.ramp_paid),
                granted=sorted(acct.granted), paid=sorted(acct.paid))


# ------------------------------------------------------------------------------- worker
def build_env(sim_variant, max_sim_steps, ladder, far_release=False, tip_guard=None):
    """FullTaskEnv in the END-TO-END contract MDP -- the same knobs baselines/record_demos.py
    build_env and cluster/sbatch_rlpd_e2e.sh use, every one asserted (silent-default rule)."""
    sys.path.insert(0, str(REPO / 'baselines'))
    sys.path.insert(0, str(REPO / 'baselines' / 'rl'))
    sys.path.insert(0, str(REPO / 'can_pos_recovery'))
    os.environ['GENESIS_SIM_VARIANT'] = sim_variant
    from sim_variant_hook import apply_pre, apply_post
    from full_env import FullTaskEnv, refuse_legacy_gates
    refuse_legacy_gates()
    apply_pre(sim_variant)
    t0 = time.time()
    import full_env as _FE
    tip_guard = _FE.TIP_GUARD_DEFAULT if tip_guard is None else str(tip_guard)
    env = FullTaskEnv(backend='cpu', max_steps=int(max_sim_steps), scope='full', ladder=ladder,
                      far_release=bool(far_release), tip_guard=tip_guard,
                      action_mode='delta_joint', delta_cap=0.025, delta_leash_mult=5.0,
                      action_repeat=4, delta_ref='target', camera_rig=False)
    apply_post(env, sim_variant)
    assert env.scope == 'full' and env.action_mode == 'delta_joint' and env.delta_ref == 'target'
    assert env.action_repeat == 4 and abs(env.delta_cap - 0.025) < 1e-12
    assert abs(env.delta_leash - 0.125) < 1e-12, env.delta_leash
    assert env.genv.max_steps >= 10 ** 8, 'inner env must never truncate (#26)'
    assert not env.goalward_shaping, 'a relabel must use the unshaped ladder'
    assert env.tip_guard == tip_guard, (env.tip_guard, tip_guard)
    print(f'[env] built in {time.time() - t0:.1f}s | variant {sim_variant} | max_sim {env.max_steps} | '
          f'shelf_top_z {env.shelf_top_z:.3f} | tip_guard {env.tip_guard}', flush=True)
    return env


def tape_layout(z):
    """'segment' = the r2dreamer-native FULL-scope rows BOTH launchers assert on
    ($W/demos_state_full/<arm>, keys state/action/reward/is_terminal, action backward-shifted
    so action[t] led INTO state[t] and action[0] == 0). 'v1' = a contract-v1 recorder tape
    (states/actions_delta/rewards/n). The `_rx` sets in flight are segment sets, so that is
    the primary format; v1 is supported because the recorder writes it and the DP set builder
    reads it."""
    keys = set(z.files)
    if {'state', 'action', 'reward', 'is_terminal'} <= keys:
        return 'segment'
    if {'states', 'actions_delta', 'rewards', 'n'} <= keys:
        return 'v1'
    raise ValueError(f'unrecognised tape layout; keys = {sorted(keys)}')


def tape_stream(z, layout):
    """-> (first_state (17,), executed actions (n,7), recorded per-decision rewards (n,),
    expected next states (n,17) or None). `n` is the number of DECISIONS actually executed."""
    if layout == 'segment':
        st = np.asarray(z['state'], np.float64)
        ac = np.asarray(z['action'], np.float32)
        rw = np.asarray(z['reward'], np.float64).reshape(-1)
        T = st.shape[0]
        assert ac.shape == (T, 7) and rw.shape == (T,), (ac.shape, rw.shape, T)
        assert not np.abs(ac[0]).any(), 'segment layout: action[0] must be 0 (backward-shifted)'
        # decision t executes action[t+1] and lands on state[t+1]
        return st[0], ac[1:], rw[1:], st[1:]
    n = int(scalar(z, 'n'))
    st = np.asarray(z['states'], np.float64)
    return (st[0], np.asarray(z['actions_delta'], np.float32)[:n],
            np.asarray(z['rewards'], np.float64)[:n],
            np.concatenate([st[1:n], np.asarray(z['final_state'], np.float64)[None, :]]) if n >= 1 else None)


def reset_to_tape_ic(env, z):
    """Restore the tape's initial condition and PROVE it matched.

    uid ICs go through the env's own `reset(options={'uid': ...})`, so the placement table,
    the corrected static goal and every piece of FullTaskEnv bookkeeping are the env's code,
    not a copy of it. The restored can pose is then checked against the tape's own first
    state -- an IC that silently differs is the whole failure mode of a re-execution.

    Segment tapes carry no `ic_uid`, so their start is restored from state[0] directly
    (can xyz 8:11, can quat 11:15, goal xy 15:17) -- which is the SAME information the uid
    path would look up, and it is then verified against that state either way."""
    ic_uid = scalar(z, 'ic_uid')
    s0 = tape_stream(z, tape_layout(z))[0]
    if ic_uid is not None and int(ic_uid) in env.genv.placements:
        obs, _ = env.reset(options={'uid': int(ic_uid)})
        how = f'uid {int(ic_uid)}'
    else:
        from replay_harness import STATIC_BOTTLE_POSITION
        obs, _ = env.reset_to(dict(can_pos=[float(v) for v in s0[8:11]],
                                   can_quat=[float(v) for v in s0[11:15]],
                                   goal_pos=(float(STATIC_BOTTLE_POSITION[0]),
                                             float(STATIC_BOTTLE_POSITION[1]),
                                             float(env.genv.w['goal_start_z']))))
        how = 'pose'
    env.genv._calib_tool_offset()
    got = np.asarray(obs, np.float64)
    d_can = float(np.linalg.norm(got[8:11] - s0[8:11]))
    d_goal = float(np.linalg.norm(got[15:17] - s0[15:17]))
    return how, d_can, d_goal


def relabel_one(env, path, out_dir, ic_tol):
    z = np.load(path, allow_pickle=True)
    layout = tape_layout(z)
    if layout == 'v1':
        assert str(scalar(z, 'scope')) == 'full', f'{path}: scope={scalar(z, "scope")!r}, expected full'
        assert str(scalar(z, 'contract')) == 'v1', f'{path}: not a contract-v1 tape'
    _s0, acts, old_rew, want_states = tape_stream(z, layout)
    n = int(acts.shape[0])
    src_sha = sha_actions(acts)

    how, d_can, d_goal = reset_to_tape_ic(env, z)
    assert d_can <= ic_tol and d_goal <= ic_tol, (
        f'{os.path.basename(path)}: restored IC differs from the tape ({how}): can {d_can * 1000:.1f} mm, '
        f'goal {d_goal * 1000:.1f} mm > {ic_tol * 1000:.1f} mm. Refusing to relabel a tape against a '
        f'different start.')

    rew = np.zeros(n, np.float32)
    grants, end_reason, end_dec = {}, 'stream_exhausted', n
    seen = set()
    tipped = False
    # FIDELITY, measured rather than assumed: a re-execution is only a RELABEL if it follows
    # the recorded trajectory. Reported PER CHANNEL GROUP, because a raw max over the 17-dim
    # state is meaningless -- state[7] is grip EFFORT, a summed control force of order 1-5 N,
    # which dominates every other channel and would read as a catastrophic divergence when
    # the arm is tracking to 0.0004 rad (measured, 2026-09-11). What matters is the CAN.
    can_dev = 0.0        # m, centre-to-centre
    joint_dev = 0.0      # rad, worst arm joint
    t0 = time.time()
    for t in range(n):
        obs, r, term, trunc, info = env.step(acts[t])
        rew[t] = float(r)
        if want_states is not None and t < len(want_states):
            got = np.asarray(obs, np.float64)
            can_dev = max(can_dev, float(np.linalg.norm(got[8:11] - want_states[t][8:11])))
            joint_dev = max(joint_dev, float(np.abs(got[:6] - want_states[t][:6]).max()))
        tipped = tipped or bool(info.get('tipped'))
        gr = set(env._granted) | {k for k in REPORT_STAGES if info.get(k)}
        for k in gr - seen:
            if k in REPORT_STAGES:
                grants[k] = int(t)
        seen |= gr
        if term or trunc:
            end_reason = ('tipped' if info.get('tipped') else
                          'truncated' if trunc else
                          next((s for s in env.terminal_stages if info.get(s)), 'terminated'))
            end_dec = t + 1
            break

    d = {k: z[k] for k in z.files}
    old_sum = float(np.asarray(old_rew, np.float64).sum())
    if layout == 'segment':
        # reward[t+1] is the reward of the transition INTO state[t+1]; row 0 carries none,
        # exactly as the source set has it.
        out_rew = np.zeros(d['reward'].shape, np.float32)
        out_rew[1:1 + n] = rew
        d['reward'] = out_rew
        assert sha_actions(np.asarray(d['action'], np.float32)[1:]) == src_sha, 'action stream moved'
    else:
        d['rewards'] = rew
        assert sha_actions(np.asarray(d['actions_delta'], np.float32)[:n]) == src_sha, 'action stream moved'
    d['rz_layout'] = layout
    d['rz_can_dev_max_m'] = np.float64(can_dev)
    d['rz_joint_dev_max_rad'] = np.float64(joint_dev)
    d['reward_ladder'] = json.dumps(dict(env.stage_reward))
    d['reward_ladder_name'] = str(env.ladder)
    d['reward_relabelled'] = 'reexecution_2026-09-10'
    d['rz_grants'] = json.dumps(grants)
    d['rz_end_reason'] = end_reason
    d['rz_end_decision'] = np.int64(end_dec)
    d['rz_decisions'] = np.int64(n)
    d['rz_actions_sha256'] = src_sha
    # agreement between what the RECORDING said and what the re-execution did. Reported,
    # never asserted: the recording ran under the old ladder, whose terminal differed, so a
    # disagreement is information about the two MDPs rather than a failure of this script.
    agree = {}
    for k in ('picked', 'contact', 'nested'):
        if k in z.files:
            agree[k] = [bool(np.asarray(z[k], bool)[:n].any()), bool(k in seen)]
    if 'tipped' in z.files:
        # `tipped` is NOT a stage and never enters `_granted`, so it has to be tracked from
        # info -- reading it out of `seen` reported False for every tipped re-execution.
        agree['tipped'] = [bool(np.asarray(z['tipped'], bool)[:n].any()), bool(tipped)]
    d['rz_recorded_vs_reexecuted'] = json.dumps(agree)
    # Ladder-N diagnostics, produced under EVERY ladder because the tracker computes them
    # under every ladder: how far the can was pushed from the far side, and whether the
    # release that counts was far enough out for the `far_release` switch to accept it.
    ep = env.tracker.episode() if env.tracker is not None else {}
    d['rz_slide_gain_m'] = np.float64(ep.get('slide_gain_m', float('nan')))
    d['rz_release_dist_m'] = np.float64(ep.get('release_dist_m', float('nan')))
    d['rz_release_far'] = bool(ep.get('release_far', False))
    d['rz_far_release'] = bool(env.far_release)
    d['rz_tip_guard'] = str(env.tip_guard)
    out = os.path.join(out_dir, os.path.basename(path))
    np.savez_compressed(out, **d)
    return dict(file=os.path.basename(path), n=n, layout=layout, ic=how,
                ic_can_mm=round(d_can * 1000, 2), ic_goal_mm=round(d_goal * 1000, 2),
                can_dev_max_m=round(can_dev, 6), joint_dev_max_rad=round(joint_dev, 6),
                old_reward=old_sum,
                new_reward=float(rew.sum()), grants=grants, end_reason=end_reason,
                end_decision=end_dec, actions_sha256=src_sha,
                slide_gain_m=round(float(ep.get('slide_gain_m', float('nan'))), 6),
                release_dist_m=round(float(ep.get('release_dist_m', float('nan'))), 6),
                release_far=bool(ep.get('release_far', False)),
                agree=agree, seconds=round(time.time() - t0, 1))


def set_meta(in_dir, files, args):
    """Provenance of the SOURCE set, read from repeat.json (segment sets, what both launchers
    assert on) or from the tapes' own stamps (contract-v1 sets). Never defaulted."""
    rj = os.path.join(in_dir, 'repeat.json')
    if os.path.exists(rj):
        m = json.load(open(rj))
        assert str(m.get('scope')) == 'full', f'{rj}: scope={m.get("scope")!r}, expected full'
        assert m.get('with_state') is True and int(m.get('state_dim') or 0) == 17, m
        assert m.get('reward_from_tape') is True, f'{rj}: rewards must come from the tape'
        assert int(m['action_repeat']) == 4 and abs(float(m['delta_cap']) - 0.025) < 1e-9, m
        return dict(kind='segment', sim_variant=str(m['sim_variant']),
                    max_sim_steps=int(args.max_sim_steps or 2400), repeat_json=m)
    z0 = np.load(files[0], allow_pickle=True)
    return dict(kind='v1', sim_variant=str(scalar(z0, 'sim_variant')),
                max_sim_steps=int(args.max_sim_steps or scalar(z0, 'max_sim_steps', 2400)),
                repeat_json=None)


def record_one(env, path, out_dir, ic_tol, sim_variant, max_sim_steps):
    """Re-execute ONE tape with termination suppressed and write its per-frame stage record.

    The record is ladder-INDEPENDENT: nothing in it depends on which rungs pay, so one record
    scores every candidate ladder offline. `never_terminate` is set on the instance for the
    duration and cleared after."""
    z = np.load(path, allow_pickle=True)
    layout = tape_layout(z)
    _s0, acts, old_rew, want_states = tape_stream(z, layout)
    n = int(acts.shape[0])
    src_sha = sha_actions(acts)
    how, d_can, d_goal = reset_to_tape_ic(env, z)
    assert d_can <= ic_tol and d_goal <= ic_tol, (
        f'{os.path.basename(path)}: restored IC differs from the tape ({how}): can {d_can * 1000:.1f} mm, '
        f'goal {d_goal * 1000:.1f} mm > {ic_tol * 1000:.1f} mm.')
    env.never_terminate = True
    rec = FrameRecorder(env)
    can_dev = joint_dev = 0.0
    t0 = time.time()
    try:
        for t in range(n):
            rec.new_decision(t, grip_phys_from_action(acts[t]))
            obs, r, term, trunc, info = env.step(acts[t])
            if want_states is not None and t < len(want_states):
                got = np.asarray(obs, np.float64)
                can_dev = max(can_dev, float(np.linalg.norm(got[8:11] - want_states[t][8:11])))
                joint_dev = max(joint_dev, float(np.abs(got[:6] - want_states[t][:6]).max()))
        arrs = rec.arrays()
    finally:
        rec.detach()
        env.never_terminate = False
    # ONE end-of-episode settle, at the end of the WHOLE stream: the settled reference column
    # (`nested_honest`) that nested_v2 is validated against. It steps the sim, so it runs once
    # and last -- after every frame the record holds.
    end = env.genv.end_of_episode()
    meta = dict(
        source_tape=os.path.abspath(path), layout=layout, n_decisions=n,
        action_repeat=int(env.action_repeat), max_steps=int(env.max_steps),
        shelf_top_z=float(env.shelf_top_z), sim_variant=str(sim_variant),
        actions_sha256=src_sha, reward_recorded_total=float(np.asarray(old_rew, np.float64).sum()),
        ic=how, ic_can_mm=round(d_can * 1000, 2), ic_goal_mm=round(d_goal * 1000, 2),
        can_dev_max_m=round(can_dev, 6), joint_dev_max_rad=round(joint_dev, 6),
        nested_honest=bool(end['nested']), slide_success_settle=bool(end['slide_success']),
        slide_route=str(end.get('slide_route')),
        record_ladder=str(env.ladder), never_terminate=True,
        node=json.dumps(_cpu_stamp()), provenance=json.dumps(env.provenance()),
        seconds=round(time.time() - t0, 1))
    out = os.path.join(out_dir, os.path.basename(path))
    np.savez_compressed(out, **arrs, **{k: np.asarray(v) for k, v in meta.items()})
    meta['file'] = os.path.basename(path)
    meta['frames'] = int(arrs['dec'].shape[0])
    return meta


def run_record_shard(args, files, meta):
    sv = args.sim_variant or meta['sim_variant']
    # the ladder is irrelevant to a RECORD, and so is the tip guard: `never_terminate` means
    # nothing stops, and the record holds the guard's INPUTS (grip_cmd, tool_xy, can_pos) so
    # either guard can be applied to it offline. The record is stamped with the guard the
    # recording env carried, for provenance only.
    env = build_env(sv, meta['max_sim_steps'], 'staged', tip_guard=args.tip_guard)
    rows = []
    for i, f in enumerate(files):
        r = record_one(env, f, args.records_out, args.ic_tol, sv, meta['max_sim_steps'])
        rows.append(r)
        print(f'[{i + 1}/{len(files)}] {r["file"]}: {r["n_decisions"]} decisions / {r["frames"]} frames, '
              f'nested_honest={int(r["nested_honest"])}, can_dev {r["can_dev_max_m"] * 1000:.1f} mm '
              f'[{r["seconds"]:.0f}s]', flush=True)
    return rows, sv


def offline_one(rec_path, src_path, out_dir, ladder, far_release, tip_guard=None):
    """Score one stage record under one ladder and write the relabelled tape (same schema the
    direct path writes, so the two are diffable file for file)."""
    t0 = time.time()
    rz = np.load(rec_path, allow_pickle=True)
    res = offline_episode(rz, ladder, far_release=far_release, tip_guard=tip_guard)
    z = np.load(src_path, allow_pickle=True)
    layout = tape_layout(z)
    _s0, acts, old_rew, _ = tape_stream(z, layout)
    n = int(acts.shape[0])
    assert res['n_dec'] == n, (res['n_dec'], n, rec_path)
    src_sha = sha_actions(acts)
    assert str(scalar(rz, 'actions_sha256')) == src_sha, (
        f'{os.path.basename(src_path)}: the record was made from a different action stream')
    rew = np.asarray(res['rewards'], np.float32)
    d = {k: z[k] for k in z.files}
    if layout == 'segment':
        out_rew = np.zeros(d['reward'].shape, np.float32)
        out_rew[1:1 + n] = rew
        d['reward'] = out_rew
        assert sha_actions(np.asarray(d['action'], np.float32)[1:]) == src_sha, 'action stream moved'
    else:
        d['rewards'] = rew
        assert sha_actions(np.asarray(d['actions_delta'], np.float32)[:n]) == src_sha, 'action stream moved'
    ep = res['episode']
    d['rz_layout'] = layout
    d['rz_can_dev_max_m'] = np.float64(scalar(rz, 'can_dev_max_m', float('nan')))
    d['rz_joint_dev_max_rad'] = np.float64(scalar(rz, 'joint_dev_max_rad', float('nan')))
    d['reward_ladder'] = json.dumps(_ladder_reward_dict(ladder))
    d['reward_ladder_name'] = str(ladder)
    d['reward_relabelled'] = 'offline_from_stage_record_2026-09-11'
    d['rz_grants'] = json.dumps(res['grants'])
    d['rz_end_reason'] = res['end_reason']
    d['rz_end_decision'] = np.int64(res['end_decision'])
    d['rz_decisions'] = np.int64(n)
    d['rz_actions_sha256'] = src_sha
    d['rz_slide_gain_m'] = np.float64(ep['slide_gain_m'])
    d['rz_release_dist_m'] = np.float64(ep['release_dist_m'])
    d['rz_release_far'] = bool(ep['release_far'])
    d['rz_far_release'] = bool(far_release)
    d['rz_tip_guard'] = str(tip_guard or 'grip')
    d['rz_ramp_paid'] = np.float64(res['ramp_paid'])
    d['rz_nested_honest'] = bool(scalar(rz, 'nested_honest', False))
    d['rz_record'] = os.path.abspath(rec_path)
    np.savez_compressed(os.path.join(out_dir, os.path.basename(src_path)), **d)
    return dict(file=os.path.basename(src_path), n=n, layout=layout,
                ic=str(scalar(rz, 'ic', '')),
                ic_can_mm=float(scalar(rz, 'ic_can_mm', 0.0)),
                ic_goal_mm=float(scalar(rz, 'ic_goal_mm', 0.0)),
                can_dev_max_m=float(scalar(rz, 'can_dev_max_m', 0.0)),
                joint_dev_max_rad=float(scalar(rz, 'joint_dev_max_rad', 0.0)),
                old_reward=float(np.asarray(old_rew, np.float64).sum()),
                new_reward=float(rew.sum()), grants=res['grants'],
                end_reason=res['end_reason'], end_decision=int(res['end_decision']),
                actions_sha256=src_sha,
                slide_gain_m=round(float(ep['slide_gain_m']), 6),
                release_dist_m=round(float(ep['release_dist_m']), 6),
                release_far=bool(ep['release_far']),
                ramp_paid=round(float(res['ramp_paid']), 6),
                nested_honest=bool(scalar(rz, 'nested_honest', False)),
                agree={}, seconds=round(time.time() - t0, 2))


def _ladder_reward_dict(ladder):
    sys.path.insert(0, str(REPO / 'baselines'))
    sys.path.insert(0, str(REPO / 'baselines' / 'rl'))
    import full_env as FE
    return FE.ladder_spec(ladder)[0]


def run_shard(args, files, meta):
    sv = args.sim_variant or meta['sim_variant']
    env = build_env(sv, meta['max_sim_steps'], args.ladder, args.far_release, args.tip_guard)
    rows = []
    for i, f in enumerate(files):
        z = np.load(f, allow_pickle=True)
        if meta['kind'] == 'v1':     # per-tape stamps exist only on recorder tapes
            assert str(scalar(z, 'sim_variant')) == sv, (f, scalar(z, 'sim_variant'), sv)
            assert int(scalar(z, 'action_repeat')) == 4 and abs(float(scalar(z, 'delta_cap')) - 0.025) < 1e-9, f
            assert str(scalar(z, 'delta_ref')) == 'target', f
        r = relabel_one(env, f, args.out, args.ic_tol)
        rows.append(r)
        print(f'[{i + 1}/{len(files)}] {r["file"]}: {r["n"]} decisions, reward {r["old_reward"]:.1f} -> '
              f'{r["new_reward"]:.1f}, end {r["end_reason"]}@{r["end_decision"]}, '
              f'can_dev {r["can_dev_max_m"] * 1000:.1f} mm / joint {r["joint_dev_max_rad"]:.5f} rad, '
              f'grants { {k: v for k, v in sorted(r["grants"].items())} } [{r["seconds"]:.0f}s]', flush=True)
    return rows, sv


# -------------------------------------------------------------------------------- driver
def summarize(rows, files, in_dir, out_dir, sv, ladder, far_release=False, method=None,
              tip_guard=None):
    sys.path.insert(0, str(REPO / 'baselines'))
    sys.path.insert(0, str(REPO / 'baselines' / 'rl'))
    import full_env
    per_rung = {k: sum(1 for r in rows if k in r['grants']) for k in REPORT_STAGES}
    ends = {}
    for r in rows:
        ends[r['end_reason']] = ends.get(r['end_reason'], 0) + 1
    gains = sorted(float(r.get('slide_gain_m') or 0.0) for r in rows)
    def _pct(q):
        if not gains:
            return None
        return float(np.percentile(np.asarray(gains, float), q))
    man = dict(
        set=os.path.basename(os.path.normpath(out_dir)), source=os.path.abspath(in_dir),
        built=time.strftime('%Y-%m-%dT%H:%M:%S'), builder='baselines/rl/relabel_reward.py (D5 re-execution)',
        method=(method or
                're-executed through FullTaskEnv(scope=full) on the training code path; the reward column is '
                'what the env paid, not what a classifier scored'),
        sim_variant=sv, scope='full', ladder=ladder, far_release=bool(far_release),
        tip_guard=str(tip_guard or full_env.TIP_GUARD_DEFAULT),
        tip_guard_sustain_frames=int(full_env.TIP_GUARD_SUSTAIN[
            str(tip_guard or full_env.TIP_GUARD_DEFAULT)]),
        contract='v1', action_repeat=4, delta_cap=0.025,
        delta_ref='target',
        n_tapes=len(rows), decisions_total=int(sum(r['n'] for r in rows)),
        reward_total_old=float(sum(r['old_reward'] for r in rows)),
        reward_total_new=float(sum(r['new_reward'] for r in rows)),
        tapes_granting=per_rung, end_reasons=ends,
        # Ladder-N demo-side columns (LADDER_N_DEMO_CHECK_2026-09-11)
        slide_gain_p10_m=_pct(10), slide_gain_p50_m=_pct(50), slide_gain_p90_m=_pct(90),
        slide_gain_max_m=(gains[-1] if gains else None),
        n_tapes_slide_gain_over_10cm=sum(1 for g in gains if g >= 0.10),
        n_tapes_slide_gain_over_1cm=sum(1 for g in gains if g >= 0.01),
        n_tapes_release_far=sum(1 for r in rows if r.get('release_far')),
        n_tapes_nested_honest=sum(1 for r in rows if r.get('nested_honest')),
        actions_sha256=hashlib.sha256(''.join(r['actions_sha256'] for r in
                                              sorted(rows, key=lambda x: x['file'])).encode()).hexdigest(),
        ladder_provenance=full_env.ladder_provenance(
            ladder, None, far_release, str(tip_guard or full_env.TIP_GUARD_DEFAULT)),
        ladder_stamp=full_env.ladder_stamp(
            ladder, None, far_release, str(tip_guard or full_env.TIP_GUARD_DEFAULT)),
        node=_cpu_stamp(),
        layout=sorted({r['layout'] for r in rows}),
        # Trajectory fidelity of the re-execution, per channel group (see relabel_one).
        # A tape whose CAN diverges is one whose reward column describes a different
        # trajectory from the recorded one -- the relabel must run on the hardware class the
        # tapes were recorded on, and these numbers are how a reader checks that it did.
        can_dev_max_m=max(r['can_dev_max_m'] for r in rows),
        can_dev_p50_m=float(np.median([r['can_dev_max_m'] for r in rows])),
        joint_dev_max_rad=max(r['joint_dev_max_rad'] for r in rows),
        n_tapes_can_dev_over_1cm=sum(1 for r in rows if r['can_dev_max_m'] > 0.01),
        per_tape=sorted(rows, key=lambda r: r['file']))
    return man


def write_repeat_json(man, rows, in_dir, out_dir, ladder, tip_guard=None):
    """Emit the set manifest BOTH LAUNCHERS GATE ON (`repeat.json`), inherited from the source.

    `manifest.json` (above) is this builder's own record; neither launcher reads it.
    `cluster/sbatch_rlpd_e2e.sh` and `cluster/wmfix_full.sbatch` both open
    `<set>/repeat.json` and assert sim_variant / scope=full / with_state / action_repeat /
    reward_from_tape / delta_cap / terminal_reward / n_written, and the RLPD one additionally
    asserts the SELECTION attestation (`one_per_ic_first` for the de-selected machine arm).
    A relabelled set without this file cannot be trained on at all.

    The selection flags are INHERITED VERBATIM from the source manifest and never re-derived
    from a CLI flag -- that is the 2026-09-09 defect (`b756259`): `to_dreamer_native.py`
    stamped `one_per_ic_first` from its own argv, so a set selected upstream recorded
    `False`, a FALSE CLAIM rather than a missing key, and the launcher gate correctly refused
    8 jobs. Here the source dict is copied and only the counts the relabel actually changed
    are overwritten.

    `terminal_reward` is one of the inherited SOURCE stamps: it describes how
    `to_dreamer_native.py` built the tapes, not the re-executed reward column. It is kept
    because the world-model launcher asserts it; it is not evidence about this ladder.
    """
    rj = os.path.join(in_dir, 'repeat.json')
    if not os.path.exists(rj):
        print(f'NOTE: {in_dir} has no repeat.json (contract-v1 layout); none written for {out_dir}')
        return None
    raw = open(rj, 'rb').read()
    src = json.loads(raw.decode())
    m = dict(src)
    dec = [int(r['n']) for r in rows]
    n_pick = sum(1 for r in rows if 'picked' in r['grants'])
    m.update(
        n_written=len(rows),
        total_reward=float(man['reward_total_new']),
        n_pick=n_pick, n_nopick=len(rows) - n_pick,
        decisions_min=int(min(dec)), decisions_max=int(max(dec)),
        decisions_median=int(np.median(dec)),
        src=os.path.abspath(in_dir),
        src_manifest_sha=hashlib.sha256(raw).hexdigest(),
        generator='baselines/rl/relabel_reward.py (D5 re-execution)',
        created=time.strftime('%Y-%m-%dT%H:%M:%S'),
        relabel=dict(
            ladder=ladder, ladder_stamp=man['ladder_stamp'],
            # amendment (aa): which tip guard produced this reward column. A launcher gates
            # on repeat.json, so the guard has to be readable there and not only in the
            # builder's own manifest.
            tip_guard=man['tip_guard'],
            tip_guard_sustain_frames=man['tip_guard_sustain_frames'],
            source_set=os.path.abspath(in_dir), source_generator=src.get('generator'),
            source_total_reward=src.get('total_reward'),
            source_n_pick=src.get('n_pick'), source_n_nopick=src.get('n_nopick'),
            actions_sha256=man['actions_sha256'],
            selection_inherited={k: src.get(k) for k in ('one_per_ic_best', 'one_per_ic_first')
                                 if k in src},
            tapes_granting=man['tapes_granting'], end_reasons=man['end_reasons'],
            can_dev_max_m=man['can_dev_max_m'], can_dev_p50_m=man['can_dev_p50_m'],
            n_tapes_can_dev_over_1cm=man['n_tapes_can_dev_over_1cm'],
            inherited_source_stamps=['terminal_reward', 'reward_from_tape', 'grant_slack_decisions',
                                     'n_tipped_terminal', 'n_cap_truncated', 'n_double_grant'],
            is_terminal='unchanged from the source tape (LADDER_IMPL_NOTES §4): the re-execution '
                        'terminal is recorded per tape as rz_end_reason/rz_end_decision, and the '
                        'action stream is kept whole so its sha256 still matches the source'),
    )
    json.dump(m, open(os.path.join(out_dir, 'repeat.json'), 'w'), indent=1)
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--in', dest='inp', required=True,
                    help='source set: an r2dreamer-native FULL-scope SEGMENT dir (with repeat.json -- what both '
                         'launchers assert on, e.g. $W/demos_state_full/dHfull_all) or a contract-v1 recorder set')
    ap.add_argument('--out', default=None,
                    help="destination; the suffix is ASSERTED against --ladder (%s), plus a trailing 'f' "
                         "when --far-release is on, so a set can never be handed to a launcher expecting "
                         "a different objective. Not needed with --records-out."
                         % ', '.join(f'{k}{v}' for k, v in LADDER_SUFFIX.items()))
    ap.add_argument('--ladder', choices=LADDER_CHOICES, default='staged',
                    help="WHICH ladder the re-execution pays (FullTaskEnv(ladder=...)). Same code path, "
                         "same action streams, same world -- only the reward column differs. It is "
                         "recorded in the manifest's ladder_provenance.")
    ap.add_argument('--far-release', action='store_true',
                    help="Ladder N switch: the release that counts for farside/home must happen at least "
                         "0.10 m from the goal, so a drop-and-nudge cannot pay for a slide. Part of the "
                         "ladder stamp and of the output-set suffix.")
    ap.add_argument('--tip-guard', choices=TIP_GUARD_CLI, required=True,
                    help="REQUIRED, no default (PHASE_PLAN amendment (aa)). WHICH guard the tip rule "
                         "uses -- it decides where every episode ENDS and therefore what the tape pays "
                         "after that point. 'grip' = the rule of record (commanded grip < 0.3, sustain 1), "
                         "which every set built before 2026-09-11 used; 'not_in_hand' = the tracker's "
                         "in_hand (|tool_xy - can_xy| >= 0.025 m, no gripper term) sustained 4 env frames "
                         "together with the tilt clause. Recorded in the manifest's ladder_provenance and "
                         "in repeat.json, so a set can never be handed to a launcher expecting the other.")
    ap.add_argument('--records-out', default=None,
                    help='RECORD MODE: re-execute each tape ONCE with termination suppressed and write a '
                         'per-env-frame stage record here. Ladder-independent; score it with --from-records.')
    ap.add_argument('--from-records', default=None,
                    help='OFFLINE MODE: score the stage records in this directory under --ladder, with no '
                         'Genesis and no simulation. --in must still name the SOURCE set (the action '
                         'streams are copied from it and their sha256 asserted against the record).')
    ap.add_argument('--verify-against', default=None,
                    help='VERIFY MODE: score --in directly (re-execution) AND offline from the stage '
                         'records in this directory, then assert the two reward columns and terminal '
                         'decisions are equal. Writes nothing but a report.')
    ap.add_argument('--sim-variant', default=None, help="default: the tapes' own stamp (asserted equal across the set)")
    ap.add_argument('--max-sim-steps', type=int, default=None, help="default: the tapes' own max_sim_steps stamp")
    ap.add_argument('--ic-tol', type=float, default=0.002,
                    help='m; the restored can/goal must match the tape first state within this (default 2 mm)')
    ap.add_argument('--procs', type=int, default=1, help=f'parallel worker processes, <= {MAX_PROCS} (one world each)')
    ap.add_argument('--limit', type=int, default=None, help='first N tapes only (dry runs)')
    ap.add_argument('--shard', type=int, default=None, help='worker mode: this shard index')
    ap.add_argument('--nshards', type=int, default=None, help='worker mode: total shards')
    ap.add_argument('--dry-run', action='store_true', help='print the plan and exit')
    args = ap.parse_args()

    modes = [bool(args.records_out), bool(args.from_records), bool(args.verify_against)]
    if sum(modes) > 1:
        sys.exit('FATAL: --records-out, --from-records and --verify-against are three modes; pick one')
    if not args.records_out and not args.verify_against:
        if not args.out:
            sys.exit('FATAL: --out is required unless --records-out or --verify-against is given')
        want_suffix = (LADDER_SUFFIX[args.ladder] + ('f' if args.far_release else '')
                       + TIP_GUARD_SUFFIX[args.tip_guard])
        if not os.path.basename(os.path.normpath(args.out)).endswith(want_suffix):
            sys.exit(f'FATAL: --ladder {args.ladder}'
                     f'{" --far-release" if args.far_release else ""} --tip-guard {args.tip_guard} '
                     f'writes a {want_suffix} set, but '
                     f'--out is {os.path.basename(os.path.normpath(args.out))!r}. The suffix is how a '
                     f'launcher tells them apart; refusing to write a mislabelled set.')
    files = sorted(glob.glob(os.path.join(args.inp, '*.npz')))
    assert files, f'no npz in {args.inp}'
    if args.limit:
        files = files[:int(args.limit)]
    if args.out:
        os.makedirs(args.out, exist_ok=True)
    if args.records_out:
        os.makedirs(args.records_out, exist_ok=True)

    meta = set_meta(args.inp, files, args)

    # ---- OFFLINE MODE: no Genesis, no world, no sim -------------------------------------
    if args.from_records:
        t_off = time.time()
        rows = []
        for i, f in enumerate(files):
            rec = os.path.join(args.from_records, os.path.basename(f))
            if not os.path.exists(rec):
                sys.exit(f'FATAL: no stage record for {os.path.basename(f)} in {args.from_records}')
            r = offline_one(rec, f, args.out, args.ladder, args.far_release, args.tip_guard)
            rows.append(r)
            print(f'[{i + 1}/{len(files)}] {r["file"]}: reward {r["old_reward"]:.1f} -> {r["new_reward"]:.1f}, '
                  f'end {r["end_reason"]}@{r["end_decision"]}, slide_gain {r["slide_gain_m"] * 1000:.1f} mm, '
                  f'release {r["release_dist_m"] * 1000:.0f} mm (far={int(r["release_far"])}), '
                  f'grants { {k: v for k, v in sorted(r["grants"].items())} } [{r["seconds"]:.2f}s]', flush=True)
        man = summarize(rows, files, args.inp, args.out, args.sim_variant or meta['sim_variant'],
                        args.ladder, args.far_release, tip_guard=args.tip_guard,
                        method=('scored OFFLINE from per-frame stage records through '
                                'stage_predicates.StageTracker + full_env.LadderAccountant -- the same '
                                'accountant object the env runs; records made by --records-out'))
        man['stage_records'] = os.path.abspath(args.from_records)
        man['offline_seconds'] = round(time.time() - t_off, 2)
        json.dump(man, open(os.path.join(args.out, 'manifest.json'), 'w'), indent=1)
        rep = write_repeat_json(man, rows, args.inp, args.out, args.ladder, args.tip_guard)
        _report(man, rep, args.out)
        return

    # ---- VERIFY MODE: direct re-execution vs the offline replay --------------------------
    if args.verify_against:
        sv = args.sim_variant or meta['sim_variant']
        env = build_env(sv, meta['max_sim_steps'], args.ladder, args.far_release, args.tip_guard)
        bad = 0
        t_dir = t_off = 0.0
        for i, f in enumerate(files):
            rec = os.path.join(args.verify_against, os.path.basename(f))
            assert os.path.exists(rec), f'no stage record for {os.path.basename(f)}'
            z = np.load(f, allow_pickle=True)
            layout = tape_layout(z)
            _s0, acts, _old, _ = tape_stream(z, layout)
            n = int(acts.shape[0])
            how, d_can, d_goal = reset_to_tape_ic(env, z)
            assert d_can <= args.ic_tol and d_goal <= args.ic_tol, (f, d_can, d_goal)
            t0 = time.time()
            direct = np.zeros(n, np.float32)
            d_end, d_reason = n, 'stream_exhausted'
            for t in range(n):
                _o, r, term, trunc, info = env.step(acts[t])
                direct[t] = float(r)
                if term or trunc:
                    d_reason = ('tipped' if info.get('tipped') else 'truncated' if trunc else
                                next((s for s in env.terminal_stages if info.get(s)), 'terminated'))
                    d_end = t + 1
                    break
            t_dir += time.time() - t0
            t0 = time.time()
            res = offline_episode(np.load(rec, allow_pickle=True), args.ladder,
                                  far_release=args.far_release, tip_guard=args.tip_guard)
            t_off += time.time() - t0
            off = np.asarray(res['rewards'], np.float32)
            same_r = bool(np.array_equal(direct, off))
            same_e = bool(d_end == res['end_decision'] and d_reason == res['end_reason'])
            if not (same_r and same_e):
                bad += 1
                nz = np.nonzero(direct != off)[0]
                print(f'  MISMATCH {os.path.basename(f)}: reward equal={same_r} '
                      f'(first differing decision {nz[0] if len(nz) else None}, '
                      f'sum {direct.sum():.3f} vs {off.sum():.3f}), '
                      f'end direct {d_reason}@{d_end} vs offline {res["end_reason"]}@{res["end_decision"]}',
                      flush=True)
            else:
                print(f'[{i + 1}/{len(files)}] {os.path.basename(f)}: OK  reward {direct.sum():.1f}, '
                      f'end {d_reason}@{d_end}', flush=True)
        print('\n' + '=' * 78)
        print(f'VERIFY ladder={args.ladder} far_release={args.far_release} '
              f'tip_guard={args.tip_guard}: '
              f'{len(files) - bad}/{len(files)} tapes identical (reward column AND terminal decision)')
        print(f'wall clock: direct re-execution {t_dir:.1f}s, offline replay {t_off:.2f}s '
              f'({(t_dir / t_off) if t_off else float("nan"):.0f}x)')
        sys.exit(1 if bad else 0)
    if args.dry_run:
        print(f'[dry] {len(files)} tapes {args.inp} ({meta["kind"]} layout) -> {args.out}, '
              f'ladder {args.ladder}, tip_guard {args.tip_guard}, procs {args.procs}, '
              f'variant {meta["sim_variant"]}')
        for f in files[:5]:
            z = np.load(f, allow_pickle=True)
            _s0, a, r, _ = tape_stream(z, tape_layout(z))
            print('   ', os.path.basename(f), 'decisions', int(a.shape[0]),
                  'recorded reward', float(np.asarray(r).sum()))
        return

    work_dir = args.records_out or args.out
    shard_fn = run_record_shard if args.records_out else run_shard

    # ---- worker ----
    if args.shard is not None:
        assert args.nshards and 0 <= args.shard < args.nshards
        mine = [f for i, f in enumerate(files) if i % args.nshards == args.shard]
        print(f'[shard {args.shard}/{args.nshards}] {len(mine)} tapes', flush=True)
        rows, sv = shard_fn(args, mine, meta)
        json.dump(dict(rows=rows, sim_variant=sv),
                  open(os.path.join(work_dir, f'_shard{args.shard}.json'), 'w'), indent=1)
        return

    # ---- driver ----
    t_wall = time.time()
    n_proc = max(1, min(int(args.procs), MAX_PROCS, len(files)))
    if n_proc == 1:
        rows, sv = shard_fn(args, files, meta)
    else:
        base = [sys.executable, os.path.abspath(__file__), '--in', args.inp,
                '--ladder', args.ladder, '--tip-guard', args.tip_guard,
                '--ic-tol', str(args.ic_tol)]
        base += ['--records-out', args.records_out] if args.records_out else ['--out', args.out]
        if args.far_release:
            base += ['--far-release']
        if args.limit:
            base += ['--limit', str(args.limit)]
        if args.sim_variant:
            base += ['--sim-variant', args.sim_variant]
        base += ['--max-sim-steps', str(meta['max_sim_steps'])]
        procs = [subprocess.Popen(base + ['--shard', str(i), '--nshards', str(n_proc)])
                 for i in range(n_proc)]
        rcs = [pp.wait() for pp in procs]
        assert all(rc == 0 for rc in rcs), f'shard failures: {rcs}'
        rows, svs = [], set()
        for i in range(n_proc):
            d = json.load(open(os.path.join(work_dir, f'_shard{i}.json')))
            rows += d['rows']; svs.add(d['sim_variant'])
        assert len(svs) == 1, svs
        sv = svs.pop()
    assert len(rows) == len(files), (len(rows), len(files))
    for f in glob.glob(os.path.join(work_dir, '_shard*.json')):
        os.remove(f)

    if args.records_out:
        man = dict(kind='stage_records', source=os.path.abspath(args.inp),
                   dir=os.path.abspath(args.records_out),
                   built=time.strftime('%Y-%m-%dT%H:%M:%S'),
                   builder='baselines/rl/relabel_reward.py --records-out',
                   method=('each tape re-executed ONCE through FullTaskEnv(scope=full) with termination '
                           'suppressed; one row per ENV FRAME of every input the stage predicates and the '
                           'reward loop consume. Ladder-independent: score with --from-records.'),
                   sim_variant=sv, n_tapes=len(rows),
                   # amendment (aa): a record is guard-INDEPENDENT (nothing terminates) and
                   # holds both guards' inputs, so this stamp is provenance, not a constraint.
                   recording_env_tip_guard=str(args.tip_guard),
                   tip_guard_applicable=list(TIP_GUARD_CLI),
                   frames_total=int(sum(r['frames'] for r in rows)),
                   decisions_total=int(sum(r['n_decisions'] for r in rows)),
                   n_nested_honest=sum(1 for r in rows if r['nested_honest']),
                   can_dev_max_m=max(r['can_dev_max_m'] for r in rows),
                   can_dev_p50_m=float(np.median([r['can_dev_max_m'] for r in rows])),
                   n_tapes_can_dev_over_1cm=sum(1 for r in rows if r['can_dev_max_m'] > 0.01),
                   node=_cpu_stamp(), wall_seconds=round(time.time() - t_wall, 1), procs=n_proc,
                   hardware_caveat=('Genesis is not bit-identical across CPU classes. Records of record '
                                    'must be made on the class the cells of record use (64-core).'),
                   per_tape=sorted(rows, key=lambda r: r['file']))
        json.dump(man, open(os.path.join(args.records_out, 'manifest.json'), 'w'), indent=1)
        print('\n' + '=' * 78)
        print(f'stage records        : {man["n_tapes"]} tapes, {man["frames_total"]} frames '
              f'({man["decisions_total"]} decisions)')
        print(f'nested_honest        : {man["n_nested_honest"]}/{man["n_tapes"]}')
        print(f'can deviation        : p50 {man["can_dev_p50_m"] * 1000:.2f} mm, max '
              f'{man["can_dev_max_m"] * 1000:.1f} mm, {man["n_tapes_can_dev_over_1cm"]} tapes over 1 cm')
        print(f'node                 : {man["node"]}')
        print(f'wall                 : {man["wall_seconds"]:.0f}s on {n_proc} processes')
        print(f'wrote                : {args.records_out}')
        return

    man = summarize(rows, files, args.inp, args.out, sv, args.ladder, args.far_release,
                    tip_guard=args.tip_guard)
    man['wall_seconds'] = round(time.time() - t_wall, 1)
    json.dump(man, open(os.path.join(args.out, 'manifest.json'), 'w'), indent=1)
    rep = write_repeat_json(man, rows, args.inp, args.out, args.ladder, args.tip_guard)
    _report(man, rep, args.out)


def _report(man, rep, out):
    print('\n' + '=' * 78)
    if rep is not None:
        print(f'repeat.json          : n_written={rep["n_written"]} total_reward={rep["total_reward"]:.1f} '
              f'n_pick={rep["n_pick"]} one_per_ic_first={rep.get("one_per_ic_first")} '
              f'one_per_ic_best={rep.get("one_per_ic_best")}')
    print(f'tapes                : {man["n_tapes"]}  ({man["decisions_total"]} decisions)')
    print(f'reward sum  recorded : {man["reward_total_old"]:.1f}')
    print(f'reward sum  RE-EXEC  : {man["reward_total_new"]:.1f}')
    print(f'tapes granting       : { {k: v for k, v in man["tapes_granting"].items() if v} }')
    print(f'episode end reasons  : {man["end_reasons"]}')
    if man.get('slide_gain_p50_m') is not None:
        print(f'slide_gain (m)       : p10 {man["slide_gain_p10_m"]:.4f} p50 {man["slide_gain_p50_m"]:.4f} '
              f'p90 {man["slide_gain_p90_m"]:.4f} max {man["slide_gain_max_m"]:.4f}; '
              f'>=1cm {man["n_tapes_slide_gain_over_1cm"]}, >=10cm {man["n_tapes_slide_gain_over_10cm"]}')
        print(f'release_far          : {man["n_tapes_release_far"]}/{man["n_tapes"]} tapes released '
              f'>= 0.10 m from the goal')
    print(f'tip guard            : {man["tip_guard"]} '
          f'(sustain {man["tip_guard_sustain_frames"]} env frames)')
    print(f'ladder               : {man["ladder_stamp"]}')
    print(f'wrote                : {out}')


if __name__ == '__main__':
    main()
