"""SACfD on the FULL task: FullTaskEnv (staged reward) + demo transitions from
episodes_all relabeled with the SAME staged rewards.

Relabel per frame from the recorded 17-dim states, with per-frame geometric proxies
mirroring the env predicates:
    picked  : can_z > pick_z AND gripper cmd > 0.3        (exact mirror of env)
    placed  : picked-before AND can_z in shelf band       (proxy)
    contact : placed-before AND |can_xy - goal_xy| < 0.070 (proxy: cans touching)
    nested  : contact-before AND can tilt < 20 deg        (proxy)
Each stage's reward is GATED by the episode's env-measured final stage (npz 'stage'
field from collect_all_classified): an episode that never reached contact can never
grant contact reward, so proxy false-positives cannot manufacture reward. done=True
only at the nested frame (mirrors env termination).

Usage: train_sacfd_full.py --steps 400000 --demo-dir baselines/episodes_all
                           --out-dir baselines/rl/checkpoints/sacfd_full --device cuda
"""
import os
import argparse, glob, json, pathlib as pl, sys, time

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))
import demo_buffer  # noqa: E402
import pick_env  # noqa: E402
from full_env import (FullTaskEnv, STAGE_REWARD, pick_hold_held,  # noqa: E402
                      terminal_from_tape, pick_shaping_phi)

STAGE_RANK = {'no-pick': 0, 'picked': 1, 'placed': 2, 'contact': 3, 'nested': 4}
SHELF_LO, SHELF_HI = 0.12, 0.20    # can_z band for the placed proxy (BOX_TOP 0.11 + 1..7cm)
TOUCH_XY = 0.070                   # cans touching: centers ~1 diameter (0.066) apart


def tilt_from_quat(q):
    """Tilt (deg) of body z-axis from world z for wxyz quats (vectorized)."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    cz = 1.0 - 2.0 * (x * x + y * y)
    return np.degrees(np.arccos(np.clip(cz, -1.0, 1.0)))


def relabel_full(paths, pick_z, scope='full', terminal_guard=True):
    """-> (list of (obs, action, reward, next_obs, done) with staged rewards, stats).

    terminal_guard (DEFAULT ON, PREREG 2026-08-23 §4.3 / AUDIT_impl F1): the tape ends
    where the env would have ended it -- full_env.terminal_from_tape decides:
      * tip rule (grip commanded open & can tilt > TIP_DEG after the step) -> done=True
        on that transition, TIP_PENALTY (0 in pick/full scope), EVERY LATER FRAME DROPPED;
      * scope='pick': the pick transition (relabel_full's own j_pick) -> done=True,
        later frames dropped (the env terminates on the pick; the post-pick tail of a
        harvested tape is a state the online MDP never continues from);
      * tape end without a terminal -> done=False on the last transition (truncation,
        the critic bootstraps) -- unchanged.
    scope='full' keeps the staged rewards and nested terminal; only the tip rule cuts.
    terminal_guard=False reproduces the pre-2026-08-23 output byte for byte (scope is
    then ignored: pick-scope done marking stays the caller's job, as before).
    stats gains guard counters: guard_tip, guard_pick, guard_none, guard_frames_dropped.
    """
    transitions, stats = [], {k: 0 for k in STAGE_REWARD}
    stats.update(guard_tip=0, guard_pick=0, guard_none=0, guard_frames_dropped=0,
                 guard_on=bool(terminal_guard), guard_scope=scope)
    for p in paths:
        d = np.load(p, allow_pickle=True)
        s, a = d['states'].astype(np.float32), d['actions'].astype(np.float32)
        ep_stage = str(d['stage']) if 'stage' in d.files else None
        if ep_stage is None:
            raise KeyError(f'{p}: npz has no stage field — refusing to guess '
                           '(the old contact default granted rewards on failure tapes)')
        rank = STAGE_RANK.get(ep_stage, 0)
        n = len(s) - 1
        if n < 2:
            continue
        can_z = s[:, 10]; grip = a[:, 6]
        picked_f = (can_z[:-1] > pick_z) & (grip[:-1] > pick_env.GRIP_CLOSED_FRAC)
        rew = np.zeros(n, dtype=np.float32)
        done = np.zeros(n, dtype=bool)
        granted_at = {}
        j_pick = int(np.argmax(picked_f)) if picked_f.any() and rank >= 1 else -1
        if j_pick == 0:
            continue   # bad state at frame 0
        if j_pick > 0:
            rew[j_pick] += STAGE_REWARD['picked']; granted_at['picked'] = j_pick
            stats['picked'] += 1
            if rank >= 2:
                pl_f = (np.arange(n) > j_pick) & (can_z[:-1] > SHELF_LO) & (can_z[:-1] < SHELF_HI)
                j_pl = int(np.argmax(pl_f)) if pl_f.any() else -1
                if j_pl > 0:
                    rew[j_pl] += STAGE_REWARD['placed']; granted_at['placed'] = j_pl
                    stats['placed'] += 1
                    if rank >= 3:
                        dxy = np.hypot(s[:-1, 8] - s[:-1, 15], s[:-1, 9] - s[:-1, 16])
                        c_f = (np.arange(n) > j_pl) & (dxy < TOUCH_XY)
                        j_c = int(np.argmax(c_f)) if c_f.any() else -1
                        if j_c > 0:
                            rew[j_c] += STAGE_REWARD['contact']; granted_at['contact'] = j_c
                            stats['contact'] += 1
                            if rank >= 4:
                                tilt = tilt_from_quat(s[:-1, 11:15])
                                n_f = (np.arange(n) > j_c) & (tilt < 20.0)
                                j_n = int(np.argmax(n_f)) if n_f.any() else -1
                                if j_n > 0:
                                    rew[j_n] += STAGE_REWARD['nested']
                                    done[j_n] = True
                                    stats['nested'] += 1
        end = int(np.argmax(done)) + 1 if done.any() else n
        if terminal_guard:
            # ONE terminal definition (full_env.terminal_from_tape): env-consistent tip
            # transition, relabel_full's j_pick in pick scope, earliest wins; nested
            # (scope full) is already marked above. j_pick passed explicitly so the
            # guard and the reward placement can never disagree.
            tm = terminal_from_tape(d, pick_z=pick_z, scope=scope,
                                    j_pick=(j_pick if scope == 'pick' else None))
            if tm['t_term'] is not None and tm['t_term'] < end:
                t = int(tm['t_term'])
                if tm['kind'] == 'tip':
                    rew[t] += float(tm['reward'])       # TIP_PENALTY (0.0 pick/full)
                    stats['guard_tip'] += 1
                elif tm['kind'] == 'pick':
                    stats['guard_pick'] += 1
                done[t] = True
                stats['guard_frames_dropped'] += int(end - (t + 1))
                end = t + 1
            else:
                stats['guard_none'] += 1                # truncation (or nested already cut it)
        for i in range(end):
            transitions.append((s[i], a[i], float(rew[i]), s[i + 1], bool(done[i])))
    return transitions, stats


def _delta_actions(trans, cap, delta_ref):
    """Normalized delta actions for ONE episode's transitions -- one definition,
    shared by the stride-1 delta encoders and the hold-region encoder.

    'target'   : arm = clip((cmd_t - cmd_{t-1}) / cap, -1, 1), row 0 differenced
                 against the measured start pose (max |cmd_0 - q_0| over demos is
                 0.012 rad). Mirrors FullTaskEnv(delta_ref='target').
    'measured' : arm = clip((cmd_t - qmeas_t) / (5*cap), -1, 1) -- the recorded PD
                 lead, LEASH-scaled. Mirrors FullTaskEnv(delta_ref='measured').
    Grip is absolute in both: raw 0..1 -> [-1, 1].
    """
    assert delta_ref in ('target', 'measured'), delta_ref
    cmds = np.stack([t[1] for t in trans]).astype(np.float64)
    if delta_ref == 'measured':
        ref = np.stack([t[0][:6] for t in trans]).astype(np.float64)
        dq = np.clip((cmds[:, :6] - ref) / (5.0 * cap), -1.0, 1.0)
    else:
        ref = np.concatenate([trans[0][0][None, :6].astype(np.float64),
                              cmds[:-1, :6]])
        dq = np.clip((cmds[:, :6] - ref) / cap, -1.0, 1.0)
    grip = np.clip(cmds[:, 6], 0.0, 1.0) * 2.0 - 1.0
    return np.concatenate([dq, grip[:, None]], axis=1).astype(np.float32)


def relabel_hold_region(paths, pick_z, hold_k, terminal_guard=True):
    """REWARD-DENSITY relabel (2026-08-14): per-frame HOLD reward, raw actions.

    Offline mirror of FullTaskEnv(scope='pick', pick_hold_reward=True):
      * +1 on EVERY frame the honest hold condition holds -- full_env.pick_hold_held,
        the SAME function the env calls (can above pick_z AND grip commanded closed).
        pick_z is the caller's (train_rlpd passes env.pick_z); the closure threshold
        is pick_env.GRIP_CLOSED_FRAC, imported, never copied.
      * done=True on the frame where the run of consecutive held frames first reaches
        hold_k, and EVERY LATER FRAME IS DROPPED -- exactly the env's termination, so
        the critic never bootstraps through a state the env would not have continued
        from (and the demo buffer is not padded with post-success transport).
      * demos whose env-measured stage never reached 'picked' get all-zero reward and
        no terminal (unchanged negatives). The stage gate is what makes a geometric
        proxy false-positive unable to manufacture reward, as in relabel_full.

    Indexing note (deliberate deviation from relabel_full): the env pays step i's
    reward from the state REACHED, so held_i = pick_hold_held(s[i+1], a[i]).
    relabel_full instead evaluates its pick proxy at s[i] -- one frame late relative
    to the env. Here the env-exact convention is used, because the whole point of
    this lever is that the demo reward stream matches the env's per-step reward
    stream frame for frame (gate c).

    Requires FULL-LENGTH tapes (episodes_all / episodes_delta_rerecord). On a
    pick-TRUNCATED set (episodes_pick_phase_all, cut ~2 frames past the lift) almost
    no demo can show hold_k consecutive held frames -- the census reports that as
    n_terminal ~ 0 and callers should assert on it.

    Returns (transitions, census).
    """
    hold_k = int(hold_k)
    assert hold_k >= 1, hold_k
    transitions = []
    census = dict(n_demos=0, n_positive=0, n_negative=0, n_skipped_short=0,
                  n_skipped_t0=0, n_terminal=0, n_holding_never_k=0,
                  n_positive_zero_reward=0, n_frames=0, n_rewarded_frames=0,
                  total_reward=0.0, n_negative_rewarded_frames=0,
                  n_negative_proxy_frames=0, hold_lens=[], hold_starts=[],
                  hold_k=hold_k, pick_z=float(pick_z), guard_tip=0, guard_frames_dropped=0)
    for p in paths:
        d = np.load(p, allow_pickle=True)
        s, a = d['states'].astype(np.float32), d['actions'].astype(np.float32)
        ep_stage = str(d['stage']) if 'stage' in d.files else None
        if ep_stage is None:
            raise KeyError(f'{f}: npz has no stage field — refusing to guess '
                           '(the old contact default granted rewards on failure tapes)')
        rank = STAGE_RANK.get(ep_stage, 0)
        n = len(s) - 1
        if n < 2:
            census['n_skipped_short'] += 1
            continue
        # env-exact: reward for transition i comes from the state REACHED, s[i+1]
        proxy = np.asarray(pick_hold_held(s[1:, 10], a[:-1, 6], pick_z), dtype=bool)
        census['n_demos'] += 1
        if rank < 1:
            # negatives: the episode never picked (env-measured stage), so no proxy
            # positive may pay -- identical gating principle to relabel_full
            census['n_negative'] += 1
            census['n_negative_proxy_frames'] += int(proxy.sum())
            held = np.zeros(n, dtype=bool)
        else:
            census['n_positive'] += 1
            held = proxy
            if bool(pick_hold_held(s[0, 10], a[0, 6], pick_z)):
                census['n_skipped_t0'] += 1     # bad state at frame 0 (cf relabel_full)
                census['n_demos'] -= 1
                census['n_positive'] -= 1
                continue
        run, j_term = 0, -1
        for i in range(n):
            run = run + 1 if held[i] else 0
            if run >= hold_k:
                j_term = i
                break
        end = j_term + 1 if j_term >= 0 else n
        rew = held[:end].astype(np.float32)
        done = np.zeros(end, dtype=bool)
        if terminal_guard:
            # env tip rule (full_env.terminal_from_tape, scope 'full' = tip only): a
            # tape that tips before its K-th held frame ends there, done=True, reward
            # TIP_PENALTY (0), later frames dropped -- the env would have terminated.
            tm = terminal_from_tape(d, pick_z=pick_z, scope='full')
            if tm['t_term'] is not None and tm['kind'] == 'tip' and tm['t_term'] < end:
                t = int(tm['t_term'])
                census['guard_tip'] = census.get('guard_tip', 0) + 1
                census['guard_frames_dropped'] = census.get('guard_frames_dropped', 0) + int(end - (t + 1))
                end = t + 1
                rew = held[:end].astype(np.float32); rew[t] += float(tm['reward'])
                done = np.zeros(end, dtype=bool); done[t] = True
                j_term = -1                      # the hold terminal (if any) is past the tip
        if j_term >= 0:
            done[j_term] = True
            census['n_terminal'] += 1
            census['hold_lens'].append(int(rew.sum()))
            census['hold_starts'].append(int(np.argmax(held)))
        elif held.any():
            census['n_holding_never_k'] += 1
        elif rank >= 1:
            census['n_positive_zero_reward'] += 1
        census['n_frames'] += end
        census['n_rewarded_frames'] += int(rew.sum())
        census['total_reward'] += float(rew.sum())
        if rank < 1:
            census['n_negative_rewarded_frames'] += int(rew.sum())
        for i in range(end):
            transitions.append((s[i], a[i], float(rew[i]), s[i + 1], bool(done[i])))
    return transitions, census


def print_hold_census(census, tag=''):
    """One-screen reward census for the hold-region relabel (gate b)."""
    hl = np.array(sorted(census['hold_lens']), dtype=float)
    q = (lambda f: float(np.percentile(hl, f)) if hl.size else float('nan'))
    print(f'[hold-census]{" " + tag if tag else ""} K={census["hold_k"]} '
          f'pick_z={census["pick_z"]:.4f}', flush=True)
    print(f'[hold-census] demos {census["n_demos"]} = {census["n_positive"]} '
          f'>=picked + {census["n_negative"]} negatives | skipped '
          f'{census["n_skipped_short"]} short, {census["n_skipped_t0"]} bad-t0',
          flush=True)
    print(f'[hold-census] terminal (reached K consecutive held) {census["n_terminal"]} '
          f'| held-but-never-K {census["n_holding_never_k"]} | positive-with-zero-hold '
          f'{census["n_positive_zero_reward"]}', flush=True)
    frac = (100.0 * census['n_rewarded_frames'] / census['n_frames']
            if census['n_frames'] else 0.0)
    print(f'[hold-census] rewarded frames {census["n_rewarded_frames"]} / '
          f'{census["n_frames"]} transitions ({frac:.2f}%) | total reward '
          f'{census["total_reward"]:.1f}', flush=True)
    print(f'[hold-census] per-demo rewarded-frame count (terminal demos): '
          f'min {hl.min() if hl.size else float("nan"):.0f} p25 {q(25):.0f} '
          f'med {q(50):.0f} p75 {q(75):.0f} max '
          f'{hl.max() if hl.size else float("nan"):.0f}', flush=True)
    print(f'[hold-census] negatives paid {census["n_negative_rewarded_frames"]} frames '
          f'(MUST be 0); their proxy-positive frames suppressed by the stage gate: '
          f'{census["n_negative_proxy_frames"]}', flush=True)
    assert census['n_negative_rewarded_frames'] == 0, census


def hold_region_encode_transitions(paths, pick_z, cap, hold_k, delta_ref='target',
                                   terminal_guard=True):
    """Delta-encoded sibling of relabel_hold_region (the RLPD demo path).

    Same per-episode delta math as delta_encode_transitions / ..._measured (shared
    _delta_actions), applied to the hold-region relabel instead of the staged one.
    No scope argument: the hold region IS the pick scope. No action-repeat variant
    yet -- train_rlpd asserts action_repeat == 1 with this encoder rather than
    silently striding (the silent-default rule).

    Returns (transitions, census).
    """
    out, census = [], None
    for p in paths:
        trans, c = relabel_hold_region([p], pick_z, hold_k, terminal_guard=terminal_guard)
        if census is None:
            census = c
        else:
            for k, v in c.items():
                if isinstance(v, list):
                    census[k] = census[k] + v
                elif isinstance(v, (int, float)) and k not in ('hold_k', 'pick_z'):
                    census[k] = census[k] + v
        if not trans:
            continue
        acts = _delta_actions(trans, cap, delta_ref)
        out.extend((o, acts[i], r, o2, d)
                   for i, (o, _, r, o2, d) in enumerate(trans))
    return out, (census or dict(n_demos=0))


def delta_encode_transitions(paths, pick_z, scope, cap, terminal_guard=True):
    """relabel_full per episode, then replace each transition's RAW absolute
    action [6 rad, grip 0..1] with the delta-normalized action the delta_joint
    env expects: arm = clip((cmd_t - cmd_{t-1})/cap, -1, 1) (row 0 vs the
    measured start pose -- max |cmd_0 - q_0| over demos is 0.012 rad), grip =
    raw*2-1. Same encoding as to_dreamer_demos --action-encoding delta_joint;
    integrating these through FullTaskEnv(action_mode='delta_joint') reproduces
    the validated commanded replay (gate: sacfd_delta_gate below).

    terminal_guard (DEFAULT ON, 2026-08-23): relabel_full ends the tape where the env
    terminates (tip rule; the pick in scope='pick') -- see relabel_full. The pick-scope
    done marking below is then already done and idempotent; with the guard OFF it is
    the only terminal (pre-08-23 behaviour, fail tapes whole with done=False)."""
    out = []
    for p in paths:
        trans, _ = relabel_full([p], pick_z, scope=scope, terminal_guard=terminal_guard)
        if not trans:
            continue
        if scope == 'pick':
            trans = [(o, a, r, o2, True) if r >= STAGE_REWARD['picked'] else
                     (o, a, r, o2, d) for (o, a, r, o2, d) in trans]
        acts = _delta_actions(trans, cap, 'target')
        out.extend((o, acts[i], r, o2, d)
                   for i, (o, _, r, o2, d) in enumerate(trans))
    return out


def delta_encode_transitions_repeat(paths, pick_z, scope, cap, repeat, terminal_guard=True):
    """Action-repeat (decision-level) sibling of delta_encode_transitions.

    Re-encodes each demo at STRIDE = `repeat` so one decision matches exactly what
    FullTaskEnv(action_mode='delta_joint', action_repeat=repeat) executes: the same
    normalized arm delta held `repeat` sim steps advances the joint target by
    repeat*a*cap total, so to span the demo command from just-before the window to
    the window's last frame the decision action is
        a_arm = clip((cmd_end - cmd_prev) / (repeat*cap), -1, 1),
    where cmd_prev is the standing command before the window (the measured start pose
    for window 0, else the frame just before the window). Dividing by `repeat` (NOT the
    window length) makes the target REACH cmd_end after `repeat` repeats regardless of
    how many demo frames the (possibly short final) window held. This is the average of
    the `repeat` constituent stride-1 deltas, so it is <= each in magnitude -> clips
    LESS and sits comfortably inside the same leash the stride-1 gate already passes.

    Per decision: obs = window-start state; next_obs = state after the last sim step in
    the window; reward = SUM over the window (the terminal +1 is preserved); done = ANY
    within the window; grip = the LAST frame's grip command in the window (raw*2-1) --
    chosen over a majority vote so the pick's grip-close is never lost to a late-window
    closure (grip is held constant across the window in both env and eval).

    Do NOT route the stride-1 path here: repeat==1 still strides/averages trivially and
    is arithmetically identical to delta_encode_transitions, but callers select this
    sibling EXPLICITLY (the silent-default rule); repeat==1 callers keep the stride-1
    encoder.
    """
    assert int(repeat) >= 1, repeat
    repeat = int(repeat)
    out = []
    for p in paths:
        trans, _ = relabel_full([p], pick_z, scope=scope, terminal_guard=terminal_guard)
        if not trans:
            continue
        if scope == 'pick':
            trans = [(o, a, r, o2, True) if r >= STAGE_REWARD['picked'] else
                     (o, a, r, o2, d) for (o, a, r, o2, d) in trans]
        n = len(trans)
        cmds = np.stack([t[1] for t in trans]).astype(np.float64)   # (n, 7)
        C = cmds[:, :6]
        start_q = trans[0][0][:6].astype(np.float64)
        for k in range(0, n, repeat):
            end = min(k + repeat - 1, n - 1)          # last frame index in the window
            prev = start_q if k == 0 else C[k - 1]    # standing command before window
            dq = np.clip((C[end] - prev) / (repeat * cap), -1.0, 1.0)
            grip = np.clip(cmds[end, 6], 0.0, 1.0) * 2.0 - 1.0
            act = np.concatenate([dq, [grip]]).astype(np.float32)
            obs = trans[k][0]
            nobs = trans[end][3]
            rew = float(sum(trans[i][2] for i in range(k, end + 1)))
            done = bool(any(trans[i][4] for i in range(k, end + 1)))
            out.append((obs, act, rew, nobs, done))
    return out


def delta_encode_transitions_measured(paths, pick_z, scope, cap, terminal_guard=True):
    """Measured-referenced sibling of delta_encode_transitions (2026-08-14,
    user-directed after P1). Action = clip((cmd_t - qmeas_t)/cap, -1, 1) where
    qmeas_t = the demo's RECORDED measured qpos (states[t,:6]) -- the reference the
    demo files already carry. Executed in FullTaskEnv(delta_ref='measured') each
    action re-references the actual arm, so a clipped frame cannot leave the P1
    permanent offset: the next action points from wherever the arm IS toward the
    recorded command. Grip unchanged (raw*2-1)."""
    out = []
    for p in paths:
        trans, _ = relabel_full([p], pick_z, scope=scope, terminal_guard=terminal_guard)
        if not trans:
            continue
        if scope == 'pick':
            trans = [(o, a, r, o2, True) if r >= STAGE_REWARD['picked'] else
                     (o, a, r, o2, d) for (o, a, r, o2, d) in trans]
        # divide by the LEASH (env measured-mode scale): a = normalized recorded
        # PD lead. cap-scaling under-drove 5x (gate 0/5, 2026-08-14).
        acts = _delta_actions(trans, cap, 'measured')
        out.extend((o, acts[i], r, o2, d)
                   for i, (o, _, r, o2, d) in enumerate(trans))
    return out


def delta_encode_transitions_measured_repeat(paths, pick_z, scope, cap, repeat, terminal_guard=True):
    """Measured-referenced decision-level (skip-N) encoder. Per window of `repeat`
    sim steps the decision action is a = clip((cmd_end - qmeas_start)/(repeat*cap),
    -1, 1): reach the window's final recorded command from the window-START recorded
    arm pose, spread over `repeat` measured-referenced steps. Window rules match
    delta_encode_transitions_repeat exactly: grip = LAST frame, reward = SUM,
    done = ANY, obs = window start, next_obs = window end."""
    assert int(repeat) >= 1, repeat
    repeat = int(repeat)
    out = []
    for p in paths:
        trans, _ = relabel_full([p], pick_z, scope=scope, terminal_guard=terminal_guard)
        if not trans:
            continue
        if scope == 'pick':
            trans = [(o, a, r, o2, True) if r >= STAGE_REWARD['picked'] else
                     (o, a, r, o2, d) for (o, a, r, o2, d) in trans]
        n = len(trans)
        cmds = np.stack([t[1] for t in trans]).astype(np.float64)
        C = cmds[:, :6]
        for k in range(0, n, repeat):
            end = min(k + repeat - 1, n - 1)
            qref = trans[k][0][:6].astype(np.float64)   # measured qpos, window start
            # leash scaling (matches env delta_ref='measured'); window gap may exceed
            # the leash at large N -> clipped, healed by next window's re-reference.
            dq = np.clip((C[end] - qref) / (5.0 * cap), -1.0, 1.0)
            grip = np.clip(cmds[end, 6], 0.0, 1.0) * 2.0 - 1.0
            act = np.concatenate([dq, [grip]]).astype(np.float32)
            rew = float(sum(trans[i][2] for i in range(k, end + 1)))
            done = bool(any(trans[i][4] for i in range(k, end + 1)))
            out.append((trans[k][0], act, rew, trans[end][3], done))
    return out


# --- contract-v1 (record_demos.py) demo tapes: NO re-encoding ---------------------------
# PREREG_final_round_robin_2026-08-23 §4.1: a contract-v1 tape is one ROW per decision
# recorded through the learners' own MDP (FullTaskEnv delta_joint/target, repeat N, tip
# rule): actions_delta IS the learner's action, terminated IS the env's terminal. The
# loader below builds transitions verbatim -- no delta math, no relabel predicate, no
# terminal guess -- and asserts the tape's stamps against the run's env so a repeat-4
# tape can never feed a repeat-1 run silently (the silent-default rule).
CONTRACT_V1_REQUIRED = ('states', 'final_state', 'actions_delta', 'actions', 'rewards',
                        'terminated', 'truncated', 'picked', 'tipped', 'eef_pos',
                        'action_repeat', 'delta_cap', 'delta_leash', 'delta_ref',
                        'contract', 'label')


def demo_dir_sha256(paths):
    """sha256 over (basename + bytes) of the sorted npz list -- the SAME formula as
    make_dDPsucc.py's manifest content_sha256 / record_demos.py's manifest, so a
    sidecar/registry fingerprint can be compared to a set manifest directly."""
    import hashlib
    h = hashlib.sha256()
    for p in sorted(paths, key=lambda x: os.path.basename(x)):
        with open(p, 'rb') as fh:
            h.update(os.path.basename(p).encode()); h.update(fh.read())
    return h.hexdigest()


def _scalar(v):
    v = np.asarray(v)
    return v.item() if v.ndim == 0 else v


def native_demo_transitions(paths, expect, gamma=None, shaping=False, phi_scale=None):
    """contract-v1 tapes -> (transitions [(obs, a, r, next_obs, done)], census).

    expect: dict with action_repeat, delta_cap, delta_leash, delta_ref ('target') and
            optionally scope ('pick'); every tape's stamps must equal them (assert).
    shaping: add the pick-scope potential term to each transition from the RECORDED
            eef_pos / can xyz via full_env.pick_shaping_phi (the env's own function):
            r_t += gamma*phi(s_{t+1}) - phi(s_t), phi(terminal)=0 on terminated rows
            (matches FullTaskEnv.step with pick_shaping_terminal_zero=True). gamma MUST
            be the run's discount (the caller asserts it equals the env's shaping gamma).
    Transitions: (states[t], actions_delta[t], rewards[t], states[t+1] | final_state,
    terminated[t]); a truncated last row bootstraps (done=False).
    """
    if shaping:
        assert gamma is not None and 0.0 < float(gamma) < 1.0, gamma
    out = []
    census = dict(n_tapes=0, n_transitions=0, n_rewarded=0, n_success=0, n_fail=0,
                  n_tip=0, n_truncated=0, shaping_sum=0.0, first_lift=[], lens=[],
                  shaping=bool(shaping), gamma=(float(gamma) if gamma is not None else None))
    for p in paths:
        d = np.load(p, allow_pickle=True)
        missing = [k for k in CONTRACT_V1_REQUIRED if k not in d.files]
        assert not missing, f'{p}: not a contract-v1 tape, missing {missing}'
        assert str(_scalar(d['contract'])) == 'v1', (p, _scalar(d['contract']))
        sv = str(_scalar(d['sim_variant'])) if 'sim_variant' in d.files else 'base'
        want_sv = str(expect.get('sim_variant', 'base'))
        assert sv == want_sv, f'{p}: tape sim_variant={sv} but this run expects {want_sv} (world mismatch)'
        for k in ('action_repeat', 'delta_cap', 'delta_leash', 'delta_ref'):
            if k in expect:
                got, want = _scalar(d[k]), expect[k]
                ok = (str(got) == str(want)) if k == 'delta_ref' else np.isclose(float(got), float(want))
                assert ok, f'{p}: tape {k}={got} but this run expects {want} (silent-default rule)'
        s = np.asarray(d['states'], dtype=np.float32)
        fs = np.asarray(d['final_state'], dtype=np.float32)
        a = np.asarray(d['actions_delta'], dtype=np.float32)
        r = np.asarray(d['rewards'], dtype=np.float32).copy()
        # 2026-08-28: normalise the env's double-grant rows (+2 on a fast pick) to the single
        # +1 terminal of a single-stage task (full_env.py fixed the same day); counted in census.
        census['n_double_grant'] = census.get('n_double_grant', 0) + int((r > 1.0).sum()); r = np.minimum(r, 1.0)
        term = np.asarray(d['terminated'], dtype=bool)
        trunc = np.asarray(d['truncated'], dtype=bool)
        n = len(s)
        assert n >= 1 and a.shape == (n, 7) and r.shape == (n,) and term.shape == (n,), p
        assert s.shape[1] == fs.shape[0] == 17, (p, s.shape, fs.shape)
        assert not term[:-1].any(), f'{p}: terminal before the last row (contract violation)'
        assert np.all(np.abs(a) <= 1.0 + 1e-6), f'{p}: actions_delta outside [-1,1]'
        nxt = np.concatenate([s[1:], fs[None, :]], axis=0)
        if shaping:
            eef = np.asarray(d['eef_pos'], dtype=np.float64)
            assert eef.shape == (n + 1, 3), (p, eef.shape)
            can = np.concatenate([s[:, 8:11], fs[None, 8:11]], axis=0).astype(np.float64)
            phi = np.asarray([pick_shaping_phi(eef[t], can[t], scale=phi_scale) for t in range(n + 1)])
            phi_next = phi[1:].copy()
            phi_next[term] = 0.0                     # phi(terminal) = 0
            F = float(gamma) * phi_next - phi[:-1]
            r = (r + F.astype(np.float32)).astype(np.float32)
            census['shaping_sum'] += float(F.sum())
        for t in range(n):
            out.append((s[t], a[t], float(r[t]), nxt[t], bool(term[t])))
        census['n_tapes'] += 1; census['n_transitions'] += n; census['lens'].append(n)
        census['n_rewarded'] += int((np.asarray(d['rewards']) > 0).sum())
        picked = np.asarray(d['picked'], dtype=bool)
        if picked.any():
            census['n_success'] += 1; census['first_lift'].append(int(np.argmax(picked)))
        else:
            census['n_fail'] += 1
        if term.any() and bool(np.asarray(d['tipped'], dtype=bool)[int(np.argmax(term))]):
            census['n_tip'] += 1
        if not term.any():
            census['n_truncated'] += 1
            assert trunc[-1], f'{p}: no terminal and no truncation on the last row'
    return out, census


def print_native_census(c, tag=''):
    fl = np.asarray(c['first_lift'], dtype=float)
    print(f'[native-demos]{" " + tag if tag else ""} tapes {c["n_tapes"]} = '
          f'{c["n_success"]} success + {c["n_fail"]} fail (tip-terminated {c["n_tip"]}, '
          f'truncated {c["n_truncated"]}) | transitions {c["n_transitions"]} '
          f'(len p50 {np.median(c["lens"]) if c["lens"] else float("nan"):.0f}) | '
          f'rewarded {c["n_rewarded"]} | first-lift p50 '
          f'{np.median(fl) if fl.size else float("nan"):.0f} decisions | shaping '
          f'{"on, sum %.2f, gamma %s" % (c["shaping_sum"], c["gamma"]) if c["shaping"] else "off"}',
          flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--steps', type=int, default=400_000)
    ap.add_argument('--demo-dir', default='baselines/episodes_all')
    ap.add_argument('--out-dir', default='baselines/rl/checkpoints/sacfd_full')
    ap.add_argument('--duplicate', type=int, default=3)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--device', default='cuda')
    ap.add_argument('--warm-start', default=None, metavar='CKPT',
                    help='initialize actor/critic from a trained pick-SACfD .zip (same '
                         '17/7 spaces) -- staged training: explore from "can already pick"')
    ap.add_argument('--train-max-steps', type=int, default=900,
                    help='training episode horizon. Cartesian needs ~1800: the 0.11 '
                         'm/s VCAP hard-caps the arm at human teleop speed (median '
                         'demo pick frame 666, 23/67 picks past 900, median full '
                         'demo 1707) -- joint SAC could outrun the demonstrator, '
                         'cartesian cannot by construction.')
    ap.add_argument('--control', choices=['vel', 'delta'], default='vel',
                    help='cartesian control mode; must match --demo-dir')
    ap.add_argument('--cartesian', action='store_true',
                    help='CartesianFullTaskEnv (5-dim ee-velocity actions, tip '
                         'termination) + episodes_cartesian demos relabeled on the '
                         '18-dim layout. The EEF-modality RLfD leg.')
    ap.add_argument('--no-wandb', action='store_true')
    ap.add_argument('--run-name', default=None, help='wandb run name (default: out-dir stem)')
    ap.add_argument('--eval-freq', type=int, default=25_000,
                    help='video-eval subprocess cadence (0 disables)')
    ap.add_argument('--eval-max-steps', type=int, default=1200,
                    help='eval rollout horizon (#21 lever: 400 for pick-only curves)')
    ap.add_argument('--action-mode', choices=['absolute', 'delta_joint'],
                    default='absolute',
                    help='delta_joint: env actions are per-step joint-target '
                         'deltas (cap 0.025 = demo p99 speed, leash 5*cap) and '
                         'demo-buffer actions are delta-encoded to match -- the '
                         'r2dreamer fix ported to SACfD (2026-08-11). A sidecar '
                         '<out>/sacfd_final.action_mode.json records the mode '
                         'for wandb_eval --action-mode auto.')
    ap.add_argument('--gamma', type=float, default=0.998,
                    help='discount. 0.998 ~ 500-step credit window; the old '
                         'build_model default 0.98 made the demo pick terminal '
                         '(median frame 662) worth ~1.6e-6 from start states -- '
                         'a silent cause of the uniform SACfD zeros.')
    ap.add_argument('--scope', choices=['full', 'pick'], default='full',
                    help='pick: +1 and terminate on the pick (phase-1 paper core)')
    ap.add_argument('--project', default='genesis_pickaplace', help='wandb project')
    ap.add_argument('--demo-terminal-guard', choices=['on', 'off'], default='on',
                    help='on (default, 2026-08-23): demo tapes end where the env would '
                         'have terminated (tip rule; the pick in scope=pick) -- see '
                         'relabel_full / full_env.terminal_from_tape. off = pre-08-23 '
                         'tensors (fail tapes whole, done=False to the last frame).')
    args = ap.parse_args()

    t0 = time.time()
    if args.cartesian:
        from full_env import CartesianFullTaskEnv
        env = CartesianFullTaskEnv(backend='cpu', max_steps=args.train_max_steps,
                                   control=getattr(args, 'control', 'vel'),
                                   scope=args.scope)
    else:
        env = FullTaskEnv(backend='cpu', max_steps=args.train_max_steps,
                          scope=args.scope, action_mode=args.action_mode)
    print(f'[env] {type(env).__name__} built in {time.time() - t0:.1f}s '
          f'| pick_z={env.pick_z:.4f}', flush=True)

    if args.warm_start:
        from stable_baselines3 import SAC
        model = SAC.load(args.warm_start, env=env, device=args.device)
        model.num_timesteps = 0   # fresh step budget for the full-task phase
        print(f'[warm-start] loaded {args.warm_start}', flush=True)
    else:
        from train_sacfd import build_model
        model = build_model(env, args.seed, args.device, gamma=args.gamma)
        print(f'[cfg] gamma={args.gamma} scope={args.scope} action_mode={args.action_mode}', flush=True)

    paths = sorted(glob.glob(str(REPO / args.demo_dir / '*.npz')))
    assert paths, f'no npz in {args.demo_dir}'
    if args.cartesian:
        from relabel_cartesian import relabel_cartesian
        transitions, stats = relabel_cartesian(paths, env.pick_z)
    else:
        transitions, stats = relabel_full(paths, env.pick_z, scope=args.scope,
                                          terminal_guard=(args.demo_terminal_guard == 'on'))
    if args.scope == 'pick':
        # demo termination must match the env's: the pick grant ENDS the episode,
        # so its transition is terminal (done=True) or the critic bootstraps
        # through a state the env never continues from
        transitions = [(o, a, r, o2, True) if r >= STAGE_REWARD['picked'] else
                       (o, a, r, o2, d) for (o, a, r, o2, d) in transitions]
    n_r = sum(1 for t in transitions if t[2] > 0)
    print(f'[demos] {len(paths)} episodes -> {len(transitions)} transitions, '
          f'{n_r} rewarded | stage grants: {stats}', flush=True)
    if args.cartesian:
        from cartesian_env import CartesianCanEnv
        _norm = CartesianCanEnv.normalize_action
    elif args.action_mode == 'delta_joint':
        # Delta encoding needs per-episode context (delta_t = (cmd_t - cmd_{t-1})
        # / cap; row 0 from the measured start pose) -- a stateless per-transition
        # action_transform cannot do it, so re-encode the transitions' actions
        # here, per episode, with the SAME math as to_dreamer_demos (commanded
        # deltas, grip absolute [-1,1]). Transitions were built per-path in order
        # so episode boundaries are recoverable from the paths loop below.
        transitions = delta_encode_transitions(paths, env.pick_z, args.scope,
                                               env.delta_cap,
                                               terminal_guard=(args.demo_terminal_guard == 'on'))
        _norm = None
    else:
        _norm = pick_env.normalize_action
    n_added = demo_buffer.inject_into_replay_buffer(
        model, transitions, action_transform=_norm,
        duplicate=args.duplicate)
    print(f'[demos] x{args.duplicate} -> {n_added} added; buffer pos={model.replay_buffer.pos} '
          f'full={model.replay_buffer.full}', flush=True)

    from stable_baselines3.common.callbacks import CheckpointCallback, CallbackList
    from wandb_utils import init_wandb, WandbScalarCallback, VideoEvalCallback
    out = pl.Path(args.out_dir); out.mkdir(parents=True, exist_ok=True)
    run = init_wandb(args, name=args.run_name or out.name, tags=('sacfd',),
                     project=args.project)
    cbs = [CheckpointCallback(save_freq=50_000, save_path=str(out), name_prefix='sacfd'),
           WandbScalarCallback(run)]
    if args.eval_freq:
        cbs.append(VideoEvalCallback(run, out, eval_freq=args.eval_freq,
                                     max_steps=args.eval_max_steps, seed=args.seed,
                                     cartesian=args.cartesian,
                                     control=getattr(args, 'control', 'vel'),
                                     action_mode=(args.action_mode if args.action_mode != 'absolute' else None)))
    model.learn(total_timesteps=args.steps, log_interval=10, callback=CallbackList(cbs))
    model.save(str(out / 'sacfd_final'))
    # sidecar: lets wandb_eval --action-mode auto pick the right control path
    # (the silent-default bug family rule: control mode travels WITH the artifact)
    (out / 'sacfd_final.action_mode.json').write_text(
        json.dumps({'action_mode': args.action_mode}))
    if run is not None:
        run.finish()
    print(f'[full] done in {(time.time() - t0)/3600:.1f}h -> {out}/sacfd_final.zip', flush=True)


if __name__ == '__main__':
    main()
