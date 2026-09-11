"""FullTaskEnv: gymnasium wrapper around GenesisCanEnv for the FULL task (plan E).

scope='full' runs a UNIFIED LADDER (LADDER_UNIFY_BRIEF_2026-09-10, D1/D2), one definition
shared by {RLPD}, {r2dreamer} and {DP}, selected by the `ladder=` CONSTRUCTOR ARGUMENT and
never by an environment variable:
    ladder='staged' (default)  picked +1, placed_v2 +1, contact_push +2,
                               slide_success +4 (TERMINAL)          max return 8
    ladder='sparse'            nested_v2 +1 (TERMINAL)              max return 1
The two differ in reward and terminal ONLY: every logged stage, tracker diagnostic, the tip
rule, the episode record and the observation are identical.
The tip rule also terminates. Nothing else does -- in particular the legacy `nested`
proxy is computed and logged but never paid and never terminal, because its clauses are
a subset of the slide's and it used to end the episode on the first frame of the slide
window, leaving the +4 rung unpayable in training (E2E_AUDIT_BRIEF defect 5).
`contact_push` and `slide_success` come from baselines/stage_predicates.StageTracker,
fed once per env frame after the sim step; both require `placed_v2` to have been granted
first, so pressing a still-HELD can against the goal earns nothing.
Truncates at max_steps (full/pick default 900 sim steps; the e2e protocol passes 1200).
Same normalized [-1,1]^7 action convention as PickOnlyEnv (see pick_env.py docstring).

scope='pick':  +1 and terminate on the pick grant.
    Constructor pick_hold_reward=True (pick scope ONLY, default False) switches the
    pick reward from that single terminal grant to a PER-STEP HOLD reward: +1 on
    EVERY step the honest pick condition holds (pick_hold_held: can above pick_z AND
    gripper commanded closed), terminating after pick_hold_k (default 25) CONSECUTIVE
    held steps. Literature precedent (paper/rlpd_literature_comparison_2026-08-13.md
    RQ1/RQ5): ManiSkill pays +1 per solved step and terminates on success; sparse
    Adroit (RLPD's own sparse-manipulation domain, Ball et al. 2023) pays +1 per
    solved step to the horizon, so its return IS the fraction of solved timesteps.
    Our terminal-only variant put 66 rewarded frames in 83,465 demo transitions
    (0.08%) -- ~1000x sparser than any published RLPD setup and the top-ranked
    explanatory delta for slow ignition. This flag closes that gap; the honest
    EVALUATED metric is unchanged (eval still asks "did it pick").
scope='place': reset restores a random banked POST-PICK entry state
    (constructor entry_bank=; default baselines/pick_entry_states.json = one
    entry per demo at the pick grant. A DENSE bank -- a JSON LIST of entries,
    e.g. baselines/place_entry_states_dense.json from make_place_entry_bank.py,
    every 25th frame along each demo's carry segment -- is sampled uniformly
    over ENTRIES; arm qpos + finger closure + held can pose +
    goal), settles ~20 steps holding the entry commands and verifies the can is
    still held (resamples otherwise); +1 and terminate on PLACED_V2 = grip cmd
    released (<0.45) + can in shelf footprint/z-band + tilt<20 deg, sustained 10
    consecutive frames. Default cap 600 steps. Tip rule applies in all scopes;
    scope='place' ONLY also pays PLACE_TIP_PENALTY (-0.25) on the tip
    termination (other scopes keep TIP_PENALTY = 0.0, termination only).
    Constructor shaping=True (place only, TRAINING-ONLY, default False) adds a
    potential-based dense term toward the shelf + a per-step cost; the honest
    reported metric remains the sparse placed_v2 terminal (see PLACE_SHAPING_*).
"""
import os
import sys
import json
import pathlib as pl

import numpy as np
import gymnasium as gym
from gymnasium import spaces

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))
from genesis_can_env import GenesisCanEnv, np_  # noqa: E402
from pick_env import (STATE_DIM, ACT_DIM, ARM_LO, ARM_HI,  # noqa: E402
                      GRIP_CLOSED_FRAC, denormalize_action)
sys.path.insert(0, str(REPO / 'can_pos_recovery'))
from replay_harness import (tilt_deg, in_shelf_footprint, BOX_TOP_Z,  # noqa: E402
                            BOX_POS, HARDCODED_START, gripper_targets)

import hashlib          # noqa: E402  (ladder provenance stamp, D6)
import subprocess        # noqa: E402
from stage_predicates import StageTracker   # noqa: E402  (Lane-1 module; D1 "one shared module")

# ============================== THE LADDERS (LADDER_UNIFY_BRIEF D1/D2, 2026-09-10) ======
# No environment-variable gate, ever. `FULLENV_REWARD_X` is GONE -- the two learners trained
# on different ladders precisely because a gate was silently inert in one of three code trees
# (E2E_AUDIT_BRIEF §2), so the gate itself is the defect, not its default. Which ladder a run
# uses is a CONSTRUCTOR ARGUMENT (`ladder=`), named by the launcher and carried in the
# provenance stamp, so no run can be ambiguous about the objective it optimised.
#
# 'staged' (default, the D2 ladder):
#   rung           reward  terminal   definition
#   picked           1       no       genesis_can_env's hardened held-can flag (unchanged)
#   placed_v2        1       no       release + shelf footprint + z-band + tilt, 10 frames
#   contact_push     2       no       placed_v2 GRANTED + can<->goal contact + tool on the
#                                     far side + no gripper<->goal contact  (stage_predicates)
#   slide_success    4      YES       placed_v2 granted + pushed + nested_v2 (stage_predicates)
#   Max return 8.0 -- the registered return_clamp 8 needs no revision.
#
# 'sparse' (user, 2026-09-11): ONE rung, the outcome itself.
#   nested_v2        1      YES       the state-only arrived-and-at-rest predicate (D3)
#   Max return 1.0. NOTE the return clamp must follow the ladder: r2dreamer's clamp is the
#   task's max return, so a sparse run needs 1.0, not 8.0.
#
# The two ladders differ in REWARD AND TERMINAL ONLY. Every logged stage, every tracker
# diagnostic, the tip rule, the episode record and the observation are identical, so an
# arm-vs-arm comparison across ladders is a comparison of objectives and nothing else.
#
# `tipped` terminates with its existing penalty (0.0 outside scope='place'). NOTHING ELSE
# TERMINATES: the (x) batch terminated on the UNPAID `nested` proxy, whose clauses are a
# subset of the slide's, so the episode ended on the first frame of the slide window and
# the +4 rung could never be paid in training (E2E_AUDIT_BRIEF defect 5).
# --- LADDER N (user, 2026-09-11; LADDER_UNIFY_BRIEF "Ladder N") -------------------------
# "nest such that max reward (or sparse reward) comes from release, moving the gripper to the
# opposite side of the can, and sliding it towards the goal can. If we didn't think sparse was
# going to work we could ramp up reward for the slide and give a big boost for contact."
#
# 'nested_sparse':  home 1.0, TERMINAL. Nothing else pays. Max return 1. This is the
#                   user's "version of sparse that enforces the slide": `home` is
#                   slide_event AND nested_v2, and `slide_event` is the three clauses
#                   they named -- the can was put DOWN, the tool went to the OPPOSITE
#                   side of it from the goal, and the can TRAVELLED goalward from there.
#                   Its constants are calibrated on the 74 human tapes, not chosen.
# 'nested_ramp':    picked 1 -> placed_v2 1 -> farside 1 -> a DENSE slide ramp worth up to 2
#                   -> home 4, TERMINAL. Max return 9.
#
# Two structures the first two ladders did not need, both declared here and read by
# LadderAccountant, never re-implemented at a call site:
#   `requires` -- a rung pays only once the rung below it has been REACHED. The tracker
#       already enforces the chain geometrically (farside needs released needs placed_v2 and
#       picked), so this is a second, explicit statement of the same order: a ladder whose
#       rungs are "each REQUIRES the previous one" should say so in the table a reader checks.
#   `ramp`   -- the one dense term. Paid incrementally as a MONOTONE tracker quantity grows:
#       reward = scale * min(1, value / span), and each frame pays the increase since the last
#       frame. `slide_gain_m` only grows on new minima of the can-goal distance made from the
#       far side, so the ramp cannot be farmed by oscillating the can (stage_predicates
#       `_update_slide_gain`), and it can never be clawed back.
LADDERS = {
    'staged': dict(stage_reward=dict(picked=1.0, placed_v2=1.0, contact_push=2.0, slide_success=4.0),
                   terminal=('slide_success',)),
    'sparse': dict(stage_reward=dict(nested_v2=1.0),
                   terminal=('nested_v2',)),
    'nested_sparse': dict(stage_reward=dict(home=1.0),
                          terminal=('home',)),
    'nested_ramp': dict(stage_reward=dict(picked=1.0, placed_v2=1.0, farside=1.0, home=4.0),
                        requires=dict(placed_v2='picked', farside='placed_v2',
                                      slide_event='farside', home='slide_event'),
                        # span 0.05 m per PHASE_PLAN amendment (aa): the 13 human sim-slides
                        # gain 1.1-6.4 cm, so at 0.10 m none reached the ramp maximum (mean
                        # 7.80 of 9); at 0.05 m they average 8.39 and four saturate it
                        # (paper/LADDER_N_DEMO_CHECK_2026-09-11.md section 5).
                        ramp=dict(key='slide_gain_m', scale=2.0, span=0.05, requires='farside'),
                        terminal=('home',)),
}
LADDER_DEFAULT = 'staged'
# The ladders whose rungs are the Ladder-N predicates. Used by callers that must pick a
# `far_release` default or a return clamp without hard-coding a name list twice.
NESTED_LADDERS = ('nested_sparse', 'nested_ramp')

# --- THE TIP GUARD (PHASE_PLAN amendment (aa); measured in paper/TIP_RULE_2026-09-11.md) ---
# The tip rule is `tilt_deg(can) > TIP_DEG` AND a GUARD that says the can is not being held.
# Threshold (60 deg), termination and penalty (0.0 outside scope='place') are UNCHANGED by
# this amendment; only the guard moves, and only when a caller asks for it.
#
#   'grip'        the rule of record and the class default: the COMMANDED grip
#                 `a_phys[6] < GRIP_OPEN (0.3)`, fired on the first frame that satisfies it.
#                 Every run before 2026-09-11, the ladder pilot included, ran this.
#   'not_in_hand' amendment (aa): the StageTracker's `not in_hand`
#                 (|tool_xy - can_xy| >= stage_predicates.HELD_LEVER_M), NO gripper term,
#                 required on TIP_GUARD_SUSTAIN consecutive ENV FRAMES (= one decision at
#                 action_repeat 4) TOGETHER with the tilt clause.
#
# Why (Lane 6, all 146 demonstration tapes of the pilot sets): the grip guard fires on 6 of
# its 25 firings while the can is still in the gripper (lever 0.002-0.021 m), so `tipped`
# labels a leaning RELEASE; and it is silent on 29 of the 48 tapes that put a free can flat,
# because the demonstrator's fingers are commanded >= 0.31 then (the amendment-(p) fist
# posture). The tracker guard measures 0 in-hand firings, 2 misses and 0 lost recoveries.
#
# THE SUSTAIN IS ON THE CONJUNCTION, not on the guard alone -- that is what Lane 6 measured
# (`tip_rule_probe._first_sustained((tilt > T) & guard, k)`), and the 4 frames are what
# removes the corpus's single recovery (machine 250: free at 67.7 deg on one decision, back
# at 18.8 deg on the next).
TIP_GUARD_CHOICES = ('grip', 'not_in_hand')
TIP_GUARD_DEFAULT = 'grip'          # NEVER change: it is the pilot's and every prior run's rule
# Consecutive env frames the (tilt AND guard) conjunction must hold before the rule fires.
# 'grip' keeps 1 so the rule of record is bit-identical, sustain and all.
TIP_GUARD_SUSTAIN = {'grip': 1, 'not_in_hand': 4}


def ladder_spec(ladder=LADDER_DEFAULT):
    """(stage_reward dict, terminal-stage tuple) for a named ladder. Unknown names RAISE --
    a typo must not silently fall back to the default objective.

    NOTE this returns the two fields every caller has always read. The `requires` and `ramp`
    fields of a Ladder-N spec are read through `ladder_extras()`; a caller that only knows
    about stage_reward/terminal therefore sees a truthful (if incomplete) view rather than a
    wrong one."""
    if ladder not in LADDERS:
        raise ValueError(f'unknown ladder {ladder!r}; choose from {sorted(LADDERS)}')
    spec = LADDERS[ladder]
    return dict(spec['stage_reward']), tuple(spec['terminal'])


def ladder_extras(ladder=LADDER_DEFAULT):
    """(requires dict, ramp dict or None) -- the Ladder-N structure. Empty/None elsewhere."""
    if ladder not in LADDERS:
        raise ValueError(f'unknown ladder {ladder!r}; choose from {sorted(LADDERS)}')
    spec = LADDERS[ladder]
    ramp = spec.get('ramp')
    return dict(spec.get('requires') or {}), (dict(ramp) if ramp else None)


# Every non-full scope pays exactly ONE terminal +1 (pick: STAGE_REWARD['picked'] and terminate;
# place: the placed_v2 terminal; contact/carrycontact: the contact grant; reach/touchgoal/
# reach_goal: +1.0) -- the ladder is irrelevant there, and the clamp of record for the pick
# recipe is 1.0 (DV3_DEBUG_2026-09-05, RESULTS_WM_HUMAN_VS_MACHINE_2026-09-04).
PHASE_SCOPE_MAX_RETURN = 1.0


def max_return(ladder=LADDER_DEFAULT, scope='full'):
    """Maximum per-episode return for THIS ladder in THIS scope. r2dreamer's return_clamp MUST
    equal this. Scope-aware since 2026-09-11: the first version ignored `scope` and refused the
    pick recipe's clamp of 1.0 because the nominal ladder ('staged') sums to 8."""
    if scope != 'full':
        return float(PHASE_SCOPE_MAX_RETURN)
    _ramp = ladder_extras(ladder)[1]
    return float(sum(ladder_spec(ladder)[0].values())
                 + (float(_ramp['scale']) if _ramp else 0.0))


# Module-level view of the DEFAULT ladder. Kept because callers import it by name
# (terminal_from_tape's pick reward, evaluator banners); a run's actual ladder is
# env.stage_reward / env.terminal_stages, which is what the reward loop reads.
STAGE_REWARD, TERMINAL_STAGES = ladder_spec(LADDER_DEFAULT)
# Computed, logged, and entered into `_granted` -- but paying NOTHING (D2/D3). Kept for
# continuity with every stored row: `nested` is the withdrawn training proxy, `placed` the
# stale base-world band, `contact` the carry-in predicate, `nested_v2` its replacement.
LOGGED_STAGES = ('picked', 'placed', 'placed_v2', 'contact', 'contact_push',
                 'nested', 'nested_v2', 'slide_success', 'farside', 'slide_event', 'home')
# CartesianFullTaskEnv is a DIFFERENT arm (4-DOF teleop actions) and is not part of the
# end-to-end unification. It keeps the ladder it has always run so its behaviour is
# byte-identical to every cartesian run on record.
_CARTESIAN_STAGE_REWARD = dict(picked=1.0, placed=1.0, contact=2.0, nested=4.0)
PLACE_ENTRY_BANK = REPO / 'baselines' / 'pick_entry_states.json'


def _sha256_file(p):
    try:
        return hashlib.sha256(pl.Path(p).read_bytes()).hexdigest()
    except Exception:
        return 'unreadable'


def _git_describe():
    try:
        r = subprocess.run(['git', 'describe', '--always', '--dirty', '--tags'],
                           cwd=str(REPO), capture_output=True, text=True, timeout=5)
        return (r.stdout or '').strip() or 'unknown'
    except Exception:
        return 'unknown'


_PROV_CACHE = {}


def ladder_provenance(ladder=LADDER_DEFAULT, shaping=None, far_release=False,
                      tip_guard=TIP_GUARD_DEFAULT, scope='full'):
    """D6: the stamp that says WHICH ladder and WHICH code produced a number.

    `tip_guard` (amendment (aa)) changes WHERE EVERY EPISODE ENDS and therefore what any of
    it pays, so it is part of the stamp. The parameter defaults to the class default 'grip'
    so a stamp printed for a configuration nobody named still describes the rule of record --
    but every trainer, launcher, relabel and annotator passes it EXPLICITLY, and their own
    flags have no default.

    `ladder` names the reward/terminal structure; `shaping` is the goalward-shaping
    configuration of the env being stamped (None when off); `far_release` is the Ladder-N
    switch (the release that counts for farside/home must be >= 0.10 m from the goal), which
    changes what the SAME ladder pays and therefore belongs in the stamp. Every trainer writes this to
    <logdir>/ladder_provenance.json and every evaluator into metrics.json; a table builder
    REFUSES to merge rows whose stamps differ. Motivation: three trees diverged in both
    directions and no run stamped the code it loaded, so a gate could be set at submission
    and inert in the job (audit brief §2/§4).
    """
    stage_reward, terminal = ladder_spec(ladder)
    requires, ramp = ladder_extras(ladder)
    if tip_guard not in TIP_GUARD_CHOICES:
        raise ValueError(f'unknown tip_guard {tip_guard!r}; choose from {list(TIP_GUARD_CHOICES)}')
    if 'code' not in _PROV_CACHE:
        here = pl.Path(__file__).resolve()
        _PROV_CACHE['code'] = dict(
            sha256=dict(
                full_env=_sha256_file(here),
                genesis_can_env=_sha256_file(REPO / 'baselines' / 'genesis_can_env.py'),
                stage_predicates=_sha256_file(REPO / 'baselines' / 'stage_predicates.py'),
            ),
            git=_git_describe(),
            repo=str(REPO),
        )
    code = _PROV_CACHE['code']
    return dict(
        ladder=str(ladder),
        spec_version='unified-2026-09-10',
        stage_reward=stage_reward,
        requires=requires,
        ramp=ramp,
        terminal_stages=list(terminal) + ['tipped'],
        logged_stages=list(LOGGED_STAGES),
        scope=str(scope),
        max_return=max_return(ladder, scope),
        return_clamp_required=max_return(ladder, scope),   # r2dreamer's clamp MUST equal this (scope-aware)
        shaping=(dict(shaping) if shaping else None),
        far_release=bool(far_release),
        tip_guard=str(tip_guard),
        tip_deg=float(FullTaskEnv.TIP_DEG),
        tip_penalty=float(FullTaskEnv.TIP_PENALTY),
        tip_guard_sustain_frames=int(TIP_GUARD_SUSTAIN[tip_guard]),
        sha256=dict(code['sha256']), git=code['git'], repo=code['repo'],
    )


def ladder_stamp(ladder=LADDER_DEFAULT, shaping=None, far_release=False,
                 tip_guard=TIP_GUARD_DEFAULT):
    """One-line form of ladder_provenance() -- what the `[ladder]` log line prints and what
    a table builder compares. Two rows with the same stamp ran the same ladder AND the same
    tip guard AND the same predicate/env code."""
    p = ladder_provenance(ladder, shaping, far_release, tip_guard)
    rungs = ' '.join(f'{k}={v:g}' for k, v in p['stage_reward'].items())
    if p['ramp']:
        rungs += ' ramp:%s=%g/%gm' % (p['ramp']['key'], p['ramp']['scale'], p['ramp']['span'])
    sh = 'off' if not p['shaping'] else ('goalward scale=%g gamma=%g' % (
        p['shaping'].get('scale'), p['shaping'].get('gamma')))
    tip = (f"tilt>{p['tip_deg']:g}deg&{p['tip_guard']}"
           f"@{p['tip_guard_sustain_frames']}f")
    return (f"{p['spec_version']} | ladder={p['ladder']} | {rungs} | max_return={p['max_return']:g} | "
            f"terminal={'+'.join(p['terminal_stages'])} | shaping={sh} | "
            f"far_release={'on' if p['far_release'] else 'off'} | "
            f"tip={tip} | "
            f"full_env={p['sha256']['full_env'][:12]} genesis_can_env={p['sha256']['genesis_can_env'][:12]} "
            f"stage_predicates={p['sha256']['stage_predicates'][:12]} | git={p['git']}")


class LadderAccountant:
    """THE reward loop. One definition, two call sites.

    `FullTaskEnv._step_once` feeds it the live `info` dict once per ENV FRAME; the offline
    relabel (`relabel_reward.py --from-records`) feeds it the same dict reconstructed from a
    recorded stage record. Those two must pay the same reward for the same trajectory, and
    the only way to guarantee that is for there to be one implementation -- this project's
    recurring bug family (the grip column three times, the control mode three times, the (x)
    gate inert in one of three trees) is re-implemented predicate/reward math every time.

    What it does, in the order `_step_once` has always done it:
      1. pay each unpaid rung whose flag is set in `info` and whose `requires` rung has been
         REACHED, then record it as paid;
      2. pay the ramp increment, if the ladder has one;
      3. record every LOGGED_STAGES flag in `granted` (paid or not);
      4. report whether any terminal stage fired.

    `granted` ("reached") and `paid` are deliberately separate sets. `granted` is what every
    evaluator, annotator and the r2dreamer adapter read as the sticky episode record, and it
    also collects rungs that were reached before their requirement was -- which must NOT
    silently forfeit the payment when the requirement arrives later. `paid` alone gates money.
    On the 'staged'/'sparse' ladders the two sets move together frame for frame, so those
    ladders' rewards are unchanged by the split (test_ladder_unified covers this)."""

    # Which rungs a given configuration is allowed to pay -- verbatim from the reward loop
    # this class replaces: scope='full' pays the whole ladder; scope='pick' pays only its own
    # `picked` terminal; every other scope pays only its own terminal (handled outside this
    # class) and the ladder pays nothing; the pick_hold_reward lever replaces the one-shot
    # pick grant with a per-step hold reward, so it pays no rung at all.
    @staticmethod
    def pay_stages_for(scope, pick_hold_reward=False):
        if pick_hold_reward:
            return frozenset()
        if scope == 'full':
            return None                    # None = every rung in the ladder
        if scope == 'pick':
            return frozenset({'picked'})
        return frozenset()

    def __init__(self, ladder=LADDER_DEFAULT, scope='full', pay_stages=None):
        self.ladder = str(ladder)
        self.stage_reward, self.terminal_stages = ladder_spec(self.ladder)
        self.requires, self.ramp = ladder_extras(self.ladder)
        self.scope = str(scope)
        # None = every rung pays; a frozenset = only those rungs; empty = nothing pays, but
        # grants are still tracked so the logged record is identical.
        self.pay_stages = pay_stages
        self.granted = set()
        self.paid = set()
        self.ramp_paid = 0.0

    def reset(self, granted=()):
        """New episode. `granted` pre-grants stages (the place/contact scopes restore a
        post-pick state, so `picked` is already true and must not be paid again)."""
        self.granted = set(granted)
        self.paid = set(granted)
        self.ramp_paid = 0.0
        return self.granted

    def frame(self, info):
        """-> (reward, terminated). Call once per ENV FRAME, after the predicates are in
        `info`. Mutates `granted` / `paid` / `ramp_paid`."""
        reward = 0.0
        # `requires` is checked against what has been reached INCLUDING this frame, not only
        # what was reached before it. Two rungs can first become true on the same frame (the
        # tracker's flags are sticky and are all computed from one state), and if the upper one
        # is also the ladder's TERMINAL, deferring it to the next frame would end the episode
        # with the rung unpaid -- the shape of defect 5, re-created one level down.
        reached = self.granted | {s for s in LOGGED_STAGES if info.get(s)}
        for stage, r in self.stage_reward.items():
            if not info.get(stage) or stage in self.paid:
                continue
            req = self.requires.get(stage)
            if req is not None and req not in reached:
                # reached out of order (e.g. `placed_v2` true at reset with no pick,
                # CONFOUNDS row 82). Not paid, and NOT marked paid: if the requirement is
                # met later while the flag is still set, the rung pays then.
                continue
            if self.pay_stages is None or stage in self.pay_stages:
                reward += r
            self.granted.add(stage)
            self.paid.add(stage)
        if self.ramp is not None:
            req = self.ramp.get('requires')
            if req is None or req in reached:
                v = float(info.get(self.ramp['key']) or 0.0)
                want = float(self.ramp['scale']) * min(1.0, v / float(self.ramp['span']))
                if want > self.ramp_paid:
                    if self.pay_stages is None:
                        reward += want - self.ramp_paid
                    self.ramp_paid = want
        for _s in LOGGED_STAGES:
            if info.get(_s):
                self.granted.add(_s)
        terminated = any(bool(info.get(s)) for s in self.terminal_stages)
        return reward, terminated


def refuse_legacy_gates():
    """D1: this tree has no reward gates. A launcher or trainer that still exports the old
    one is running against an assumption that is now false, so stop rather than run.

    `FULLENV_TIP_GUARD` is listed for the same reason even though it never existed: the
    launchers export TIP_GUARD (a shell variable they read themselves and pass as a FLAG),
    and someone reading that could reasonably guess the env also reads a variable of its
    own. It does not, and a tree that silently ignored one would be the (x) defect again."""
    for _v in ('FULLENV_REWARD_X', 'FULLENV_TIP_GUARD'):
        if os.environ.get(_v, ''):
            raise SystemExit(f'FATAL: legacy gate set ({_v}={os.environ[_v]!r}); '
                             'this tree has no gates -- the tip guard is a constructor '
                             'argument (PHASE_PLAN (aa)), never an environment variable')

# --- reward-density lever (2026-08-14): ONE definition of the honest pick condition -
# The hold reward is paid per step by the ENV and per frame by the OFFLINE relabeler
# (train_sacfd_full.hold_region_*). Those two must agree exactly or the demo buffer
# teaches a reward the env never pays. This repo's recurring bug family (grip column
# x3, control mode x3) lived precisely in re-implemented predicate math, so both sides
# CALL THIS FUNCTION; pick_z comes from the env instance (genv.w['pick_z']) and the
# closure threshold is pick_env.GRIP_CLOSED_FRAC -- neither is ever copied.
#
# Note this is the RELABELER's predicate (can above pick_z AND gripper COMMANDED
# closed), NOT genesis_can_env's hardened `picked` (which additionally requires
# |eef-can| < PICK_EEF_DIST sustained PICK_SUSTAIN=10 frames). The hardened guard
# exists to stop a policy from WHACKING the can airborne and collecting a one-shot
# grant; here the K-consecutive-frame requirement (default 25 > PICK_SUSTAIN) supplies
# that same anti-gaming sustain, and using the relabeler's predicate is what lets the
# demo tapes -- which carry only the recorded 17-dim state, no eef position -- label
# the identical condition offline.
def pick_hold_held(can_z, grip_cmd, pick_z):
    """HONEST per-step pick condition: can above pick_z AND gripper commanded closed.

    Scalars or numpy arrays (elementwise); returns np.bool_ / bool array."""
    return ((np.asarray(can_z) > float(pick_z))
            & (np.asarray(grip_cmd) > GRIP_CLOSED_FRAC))


# --- ONE definition of the pick-scope shaping potential (PREREG 2026-08-23 §2) --------
# phi(s) = -SCALE * ||eef - can||. The ENV applies it once per decision in step(); the
# DEMO encoder (train_sacfd_full.native_demo_transitions, --demo-shaping) applies the
# same function to the recorded eef_pos / can xyz of contract-v1 tapes. Both sides CALL
# THIS FUNCTION -- the two shaped reward streams must be one definition, not two
# implementations (the grip-column / control-mode bug family lived in duplicated math).
# phi(terminal) = 0 by Ng/Harada/Russell's episodic form (the absorbing state carries no
# potential); callers pass terminal=True for a terminated transition's s' (see step()).
def pick_shaping_phi(eef_xyz, can_xyz, scale=None, terminal=False):
    """-SCALE * ||eef - can|| (training-only approach potential); 0.0 at a terminal."""
    if terminal:
        return 0.0
    scale = FullTaskEnv.PICK_SHAPING_SCALE if scale is None else float(scale)
    ee = np.asarray(eef_xyz, dtype=np.float64)[:3]
    bp = np.asarray(can_xyz, dtype=np.float64)[:3]
    return -scale * float(np.linalg.norm(ee - bp))


# --- ONE definition of the env terminal, readable OFFLINE from a demo tape --------------
# (PREREG_final_round_robin_2026-08-23 §4.3). The env terminates on (a) the hardened
# pick (scope pick), (b) the tip rule: can tilt > TIP_DEG while the grip is COMMANDED
# open (< GRIP_OPEN), evaluated AFTER the step; (c) nested (scope full); it truncates at
# max_steps. Demo encoders used to emit fail tapes whole with done=False to the last
# frame, so the critic bootstrapped through states the env would never continue from
# (ROUND_ROBIN_RESULTS_2026-08-22 "Why dDP_RLPD < dH_RLPD"; AUDIT_impl F1). Every
# consumer now asks THIS function where the tape ends.
#
# Two tape layouts:
#   * contract-v1 (record_demos.py): one ROW per decision; `terminated[t]` says row t's
#     transition terminated; `picked/tipped[t]` say why. Read verbatim.
#   * legacy stride-1 (states (T,17) / actions (T,7), transitions i = 0..T-2 taking
#     states[i] -> states[i+1] under actions[i]): env-consistent tip index is the FIRST
#     transition i with actions[i,6] < GRIP_OPEN and tilt(states[i+1]) > TIP_DEG (the env
#     checks the command just applied and the pose after the step). The pick index is
#     relabel_full's j_pick (its proxy predicate at the window-start state): the
#     hardened env pick (|eef-can| sustained PICK_SUSTAIN frames) is NOT computable
#     without eef, so offline pick placement is approximate for legacy tapes -- one
#     warning per process. A legacy tape that carries a per-FRAME `terminated` array
#     (make_dDPsucc --mode tiptrunc) flags the frame the env rule fired on; the
#     terminal TRANSITION is the one that reached it (index - 1, floored at 0).
_TERMINAL_LEGACY_WARNED = [False]
PICK_SUSTAIN_NOTE = 'sustained PICK_SUSTAIN=10 frames'


def tape_tilt_deg(quats):
    """Per-frame can tilt (deg) for an (n,4) wxyz quat array -- calls the env's own
    replay_harness.tilt_deg per row (reuse, not re-implementation)."""
    q = np.asarray(quats, dtype=np.float64)
    return np.asarray([tilt_deg(row) for row in q], dtype=np.float64)


def terminal_from_tape(tape, pick_z=None, scope='pick', j_pick=None,
                       tip_deg=None, grip_open=None, tip_guard=TIP_GUARD_DEFAULT):
    """-> dict(t_term, kind, reward, layout). t_term = index of the terminal
    TRANSITION (row for contract-v1, transition index for legacy) or None when the
    tape ends by truncation/cap; kind in {'pick','tip','nested','other','none'}.

    tape: np.load(...) NpzFile or dict. pick_z: required for legacy pick detection
    (scope='pick') unless j_pick (relabel_full's pick transition) is given. scope
    'pick' terminates on the pick; 'full' only on tip (and nested, which relabel_full
    itself marks). tip_deg/grip_open default to FullTaskEnv's constants.
    """
    # amendment (aa): this function reads DEMO TAPES, and neither tape layout carries the
    # tool point, so `not_in_hand` is not computable here -- a legacy stride-1 tape has
    # states/actions only, and a contract-v1 tape's `tipped` column is whatever guard the
    # RECORDER ran (stated, since that provenance is not otherwise visible). Refuse rather
    # than silently apply the old guard to a caller who asked for the new one. Nothing in the
    # end-to-end path reaches here: `--demo-format segment` reads `is_terminal` off the tape
    # (full_demos.segment_transitions_full); this is the pick/SACfD/legacy route.
    if tip_guard != TIP_GUARD_DEFAULT:
        raise NotImplementedError(
            f"terminal_from_tape(tip_guard={tip_guard!r}): the demo tape layouts carry no "
            f"tool_xy, so the tracker's in_hand cannot be recomputed offline from them. Score "
            f"a stage record instead (relabel_reward.py --from-records --tip-guard).")
    tip_deg = FullTaskEnv.TIP_DEG if tip_deg is None else float(tip_deg)
    grip_open = FullTaskEnv.GRIP_OPEN if grip_open is None else float(grip_open)
    keys = set(tape.files) if hasattr(tape, 'files') else set(tape.keys())
    is_v1 = ('actions_delta' in keys) and ('terminated' in keys)
    if is_v1:
        term = np.asarray(tape['terminated'], dtype=bool)
        if not term.any():
            return dict(t_term=None, kind='none', reward=0.0, layout='v1')
        t = int(np.argmax(term))
        picked = np.asarray(tape['picked'], dtype=bool) if 'picked' in keys else None
        tipped = np.asarray(tape['tipped'], dtype=bool) if 'tipped' in keys else None
        if picked is not None and picked[t]:
            kind = 'pick'
        elif tipped is not None and tipped[t]:
            kind = 'tip'
        else:
            kind = 'other'
        rew = float(np.asarray(tape['rewards'])[t]) if 'rewards' in keys else (
            1.0 if kind == 'pick' else 0.0)
        return dict(t_term=t, kind=kind, reward=rew, layout='v1')
    # ---- legacy stride-1 frame tape ----
    s = np.asarray(tape['states'], dtype=np.float64)
    a = np.asarray(tape['actions'], dtype=np.float64)
    n = len(s) - 1                       # transitions
    if n < 1:
        return dict(t_term=None, kind='none', reward=0.0, layout='legacy')
    cands = []
    if 'terminated' in keys:             # per-FRAME flag (tiptrunc): transition that reached it
        tf = np.asarray(tape['terminated'], dtype=bool)
        if tf.any():
            kind = 'tip'
            if 'terminal_kind' in keys:
                kind = str(tape['terminal_kind'].item() if hasattr(tape['terminal_kind'], 'item')
                           else tape['terminal_kind'])
            cands.append((max(int(np.argmax(tf)) - 1, 0), kind))
    tilt_next = tape_tilt_deg(s[1:, 11:15])          # pose AFTER each transition
    tip = (a[:n, 6] < grip_open) & (tilt_next > tip_deg)
    if tip.any():
        cands.append((int(np.argmax(tip)), 'tip'))
    if scope == 'pick':
        if j_pick is None:
            if pick_z is None:
                raise ValueError('terminal_from_tape(scope=pick) on a legacy tape needs '
                                 'pick_z or j_pick (relabel_full\'s pick transition)')
            if not _TERMINAL_LEGACY_WARNED[0]:
                print('[terminal_from_tape] WARNING: legacy tape -- pick placement uses '
                      'the relabeler proxy (can above pick_z & grip closed at the '
                      'window-start state); the hardened env pick (eef distance, '
                      f'{PICK_SUSTAIN_NOTE}) is not computable offline. Exact only for '
                      'contract-v1 tapes.', flush=True)
                _TERMINAL_LEGACY_WARNED[0] = True
            pf = np.asarray(pick_hold_held(s[:n, 10], a[:n, 6], pick_z), dtype=bool)
            j_pick = int(np.argmax(pf)) if pf.any() else -1
        if j_pick is not None and j_pick > 0:
            cands.append((int(j_pick), 'pick'))
    if not cands:
        return dict(t_term=None, kind='none', reward=0.0, layout='legacy')
    t, kind = min(cands, key=lambda c: c[0])
    rew = 0.0
    if kind == 'pick':
        rew = STAGE_REWARD['picked']
    elif kind == 'tip':
        rew = FullTaskEnv.TIP_PENALTY if scope != 'place' else FullTaskEnv.PLACE_TIP_PENALTY
    return dict(t_term=int(t), kind=kind, reward=float(rew), layout='legacy')



# Amendment (w) episode record -- ALWAYS ON (brief D8; the FULLENV_EPISODE_RECORD gate is
# removed). It is logging only: scalars written into `info` on the single exit path both
# termination and truncation reach. It was gated when e2e jobs were queued against this
# file; there is nothing in flight against THIS tree, and the gate is what made `placed_v2`
# and the slide predicate invisible on truncated episodes.
#
# NOTE (2026-09-10): this block, and the contact_grant selector below, were both present at
# 7096fe6 and were silently REVERTED by 809601d, which copied full_env.py in wholesale from
# another tree. train_rlpd.py has passed `contact_grant=` unconditionally since (h), so on
# this tree every train_rlpd run raised TypeError. Restored here, unchanged apart from the
# gate removal. (Lane 3 owns the general tree reconciliation.)


class FullTaskEnv(gym.Env):
    TIP_DEG = 60.0
    TIP_PENALTY = 0.0        # pick/full scopes: tip terminates but carries no penalty
    # --- scope='reach' (2026-08-28, dv3 learnability probe): +1 and TERMINATE when the tool
    # frame comes within REACH_DIST of the can centre. A HELD can sits ~0.146 m from the tool
    # frame (genesis_can_env PICK_EEF_DIST), so 0.17 m = "the gripper is at the can" -- a
    # strict subset of the motion a pick needs, no grasp required. Same obs/action/repeat/tip
    # rule as pick; the only change is the terminal predicate. Used to test whether a learner
    # that cannot pick can at least reach (user, 08-28: "make the task even simpler").
    REACH_DIST = 0.17
    # --- scope='touchgoal' (2026-08-28, user): +1 and TERMINATE on the first physical contact
    # between any robot link and the GOAL can (the static can across the workspace). Needs a real
    # reach across the table -- the arm starts adjacent to the pick can, not the goal -- but no
    # grasp. Contact read from the solver (goal.get_contacts(kinova)), not a distance threshold.
    GRIP_OPEN = 0.3          # grip command below this = not holding
    # --- scope='place' (PLACED_V2, release-based -- PAPER_PLAN stage-wise matrix) ---
    PLACE_TIP_PENALTY = -0.25  # v8: off-shelf drop; on-shelf near-miss stays -0.1  # scope='place' ONLY: penalty on the tip termination
    PLACE_RELEASE = 0.45     # grip command below this counts as released
    PLACE_TILT_DEG = 20.0    # can must be near-upright
    PLACE_SUSTAIN = 10       # predicate must hold this many CONSECUTIVE frames
    PLACE_HELD_Z = 0.13      # entry-restore verification: can center above this = still held
    PLACE_SETTLE = 20        # physics steps holding the entry pose before verifying
    PLACE_MAX_TRIES = 30     # resamples per reset before giving up
    # --- scope='place' TRAINING-ONLY shaping (constructor shaping=True; run-3 lever) ---
    # Potential-based term r += GAMMA*phi(s') - phi(s), phi = -SCALE * xy-dist(can,
    # shelf target), plus a small per-step cost that breaks the hold-forever
    # equilibrium (run 2: 600-step timeouts, zero placed_v2 ever experienced).
    # Target = center of the SAME shelf footprint rectangle the placed_v2 predicate
    # checks (in_shelf_footprint: BOX_POS +- BOX_SIZE/2) -> (0.75, -0.1875).
    # GAMMA deliberately matches the AGENT's discount (r2dreamer horizon 1000 ->
    # 1 - 1/1000 = 0.999), NOT the spec'd generic 0.99: a stationary agent leaks
    # +(1-GAMMA)*SCALE*d per step, and at 0.99 that is +0.02d -- net-POSITIVE of
    # the step cost at 65/66 banked entry states (d 0.24-0.48 m, median 0.34 ->
    # +0.0018/step -> +1.1 over a 600-step timeout, MORE than the +1 terminal:
    # the shaping would re-create the hold-forever it exists to break; measured
    # in test_place_shaping_unit.py). At 0.999 the leak is 0.002d <= 0.001 <
    # step cost everywhere reachable, holding is net-negative, and matched-gamma
    # potential shaping is exactly policy-invariant (Ng et al. 1999).
    # SHAPING IS TRAINING-ONLY: eval paths construct FullTaskEnv without shaping=,
    # and the honest reported metric remains the sparse placed_v2 terminal.
    PLACE_SHAPING_GAMMA = 0.999
    PLACE_SHAPING_SCALE = 2.0
    PLACE_STEP_COST = 0.0     # v8: NO step cost -- it made quick-tip beat holding (v4/v5 collapse); hold~0 > tip ensures no termination farming
    PLACE_SHAPING_TARGET = (BOX_POS[0], BOX_POS[1])
    # --- scope='pick' TRAINING-ONLY shaping (constructor pick_shaping=True, 08-18) ---
    # Same potential-based form as place: r += GAMMA*phi(s') - phi(s), with
    # phi = -SCALE * ||eef - can||. Policy-invariant under matched gamma (Ng 1999);
    # exists to give exploration a gradient INTO the grasp basin (the ignition
    # lottery is basin-entry, four waves + n=16 cluster). GAMMA matches the RLPD
    # agent discount 0.998 (train_rlpd default) so a stationary agent's leak
    # +(1-GAMMA)*SCALE*d = 0.002*SCALE*d/step is <= the sparse pick +1 over any
    # 400-step episode only if SCALE*d < 1.25 -- at SCALE 2 and d <= 0.5m that
    # is 1.0 <= 1.25: hover cannot out-earn completion (audit C1 hover math).
    # No step cost (v8 lesson: step cost made quick-tip beat holding). Terminate-
    # on-pick stays; the honest metric is the sparse picked terminal; eval envs
    # never set pick_shaping.
    PICK_SHAPING_GAMMA = 0.998
    PICK_SHAPING_SCALE = 2.0
    metadata = {'render_modes': []}

    # --- scope='full' OPTIONAL potential-based shaping (brief D7; constructor arg, NEVER an
    # env var, default OFF, and part of the provenance stamp). phi = -GOALWARD_SCALE *
    # xy-dist(can, goal), active ONLY while placed_v2 has been granted AND the can is not in
    # hand AND the can touches the goal; 0 everywhere else. Applied once per DECISION in
    # step() -- the boundary the agent's discount ticks at (the 08-19 timescale bug).
    # It exists for the registered sparsity fallback: if by 50% of budget no seed of an arm
    # has any contact_push in training rollouts, that arm is rerun with shaping on, disclosed.
    GOALWARD_SCALE = 2.0
    GOALWARD_GAMMA = 0.999   # override with goalward_gamma= to match the consuming agent

    def __init__(self, backend='cpu', max_steps=None, fixed_uid=None, render_size=None,
                 camera_rig=False, workspace_limit=False, scope='full', shaping=False,
                 entry_bank=None, phase_sparse=False, contact_grant=None, action_mode='absolute',
                 delta_cap=0.025,
                 delta_leash_mult=5.0, action_repeat=1, delta_ref='target',
                 pick_hold_reward=False, pick_hold_k=25, pick_shaping=False,
                 pick_shaping_gamma=None, pick_shaping_terminal_zero=True,
                 ladder=LADDER_DEFAULT, far_release=False, tip_guard=TIP_GUARD_DEFAULT,
                 goalward_shaping=False, goalward_gamma=None, goalward_scale=None,
                 quiet_ladder=False):
        super().__init__()
        # --- which ladder (user, 2026-09-11). A CONSTRUCTOR ARGUMENT, never an env var: the
        # reward structure is the one thing that must never be selectable by something a job
        # can silently fail to receive. 'staged' is the D2 ladder; 'sparse' pays only the
        # outcome (nested_v2 = 1) and terminates on it; 'nested_sparse'/'nested_ramp' are
        # Ladder N. They differ in reward and terminal ONLY -- every logged stage, diagnostic
        # and observation is identical.
        self.ladder = str(ladder)
        self.stage_reward, self.terminal_stages = ladder_spec(self.ladder)
        assert self.ladder == LADDER_DEFAULT or scope == 'full', (
            f'ladder={self.ladder!r} is a scope=full lever (the phase scopes pay exactly '
            f'their own terminal); got scope={scope!r}')
        # Ladder N's `far_release` switch: the release that counts for farside/home must have
        # happened at least stage_predicates.FAR_RELEASE_DIST_M from the goal, so a
        # drop-and-nudge at the goal cannot pay for a slide. A constructor argument for the
        # same reason the ladder is, and part of the provenance stamp.
        self.far_release = bool(far_release)
        assert not (self.far_release and self.ladder not in NESTED_LADDERS), (
            f'far_release is a Ladder-N switch (it gates farside/home); ladder={self.ladder!r} '
            f'has no such rung, so setting it would be a silent no-op')
        # --- the tip guard (amendment (aa)). A CONSTRUCTOR ARGUMENT, never an env var, for
        # the same reason the ladder is: it decides where every episode ENDS, and the two
        # learners already once optimised different objectives because a gate could be absent
        # and nothing said so. The class default stays 'grip' -- the rule the ladder pilot and
        # every prior run ran -- so no existing caller moves; the launchers and trainers
        # REQUIRE their own flag and pass it explicitly.
        assert tip_guard in TIP_GUARD_CHOICES, (
            f'tip_guard must be one of {list(TIP_GUARD_CHOICES)}, got {tip_guard!r}')
        self.tip_guard = str(tip_guard)
        self.tip_guard_sustain = int(TIP_GUARD_SUSTAIN[self.tip_guard])
        # 'not_in_hand' reads the StageTracker's per-frame `in_hand`, and the tracker only
        # runs in scope='full' (the phase scopes score their own single terminal and are
        # deliberately left on the rule of record). Refuse rather than silently fall back:
        # a phase env asked for the new guard and given the old one is exactly the class of
        # defect this whole amendment exists to remove.
        assert self.tip_guard == TIP_GUARD_DEFAULT or scope == 'full', (
            f"tip_guard={self.tip_guard!r} needs the StageTracker's in_hand, which only "
            f"scope='full' runs; got scope={scope!r}. The phase scopes stay on 'grip'.")
        # consecutive ENV FRAMES the (tilt AND guard) conjunction has held; reset every reset
        self._tip_free_run = 0
        # FAST pre-check (PHASE_PLAN (p)): reject a contact-scope misconfiguration BEFORE the ~60 s world build; the
        # authoritative validation with the full explanation runs below, after the scope fields are set.
        if scope == 'contact':
            assert contact_grant in ('bare_contact', 'slide_success', 'prior_release'), (
                f'scope=contact needs an explicit contact_grant (got {contact_grant!r}) -- PHASE_PLAN (p): (l)\'s grip '
                'clause is withdrawn, (o) was stopped, and the corrected predicate is uncalibrated, so no default exists.')
            assert not (contact_grant == 'slide_success' and not os.environ.get('CONTACT_GRANT_ALLOW_WITHDRAWN')), (
                'contact_grant="slide_success" is the WITHDRAWN (o)/(l) reward (grip<0.3, passes 2 of 74 demos); '
                'set CONTACT_GRANT_ALLOW_WITHDRAWN=1 only to reproduce the recorded (o) diagnostic.')
            if contact_grant == 'prior_release':
                raise NotImplementedError("contact_grant='prior_release' is PHASE_PLAN (p); its clause-5 threshold "
                                          "(can supported by the shelf, not clamped) is not calibrated yet.")
        # goalward shaping (D7): a CONSTRUCTOR argument on purpose. An env var would be a
        # fourth silent lever of exactly the kind that produced the two-ladder confound.
        assert not (goalward_shaping and scope != 'full'), 'goalward shaping is a scope=full lever'
        self.goalward_shaping = bool(goalward_shaping)
        self.goalward_scale = float(goalward_scale) if goalward_scale is not None else self.GOALWARD_SCALE
        self.goalward_gamma = float(goalward_gamma) if goalward_gamma is not None else self.GOALWARD_GAMMA
        self._goalward_phi_prev = 0.0
        # pick_hold_reward (2026-08-14, REWARD-DENSITY lever): see class docstring.
        # Default False keeps every existing caller byte-identical (single +1 via the
        # STAGE_REWARD 'picked' grant, terminate on the env's hardened picked flag).
        # Passed EXPLICITLY by train_rlpd and recorded in the checkpoint sidecar.
        assert not (pick_hold_reward and scope != 'pick'), (
            'pick_hold_reward is a scope=pick lever (the hold region is the pick '
            f'itself); got scope={scope}')
        assert int(pick_hold_k) >= 1, pick_hold_k
        self.pick_hold_reward = bool(pick_hold_reward)
        self.pick_hold_k = int(pick_hold_k)
        self._hold_run = 0
        # action_repeat N (2026-08-13): ONE policy decision is held for N consecutive
        # env (sim) steps -- the SAME normalized action feeds _step_once N times, so in
        # delta_joint mode the arm target advances up to N*a*delta_cap total and the
        # grip command is constant over the window; rewards accumulate; the window
        # breaks early on terminate/truncate. max_steps stays a SIM-step budget, so a
        # 900-sim-step episode is ceil(900/N) decisions -- shrinking the decision
        # horizon inside the gamma credit window (the r2dreamer repeat-4 lever). N=1 is
        # the exact stride-1 behaviour (loop runs once), so no existing caller changes.
        # MUST match the demo encoding (train_sacfd_full.delta_encode_transitions_repeat)
        # AND the eval-time repeat (wandb_eval reads action_repeat from the sidecar).
        assert int(action_repeat) >= 1, action_repeat
        self.action_repeat = int(action_repeat)
        # action_mode 'delta_joint' (2026-08-11, user: "do 1" -- port the delta
        # action space to SACfD): arm dims in [-1,1] are per-STEP joint-target
        # deltas of a*delta_cap rad integrated onto a persistent target (init =
        # measured qpos at every reset; clipped to ARM_LO/HI; leashed to measured
        # qpos within delta_leash_mult*cap). Grip dim stays absolute ([-1,1] ->
        # 0..1). Same geometry that fixed the r2dreamer arm: absolute joint
        # targets turn an exploring policy's sampled actions into arm thrash,
        # which the hardened pick predicate (sustained hold) can never reward.
        # Cap 0.025 = the demos' p99 per-frame commanded delta (44 deg/s
        # saturated); leash 5*0.025 = 0.125 covers the demos' |cmd-q| PD lead
        # (p99 0.126). MUST match the demo-buffer delta encoding
        # (train_sacfd_full --action-mode delta_joint uses the same cap).
        assert action_mode in ('absolute', 'delta_joint'), action_mode
        self.action_mode = action_mode
        self.delta_cap = float(delta_cap)
        self.delta_leash = float(delta_leash_mult) * float(delta_cap)
        # delta_ref (2026-08-14, user-directed after P1): what a delta is APPLIED TO.
        # 'target'   = existing behavior: sp = running_target + a*cap. Open-loop
        #              integration -- a clipped frame leaves a PERMANENT offset the
        #              replay never heals (P1 frozen drift; kills downstream phases).
        # 'measured' = sp = measured_qpos + a*cap (ManiSkill pd_delta style). Each
        #              action re-references the actual arm, so errors cannot
        #              accumulate; the demos' recorded qpos supplies the reference
        #              for offline encoding (delta_encode_transitions_measured*).
        # Default 'target' so every existing call site/protocol is unchanged;
        # callers opt in EXPLICITLY (silent-default rule). Note the trade-off: in
        # 'measured' mode a stalled arm keeps being pushed +a*cap relative to where
        # it IS (sustained contact push), vs 'target' which caps total intent.
        assert delta_ref in ('target', 'measured'), delta_ref
        self.delta_ref = delta_ref
        # TRAINING-ONLY dense shaping (scope='place' only; see PLACE_SHAPING_*
        # constants). Default False so eval and every other caller are unchanged.
        assert not (shaping and scope != 'place'), 'shaping is a scope=place lever'
        self.shaping = bool(shaping)
        assert not (pick_shaping and scope not in ('pick', 'reach', 'touchgoal', 'reach_goal')), 'pick_shaping is a scope=pick/reach/touchgoal/reach_goal lever'
        self.pick_shaping = bool(pick_shaping)
        # gamma MUST match the consuming agent's discount for exact Ng-invariance
        # (RLPD 0.998 default; r2dreamer passes 0.999, dv3 0.997).
        self._pick_gamma = float(pick_shaping_gamma) if pick_shaping_gamma else self.PICK_SHAPING_GAMMA
        # phi(terminal) = 0 (PREREG 2026-08-23 §2; AUDIT_sources §6): Ng et al.'s
        # episodic form gives the absorbing state zero potential, so a terminated
        # decision pays F = gamma*0 - phi(s) = -phi(s). The 08-19 dense round left
        # phi(s_T) = -2*d_T (a bias against terminating, small at the pick, up to -1 at a
        # tip); pass pick_shaping_terminal_zero=False ONLY to reproduce those runs.
        self.pick_shaping_terminal_zero = bool(pick_shaping_terminal_zero)
        self._pick_phi_prev = 0.0
        self.genv = GenesisCanEnv(backend=backend, render_size=render_size,
                                  camera_rig=camera_rig,
                                  workspace_limit=workspace_limit)
        # CRITICAL (#26 regression, found by the P1 trace 2026-08-13): the inner
        # env's default max_steps=1200 is an EVAL horizon. Once its `done` goes true,
        # genv.step runs _nested() -- 100 phantom sim steps -- on EVERY later call,
        # silently perturbing physics for any rollout past 1200 inner steps (54/72
        # sweep replays; recovered 2 lost picks, removed 2 spurious nesteds).
        # collect_all_classified.py has carried this exact fix since 07-20;
        # FullTaskEnv never got it. THIS env's own max_steps does the truncating.
        self.genv.max_steps = 10 ** 9
        # scope='pick': +1 and terminate on the pick (matches CartesianFullTaskEnv).
        # step() has referenced self.scope since the pick-scope work, but this
        # constructor never set it -- every FullTaskEnv.step crashed (found by the
        # eval-side genesis_scope restore, 2026-08-01: ALL periodic evals of the
        # joint dv3 smoke runs were failing on this).
        # scope='place': reset restores a banked post-pick entry state
        # (baselines/pick_entry_states.json, written by make_pick_phase_datasets);
        # +1 and terminate on PLACED_V2 = released (grip cmd < PLACE_RELEASE) +
        # in shelf z-band/footprint + near-upright, sustained PLACE_SUSTAIN frames.
        self.scope = scope
        # Explicit per-scope default cap (NOT silently scope-mangled when the caller
        # passes a value): full/pick 900, place 600 (entry->release is much shorter).
        if max_steps is None:
            max_steps = 600 if scope in ('place', 'contact', 'carrycontact') else 900
        self.max_steps = int(max_steps)
        self.fixed_uid = fixed_uid
        self.success_uids = sorted(
            u for u, r in self.genv.placements.items() if r.get('label') == 'success')
        assert scope in ('full', 'pick', 'place', 'contact', 'carrycontact', 'reach', 'touchgoal', 'reach_goal'), f'unknown scope {scope!r}'
        self.phase_sparse = bool(phase_sparse)   # PHASE PLAN: tips terminate only (no penalty) in place/contact
        # scope='contact' GRANT SELECTOR -- explicit, no default (PHASE_PLAN (p), 2026-09-07). History: (m) paid on bare
        # `contact`, which pays for driving a HELD can into the goal; (o) would have paid on `slide_success`, but (p)
        # WITHDREW that predicate's grip clause -- it passes 2 of 74 demonstrations and 44 failures are the grip clause
        # alone, because the human releases fully and then pushes the can home with the fingers re-closed to ~0.4.
        # (p)'s replacement (prior release + can supported-not-clamped at contact) is registered but its clause-5
        # threshold is NOT calibrated yet, so it is deliberately NOT implemented here. A contact-scope env therefore
        # REFUSES to build unless the caller names the grant it wants, and the withdrawn one needs an explicit override.
        self.contact_grant = contact_grant
        if scope == 'contact':
            _allowed = ('bare_contact', 'slide_success', 'prior_release')
            assert contact_grant in _allowed, (
                f'scope=contact needs contact_grant={_allowed} passed EXPLICITLY (got {contact_grant!r}). '
                'PHASE_PLAN (p): (l)\'s grip<0.3 clause is withdrawn and (o) was stopped before landing; the corrected '
                'prior-release predicate awaits its clause-5 calibration, so there is currently NO correct default.')
            if contact_grant == 'slide_success' and not os.environ.get('CONTACT_GRANT_ALLOW_WITHDRAWN'):
                raise AssertionError(
                    'contact_grant="slide_success" is the WITHDRAWN (o)/(l) reward (grip<0.3): it contradicts the '
                    'demonstrations (2/74) and would train the arm away from the demonstrated half-closed push. '
                    'Set CONTACT_GRANT_ALLOW_WITHDRAWN=1 only to reproduce the recorded (o) diagnostic runs.')
            if contact_grant == 'prior_release':
                raise NotImplementedError(
                    'contact_grant="prior_release" is PHASE_PLAN (p) clause 1-5; clause 5 (can supported by the shelf, '
                    'not clamped, at contact) must be calibrated from the demonstration traces before it is implemented.')
        # PHASE PLAN: shelf-referenced band follows the WORLD's shelf (sim_variants shelf_dz), not the stale constant.
        import os as _os, sim_variants as _sv
        _vn = _os.environ.get('R2D_SIM_VARIANT') or _os.environ.get('GENESIS_SIM_VARIANT') or 'base'
        _vd = getattr(_sv, 'VARIANTS', None) or getattr(_sv, '_VARIANTS', None) or {}
        _dz = float((_vd.get(_vn) or {}).get('shelf_dz', 0.0)) if isinstance(_vd, dict) else 0.0
        self.shelf_top_z = float(BOX_TOP_Z) + _dz
        if scope in ('place', 'contact', 'carrycontact'):
            assert _vn != 'base' or _dz == 0.0, 'variant lookup failed'
            print(f'[phase] variant {_vn}: shelf_top_z {self.shelf_top_z:.3f} (band {self.shelf_top_z+0.01:.3f}..{self.shelf_top_z+0.07:.3f})', flush=True)
        # amendment (j) 2026-09-07 (ADVERSARIAL_REVIEW_eval_env S3-7): the band above is derived from an ENV VAR while the
        # world is built by sim_variant_hook.apply_pre() -- nothing tied them together. Assert against the BUILT shelf box
        # (sim_variants.install() moves it by shelf_dz at build time) on EVERY path that constructs this env.
        _built_top = self._world_shelf_top()
        assert abs(_built_top - self.shelf_top_z) < 1e-6, (
            f'shelf band mismatch: variant env var {_vn!r} gives shelf_top_z {self.shelf_top_z:.4f} but the BUILT world\'s shelf top is '
            f'{_built_top:.4f} -- export R2D_SIM_VARIANT/GENESIS_SIM_VARIANT to the variant the world was built with')
        if scope in ('place', 'contact', 'carrycontact'):
            # entry_bank: path to the bank JSON. Default = the legacy single-
            # entry-per-demo bank (uid-keyed DICT) so existing runs are byte-
            # identical. A dense bank (make_place_entry_bank.py) is a LIST of
            # entries each carrying 'uid'; reset samples uniformly over ENTRIES.
            bank_path = pl.Path(entry_bank) if entry_bank else PLACE_ENTRY_BANK
            raw = json.loads(bank_path.read_text())
            if isinstance(raw, dict):
                self._entries = [dict(e, uid=int(u)) for u, e in raw.items()]
            else:
                self._entries = [dict(e, uid=int(e['uid'])) for e in raw]
            assert self._entries, f'empty entry bank {bank_path}'
            # legacy uid->entry view (earliest frame per uid) kept for external
            # consumers (verify_place_scope reads _entry_bank[uid]['frame'])
            self._entry_bank = {}
            for e in sorted(self._entries, key=lambda x: (x['uid'], x['frame'])):
                self._entry_bank.setdefault(e['uid'], e)
            self.success_uids = sorted(self._entry_bank)
            # survival accounting (printed once after the first reset; entries whose
            # held-can state does not survive the restore+settle are resampled)
            self.place_attempts = 0
            self.place_survived = 0
            self._survival_reported = False
        self.pick_z = float(self.genv.w['pick_z'])
        self.observation_space = spaces.Box(-np.inf, np.inf, (STATE_DIM,), np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, (ACT_DIM,), np.float32)
        self._t = 0
        # THE reward loop, shared with the offline relabel (see LadderAccountant). `_granted`
        # IS the accountant's own set object, so every existing reader of `env._granted`
        # (evaluators, annotators, the r2dreamer adapter) is unchanged.
        self._acct = LadderAccountant(
            self.ladder, scope=self.scope,
            pay_stages=LadderAccountant.pay_stages_for(self.scope, self.pick_hold_reward))
        self._granted = self._acct.reset()
        # RECORDING-ONLY instance knob (never a constructor default, never an env var): when
        # True, step() runs the whole action stream and reports terminated/truncated without
        # stopping. Only baselines/rl/relabel_reward.py --records-out sets it, so that one
        # re-execution can carry every ladder's terminal instead of one re-execution per
        # ladder. A training or evaluation env that set this would score a different MDP.
        self.never_terminate = False
        self._pv2_run = 0
        self._attempted = False
        self._phi = 0.0
        # --- the shared stage tracker (brief D1/D3). scope='full' only: the phase scopes
        # score their own single terminal and are deliberately left unchanged. The tracker
        # is fed once per ENV FRAME after the sim step (see _step_once) and owns
        # contact_push / nested_v2 / slide_success; full_env keeps picked / placed_v2 /
        # tipped and the reward loop.
        self.tracker = None
        if scope == 'full':
            _g = np_(self.genv.w['goal'].get_pos())
            self.tracker = StageTracker(goal_xy=(float(_g[0]), float(_g[1])),
                                        shelf_top_z=float(self.shelf_top_z),
                                        far_release=self.far_release)
        self._track = {}          # last tracker output (diagnostics -> info)
        # D6: say out loud which ladder and which code this env is running. A run that does
        # not stamp the code it loaded is how a gate can be set at submission and inert in
        # the job (audit brief §2/§4).
        if not quiet_ladder:
            print('[ladder] ' + ladder_stamp(self.ladder, self.shaping_config(), self.far_release,
                                             self.tip_guard),
                  flush=True)

    def shaping_config(self):
        """The goalward-shaping configuration, for the provenance stamp. None when off."""
        if not getattr(self, 'goalward_shaping', False):
            return None
        return dict(kind='goalward_potential', scale=self.goalward_scale, gamma=self.goalward_gamma,
                    gate='placed_v2 granted AND not in_hand AND can<->goal contact')

    def provenance(self):
        """This env's ladder provenance (D6) -- what trainers and evaluators write out."""
        p = ladder_provenance(self.ladder, self.shaping_config(), self.far_release,
                              self.tip_guard)
        p['scope'] = self.scope
        p['stamp'] = ladder_stamp(self.ladder, self.shaping_config(), self.far_release,
                                  self.tip_guard)
        if self.tracker is not None:
            p['predicate_constants'] = self.tracker.constants()
        if self.never_terminate:
            # only the offline stage recorder sets this; a cell stamped with it is not a cell
            p['never_terminate'] = True
        return p

    def _world_shelf_top(self):
        """Top z of the shelf box AS BUILT (the Box entity whose morph size is replay_harness.BOX_SIZE; its base-link
        position reflects sim_variants.install()'s shelf_dz shift). amendment (j) 2026-09-07."""
        from replay_harness import BOX_SIZE
        for ent in self.genv.w['scene'].entities:
            m = getattr(ent, 'morph', None)
            if m is not None and type(m).__name__ == 'Box' and getattr(m, 'size', None) is not None \
                    and np.allclose(np.asarray(m.size, float), np.asarray(BOX_SIZE, float), atol=1e-9):
                return float(np.asarray(np_(ent.get_pos()), dtype=np.float64).reshape(-1)[2]) + float(BOX_SIZE[2]) / 2.0
        raise RuntimeError('shelf box entity (Box morph of size BOX_SIZE) not found in the built world')

    def _sync_dj_target(self):
        """(Re-)seed the delta_joint persistent target from measured qpos.

        Must run on EVERY reset variant (reset/_reset_place/reset_to) or the
        target carries over from the previous episode and the first steps lunge
        toward a stale pose. No-op in absolute mode."""
        if self.action_mode != 'delta_joint':
            return
        q = np.asarray(self.genv._obs()['state'][:6], dtype=np.float64)
        self._dj_target = q.copy()
        self._dj_qmeas = q.copy()

    def _pick_phi(self):
        """-SCALE * ||eef - can||, the pick-scope approach potential (training-only).
        Delegates to the module-level pick_shaping_phi -- the same function the demo
        encoder applies to recorded eef_pos (one definition)."""
        ee = np.asarray(self.genv.tool_pos(), dtype=np.float64)
        bp = np_(self.genv.w['bottle'].get_pos())
        return pick_shaping_phi(ee, bp, scale=self.PICK_SHAPING_SCALE)

    def _place_phi(self, bp):
        """Shaping potential: -SCALE * xy-distance(can center, shelf target)."""
        dx = float(bp[0]) - self.PLACE_SHAPING_TARGET[0]
        dy = float(bp[1]) - self.PLACE_SHAPING_TARGET[1]
        return -self.PLACE_SHAPING_SCALE * float(np.hypot(dx, dy))

    def _restore_place_entry(self, e):
        """Restore one banked post-pick entry dict; True if the can survives the
        settle.

        Order matters: genv.reset first (clears velocities, seeds goal/can, steps
        once with the arm at HARDCODED_START), THEN overwrite arm+finger joints and
        re-set the can pose (the reset's single step lets the unsupported can drop
        ~0.5 mm), THEN hold the entry commands for PLACE_SETTLE physics steps so
        the grasp re-engages before the policy sees the state.
        """
        w = self.genv.w
        goal_pos = (e['goal_xy'][0], e['goal_xy'][1], w['goal_start_z'])
        self.genv.reset(can_pos=e['can_pos'], can_quat=e['can_quat'],
                        goal_pos=goal_pos)
        kin = w['kinova']
        q = np.array(HARDCODED_START, dtype=np.float64)
        q[:6] = e['qpos']
        # fingers at the MEASURED closure (grip_obs), not the command -- the command
        # overdrives into the can; teleporting fingers to it would intersect geometry
        q[6:] = gripper_targets(float(e['grip_obs']) * 100.0)
        kin.set_dofs_position(q, w['kdofs'])
        kin.zero_all_dofs_velocity()
        w['bottle'].set_pos(e['can_pos'])
        w['bottle'].set_quat(list(e['can_quat']))
        try:
            w['bottle'].zero_all_dofs_velocity()
        except Exception:
            pass
        # hold the entry pose: arm at qpos, gripper commanded closed at grip_cmd
        kin.control_dofs_position(np.asarray(e['qpos'], np.float64),
                                  dofs_idx_local=w['kdofs'][:6])
        kin.control_dofs_position(np.array(gripper_targets(float(e['grip_cmd']) * 100.0)),
                                  dofs_idx_local=np.array(w['kdofs'][-4:]))
        for _ in range(self.PLACE_SETTLE):
            w['scene'].step()
        bp = np_(w['bottle'].get_pos())
        return bool(bp[2] > self.PLACE_HELD_Z)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        uid = (options or {}).get('uid') or self.fixed_uid
        if self.scope in ('place', 'carrycontact'):
            return self._reset_place(uid)
        if self.scope == 'contact':
            return self._reset_contact(uid)
        if uid is None:
            uid = int(self.np_random.choice(self.success_uids))
        obs = self.genv.reset(uid=int(uid))
        self._t = 0
        self._granted = self._acct.reset()
        self._hold_run = 0
        self._pv2_run = 0   # amendment (j): placed_v2 is computed in scope=full too
        self._pick_phi_prev = self._pick_phi() if self.pick_shaping else 0.0
        self._reset_tracker()
        self._sync_dj_target()
        return obs['state'].astype(np.float32), {'uid': int(uid)}

    def _reset_tracker(self):
        """Per-episode reset of the shared stage tracker + the goalward potential.

        MUST run on every reset variant that scope='full' can take (reset / reset_to), or
        the sticky flags carry over from the previous episode -- the same failure mode
        _sync_dj_target exists for."""
        self._track = {}
        self._goalward_phi_prev = 0.0
        # amendment (aa): the tip guard's sustain run is per-EPISODE state and resets here
        # for the same reason. (It cannot change behaviour under tip_guard='grip', whose
        # sustain is 1, but a counter that survives a reset is a bug waiting for a longer
        # sustain to expose it.)
        self._tip_free_run = 0
        if self.tracker is not None:
            self.tracker.reset()
            _g = np_(self.genv.w['goal'].get_pos())
            # the tracker's static goal (its fallback when update() gets goal_pos=None; the
            # attribute is `goal_xy`, a (2,) array -- a wrong name here would be a silent no-op)
            self.tracker.goal_xy = np.asarray([float(_g[0]), float(_g[1])], dtype=np.float64)

    def _reset_place(self, uid=None):
        tried = []
        # explicit uid: resample only among that uid's entries (dense bank may
        # hold several); no cross-uid swap (fail loudly, as before)
        pool = self._entries if uid is None else \
            [e for e in self._entries if e['uid'] == int(uid)]
        assert pool, f'no bank entries for uid {uid}'
        for _ in range(self.PLACE_MAX_TRIES):
            e = pool[int(self.np_random.integers(len(pool)))]
            u = e['uid']
            self.place_attempts += 1
            ok = self._restore_place_entry(e)
            if ok:
                self.place_survived += 1
            if not self._survival_reported and self.place_attempts >= 1 and ok:
                # one-shot startup line; running counters stay queryable
                print(f'[place] entry-restore survival so far: '
                      f'{self.place_survived}/{self.place_attempts}', flush=True)
                self._survival_reported = True
            if ok:
                self._t = 0
                self._tip_free_run = 0          # amendment (aa), per-episode state
                self._pv2_run = 0; self._attempted = False
                # seed the shaping potential at the settled entry state (cheap;
                # computed unconditionally so shaping toggling never desyncs it)
                self._phi = self._place_phi(np_(self.genv.w['bottle'].get_pos()))
                # the pick already happened in the demo this state came from: the
                # env-level picked flag must be up for placed/contact predicates,
                # and 'picked' is pre-granted so the restored pick pays no reward
                self.genv._picked = True
                self._granted = self._acct.reset({'picked'})
                self._sync_dj_target()
                return (self.genv._obs()['state'].astype(np.float32),
                        {'uid': u, 'entry_frame': int(e['frame']),
                         'entry_frac': float(e.get('frac', 0.0))})
            tried.append((u, int(e['frame'])))
            print(f'[place] entry {u}@{int(e["frame"])} did not survive restore '
                  f'(can dropped), resampling', flush=True)
        raise RuntimeError(f'scope=place reset: no entry survived restore '
                           f'(tried {tried})')

    def _restore_contact_entry(self, e):
        """Restore a banked placed_v2-grant entry (can released on the shelf, gripper open); True if the can
        is still upright and inside the shelf band after the settle."""
        w = self.genv.w
        goal_pos = (e['goal_xy'][0], e['goal_xy'][1], w['goal_start_z'])
        self.genv.reset(can_pos=e['can_pos'], can_quat=e['can_quat'], goal_pos=goal_pos)
        kin = w['kinova']
        q = np.array(HARDCODED_START, dtype=np.float64)
        q[:6] = e['qpos']
        q[6:] = gripper_targets(float(e['grip_obs']) * 100.0)
        kin.set_dofs_position(q, w['kdofs'])
        kin.zero_all_dofs_velocity()
        w['bottle'].set_pos(e['can_pos'])
        w['bottle'].set_quat(list(e['can_quat']))
        try:
            w['bottle'].zero_all_dofs_velocity()
        except Exception:
            pass
        kin.control_dofs_position(np.asarray(e['qpos'], np.float64), dofs_idx_local=w['kdofs'][:6])
        kin.control_dofs_position(np.array(gripper_targets(float(e['grip_cmd']) * 100.0)),
                                  dofs_idx_local=np.array(w['kdofs'][-4:]))
        for _ in range(self.PLACE_SETTLE):
            w['scene'].step()
        bp = np_(w['bottle'].get_pos())
        return bool(in_shelf_footprint(bp[:2]) and self.shelf_top_z + 0.01 < bp[2] < self.shelf_top_z + 0.07
                    and tilt_deg(np_(w['bottle'].get_quat())) < self.PLACE_TILT_DEG)

    def _reset_contact(self, uid=None):
        tried = []
        pool = self._entries if uid is None else [e for e in self._entries if e['uid'] == int(uid)]
        assert pool, f'no bank entries for uid {uid}'
        for _ in range(self.PLACE_MAX_TRIES):
            e = pool[int(self.np_random.integers(len(pool)))]
            u = e['uid']
            self.place_attempts += 1
            ok = self._restore_contact_entry(e)
            if ok:
                self.place_survived += 1
            if not self._survival_reported and ok:
                print(f'[contact] entry-restore survival so far: {self.place_survived}/{self.place_attempts}', flush=True)
                self._survival_reported = True
            if ok:
                self._t = 0
                self._tip_free_run = 0          # amendment (aa), per-episode state
                # the pick and the release already happened in the demo this state came from
                self.genv._picked = True
                self._granted = self._acct.reset({'picked', 'placed_v2'})
                self._sync_dj_target()
                return (self.genv._obs()['state'].astype(np.float32),
                        {'uid': u, 'entry_frame': int(e['frame'])})
            tried.append((u, int(e['frame'])))
            print(f'[contact] entry {u}@{int(e["frame"])} did not survive restore, resampling', flush=True)
        raise RuntimeError(f'scope=contact reset: no entry survived restore (tried {tried})')

    def reset_to(self, ic):
        """Reset to an explicit IC dict (ic_sampling-style) -- random-IC eval."""
        obs = self.genv.reset(**ic)
        self._t = 0
        self._granted = self._acct.reset()
        self._hold_run = 0
        self._pv2_run = 0   # amendment (j)
        self._pick_phi_prev = self._pick_phi() if self.pick_shaping else 0.0
        self._reset_tracker()
        self._sync_dj_target()
        return obs['state'].astype(np.float32), {}

    def step(self, action):
        # action_repeat: hold the SAME decision for N sim steps, accumulate reward,
        # break early on terminate/truncate. N=1 -> a single _step_once (identical to
        # the pre-repeat behaviour). The delta integration in _step_once re-adds a*cap
        # each of the N calls, so total target advance = N*a*cap -- exactly what the
        # decision-level demo encoding (delta_encode_transitions_repeat) assumes.
        total_reward = 0.0
        for _ in range(self.action_repeat):
            obs, reward, terminated, truncated, info = self._step_once(action)
            total_reward += reward
            if (terminated or truncated) and not self.never_terminate:
                break
        if self.pick_shaping:
            # TRAINING-ONLY approach potential, applied ONCE per step() call --
            # the boundary the agent's discount actually ticks at. Applying it
            # inside _step_once under action_repeat>1 leaves a path-dependent
            # -(1-gamma)*sum(phi_substeps) residual: a reward for staying FAR
            # from the can, per episode the same order as the sparse terminal.
            # For action_repeat=1 this is numerically identical to the old
            # per-substep placement (same phi points, same formula). Terminal
            # steps are still shaped (runs after the break).
            # phi(terminal) = 0: a TERMINATED decision's successor is the absorbing
            # state (no potential). Truncation is not a terminal -- s' keeps its phi
            # and the critic bootstraps there.
            phi = 0.0 if (terminated and self.pick_shaping_terminal_zero) else self._pick_phi()
            total_reward += self._pick_gamma * phi - self._pick_phi_prev
            self._pick_phi_prev = phi
        if self.goalward_shaping:
            # D7 sparsity fallback, OFF unless the constructor asked for it. Potential-based
            # (Ng/Harada/Russell): r += gamma*phi(s') - phi(s), applied ONCE per DECISION --
            # the timescale the agent discounts at, the same lesson as pick_shaping above.
            # phi = -scale * xy-dist(can, goal) INSIDE the gate and 0 outside it, so the
            # term is a well-defined potential over the whole state space; phi(terminal)=0.
            # HONEST CAVEAT, not to be lost in a summary: a gated potential is exactly
            # policy-invariant only for the potential as defined here (gate included). The
            # gate makes phi discontinuous at its boundary, so crossing the gate pays a
            # one-off +/- scale*d -- entering the gate (can released, touching the goal)
            # costs -scale*d and leaving it refunds it. Bounded and sign-consistent, but it
            # is shaping, and any arm run with it is DISCLOSED (registered trigger, Lane 4).
            phi = 0.0 if terminated else self._goalward_phi()
            total_reward += self.goalward_gamma * phi - self._goalward_phi_prev
            self._goalward_phi_prev = phi
        if terminated or truncated:
            # Amendment (w) episode record -- ALWAYS ON (brief D8). ONE record, emitted from
            # the SINGLE exit both termination and truncation reach. `self._granted` is
            # sticky and cumulative, so this reports what the episode actually reached.
            #
            # Why it is needed: the per-step stage flags are written only when an episode
            # terminates INSIDE the adapter, so a horizon truncation logged all zeros even
            # for an episode that had picked (1198/2911 episodes on one run, every one at
            # exactly the horizon, 608 of them having scored). That left the accumulated
            # reward as the only truncation-proof channel -- which is why placed_v2 and the
            # slide predicate had no full-scope learning curves: neither was a rung.
            #
            # LOGGING ONLY. No simulation advanced, no state mutated, no reward term.
            # Scalars only -- no containers -- so no logger can choke on the type.
            info = dict(info)
            info['episode_end'] = True
            for _stage in LOGGED_STAGES:
                info['ep_' + _stage] = bool(_stage in self._granted)
            info['ep_tipped'] = bool(info.get('tipped'))
        return obs, total_reward, terminated, truncated, info

    def _full_scope_predicates(self, a_phys, info):
        """scope='full' ONLY: settle `placed_v2`, then feed the shared StageTracker ONE
        env frame (brief D1/D3) and write its answers into `info`.

        Called from _step_once immediately after the sim step and BEFORE the reward loop,
        because `contact_push` and `slide_success` both require `placed_v2` to have been
        granted. full_env owns picked / placed_v2 / tipped; the tracker owns everything
        that needs a short history (in_hand, at_rest, released, pushed, contact_push,
        nested_v2, slide_success). Neither re-implements the other.

        Everything here is a READ of the solver state plus arithmetic -- no sim step is
        advanced, so episode dynamics are unchanged by the measurement (the #26 trace
        ablation established that per-step state reads do not perturb Genesis)."""
        w = self.genv.w
        bp = np_(w['bottle'].get_pos())
        bq = np_(w['bottle'].get_quat())
        gp = np_(w['goal'].get_pos())
        gq = np_(w['goal'].get_quat())
        # --- placed_v2, unchanged predicate (amendment (j)): grip commanded open, can
        # inside the shelf footprint and the WORLD's shelf band, near-upright, sustained
        # PLACE_SUSTAIN consecutive frames. It is now a PAID rung (+1) instead of a
        # logged-only flag; the predicate itself is byte-identical.
        _ok = (float(a_phys[6]) < self.PLACE_RELEASE and in_shelf_footprint(bp)
               and self.shelf_top_z + 0.01 < bp[2] < self.shelf_top_z + 0.07
               and tilt_deg(bq) < self.PLACE_TILT_DEG)
        self._pv2_run = self._pv2_run + 1 if _ok else 0
        if self._pv2_run >= self.PLACE_SUSTAIN:
            info['placed_v2'] = True
        placed_granted = bool(info.get('placed_v2') or 'placed_v2' in self._granted)
        # --- the per-frame solver contacts the tracker needs. genesis_can_env computes
        # both and now exports them per frame, so they are read once, in one place.
        cg = bool(info.get('can_goal_touch'))
        gg = bool(info.get('gripper_goal_touch'))
        tool = np.asarray(self.genv.tool_pos(), dtype=np.float64)
        flags = self.tracker.update(
            can_pos=(float(bp[0]), float(bp[1]), float(bp[2])),
            can_quat=[float(v) for v in np.asarray(bq).reshape(-1)[:4]],
            goal_pos=(float(gp[0]), float(gp[1]), float(gp[2])),
            goal_quat=[float(v) for v in np.asarray(gq).reshape(-1)[:4]],
            tool_xy=(float(tool[0]), float(tool[1])),
            grip_cmd=float(a_phys[6]),
            picked=bool(info.get('picked')),
            can_goal_contact=cg,
            gripper_goal_contact=gg,
            placed_v2=placed_granted)
        flags['can_goal_contact'] = cg          # the goalward gate reads this back
        self._track = flags
        # The tracker is AUTHORITATIVE for these three in full scope. genesis_can_env also
        # computes a `contact_push` ((g): no release required) and a `slide_success` ((l):
        # grip < 0.3, a clause (p) withdrew); both are kept under *_legacy so the old
        # columns stay readable, and neither is paid or terminal.
        info['contact_push_legacy'] = bool(info.get('contact_push')
                                           or getattr(self.genv, '_contact_push', False))
        info['slide_success_legacy'] = bool(info.get('slide_success'))
        info['contact_push'] = bool(flags['contact_push'])
        info['slide_success'] = bool(flags['slide_success'])
        info['nested_v2'] = bool(flags['nested_v2'])
        info['released'] = bool(flags['released'])
        info['pushed'] = bool(flags['pushed'])
        info['in_hand'] = bool(flags['in_hand'])
        info['at_rest'] = bool(flags['at_rest'])
        info['goalward_gain_m'] = float(flags['goalward_gain_m'])
        info['lever_m'] = float(flags['lever_m'])
        # Ladder N: computed and logged under EVERY ladder (so a staged run's records can be
        # re-scored under a nested ladder without re-simulating), paid only under one.
        info['farside'] = bool(flags['farside'])
        info['slide_event'] = bool(flags['slide_event'])
        info['home'] = bool(flags['home'])
        info['release_far'] = bool(flags['release_far'])
        info['settled_after_release'] = bool(flags['settled_after_release'])
        info['slide_gain_m'] = float(flags['slide_gain_m'])
        info['gain_during_release_m'] = float(flags['gain_during_release_m'])

    def _goalward_phi(self):
        """D7 potential: -scale * xy-dist(can, goal) while the gate holds, else 0.

        Gate (brief D7): placed_v2 GRANTED and the can is NOT in hand and the can touches
        the goal. `in_hand` and the contact flag come from the tracker's last update, so
        the gate is the same predicate the ladder uses -- never a second implementation."""
        if 'placed_v2' not in self._granted:
            return 0.0
        tr = self._track or {}
        if tr.get('in_hand', True) or not tr.get('can_goal_contact', False):
            return 0.0
        bp = np_(self.genv.w['bottle'].get_pos())
        gp = np_(self.genv.w['goal'].get_pos())
        return -self.goalward_scale * float(np.hypot(float(bp[0]) - float(gp[0]),
                                                     float(bp[1]) - float(gp[1])))

    def _step_once(self, action):
        if self.action_mode == 'delta_joint':
            a = np.asarray(action, dtype=np.float64)
            if self.delta_ref == 'measured':
                # Measured mode scales by the LEASH, not the cap: the action is the
                # normalized desired PD ERROR (target offset from the actual arm).
                # The demos drive with lead up to ~0.126 rad (p99 == leash); scaling
                # by cap under-drives 5x and the arm never keeps the demo's timing
                # (smoke 0/5, can untouched). With leash scaling, replaying the
                # recorded lead reproduces the recorded drive.
                d = np.clip(a[:6], -1.0, 1.0) * self.delta_leash
                sp = np.clip(self._dj_qmeas + d, ARM_LO, ARM_HI)
            else:
                d = np.clip(a[:6], -1.0, 1.0) * self.delta_cap
                sp = np.clip(self._dj_target + d, ARM_LO, ARM_HI)
            self._dj_target = self._dj_qmeas + np.clip(
                sp - self._dj_qmeas, -self.delta_leash, self.delta_leash)
            a_phys = np.concatenate(
                [self._dj_target, [(np.clip(a[6], -1.0, 1.0) + 1.0) / 2.0]])
        else:
            a_phys = denormalize_action(action)
        obs, _env_done, info = self.genv.step(a_phys)
        if self.action_mode == 'delta_joint':
            self._dj_qmeas = np.asarray(obs['state'][:6], dtype=np.float64)
        self._t += 1
        # ---- scope='full': placed_v2 FIRST, then the shared tracker, then the reward loop.
        # Order matters and used to be wrong: placed_v2 was computed at the BOTTOM of this
        # method, after the reward loop, so the ladder could never condition on it. The
        # brief's contact_push and slide_success both REQUIRE placed_v2 to be granted, so
        # the release predicate has to be settled before the tracker is fed.
        if self.scope == 'full':
            self._full_scope_predicates(a_phys, info)
        # GenesisCanEnv only computes the honest (settled) nested at its own horizon, and
        # _nested() steps the sim so it can't run per-step. The LEGACY training proxy:
        # contact + can & goal upright + gripper commanded open. It pays NOTHING and
        # terminates NOTHING now (brief D2); it is kept only so stored rows stay comparable
        # and is reported as `nested_proxy`. nested_v2 (state-only, no settle) replaces it.
        if info.get('contact') and float(a_phys[6]) < 0.3 \
                and 'nested' not in self._granted:
            w = self.genv.w
            if tilt_deg(np_(w['bottle'].get_quat())) < 20 \
                    and tilt_deg(np_(w['goal'].get_quat())) < 20:
                info['nested'] = True
        # THE reward loop -- LadderAccountant, the SAME object the offline relabel runs
        # (relabel_reward.py --from-records). It pays each unpaid rung of THIS env's ladder
        # whose `requires` rung has been reached, pays the Ladder-N ramp increment, records
        # every LOGGED_STAGES flag in `_granted`, and reports the ladder terminal.
        #
        # Which rungs may pay is fixed at construction (pay_stages_for): scope='full' pays the
        # whole ladder; scope='pick' pays only its own `picked` terminal -- before 2026-08-28
        # it also paid any other stage that flipped in the same step (e.g. 'placed' when the
        # lifted can crossed the shelf plane), +2 on 8/56 dR2D demos; scope='place'/'contact'
        # pay only their own terminal below, and the restored pick is pre-granted so it costs
        # nothing; pick_hold_reward pays only the per-step hold reward below, because keeping
        # the one-shot 'picked' grant too would double-pay the lift.
        #
        # NOTE (Lane 3 audit, 2026-09-10): the 2026-09-09 "contact_push exposure fix" that used
        # to sit here was a NO-OP and is deleted. Its comment claimed genesis_can_env "never
        # puts it into `info`", but `genesis_can_env.step()` has written
        # `contact_push=self._contact_push` into its info dict in every live tree. The real
        # reason contact_push read 0.000 in full scope was that the OLD ladder's STAGE_REWARD
        # did not contain the key, so no grant bookkeeping ran on it -- which the accountant's
        # logged-grant pass handles for every stage, paid or not. In scope='full' the value is
        # the TRACKER's anyway (it requires a prior release); the (g) predicate is kept beside
        # it as contact_push_legacy.
        reward, ladder_terminated = self._acct.frame(info)
        if self.scope == 'touchgoal':
            c = self.genv.w['goal'].get_contacts(self.genv.w['kinova'])
            n_c = int(np.asarray(np_(c['link_a'])).reshape(-1).shape[0])
            info['goal_contacts'] = n_c
            if n_c > 0:
                info['touched_goal'] = True; reward += 1.0
                return (obs['state'].astype(np.float32), reward, True, False, info)
        if self.scope == 'reach':
            ee = np.asarray(self.genv.tool_pos(), dtype=np.float64)
            bp = np_(self.genv.w['bottle'].get_pos())
            d = float(np.linalg.norm(ee - bp)); info['reach_dist'] = d
            if d < self.REACH_DIST:
                info['reached'] = True; reward += 1.0
                return (obs['state'].astype(np.float32), reward, True, False, info)
        if self.scope == 'reach_goal':
            # WM fix stage 1b (2026-09-03): sparse +1 when the tool comes within REACH_GOAL_DIST of the GOAL can.
            # Threshold is REQUIRED from the environment (no silent default); calibrated by the random-policy probe.
            import os as _os
            _r = _os.environ.get('REACH_GOAL_DIST')
            assert _r, "scope=reach_goal needs REACH_GOAL_DIST (metres) exported"
            ee = np.asarray(self.genv.tool_pos(), dtype=np.float64)
            gp = np_(self.genv.w['goal'].get_pos())
            d = float(np.linalg.norm(ee - gp)); info['reach_goal_dist'] = d
            if d < float(_r):
                info['reached_goal'] = True; reward += 1.0
                return (obs['state'].astype(np.float32), reward, True, False, info)
        if self.scope == 'pick':
            # pick_shaping is applied in step(), once per decision -- NOT here.
            # Per-substep application under action_repeat>1 is a bug (see step()).
            if self.pick_hold_reward:
                # REWARD-DENSITY lever (ManiSkill/Adroit semantics; class docstring):
                # +1 for EVERY step the honest hold condition holds, terminate after
                # pick_hold_k CONSECUTIVE held steps. The run counter -- not a
                # separate predicate -- is what makes a whack-fling unprofitable: a
                # batted can separates and falls back through pick_z in a few frames,
                # collecting a few +1s but never the K-frame terminal, while a real
                # grasp holds indefinitely. Mirrored offline frame-for-frame by
                # train_sacfd_full.hold_region_encode_transitions (same K, same
                # pick_hold_held call, same "drop everything after the terminal").
                held = bool(pick_hold_held(float(obs['state'][10]),
                                           float(a_phys[6]), self.pick_z))
                self._hold_run = self._hold_run + 1 if held else 0
                reward += 1.0 if held else 0.0
                info['pick_held'] = held
                info['pick_hold_run'] = int(self._hold_run)
                if self._hold_run >= self.pick_hold_k:
                    info['pick_hold_done'] = True
                    return (obs['state'].astype(np.float32), reward, True, False, info)
            else:
                terminated = bool(info.get('picked'))
                if terminated:
                    truncated = False
                    return (obs['state'].astype(np.float32), reward, True, False, info)
        if self.scope == 'carrycontact':
            # PHASE PLAN: +1 and terminate on the env's contact predicate (can touches the goal can, picked history,
            # eef behind the can). UNCHANGED by amendment (o): (l)(c) makes carrycontact the explicit "contact by any
            # route, including still held" control against which the release-based slide statistic is read.
            if info.get('contact'):
                self._granted.add('contact')
                return (obs['state'].astype(np.float32), reward + 1.0, True, False, info)
        if self.scope == 'contact':
            # PHASE_PLAN (p): the grant is whatever the caller named. 'bare_contact' = the (m)/world-model-of-record
            # behaviour. 'slide_success' = the WITHDRAWN (o) reward, reachable only under CONTACT_GRANT_ALLOW_WITHDRAWN
            # and kept so the recorded (o) diagnostic (150-decision episodes, r=0, reason grip_closed) reproduces.
            # KEPT FROM (o) AND STILL RIGHT (coordinator, 2026-09-07): bare `contact` is logged and granted but does NOT
            # end the episode when the grant is not bare_contact, so credit for driving a still-carried can into the
            # goal is not paid; (p)'s prior-release predicate will formalise that.
            # 2026-09-10: restored verbatim from 7096fe6 (809601d dropped it together with the
            # contact_grant selector). The slide/contact PHASE is still on hold under (p);
            # the ladder unification changes scope='full' only.
            if self.contact_grant == 'bare_contact':
                if info.get('contact'):
                    self._granted.add('contact')
                    return (obs['state'].astype(np.float32), reward + 1.0, True, False, info)
            if info.get('contact'):
                self._granted.add('contact')
            if info.get('slide_success'):
                self._granted.add('slide_success')
                return (obs['state'].astype(np.float32), reward + 1.0, True, False, info)
        if self.scope == 'place':
            # PLACED_V2 (release-based, supersedes the mid-lift z-band proxy):
            # grip commanded open + can inside the shelf footprint/z-band +
            # near-upright, sustained PLACE_SUSTAIN consecutive frames.
            w = self.genv.w
            bp = np_(w['bottle'].get_pos())
            if self.shaping:
                # TRAINING-ONLY potential-based shaping + per-step cost (run-3
                # lever; see PLACE_SHAPING_* constants). Applied before the
                # placed_v2 early return so the terminal step is shaped too.
                # The honest metric remains the sparse placed_v2 terminal --
                # eval envs are built with shaping=False (the default).
                phi = self._place_phi(bp)
                reward += (self.PLACE_SHAPING_GAMMA * phi - self._phi
                           - self.PLACE_STEP_COST)
                self._phi = phi
            ok = (float(a_phys[6]) < self.PLACE_RELEASE
                  and in_shelf_footprint(bp)
                  and self.shelf_top_z + 0.01 < bp[2] < self.shelf_top_z + 0.07
                  and tilt_deg(np_(w['bottle'].get_quat())) < self.PLACE_TILT_DEG)
            self._pv2_run = self._pv2_run + 1 if ok else 0
            info['placed_v2'] = self._pv2_run >= self.PLACE_SUSTAIN
            if info['placed_v2']:
                self._granted.add('placed_v2')
                return (obs['state'].astype(np.float32), reward + 1.0, True, False,
                        info)   # v6: success must dominate the shaped-return landscape
        # place scope does NOT terminate on the nested proxy: a nesting release is
        # the best place outcome and satisfies placed_v2 ~10 frames later -- letting
        # nested cut the sustain window paid 0 for it (seen on uid 242's replay,
        # terminated rewardless at step 274, 9 frames short of its placed_v2).
        # (pick_hold_reward reaches this line only on a NON-held step; the nested proxy
        # needs contact, which needs the hardened picked -- 10 held frames -- plus a
        # carry and release, so it cannot pre-empt a 25-frame hold in practice, and the
        # tip rule below cannot fire mid-hold either: it requires grip OPEN.)
        # scope='full' placed_v2 / tracker now run at the TOP of this method
        # (_full_scope_predicates), because the ladder conditions on them.
        if self.scope == 'full':
            # THE ONLY full-scope terminals besides the tip rule (brief D2): this ladder's
            # own terminal stage(s), which are PAID by construction -- 'slide_success' under
            # staged, 'nested_v2' under sparse, 'home' under both Ladder-N variants. The
            # legacy `nested` proxy terminates nothing: its clauses are a subset of the
            # slide's, so it used to end the episode on the first frame of the slide window
            # and the +4 could never be paid (defect 5). The accountant computed this from
            # the same `info` the reward came from -- one read, not two.
            terminated = ladder_terminated
        else:
            terminated = bool(info.get('nested')) and self.scope != 'place'
        # THE TIP RULE. Threshold (TIP_DEG), termination and penalty are the rule of record;
        # the GUARD is selected by the constructor's tip_guard (amendment (aa), see the
        # TIP_GUARD_* block at the top of this file).
        #   'grip'        commanded grip open. a_phys[6] in the 7-dim joint action -- a_phys[4]
        #                 is a JOINT angle (the grip-column bug, 4th sighting). Sustain 1, so
        #                 this branch is bit-identical to every run before 2026-09-11.
        #   'not_in_hand' the tracker's own `in_hand`, already computed this frame by
        #                 _full_scope_predicates (scope='full' only, asserted in __init__).
        #                 NOT re-derived here: one definition, two readers.
        # The sustain counts consecutive ENV FRAMES of the CONJUNCTION (tilt AND guard), which
        # is what Lane 6 measured; 4 frames = 1 decision at action_repeat 4.
        if not terminated:
            if self.tip_guard == 'grip':
                _free = float(a_phys[6]) < self.GRIP_OPEN
            else:
                # `in_hand` missing would mean the tracker did not run this frame; treat that
                # as "held" (never fire) rather than as a fall-through to firing.
                _free = not bool((self._track or {}).get('in_hand', True))
            self._tip_free_run = (
                self._tip_free_run + 1
                if (_free and tilt_deg(np_(self.genv.w['bottle'].get_quat())) > self.TIP_DEG)
                else 0)
        if not terminated and self._tip_free_run >= self.tip_guard_sustain:
            # scope='place' ONLY pays a penalty for the tip (dropped the held
            # can); pick/full keep TIP_PENALTY = 0.0 (termination only).
            if self.scope == 'place' and not self.phase_sparse:
                # v7: a tip ON the shelf is a near-miss from a place ATTEMPT --
                # cheap (-0.1) so failure during experimentation stays affordable;
                # a drop elsewhere is a real failure (-1). A flat -1 (v6) taught
                # the actor that opening the grip is globally dangerous: release
                # attempts -> 0, places -> 0 despite the curriculum.
                reward += (-0.1 if in_shelf_footprint(np_(self.genv.w['bottle'].get_pos()))
                           else self.PLACE_TIP_PENALTY)
            else:
                reward += self.TIP_PENALTY
            terminated = True
            info['tipped'] = True
        truncated = (not terminated) and self._t >= self.max_steps
        return obs['state'].astype(np.float32), reward, terminated, truncated, info


class CartesianFullTaskEnv(gym.Env):
    """Full staged task with 4-DOF Cartesian velocity actions (the demos' native
    teleop modality). action = [-1,1]^5 -> [vx,vy,vz (VCAP), v_pitch (PITCH_CAP),
    grip 0..1]. Same staged reward machine as FullTaskEnv; workspace enforcement is
    inherent (CartesianCanEnv clamps the tool setpoint to the teleop box)."""

    metadata = {'render_modes': []}
    # Terminate + small penalty when the can lies tipped FREE (tilt>60 AND grip open).
    # Demo census (2026-07-26, all 91): once past 60 deg the can NEVER returns upright
    # (31/32; sole exception a no-pick) -> tipped-free is a true dead end. But the
    # grip-open guard is essential: demos routinely CARRY the can pitched >60 in the
    # gripper (4/5 contact-stage demos reach goal contact that way) -- a bare tilt rule
    # would outlaw demonstrated strategy. Picked-can only: goal-can orientation is not
    # in recorded states, so demos could not mirror a goal-tip rule without a replay.
    TIP_DEG = 60.0
    TIP_PENALTY = 0.0
    GRIP_OPEN = 0.3          # grip command below this = not holding

    def __init__(self, backend='cpu', max_steps=900, fixed_uid=None, render_size=None,
                 camera_rig=False, control='vel', scope='full',
                 tip_guard=TIP_GUARD_DEFAULT):
        super().__init__()
        # Amendment (aa)'s guard, the same two choices FullTaskEnv takes and the same default
        # 'grip' (this class has no live runs; the argument exists so the two tip sites cannot
        # drift apart again). This class runs no StageTracker, so `not_in_hand` is answered by
        # stage_predicates.is_in_hand -- the function the tracker itself calls, applied to the
        # tool point and the can centre. `in_hand` is instantaneous, so nothing is lost.
        assert tip_guard in TIP_GUARD_CHOICES, (
            f'tip_guard must be one of {list(TIP_GUARD_CHOICES)}, got {tip_guard!r}')
        self.tip_guard = str(tip_guard)
        self.tip_guard_sustain = int(TIP_GUARD_SUSTAIN[self.tip_guard])
        self._tip_free_run = 0
        from cartesian_env import CartesianCanEnv
        self.control = control
        # scope='pick': +1 and TERMINATE on the pick. Collapses the credit-assignment
        # horizon from ~1700 steps to ~600 (median demo pick frame) and removes all
        # downstream noise -- the simplest configuration that still exercises the
        # whole stack, for use as a positive control.
        self.scope = scope
        self.cenv = CartesianCanEnv(backend=backend, render_size=render_size,
                                    max_steps=10 ** 9, camera_rig=camera_rig,
                                    control=control)
        self.genv = self.cenv.env               # underlying GenesisCanEnv
        self.max_steps = int(max_steps)
        self.fixed_uid = fixed_uid
        self.success_uids = sorted(self.genv.solved_uids)
        self.pick_z = float(self.genv.w['pick_z'])
        self.observation_space = spaces.Box(-np.inf, np.inf, (18,), np.float32)
        _adim = 7 if control in ('abs6', 'delta6') else 5
        self.action_space = spaces.Box(-1.0, 1.0, (_adim,), np.float32)
        self._t = 0
        self._granted = set()

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        uid = (options or {}).get('uid') or self.fixed_uid
        if uid is None:
            uid = int(self.np_random.choice(self.success_uids))
        obs = self.cenv.reset(uid=int(uid))
        self._t = 0
        self._tip_free_run = 0          # amendment (aa), per-episode state
        self._granted = set()
        return obs['state'].astype(np.float32), {'uid': int(uid)}

    def reset_to(self, ic):
        obs = self.cenv.reset(**ic)
        self._t = 0
        self._tip_free_run = 0          # amendment (aa), per-episode state
        self._granted = set()
        return obs['state'].astype(np.float32), {}

    def step(self, action):
        # Explicit per-mode dispatch. The previous 'delta else velocity' fallback
        # silently gave 7-dim abs6/delta6 actions the 5-dim VELOCITY denormalisation
        # -> IndexError at a[6], i.e. CartesianFullTaskEnv was broken for BOTH 6-DOF
        # modes (single-env path: RLPD/SACfD and non-VEC dv3; the batched world has
        # its own dispatch and was unaffected).
        _denorm = {
            'delta': self.cenv.denormalize_delta,
            'delta6': self.cenv.denormalize_delta6,
            'abs': self.cenv.denormalize_abs,
            'abs6': self.cenv.denormalize_abs6,
        }.get(self.control, self.cenv.denormalize_action)
        a_phys = _denorm(np.asarray(action, np.float32))
        obs, _env_done, info = self.cenv.step(a_phys)
        self._t += 1
        # same per-step nested proxy as FullTaskEnv (honest settled nested is eval-only)
        # grip column is mode-dependent (6th sighting of the grip-column bug:
        # this proxy read a_phys[4] — a wrist ROTATION axis in 7-dim modes —
        # while the tip rule below was already mode-aware)
        _grip_p = float(a_phys[6] if len(a_phys) >= 7 else a_phys[4])
        if info.get('contact') and _grip_p < 0.3 \
                and 'nested' not in self._granted:
            w = self.genv.w
            if tilt_deg(np_(w['bottle'].get_quat())) < 20 \
                    and tilt_deg(np_(w['goal'].get_quat())) < 20:
                info['nested'] = True
        reward = 0.0
        # _CARTESIAN_STAGE_REWARD, NOT the unified STAGE_REWARD: the 4-DOF teleop arm is a
        # different experiment and is not part of the end-to-end unification, so it keeps
        # the ladder every cartesian run on record trained under (2026-09-10).
        for stage, r in _CARTESIAN_STAGE_REWARD.items():
            if info.get(stage) and stage not in self._granted:
                reward += r
                self._granted.add(stage)
        if self.scope == 'reach_goal':
            # WM fix (EEF arm, 2026-09-03): sparse +1 when the tool reaches the GOAL can.
            # Threshold REQUIRED from the environment -- no silent default.
            import os as _os
            _r = _os.environ.get('REACH_GOAL_DIST')
            assert _r, "scope=reach_goal needs REACH_GOAL_DIST (metres) exported"
            ee = np.asarray(self.genv.tool_pos(), dtype=np.float64)
            gp = np_(self.genv.w['goal'].get_pos())
            d = float(np.linalg.norm(ee - gp)); info['reach_goal_dist'] = d
            if d < float(_r):
                info['reached_goal'] = True; reward += 1.0
                return (obs['state'].astype(np.float32), reward, True, False, info)
        if self.scope == 'pick':
            terminated = bool(info.get('picked'))
            if terminated:
                truncated = False
                return (obs['state'].astype(np.float32), reward, True, False, info)
        terminated = bool(info.get('nested'))
        # grip column is mode-dependent: index 4 in 5-dim vel/delta/abs actions,
        # index 6 in 7-dim abs6/delta6 (index 4 there is a ROTATION axis -- the
        # grip-column bug, 5th sighting; misread grip made the tip rule fire on
        # wrist rotation in 6-DOF modes, a candidate cause of the abs6-RL
        # 13-33-step degenerate episodes)
        _grip = float(a_phys[6] if len(a_phys) >= 7 else a_phys[4])
        if not terminated:
            if self.tip_guard == 'grip':
                _free = _grip < self.GRIP_OPEN
            else:
                from stage_predicates import is_in_hand
                _bp = np_(self.genv.w['bottle'].get_pos())
                _free = not is_in_hand(np.asarray(self.genv.tool_pos(), np.float64)[:2],
                                       np.asarray(_bp, np.float64)[:2])
            self._tip_free_run = (
                self._tip_free_run + 1
                if (_free and tilt_deg(np_(self.genv.w['bottle'].get_quat())) > self.TIP_DEG)
                else 0)
        if not terminated and self._tip_free_run >= self.tip_guard_sustain:
            reward += self.TIP_PENALTY
            terminated = True
            info['tipped'] = True
        truncated = (not terminated) and self._t >= self.max_steps
        return obs['state'].astype(np.float32), reward, terminated, truncated, info
