"""FullTaskEnv: gymnasium wrapper around GenesisCanEnv for the FULL task (plan E).

Staged sparse reward, each granted the FIRST time the env's own honest predicate flips:
    picked +1, placed +1, contact +2, nested +4.  Terminates on nested; truncates at
max_steps (default 900 = 30 s at 30 Hz; demo contact lands well inside that).
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

STAGE_REWARD = dict(picked=1.0, placed=1.0, contact=2.0, nested=4.0)
PLACE_ENTRY_BANK = REPO / 'baselines' / 'pick_entry_states.json'

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


class FullTaskEnv(gym.Env):
    TIP_DEG = 60.0
    TIP_PENALTY = 0.0        # pick/full scopes: tip terminates but carries no penalty
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

    def __init__(self, backend='cpu', max_steps=None, fixed_uid=None, render_size=None,
                 camera_rig=False, workspace_limit=False, scope='full', shaping=False,
                 entry_bank=None, action_mode='absolute', delta_cap=0.025,
                 delta_leash_mult=5.0, action_repeat=1, delta_ref='target',
                 pick_hold_reward=False, pick_hold_k=25, pick_shaping=False,
                 pick_shaping_gamma=None):
        super().__init__()
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
        assert not (pick_shaping and scope != 'pick'), 'pick_shaping is a scope=pick lever'
        self.pick_shaping = bool(pick_shaping)
        # gamma MUST match the consuming agent's discount for exact Ng-invariance
        # (RLPD 0.998 default; r2dreamer passes 0.999, dv3 0.997).
        self._pick_gamma = float(pick_shaping_gamma) if pick_shaping_gamma else self.PICK_SHAPING_GAMMA
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
            max_steps = 600 if scope == 'place' else 900
        self.max_steps = int(max_steps)
        self.fixed_uid = fixed_uid
        self.success_uids = sorted(
            u for u, r in self.genv.placements.items() if r.get('label') == 'success')
        if scope == 'place':
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
        self._granted = set()
        self._pv2_run = 0
        self._attempted = False
        self._phi = 0.0

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
        """-SCALE * ||eef - can||, the pick-scope approach potential (training-only)."""
        ee = np.asarray(self.genv.tool_pos(), dtype=np.float64)
        bp = np_(self.genv.w['bottle'].get_pos())
        return -self.PICK_SHAPING_SCALE * float(np.linalg.norm(ee[:3] - np.asarray(bp[:3], dtype=np.float64)))

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
        if self.scope == 'place':
            return self._reset_place(uid)
        if uid is None:
            uid = int(self.np_random.choice(self.success_uids))
        obs = self.genv.reset(uid=int(uid))
        self._t = 0
        self._granted = set()
        self._hold_run = 0
        self._pick_phi_prev = self._pick_phi() if self.pick_shaping else 0.0
        self._sync_dj_target()
        return obs['state'].astype(np.float32), {'uid': int(uid)}

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
                self._pv2_run = 0; self._attempted = False
                # seed the shaping potential at the settled entry state (cheap;
                # computed unconditionally so shaping toggling never desyncs it)
                self._phi = self._place_phi(np_(self.genv.w['bottle'].get_pos()))
                # the pick already happened in the demo this state came from: the
                # env-level picked flag must be up for placed/contact predicates,
                # and 'picked' is pre-granted so the restored pick pays no reward
                self.genv._picked = True
                self._granted = {'picked'}
                self._sync_dj_target()
                return (self.genv._obs()['state'].astype(np.float32),
                        {'uid': u, 'entry_frame': int(e['frame']),
                         'entry_frac': float(e.get('frac', 0.0))})
            tried.append((u, int(e['frame'])))
            print(f'[place] entry {u}@{int(e["frame"])} did not survive restore '
                  f'(can dropped), resampling', flush=True)
        raise RuntimeError(f'scope=place reset: no entry survived restore '
                           f'(tried {tried})')

    def reset_to(self, ic):
        """Reset to an explicit IC dict (ic_sampling-style) -- random-IC eval."""
        obs = self.genv.reset(**ic)
        self._t = 0
        self._granted = set()
        self._hold_run = 0
        self._pick_phi_prev = self._pick_phi() if self.pick_shaping else 0.0
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
            if terminated or truncated:
                break
        return obs, total_reward, terminated, truncated, info

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
        # GenesisCanEnv only computes the honest (settled) nested at its own horizon, and
        # _nested() steps the sim so it can't run per-step. TRAINING uses a cheap proxy:
        # contact + can & goal upright + gripper commanded open. EVAL keeps the settled
        # metric (eval_sac -> eval_core on GenesisCanEnv), so reported numbers stay honest.
        if info.get('contact') and float(a_phys[6]) < 0.3 \
                and 'nested' not in self._granted:
            w = self.genv.w
            if tilt_deg(np_(w['bottle'].get_quat())) < 20 \
                    and tilt_deg(np_(w['goal'].get_quat())) < 20:
                info['nested'] = True
        reward = 0.0
        for stage, r in STAGE_REWARD.items():
            if info.get(stage) and stage not in self._granted:
                # scope='place' pays ONLY the +1 placed_v2 terminal (below); stage
                # grants are still tracked for logging (r2dreamer adapter reads
                # _granted) but carry no reward -- the restored pick is pre-granted.
                # pick_hold_reward likewise pays ONLY the per-step hold reward below:
                # keeping the one-shot 'picked' grant too would double-pay the lift
                # and re-import the terminal-only signal the lever exists to replace.
                if self.scope != 'place' and not self.pick_hold_reward:
                    reward += r
                self._granted.add(stage)
        if self.scope == 'pick':
            if self.pick_shaping:
                # TRAINING-ONLY approach potential (PICK_SHAPING_*); applied before
                # the pick early-return so the terminal step is shaped too.
                phi = self._pick_phi()
                reward += self._pick_gamma * phi - self._pick_phi_prev
                self._pick_phi_prev = phi
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
                  and BOX_TOP_Z + 0.01 < bp[2] < BOX_TOP_Z + 0.07
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
        terminated = bool(info.get('nested')) and self.scope != 'place'
        # grip is a_phys[6] in the 7-dim joint action (a_phys[4] is a JOINT angle --
        # the grip-column bug, 4th sighting; this block also never ran before
        # 2026-08-01: self.scope and the class constants were missing entirely, so
        # every FullTaskEnv.step crashed and all joint dv3 periodic evals failed)
        if not terminated and float(a_phys[6]) < self.GRIP_OPEN \
                and tilt_deg(np_(self.genv.w['bottle'].get_quat())) > self.TIP_DEG:
            # scope='place' ONLY pays a penalty for the tip (dropped the held
            # can); pick/full keep TIP_PENALTY = 0.0 (termination only).
            if self.scope == 'place':
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
                 camera_rig=False, control='vel', scope='full'):
        super().__init__()
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
        self._granted = set()
        return obs['state'].astype(np.float32), {'uid': int(uid)}

    def reset_to(self, ic):
        obs = self.cenv.reset(**ic)
        self._t = 0
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
        for stage, r in STAGE_REWARD.items():
            if info.get(stage) and stage not in self._granted:
                reward += r
                self._granted.add(stage)
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
        if not terminated and _grip < self.GRIP_OPEN \
                and tilt_deg(np_(self.genv.w['bottle'].get_quat())) > self.TIP_DEG:
            reward += self.TIP_PENALTY
            terminated = True
            info['tipped'] = True
        truncated = (not terminated) and self._t >= self.max_steps
        return obs['state'].astype(np.float32), reward, terminated, truncated, info
