"""FullTaskEnv: gymnasium wrapper around GenesisCanEnv for the FULL task (plan E).

Staged sparse reward, each granted the FIRST time the env's own honest predicate flips:
    picked +1, placed +1, contact +2, nested +4.  Terminates on nested; truncates at
max_steps (default 900 = 30 s at 30 Hz; demo contact lands well inside that).
Same normalized [-1,1]^7 action convention as PickOnlyEnv (see pick_env.py docstring).

scope='pick':  +1 and terminate on the pick grant.
scope='place': reset restores a random banked POST-PICK entry state
    (baselines/pick_entry_states.json; arm qpos + finger closure + held can pose +
    goal), settles ~20 steps holding the entry commands and verifies the can is
    still held (resamples otherwise); +1 and terminate on PLACED_V2 = grip cmd
    released (<0.45) + can in shelf footprint/z-band + tilt<20 deg, sustained 10
    consecutive frames. Default cap 600 steps. Tip rule applies in all scopes.
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
from pick_env import STATE_DIM, ACT_DIM, denormalize_action  # noqa: E402
sys.path.insert(0, str(REPO / 'can_pos_recovery'))
from replay_harness import (tilt_deg, in_shelf_footprint, BOX_TOP_Z,  # noqa: E402
                            HARDCODED_START, gripper_targets)

STAGE_REWARD = dict(picked=1.0, placed=1.0, contact=2.0, nested=4.0)
PLACE_ENTRY_BANK = REPO / 'baselines' / 'pick_entry_states.json'


class FullTaskEnv(gym.Env):
    TIP_DEG = 60.0
    TIP_PENALTY = 0.0
    GRIP_OPEN = 0.3          # grip command below this = not holding
    # --- scope='place' (PLACED_V2, release-based -- PAPER_PLAN stage-wise matrix) ---
    PLACE_RELEASE = 0.45     # grip command below this counts as released
    PLACE_TILT_DEG = 20.0    # can must be near-upright
    PLACE_SUSTAIN = 10       # predicate must hold this many CONSECUTIVE frames
    PLACE_HELD_Z = 0.13      # entry-restore verification: can center above this = still held
    PLACE_SETTLE = 20        # physics steps holding the entry pose before verifying
    PLACE_MAX_TRIES = 30     # resamples per reset before giving up
    metadata = {'render_modes': []}

    def __init__(self, backend='cpu', max_steps=None, fixed_uid=None, render_size=None,
                 camera_rig=False, workspace_limit=False, scope='full'):
        super().__init__()
        self.genv = GenesisCanEnv(backend=backend, render_size=render_size,
                                  camera_rig=camera_rig,
                                  workspace_limit=workspace_limit)
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
            self._entry_bank = {int(u): e for u, e in
                                json.loads(PLACE_ENTRY_BANK.read_text()).items()}
            assert self._entry_bank, f'empty entry bank {PLACE_ENTRY_BANK}'
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

    def _restore_place_entry(self, uid):
        """Restore one banked post-pick state; True if the can survives the settle.

        Order matters: genv.reset first (clears velocities, seeds goal/can, steps
        once with the arm at HARDCODED_START), THEN overwrite arm+finger joints and
        re-set the can pose (the reset's single step lets the unsupported can drop
        ~0.5 mm), THEN hold the entry commands for PLACE_SETTLE physics steps so
        the grasp re-engages before the policy sees the state.
        """
        e = self._entry_bank[uid]
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
        return obs['state'].astype(np.float32), {'uid': int(uid)}

    def _reset_place(self, uid=None):
        tried = []
        for _ in range(self.PLACE_MAX_TRIES):
            u = int(uid) if uid is not None else int(self.np_random.choice(self.success_uids))
            self.place_attempts += 1
            ok = self._restore_place_entry(u)
            if ok:
                self.place_survived += 1
            if not self._survival_reported and self.place_attempts >= 1 and ok:
                # one-shot startup line; running counters stay queryable
                print(f'[place] entry-restore survival so far: '
                      f'{self.place_survived}/{self.place_attempts}', flush=True)
                self._survival_reported = True
            if ok:
                self._t = 0
                self._pv2_run = 0
                # the pick already happened in the demo this state came from: the
                # env-level picked flag must be up for placed/contact predicates,
                # and 'picked' is pre-granted so the restored pick pays no reward
                self.genv._picked = True
                self._granted = {'picked'}
                return (self.genv._obs()['state'].astype(np.float32),
                        {'uid': u, 'entry_frame': int(self._entry_bank[u]['frame'])})
            tried.append(u)
            print(f'[place] entry {u} did not survive restore (can dropped), '
                  f'resampling', flush=True)
            if uid is not None:      # explicit uid requested: fail loudly, no swap
                break
        raise RuntimeError(f'scope=place reset: no entry survived restore '
                           f'(tried {tried})')

    def reset_to(self, ic):
        """Reset to an explicit IC dict (ic_sampling-style) -- random-IC eval."""
        obs = self.genv.reset(**ic)
        self._t = 0
        self._granted = set()
        return obs['state'].astype(np.float32), {}

    def step(self, action):
        a_phys = denormalize_action(action)
        obs, _env_done, info = self.genv.step(a_phys)
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
                if self.scope != 'place':
                    reward += r
                self._granted.add(stage)
        if self.scope == 'pick':
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
            ok = (float(a_phys[6]) < self.PLACE_RELEASE
                  and in_shelf_footprint(bp)
                  and BOX_TOP_Z + 0.01 < bp[2] < BOX_TOP_Z + 0.07
                  and tilt_deg(np_(w['bottle'].get_quat())) < self.PLACE_TILT_DEG)
            self._pv2_run = self._pv2_run + 1 if ok else 0
            info['placed_v2'] = self._pv2_run >= self.PLACE_SUSTAIN
            if info['placed_v2']:
                self._granted.add('placed_v2')
                return (obs['state'].astype(np.float32), reward + 1.0, True, False,
                        info)
        # place scope does NOT terminate on the nested proxy: a nesting release is
        # the best place outcome and satisfies placed_v2 ~10 frames later -- letting
        # nested cut the sustain window paid 0 for it (seen on uid 242's replay,
        # terminated rewardless at step 274, 9 frames short of its placed_v2).
        terminated = bool(info.get('nested')) and self.scope != 'place'
        # grip is a_phys[6] in the 7-dim joint action (a_phys[4] is a JOINT angle --
        # the grip-column bug, 4th sighting; this block also never ran before
        # 2026-08-01: self.scope and the class constants were missing entirely, so
        # every FullTaskEnv.step crashed and all joint dv3 periodic evals failed)
        if not terminated and float(a_phys[6]) < self.GRIP_OPEN \
                and tilt_deg(np_(self.genv.w['bottle'].get_quat())) > self.TIP_DEG:
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
        if info.get('contact') and float(a_phys[4]) < 0.3 \
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
