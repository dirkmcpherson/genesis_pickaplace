"""Minimal 30 Hz env wrapper around the pick-a-place world for policy evaluation.

Interface (gym-like, no gym dependency):
    env = GenesisCanEnv(render_size=(96, 96))          # builds world once (one per process)
    obs = env.reset(uid=243)        # solved-trial initial conditions from trial_placements
    obs, done, info = env.step(action)   # action: [6 joint targets (rad), gripper 0..1]
    info carries picked/placed/contact/nested checkpoint flags.

Observation dict:
    state: float32[16] = 6 joint pos, gripper motor/100, can xyz, can quat, goal xy
    image: uint8[H,W,3] scene camera (roughly matches the real 96x96 cam obs)

The action convention matches the demos: absolute joint-position targets applied for
3 physics steps (dt=0.01, substeps from the placement table's world config).
"""
import os
import json, sys, pathlib as pl
import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO / 'can_pos_recovery'))
import torch
from replay_harness import (build_world, gripper_targets, tilt_deg, in_shelf_footprint,
                            HARDCODED_START, BOX_TOP_Z, GP_CLOSE, STATIC_BOTTLE_POSITION,
                            NESTED_TOUCH_DIST)

def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)


# --- teleop workspace constraint (see WORKSPACE_CONSTRAINT.md) -------------------
# The real gen3_lite teleop plugin clamped the TOOL (gripper-tip) setpoint into this
# base-frame box, so no human demo could ever leave it. Training had no such bound
# (only ARM_LIMIT=pi on joint targets), letting RL explore behind the base, through
# the table and above the shelf -- regions with no demo support.
# Box = the plugin's nominal box, which was verified to CONTAIN all replayed demo
# tool positions (demo extremes x[0.326,0.742] y[-0.231,0.111] z[0.031,0.252]), so
# enforcing it cannot clip real behavior. The tighter p1/p99 empirical box was
# REJECTED: it excludes 9-13% of genuine demo frames.
WS_LO = np.array([0.30, -0.25, 0.015])
WS_HI = np.array([0.80,  0.25, 0.60])
WS_MARGIN = 0.02   # recovery poses must be >=2cm inside the box (anti boundary-hug)
# Fixed wrist(eef link) -> tool(gripper tip) offset in the wrist frame. Genesis merges
# the URDF tool_frame link, so the tool must be reconstructed; calibrated from the real
# tool_pose at HARDCODED_START (cartesian_env.REF_TOOL_AT_START). ~140mm.
REF_TOOL_AT_START = np.array([0.367, 0.011, 0.09])

# --- picked predicate guard (2026-08-09, r2dreamer pick_dH_v4 audit) -------------
# `can_z > pick_z AND grip commanded closed` alone is RL-gameable: the v4 dreamer
# policy learned to WHACK the can airborne with fingers closed -- "picks" at 8-24
# sim steps from reset (physically impossible grasps; 148 episodes even collected
# placed/contact on the same instant as the flung can crossed the shelf, scores
# 200/400 at reward_scale 100). Guard: the can must RIDE the gripper -- all of
# (z > pick_z, grip closed, |eef-can| < PICK_EEF_DIST) sustained PICK_SUSTAIN
# consecutive frames. A genuinely held can tracks the eef at ~0.146 m, sub-mm
# stable, for the whole lift (measured on the uid232 raw-action replay); a batted
# can separates ballistically and falls back through pick_z. Cost: the grant lands
# PICK_SUSTAIN-1 frames (~0.3 s) later than before; demos still grant (verified).
PICK_EEF_DIST = 0.20
PICK_SUSTAIN = 10
# --- amendment (l) 2026-09-07: slide_success = the task as demonstrated (can ON THE SHELF, RELEASED, touching the
# goal), sustained 3 decisions. One decision = action_repeat 4 env frames; one env frame = the 3 scene steps step()
# takes -> SLIDE_SUSTAIN frames. GRIP_OPEN_CMD is the same threshold the legacy nested proxy uses.
SLIDE_SUSTAIN_DECISIONS = 3
SLIDE_SUSTAIN = SLIDE_SUSTAIN_DECISIONS * 4   # env frames
GRIP_OPEN_CMD = 0.3
SETTLE_STEPS = 100        # the post-episode settle _nested() has always run (scene steps)


def _quat_to_R(q):
    """wxyz -> 3x3 rotation matrix (avoids a scipy import on the hot path)."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)]])


class GenesisCanEnv:
    def __init__(self, backend='cpu', render_size=None, max_steps=1200, camera_rig=False,
                 workspace_limit=False):
        table = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())
        self.world_cfg = table['world']
        self.placements = {int(u): r for u, r in table['trials'].items()
                           if r['status'] in ('ok', 'ok_batch')}
        self.max_steps = max_steps
        self.render_size = render_size
        w = build_world(backend=backend,
                        finger_force=self.world_cfg['finger_force'],
                        finger_kp=self.world_cfg['finger_kp'],
                        can_height=self.world_cfg['can_height'],
                        can_rho=self.world_cfg['can_rho'],
                        substeps=self.world_cfg.get('substeps', 1),
                        table=self.world_cfg.get('table', False),
                        can_radius=self.world_cfg.get('can_radius', 0.035),
                        camera='rig' if camera_rig else (render_size is not None))
        self.w = w
        self._t = 0
        self._pick_run = 0
        self._uid = None
        self.camera_rig = camera_rig
        self.workspace_limit = workspace_limit   # enforce the teleop tool box
        self._tool_offset = None                 # wrist-frame wrist->tool offset (cached)
        self._last_valid_q = None                # last MEASURED in-box joint config
        self.ws_violations = 0                   # per-episode count (reported in info)

    def _calib_tool_offset(self):
        """Cache the constant wrist->tool offset in the wrist frame (once per world)."""
        w = self.w
        wp = np_(w['eef'].get_pos()); wq = np_(w['eef'].get_quat())
        self._tool_offset = _quat_to_R(wq).T @ (REF_TOOL_AT_START - wp)

    def tool_pos(self):
        """Current tool (gripper-tip) position from state reads the env already makes.

        CHEAP path (~0.1ms): reuses eef get_pos/get_quat. Used for the reactive
        constraint. Overshoot is bounded by one step of motion (~4mm at the teleop
        velocity cap), which is why predicting with FK is not worth 3x the step cost.
        """
        w = self.w
        if self._tool_offset is None:
            self._calib_tool_offset()
        wp = np_(w['eef'].get_pos()); wq = np_(w['eef'].get_quat())
        return wp + _quat_to_R(wq) @ self._tool_offset

    def tool_pos_for(self, arm_target):
        """Tool position the commanded joint targets WOULD reach (PRE-EMPTIVE, exact).

        Uses genesis forward_kinematics on the target config. Correct but expensive:
        measured 50.3ms/step vs 17.6ms unconstrained (~3x). Kept for offline analysis
        and for anyone who needs a hard pre-emptive bound; the training path uses the
        reactive tool_pos() check instead.
        """
        w = self.w
        if self._tool_offset is None:
            self._calib_tool_offset()
        q = np_(w['kinova'].get_dofs_position(dofs_idx_local=w['kdofs'])).copy()
        q[:6] = arm_target
        links_pos, links_quat = w['kinova'].forward_kinematics(
            qpos=torch.as_tensor(q, dtype=torch.float32))
        i = w['eef'].idx_local
        wp = np_(links_pos[i]); wq = np_(links_quat[i])
        return wp + _quat_to_R(wq) @ self._tool_offset

    def in_workspace(self, tool, margin=0.0):
        """margin>0 shrinks the box (used to pick recovery poses that are solidly
        interior, so recovery does not stall hugging the boundary)."""
        return bool(np.all((tool >= WS_LO + margin) & (tool <= WS_HI - margin)))

    def rig_obs(self):
        """(64,64,6) uint8: topB overhead RGB ++ through-gripper wrist RGB (dv3 rig)."""
        w = self.w
        w['cam_wrist'].move_to_attach()
        top = np.asarray(w['cam_top'].render()[0], dtype=np.uint8)
        wrist = np.asarray(w['cam_wrist'].render()[0], dtype=np.uint8)
        return np.concatenate([top, wrist], axis=-1)

    @property
    def solved_uids(self):
        return sorted(self.placements)

    def reset(self, uid=None, can_pos=None, can_quat=None, goal_pos=None):
        w = self.w
        if uid is not None:
            r = self.placements[uid]
            can_pos = r['can_pos']
            # #27: use the single corrected static goal, NOT the stale per-trial r['goal_pos']
            # (computed under the old goal) -- so demo-replay training obs carry the right target.
            goal_pos = (STATIC_BOTTLE_POSITION[0], STATIC_BOTTLE_POSITION[1], w['goal_start_z'])
            can_quat = r.get('can_quat') or [1, 0, 0, 0]
        self._uid = uid
        kin = w['kinova']
        kin.set_dofs_position(np.array(HARDCODED_START), w['kdofs'])
        kin.zero_all_dofs_velocity()
        # P2 FIX (2026-08-14): re-issue the CONTROLLER TARGETS, not just the poses.
        # set_dofs_position teleports qpos but the PD controllers keep the PREVIOUS
        # episode's final control_dofs_position target, so the scene.step() below --
        # and every step until the next command lands -- ran under stale, history-
        # dependent control. Probe: post-reset obs differed by history (grip-effort
        # dim by 33.9) and identical command sequences diverged 0.05 rad (2x delta
        # cap) across histories; eval episodes were NOT independent (uid 243 picked
        # as episode 7 of a sequence, failed alone). Mirrors step()'s own calls.
        kin.control_dofs_position(np.array(HARDCODED_START[:6]),
                                  dofs_idx_local=w['kdofs'][:6])
        kin.control_dofs_position(np.array(HARDCODED_START[-4:]),
                                  dofs_idx_local=np.array(w['kdofs'][-4:]))
        w['bottle'].set_pos(can_pos); w['bottle'].set_quat(list(can_quat or [1, 0, 0, 0]))
        w['goal'].set_pos(goal_pos); w['goal'].set_quat([1, 0, 0, 0])
        for ent in (w['bottle'], w['goal']):
            try: ent.zero_all_dofs_velocity()
            except Exception: pass
        w['scene'].step()
        self._t = 0
        self._picked = self._placed = self._contact = False
        # contact_push (2026-09-07, logged only -- `contact` is unchanged): see step()
        self._contact_push = False
        self._contact_frame = None; self._contact_push_frame = None
        self._contact_gripper_goal = False   # gripper touched the GOAL while the pick-can touched it
        self._contact_farside = False        # some pick-can/goal contact frame had the TOOL on the far side
        self._contact_farside_wrist = False  # ... the WRIST on the far side (the withdrawn first definition)
        # slide_success (amendment (l), logged only): sticky; route 'sustained' (in-episode) or 'settle' (held continuation)
        self._slide_success = False; self._slide_run = 0; self._slide_frame = None; self._slide_route = None
        self._last_grip_cmd = None   # last COMMANDED grip (physical 0..1); held through the post-episode settle
        self._pick_run = 0   # consecutive frames satisfying the held-can guard
        # Seed with the reset configuration: HARDCODED_START is inside the box by
        # construction, so the very first out-of-box action can be held against it.
        # (Leaving this None meant the cache could never populate -- the first action
        # already left the box, and only in-box steps refresh it.)
        self._last_valid_q = np.array(HARDCODED_START[:6], dtype=np.float64)
        # Calibrate the wrist->tool offset HERE, where the arm is known to be at
        # HARDCODED_START (the pose REF_TOOL_AT_START was measured at). Calibrating
        # lazily on first use was a bug: if the first tool_pos() call happened
        # mid-episode, the offset was fitted at the wrong pose and every tool position
        # after it was wrong.
        # contact_push (2026-09-07) reads tool_pos() every contact frame, so the offset must exist for EVERY
        # env, not only workspace-limited ones: calibrate once per world, here, at the known start pose.
        if self.workspace_limit or self._tool_offset is None:
            self._calib_tool_offset()
        self.ws_violations = 0
        return self._obs()

    def step(self, action, grip_motor=None, arm_cmd=None):
        """grip_motor/arm_cmd: raw recorded commands (gripper 0..100 scale, may
        overshoot <0/>100; arm float64 joint targets). DEMO COLLECTION ONLY --
        policies keep the clipped-float32 action convention. Replaying a demo needs
        the EXACT tape: the [0,1] grip clip (recorded overshoot to -0.46) AND the
        float32 truncation of arm targets (~1e-7 rad) each independently split the
        trajectory 42mm from replay over a full episode (chaotic contact dynamics);
        with both raw the env path is bit-identical to replay_harness (12 solved
        demos regain their pick -- see baselines/diagnostics/trace_env_ablate.py)."""
        w = self.w
        action = np.asarray(action, dtype=np.float64)
        arm = np.asarray(arm_cmd, dtype=np.float64) if arm_cmd is not None \
            else action[:6]
        gm = float(grip_motor) if grip_motor is not None \
            else float(np.clip(action[6], 0.0, 1.0)) * 100.0
        grip = np.clip(gm, 0.0, 100.0) / 100.0   # metrics only (picked predicate)
        # Workspace constraint: ZERO OUT an action whose commanded joint target would
        # put the TOOL outside the teleop box -- i.e. hold the last accepted target, so
        # the arm does not move. Gripper is never blocked (it cannot leave the box).
        # Demo collection passes arm_cmd (raw tape) and is exempt: demos are inside the
        # box by construction and must replay bit-exactly.
        ws_blocked = False
        if self.workspace_limit and arm_cmd is None:
            # Reactive: if the arm is ALREADY outside the box, zero this action -- hold
            # the last target KNOWN to keep the tool inside, so the controller pulls
            # back. The cache is updated AFTER stepping (below) from the resulting
            # position: caching on the pre-step position would bless the very action
            # that leaves the box and then hold it there forever.
            # REACTIVE + RECOVERY. If the tool is outside the box, override the action
            # with the last joint configuration at which the tool was MEASURED inside.
            # Caching measured positions (not commanded targets) is what makes return
            # possible: a measured pose is reachable and verified in-box, whereas a
            # commanded target gets cached while the arm is still travelling toward it
            # and can be the very target that leaves the box.
            # Normal control resumes the moment the tool is back inside.
            if not self.in_workspace(self.tool_pos()):
                arm = self._last_valid_q.copy()     # drive back to a known-good pose
                ws_blocked = True
                self.ws_violations += 1
        w['kinova'].control_dofs_position(arm, dofs_idx_local=w['kdofs'][:6])
        w['kinova'].control_dofs_position(np.array(gripper_targets(gm)),
                                          dofs_idx_local=np.array(w['kdofs'][-4:]))
        for _ in range(3):
            w['scene'].step()
        if self.workspace_limit and arm_cmd is None and \
                self.in_workspace(self.tool_pos(), margin=WS_MARGIN):
            # measured configuration well INSIDE the box -> a decisive recovery target.
            # Requiring the margin here (not just "inside") stops the arm settling into
            # an equilibrium a couple of mm outside the face it exited through.
            self._last_valid_q = np_(
                w['kinova'].get_dofs_position(dofs_idx_local=w['kdofs'][:6])).copy()
        self._t += 1
        bp = np_(w['bottle'].get_pos())
        ee = np_(w['eef'].get_pos())
        # held-can guard, sustained (see PICK_EEF_DIST/PICK_SUSTAIN above): z +
        # commanded-grip alone was gamed by whack-flings (r2dreamer v4 audit).
        if bp[2] > w['pick_z'] and grip * 100.0 > GP_CLOSE \
                and float(np.linalg.norm(ee - bp)) < PICK_EEF_DIST:
            self._pick_run += 1
        else:
            self._pick_run = 0
        if self._pick_run >= PICK_SUSTAIN: self._picked = True
        if self._picked and in_shelf_footprint(bp) and \
           BOX_TOP_Z + 0.01 < bp[2] < BOX_TOP_Z + 0.07: self._placed = True
        # contact counts only if the can was actually PICKED first -- otherwise a can
        # shoved along the table into the goal's base registers as task success without
        # any pick/place/slide (confirmed on trial 284: contact at table level, z=0.10)
        c = np_(w['bottle'].get_contacts(w['goal'])['position'])
        bg_touch = bool(c.size and c.shape[0])
        # gripper<->goal solver contact, read on EVERY can<->goal contact frame (2026-09-10,
        # LADDER_UNIFY_BRIEF D1): the shared stage tracker needs the PER-FRAME value, and one
        # read in one place is the alternative to full_env re-reading the same solver state.
        # Previously this was read only inside the `picked and bg_touch` branch below; the
        # branch now consumes this value rather than re-reading it, so nothing double-reads.
        # Reads do not perturb the solver (established by the #26 trace ablation).
        gg_touch = False
        if bg_touch:
            _gg = np_(w['goal'].get_contacts(w['kinova'])['position'])
            gg_touch = bool(_gg.size and _gg.shape[0])
        if self._picked and bg_touch and \
           float(ee[0]) < float(bp[0]):
            if not self._contact:
                self._contact_frame = self._t
            self._contact = True
        # contact_push (2026-09-07, PHASE_PLAN amendment (g)): STRICTER contact, LOGGED ONLY -- `contact`
        # above is unchanged and stays the predicate of record. User: 'contact is ideally through a slide
        # where the gripper and the goal can are on opposite sides of the pick-can'. contact_push =
        #   picked (earlier in the episode)  AND  pick-can<->goal solver contact THIS step
        #   AND  dot(tool_xy - can_xy, goal_xy - can_xy) < 0   (the TOOL point -- tool_pos(), the gripper tip
        #        reconstructed from the wrist -- on the far side of the pick-can along the can->goal line, table
        #        plane). NOT the wrist link `ee` that `contact` uses: the wrist sits ~0.145 m behind the tool, so
        #        a wrist-based far-side test fires on 132/148 banked HELD states and `contact`'s own ee_x < can_x
        #        holds on 332/332 banked states (ADVERSARIAL_REVIEW_eval_env_2026-09-07 S2-5) -- it would grant
        #        on the pure carry the predicate exists to exclude.
        #   AND  no gripper<->goal solver contact THIS step  (goal.get_contacts(kinova) empty).
        # Sticky once true, like `contact`. Diagnostics (why a `contact` episode fails the stricter test):
        # contact_gripper_goal = any gripper-goal contact on a picked pick-can/goal contact frame;
        # contact_farside = any such frame with dot < 0. Also logged: the wrist-based dot sign (contact_farside_wrist)
        # so the two definitions can be compared post hoc.
        if self._picked and bg_touch:
            gp_ = np_(w['goal'].get_pos())
            tool = np.asarray(self.tool_pos(), dtype=np.float64)
            dot = float((tool[0] - bp[0]) * (gp_[0] - bp[0]) + (tool[1] - bp[1]) * (gp_[1] - bp[1]))
            dot_w = float((ee[0] - bp[0]) * (gp_[0] - bp[0]) + (ee[1] - bp[1]) * (gp_[1] - bp[1]))
            if dot_w < 0.0:
                self._contact_farside_wrist = True
            if gg_touch:
                self._contact_gripper_goal = True
            if dot < 0.0:
                self._contact_farside = True
            if dot < 0.0 and not gg_touch and not self._contact_push:
                self._contact_push = True
                self._contact_push_frame = self._t
        # --- slide_success (amendment (l)): all four clauses on THIS frame; sticky once sustained --------------
        self._last_grip_cmd = float(grip)
        slide_ok = bool(self._picked and bg_touch and float(grip) < GRIP_OPEN_CMD
                        and in_shelf_footprint(bp) and tilt_deg(np_(w['bottle'].get_quat())) < 20.0)
        self._slide_run = self._slide_run + 1 if slide_ok else 0
        if self._slide_run >= SLIDE_SUSTAIN and not self._slide_success:
            self._slide_success = True; self._slide_frame = self._t; self._slide_route = 'sustained'
        done = self._t >= self.max_steps
        info = dict(picked=self._picked, placed=self._placed, contact=self._contact,
                    # PER-FRAME solver contacts (2026-09-10): what the shared stage tracker
                    # consumes. Instantaneous, NOT sticky -- unlike `contact`/`contact_push`.
                    can_goal_touch=bool(bg_touch), gripper_goal_touch=bool(gg_touch),
                    contact_push=self._contact_push, contact_frame=self._contact_frame,
                    contact_push_frame=self._contact_push_frame,
                    contact_gripper_goal=self._contact_gripper_goal,
                    contact_farside=self._contact_farside,
                    contact_farside_wrist=self._contact_farside_wrist,
                    slide_success=self._slide_success, slide_ok=slide_ok, slide_run=self._slide_run,
                    slide_frame=self._slide_frame, slide_route=self._slide_route,
                    t=self._t, uid=self._uid, ws_blocked=ws_blocked,
                    ws_violations=self.ws_violations)
        if done:
            # ONE post-episode settle yields both the settled `nested` and slide_success's held window (amendment (l));
            # identical step count and identical nested measurement point as the old info['nested'] = self._nested().
            _e = self.end_of_episode()
            info['nested'] = _e['nested']
            info['slide_success'] = _e['slide_success']; info['slide_route'] = _e['slide_route']
            info['slide_fail_reason'] = _e['slide_fail_reason']; info['slide_fail_frame'] = _e['slide_fail_frame']
            info['end_of_episode'] = True
        return self._obs(), done, info

    def _nested(self):
        w = self.w
        for _ in range(100):
            w['scene'].step()
        bp = np_(w['bottle'].get_pos()); gp_ = np_(w['goal'].get_pos())
        touch = float(np.hypot(bp[0] - gp_[0], bp[1] - gp_[1])) <= NESTED_TOUCH_DIST
        # nested requires the can was actually picked (same precondition as contact) --
        # a shoved-in can that touches the goal upright is not a completed task.
        # touch is proximity-based (see replay_harness.NESTED_TOUCH_DIST): hard contact
        # tests fail human-validated placements by mm-level physics noise.
        return bool(self._picked and touch and tilt_deg(np_(w['bottle'].get_quat())) < 20
                    and tilt_deg(np_(w['goal'].get_quat())) < 20)

    def _slide_clauses(self):
        """amendment (l): the four slide_success clauses on the CURRENT world state, with the last COMMANDED grip
        (the controller target is unchanged through the settle, so the command still stands).
        Returns (ok, reason); reason names the FIRST clause that failed, so a zero column can be explained."""
        w = self.w
        if not self._picked:
            return False, 'not_picked'
        if self._last_grip_cmd is None or float(self._last_grip_cmd) >= GRIP_OPEN_CMD:
            return False, 'grip_closed'
        c = np_(w['bottle'].get_contacts(w['goal'])['position'])
        if not (c.size and c.shape[0]):
            return False, 'no_contact'
        bp = np_(w['bottle'].get_pos())
        if not (in_shelf_footprint(bp) and tilt_deg(np_(w['bottle'].get_quat())) < 20.0):
            return False, 'off_shelf_or_tilted'
        return True, None

    def end_of_episode(self):
        """The single post-episode settle (SETTLE_STEPS scene steps, last command held), yielding BOTH the honest
        settled `nested` (amendment (j)/S1-1) and slide_success's held window (amendment (l)).

        Why the window lives here: scope='contact' terminates on the first contact frame and scope='full' on the
        nested proxy, so an in-episode counter can never reach 3 decisions in the two scopes where slide_success is
        the statistic of record. The clauses are therefore required to hold continuously over the first
        SLIDE_SUSTAIN frames (3 scene steps each) of this continuation. The step BUDGET and the point at which
        nested is measured are unchanged (always SETTLE_STEPS steps before the nested read), so nested is bit-identical
        to the previous _nested() path; only extra state READS happen during the window (reads do not perturb the
        solver -- established by the #26 trace ablation)."""
        w = self.w
        slide, route, steps = bool(self._slide_success), self._slide_route, 0
        reason, fail_frame = None, None
        if not slide:
            held = True
            for f in range(SLIDE_SUSTAIN):
                for _ in range(3):
                    w['scene'].step()
                steps += 3
                ok, why = self._slide_clauses()
                if not ok:
                    held, reason, fail_frame = False, why, f
                    break
            if held:
                slide, route = True, 'settle'
                self._slide_success = True; self._slide_route = route; self._slide_frame = self._t
        for _ in range(SETTLE_STEPS - steps):
            w['scene'].step()
        bp = np_(w['bottle'].get_pos()); gp_ = np_(w['goal'].get_pos())
        touch = float(np.hypot(bp[0] - gp_[0], bp[1] - gp_[1])) <= NESTED_TOUCH_DIST
        nested = bool(self._picked and touch and tilt_deg(np_(w['bottle'].get_quat())) < 20
                      and tilt_deg(np_(w['goal'].get_quat())) < 20)
        return dict(nested=nested, slide_success=bool(slide), slide_route=route,
                    slide_fail_reason=reason, slide_fail_frame=fail_frame)

    def _obs(self):
        w = self.w
        q = np_(w['kinova'].get_dofs_position(dofs_idx_local=w['kdofs']))
        # gripper motor 0..1 from driver joint angle (invert of gripper_targets)
        theta = float(q[7])
        motor = 1.0 - (theta - (-0.09)) / (0.96 - (-0.09))
        # grip effort: |applied control force| on the bottom finger drivers -- the sim
        # analog of the real gripper's motor current (the grasp-contact 'feel' signal)
        bots = sorted(int(x) for x in w['kdofs'][-4:-2])
        try:
            grip_effort = float(np.abs(np_(w['kinova'].get_dofs_control_force(
                dofs_idx_local=bots))).sum())
        except Exception:
            grip_effort = 0.0
        bp = np_(w['bottle'].get_pos()); bq = np_(w['bottle'].get_quat())
        gp_ = np_(w['goal'].get_pos())
        state = np.concatenate([q[:6], [np.clip(motor, 0, 1)], [grip_effort],
                                bp, bq, gp_[:2]]).astype(np.float32)
        obs = dict(state=state)
        if self.render_size is not None and w.get('cam') is not None:
            rgb = np.asarray(w['cam'].render()[0]).astype(np.uint8)
            if rgb.shape[:2] != tuple(self.render_size):
                import cv2
                rgb = cv2.resize(rgb, (self.render_size[1], self.render_size[0]))
            obs['image'] = rgb
        return obs
