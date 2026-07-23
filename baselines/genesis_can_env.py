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

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', '/home/j/workspace/genesis_pickaplace'))
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
        w['bottle'].set_pos(can_pos); w['bottle'].set_quat(list(can_quat or [1, 0, 0, 0]))
        w['goal'].set_pos(goal_pos); w['goal'].set_quat([1, 0, 0, 0])
        for ent in (w['bottle'], w['goal']):
            try: ent.zero_all_dofs_velocity()
            except Exception: pass
        w['scene'].step()
        self._t = 0
        self._picked = self._placed = self._contact = False
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
        if self.workspace_limit:
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
        if bp[2] > w['pick_z'] and grip * 100.0 > GP_CLOSE: self._picked = True
        if self._picked and in_shelf_footprint(bp) and \
           BOX_TOP_Z + 0.01 < bp[2] < BOX_TOP_Z + 0.07: self._placed = True
        # contact counts only if the can was actually PICKED first -- otherwise a can
        # shoved along the table into the goal's base registers as task success without
        # any pick/place/slide (confirmed on trial 284: contact at table level, z=0.10)
        c = np_(w['bottle'].get_contacts(w['goal'])['position'])
        if self._picked and (c.size and c.shape[0]) and \
           float(np_(w['eef'].get_pos())[0]) < float(bp[0]):
            self._contact = True
        done = self._t >= self.max_steps
        info = dict(picked=self._picked, placed=self._placed, contact=self._contact,
                    t=self._t, uid=self._uid, ws_blocked=ws_blocked,
                    ws_violations=self.ws_violations)
        if done:
            info['nested'] = self._nested()
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
