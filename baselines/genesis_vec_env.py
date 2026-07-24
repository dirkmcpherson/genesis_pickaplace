"""BatchedCanWorld: N pick-and-place envs in ONE genesis scene (scene.build(n_envs=N)).

The batched counterpart of GenesisCanEnv + FullTaskEnv, for DreamerV3 collection.
Design + probe evidence in BATCHED_ENV_PLAN.md §8-9. Key facts this code relies on:

  - every setter takes envs_idx; eef.get_pos() etc. return (N, ...) [probed]
  - per-env reset leaves other envs within ~1.2mm of unperturbed (GPU atomics noise,
    same magnitude as baseline repeatability) [probed]
  - env_separate_rigid=True: static camera renders (N,H,W,3) in ONE call; the wrist
    camera cannot attach (engine raises) so it is posed per env from the batched link
    pose and row i of the stack is taken (strategy S1) [probed]
  - resets must COMMAND the reset pose, not just set it: a scene.step under stale
    control targets is history-dependent (the probe's false-nondeterminism bug)
  - env_spacing is visual-only; physics runs in the same local frame as the
    validated single-env world [probed]

Honesty contract (same split as full_env.py): TRAINING uses per-step geometric
proxies (contact = picked & center-dist & eef-behind, nested-proxy = contact &
upright & gripper open). The honest settled metrics live in the fresh-process
single-env CPU eval (genesis_eval.py) so reported numbers stay comparable to
DP/SACfD. Do NOT quote this class's stage flags as results.

GPU-backend physics: a DIFFERENT numeric realization than the validated CPU path
(historical transfer cliff). Run the bridge experiment (replay demo subset here,
compare stage counts to CPU) before comparing any numbers across the two.
"""
import os
import json, sys, pathlib as pl

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', '/home/j/workspace/genesis_pickaplace'))
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / 'can_pos_recovery'))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))


def np_(x):
    try:
        return x.detach().cpu().numpy()
    except AttributeError:
        return np.asarray(x)


class BatchedCanWorld:
    """One genesis scene, N envs, staged-reward pick-and-place semantics per env."""

    STAGE_REWARD = dict(picked=1.0, placed=1.0, contact=2.0, nested=4.0)

    def __init__(self, n_envs, size=(64, 64), pixels=True, workspace_limit=False,
                 max_steps=1200, seed=0):
        import genesis as gs
        from kinova import JOINT_NAMES, EEF_NAME
        from replay_harness import (BOX_POS, BOX_SIZE, STATIC_BOTTLE_POSITION,
                                    HARDCODED_START, joint_dofs)
        from genesis_can_env import WS_LO, WS_HI, WS_MARGIN, REF_TOOL_AT_START, _quat_to_R

        table = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())
        wcfg = table['world']
        self.placements = {int(u): r for u, r in table['trials'].items()
                           if r['status'] in ('ok', 'ok_batch')}
        self.success_uids = sorted(u for u, r in self.placements.items()
                                   if r.get('label') == 'success')
        self.n = int(n_envs)
        self.size = tuple(size)
        self.pixels = bool(pixels)
        self.workspace_limit = bool(workspace_limit)
        self.max_steps = int(max_steps)
        self.rng = np.random.RandomState(seed)
        self._start_q = np.array(HARDCODED_START)
        self._static_goal = STATIC_BOTTLE_POSITION

        gs.init(backend=gs.gpu, seed=0, precision="32", logging_level="warning")
        scene = gs.Scene(
            show_viewer=False,
            sim_options=gs.options.SimOptions(dt=0.01, substeps=wcfg.get('substeps', 8)),
            vis_options=gs.options.VisOptions(env_separate_rigid=True,
                                              show_world_frame=False),
        )
        scene.add_entity(gs.morphs.Plane())
        scene.add_entity(material=gs.materials.Rigid(rho=1000, friction=0.5),
                         morph=gs.morphs.Box(size=BOX_SIZE, pos=BOX_POS))
        scene.add_entity(material=gs.materials.Rigid(rho=1000, friction=0.5),
                         morph=gs.morphs.Box(size=(0.419, 1.2, 0.05),
                                             pos=(0.3395, -0.1875, 0.025), fixed=True))
        self.kinova = scene.add_entity(
            gs.morphs.URDF(file=str(REPO / 'gen3_lite_2f_robotiq_85.urdf'),
                           fixed=True, pos=(0.0, 0.0, 0.05)))
        can_r = wcfg.get('can_radius', 0.033); can_h = wcfg['can_height']
        self.bottle = scene.add_entity(
            material=gs.materials.Rigid(rho=wcfg['can_rho'], friction=0.2),
            morph=gs.morphs.Cylinder(pos=(0.4381, 0.1, 0.113), radius=can_r, height=can_h))
        self.goal = scene.add_entity(
            material=gs.materials.Rigid(rho=1000, friction=2.0),
            morph=gs.morphs.Cylinder(pos=STATIC_BOTTLE_POSITION, radius=can_r, height=can_h))
        if self.pixels:
            self.cam_top = scene.add_camera(res=self.size, pos=(0.40, -0.08, 1.15),
                                            lookat=(0.40, -0.08, 0.10), up=(1, 0, 0),
                                            fov=68, GUI=False)
            self.cam_wrist = scene.add_camera(res=self.size, pos=(0, 0, 1.0),
                                              lookat=(0, 0, 0), fov=80, GUI=False)
        scene.build(n_envs=self.n)
        self.scene = scene
        self.kdofs = [joint_dofs(self.kinova.get_joint(nm)) for nm in JOINT_NAMES]
        self.eef = self.kinova.get_link(EEF_NAME)

        # validated gains (build_batched_world convention, world-cfg values)
        fkp = [wcfg['finger_kp']] * 4
        self.kinova.set_dofs_kp(kp=np.array([200, 200, 150, 100, 60, 60] + fkp),
                                dofs_idx_local=self.kdofs)
        self.kinova.set_dofs_kv(kv=np.array([20, 20, 15, 10, 6, 6] + [10] * 4),
                                dofs_idx_local=self.kdofs)
        ff = [wcfg['finger_force']] * 4
        self.kinova.set_dofs_force_range(
            lower=np.array([-87, -87, -52, -30, -14, -14] + [-f for f in ff]),
            upper=np.array([87, 87, 52, 30, 14, 14] + ff),
            dofs_idx_local=self.kdofs)

        from replay_harness import (BOX_TOP_Z, GP_CLOSE, NESTED_TOUCH_DIST, PICK_Z,
                                    gripper_targets, TABLE_TOP_Z)
        self._grip_targets = gripper_targets
        self.GP_CLOSE = GP_CLOSE
        self.BOX_TOP_Z = BOX_TOP_Z
        self.NESTED_TOUCH = NESTED_TOUCH_DIST
        self.pick_z = TABLE_TOP_Z + can_h / 2 + 0.05      # GenesisCanEnv w['pick_z']
        self.can_start_z = TABLE_TOP_Z + can_h / 2 + 0.0125

        # per-env episode state
        self.t = np.zeros(self.n, dtype=np.int64)
        self.picked = np.zeros(self.n, dtype=bool)
        self.placed = np.zeros(self.n, dtype=bool)
        self.contact = np.zeros(self.n, dtype=bool)
        self.granted = np.zeros((self.n, 4), dtype=bool)   # picked/placed/contact/nested
        self.ws_violations = np.zeros(self.n, dtype=np.int64)
        self._last_valid_q = np.tile(self._start_q[:6], (self.n, 1))
        # workspace tool offset (constant, from the calibration reference)
        self._ws_lo, self._ws_hi, self._ws_margin = WS_LO, WS_HI, WS_MARGIN
        self._ref_tool = REF_TOOL_AT_START
        self._quat_to_R = _quat_to_R
        self._tool_offset = None
        import genesis.utils.geom as gu
        self._gu = gu
        self._wristT = np.eye(4)
        # top-of-wrist mount (user-selected): +x 10cm up, 3cm back, 30deg down, -90 roll
        _z = np.array([0.0, 0.0, 1.0]); _up = np.array([0.0, 1.0, 0.0])
        _x = np.cross(_up, -_z); _x /= np.linalg.norm(_x)
        _y = np.cross(-_z, _x)
        _R = np.stack([_x, _y, -_z], axis=1)
        _a = np.deg2rad(30.0); _c, _s = np.cos(_a), np.sin(_a)
        _pitch = np.array([[_c, 0, -_s], [0, 1, 0], [_s, 0, _c]])
        _r = np.deg2rad(90.0); _rc, _rs = np.cos(_r), np.sin(_r)
        _roll = np.array([[_rc, -_rs, 0], [_rs, _rc, 0], [0, 0, 1]])
        self._wristT[:3, :3] = _R @ _pitch @ _roll
        self._wristT[:3, 3] = (0.10, 0.0, -0.03)

    # ---------------- reset ----------------
    def reset_envs(self, idx, uids=None):
        """Reset the given env indices to success-demo ICs (random unless `uids`
        pins them, e.g. for the bridge experiment). NO scene step (a global step
        would advance other envs mid-episode); state is set exactly and control
        targets are COMMANDED so the next global step is deterministic w.r.t. this
        reset (stale-target lesson, plan §8)."""
        idx = np.atleast_1d(np.asarray(idx, dtype=np.int64))
        k = len(idx)
        if uids is None:
            uids = [int(self.rng.choice(self.success_uids)) for _ in range(k)]
        can_pos = np.array([self.placements[u]['can_pos'] for u in uids])
        can_quat = np.array([self.placements[u].get('can_quat') or [1, 0, 0, 0]
                             for u in uids], dtype=np.float64)
        q = np.tile(self._start_q, (k, 1))
        self.kinova.set_dofs_position(q, self.kdofs, envs_idx=idx)
        self.kinova.control_dofs_position(q, dofs_idx_local=self.kdofs, envs_idx=idx)
        self.kinova.zero_all_dofs_velocity(envs_idx=idx)
        self.bottle.set_pos(can_pos, envs_idx=idx)
        self.bottle.set_quat(can_quat, envs_idx=idx)
        self.bottle.zero_all_dofs_velocity(envs_idx=idx)
        gz = self.BOX_TOP_Z + 0.101 / 2 + 0.0425
        self.goal.set_pos(np.tile((self._static_goal[0], self._static_goal[1], gz),
                                  (k, 1)), envs_idx=idx)
        self.goal.set_quat(np.tile((1.0, 0, 0, 0), (k, 1)), envs_idx=idx)
        self.goal.zero_all_dofs_velocity(envs_idx=idx)
        self.t[idx] = 0
        self.picked[idx] = self.placed[idx] = self.contact[idx] = False
        self.granted[idx] = False
        self.ws_violations[idx] = 0
        self._last_valid_q[idx] = self._start_q[:6]

    # ---------------- step ----------------
    def step_batch(self, actions):
        """actions: (N,7) normalized [-1,1]. One global step for ALL envs.
        Returns state (N,17), reward (N,), terminated (N,), info dict of arrays."""
        from pick_env import denormalize_action
        a_phys = denormalize_action(np.asarray(actions))
        arm = a_phys[:, :6].astype(np.float64)
        grip = np.clip(a_phys[:, 6], 0.0, 1.0)

        if self.workspace_limit:
            tool = self._tool_pos_batch()
            inside = np.all((tool >= self._ws_lo) & (tool <= self._ws_hi), axis=1)
            blocked = ~inside
            arm[blocked] = self._last_valid_q[blocked]
            self.ws_violations += blocked
            deep = np.all((tool >= self._ws_lo + self._ws_margin) &
                          (tool <= self._ws_hi - self._ws_margin), axis=1)
        else:
            blocked = np.zeros(self.n, dtype=bool); deep = None

        self.kinova.control_dofs_position(arm, dofs_idx_local=self.kdofs[:6])
        gt = np.stack([self._grip_targets(g * 100.0) for g in grip])
        self.kinova.control_dofs_position(gt, dofs_idx_local=np.array(self.kdofs[-4:]))
        for _ in range(3):
            self.scene.step()
        self.t += 1

        bp = np_(self.bottle.get_pos())            # (N,3)
        bq = np_(self.bottle.get_quat())           # (N,4)
        gp_ = np_(self.goal.get_pos())
        gq = np_(self.goal.get_quat())
        qpos = np_(self.kinova.get_dofs_position(dofs_idx_local=self.kdofs))  # (N,10)
        eefp = np_(self.eef.get_pos())

        if self.workspace_limit and deep is not None:
            # measured-inside poses become recovery targets (post-step, plan §8)
            self._last_valid_q[deep] = qpos[deep, :6]

        # ---- predicates (training proxies; see module docstring) ----
        new_picked = (bp[:, 2] > self.pick_z) & (grip * 100.0 > self.GP_CLOSE)
        self.picked |= new_picked
        # exact in_shelf_footprint, vectorized: BOX_POS (0.75,-0.1875), BOX_SIZE (0.4,0.75)
        in_shelf = ((np.abs(bp[:, 0] - 0.75) < 0.2) &
                    (np.abs(bp[:, 1] - (-0.1875)) < 0.375))
        self.placed |= (self.picked & in_shelf &
                        (bp[:, 2] > self.BOX_TOP_Z + 0.01) & (bp[:, 2] < self.BOX_TOP_Z + 0.07))
        dxy = np.hypot(bp[:, 0] - gp_[:, 0], bp[:, 1] - gp_[:, 1])
        eef_behind = eefp[:, 0] < bp[:, 0]
        self.contact |= (self.picked & (dxy <= 0.070) & eef_behind)

        def tilt_ok(q):
            w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
            cz = 1.0 - 2.0 * (x * x + y * y)
            return np.degrees(np.arccos(np.clip(cz, -1.0, 1.0))) < 20.0
        nested_now = self.contact & (grip < 0.3) & tilt_ok(bq) & tilt_ok(gq)

        # staged first-time rewards
        stages = np.stack([self.picked, self.placed, self.contact, nested_now], axis=1)
        newly = stages & ~self.granted
        self.granted |= stages
        rew_w = np.array([self.STAGE_REWARD['picked'], self.STAGE_REWARD['placed'],
                          self.STAGE_REWARD['contact'], self.STAGE_REWARD['nested']])
        reward = (newly * rew_w).sum(axis=1).astype(np.float32)
        terminated = nested_now.copy()
        truncated = (~terminated) & (self.t >= self.max_steps)

        # ---- 17-dim state (GenesisCanEnv._obs layout) ----
        theta = qpos[:, 7]
        motor = np.clip(1.0 - (theta - (-0.09)) / (0.96 - (-0.09)), 0, 1)
        bots = sorted(int(x) for x in self.kdofs[-4:-2])
        try:
            effort = np.abs(np_(self.kinova.get_dofs_control_force(
                dofs_idx_local=bots))).sum(axis=-1)
        except Exception:
            effort = np.zeros(self.n)
        state = np.concatenate([
            qpos[:, :6], motor[:, None], effort[:, None], bp, bq,
            np.tile(self._static_goal[:2], (self.n, 1))], axis=1).astype(np.float32)

        info = dict(picked=self.picked.copy(), placed=self.placed.copy(),
                    contact=self.contact.copy(), nested=nested_now,
                    ws_blocked=blocked, t=self.t.copy())
        return state, reward, terminated, truncated, info

    # ---------------- obs ----------------
    def _tool_pos_batch(self):
        eefp = np_(self.eef.get_pos()); eefq = np_(self.eef.get_quat())
        if self._tool_offset is None:
            # all envs identical at start; calibrate once from env 0 at the start pose
            R0 = self._quat_to_R(eefq[0])
            self._tool_offset = R0.T @ (self._ref_tool - eefp[0])
        out = np.empty((self.n, 3))
        for i in range(self.n):
            out[i] = eefp[i] + self._quat_to_R(eefq[i]) @ self._tool_offset
        return out

    def render_batch(self):
        """(N,64,64,6) uint8: batched top render ++ batched PER-ENV-POSE wrist render.

        The wrist uses the per-env camera-pose renderer patch (third_party/
        genesis-per-env-camera.patch): the camera node carries env_poses (N,4,4) and
        env i's ENV_SEPARATE pass views from pose i. Validated BIT-IDENTICAL to the
        serial set_pose ground truth and ~227x faster (2.1ms vs 468ms at N=8)."""
        assert self.pixels
        top = np.asarray(self.cam_top.render()[0], dtype=np.uint8)      # (N,H,W,3)
        if top.ndim == 3:
            top = top[None]
        eefp = np_(self.eef.get_pos()); eefq = np_(self.eef.get_quat())
        poses = np.stack([self._gu.trans_quat_to_T(eefp[i], eefq[i]) @ self._wristT
                          for i in range(self.n)])
        node = self.cam_wrist._rasterizer._camera_nodes[self.cam_wrist.uid]
        node.env_poses = poses
        try:
            wrist = np.asarray(self.cam_wrist.render()[0], dtype=np.uint8)
        finally:
            node.env_poses = None
        if wrist.ndim == 3:
            wrist = wrist[None]
        return np.concatenate([top, wrist], axis=-1)

    # ---------------- raw-tape replay (bridge experiment) ----------------
    def replay_step_batch(self, arm_raw, gp_raw):
        """Collector-faithful stepping for demo tapes: RAW float64 arm targets and
        RAW gripper motor values (may overshoot [0,100]) -- the exact-command lesson
        from the single-env replay work. Updates the same stage predicates as
        step_batch. arm_raw (N,6) float64, gp_raw (N,) float."""
        self.kinova.control_dofs_position(np.asarray(arm_raw, dtype=np.float64),
                                          dofs_idx_local=self.kdofs[:6])
        gt = np.stack([self._grip_targets(g) for g in np.asarray(gp_raw, dtype=np.float64)])
        self.kinova.control_dofs_position(gt, dofs_idx_local=np.array(self.kdofs[-4:]))
        for _ in range(3):
            self.scene.step()
        self.t += 1
        bp = np_(self.bottle.get_pos()); gp_ = np_(self.goal.get_pos())
        eefp = np_(self.eef.get_pos())
        grip100 = np.asarray(gp_raw, dtype=np.float64)
        self.picked |= (bp[:, 2] > self.pick_z) & (grip100 > self.GP_CLOSE)
        in_shelf = ((np.abs(bp[:, 0] - 0.75) < 0.2) &
                    (np.abs(bp[:, 1] - (-0.1875)) < 0.375))
        self.placed |= (self.picked & in_shelf &
                        (bp[:, 2] > self.BOX_TOP_Z + 0.01) & (bp[:, 2] < self.BOX_TOP_Z + 0.07))
        dxy = np.hypot(bp[:, 0] - gp_[:, 0], bp[:, 1] - gp_[:, 1])
        self.contact |= (self.picked & (dxy <= 0.070) & (eefp[:, 0] < bp[:, 0]))

    def stages_after_settle(self, settle_steps=100):
        """Hold current targets, settle, score proximity-nested (collector scoring)."""
        for _ in range(settle_steps):
            self.scene.step()
        bp = np_(self.bottle.get_pos()); gp_ = np_(self.goal.get_pos())
        bq = np_(self.bottle.get_quat()); gq = np_(self.goal.get_quat())
        def tilt_ok(q):
            w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
            return np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1, 1))) < 20
        touch = np.hypot(bp[:, 0] - gp_[:, 0], bp[:, 1] - gp_[:, 1]) <= self.NESTED_TOUCH
        nested = self.picked & touch & tilt_ok(bq) & tilt_ok(gq)
        stage = np.where(nested, 'nested',
                 np.where(self.contact, 'contact',
                  np.where(self.placed, 'placed',
                   np.where(self.picked, 'picked', 'no-pick'))))
        return stage
