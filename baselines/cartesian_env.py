"""4-DOF Cartesian (end-effector velocity) action wrapper around GenesisCanEnv.

The demos were collected by CARTESIAN-velocity joystick teleop (see trial_reader + the
gen3_lite plugin): the human commanded ee velocity in [x, y, z, pitch] (roll/yaw fixed),
capped at 0.11 m/s, within an absolute base-frame workspace box. This wrapper exposes that
same action space so policies can learn in the demonstrations' NATIVE modality (4 action
dims vs 7 joint dims), instead of the joint-position space GenesisCanEnv uses natively.

action = [vx, vy, vz, v_pitch, gripper]
  * vx,vy,vz  : ee linear velocity (m/s), clamped to +-VCAP (0.11, the plugin's cap)
  * v_pitch   : ee pitch rate (rad/s) about the base Y axis
  * gripper   : 0..1 (same as GenesisCanEnv)
Internally integrates an ee setpoint (clamped to the teleop workspace), solves IK each step
for the 6 arm joints, and delegates to GenesisCanEnv.step (joint-position targets + PD).

_pose_step(pos, quat) exposes absolute ee-pose control too -- used to validate the IK
pipeline by replaying a demo's FK ee-pose trajectory.
"""
import os
import sys
import pathlib as pl

import numpy as np
import torch
from scipy.spatial.transform import Rotation as R

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'can_pos_recovery'))
from genesis_can_env import GenesisCanEnv  # noqa: E402


def np_(x):
    return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)


def _gs_to_xyzw(q):  # genesis [w,x,y,z] -> scipy [x,y,z,w]
    q = np.asarray(q, float)
    return np.array([q[1], q[2], q[3], q[0]])


def _xyzw_to_gs(q):  # scipy [x,y,z,w] -> genesis [w,x,y,z]
    q = np.asarray(q, float)
    return np.array([q[3], q[0], q[1], q[2]])


class CartesianCanEnv:
    VCAP = 0.11                                    # plugin VELOCITY_CAP (m/s)
    WS = (np.array([0.3, -0.25, 0.015]), np.array([0.8, 0.25, 0.6]))  # base-frame ee box
    DT = 0.025                                     # demo frame period (cmd_vel integrates to tool_pose at this dt)
    # real tool_pose at HARDCODED_START (~const across demos) -- used to calibrate the fixed
    # wrist(eef link) -> tool(gripper tip) offset, since Genesis merges the URDF tool_frame link.
    REF_TOOL_AT_START = np.array([0.367, 0.011, 0.09])

    PITCH_CAP = 1.0                                # plugin pitch-rate cap (rad/s, measured max |wy| over demos)
    # Pitch SETPOINT clamp. The integrator was unbounded -- RL wound the wrist through
    # full revolutions (a sim affordance the hardware lacks; humans centered the stick
    # so demos never exposed it). Demo envelope [-0.55,+0.30] rad; clamp at 2x demo
    # span per the joint-limit convention (headroom for alternate paths).
    PITCH_RANGE = (-1.1, 0.6)

    # Delta caps ~3.6x the max per-step demo motion (VCAP*DT=2.75mm). Cap==demo-max
    # left zero tracking authority: delta-on-measured has no feed-forward (unlike the
    # velocity integrator, whose setpoint LEADS the arm), so the arm moved at the
    # fraction of 2.75mm PD covers per step -- open-loop under-travel, census 0/5.
    # Larger caps let a closed-loop policy servo at full demo speed.
    DCAP = 0.01                               # m/step
    DPITCH_CAP = 0.075                        # rad/step
    # Setpoint leash (delta mode): deltas INTEGRATE into a setpoint (feed-forward --
    # the PD needs ~20mm of lead to track at demo speed; trace showed pure
    # delta-on-measured stalls at ~20% speed with 9-20mm lag), but the setpoint is
    # clamped to within LEASH of the MEASURED pose, so imitation errors cannot
    # accumulate beyond it (the velocity integrator's failure). Anti-windup impedance.
    LEASH = 0.025                             # m
    LEASH_PITCH = 0.15                        # rad

    # 6-DOF absolute pose: [x,y,z, rotvec3 (rel. to the reset orientation), grip].
    #
    # WHY (corrected 2026-07-29): the real teleop commanded only 4 DOF -- verified,
    # recorded wx=wz=0 in 25/25 demos, pitch (wy) nonzero in 25/25 -- but its
    # Jacobian controller left roll/yaw FREE, so they drifted through the null space
    # during demonstration (measured up to 1.03/1.34 rad). Our env instead PINS
    # roll/yaw at their reset values, so the sim wrist cannot reach the orientations
    # the demos grasped from (median 8.2deg, worst 88.8deg apart) and grasps fail.
    # The command space was never insufficient; we constrained two DOF the real
    # system left free. abs6 gives the policy explicit control of them -- which is
    # also hardware-realizable (the gen3-lite accepts cartesian pose commands) and
    # more reproducible than imitating null-space drift. FK-equivalent to absolute
    # joint targets, which is why joint BC reaches 0.67 in-distribution while every
    # roll/yaw-pinned cartesian encoding sits at 0.00-0.07.
    ROTVEC_CAP = 1.6                           # rad per axis (~2x the demo max 1.34)
    DROT_CAP = 0.06                            # per-step rotation delta (3.6x the demo
                                               # median-of-max 0.016, matching DCAP's
                                               # tracking-authority convention)

    @classmethod
    def denormalize_delta6(cls, a):
        """[-1,1]^7 -> per-step [dx,dy,dz, drotvec3, grip]. 6-DOF DIFFERENTIAL:
        isolates 'absolute vs relative' from 'orientation control' (the 4-DOF
        variants confounded them by pinning roll/yaw)."""
        a = np.asarray(a, float)
        return np.concatenate([np.clip(a[..., :3], -1, 1) * cls.DCAP,
                               np.clip(a[..., 3:6], -1, 1) * cls.DROT_CAP,
                               (a[..., 6:7] + 1.0) / 2.0], axis=-1)

    @classmethod
    def normalize_delta6(cls, a):
        a = np.asarray(a, float)
        return np.clip(np.concatenate([a[..., :3] / cls.DCAP,
                                       a[..., 3:6] / cls.DROT_CAP,
                                       a[..., 6:7] * 2.0 - 1.0], axis=-1), -1.0, 1.0)

    @classmethod
    def denormalize_abs6(cls, a):
        a = np.asarray(a, float)
        lo, hi = cls.WS
        pos = lo + (np.clip(a[..., :3], -1, 1) + 1.0) * 0.5 * (hi - lo)
        rv = np.clip(a[..., 3:6], -1, 1) * cls.ROTVEC_CAP
        grip = (a[..., 6:7] + 1.0) / 2.0
        return np.concatenate([pos, rv, grip], axis=-1)

    @classmethod
    def normalize_abs6(cls, a):
        a = np.asarray(a, float)
        lo, hi = cls.WS
        pos = 2.0 * (a[..., :3] - lo) / (hi - lo) - 1.0
        rv = a[..., 3:6] / cls.ROTVEC_CAP
        return np.clip(np.concatenate([pos, rv, a[..., 6:7] * 2.0 - 1.0], axis=-1),
                       -1.0, 1.0)

    @classmethod
    def denormalize_abs(cls, a):
        """[-1,1]^5 -> absolute tool pose [x,y,z] in the workspace box, pitch, grip."""
        a = np.asarray(a, float)
        lo, hi = cls.WS
        pos = lo + (np.clip(a[:3], -1, 1) + 1.0) * 0.5 * (hi - lo)
        pit = cls.PITCH_RANGE[0] + (np.clip(a[3], -1, 1) + 1.0) * 0.5 * (
            cls.PITCH_RANGE[1] - cls.PITCH_RANGE[0])
        return np.concatenate([pos, [pit], [(a[4] + 1.0) / 2.0]])

    @classmethod
    def normalize_abs(cls, a):
        a = np.asarray(a, float)
        lo, hi = cls.WS
        pos = 2.0 * (a[..., :3] - lo) / (hi - lo) - 1.0
        pit = 2.0 * (a[..., 3:4] - cls.PITCH_RANGE[0]) / (
            cls.PITCH_RANGE[1] - cls.PITCH_RANGE[0]) - 1.0
        return np.clip(np.concatenate([pos, pit, a[..., 4:5] * 2.0 - 1.0], axis=-1),
                       -1.0, 1.0)

    def __init__(self, backend='cpu', render_size=None, max_steps=1200, camera_rig=False,
                 control='vel'):
        # control: 'vel'   -- ee-velocity, integrated SETPOINT (teleop-faithful, but a
        #                     hidden integrator: imitation errors accumulate -> BC drifts
        #                     OOD; DP 0.067/ACT 0.00 on random ICs).
        #          'delta' -- ee-position DELTA applied to the MEASURED tool pose each
        #                     step (target = clamp(measured + delta)): the reference is
        #                     the actual robot state, so errors self-correct like joint
        #                     position targets. Field-standard EEF encoding.
        self.control = control
        self.env = GenesisCanEnv(backend=backend, render_size=render_size,
                                 max_steps=max_steps, camera_rig=camera_rig)
        self.w = self.env.w
        self.arm = self.env.w['kdofs'][:6]
        self.eef = self.env.w['eef']
        self.kin = self.env.w['kinova']
        self.render_size = render_size
        self._offset_local = None   # fixed wrist->tool offset in the wrist frame (cached at first reset)

    # delegate the attrs ic_sampling / eval_core reach for
    @property
    def placements(self): return self.env.placements
    @property
    def solved_uids(self): return self.env.solved_uids
    @property
    def world_cfg(self): return self.env.world_cfg
    @property
    def max_steps(self): return self.env.max_steps
    def rig_obs(self): return self.env.rig_obs()

    @classmethod
    def denormalize_delta(cls, a):
        a = np.asarray(a, float)
        return np.concatenate([a[:3] * cls.DCAP, [a[3] * cls.DPITCH_CAP],
                               [(a[4] + 1.0) / 2.0]])

    @classmethod
    def normalize_delta(cls, a):
        a = np.asarray(a, float)
        out = np.concatenate([a[..., :3] / cls.DCAP, a[..., 3:4] / cls.DPITCH_CAP,
                              a[..., 4:5] * 2.0 - 1.0], axis=-1)
        return np.clip(out, -1.0, 1.0)

    # normalized [-1,1]^5 <-> physical [vx,vy,vz (m/s), v_pitch (rad/s), grip 0..1]
    @classmethod
    def denormalize_action(cls, a):
        a = np.asarray(a, float)
        return np.concatenate([a[:3] * cls.VCAP, [a[3] * cls.PITCH_CAP],
                               [(a[4] + 1.0) / 2.0]])

    @classmethod
    def normalize_action(cls, a):
        a = np.asarray(a, float)
        out = np.concatenate([a[..., :3] / cls.VCAP, a[..., 3:4] / cls.PITCH_CAP,
                              a[..., 4:5] * 2.0 - 1.0], axis=-1)
        return np.clip(out, -1.0, 1.0)

    # --- absolute ee-pose control (IK core) ---
    def _pose_step(self, pos, quat):
        qpos = self.kin.inverse_kinematics(
            link=self.eef, pos=np.asarray(pos, float), quat=np.asarray(quat, float),
            dofs_idx_local=self.arm, max_samples=1, max_solver_iters=15)
        joints = np_(qpos)[:6]
        return joints

    def _tool_pos(self):
        """Current tool (gripper-tip) position = wrist + R_wrist @ offset_local."""
        wp = np_(self.eef.get_pos()); wq = np_(self.eef.get_quat())
        return wp + R.from_quat(_gs_to_xyzw(wq)).apply(self._offset_local)

    def reset(self, **kw):
        obs = self.env.reset(**kw)
        wp = np_(self.eef.get_pos()).astype(float)
        wq = np_(self.eef.get_quat()).astype(float)
        if self._offset_local is None:
            # calibrate the fixed wrist->tool offset from the reset alignment (constant geometry)
            self._offset_local = R.from_quat(_gs_to_xyzw(wq)).inv().apply(self.REF_TOOL_AT_START - wp)
        self._sp = self._tool_pos()                # TOOL setpoint (not wrist)
        self._q0 = wq                              # fixed roll/yaw base orientation (wrist==tool orientation)
        self._pitch = 0.0
        self._rotvec = (np.zeros(3) if self.control in ('abs6', 'delta6') else None)
        self._grip = 0.0
        return self._obs(obs)

    def _measured_pitch(self):
        """Realized pitch of the wrist relative to the reset orientation q0."""
        wq = np_(self.eef.get_quat())
        rel = (R.from_quat(_gs_to_xyzw(wq)) * R.from_quat(_gs_to_xyzw(self._q0)).inv())
        return float(rel.as_rotvec()[1])

    def _target_quat(self):
        rv = (self._rotvec if getattr(self, '_rotvec', None) is not None
              else [0.0, self._pitch, 0.0])
        r = R.from_rotvec(rv) * R.from_quat(_gs_to_xyzw(self._q0))
        return _xyzw_to_gs(r.as_quat())

    def step(self, action):
        a = np.asarray(action, float)
        if self.control == 'delta6':
            # integrate position AND orientation deltas, each leashed to the measured
            # pose so imitation error cannot run away (feed-forward + bounded drift)
            d = np.clip(a[:3], -self.DCAP, self.DCAP)
            sp = np.clip(self._sp + d, self.WS[0], self.WS[1])
            cur = self._tool_pos()
            self._sp = cur + np.clip(sp - cur, -self.LEASH, self.LEASH)
            drv = np.clip(np.asarray(a[3:6], float), -self.DROT_CAP, self.DROT_CAP)
            tgt_rv = np.clip(np.asarray(self._rotvec) + drv,
                             -self.ROTVEC_CAP, self.ROTVEC_CAP)
            meas_rv = (R.from_quat(_gs_to_xyzw(np_(self.eef.get_quat())))
                       * R.from_quat(_gs_to_xyzw(self._q0)).inv()).as_rotvec()
            self._rotvec = meas_rv + np.clip(tgt_rv - meas_rv,
                                             -self.LEASH_PITCH, self.LEASH_PITCH)
            self._grip = float(np.clip(a[6], 0.0, 1.0))
            tgt_quat = self._target_quat()
            Rw = R.from_quat(_gs_to_xyzw(np_(self.eef.get_quat())))
            wrist_target = self._sp - Rw.apply(self._offset_local)
            joints = self._pose_step(wrist_target, tgt_quat)
            self._last_joint_cmd = np.concatenate([joints, [self._grip]])
            obs, done, info = self.env.step(self._last_joint_cmd)
            return self._obs(obs), done, info
        if self.control == 'abs6':
            self._sp = np.clip(a[:3], self.WS[0], self.WS[1])
            self._rotvec = np.asarray(a[3:6], float)
            self._grip = float(np.clip(a[6], 0.0, 1.0))
            tgt_quat = self._target_quat()
            Rw = R.from_quat(_gs_to_xyzw(np_(self.eef.get_quat())))
            wrist_target = self._sp - Rw.apply(self._offset_local)
            joints = self._pose_step(wrist_target, tgt_quat)
            self._last_joint_cmd = np.concatenate([joints, [self._grip]])
            obs, done, info = self.env.step(self._last_joint_cmd)
            return self._obs(obs), done, info
        if self.control == 'abs':
            # ABSOLUTE tool-pose target: the ONLY self-correcting EEF encoding. A
            # relative command ('move +2.75mm x') is equally valid wherever the arm
            # is, so BC action error INTEGRATES (measured: ~0.5mm/step -> 15cm off
            # the demo path by t=150, gripper never closes). An absolute target is
            # independent of current pose, exactly like the joint-position targets
            # that make joint-space BC work.
            self._sp = np.clip(a[:3], self.WS[0], self.WS[1])
            self._pitch = float(np.clip(a[3], self.PITCH_RANGE[0], self.PITCH_RANGE[1]))
        elif self.control == 'delta':
            d = np.clip(a[:3], -self.DCAP, self.DCAP)
            sp = np.clip(self._sp + d, self.WS[0], self.WS[1])
            cur = self._tool_pos()
            self._sp = cur + np.clip(sp - cur, -self.LEASH, self.LEASH)   # leash
            dpitch = np.clip(float(a[3]), -self.DPITCH_CAP, self.DPITCH_CAP)
            p = np.clip(self._pitch + dpitch, self.PITCH_RANGE[0], self.PITCH_RANGE[1])
            mp = self._measured_pitch()
            self._pitch = float(mp + np.clip(p - mp, -self.LEASH_PITCH, self.LEASH_PITCH))
        else:
            v = np.clip(a[:3], -self.VCAP, self.VCAP)
            self._sp = np.clip(self._sp + v * self.DT, self.WS[0], self.WS[1])  # integrate the TOOL setpoint
            self._pitch = float(np.clip(self._pitch + float(a[3]) * self.DT,
                                        self.PITCH_RANGE[0], self.PITCH_RANGE[1]))
        self._grip = float(np.clip(a[4], 0.0, 1.0))
        tgt_quat = self._target_quat()
        # IK the WRIST so the TOOL lands on its setpoint: wrist_target = tool_sp - R @ offset_local
        Rw = R.from_quat(_gs_to_xyzw(np_(self.eef.get_quat())))
        wrist_target = self._sp - Rw.apply(self._offset_local)
        joints = self._pose_step(wrist_target, tgt_quat)
        self._last_joint_cmd = np.concatenate([joints, [self._grip]])
        obs, done, info = self.env.step(self._last_joint_cmd)
        return self._obs(obs), done, info

    def _obs(self, obs):
        # ee-centric obs, IDENTICAL layout to collect_cartesian_dataset.py so a policy
        # trained on that dataset sees the same thing at eval:
        #   ee_pos(3), ee_quat(4), gripper(1), grip_effort(1), can xyz(3), can quat(4), goal xy(2) = 18
        s = obs['state']
        ee = np_(self.eef.get_pos()); eq = np_(self.eef.get_quat())
        state = np.concatenate([ee, eq, s[6:8], s[8:17]]).astype(np.float32)
        out = dict(state=state, joint_state=s)
        if self.render_size is not None and self.w.get('cam') is not None:
            out['image'] = obs.get('image')
        return out
