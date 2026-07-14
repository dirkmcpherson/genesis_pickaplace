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
import sys
import pathlib as pl

import numpy as np
import torch
from scipy.spatial.transform import Rotation as R

REPO = pl.Path('/home/james/workspace/genesis_pickaplace')
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
    DT = 0.03                                      # env step physics time (3 substeps x 0.01)

    def __init__(self, backend='cpu', render_size=None, max_steps=1200):
        self.env = GenesisCanEnv(backend=backend, render_size=render_size, max_steps=max_steps)
        self.w = self.env.w
        self.arm = self.env.w['kdofs'][:6]
        self.eef = self.env.w['eef']
        self.kin = self.env.w['kinova']
        self.render_size = render_size

    # --- absolute ee-pose control (IK core) ---
    def _pose_step(self, pos, quat):
        qpos = self.kin.inverse_kinematics(
            link=self.eef, pos=np.asarray(pos, float), quat=np.asarray(quat, float),
            dofs_idx_local=self.arm, max_samples=1, max_solver_iters=15)
        joints = np_(qpos)[:6]
        return joints

    def reset(self, **kw):
        obs = self.env.reset(**kw)
        self._sp = np_(self.eef.get_pos()).astype(float)          # ee position setpoint
        self._q0 = np_(self.eef.get_quat()).astype(float)         # fixed roll/yaw+base orientation
        self._pitch = 0.0                                         # accumulated pitch (rad)
        self._grip = 0.0
        return self._obs(obs)

    def _target_quat(self):
        r = R.from_rotvec([0.0, self._pitch, 0.0]) * R.from_quat(_gs_to_xyzw(self._q0))
        return _xyzw_to_gs(r.as_quat())

    def step(self, action):
        a = np.asarray(action, float)
        v = np.clip(a[:3], -self.VCAP, self.VCAP)
        self._sp = np.clip(self._sp + v * self.DT, self.WS[0], self.WS[1])
        self._pitch += float(a[3]) * self.DT
        self._grip = float(np.clip(a[4], 0.0, 1.0))
        joints = self._pose_step(self._sp, self._target_quat())
        obs, done, info = self.env.step(np.concatenate([joints, [self._grip]]))
        return self._obs(obs), done, info

    def _obs(self, obs):
        # ee-centric observation for a Cartesian policy: ee pos(3), pitch(1), gripper(1),
        # grip_effort(1), can xyz(3), can quat(4), goal xy(2) = 15-dim.
        s = obs['state']
        ee = np_(self.eef.get_pos())
        state = np.concatenate([ee, [self._pitch], s[6:8], s[8:15], s[15:17]]).astype(np.float32)
        out = dict(state=state, joint_state=s)
        if self.render_size is not None and self.w.get('cam') is not None:
            out['image'] = obs.get('image')
        return out
