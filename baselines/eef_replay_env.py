"""Recorded EEF differential actions over an already configured GenesisCanEnv.

Actions are world dx/dy/dz in metres, left-composed world rotation vectors in
radians, and recorded motor feedback divided by 100. Targets integrate from the
previous target. Zero Cartesian action holds that target. This replay adapter
preserves recorded overshoot and uses unconstrained IK targets with the existing
physical joint limits; it is not a clipped policy-action interface.
"""
from pathlib import Path

import numpy as np

from eef_delta_control import ArmKinematics, EEFDeltaController, FKPrecisionRefiner, from_genesis


class EEFReplayEnv:
    def __init__(self, env, urdf=None):
        self.env = env
        self.fk = ArmKinematics(urdf or Path(__file__).resolve().parents[1] / 'gen3_lite_2f_robotiq_85.urdf')
        self.controller = None
        self.refiner = None
        self.last_joint_target = None

    def reset(self, **kwargs):
        observation = self.env.reset(**kwargs)
        world = self.env.w
        robot = world['kinova']
        array = EEFDeltaController.array
        base = robot.get_link(self.fk.base_link)
        mount = from_genesis(array(base.get_pos()), array(base.get_quat()))
        self.controller = EEFDeltaController(robot, world['eef'], world['kdofs'][:6],
                                             self.fk.wrist_to_tool, respect_joint_limit=False)
        self.refiner = FKPrecisionRefiner(self.fk, mount)
        self.last_joint_target = None
        return observation

    def step(self, action):
        if self.controller is None:
            raise RuntimeError('Call reset before stepping EEF actions')
        action = np.asarray(action, dtype=np.float64)
        if action.shape != (7,) or not np.isfinite(action).all():
            raise ValueError('Expected seven finite EEF/motor action values')
        command = self.controller.command(action[:6])
        command = self.refiner.refine(command, self.controller.target)
        self.last_joint_target = command.copy()
        return self.env.step(np.r_[command, action[6]], arm_cmd=command,
                             grip_motor=float(action[6]) * 100)
