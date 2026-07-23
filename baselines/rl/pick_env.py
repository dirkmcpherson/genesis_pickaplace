"""PickOnlyEnv: gymnasium wrapper around GenesisCanEnv for the PICK subtask.

Task: lift the can (can z > pick_z = table 0.05 + can_h/2 0.0505 + 0.05 = 0.1505)
while the gripper is commanded closed (grip > 0.3, mirroring GP_CLOSE=30 on the
0..100 motor scale). Sparse reward: 1.0 on the step 'picked' first becomes True,
0.0 otherwise. Episode terminates on pick; truncates at max_steps (default 300,
~10 s at 30 Hz -- note demo picks land at index 169..875, median ~380, so a
policy must be somewhat faster than the slowest human demos, or raise max_steps).

Action convention -- NORMALIZED [-1, 1] on all 7 dims (documented choice):
    SB3 SAC squashes its Gaussian through tanh, so its native output lives in
    [-1, 1]; SB3 also *stores* [-1,1]-scaled actions in the replay buffer for Box
    spaces. Making the env's action_space itself Box(-1, 1) means policy output,
    replay-buffer contents, and injected demo actions all share one space with no
    hidden rescaling inside SB3 (policy.scale_action becomes the identity).
    Physical mapping (see normalize_action / denormalize_action):
        arm joints:  physical target rad in [-pi, pi]  <->  a / pi
        gripper:     physical 0..1                     <->  g * 2 - 1
    Demo arm targets max out at |2.62| rad, so the +-pi clip is lossless.

One env instance per process (gs.init constraint); backend='cpu'.
"""
import sys
import pathlib as pl

import numpy as np
import gymnasium as gym
from gymnasium import spaces

REPO = pl.Path('/home/j/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO / 'baselines'))
from genesis_can_env import GenesisCanEnv  # noqa: E402

STATE_DIM = 17   # [6 joints, gripper 0..1, grip_effort, can xyz, can quat, goal xy]
ACT_DIM = 7      # [6 arm joint position targets (rad), gripper 0..1]
ARM_LIMIT = np.pi   # legacy scalar; kept for checkpoints trained before per-joint limits

# Per-joint action bounds = the demo command envelope, span DOUBLED and clamped to the
# +-pi actuator range. Rationale: demos occupy only 1/188th of [-pi,pi]^6, so ~99.5% of
# every action a uniform policy samples is motion no human ever commanded -- the actor
# spends its capacity proposing useless flailing. Doubling (rather than hugging the
# demo envelope) deliberately leaves room to discover alternate paths to the goal.
# Result: 1/188 -> 1/5.6 of the full volume, i.e. ~34x less irrelevant action space
# while still allowing well beyond demonstrated motion.
ARM_LO = np.array([-2.0887, -3.1416, -2.8338, -3.1416, -2.0145, -3.1416])
ARM_HI = np.array([ 2.5855,  1.4454,  3.1416,  1.6596,  3.1416,  0.3380])
ARM_MID = (ARM_HI + ARM_LO) / 2.0
ARM_HALF = (ARM_HI - ARM_LO) / 2.0
GRIP_CLOSED_FRAC = 0.3   # GP_CLOSE (30) on the 0..1 gripper scale


def normalize_action(a_phys):
    """Physical action [6 joint rad, grip 0..1] -> normalized [-1, 1]^7."""
    a = np.asarray(a_phys, dtype=np.float32)
    out = np.empty_like(a)
    out[..., :6] = np.clip((a[..., :6] - ARM_MID) / ARM_HALF, -1.0, 1.0)
    out[..., 6] = np.clip(a[..., 6] * 2.0 - 1.0, -1.0, 1.0)
    return out


def denormalize_action(a_norm):
    """Normalized [-1, 1]^7 -> physical action [6 joint rad, grip 0..1]."""
    a = np.asarray(a_norm, dtype=np.float64)
    out = np.empty_like(a)
    out[..., :6] = np.clip(a[..., :6], -1.0, 1.0) * ARM_HALF + ARM_MID
    out[..., 6] = (np.clip(a[..., 6], -1.0, 1.0) + 1.0) / 2.0
    return out


class PickOnlyEnv(gym.Env):
    """Gymnasium Box-obs / Box-action wrapper of GenesisCanEnv (pick subtask).

    SB3 SAC needs a Box observation space, so the GenesisCanEnv dict obs is
    flattened to the 17-dim 'state' vector here (known SB3 pitfall).
    """
    metadata = {'render_modes': []}

    def __init__(self, backend='cpu', max_steps=300, fixed_uid=None):
        super().__init__()
        self.genv = GenesisCanEnv(backend=backend)   # builds the world ONCE
        self.max_steps = int(max_steps)
        self.fixed_uid = fixed_uid
        # PICK training uses only success-labeled solved trials
        self.success_uids = sorted(
            u for u, r in self.genv.placements.items() if r.get('label') == 'success')
        assert self.success_uids, 'no success-labeled trials in trial_placements.json'
        self.pick_z = float(self.genv.w['pick_z'])   # 0.1505 with the v2 world
        self.observation_space = spaces.Box(-np.inf, np.inf, (STATE_DIM,), np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, (ACT_DIM,), np.float32)
        self._t = 0

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        uid = None
        if options:
            uid = options.get('uid')
        if uid is None:
            uid = self.fixed_uid
        if uid is None:
            uid = int(self.np_random.choice(self.success_uids))
        obs = self.genv.reset(uid=int(uid))
        self._t = 0
        return obs['state'].astype(np.float32), {'uid': int(uid)}

    def step(self, action):
        a_phys = denormalize_action(action)
        obs, _env_done, info = self.genv.step(a_phys)
        self._t += 1
        # Sparse reward: 1.0 the first time picked flips True. Because we
        # terminate immediately on pick, this fires at most once per episode.
        terminated = bool(info['picked'])
        reward = 1.0 if terminated else 0.0
        truncated = (not terminated) and self._t >= self.max_steps
        return obs['state'].astype(np.float32), reward, terminated, truncated, info
