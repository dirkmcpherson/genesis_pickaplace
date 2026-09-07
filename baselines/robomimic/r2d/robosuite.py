"""robosuite Can suite for r2dreamer (task: robosuite_can) -- the robomimic leg's world-model arm.

Installed by cluster/robomimic/install_r2d_files.sh as $LAB/robomimic_r2d/envs/robosuite.py (a COPY of the WM-fix
r2dreamer tree; the Genesis tree is never touched). Same env API as envs/genesis.py / envs/maniskill.py: bare-dict
reset, old 4-tuple step, is_first / is_last / is_terminal in every obs, log_* flags on the episode-final step only.

Observation: `state` f32[23] (robo_common.STATE_KEYS: eef_pos, eef_quat, gripper_qpos, object) -- the encoder /
decoder mlp_keys; `image` zeros u8 (H, W, 3) -- a placeholder the cnn_keys '$^' never read (kept tiny, 16x16,
because the CPU replay stores it); `log_success`. Action Box(-1, 1, (7,)) = the data's OSC_POSE deltas + gripper
(no NormalizeActions needed). Reward +reward_scale once at the first success (env.is_success()["task"], the can in
its bin), is_terminal there; TimeLimit(400) (envs/__init__.py) owns truncation, non-terminal (bootstrap).
The inner robomimic EnvRobosuite is built lazily in the spawn worker from the hdf5's env_meta.
"""
import os
import sys

import gymnasium as gym
import numpy as np

REPO = os.environ.get("GENESIS_PICKAPLACE_ROOT", "/cluster/tufts/shortlab/jstale02/genesis_pickaplace")
LOG_KEYS = ("success",)
STATE_DIM = 23
ACT_DIM = 7


class RobosuiteCan(gym.Env):
    def __init__(self, task="can", size=(16, 16), seed=0, hdf5=None, reward_scale=1.0, image_channels=3):
        assert task == "can", task
        self._task = task; self._size = tuple(size); self._seed = int(seed)
        self._hdf5 = hdf5; self._reward_scale = float(reward_scale); self._ch = int(image_channels)
        self._env = None; self._state = None
        self.reward_range = [-np.inf, np.inf]

    def _build(self):
        p = f"{REPO}/baselines/robomimic"
        if p not in sys.path:
            sys.path.insert(0, p)
        import robo_common
        self._rc = robo_common
        self._env, self._meta = robo_common.make_env(self._hdf5 or robo_common.HDF5["ph"])
        np.random.seed(self._seed)          # robosuite's placement sampler draws from the global np.random

    @property
    def observation_space(self):
        spaces = {"image": gym.spaces.Box(0, 255, self._size + (self._ch,), dtype=np.uint8),
                  "state": gym.spaces.Box(-np.inf, np.inf, (STATE_DIM,), dtype=np.float32)}
        for k in LOG_KEYS:
            spaces[f"log_{k}"] = gym.spaces.Box(-np.inf, np.inf, (1,), dtype=np.float32)
        return gym.spaces.Dict(spaces)

    @property
    def action_space(self):
        # plain Box, no .discrete attribute -> continuous actor (see envs/genesis.py)
        return gym.spaces.Box(-1.0, 1.0, (ACT_DIM,), dtype=np.float32)

    def _image(self):
        return np.zeros(self._size + (self._ch,), np.uint8)

    def _obs(self, first, last, terminal, success_flag):
        obs = {"is_first": first, "is_last": last, "is_terminal": terminal, "image": self._image(), "state": self._state}
        obs["log_success"] = np.float32(float(success_flag))
        return obs

    def reset(self, **kwargs):
        if self._env is None:
            self._build()
        if kwargs.get("entry") is not None:            # evaluators: restore a bank entry (model xml + flattened state)
            self._state = self._rc.reset_env_to(self._env, kwargs["entry"]).astype(np.float32)
        else:
            self._state = self._rc.reset_env(self._env).astype(np.float32)   # fresh obs (reset()'s dict is stale)
        return self._obs(True, False, False, False)

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=np.float32).reshape(-1), -1.0, 1.0)
        assert a.shape == (ACT_DIM,) and np.isfinite(a).all(), a
        o, r_env, d_env, info = self._env.step(a)
        self._state = self._rc.state_from_obs(o).astype(np.float32)
        success = bool(self._env.is_success()["task"])
        reward = (1.0 if success else 0.0) * self._reward_scale
        done = success
        return self._obs(False, done, success, done and success), np.float32(reward), done, {"success": success}

    def render(self):
        return self._image()

    def close(self):
        pass
