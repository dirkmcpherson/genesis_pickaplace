"""RobosuiteCanEnv: gymnasium env over robomimic's EnvRobosuite for the RLPD arm (plan §4).

obs Box(23) = robo_common.STATE_KEYS; act Box(7) in [-1,1] (OSC_POSE deltas + gripper, the data's own
controller). Reward +1.0 and terminated=True at the FIRST success (env.is_success()["task"], the can in its
target bin); truncated at `horizon` decisions (400). reset(): robosuite's own random placement;
reset(options={"entry": {...}}) restores a bank / hdf5 entry via reset_to (the evaluators).
The inner env is built lazily (constructed in the process that uses it).
"""
import pathlib as pl
import sys

import gymnasium as gym
import numpy as np

sys.path.insert(0, str(pl.Path(__file__).resolve().parent))
from robo_common import STATE_DIM, ACT_DIM, HORIZON, HDF5, make_env, state_from_obs, reset_env_to, reset_env  # noqa: E402


class RobosuiteCanEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, hdf5=str(HDF5["ph"]), horizon=HORIZON):
        super().__init__()
        self._hdf5 = str(hdf5); self._horizon = int(horizon)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (STATE_DIM,), dtype=np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (ACT_DIM,), dtype=np.float32)
        self._env = None; self._t = 0; self._success = False; self.env_meta = None

    def _build(self):
        self._env, self.env_meta = make_env(self._hdf5)

    @property
    def inner(self):
        if self._env is None:
            self._build()
        return self._env

    def reset(self, seed=None, options=None):
        if self._env is None:
            self._build()
        if options and options.get("entry") is not None:
            s = reset_env_to(self._env, options["entry"])
        else:
            s = reset_env(self._env, seed=seed)   # seed -> global np.random = robosuite's placement source (plan §2); fresh obs
        self._t = 0; self._success = False
        return s.astype(np.float32), {}

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=np.float32).reshape(-1), -1.0, 1.0)
        assert a.shape == (ACT_DIM,), a.shape
        obs, r_env, done_env, info = self._env.step(a)
        self._t += 1
        s = state_from_obs(obs)
        succ = bool(self._env.is_success()["task"])
        reward = 1.0 if succ else 0.0
        terminated = succ
        truncated = (not succ) and self._t >= self._horizon
        self._success = self._success or succ
        return s.astype(np.float32), float(reward), bool(terminated), bool(truncated), {"success": succ, "t": self._t}

    def close(self):
        pass


if __name__ == "__main__":
    import time
    env = RobosuiteCanEnv()
    s, _ = env.reset(seed=0)
    t0 = time.time(); n = 0; R = 0.0
    for ep in range(2):
        s, _ = env.reset()
        for t in range(400):
            s, r, te, tr, info = env.step(env.action_space.sample()); n += 1; R += r
            if te or tr:
                break
    print(f"[smoke] {n} steps in {time.time() - t0:.1f}s ({n / (time.time() - t0):.0f} steps/s), reward {R}")
