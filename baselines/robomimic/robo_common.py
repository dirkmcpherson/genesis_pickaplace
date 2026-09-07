"""Shared pieces of the robomimic Can leg (paper/ROBOMIMIC_PLAN_2026-09-05.md §2-§3).

One definition for everything the three learners, the BC-RNN control, the gates and the
evaluators must agree on:

  * STATE_KEYS   the 23-dim study observation (robomimic generate_paper_configs default low-dim
                 obs: eef_pos 3 + eef_quat 4 + gripper_qpos 2 + object 14) in FIXED order.
                 state[:PROPRIO_DIM] is proprioception, state[PROPRIO_DIM:] the object block --
                 the DP arm splits at PROPRIO_DIM into observation.state / observation.environment_state.
  * make_env     robomimic EnvRobosuite from the hdf5's own env_meta (same controller config as
                 the data => demo actions are valid env actions; ignore_done=True, so OUR wrapper
                 owns termination: +1 and terminate at the first success, truncate at HORIZON).
  * Bank         bank_can50.npz (50 initial MuJoCo states + model xml) replayed through reset_to
                 for every learner / seed / checkpoint; its sha256 goes in every metrics.json.
  * run_bank_eval  the ONE evaluation loop: policy_fn(state23) -> action7, HORIZON decisions,
                 success = env.is_success()["task"], metrics.json in our layout
                 (episodes, success, n_success, per_episode, bank_sha256, mode, ...).

Runs in $LAB/robo_venv (py3.10) and $LAB/r2d_venv_robo (py3.11); numpy + h5py + robomimic only.
"""
import hashlib
import json
import os
import pathlib as pl
import time

import numpy as np

STATE_KEYS = ("robot0_eef_pos", "robot0_eef_quat", "robot0_gripper_qpos", "object")
# `object` (robosuite PickPlace single-object `object-state`, VERIFIED live 2026-09-06 against the sim body pose and the
# file's obs): [Can_to_robot0_eef_pos (3, can pose in the GRIPPER frame), Can_to_robot0_eef_quat (4), Can_pos (3, WORLD),
# Can_quat (4)]. In the 23-dim state: eef_pos 0:3, eef_quat 3:7, gripper 7:9, can->eef 9:16, CAN_POS 16:19, CAN_QUAT 19:23.
CAN_POS = slice(16, 19); CAN_QUAT = slice(19, 23); CAN_REL = slice(9, 16)
# Two robosuite observation facts every reader of this state must know (probe 2026-09-06 23:40, robo_common/obs_probe):
#  (1) the dict RETURNED by env.reset()/reset_to() carries a STALE object block (zeros on the first reset, the previous
#      episode's values later); only env.get_observation() after the reset is fresh -> reset_env_to / the gym env / the
#      r2dreamer adapter always re-read through get_observation().
#  (2) the FILES' t=0 rows carry robosuite's empty-cache artefact in object[0:7] (the object pose treated as identity:
#      -R_eef^T p_eef and conj(q_eef), norm exactly 1); rows t>=1 are consistent. Identical in PH/MH/MG (one generator),
#      so it does not bias the source comparison; disclosed in the plan log.
STATE_DIMS = {"robot0_eef_pos": 3, "robot0_eef_quat": 4, "robot0_gripper_qpos": 2, "object": 14}
STATE_DIM = sum(STATE_DIMS[k] for k in STATE_KEYS)          # 23
PROPRIO_DIM = 9                                              # eef_pos + eef_quat + gripper_qpos
ACT_DIM = 7                                                  # OSC_POSE (6) + gripper (1), all in [-1, 1]
HORIZON = 400                                                # decisions per episode (plan §2: one horizon for every arm)
CONTROL_HZ = 20
LAB = os.environ.get("LAB", "/cluster/tufts/shortlab/jstale02")
DATA_ROOT = pl.Path(os.environ.get("ROBOMIMIC_DATA", f"{LAB}/robomimic_data"))
HDF5 = {
    "ph": DATA_ROOT / "v1.5/can/ph/low_dim_v15.hdf5",
    "mh": DATA_ROOT / "v1.5/can/mh/low_dim_v15.hdf5",
    "mg": DATA_ROOT / "v1.5/can/mg/low_dim_sparse_v15.hdf5",
    "paired": DATA_ROOT / "v1.5/can/paired/low_dim_v15.hdf5",
}
BANK_PATH = DATA_ROOT / "bank_can50.npz"


def sha256_file(path, chunk=1 << 22):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for blk in iter(lambda: fh.read(chunk), b""):
            h.update(blk)
    return h.hexdigest()


def state_from_obs(obs):
    """robomimic obs dict -> (23,) float32 in STATE_KEYS order."""
    parts = []
    for k in STATE_KEYS:
        v = np.asarray(obs[k], dtype=np.float32).reshape(-1)
        assert v.shape[0] == STATE_DIMS[k], (k, v.shape)
        parts.append(v)
    return np.concatenate(parts)


def state_from_hdf5_group(g, n=None):
    """h5py group holding obs/next_obs keys -> (T,23) float32 in STATE_KEYS order."""
    cols = [np.asarray(g[k], dtype=np.float32) for k in STATE_KEYS]
    out = np.concatenate(cols, axis=1)
    assert out.shape[1] == STATE_DIM, out.shape
    if n is not None:
        assert out.shape[0] == n, (out.shape, n)
    return out


def load_env_meta(hdf5_path):
    import h5py
    with h5py.File(str(hdf5_path), "r") as f:
        meta = json.loads(f["data"].attrs["env_args"])
    return meta


def make_env(hdf5_path=None, env_meta=None):
    """EnvRobosuite from the dataset's env_meta (state only, no renderer). Initialises robomimic's
    ObsUtils with the low-dim keys first (EnvRobosuite.get_observation needs the registry)."""
    import robomimic.utils.obs_utils as ObsUtils
    import robomimic.utils.env_utils as EnvUtils
    if env_meta is None:
        env_meta = load_env_meta(hdf5_path)
    ObsUtils.initialize_obs_utils_with_obs_specs({"obs": {"low_dim": list(STATE_KEYS), "rgb": []}})
    env = EnvUtils.create_env_from_metadata(env_meta=env_meta, render=False, render_offscreen=False, use_image_obs=False)
    return env, env_meta


class Bank:
    """bank_can50.npz: states (K, D) float64, models (K,) str, ep_meta (K,) str, can_xy0 (K,2), seeds (K,)."""

    def __init__(self, path=BANK_PATH):
        self.path = pl.Path(path)
        z = np.load(self.path, allow_pickle=True)
        self.states = z["states"]
        self.models = [str(m) for m in z["models"]]
        self.ep_meta = [str(m) for m in z["ep_meta"]] if "ep_meta" in z.files else [None] * len(self.models)
        self.can_xy0 = z["can_xy0"]
        self.seeds = z["seeds"]
        self.sha256 = sha256_file(self.path)
        self.n = int(self.states.shape[0])

    def entry(self, k):
        d = {"states": self.states[k], "model": self.models[k]}
        if self.ep_meta[k]:
            d["ep_meta"] = self.ep_meta[k]
        return d


def reset_env_to(env, entry):
    """Restore a bank / hdf5 entry (model xml + flattened state) and return the 23-dim state, re-read through
    get_observation() (the dict reset_to returns can be stale -- fact (1) above)."""
    env.reset_to(entry)
    return state_from_obs(env.get_observation())


def reset_env(env, seed=None):
    """Plain robosuite reset (random placement; seed -> np.random, the sampler's source) + a FRESH observation."""
    if seed is not None:
        np.random.seed(int(seed))
    env.reset()
    return state_from_obs(env.get_observation())


def run_bank_eval(env, bank, policy_fn, out_dir, *, reset_fn=None, horizon=HORIZON, indices=None, tag="",
                  extra=None, verbose=True):
    """The one evaluation loop. policy_fn(state23) -> action7 (clipped to [-1,1] here).
    reset_fn() is called at the start of every episode (policy-side state, e.g. DP action queue).
    Writes <out_dir>/metrics.json: episodes, n_success, success, per_episode [{k, success, steps,
    can_xy0}], bank_sha256, horizon, plus `extra` verbatim. Success = env.is_success()["task"] at any
    decision (terminates the episode, like the training wrapper)."""
    out_dir = pl.Path(out_dir); out_dir.mkdir(parents=True, exist_ok=True)
    idx = list(range(bank.n)) if indices is None else [int(i) for i in indices]
    per = []
    t_all = time.time()
    for k in idx:
        t0 = time.time()
        if reset_fn is not None:
            reset_fn()
        s = reset_env_to(env, bank.entry(k))
        success = False; steps = 0
        for t in range(horizon):
            a = np.clip(np.asarray(policy_fn(s), dtype=np.float32).reshape(-1), -1.0, 1.0)
            assert a.shape == (ACT_DIM,), a.shape
            obs, r, done, info = env.step(a)
            s = state_from_obs(obs); steps = t + 1
            if bool(env.is_success()["task"]):
                success = True; break
        per.append(dict(k=k, success=bool(success), steps=int(steps), seconds=round(time.time() - t0, 2),
                        can_xy0=[float(x) for x in bank.can_xy0[k]]))   # world can xy at t0 (bank: state[CAN_POS][:2])
        if verbose:
            print(f"[eval{(' ' + tag) if tag else ''}] ep{k}: {'success' if success else 'fail'} ({steps} steps, {time.time() - t0:.1f}s)", flush=True)
    n = len(per); ns = sum(p["success"] for p in per)
    summary = dict(episodes=n, n_success=int(ns), success=(ns / n if n else 0.0), horizon=int(horizon),
                   bank=str(bank.path), bank_sha256=bank.sha256, seconds=round(time.time() - t_all, 1),
                   per_episode=per, **(extra or {}))
    (out_dir / "metrics.json").write_text(json.dumps(summary, indent=1))
    if verbose:
        print(f"[eval{(' ' + tag) if tag else ''}] {ns}/{n} = {summary['success']:.3f} -> {out_dir}/metrics.json", flush=True)
    return summary
