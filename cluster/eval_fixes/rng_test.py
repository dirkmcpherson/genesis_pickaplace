#!/usr/bin/env python3
"""Does the policy's action depend on the GLOBAL torch RNG (i.e. on how many decisions preceded this episode)?
Same env state, same agent, three probes in ONE process:
  a1: fresh RNG (seed 0)            -> reference
  a2: RNG re-seeded to 0 again      -> must equal a1 if act() is RNG-dependent-but-seeded
  a3: RNG advanced (torch.randn)    -> if a3 != a1, act() consumes global RNG => episodes in a sequence are NOT
                                       independent, and a per-episode re-seed makes them so.
Runs in eval/mode (deterministic actor mode) -- the same setting every cell of record used."""
import os, sys
import numpy as np
r2d, ckpt = sys.argv[1:3]
sys.path.insert(0, r2d)
import torch
torch.set_num_threads(4)
from omegaconf import OmegaConf
from tensordict import TensorDict
import tools
from dreamer import Dreamer
from envs.genesis import STAGE_KEYS, GenesisPick
cfg = OmegaConf.load(os.path.join(os.path.dirname(ckpt), ".hydra", "config.yaml"))
cfg.device = "cpu"; cfg.model.compile = False
tools.set_seed_everywhere(0)
env = GenesisPick(str(cfg.env.task).split("_", 1)[1], size=tuple(cfg.env.size), seed=0, scope=str(cfg.env.scope),
                  action_repeat=int(cfg.env.get("action_repeat", 1) or 1), action_mode=str(cfg.env.get("action_mode", "absolute")),
                  delta_cap=cfg.env.get("delta_cap", None), delta_leash_mult=cfg.env.get("delta_leash_mult", None),
                  reward_scale=float(cfg.env.get("reward_scale", 1.0)), state_obs=bool(cfg.env.get("state_obs", False)))
agent = Dreamer(cfg.model, env.observation_space, env.action_space).to("cpu")
ck = torch.load(ckpt, map_location="cpu", weights_only=False)
agent.load_state_dict(ck["agent_state_dict"], strict=False)
agent.clone_and_freeze(); agent.requires_grad_(False); agent.eval()
env._build()

def obs_reset(uid):
    env._env.reset(options={"uid": int(uid)})
    getattr(env, "sync_delta_target", lambda: None)()
    o = {"is_first": True, "is_last": False, "is_terminal": False, "image": env._image()}
    if getattr(env, "_state_obs", False):
        o["state"] = env._state_vec(env._env.genv._obs()["state"]); env._last_state = o["state"]
    for k in STAGE_KEYS + ("task_success",):
        o[f"log_{k}"] = np.float32(0.0)
    return o

def pack(o):
    d = {k: torch.as_tensor(np.asarray(v)[None]) for k, v in o.items()}
    d["reward"] = torch.tensor([0.0], dtype=torch.float32)
    td = TensorDict(d, batch_size=(1,), device="cpu")
    for k in td.keys():
        if td[k].ndim == 1: td[k] = td[k].unsqueeze(-1)
    return td

def first_action(o):
    st = agent.get_initial_state(1)
    act, _ = agent.act(pack(o), st, eval=True)      # eval=True == --mode mode, the cells of record
    return act[0].detach().cpu().numpy().astype(np.float64)

o = obs_reset(254); s_ref = np.asarray(o["state"], np.float64).copy()
torch.manual_seed(0); a1 = first_action(o)
o2 = obs_reset(254); torch.manual_seed(0); a2 = first_action(o2)
o3 = obs_reset(254); _ = torch.randn(1000); a3 = first_action(o3)
print(f"[rng] obs identical across the three resets: "
      f"{np.max(np.abs(np.asarray(o2['state'], np.float64) - s_ref)):.3e} / {np.max(np.abs(np.asarray(o3['state'], np.float64) - s_ref)):.3e}")
print(f"[rng] a2 vs a1 (same seed)          max|diff| = {np.max(np.abs(a2 - a1)):.3e}")
print(f"[rng] a3 vs a1 (RNG advanced)       max|diff| = {np.max(np.abs(a3 - a1)):.3e}")
print("[rng] VERDICT: " + ("act() CONSUMES GLOBAL RNG -> episodes in one process are not independent; "
                           "a per-episode re-seed makes subset reruns reproduce"
                           if np.max(np.abs(a3 - a1)) > 0 else
                           "act() is RNG-independent -> the order effect is NOT the policy"))
