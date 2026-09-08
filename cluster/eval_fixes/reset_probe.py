#!/usr/bin/env python3
"""Coordinator's question: is the full-scope episode-order effect an INCOMPLETE RESET (defect) or amplification?
Dumps the COMPLETE observable env state immediately after reset(uid=254) in two conditions on the SAME node:
  solo : that reset is the first of the process
  seq  : it follows a full policy episode on uid 252 (the record's ordering)
then runs K further decisions with the same policy in both and reports the first decision at which they diverge.
Identical post-reset dumps + diverging trajectories => hidden solver state (still a reset defect, invisible at API level).
Differing dumps => incomplete reset, and the first differing field names it.
usage: reset_probe.py <r2d_root> <checkpoint> <mode solo|seq> <out.json> [K]"""
import json, os, sys
import numpy as np
r2d, ckpt, mode, out = sys.argv[1:5]
K = int(sys.argv[5]) if len(sys.argv) > 5 else 20
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
REPEAT = int(cfg.env.get("action_repeat", 1) or 1)
env = GenesisPick(str(cfg.env.task).split("_", 1)[1], size=tuple(cfg.env.size), seed=0, scope=str(cfg.env.scope),
                  action_repeat=REPEAT, action_mode=str(cfg.env.get("action_mode", "absolute")),
                  delta_cap=cfg.env.get("delta_cap", None), delta_leash_mult=cfg.env.get("delta_leash_mult", None),
                  reward_scale=float(cfg.env.get("reward_scale", 1.0)), state_obs=bool(cfg.env.get("state_obs", False)))
agent = Dreamer(cfg.model, env.observation_space, env.action_space).to("cpu")
ck = torch.load(ckpt, map_location="cpu", weights_only=False)
agent.load_state_dict(ck["agent_state_dict"], strict=False)
agent.clone_and_freeze(); agent.requires_grad_(False); agent.eval()
env._build()

def pack(o, r):
    d = {k: torch.as_tensor(np.asarray(v)[None]) for k, v in o.items()}
    d["reward"] = torch.tensor([r], dtype=torch.float32)
    td = TensorDict(d, batch_size=(1,), device="cpu")
    for k in td.keys():
        if td[k].ndim == 1: td[k] = td[k].unsqueeze(-1)
    return td

def obs_after_reset(uid):
    env._env.reset(options={"uid": int(uid)})
    getattr(env, "sync_delta_target", lambda: None)()
    o = {"is_first": True, "is_last": False, "is_terminal": False, "image": env._image()}
    if getattr(env, "_state_obs", False):
        o["state"] = env._state_vec(env._env.genv._obs()["state"]); env._last_state = o["state"]
    for k in STAGE_KEYS + ("task_success",):
        o[f"log_{k}"] = np.float32(0.0)
    return o

def np_(x): return x.detach().cpu().numpy() if hasattr(x, "detach") else np.asarray(x)

def dump_state():
    g = env._env.genv; w = g.w; kin = w["kinova"]
    d = {}
    d["qpos"] = [float(x) for x in np_(kin.get_dofs_position(dofs_idx_local=w["kdofs"]))]
    try: d["qvel"] = [float(x) for x in np_(kin.get_dofs_velocity(dofs_idx_local=w["kdofs"]))]
    except Exception as e: d["qvel"] = f"n/a {e}"
    for name in ("bottle", "goal"):
        e = w[name]
        d[f"{name}_pos"] = [float(x) for x in np_(e.get_pos())]
        d[f"{name}_quat"] = [float(x) for x in np_(e.get_quat())]
        for fn in ("get_vel", "get_ang"):
            try: d[f"{name}_{fn[4:]}"] = [float(x) for x in np_(getattr(e, fn)())]
            except Exception: pass
    d["obs_state"] = [float(x) for x in np_(g._obs()["state"])]
    try:
        c = np_(w["bottle"].get_contacts(w["goal"])["position"]); d["n_contact_bottle_goal"] = int(c.shape[0]) if c.size else 0
        c2 = np_(w["goal"].get_contacts(w["kinova"])["position"]); d["n_contact_goal_arm"] = int(c2.shape[0]) if c2.size else 0
    except Exception as e: d["contacts"] = f"n/a {e}"
    d["env_t"] = int(env._env._t); d["granted"] = sorted(env._env._granted)
    d["genv_picked"] = bool(g._picked); d["genv_contact"] = bool(g._contact); d["pick_run"] = int(g._pick_run)
    d["pv2_run"] = int(getattr(env._env, "_pv2_run", -1))
    for f in ("_dj_target", "_dj_qmeas"):
        v = getattr(env, f, None)
        if v is not None: d[f] = [float(x) for x in np.asarray(v).reshape(-1)]
    return d

if mode == "seq":
    o = obs_after_reset(252)
    st = agent.get_initial_state(1); tr = pack(o, 0.0); done = False; t = 0
    while not done and t * REPEAT < 1200:
        act, st = agent.act(tr, st, eval=True)
        a = act[0].detach().cpu().numpy().astype(np.float32)
        o, r, done, info = env.step(a); tr = pack(o, float(r)); t += 1
    print(f"[reset-probe] preceding uid252 episode: {t} decisions, done={done}", flush=True)

o = obs_after_reset(254)
rec = {"mode": mode, "post_reset": dump_state(), "steps": []}
st = agent.get_initial_state(1); tr = pack(o, 0.0)
for t in range(K):
    act, st = agent.act(tr, st, eval=True)
    a = act[0].detach().cpu().numpy().astype(np.float32)
    o, r, done, info = env.step(a); tr = pack(o, float(r))
    rec["steps"].append({"t": t, "action": [float(x) for x in a], "reward": float(r),
                         "state": [float(x) for x in np.asarray(o["state"]).reshape(-1)], "done": bool(done)})
    if done: break
json.dump(rec, open(out, "w"), indent=1)
print(f"[reset-probe] {mode}: wrote {out} ({len(rec['steps'])} steps)")
