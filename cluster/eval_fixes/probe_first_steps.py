#!/usr/bin/env python3
"""Coordinator's hint #1: does the policy see different INPUTS or emit different ACTIONS from step 0?
Rebuilds the eval stack exactly as eval_genesis.py does (same adapter, same Dreamer construction, same pack(), same
mode/seed) for ONE ic, and dumps the first N (state, action, reward, stage-flag) tuples as JSON so two trees can be
diffed element-wise. Trees are selected by GENESIS_PICKAPLACE_ROOT + the r2dreamer root passed as argv[1].
usage: probe_first_steps.py <r2d_root> <checkpoint> <ic_file> <ic_set> <ic_index> <out.json> [n_steps]"""
import json, os, sys
import numpy as np
r2d, ckpt, ic_file, ic_set, ic_index, out = sys.argv[1:7]
N = int(sys.argv[7]) if len(sys.argv) > 7 else 10
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
                  place_entry_bank=None, action_repeat=REPEAT, action_mode=str(cfg.env.get("action_mode", "absolute")),
                  delta_cap=cfg.env.get("delta_cap", None), delta_leash_mult=cfg.env.get("delta_leash_mult", None),
                  reward_scale=float(cfg.env.get("reward_scale", 1.0)), state_obs=bool(cfg.env.get("state_obs", False)),
                  state_extra=cfg.env.get("state_extra", None))
agent = Dreamer(cfg.model, env.observation_space, env.action_space).to("cpu")
ck = torch.load(ckpt, map_location="cpu", weights_only=False)
missing, unexpected = agent.load_state_dict(ck["agent_state_dict"], strict=False)
agent.clone_and_freeze(); agent.requires_grad_(False); agent.eval()
env._build()
ic = json.load(open(ic_file))[ic_set][int(ic_index)]
kw = {k: ic[k] for k in ("can_pos", "can_quat", "goal_pos") if ic.get(k) is not None}
env._env.reset_to(kw)
getattr(env, "sync_delta_target", lambda: None)()
obs = {"is_first": True, "is_last": False, "is_terminal": False, "image": env._image()}
if getattr(env, "_state_obs", False):
    obs["state"] = env._state_vec(env._env.genv._obs()["state"]); env._last_state = obs["state"]
for k in STAGE_KEYS + ("task_success",):
    obs[f"log_{k}"] = np.float32(0.0)

def pack(o, r):
    d = {k: torch.as_tensor(np.asarray(v)[None]) for k, v in o.items()}
    d["reward"] = torch.tensor([r], dtype=torch.float32)
    td = TensorDict(d, batch_size=(1,), device="cpu")
    for k in td.keys():
        if td[k].ndim == 1: td[k] = td[k].unsqueeze(-1)
    return td

state = agent.get_initial_state(1); trans = pack(obs, 0.0)
rows = [dict(step=-1, state=[round(float(x), 9) for x in np.asarray(obs["state"]).reshape(-1)], action=None, reward=None,
             obs_keys=sorted(obs.keys()))]
done = False
for t in range(N):
    if done: break
    act, state = agent.act(trans, state, eval=True)
    a = act[0].detach().cpu().numpy().astype(np.float32)
    obs, reward, done, info = env.step(a)
    rows.append(dict(step=t, action=[round(float(x), 9) for x in a],
                     state=[round(float(x), 9) for x in np.asarray(obs["state"]).reshape(-1)],
                     reward=round(float(reward), 9), done=bool(done),
                     flags={k: bool(info.get(k)) for k in ("picked", "contact", "nested", "tipped", "placed_v2", "slide_success")}))
    trans = pack(obs, float(reward))
json.dump(dict(r2d=r2d, gp=os.environ.get("GENESIS_PICKAPLACE_ROOT"), ckpt=ckpt, missing=len(missing),
               unexpected=len(unexpected), obs_space=sorted(env.observation_space.spaces.keys()), rows=rows),
          open(out, "w"), indent=1)
print(f"[probe] wrote {out}: {len(rows)-1} steps, missing {len(missing)}, unexpected {len(unexpected)}")
print(f"[probe] obs_space: {sorted(env.observation_space.spaces.keys())}")
