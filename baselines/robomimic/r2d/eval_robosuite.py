"""Fresh-process r2dreamer evaluator on the 50-state bank (robomimic leg; eval_genesis.py equivalent).

Loads latest.pt + the run's .hydra/config.yaml, rebuilds the agent exactly as train.py does, restores every bank
entry through the adapter's reset(entry=...) (robomimic reset_to: model xml + flattened state), rolls out
time_limit decisions with agent.act(trans, state, eval=(mode == 'mode')), and writes metrics.json in the shared
layout (episodes, n_success, success, per_episode, bank_sha256, mode, ...). Installed at $LAB/robomimic_r2d/.

  $LAB/r2d_venv_robo/bin/python eval_robosuite.py --checkpoint runs/<run>/latest.pt --mode mode --out runs/<run>/eval_bank50_mode --device cpu
"""
import argparse
import json
import os
import pathlib
import sys
import time

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
REPO = os.environ.get("GENESIS_PICKAPLACE_ROOT", "/cluster/tufts/shortlab/jstale02/genesis_pickaplace")
sys.path.insert(0, f"{REPO}/baselines/robomimic")

ap = argparse.ArgumentParser()
ap.add_argument("--checkpoint", required=True, help="latest.pt path (or a run dir containing one)")
ap.add_argument("--config", default=None, help="hydra-resolved config.yaml; default <run_dir>/.hydra/config.yaml")
ap.add_argument("--mode", choices=["sample", "mode"], default="mode")
ap.add_argument("--bank", default=None)
ap.add_argument("--episodes", type=int, default=None, help="first N bank entries (default all)")
ap.add_argument("--device", default="cpu")
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--out", required=True)
ap.add_argument("--torch-threads", type=int, default=4)
args = ap.parse_args()

import torch  # noqa: E402

torch.set_num_threads(args.torch_threads)
from omegaconf import OmegaConf  # noqa: E402
from tensordict import TensorDict  # noqa: E402

import tools  # noqa: E402
from dreamer import Dreamer  # noqa: E402
from envs.robosuite import RobosuiteCan  # noqa: E402
from robo_common import Bank, BANK_PATH  # noqa: E402

CKPT = pathlib.Path(args.checkpoint).expanduser().resolve()
if CKPT.is_dir():
    CKPT = CKPT / "latest.pt"
RUN_DIR = CKPT.parent
if not CKPT.exists():
    sys.exit(f"checkpoint not found: {CKPT}")
CFG_PATH = pathlib.Path(args.config) if args.config else RUN_DIR / ".hydra" / "config.yaml"
if not CFG_PATH.exists():
    sys.exit(f"config not found: {CFG_PATH}")
OUT = pathlib.Path(args.out); OUT.mkdir(parents=True, exist_ok=True)
tools.set_seed_everywhere(args.seed)
cfg = OmegaConf.load(CFG_PATH)
cfg.device = args.device
cfg.model.compile = False
assert str(cfg.env.task) == "robosuite_can", cfg.env.task
assert int(cfg.env.get("action_repeat", 1)) == 1, cfg.env.action_repeat
HORIZON = int(cfg.env.time_limit)
print(f"[eval] ckpt={CKPT} config={CFG_PATH} device={args.device} mode={args.mode} horizon={HORIZON} seed={args.seed}")

env = RobosuiteCan("can", size=tuple(cfg.env.size), seed=args.seed, hdf5=cfg.env.get("hdf5", None),
                   reward_scale=float(cfg.env.get("reward_scale", 1.0)), image_channels=int(cfg.env.get("image_channels", 3)))
agent = Dreamer(cfg.model, env.observation_space, env.action_space).to(args.device)
ck = torch.load(CKPT, map_location=args.device, weights_only=False)
CKPT_STEP = ck.get("step")
missing, unexpected = agent.load_state_dict(ck["agent_state_dict"], strict=False)
print(f"[eval] loaded checkpoint (missing {len(missing)}, unexpected {len(unexpected)}) step {CKPT_STEP}")
assert not any(k.startswith(("actor.", "rssm.", "encoder.")) for k in missing), missing
agent.clone_and_freeze()
agent.requires_grad_(False)
agent.eval()


def pack(obs, reward):
    d = {k: torch.as_tensor(np.asarray(v)[None]) for k, v in obs.items()}
    d["reward"] = torch.tensor([reward], dtype=torch.float32)
    td = TensorDict(d, batch_size=(1,), device="cpu")
    for k in td.keys():
        if td[k].ndim == 1:
            td[k] = td[k].unsqueeze(-1)
    return td.to(args.device)


bank = Bank(args.bank or BANK_PATH)
idx = list(range(bank.n)) if args.episodes is None else list(range(args.episodes))
per = []; t_all = time.time()
for k in idx:
    t0 = time.time()
    obs = env.reset(entry=bank.entry(k))
    state = agent.get_initial_state(1)
    trans = pack(obs, 0.0)
    done = False; t = 0; success = False; ep_r = 0.0
    while not done and t < HORIZON:
        act, state = agent.act(trans, state, eval=(args.mode == "mode"))
        a = act[0].detach().cpu().numpy().astype(np.float32)
        obs, reward, done, info = env.step(a)
        trans = pack(obs, float(reward)); ep_r += float(reward); t += 1
        success = success or bool(info.get("success"))
    per.append(dict(k=k, success=bool(success), steps=int(t), reward=ep_r, seconds=round(time.time() - t0, 2),
                    can_xy0=[float(x) for x in bank.can_xy0[k]]))
    print(f"[eval r2d {args.mode}] ep{k}: {'success' if success else 'fail'} ({t} steps, r={ep_r:.1f}, {time.time() - t0:.1f}s)", flush=True)
n = len(per); ns = sum(p["success"] for p in per)
summary = dict(episodes=n, n_success=int(ns), success=(ns / n if n else 0.0), horizon=HORIZON, bank=str(bank.path), bank_sha256=bank.sha256,
               learner="r2dreamer", checkpoint=str(CKPT), ckpt_step=CKPT_STEP, mode=args.mode, seed=args.seed, run=RUN_DIR.name,
               demo_dir=str(cfg.env.get("demo_dir", None)), seconds=round(time.time() - t_all, 1), per_episode=per)
(OUT / "metrics.json").write_text(json.dumps(summary, indent=1))
print(f"[eval r2d {args.mode}] {ns}/{n} = {summary['success']:.3f} -> {OUT}/metrics.json")
