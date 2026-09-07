"""Fresh-process Diffusion Policy evaluator on the 50-state bank (DP amendment, plan §A1).

Loads a lerobot DP checkpoint (pretrained_model dir) exactly like baselines/dp_runner.py (policy +
saved pre/post processors, device override), feeds state[:9] -> observation.state and state[9:] ->
observation.environment_state, and executes the 7-dim [-1,1] OSC action directly (no integrator: the
env's action space IS the data's). DP samples diffusion noise: per-episode torch seed = bank index (a
distinct, reproducible draw per IC, as cluster/eval_sweep.sh does); mode is recorded as 'sample'.

  $LAB/robo_venv/bin/python baselines/robomimic/eval_dp_robosuite.py --checkpoint <run>/checkpoints/last/pretrained_model --out <run>/eval_bank50
"""
import argparse
import json
import pathlib as pl
import sys

import numpy as np

sys.path.insert(0, str(pl.Path(__file__).resolve().parent))
from robo_common import Bank, BANK_PATH, HORIZON, PROPRIO_DIM, make_env, run_bank_eval, sha256_file  # noqa: E402

TASK = "pick the can and place it in its target bin"


def load_dp(checkpoint, device):
    import torch
    from lerobot.policies.factory import make_pre_post_processors
    from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
    policy = DiffusionPolicy.from_pretrained(checkpoint); policy.eval(); policy.to(device)
    pre, post = make_pre_post_processors(policy_cfg=policy.config, pretrained_path=checkpoint,
                                         preprocessor_overrides={"device_processor": {"device": str(device)}})
    proprio = policy.config.input_features["observation.state"].shape[0]
    assert proprio == PROPRIO_DIM, (proprio, PROPRIO_DIM)
    assert not any(k.startswith("observation.images") for k in policy.config.input_features), "state-only DP expected"

    def act(s):
        batch = {"observation.state": torch.from_numpy(s[:proprio]).float().unsqueeze(0).to(device),
                 "observation.environment_state": torch.from_numpy(s[proprio:]).float().unsqueeze(0).to(device), "task": [TASK]}
        batch = pre(batch); batch = {k: (v.to(device) if torch.is_tensor(v) else v) for k, v in batch.items()}
        with torch.no_grad():
            a = policy.select_action(batch)
        return post(a).squeeze(0).cpu().numpy()

    return act, policy.reset, policy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True, help="lerobot pretrained_model dir")
    ap.add_argument("--out", required=True)
    ap.add_argument("--bank", default=str(BANK_PATH))
    ap.add_argument("--episodes", type=int, default=None)
    ap.add_argument("--device", default=None)
    ap.add_argument("--hdf5", default=None)
    args = ap.parse_args()
    import torch
    device = args.device or ("cuda" if torch.cuda.is_available() else "cpu")
    ck = pl.Path(args.checkpoint)
    side_p = ck.parent / "dp_sidecar.json"
    side = json.loads(side_p.read_text()) if side_p.exists() else {}
    act, reset, policy = load_dp(str(ck), device)
    from robo_common import HDF5
    env, _ = make_env(args.hdf5 or side.get("hdf5") or HDF5["ph"]); bank = Bank(args.bank)
    idx = list(range(bank.n)) if args.episodes is None else list(range(args.episodes))
    state = {"k": 0}

    def reset_fn():
        torch.manual_seed(idx[state["k"]]); state["k"] += 1; reset()

    run_bank_eval(env, bank, act, args.out, reset_fn=reset_fn, horizon=int(side.get("horizon", HORIZON)), indices=idx, tag="dp sample",
                  extra=dict(learner="dp", checkpoint=str(ck), mode="sample", act_selection="sampled(seed=bank index)", device=device,
                             arm=side.get("arm"), sidecar=side, n_action_steps=int(policy.config.n_action_steps),
                             n_obs_steps=int(policy.config.n_obs_steps), horizon_dp=int(policy.config.horizon)))


if __name__ == "__main__":
    main()
