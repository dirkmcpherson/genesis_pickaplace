"""Fresh-process BC-RNN evaluator on the 50-state bank (G1: PH200 >= 0.90).

Loads a robomimic checkpoint with FileUtils.policy_from_checkpoint (RolloutPolicy: its own obs normalisation
and RNN state), calls policy.start_episode() per episode and policy(ob=env.get_observation()) per decision
(robomimic's run_trained_agent convention; low_noise_eval => the GMM mode). Same bank, horizon, success rule
and metrics.json layout as the other learners.

  $LAB/robo_venv/bin/python baselines/robomimic/eval_bcrnn_robosuite.py --checkpoint <run>/.../models/model_epoch_2000.pth --out <run>/eval_bank50
"""
import argparse
import json
import pathlib as pl
import sys

import numpy as np

sys.path.insert(0, str(pl.Path(__file__).resolve().parent))
from robo_common import Bank, BANK_PATH, HORIZON, HDF5, make_env, run_bank_eval  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--bank", default=str(BANK_PATH))
    ap.add_argument("--episodes", type=int, default=None)
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--arm", default=None)
    args = ap.parse_args()
    import torch
    import robomimic.utils.file_utils as FileUtils
    torch.manual_seed(0); np.random.seed(0)
    policy, ckpt = FileUtils.policy_from_checkpoint(ckpt_path=args.checkpoint, device=torch.device(args.device), verbose=False)
    env, _ = make_env(HDF5["ph"]); bank = Bank(args.bank)
    idx = None if args.episodes is None else list(range(args.episodes))

    def policy_fn(_s):
        return policy(ob=env.get_observation())

    epoch = ckpt.get("epoch", None) if isinstance(ckpt, dict) else None
    run_bank_eval(env, bank, policy_fn, args.out, reset_fn=policy.start_episode, horizon=HORIZON, indices=idx, tag="bcrnn",
                  extra=dict(learner="bcrnn", checkpoint=str(args.checkpoint), mode="mode", arm=args.arm, epoch=epoch,
                             algo=ckpt.get("algo_name") if isinstance(ckpt, dict) else None))


if __name__ == "__main__":
    main()
