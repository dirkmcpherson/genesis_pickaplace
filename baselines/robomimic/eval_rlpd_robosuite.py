"""Fresh-process RLPD evaluator on the 50-state bank (plan §2/§5). Deterministic actions (mode) by default,
--mode sample for the secondary statistic. metrics.json in our layout via robo_common.run_bank_eval.

  $LAB/robo_venv/bin/python baselines/robomimic/eval_rlpd_robosuite.py --checkpoint <run>/rlpd_final.zip --mode mode --out <run>/eval_bank50_mode
"""
import argparse
import json
import os
import pathlib as pl
import sys

import numpy as np

HERE = pl.Path(__file__).resolve().parent
REPO = pl.Path(os.environ.get("GENESIS_PICKAPLACE_ROOT", HERE.parents[1]))
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(REPO / "baselines" / "rl"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--mode", choices=["mode", "sample"], default="mode")
    ap.add_argument("--bank", default=None)
    ap.add_argument("--episodes", type=int, default=None, help="first N bank entries (default all)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    import torch as th
    th.manual_seed(args.seed); np.random.seed(args.seed)
    import rlpd_sac  # noqa: F401  (policy_class unpickle)
    from stable_baselines3 import SAC
    from robo_common import Bank, BANK_PATH, HORIZON, make_env, run_bank_eval
    ck = pl.Path(args.checkpoint)
    side = ck.with_name(ck.stem + ".sidecar.json")
    assert side.exists(), f"FATAL: no sidecar {side} (a checkpoint of record carries one)"
    side = json.loads(side.read_text())
    model = SAC.load(str(ck), device="cpu")
    env, meta = make_env(side["hdf5"])
    bank = Bank(args.bank or BANK_PATH)
    det = args.mode == "mode"

    def policy(s):
        a, _ = model.predict(s.astype(np.float32), deterministic=det)
        return a

    idx = None if args.episodes is None else list(range(args.episodes))
    run_bank_eval(env, bank, policy, args.out, horizon=int(side.get("horizon", HORIZON)), indices=idx, tag=f"rlpd {args.mode}",
                  extra=dict(learner="rlpd", checkpoint=str(ck), mode=args.mode, seed=args.seed, arm=side.get("arm"),
                             ckpt_step=side.get("ckpt_step"), sidecar=side))


if __name__ == "__main__":
    main()
