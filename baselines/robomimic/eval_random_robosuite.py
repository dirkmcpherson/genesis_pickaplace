"""G2 negative control: a uniform random policy on the 50-state bank (registered: <= 2/50).

  $LAB/robo_venv/bin/python baselines/robomimic/eval_random_robosuite.py --out $LAB/robomimic_data/eval_random_bank50
"""
import argparse
import pathlib as pl
import sys

import numpy as np

sys.path.insert(0, str(pl.Path(__file__).resolve().parent))
from robo_common import Bank, BANK_PATH, HDF5, HORIZON, ACT_DIM, make_env, run_bank_eval  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--bank", default=str(BANK_PATH))
    ap.add_argument("--episodes", type=int, default=None)
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)
    env, _ = make_env(HDF5["ph"]); bank = Bank(args.bank)
    idx = None if args.episodes is None else list(range(args.episodes))
    run_bank_eval(env, bank, lambda s: rng.uniform(-1, 1, ACT_DIM), args.out, horizon=HORIZON, indices=idx, tag="random",
                  extra=dict(learner="random", mode="sample", seed=args.seed))


if __name__ == "__main__":
    main()
