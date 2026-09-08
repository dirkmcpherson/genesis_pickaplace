"""Is the human demonstration set measurably MULTI-MODAL and the machine set not? (2026-09-08)

Motivation: the A5 fixed-head cells show a data x architecture INTERACTION — a 5-mode GMM head helps the human arm
(+~0.34) and hurts the machine arm (-~0.30). The proposed mechanism is that human demonstrations are multi-modal by
construction (several operators, several strategies for one task) while machine demonstrations come from a single
converged SAC policy and should be close to unimodal. This script tests that directly instead of asserting it.

Statistic (self-calibrating, no sklearn): for each of A anchor rows, take the k nearest rows IN STATE SPACE within the
same arm (states standardised with a scaler fit on the POOLED arms, so all arms are measured on one metric), project the
k action vectors onto their first principal component, and compute the 2-means split gain g = 1 - W2/W1 along that axis.
A unimodal cloud already yields g > 0, so the number reported is the EXCESS over a matched unimodal null: fit a Gaussian
to the same k actions, resample k points R times, recompute g, and subtract the mean. Positive excess = structure a
single Gaussian does not explain, i.e. multi-modality of the conditional action distribution p(a | s).

Also reported for the multi-operator human arm: whether neighbourhood action disagreement is BETWEEN operators
(same-operator vs different-operator action distance), the direct form of "several operators, several strategies".

  $LAB/robo_venv/bin/python baselines/robomimic/action_multimodality.py --arms PH200,MH200,MG200s,MG718s
"""
import argparse, json, pathlib as pl, sys
import numpy as np

sys.path.insert(0, str(pl.Path(__file__).resolve().parent))
from robo_common import DATA_ROOT  # noqa: E402


def split_gain(x):
    """1 - W2/W1 for 2-means on the PC1 projection of x (n,d)."""
    xc = x - x.mean(0)
    u, s, vt = np.linalg.svd(xc, full_matrices=False)
    p = xc @ vt[0]
    W1 = float(((p - p.mean()) ** 2).sum())
    if W1 <= 1e-12:
        return 0.0
    q = np.sort(p)
    best = W1
    for i in range(2, len(q) - 1):
        a, b = q[:i], q[i:]
        w = ((a - a.mean()) ** 2).sum() + ((b - b.mean()) ** 2).sum()
        best = min(best, w)
    return float(1.0 - best / W1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arms", default="PH200,MH200,MG200s,MG718s")
    ap.add_argument("--anchors", type=int, default=1200)
    ap.add_argument("--k", type=int, default=25)
    ap.add_argument("--resamples", type=int, default=3)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--arms-root", default=str(DATA_ROOT / "arms"))
    ap.add_argument("--out", default=str(DATA_ROOT / "action_multimodality.json"))
    args = ap.parse_args()
    root = pl.Path(args.arms_root)
    arms = args.arms.split(",")
    data = {}
    for a in arms:
        z = np.load(root / a / "rlpd" / "transitions.npz")
        data[a] = dict(obs=z["obs"], act=z["act"], tape=z["tape_id"])
    pooled = np.concatenate([d["obs"] for d in data.values()])
    mu, sd = pooled.mean(0), pooled.std(0) + 1e-8
    out = {}
    rng = np.random.default_rng(args.seed)
    for a in arms:
        S = (data[a]["obs"] - mu) / sd
        A = data[a]["act"]
        idx = rng.choice(len(S), size=min(args.anchors, len(S)), replace=False)
        gains, nulls, spreads = [], [], []
        for j0 in range(0, len(idx), 200):                       # chunked brute-force kNN
            q = S[idx[j0:j0 + 200]]
            d2 = ((q[:, None, :] - S[None, :, :]) ** 2).sum(-1)
            nn = np.argpartition(d2, args.k, axis=1)[:, : args.k]
            for r in range(len(q)):
                acts = A[nn[r]]
                g = split_gain(acts); gains.append(g)
                m, c = acts.mean(0), acts.std(0) + 1e-8
                nulls.append(np.mean([split_gain(rng.normal(m, c, size=acts.shape)) for _ in range(args.resamples)]))
                spreads.append(float(np.mean(np.std(acts, axis=0))))
        gains, nulls, spreads = map(np.asarray, (gains, nulls, spreads))
        exc = gains - nulls
        out[a] = dict(n_rows=int(len(S)), anchors=int(len(idx)), k=args.k,
                      gain_mean=round(float(gains.mean()), 4), null_mean=round(float(nulls.mean()), 4),
                      excess_mean=round(float(exc.mean()), 4), excess_median=round(float(np.median(exc)), 4),
                      frac_excess_gt_05=round(float((exc > 0.05).mean()), 4),
                      neighbourhood_action_sd=round(float(spreads.mean()), 4))
        print(f"{a:8s} rows {out[a]['n_rows']:7d} | conditional-action split gain {out[a]['gain_mean']:.3f} "
              f"vs unimodal null {out[a]['null_mean']:.3f} -> EXCESS {out[a]['excess_mean']:+.3f} "
              f"(median {out[a]['excess_median']:+.3f}, frac>0.05 {out[a]['frac_excess_gt_05']:.3f}) | "
              f"neighbourhood action sd {out[a]['neighbourhood_action_sd']:.3f}", flush=True)
    json.dump(dict(config=vars(args), arms=out), open(args.out, "w"), indent=1)
    print(f"[multimodality] wrote {args.out}", flush=True)


if __name__ == "__main__":
    main()
