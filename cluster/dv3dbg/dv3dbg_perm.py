#!/usr/bin/env python3
"""DV3 DEBUG G3 statistic (paper/DV3_DEBUG_2026-09-05.md §9): per-seed cells, exact two-sided permutation test on
per-seed success counts, and the minimum detectable effect (MDE) at that n so a null is reported with its power.

usage: dv3dbg_perm.py <runs_root> [--set rnd30|hold15] [--suffix _recov|_mode|''] [--arms dHv2raw,dDP]
MDE: smallest |Delta| in per-seed success probability for which the same exact test rejects at alpha=0.05 with
power >= 0.80, simulated with per-seed counts ~ Binomial(n_ep, p_i) and p_i ~ Normal(arm mean, seed SD) truncated to
[0,1]; the seed SD is estimated from the observed per-seed rates (pooled over arms, >= 0.02 to avoid a zero-variance
fantasy). Reports the test's own resolution too: the smallest attainable p at this n."""
import json, glob, os, sys, argparse, random
from itertools import combinations

ap = argparse.ArgumentParser()
ap.add_argument("root"); ap.add_argument("--set", default="rnd30"); ap.add_argument("--suffix", default="")
ap.add_argument("--arms", default="dHv2raw,dDP"); ap.add_argument("--tag", default="clamp1")
ap.add_argument("--sims", type=int, default=4000); ap.add_argument("--seed", type=int, default=0)
a = ap.parse_args()

def cells(arm):
    out = []
    for d in sorted(glob.glob(os.path.join(a.root, f"dv3dbg_pick_{arm}_{a.tag}_s*"))):
        if d.endswith("_smoke"): continue
        f = os.path.join(d, f"fresh_eval_{a.set}{a.suffix}", "metrics.json")
        if not os.path.exists(f): continue
        m = json.load(open(f)); n = int(m["policy_eval/n"]); k = round(m["policy_eval/picked"] * n)
        out.append((os.path.basename(d).split("_s")[-1], k, n))
    return out

arms = a.arms.split(",")
data = {arm: cells(arm) for arm in arms}
for arm in arms:
    print(f"{arm}: " + "  ".join(f"s{s}={k}/{n} ({k/n:.3f})" for s, k, n in data[arm]) or f"{arm}: NO CELLS")
if any(len(v) == 0 for v in data.values()):
    sys.exit("missing cells for at least one arm")
A, B = [ [k for _, k, _ in data[arm]] for arm in arms ]
NEP = data[arms[0]][0][2]
rate = lambda v: sum(v) / (NEP * len(v))
obs = rate(A) - rate(B)
allv = A + B; nA = len(A)
def pval(vals, nA, obs):
    tot = hit = 0
    for idx in combinations(range(len(vals)), nA):
        x = [vals[i] for i in idx]; y = [vals[i] for i in range(len(vals)) if i not in idx]
        tot += 1
        if abs(rate(x) - rate(y)) >= abs(obs) - 1e-12: hit += 1
    return hit, tot
hit, tot = pval(allv, nA, obs)
print(f"\n{arms[0]} {sum(A)}/{NEP*len(A)} = {rate(A):.3f}  vs  {arms[1]} {sum(B)}/{NEP*len(B)} = {rate(B):.3f}"
      f"   Delta {obs:+.3f}   exact two-sided p = {hit}/{tot} = {hit/tot:.4f}   (n = {len(A)} v {len(B)}, {NEP} episodes/seed)")
minp = 2 / tot
print(f"test resolution at this n: smallest attainable p = 2/{tot} = {minp:.4f}"
      + ("  (so p<0.05 requires the single most extreme arrangement)" if minp <= 0.05 < 4 / tot else ""))

rates = [k / NEP for k in A] + [k / NEP for k in B]
mu = sum(rates) / len(rates)
sd = (sum((r - mu) ** 2 for r in rates) / max(1, len(rates) - 1)) ** 0.5
sd = max(sd, 0.02)
rng = random.Random(a.seed)
def power(delta, base):
    rej = 0
    for _ in range(a.sims):
        sa = [sum(1 for _ in range(NEP) if rng.random() < min(1, max(0, rng.gauss(base + delta, sd)))) for _ in range(len(A))]
        sb = [sum(1 for _ in range(NEP) if rng.random() < min(1, max(0, rng.gauss(base, sd)))) for _ in range(len(B))]
        o = rate(sa) - rate(sb)
        h, t = pval(sa + sb, len(sa), o)
        if h / t <= 0.05: rej += 1
    return rej / a.sims
base = rate(B)
mde = None
for d in [x / 100 for x in range(2, 81, 2)]:
    pw = power(d, base)
    if pw >= 0.80: mde = (d, pw); break
print(f"MDE (alpha 0.05 two-sided, power 0.80, {a.sims} sims, seed SD {sd:.3f}, base rate {base:.3f}): "
      + (f"|Delta| = {mde[0]:.2f} (power {mde[1]:.2f})" if mde else "> 0.80 — no effect below 0.80 is detectable at this n"))
