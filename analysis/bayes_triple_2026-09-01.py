#!/usr/bin/env python3
"""Frequentist + Bayesian analysis of the corrected-world triple gradient (advisor briefing).

Per-seed data of record (every row cites its artifact line; RESULTS = paper/RESULTS_for_writing_2026-08-30.md):
  DP   selected ckpt, rnd-30, dH/dDP s20-29   RESULTS §1 (0.547/0.487); per-seed in paper/figures/make_figs_2026-08-31.py:53-54
  RLPD LAST ckpt,     rnd-30, dH/dDP s40-47   RESULTS §2.3 "Per-seed LAST rnd"
  WM   BEST ckpt,     rnd-30, dH/dDP s80-87   RESULTS §3.1 "n=8v8 FINAL (09-01)": dH 20,25,22,16,1,21,11,17 / dDP 1,2,3,28,22,1,12,5
       dH s84 BEST rnd = 1 is the pinned re-score (RESCORE-RESULT ..._dH_s84_W3 set=rnd picked=1/30); it is HARD-CODED
       here. AUDIT_approach_2026-09-02 finding 4: the earlier `--s84-best-rnd` flag defaulted to None and the script
       silently printed the WITHDRAWN n=7v8 row (CrI [+0.07,+0.49]); there is no flag any more and the WM row is
       asserted to be n=8 v 8 before anything is printed.
Model (Bayesian): seed k of arm a scores y_ak ~ Binomial(n_eval, theta_ak); theta_ak ~ Beta(mu_a*kappa,
(1-mu_a)*kappa) -- hierarchical over seeds (the seed is the unit; overdispersion absorbed by kappa).
Priors: mu ~ Uniform(0,1), kappa ~ Gamma(2, 0.1) (mean 20, weak). Posterior via grid+MC importance sampling
(no external deps). Reported per learner: posterior mean/95% CrI of Delta = mu_H - mu_M, P(Delta>0), and
ROPE P(|Delta|<0.05). Ignition (WM): Beta(1,1) conjugate on ignition counts. Frequentist companions follow
analysis/stats.py conventions (Welch + exact permutation; here Monte-Carlo permutation, 1e5 reps).
NOTE (AUDIT_approach finding 5): P(Delta>0) under a flat prior is ~ 1 - one-sided p; it is not evidence
independent of the permutation test. Multiplicity (Holm over hold/rnd) is in analysis/decomposition_2026-09-02.py.
Run (no flags):  python3 analysis/bayes_triple_2026-09-01.py
Output: paper/figures/bayes_draws_2026-09-01.npz (next to fig9; make_fig9_advisor_2026-09-01.py reads it).
"""
import os
import numpy as np

rng = np.random.default_rng(1)
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'paper', 'figures', 'bayes_draws_2026-09-01.npz')

N_RND = 30
DATA = {  # corrected world, human vs machine, per-seed successes on rnd-30 (sources in the docstring)
    'DP (selected ckpt)': (
        [18, 19, 18, 14, 13, 15, 16, 16, 18, 17],            # dH s20-29
        [15, 14, 14, 15, 16, 15, 11, 15, 16, 15]),           # dDP s20-29
    'RLPD (LAST ckpt)': (
        [19, 1, 19, 19, 1, 21, 20, 19],                      # dH s40-47
        [18, 19, 17, 19, 20, 0, 20, 11]),                    # dDP s40-47
    'WM r2dreamer (BEST ckpt)': (
        [20, 25, 22, 16, 1, 21, 11, 17],                     # dH s80-87 (s84 = 1, pinned re-score)
        [1, 2, 3, 28, 22, 1, 12, 5]),                        # dDP s80-87
}
EXPECT_N = {'DP (selected ckpt)': (10, 10), 'RLPD (LAST ckpt)': (8, 8), 'WM r2dreamer (BEST ckpt)': (8, 8)}
WM_IGNITION = ((7, 8), (3, 8))   # BEST hold >= 8/15: dH 7/8, dDP 3/8 (s84 NOT ignited: BEST hold 1/15)

for name, (h, m) in DATA.items():          # the n=7v8 row must never be printable again
    assert (len(h), len(m)) == EXPECT_N[name] and None not in h and None not in m, (name, len(h), len(m))


def hier_posterior(y, n_ev=N_RND, draws=60000):
    """Importance-sample (mu, kappa) for the hierarchical Beta-Binomial; return mu draws."""
    y = np.asarray(y, float)
    mu = rng.uniform(0.005, 0.995, draws)
    kap = rng.gamma(2.0, 10.0, draws)
    a = mu * kap; b = (1 - mu) * kap
    from math import lgamma
    def betaln(x, y):
        xa, ya = np.asarray(x, float), np.asarray(y, float)
        lg = np.vectorize(lgamma)
        return lg(xa) + lg(ya) - lg(xa + ya)
    ll = np.zeros(draws)
    for yk in y:
        ll += betaln(a + yk, b + n_ev - yk) - betaln(a, b)
    w = np.exp(ll - ll.max()); w /= w.sum()
    ess = 1.0 / np.sum(w ** 2)
    assert ess > 500, f'importance-sampling ESS too low ({ess:.0f}); raise draws'
    idx = rng.choice(draws, size=20000, p=w)
    return mu[idx]


def perm_p(x, y, reps=100000):
    x, y = np.asarray(x, float), np.asarray(y, float)
    obs = x.mean() - y.mean()
    pool = np.concatenate([x, y]); nx = len(x)
    cnt = 0
    for _ in range(reps):
        rng.shuffle(pool)
        if abs(pool[:nx].mean() - pool[nx:].mean()) >= abs(obs) - 1e-12:
            cnt += 1
    return cnt / reps


print(f"{'learner':<28} {'n':>5} {'dH':>6} {'dM':>6} {'Δ(freq)':>8} {'perm p':>7} | {'Δ(post)':>8} {'95% CrI':>18} {'P(Δ>0)':>7} {'ROPE':>6}")
results = {}
for name, (hv, mv) in DATA.items():
    hf = np.mean(hv) / N_RND; mf = np.mean(mv) / N_RND
    p = perm_p(hv, mv)
    muH = hier_posterior(hv); muM = hier_posterior(mv)
    d = muH - muM
    lo, hi = np.percentile(d, [2.5, 97.5])
    results[name] = dict(d=d, hv=hv, mv=mv)
    print(f"{name:<28} {len(hv)}v{len(mv):<2} {hf:6.3f} {mf:6.3f} {hf-mf:+8.3f} {p:7.4f} | {d.mean():+8.3f} [{lo:+6.3f},{hi:+6.3f}] {(d>0).mean():7.3f} {(np.abs(d)<0.05).mean():6.3f}")

# WM ignition: conjugate Beta(1,1)
(hi_k, hi_n), (mi_k, mi_n) = WM_IGNITION
pH = rng.beta(1 + hi_k, 1 + hi_n - hi_k, 20000)
pM = rng.beta(1 + mi_k, 1 + mi_n - mi_k, 20000)
from math import comb
# Fisher exact two-sided (hypergeometric)
def fisher(a, b, c, d_):
    n = a + b + c + d_; r1, c1 = a + b, a + c
    def pmf(x): return comb(c1, x) * comb(n - c1, r1 - x) / comb(n, r1)
    p0 = pmf(a)
    return sum(pmf(x) for x in range(max(0, r1 + c1 - n), min(r1, c1) + 1) if pmf(x) <= p0 + 1e-12)
print(f"\nWM ignition (BEST hold>=8/15): dH {hi_k}/{hi_n} vs dDP {mi_k}/{mi_n} | Fisher p {fisher(hi_k, hi_n-hi_k, mi_k, mi_n-mi_k):.3f} | "
      f"P(pH>pM) {(pH>pM).mean():.3f} | Δignition post mean {+(pH-pM).mean():.2f} CrI [{np.percentile(pH-pM,2.5):+.2f},{np.percentile(pH-pM,97.5):+.2f}]")
np.savez(OUT, **{k.split(' ')[0]: v['d'] for k, v in results.items()})
print(f"\nposterior draws saved for fig9 -> {os.path.normpath(OUT)}")
