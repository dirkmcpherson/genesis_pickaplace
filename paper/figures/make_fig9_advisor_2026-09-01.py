#!/usr/bin/env python3
"""fig9: the corrected-world triple gradient, frequentist + Bayesian, one advisor-facing figure.
Left: per-seed rnd-30 success by learner and source (every seed drawn; conventions: colour=learner,
circle=human, triangle=machine). Right: posterior of Delta = mu_human - mu_machine per learner
(hierarchical Beta-Binomial over seeds; draws from analysis/bayes_triple_2026-09-01.py) with the
ROPE band (|Delta|<0.05) shaded. Run bayes_triple first (draws npz in the session scratchpad).
"""
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

DRAWS = sys.argv[1] if len(sys.argv) > 1 else \
    '/tmp/claude-1000/-home-travel-workspace-genesis-pickaplace/65bc5977-e458-4033-a00d-271aecab941d/scratchpad/bayes_draws.npz'
Z = np.load(DRAWS)
C = {'DP': '#1f77b4', 'RLPD': '#d62728', 'WM': '#2ca02c'}
DATA = {
    'DP': ([18, 19, 18, 14, 13, 15, 16, 16, 18, 17], [15, 14, 14, 15, 16, 15, 11, 15, 16, 15]),
    'RLPD': ([19, 1, 19, 19, 1, 21, 20, 19], [18, 19, 17, 19, 20, 0, 20, 11]),
    'WM': ([20, 25, 22, 16, 1, 21, 11, 17], [1, 2, 3, 28, 22, 1, 12, 5]),  # final n=8v8
}
SUB = {'DP': 'DP · sel · 10v10', 'RLPD': 'RLPD · LAST · 8v8', 'WM': 'WM · BEST · 8v8'}
KEY = {'DP': 'DP', 'RLPD': 'RLPD', 'WM': 'WM'}

rng = np.random.default_rng(0)
plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False,
                     'figure.dpi': 150, 'savefig.bbox': 'tight'})
fig, (a, b) = plt.subplots(1, 2, figsize=(10.5, 4.2), gridspec_kw={'width_ratios': [1.1, 1]})

for i, (name, (h, m)) in enumerate(DATA.items()):
    xh, xm = i * 2.6, i * 2.6 + 1
    for x, v, mk in ((xh, h, 'o'), (xm, m, '^')):
        v = np.asarray(v) / 30
        a.scatter(x + rng.uniform(-0.13, 0.13, len(v)), v, marker=mk, s=48,
                  facecolor='white', edgecolor=C[name], linewidth=1.5, zorder=3)
        a.hlines(v.mean(), x - 0.3, x + 0.3, color=C[name], lw=2.6, zorder=4)
    a.text((xh + xm) / 2, -0.145, SUB[name], ha='center', fontsize=8, color=C[name],
           transform=a.get_xaxis_transform())
a.set_xticks([x for i in range(3) for x in (i * 2.6, i * 2.6 + 1)])
a.set_xticklabels(['human', 'machine'] * 3, fontsize=9)
a.set_ylabel('pick success, 30 random ICs')
a.set_ylim(-0.02, 1.0)
a.set_title('per-seed outcomes (corrected world)', fontsize=10)

b.axvspan(-0.05, 0.05, color='0.9', zorder=0)
b.text(0, 0.985, 'ROPE\n(|Δ|<0.05)', ha='center', va='top', fontsize=7.5, color='0.45',
       transform=b.get_xaxis_transform())
for name in DATA:
    d = Z[KEY[name]]
    xs = np.linspace(-0.4, 0.6, 400)
    kde = np.array([np.mean(np.abs(d - x) < 0.012) for x in xs]) / 0.024
    b.plot(xs, kde, color=C[name], lw=2, label=f"{name}: Δ={d.mean():+.2f}, P(Δ>0)={np.mean(d > 0):.2f}")
    lo, hi = np.percentile(d, [2.5, 97.5])
    y = -0.28 - 0.30 * list(DATA).index(name)
    b.plot([lo, hi], [y, y], color=C[name], lw=2.5)
    b.plot(d.mean(), y, 'o', color=C[name], ms=6)
b.axvline(0, color='0.2', lw=1)
b.set_xlabel('Δ = human − machine (posterior, hierarchical Beta-Binomial over seeds)')
b.set_yticks([])
b.set_ylim(-1.25, None)
b.legend(fontsize=8, loc='upper right', frameon=False)
b.set_title('source effect per learner: posterior + 95% CrI', fontsize=10)

fig.suptitle('One world, three learners: the demo-source effect grows with how much the learner\n'
             'uses the demonstrations beyond imitation — DP ≈ indifferent, RLPD null, world model prefers human',
             fontsize=10.5, y=1.04)
fig.savefig('paper/figures/fig9_advisor_triple.png')
fig.savefig('paper/figures/fig9_advisor_triple.pdf')
print('wrote fig9')
