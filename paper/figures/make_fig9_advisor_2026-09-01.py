#!/usr/bin/env python3
"""fig9: the corrected-world triple gradient, frequentist + Bayesian. Single-column format since
09-02: fig9a = per-seed rnd-30 success by learner and source (every seed drawn; colour = learner,
circle = human, triangle = machine; DP's human arm is "human*" = pruned tapes), fig9b = posterior of
Delta = mu_human - mu_machine per learner (hierarchical Beta-Binomial over seeds; draws from
analysis/bayes_triple_2026-09-01.py, no flags -- s84 is pinned inside it) with the ROPE band (|Delta|<0.05) shaded.
Run bayes_triple first (it writes paper/figures/bayes_draws_2026-09-01.npz next to this script), then:
    python3 paper/figures/make_fig9_advisor_2026-09-01.py [draws.npz]
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import colstyle as cs

DRAWS = sys.argv[1] if len(sys.argv) > 1 else \
    os.path.join(os.path.dirname(os.path.abspath(__file__)), 'bayes_draws_2026-09-01.npz')   # written by bayes_triple (no flags)
Z = np.load(DRAWS)
C = {'DP': cs.C['DP'], 'RLPD': cs.C['RLPD'], 'WM': cs.C['WM']}
DATA = {
    'DP': ([18, 19, 18, 14, 13, 15, 16, 16, 18, 17], [15, 14, 14, 15, 16, 15, 11, 15, 16, 15]),
    'RLPD': ([19, 1, 19, 19, 1, 21, 20, 19], [18, 19, 17, 19, 20, 0, 20, 11]),
    'WM': ([20, 25, 22, 16, 1, 21, 11, 17], [1, 2, 3, 28, 22, 1, 12, 5]),  # final n=8v8
}
SUB = {'DP': 'DP', 'RLPD': 'RLPD', 'WM': 'WM'}   # ckpt rule + n in the caption: sel 10v10 / LAST 8v8 / BEST 8v8
rng = cs.setup()

# ---- fig9a: per-seed outcomes
f, a = cs.fig(h=2.3)
ticks, labs = [], []
for i, (name, (h, m)) in enumerate(DATA.items()):
    xh, xm = i * 2.9, i * 2.9 + 1.15
    cs.seeds_pts(a, xh, h, C[name], 'human', 30, rng, s=20, jit=0.13, half=0.3)
    cs.seeds_pts(a, xm, m, C[name], 'machine', 30, rng, s=20, jit=0.13, half=0.3)
    cs.group_label(a, (xh + xm) / 2, SUB[name], y=-0.2, color=C[name], fs=8)
    ticks += [xh, xm]; labs += ['H*' if name == 'DP' else 'H', 'M']
a.set_xticks(ticks); a.set_xticklabels(labs)
a.set_ylabel('success (rnd-30)'); a.set_ylim(-0.02, 1.0); a.set_xlim(-0.7, 7.65)
a.set_title('Per-seed outcomes, corrected world')
a.text(0.99, 0.98, 'H human · M machine', transform=a.transAxes, ha='right', va='top', fontsize=7, color='0.3')
cs.footnote(f, y=-0.17)
cs.save(f, 'fig9a_triple_seeds')

# ---- fig9b: posterior of the source effect per learner
f, b = cs.fig(h=2.3)
b.axvspan(-0.05, 0.05, color='0.9', zorder=0)
b.text(0, -1.45, 'ROPE', ha='center', va='center', fontsize=7, color='0.45')
xs = np.linspace(-0.4, 0.6, 400)
for j, name in enumerate(DATA):
    d = Z[name]
    kde = np.array([np.mean(np.abs(d - x) < 0.012) for x in xs]) / 0.024
    lab = (cs.HUMAN_DP.replace('human', 'DP') if name == 'DP' else name)
    b.plot(xs, kde, color=C[name], lw=1.6, label=f"{lab}: Δ={d.mean():+.2f}, P(Δ>0)={np.mean(d > 0):.2f}")
    lo, hi = np.percentile(d, [2.5, 97.5]); y = -0.28 - 0.30 * j
    b.plot([lo, hi], [y, y], color=C[name], lw=2); b.plot(d.mean(), y, 'o', color=C[name], ms=4)
b.axvline(0, color='0.2', lw=0.8)
b.set_xlabel('Δ = human − machine'); b.set_yticks([]); b.set_ylim(-1.65, None)
b.spines['left'].set_visible(False)
b.legend(loc='upper left', fontsize=7)
b.set_title('Posterior of the source effect (95% CrI)')
cs.footnote(f, y=-0.14)
cs.save(f, 'fig9b_triple_posterior')
