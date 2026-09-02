#!/usr/bin/env python3
"""fig11 (user Q4a): Diffusion Policy on the PRUNED vs RAW human tapes, both worlds, v2 full pool.
dHpruned = matched_*/dHv2 (60/60 tapes, leading-idle-pruned; DP's arm of record, A29);
dH = matched_*/dHv2raw (69 old / 66 corrected, raw, uncapped; A25). Same recipe, seeds 50-57,
selected checkpoint. Top: 30 random ICs (the A29 statistic). Bottom: in-distribution hold =
every demo IC of the raw pool (69 old / 66 corrected). Stats: analysis/stats.py (Welch 95% CI +
exact permutation, 12870 splits), logged in SESSION_LOG 09-02.

Provenance: baselines/outputs/dp_v2full{,w3}/dHv2raw_DP_s5?/sweep/HEADLINE.txt (raw) and
dp_v2fullP{,w3}/dHv2_DP_s5?/sweep/HEADLINE.txt (pruned), fetched 09-02 11:10 (all 8 seeds done).
    python3 paper/figures/make_fig11_raw_vs_pruned_2026-09-02.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import colstyle as cs
from colstyle import C

# seeds 50-57 in order; selected-checkpoint readouts
RND = dict(old_raw=[12, 15, 9, 6, 10, 9, 15, 8], old_pruned=[18, 15, 12, 17, 15, 15, 15, 16],
           w3_raw=[9, 9, 7, 5, 10, 5, 5, 7], w3_pruned=[17, 13, 16, 19, 13, 15, 13, 16])
HOLD = dict(old_raw=[46, 50, 51, 55, 56, 49, 50, 50], old_pruned=[66, 64, 66, 65, 64, 66, 62, 67],
            w3_raw=[55, 56, 51, 59, 57, 53, 50, 55], w3_pruned=[65, 65, 62, 65, 64, 65, 64, 64])
HOLD_N = dict(old=69, w3=66)
# analysis/stats.py, pruned minus raw, 09-02
STAT = dict(rnd_old='+0.16  p=.006', rnd_w3='+0.27  p<.001', hold_old='+0.21  p<.001', hold_w3='+0.15  p<.001')

rng = cs.setup()
f, (a, b) = cs.fig(h=3.3, nrows=2, sharex=True, gridspec_kw={'hspace': 0.35})
XS = [0, 1, 2.4, 3.4]
KEYS = ['old_raw', 'old_pruned', 'w3_raw', 'w3_pruned']
for ax_, data, denom_of, ylab, sk in ((a, RND, lambda k: 30, 'success (rnd-30)', 'rnd'),
                                       (b, HOLD, lambda k: HOLD_N[k.split('_')[0]], 'success (in-dist. hold)', 'hold')):
    for x, k in zip(XS, KEYS):
        cs.violin(ax_, x, data[k], C['DP'], 0.40 if 'pruned' in k else 0.18, denom_of(k))
        cs.seeds_pts(ax_, x, data[k], C['DP'], 'human', denom_of(k), rng)
    ax_.set_ylim(0, 1.16); ax_.set_yticks(np.arange(0, 1.01, 0.25)); ax_.set_ylabel(ylab)
    cs.bracket(ax_, 0, 1, 1.0, STAT[f'{sk}_old']); cs.bracket(ax_, 2.4, 3.4, 1.0, STAT[f'{sk}_w3'])
a.set_title('DP: raw vs pruned human tapes (n=8v8 per world)')
b.set_xticks(XS); b.set_xticklabels(['raw', 'pruned*', 'raw', 'pruned*'])
cs.group_label(b, 0.5, 'old world', y=-0.22, color=C['old']); cs.group_label(b, 2.9, 'corrected world', y=-0.22, color=C['w3'])
f.text(0.0, -0.02, '*pruned = DP\'s human arm of record (dHpruned); raw = the\n'
       'RLPD/WM human arm (dH). hold = every demo IC of the raw\n'
       'pool (69 old / 66 corr.); rnd = 30 random ICs.',
       ha='left', va='top', fontsize=7, color='0.3', transform=f.transFigure)
cs.save(f, 'fig11_dp_raw_vs_pruned')
for k in KEYS:
    print(k, 'rnd %.3f' % (np.mean(RND[k]) / 30), 'hold %.3f' % (np.mean(HOLD[k]) / HOLD_N[k.split('_')[0]]))
