#!/usr/bin/env python3
"""Paper figures 1-5 (single-column format since 2026-09-02, user Q6). Data embedded literally
with provenance; regenerate with python3 + matplotlib (imports the sibling colstyle.py only):

    python3 paper/figures/make_figs_2026-08-31.py

Conventions (memory: run-naming-convention): COLOUR = algorithm, SHAPE = demo source
(human = circle, machine/DP-teacher = triangle). Seed is the statistical unit everywhere; violins
carry every seed as an overlaid point because n is 4-10. Readouts: hold = 15 in-distribution ICs,
rnd = 30 random ICs. Headline stat per PREREG A16 (RLPD) = LAST-checkpoint rnd-30; DP/r2d =
selected/BEST ckpt. DP's human label is "human*" (user Q5): DP trains on the leading-idle-PRUNED
human tapes; the frozen blocks fed that pruned base to every learner (PREREG A24/A25), the v2
blocks feed RLPD/WM the raw tapes. The old wide renders are kept as fig*_wide.{png,pdf}.

Provenance:
  DP        baselines/outputs/dp_final/*/sweep/HEADLINE.txt (cluster), fetched 08-31
  RLPD      paper/harvest_2026-08-31_0014.md §RLPD (g99 wave, old world); A20 pair s40-47
  r2d W3    paper/harvest_2026-08-31_0014.md §r2d re-scores (n12_rescore/*_W3_*)
  stats     analysis/stats.py (Welch CI + exact permutation), runs logged in SESSION_LOG
  v2 DP raw-vs-pruned (former fig3 right panel) now lives in make_fig11_raw_vs_pruned_2026-09-02.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import colstyle as cs
from colstyle import C, MK

rng = cs.setup()

# ---------------------------------------------------------------- data (per-seed)
# RLPD, old world, gamma 0.99, sparse; LAST-ckpt rnd-30 (A16 statistic).
rlpd_last_rnd = {
    'dH':          [21, 19, 21, 17, 20, 25, 24],          # s31-37 (s30 hit 12h limit in final eval)
    'dDP':         [14, 12, 17, 16, 13, 17, 15],          # s30,31,33-37 (s32 final eval missing)
    'dHHfails':    [23, 23, 20, 21, 21, 25, 22, 24],      # s30-37
    'dDPfails':    [19, 15, 3, 18, 15, 20, 20, 19],       # s30-37
    'dHsucc_dup':  [17, 23, 23, 18, 19, 25, 16, 24],      # s30-37 (n=8 top-up 08-31 pm)
    'dDPsucc_dup': [19, 13, 5, 9, 15, 17, 15, 21],        # s30-37
    'dH_w3':       [19, 1, 19, 19, 1, 21, 20, 19],        # A20 corrected world s40-47 (LAST rnd)
    'dDP_w3':      [18, 19, 17, 19, 20, 0, 20, 11],
}
# max critic loss per seed (same arms, s30-37 order); divergence threshold = 1 (A16)
rlpd_maxcl = {
    'dH':          [0.018, 0.016, 0.027, 0.157, 0.138, 0.017, 4.42, 0.019],
    'dDP':         [22.1, 83.6, 0.043, 39.2, 0.03, 5.28, 18.5, 4.56],
    'dHHfails':    [0.019, 0.018, 0.0229, 0.0188, 0.0121, 50.5, 0.823, 0.355],
    'dDPfails':    [0.0231, 0.0916, 83.3, 0.0212, 0.0698, 0.0571, 0.0285, 0.0419],
    'dHsucc_dup':  [0.0795, 0.0254, 0.0289, 0.0837, 3.44, 8.98, 0.0475, 0.0331],  # s30-37 (2/8 >= 1)
    'dDPsucc_dup': [0.147, 4.61, 1.65, 0.0668, 3.49, 26.7, 26.5, 0.115],           # s30-37 (5/8 >= 1)
}
# DP corrected world (matched_w3 gc_kp4_riser3_shelf6), seeds 20-29, selected ckpt
dp_w3 = dict(
    dH_hold=[13, 13, 14, 14, 14, 13, 14, 13, 11, 14], dH_rnd=[18, 19, 18, 14, 13, 15, 16, 16, 18, 17],
    dDP_hold=[13, 14, 12, 14, 11, 13, 13, 14, 14, 13], dDP_rnd=[15, 14, 14, 15, 16, 15, 11, 15, 16, 15])
# DP old world, seeds 10-14, selected ckpt rnd; dHunpruned s32-34 (old world; -0.056 vs dH, perm p 0.36)
# (kept for the record; not plotted since the 09-02 revision -- the v2 raw-vs-pruned block is fig11)
dp_old = dict(dH_rnd=[17, 16, 18, 15, 19], dDP_rnd=[16, 18, 16, 17, 16], dHunpruned_rnd=[13, 17, 16])
# r2dreamer corrected world (W3), dense, 3M, BEST = highest in-job sel among archived ckpts; s80-87 FINAL n=8v8 (09-01)
r2d_w3 = dict(dH_hold=[10, 15, 11, 10, 1, 11, 3, 13], dH_rnd=[20, 25, 22, 16, 1, 21, 11, 17],
              dDP_hold=[1, 2, 2, 15, 13, 0, 9, 6], dDP_rnd=[1, 2, 3, 28, 22, 1, 12, 5])

# human-minus-machine gaps for the summary forest (analysis/stats.py, Welch 95% CI, perm p)
forest = [  # (short label, diff, lo, hi, p_perm, colour, note)
    ('DP* (corr)', 0.060, 0.006, 0.114, 0.041, C['DP'], 'sel ckpt, n=10v10, unadjusted'),
    ('RLPD (old)', 0.205, 0.111, 0.299, 0.002, C['RLPD'], 'LAST ckpt, n=7v7, A16 prereg met'),
    ('RLPD +fails (old)', 0.208, 0.048, 0.368, 0.0009, C['RLPD'], 'LAST, n=8v8; perm 2x1s < 0.001'),
    ('RLPD +dup (old)', 0.213, 0.051, 0.374, 0.013, C['RLPD'], 'LAST, n=8v8'),
    ('RLPD (corr)', -0.021, -0.301, 0.259, 0.983, C['RLPD'], 'LAST, n=8v8, A20 prereg FAILED (null)'),
    ('WM (corr)', 0.246, -0.084, 0.576, 0.133, C['R2D'], 'BEST ckpt, n=8v8; ignition 7/8 v 3/8; A27 n=12 pending'),
]

# ---------------------------------------------------------------- fig 1: RLPD violins (one column)
f, ax = cs.fig(h=2.7)
order = ['dH', 'dDP', 'dHHfails', 'dDPfails', 'dHsucc_dup', 'dDPsucc_dup', 'dH_w3', 'dDP_w3']
xs = [0, 0.85, 2.0, 2.85, 4.0, 4.85, 6.3, 7.15]
for x, k in zip(xs, order):
    src = 'human' if k.startswith('dH') else 'machine'
    cs.violin(ax, x, rlpd_last_rnd[k], C['RLPD'], 0.40 if src == 'human' else 0.18, 30, width=0.7)
    cs.seeds_pts(ax, x, rlpd_last_rnd[k], C['RLPD'], src, 30, rng, s=16, jit=0.1, half=0.25)
for x0, x1, txt in [(0, 0.85, '+0.21\np=.002'), (2.0, 2.85, '+0.21\np<.001'),
                    (4.0, 4.85, '+0.21\np=.013'), (6.3, 7.15, '−0.02\np=.98')]:
    cs.bracket(ax, x0, x1, 0.93, txt, dy=0.02)
ax.axvline(5.6, color='0.6', lw=0.8, ls=':')
ax.set_xticks([0.425, 2.425, 4.425, 6.725])
ax.set_xticklabels(['success\nonly', '+ own\nfails', '+ dup.\nsuccess', 'success\nonly'])
ax.text(2.425, 1.2, 'OLD world (A16)', ha='center', va='bottom', fontsize=7.5, color=C['old'])
ax.text(6.725, 1.2, 'CORRECTED (A20)', ha='center', va='bottom', fontsize=7.5, color=C['w3'])
ax.set_ylabel('success (rnd-30, LAST)')
ax.set_ylim(0, 1.3); ax.set_yticks(np.arange(0, 1.01, 0.25)); ax.set_xlim(-0.6, 7.75)
ax.text(1.0, -0.27, cs.SRC_LEGEND, transform=ax.transAxes, ha='right', va='top', fontsize=7, color='0.3')
ax.set_title('RLPD: human − machine gap is world-dependent')
cs.save(f, 'fig1_rlpd_rnd_violin')

# ---------------------------------------------------------------- fig 2: divergence mechanism (2 rows)
f, (a, b) = cs.fig(h=3.0, nrows=2, sharex=True, gridspec_kw={'height_ratios': [0.8, 1.2], 'hspace': 0.12})
order2 = order[:6]
div = [(np.asarray(rlpd_maxcl[k]) >= 1).mean() for k in order2]
nn = [len(rlpd_maxcl[k]) for k in order2]
xs2 = [0, 0.85, 2.0, 2.85, 4.0, 4.85]
for x, d, n, k in zip(xs2, div, nn, order2):
    a.bar(x, d, width=0.7, color=C['RLPD'], alpha=0.85 if k.startswith('dH') else 0.45)
    a.text(x, d + 0.04, f'{int(round(d * n))}/{n}', ha='center', fontsize=7)
a.set_ylabel('diverged\n(max loss ≥ 1)'); a.set_ylim(0, 1.05); a.set_yticks([0, 0.5, 1])
a.set_title('RLPD critic health by demo source (old world)')
for x, k in zip(xs2, order2):
    src = 'human' if k.startswith('dH') else 'machine'
    v = np.asarray(rlpd_maxcl[k])
    b.scatter(x + rng.uniform(-0.12, 0.12, len(v)), v, marker=MK[src], s=16,
              facecolor='white', edgecolor=C['RLPD'], linewidth=0.9)
b.axhline(1, color='0.3', ls='--', lw=0.8); b.text(1.42, 1.3, 'threshold', fontsize=7, color='0.3', ha='center')
b.set_yscale('log'); b.set_ylabel('max critic loss')
b.set_xticks([0.425, 2.425, 4.425]); b.set_xticklabels(['success only', '+ own fails', '+ dup. success'])
a.text(0.47, 0.95, cs.SRC_LEGEND, transform=a.transAxes, ha='center', va='top', fontsize=7, color='0.3')
cs.save(f, 'fig2_rlpd_divergence')

# ---------------------------------------------------------------- fig 3: DP (corrected world, one panel)
f, a = cs.fig(h=2.3)
for i, (k, denom) in enumerate([('dH_hold', 15), ('dDP_hold', 15), ('dH_rnd', 30), ('dDP_rnd', 30)]):
    x = [0, 1, 2.4, 3.4][i]; src = 'human' if k.startswith('dH') else 'machine'
    cs.violin(a, x, dp_w3[k], C['DP'], 0.40 if src == 'human' else 0.18, denom)
    cs.seeds_pts(a, x, dp_w3[k], C['DP'], src, denom, rng)
a.set_xticks([0, 1, 2.4, 3.4]); a.set_xticklabels([cs.HUMAN_DP, 'machine', cs.HUMAN_DP, 'machine'])
cs.group_label(a, 0.5, 'hold (15 ICs)'); cs.group_label(a, 2.9, 'random (30 ICs)')
cs.bracket(a, 2.4, 3.4, 0.70, '+0.06  p=.041')
a.set_ylim(0, 1.12); a.set_yticks(np.arange(0, 1.01, 0.25)); a.set_ylabel('pick success (sel. ckpt)')
a.set_title('DP: nearly source-indifferent (corrected)')
cs.footnote(f, y=-0.1)
cs.save(f, 'fig3_dp_violin')

# ---------------------------------------------------------------- fig 4: r2dreamer W3
f, (a, b) = cs.fig(h=2.2, ncols=2, sharey=True, gridspec_kw={'wspace': 0.12})
for ax_, hk, rk, ttl in [(a, 'dH_hold', 'dDP_hold', 'hold (15 ICs)'), (b, 'dH_rnd', 'dDP_rnd', 'random (30 ICs)')]:
    denom = 15 if 'hold' in hk else 30
    cs.seeds_pts(ax_, 0, r2d_w3[hk], C['R2D'], 'human', denom, rng, s=26, jit=0.11, half=0.28)
    cs.seeds_pts(ax_, 1, r2d_w3[rk], C['R2D'], 'machine', denom, rng, s=26, jit=0.11, half=0.28)
    ax_.set_xticks([0, 1]); ax_.set_xticklabels(['human', 'machine'])
    ax_.set_xlim(-0.6, 1.6); ax_.set_ylim(0, 1.12); ax_.set_yticks(np.arange(0, 1.01, 0.25))
    cs.group_label(ax_, 0.5, ttl, y=-0.2)
cs.bracket(a, 0, 1, 1.0, 'ignition 7/8 v 3/8')
cs.bracket(b, 0, 1, 1.0, '+0.25  p=.13')
a.set_ylabel('pick success (BEST ckpt)')
f.suptitle('World model: human demos ignite it', y=1.01)
cs.save(f, 'fig4_r2d_w3')

# ---------------------------------------------------------------- fig 5: forest summary
f, ax = cs.fig(h=2.2, w=3.05)   # y labels hang outside the axes; total width lands at 3.35
ys = np.arange(len(forest))[::-1]
for y, (lab, d, lo, hi, p, col, note) in zip(ys, forest):
    ax.plot([lo, hi], [y, y], color=col, lw=1.6)
    ax.plot([d], [y], 'o', color=col, ms=4.5)
    ptxt = 'p<.001' if p < 0.001 else f'p={p:.3f}'.replace('0.', '.')
    ax.text(d, y + 0.2, ptxt, va='bottom', ha='center', fontsize=7, color='0.3')
ax.axvline(0, color='0.2', lw=0.8)
ax.set_yticks(ys); ax.set_yticklabels([r[0] for r in forest], fontsize=7.5)
ax.set_xlabel('human − machine (rnd-30), Welch 95% CI')
ax.set_xlim(-0.35, 0.62); ax.set_xticks([-0.25, 0, 0.25, 0.5]); ax.set_ylim(-0.6, len(forest) - 0.3)
ax.set_title('Human − machine gap by learner')
cs.footnote(f, y=-0.14)
cs.save(f, 'fig5_gap_forest')
