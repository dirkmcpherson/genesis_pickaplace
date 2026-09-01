#!/usr/bin/env python3
"""Paper figures, 2026-08-31 post-blackout state. Data embedded literally with provenance;
regenerate with any python3 + matplotlib (no repo imports):

    python3 paper/figures/make_figs_2026-08-31.py

Conventions (memory: run-naming-convention): COLOUR = algorithm, SHAPE = demo source
(human = circle, DP-teacher = triangle). Seed is the statistical unit everywhere; violins
carry every seed as an overlaid point because n is 4-10 (a violin without its points would
overstate the sample). Readouts: hold = 15 in-distribution ICs, rnd = 30 random ICs.
Headline stat per PREREG A16 (RLPD) = LAST-checkpoint rnd-30; DP/r2d = selected/BEST ckpt.

Provenance:
  DP        baselines/outputs/dp_final/*/sweep/HEADLINE.txt (cluster), fetched 08-31
  RLPD      paper/harvest_2026-08-31_0014.md §RLPD (g99 wave, old world)
  r2d W3    paper/harvest_2026-08-31_0014.md §r2d re-scores (n12_rescore/*_W3_*)
  stats     analysis/stats.py (Welch CI + exact permutation), runs logged in SESSION_LOG
"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

OUT = 'paper/figures'
C = dict(DP='#1f77b4', RLPD='#d62728', R2D='#2ca02c', DV3='#9467bd')
MK = dict(human='o', machine='^')  # shape = source

# ---------------------------------------------------------------- data (per-seed)
# RLPD, old world, gamma 0.99, sparse; LAST-ckpt rnd-30 (A16 statistic).
rlpd_last_rnd = {
    'dH':          [21, 19, 21, 17, 20, 25, 24],          # s31-37 (s30 hit 12h limit in final eval)
    'dDP':         [14, 12, 17, 16, 13, 17, 15],          # s30,31,33-37 (s32 final eval missing)
    'dHHfails':    [23, 23, 20, 21, 21, 25, 22, 24],      # s30-37
    'dDPfails':    [19, 15, 3, 18, 15, 20, 20, 19],       # s30-37
    'dHsucc_dup':  [17, 23, 23, 18, 19, 25, 16, 24],      # s30-37 (n=8 top-up 08-31 pm)
    'dDPsucc_dup': [19, 13, 5, 9, 15, 17, 15, 21],        # s30-37
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
dp_old = dict(dH_rnd=[17, 16, 18, 15, 19], dDP_rnd=[16, 18, 16, 17, 16], dHunpruned_rnd=[13, 17, 16])
# r2dreamer corrected world (W3), dense, 3M, BEST = highest in-job sel among archived ckpts; s80-83
r2d_w3 = dict(dH_hold=[10, 15, 11, 10], dH_rnd=[20, 25, 22, 16],
              dDP_hold=[1, 2, 2, 15], dDP_rnd=[1, 2, 3, 28])

# human-minus-DP gaps for the summary forest (analysis/stats.py, Welch 95% CI, perm p)
forest = [  # (label, diff, lo, hi, p_perm, colour, note)
    ('DP  (corrected world, selected rnd, n=10 v 10)', 0.060, 0.006, 0.114, 0.041, C['DP'], 'unadjusted'),
    ('RLPD sparse, OLD world  (LAST rnd, n=7 v 7)', 0.205, 0.111, 0.299, 0.002, C['RLPD'], 'A16 prereg, met'),
    ('RLPD +fails, OLD world  (LAST rnd, n=8 v 8)', 0.208, 0.048, 0.368, 0.0009, C['RLPD'], ''),  # perm 2x1s < 0.001
    ('RLPD +succ_dup, OLD world  (LAST rnd, n=8 v 8)', 0.213, 0.051, 0.374, 0.013, C['RLPD'], ''),
    ('RLPD sparse, CORRECTED world  (LAST rnd, n=8 v 8)', -0.021, -0.301, 0.259, 0.983, C['RLPD'], 'A20 prereg, FAILED'),
    ('r2dreamer  (corrected world, BEST rnd, n=4 v 4)', 0.408, -0.256, 1.073, 0.143, C['R2D'], 'n=8 ~09-01'),
]

plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False,
                     'figure.dpi': 150, 'savefig.bbox': 'tight'})
rng = np.random.default_rng(0)


def seeds_pts(ax, x, vals, colour, source, denom):
    v = np.asarray(vals) / denom
    ax.scatter(x + rng.uniform(-0.09, 0.09, len(v)), v, marker=MK[source], s=42,
               facecolor='white', edgecolor=colour, linewidth=1.4, zorder=3)
    ax.hlines(v.mean(), x - 0.22, x + 0.22, color=colour, linewidth=2.4, zorder=4)


def violin(ax, x, vals, colour, alpha, denom):
    v = np.asarray(vals) / denom
    if len(v) >= 6:
        p = ax.violinplot([v], positions=[x], widths=0.62, showextrema=False)
        for b in p['bodies']:
            b.set_facecolor(colour); b.set_alpha(alpha); b.set_edgecolor('none')


# ---------------------------------------------------------------- fig 1: RLPD violins
fig, ax = plt.subplots(figsize=(7.6, 4.2))
order = ['dH', 'dDP', 'dHHfails', 'dDPfails', 'dHsucc_dup', 'dDPsucc_dup']
labels = ['human\n(n=7)', 'DP\n(n=7)', 'human\n+fails (n=8)', 'DP\n+fails (n=8)',
          'human\n+dup (n=8)', 'DP\n+dup (n=8)']
for i, k in enumerate(order):
    src = 'human' if k.startswith('dH') else 'machine'
    violin(ax, i, rlpd_last_rnd[k], C['RLPD'], 0.40 if src == 'human' else 0.18, 30)
    seeds_pts(ax, i, rlpd_last_rnd[k], C['RLPD'], src, 30)
for x0, x1, y, txt in [(0, 1, 0.92, '+0.21  p=0.002'), (2, 3, 0.99, '+0.21  p<0.001'),
                       (4, 5, 0.92, '+0.21  p=0.013')]:
    ax.plot([x0, x0, x1, x1], [y - 0.015, y, y, y - 0.015], color='0.35', lw=1)
    ax.text((x0 + x1) / 2, y + 0.008, txt, ha='center', fontsize=8.5, color='0.25')
ax.set_xticks(range(6)); ax.set_xticklabels(labels, fontsize=8.5)
ax.set_ylabel('pick success, 30 random ICs (LAST ckpt)')
ax.set_ylim(0, 1.07); ax.set_yticks(np.arange(0, 1.01, 0.2))
ax.set_title('RLPD: human demos beat DP-teacher demos across preparations\n'
             '(old world, sparse, γ=0.99; seed = unit; exact permutation p)', fontsize=10)
fig.savefig(f'{OUT}/fig1_rlpd_rnd_violin.png'); fig.savefig(f'{OUT}/fig1_rlpd_rnd_violin.pdf')

# ---------------------------------------------------------------- fig 2: divergence mechanism
fig, (a, b) = plt.subplots(1, 2, figsize=(9.2, 3.8), gridspec_kw={'width_ratios': [1, 1.5]})
div = [(np.asarray(rlpd_maxcl[k]) >= 1).mean() for k in order]
nn = [len(rlpd_maxcl[k]) for k in order]
cl_labels = ['human\n(n=8)', 'DP\n(n=8)', 'human\n+fails\n(n=8)', 'DP\n+fails\n(n=8)',
             'human\n+dup\n(n=8)', 'DP\n+dup\n(n=8)']
for i, d in enumerate(div):
    a.bar(i, d, color=C['RLPD'], alpha=0.85 if i % 2 == 0 else 0.45)
for i, (d, n) in enumerate(zip(div, nn)):
    a.text(i, d + 0.02, f'{int(round(d * n))}/{n}', ha='center', fontsize=8.5)
a.set_xticks(range(6)); a.set_xticklabels(cl_labels, fontsize=7)
a.set_ylabel('fraction of seeds diverged\n(max critic loss ≥ 1)'); a.set_ylim(0, 1)
a.set_title('divergence rate', fontsize=10)
for i, k in enumerate(order):
    src = 'human' if k.startswith('dH') else 'machine'
    v = np.asarray(rlpd_maxcl[k])
    b.scatter(i + rng.uniform(-0.12, 0.12, len(v)), v, marker=MK[src], s=40,
              facecolor='white', edgecolor=C['RLPD'], linewidth=1.3)
b.axhline(1, color='0.3', ls='--', lw=1); b.text(5.45, 1.25, 'divergence\nthreshold', fontsize=8, color='0.3')
b.set_yscale('log'); b.set_xticks(range(6)); b.set_xticklabels(cl_labels, fontsize=7)
b.set_ylabel('max critic loss (log)'); b.set_title('per-seed max critic loss', fontsize=10)
fig.suptitle('RLPD critic health by demo source: DP-teacher demos destabilise the critic;\n'
             'adding the teacher\'s own FAILURE tapes largely cures it (6/8 → 1/8)', y=1.06, fontsize=10)
fig.savefig(f'{OUT}/fig2_rlpd_divergence.png'); fig.savefig(f'{OUT}/fig2_rlpd_divergence.pdf')

# ---------------------------------------------------------------- fig 3: DP
fig, (a, b) = plt.subplots(1, 2, figsize=(8.6, 3.9), gridspec_kw={'width_ratios': [1.25, 1]})
for i, (k, denom) in enumerate([('dH_hold', 15), ('dDP_hold', 15), ('dH_rnd', 30), ('dDP_rnd', 30)]):
    x = [0, 1, 2.4, 3.4][i]; src = 'human' if k.startswith('dH') else 'machine'
    violin(a, x, dp_w3[k], C['DP'], 0.40 if src == 'human' else 0.18, denom)
    seeds_pts(a, x, dp_w3[k], C['DP'], src, denom)
a.set_xticks([0, 1, 2.4, 3.4]); a.set_xticklabels(['human', 'DP', 'human', 'DP'], fontsize=9)
a.text(0.5, -0.16, 'hold (15 ICs)', ha='center', transform=a.get_xaxis_transform(), fontsize=9)
a.text(2.9, -0.16, 'random (30 ICs)', ha='center', transform=a.get_xaxis_transform(), fontsize=9)
a.plot([2.4, 2.4, 3.4, 3.4], [0.685, 0.70, 0.70, 0.685], color='0.35', lw=1)
a.text(2.9, 0.708, '+0.06  p=0.041', ha='center', fontsize=8.5, color='0.25')
a.set_ylim(0, 1); a.set_ylabel('pick success (selected ckpt)')
a.set_title('corrected world, n=10 v 10 (seeds 20-29)', fontsize=10)
for i, k in enumerate(['dH_rnd', 'dDP_rnd']):
    src = 'human' if i == 0 else 'machine'
    seeds_pts(b, i, dp_old[k], C['DP'], src, 30)
up = np.asarray(dp_old['dHunpruned_rnd']) / 30
b.scatter([2] * len(up) + rng.uniform(-0.06, 0.06, len(up)), up, marker='*', s=150,
          facecolor='white', edgecolor=C['DP'], linewidth=1.5)
b.hlines(up.mean(), 1.78, 2.22, color=C['DP'], linewidth=2.4)
b.set_xticks([0, 1, 2]); b.set_xticklabels(['human\n(n=5)', 'DP\n(n=5)', 'human\nUNPRUNED\n(n=3)'], fontsize=8)
b.set_ylim(0, 1); b.set_title('old world, random ICs', fontsize=10)
fig.suptitle('Diffusion Policy is nearly source-indifferent — and the control holds on RAW human tapes:\n'
             'unpruned demos cost nothing detectable (−0.06, perm p 0.36, n=3 v 5)', y=1.06, fontsize=10)
fig.savefig(f'{OUT}/fig3_dp_violin.png'); fig.savefig(f'{OUT}/fig3_dp_violin.pdf')

# ---------------------------------------------------------------- fig 4: r2dreamer W3
fig, (a, b) = plt.subplots(1, 2, figsize=(7.4, 3.7), sharey=True)
for ax_, hk, rk, ttl in [(a, 'dH_hold', 'dDP_hold', 'hold (15 ICs)'), (b, 'dH_rnd', 'dDP_rnd', 'random (30 ICs)')]:
    denom = 15 if 'hold' in hk else 30
    seeds_pts(ax_, 0, r2d_w3[hk], C['R2D'], 'human', denom)
    seeds_pts(ax_, 1, r2d_w3[rk], C['R2D'], 'machine', denom)
    ax_.set_xticks([0, 1]); ax_.set_xticklabels(['human\n(n=4)', 'DP\n(n=4)'], fontsize=9)
    ax_.set_xlim(-0.6, 1.6); ax_.set_ylim(0, 1.02); ax_.set_title(ttl, fontsize=10)
a.set_ylabel('pick success (BEST ckpt)')
b.text(0.5, 0.06, 'human learns 4/4 seeds; DP 1/4\np=0.143 (min attainable 0.029)\nn=8 v 8 lands ~09-01',
       ha='center', fontsize=8.5, color='0.3', transform=b.transAxes)
fig.suptitle('World model (return-clamped DreamerV3), corrected world: human demos ignite it,\n'
             'DP demos mostly do not — direction only at n=4', y=1.08, fontsize=10)
fig.savefig(f'{OUT}/fig4_r2d_w3.png'); fig.savefig(f'{OUT}/fig4_r2d_w3.pdf')

# ---------------------------------------------------------------- fig 5: forest summary
fig, ax = plt.subplots(figsize=(7.6, 3.6))
ys = np.arange(len(forest))[::-1]
for y, (lab, d, lo, hi, p, col, note) in zip(ys, forest):
    ax.plot([lo, hi], [y, y], color=col, lw=2)
    ax.plot([d], [y], 'o', color=col, ms=7)
    ptxt = ('p<0.001' if p < 0.001 else f'p={p:.3f}') + (f'  ({note})' if note else '')
    if hi > 0.7:
        ax.text(d, y + 0.22, ptxt, va='bottom', ha='center', fontsize=8, color='0.3')
    else:
        ax.text(hi + 0.03, y, ptxt, va='center', fontsize=8, color='0.3')
ax.axvline(0, color='0.2', lw=1)
ax.set_yticks(ys); ax.set_yticklabels([f[0] for f in forest], fontsize=9)
ax.set_xlabel('human − DP-teacher demo advantage, pick success on random ICs (Welch 95% CI)')
ax.set_xlim(-0.5, 1.2)
fig.text(0.5, -0.05, 'dv3 (unmodified DreamerV3 torch port): no learner to compare — both sources 0 success at 1M ×3 seeds',
         ha='center', fontsize=8, color=C['DV3'])
ax.set_title('Human−DP demo gap by learner and preparation: the RLPD effect is world-dependent —\n'
             'present where machine-demo coverage is narrow (old world), absent where it is human-broad (corrected)', fontsize=10)
fig.savefig(f'{OUT}/fig5_gap_forest.png'); fig.savefig(f'{OUT}/fig5_gap_forest.pdf')
print('wrote 5 figures to', OUT)
