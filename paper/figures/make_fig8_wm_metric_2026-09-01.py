#!/usr/bin/env python3
"""fig8: what still separates human from machine demos in the corrected world.
Inputs produced by baselines/diagnostics/tape_dynamics_metrics.py on the cluster
(228 tapes, matched_v2 + matched_w3, images untouched): tape_dyn_metrics.csv
(per-tape) + set_level_metrics.csv (pooled coverage family, bootstrap z).
Ranking criterion (pre-declared): C = Cohen's d(dH-dDP) in w3 MINUS same in old.
See paper/WM_METRIC_2026-09-01.md for definitions and caveats (screening study).
Run:  python3 paper/figures/make_fig8_wm_metric_2026-09-01.py [dir-with-inputs]
"""
import csv, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SRC = sys.argv[1] if len(sys.argv) > 1 else 'paper/figures'
OUT = 'paper/figures'
C = dict(old='#7f7f7f', w3='#2ca02c')      # colour = world (fig6 exception, kept)
MK = dict(dH='o', dDP='^')                 # shape = source (convention)
SETS = [('old', 'dH'), ('old', 'dDP'), ('w3', 'dH'), ('w3', 'dDP')]
LBL = ['old\nhuman', 'old\nDP', 'corrected\nhuman', 'corrected\nDP']
TEMPORAL = {'pause_frac', 'act_hf_frac', 'act_hf_frac_w32', 'jerk_per_len', 'jerk_mean',
            'joint_jerk_mean', 'speed_cv', 'accel_mean', 'act_autocorr1', 'act_signflip_rate',
            'speed_perm_entropy', 'speed_std', 'speed_mean', 'strict_stop_frac',
            'stop_frac_02mm', 'stop_frac_1mm', 'stop_frac_2mm', 'moving_speed_mean'}

rows = list(csv.DictReader(open(f'{SRC}/tape_dyn_metrics.csv')))
METRICS = [k for k in rows[0] if k not in ('world', 'arm', 'uid', 'ic_uid', 'stage')]


def col(w, a, m):
    x = np.array([float(r[m]) for r in rows if r['world'] == w and r['arm'] == a and r[m] != ''])
    return x[~np.isnan(x)]


def cohens_d(x, y):
    sp = np.sqrt(((len(x) - 1) * x.var(ddof=1) + (len(y) - 1) * y.var(ddof=1)) / (len(x) + len(y) - 2))
    return (x.mean() - y.mean()) / sp if sp > 1e-12 else 0.0


crit = []
for m in METRICS:
    d_old = cohens_d(col('old', 'dH', m), col('old', 'dDP', m))
    d_w3 = cohens_d(col('w3', 'dH', m), col('w3', 'dDP', m))
    crit.append((m, d_w3 - d_old, d_w3, d_old))
crit.sort(key=lambda t: t[1])

setr = list(csv.DictReader(open(f'{SRC}/set_level_metrics.csv')))

rng = np.random.default_rng(0)
plt.rcParams.update({'font.size': 9, 'axes.spines.top': False, 'axes.spines.right': False,
                     'figure.dpi': 150, 'savefig.bbox': 'tight'})
fig = plt.figure(figsize=(12, 6.8))
gs = fig.add_gridspec(2, 3, width_ratios=[1.5, 1, 1], hspace=0.45, wspace=0.3)

# (a) the full screen: criterion for every per-tape metric (nothing hidden)
a_ = fig.add_subplot(gs[:, 0])
names = [t[0] for t in crit]
vals = [t[1] for t in crit]
cols = ['#d62728' if n in TEMPORAL else '#7f7f7f' for n in names]
a_.barh(range(len(names)), vals, color=cols, alpha=0.85, height=0.7)
a_.set_yticks(range(len(names))); a_.set_yticklabels(names, fontsize=6.5)
a_.axvline(0, color='k', lw=0.7)
a_.set_xlabel("criterion C = Cohen's d (dH−dDP) in corrected − in old world")
a_.set_title(f'(a) all {len(METRICS)} per-tape metrics screened\n(red = temporal/burstiness family)', fontsize=9)
# set-level coverage family, annotated (bootstrap z, different scale -- listed not barred)
txt = 'set-level (bootstrap z, not d):\n' + '\n'.join(
    f"  {r['metric']}: C_z {float(r['criterion_z']):+.1f}" for r in setr)
a_.text(0.02, 0.02, txt, transform=a_.transAxes, fontsize=6.5, va='bottom',
        bbox=dict(fc='white', ec='#cccccc'))

# (b)-(e) per-set distributions of the headline metrics, every tape drawn
# panels revised 09-01 (WM_METRIC Addendum 2): absolute full-stop fraction replaces the
# fragile relative pause_frac; act_hf_frac demoted (windowed variant kills its w3 signal)
panels = [('strict_stop_frac', '(b) full-stop fraction\n(EEF speed < 0.5 mm/decision)'),
          ('moving_speed_mean', '(c) moving speed (m/decision)\n(mean over non-stopped steps)'),
          ('jerk_mean', '(d) EEF jerk per decision (m)'),
          ('act_signflip_rate', '(e) action sign-flip rate\n(dither; old-world DP artifact)')]
for pi, (m, title) in enumerate(panels):
    a_ = fig.add_subplot(gs[pi // 2, 1 + pi % 2])
    for i, (w, arm) in enumerate(SETS):
        v = col(w, arm, m)
        p = a_.violinplot([v], positions=[i], widths=0.6, showextrema=False)
        for b in p['bodies']:
            b.set_facecolor(C[w]); b.set_alpha(0.35); b.set_edgecolor('none')
        a_.scatter(i + rng.uniform(-0.08, 0.08, len(v)), v, marker=MK[arm], s=12,
                   facecolor='white', edgecolor=C[w], linewidth=0.7)
        a_.hlines(v.mean(), i - 0.2, i + 0.2, color=C[w], lw=2)
    a_.set_xticks(range(4)); a_.set_xticklabels(LBL, fontsize=7)
    a_.set_title(title, fontsize=8.5)

fig.suptitle('fig8: demo-tape properties that still separate human from machine in the CORRECTED world (228 tapes)\n'
             'colour = world (grey old / green corrected), marker = source (circle human / triangle DP); screening study —\n'
             'see WM_METRIC_2026-09-01.md +A2: stops-vs-creep is the robust headline, act_hf_frac demoted (w32 kills it)',
             fontsize=9.5, y=1.02)
fig.savefig(f'{OUT}/fig8_wm_metric.png'); fig.savefig(f'{OUT}/fig8_wm_metric.pdf')
print('wrote fig8')
