#!/usr/bin/env python3
"""fig8: what still separates human from machine demos in the corrected world.
Single-column format since 09-02: fig8a = the full 33-metric screen (tall, appendix-style),
fig8b = per-set distributions of the four headline metrics.
Inputs produced by baselines/diagnostics/tape_dynamics_metrics.py on the cluster
(228 tapes, matched_v2 + matched_w3, images untouched): tape_dyn_metrics.csv
(per-tape) + set_level_metrics.csv (pooled coverage family, bootstrap z).
Ranking criterion (pre-declared): C = Cohen's d(dH-dDP) in w3 MINUS same in old.
See paper/WM_METRIC_2026-09-01.md for definitions and caveats (screening study).
Run:  python3 paper/figures/make_fig8_wm_metric_2026-09-01.py [dir-with-inputs]
"""
import csv, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import colstyle as cs
from colstyle import C, MK

SRC = sys.argv[1] if len(sys.argv) > 1 else 'paper/figures'
SETS = [('old', 'dH'), ('old', 'dDP'), ('w3', 'dH'), ('w3', 'dDP')]
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


crit = sorted(((m, cohens_d(col('w3', 'dH', m), col('w3', 'dDP', m)) - cohens_d(col('old', 'dH', m), col('old', 'dDP', m)))
               for m in METRICS), key=lambda t: t[1])
setr = list(csv.DictReader(open(f'{SRC}/set_level_metrics.csv')))
rng = cs.setup()

# ---- fig8a: the full screen (nothing hidden)
f, a_ = cs.fig(h=4.3, w=2.7)
names = [t[0] for t in crit]; vals = [t[1] for t in crit]
a_.barh(range(len(names)), vals, color=['#d62728' if n in TEMPORAL else '#7f7f7f' for n in names], alpha=0.85, height=0.7)
a_.set_yticks(range(len(names))); a_.set_yticklabels(names, fontsize=7); a_.set_ylim(-0.7, len(names) - 0.3)
a_.axvline(0, color='k', lw=0.6)
a_.set_xlabel("C = d(human−machine) corrected − old")
a_.set_title(f'{len(METRICS)}-metric screen (red = temporal)')
a_.text(0.98, 0.02, 'set-level (bootstrap z):\n' + '\n'.join(f"{r['metric']}: {float(r['criterion_z']):+.1f}" for r in setr),
        transform=a_.transAxes, fontsize=7, va='bottom', ha='right', bbox=dict(fc='white', ec='#cccccc', lw=0.6))
cs.save(f, 'fig8a_wm_metric_screen')

# ---- fig8b: distributions of the headline metrics (WM_METRIC Addendum 2 panels)
panels = [('strict_stop_frac', 'full-stop fraction'), ('moving_speed_mean', 'moving speed (m/dec.)'),
          ('jerk_mean', 'EEF jerk (m)'), ('act_signflip_rate', 'action sign-flip rate')]
f, axs = cs.fig(h=3.0, nrows=2, ncols=2, gridspec_kw={'wspace': 0.45, 'hspace': 0.6})
for a_, (m, title) in zip(axs.ravel(), panels):
    for i, (w, arm) in enumerate(SETS):
        v = col(w, arm, m)
        p = a_.violinplot([v], positions=[i], widths=0.7, showextrema=False)
        for b in p['bodies']:
            b.set_facecolor(C[w]); b.set_alpha(0.3); b.set_edgecolor('none')
        cs.strip(a_, i, v, C[w], MK[arm], rng, s=5, half=0.25)
    a_.set_xticks(range(4)); a_.set_xticklabels(['H', 'M', 'H', 'M'])
    for t, (w, _) in zip(a_.get_xticklabels(), SETS):
        t.set_color(C[w])
    a_.set_title(title)
f.suptitle('grey old · green corrected · ○ human △ machine', y=1.0, fontsize=7.5)
cs.save(f, 'fig8b_wm_metric_panels')
