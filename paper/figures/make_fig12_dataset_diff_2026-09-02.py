#!/usr/bin/env python3
"""fig12 (user Q4b, appendix): how the RAW human set (dH = dHv2raw) differs from the PRUNED one
(dHpruned = dHv2) as DATASETS, both worlds. Inputs: raw_vs_pruned_tapes.csv (one row per tape) +
raw_vs_pruned_sets.json (set-level coverage, IC membership), produced on the cluster by
extract_raw_vs_pruned.py (contract-v1 npz; states/actions_delta/eef_pos only, images never loaded).
Definitions: idle = max |arm action delta| (6 joint columns; column 7 is the absolute gripper target)
< 1e-3 per decision; full stop = EEF displacement < 0.5 mm per decision; coverage = distinct 2 cm
voxels visited by the pooled EEF paths; ratio = raw rows / pruned rows on the same IC.
Three single-column figures: fig12a (rows + per-IC ratio), fig12b (idle + full-stop fraction),
fig12c (path length + coverage).   python3 paper/figures/make_fig12_dataset_diff_2026-09-02.py [dir]
"""
import csv, json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import colstyle as cs
from colstyle import C

SRC = sys.argv[1] if len(sys.argv) > 1 else 'paper/figures'
rows = list(csv.DictReader(open(f'{SRC}/raw_vs_pruned_tapes.csv')))
S = json.load(open(f'{SRC}/raw_vs_pruned_sets.json'))
SETS = [('old', 'raw'), ('old', 'pruned'), ('w3', 'raw'), ('w3', 'pruned')]
XS = [0, 1.6, 3.7, 5.3]   # fig12a uses these in a 1.3-wide panel; fig12b/c are stacked full-width rows
rng = cs.setup()


def col(w, a, f):
    return np.array([float(r[f]) for r in rows if r['world'] == w and r['arm'] == a])


def n_of(w, a):
    return S[w]['n_raw' if a == 'raw' else 'n_pruned']


def ticks(ax_, y=-0.2):
    ax_.set_xticks(XS); ax_.set_xticklabels([f'{a}\n({n_of(w, a)})' for w, a in SETS], fontsize=7); ax_.set_xlim(-0.75, 6.05)
    for t, (w, _) in zip(ax_.get_xticklabels(), SETS):
        t.set_color(C[w])


def strips(ax_, metric):
    for x, (w, a) in zip(XS, SETS):
        v = col(w, a, metric)
        p = ax_.violinplot([v], positions=[x], widths=0.9, showextrema=False)
        for b in p['bodies']:
            b.set_facecolor(C[w]); b.set_alpha(0.18 if a == 'raw' else 0.4); b.set_edgecolor('none')
        cs.strip(ax_, x, v, C[w], 'o', rng, s=6, half=0.3)
    ticks(ax_)


def pair(ax_, w, x):
    raw = {int(r['ic_uid']): r for r in rows if r['world'] == w and r['arm'] == 'raw'}
    pr = {int(r['ic_uid']): r for r in rows if r['world'] == w and r['arm'] == 'pruned'}
    com = sorted(set(raw) & set(pr))
    return np.array([int(raw[i]['n_rows']) / int(pr[i]['n_rows']) for i in com])


# ---- fig12a: rows per tape (log) + per-IC row ratio
f, (a, b) = cs.fig(h=1.9, ncols=2, gridspec_kw={'wspace': 0.5, 'width_ratios': [1.3, 1]})
strips(a, 'n_rows'); a.set_yscale('log'); a.set_ylabel('rows per tape'); a.set_title('Tape length (decisions)')
a.axhline(300, color='0.4', lw=0.7, ls=':'); a.text(-0.55, 315, 'old horizon', fontsize=7, color='0.4', va='bottom', ha='left')
for x, w in ((0, 'old'), (1, 'w3')):
    v = pair(b, w, x)
    cs.strip(b, x, v, C[w], 'o', rng, s=6, half=0.25)
    b.text(x, 1.02, f'n={len(v)}', ha='center', va='top', fontsize=7, color=C[w], transform=b.get_xaxis_transform())
b.axhline(1, color='0.4', lw=0.7, ls=':')
b.set_xticks([0, 1]); b.set_xticklabels(['old', 'corr.']); b.set_xlim(-0.6, 1.6)
for t, w in zip(b.get_xticklabels(), ['old', 'w3']):
    t.set_color(C[w])
b.set_ylabel('rows raw / pruned'); b.set_title('Same-IC ratio')
f.suptitle(f"N: raw (dH) {S['old']['n_raw']}/{S['w3']['n_raw']}, pruned (dHpruned) {S['old']['n_pruned']}/{S['w3']['n_pruned']}", y=1.04, fontsize=7.5)
cs.save(f, 'fig12a_rawpruned_rows')

# ---- fig12b: idle fraction + full-stop fraction (stacked rows, shared x)
f, (a, b) = cs.fig(h=3.0, nrows=2, sharex=True, gridspec_kw={'hspace': 0.35})
strips(a, 'idle_frac'); a.set_ylabel('fraction'); a.set_title('Idle decisions (|Δaction| < 1e-3)')
strips(b, 'strict_stop_frac'); b.set_ylabel('fraction'); b.set_title('Full stops (EEF step < 0.5 mm)')
a.tick_params(labelbottom=False)
cs.group_label(b, 0.8, 'old world', y=-0.42, color=C['old']); cs.group_label(b, 4.5, 'corrected world', y=-0.42, color=C['w3'])
cs.save(f, 'fig12b_rawpruned_stillness')

# ---- fig12c: path length + coverage (all ICs vs common ICs), stacked rows
f, (a, b) = cs.fig(h=3.0, nrows=2, gridspec_kw={'hspace': 0.6})
strips(a, 'path_len'); a.set_ylabel('m'); a.set_title('EEF path length per tape')
labs, xs3 = [], []
for j, w in enumerate(('old', 'w3')):
    base = j * 4.6
    for i, (key, lab, al) in enumerate((('cov_raw', 'raw\nall', 0.85), ('cov_raw_common', 'raw\ncomm.', 0.5), ('cov_pruned', 'pruned', 0.3))):
        v = S[w][key]; x = base + i * 1.3
        b.bar(x, v, width=0.75, color=C[w], alpha=al); b.text(x, v + 12, str(v), ha='center', fontsize=7)
        labs.append(lab); xs3.append(x)
b.set_xticks([1.3, 5.9]); b.set_xticklabels(['old', 'corr.'])
for t, w in zip(b.get_xticklabels(), ('old', 'w3')):
    t.set_color(C[w])
from matplotlib.patches import Patch
b.legend(handles=[Patch(color='0.3', alpha=al, label=l) for l, al in (('raw, all ICs', 0.85), ('raw, common ICs', 0.5), ('pruned', 0.3))],
         loc='upper left', fontsize=7, handlelength=1.2, labelspacing=0.2)
b.set_ylim(0, 1250); b.set_yticks([0, 250, 500, 750]); b.set_ylabel('2 cm voxels'); b.set_title('Pooled EEF coverage (extra raw = raw-only tapes)')
b.legend(handles=b.get_legend().legend_handles, labels=['raw, all ICs', 'raw, common ICs', 'pruned'], loc='upper left', fontsize=7, handlelength=1.2, labelspacing=0.2, ncol=3, columnspacing=0.8)
cs.save(f, 'fig12c_rawpruned_geometry')

# headline numbers for FIGURES notes
for w in ('old', 'w3'):
    r = pair(None, w, 0)
    print(w, {k: (round(col(w, 'raw', k).mean(), 3), round(col(w, 'pruned', k).mean(), 3)) for k in ('n_rows', 'idle_frac', 'strict_stop_frac', 'path_len', 'wander')},
          'rows', S[w]['rows_raw'], S[w]['rows_pruned'], 'ratio med %.2f max %.2f >1.5: %d/%d' % (np.median(r), r.max(), (r > 1.5).sum(), len(r)),
          'raw-only', S[w]['raw_only'], 'pruned-only', S[w]['pruned_only'], 'cov', S[w]['cov_raw'], S[w]['cov_raw_common'], S[w]['cov_pruned'], S[w]['cov_pruned_common'])
