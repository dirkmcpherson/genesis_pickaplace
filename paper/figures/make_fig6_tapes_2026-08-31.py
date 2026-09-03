#!/usr/bin/env python3
"""fig6: demo-tape numeric statistics, world x source (old/w3 x dH/dDP). Single-column format
since 09-02: split into fig6a (per-tape geometry) and fig6b (set-level structure).
Inputs produced by extract_tape_stats.py on the cluster (228 tapes, no images):
tape_stats.csv + tape_paths.npz (20-pt arc-length-resampled EEF paths).
Run:  python3 paper/figures/make_fig6_tapes_2026-08-31.py [dir-with-inputs]
Colour = world here (grey old / green corrected; single-figure exception), marker = source.
"""
import csv, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import colstyle as cs
from colstyle import C, MK

SRC = sys.argv[1] if len(sys.argv) > 1 else 'paper/figures'
SETS = [('old', 'dH'), ('old', 'dDP'), ('w3', 'dH'), ('w3', 'dDP')]
LBL = ['H', 'M', 'H', 'M']
rows = list(csv.DictReader(open(f'{SRC}/tape_stats.csv')))
Z = np.load(f'{SRC}/tape_paths.npz', allow_pickle=True)
by_key = {str(k): p for k, p in zip(Z['keys'], Z['paths'])}


def col(w, a, f):
    return np.array([float(r[f]) for r in rows if r['world'] == w and r['arm'] == a])


def ics(w, a):
    return {int(r['ic_uid']): f"{w}|{a}|{r['ic_uid']}|{r['uid']}" for r in rows if r['world'] == w and r['arm'] == a}


def world_ticks(a_):
    a_.set_xticks(range(4)); a_.set_xticklabels(LBL)
    for t, (w, _) in zip(a_.get_xticklabels(), SETS):
        t.set_color(C[w])


rng = cs.setup()
# ---- fig6a: per-tape geometry
f, ax = cs.fig(h=1.8, ncols=3, gridspec_kw={'wspace': 0.55})
for a_, (metric, name) in zip(ax, [('wander', 'wander (m)'), ('path_len', 'path len. (m)'), ('tortuosity', 'tortuosity')]):
    for i, (w, arm) in enumerate(SETS):
        v = col(w, arm, metric)
        p = a_.violinplot([v], positions=[i], widths=0.7, showextrema=False)
        for b in p['bodies']:
            b.set_facecolor(C[w]); b.set_alpha(0.3); b.set_edgecolor('none')
        cs.strip(a_, i, v, C[w], MK[arm], rng, s=6, half=0.25)
    world_ticks(a_); a_.set_title(name)
    if metric == 'tortuosity':
        a_.set_ylim(0, 15)
f.suptitle('grey old · green corrected · ○ human △ machine', y=1.04, fontsize=7.5)
cs.save(f, 'fig6a_tape_geometry')

# ---- fig6b: set-level structure
f, ax = cs.fig(h=1.8, ncols=3, gridspec_kw={'wspace': 0.55})
a_ = ax[0]
for i, (w, arm) in enumerate(SETS):
    pts = np.concatenate([by_key[k] for k in ics(w, arm).values()])
    vox = len(set(map(tuple, np.floor(pts / 0.02).astype(int))))
    a_.bar(i * 1.2, vox, width=0.8, color=C[w], alpha=0.85 if arm == 'dH' else 0.45)
    a_.text(i * 1.2, vox + 6, str(vox), ha='center', fontsize=7)
a_.set_xticks([0, 1.2, 2.4, 3.6]); a_.set_xticklabels(LBL)
for t, (w, _) in zip(a_.get_xticklabels(), SETS):
    t.set_color(C[w])
a_.set_title('coverage (vox.)'); a_.set_ylim(0, 560); a_.set_yticks([]); a_.spines['left'].set_visible(False)

a_ = ax[1]; nn = []
for i, arm in enumerate(['dH', 'dDP']):
    o, wds = ics('old', arm), []
    for ic, k3 in ics('w3', arm).items():
        if ic in o:
            wds.append(np.linalg.norm(by_key[o[ic]] - by_key[k3], axis=1).mean())
    cs.strip(a_, i, wds, '#d62728', MK[arm], rng, s=7, half=0.25); nn.append(len(wds))
a_.set_xticks([0, 1]); a_.set_xticklabels(['H', 'M']); a_.set_xlim(-0.6, 1.6)
a_.set_title('world shift (m)')

a_ = ax[2]; nn = []
for i, w in enumerate(['old', 'w3']):
    h, wds = ics(w, 'dH'), []
    for ic, kd in ics(w, 'dDP').items():
        if ic in h:
            wds.append(np.linalg.norm(by_key[h[ic]] - by_key[kd], axis=1).mean())
    cs.strip(a_, i, wds, C[w], 's', rng, s=7, half=0.25); nn.append(len(wds))
a_.set_xticks([0, 1]); a_.set_xticklabels(['old', 'corr.']); a_.set_xlim(-0.6, 1.6)
for t, w in zip(a_.get_xticklabels(), ['old', 'w3']):
    t.set_color(C[w])
a_.set_title('H↔M (m)')
f.suptitle('Set level: coverage; same-IC path distances', y=1.04, fontsize=7.5)
cs.save(f, 'fig6b_tape_sets')
for w, arm in SETS:
    print(w, arm, {m: round(col(w, arm, m).mean(), 4) for m in ['wander', 'path_len', 'tortuosity', 'idle_frac', 'n_rows']})
