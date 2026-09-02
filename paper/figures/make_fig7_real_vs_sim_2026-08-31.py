#!/usr/bin/env python3
"""fig7: REAL in-the-wild command tapes vs their sim re-executions (old + corrected world).
Single-column format since 09-02: fig7a (per-tape metrics) + fig7b (same-trial distances, coverage).
Real side = FK of the recorded 60 Hz joint-position commands through the same URDF the sim
uses (commanded/kinematic path). Sim side = the matched-set tapes of fig6. Same-trial pairing
via ic_uid == real trial uid. Run: python3 paper/figures/make_fig7_real_vs_sim_2026-08-31.py [dir]
"""
import csv, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import colstyle as cs
from colstyle import C

SRC = sys.argv[1] if len(sys.argv) > 1 else 'paper/figures'
sim = list(csv.DictReader(open(f'{SRC}/tape_stats.csv')))
real = [r for r in csv.DictReader(open(f'{SRC}/real_tape_stats.csv')) if r.get('stub') == '0']
ZS = np.load(f'{SRC}/tape_paths.npz', allow_pickle=True)
ZR = np.load(f'{SRC}/real_tape_paths.npz', allow_pickle=True)
sim_by = {str(k): p for k, p in zip(ZS['keys'], ZS['paths'])}
real_by = {int(str(k).split('|')[3]): p for k, p in zip(ZR['keys'], ZR['paths'])}
dH_ics = {w: {int(r['ic_uid']): f"{w}|dH|{r['ic_uid']}|{r['uid']}" for r in sim if r['world'] == w and r['arm'] == 'dH'} for w in ('old', 'w3')}
dDP_ics = {w: {int(r['ic_uid']): f"{w}|dDP|{r['ic_uid']}|{r['uid']}" for r in sim if r['world'] == w and r['arm'] == 'dDP'} for w in ('old', 'w3')}
real_succ = {int(r['uid']): r for r in real if r['label'] == 'success'}
matched_uids = sorted(set(dH_ics['old']) & set(real_succ))          # the 56-IC slice
rng = cs.setup()


def dist(a, b):
    return float(np.linalg.norm((a - a[0]) - (b - b[0]), axis=1).mean())   # start-aligned


def ticks(a_, labs, cols):
    a_.set_xticks(range(len(labs))); a_.set_xticklabels(labs)
    for t, c in zip(a_.get_xticklabels(), cols):
        t.set_color(c)


groups = [('real', C['real'], [real_succ[u] for u in matched_uids]),
          ('old', C['old'], [r for r in sim if r['world'] == 'old' and r['arm'] == 'dH']),
          ('corr.', C['w3'], [r for r in sim if r['world'] == 'w3' and r['arm'] == 'dH'])]
GL, GC = [g[0] for g in groups], [g[1] for g in groups]
# ---- fig7a: per-tape metrics
f, ax = cs.fig(h=1.8, ncols=3, gridspec_kw={'wspace': 0.55})
for a_, (metric, name) in zip(ax, [('wander', 'wander (m)'), ('path_len', 'path len. (m)'), ('idle_frac', 'idle frac.')]):
    for i, (lab, colr, rows_) in enumerate(groups):
        cs.strip(a_, i, [float(r[metric]) for r in rows_], colr, 'o', rng, s=6, half=0.25)
    ticks(a_, GL, GC); a_.set_title(name); a_.tick_params(axis='x', labelsize=7)
f.suptitle(f'Real command tapes vs sim replays ({len(matched_uids)} trials)', y=1.04, fontsize=7.5)
cs.save(f, 'fig7a_real_vs_sim_tapes')

# ---- fig7b: same-trial distances + coverage
f, ax = cs.fig(h=1.8, ncols=3, gridspec_kw={'wspace': 0.55})
a_ = ax[0]
for i, w in enumerate(('old', 'w3')):
    ds = [dist(real_by[u], sim_by[dH_ics[w][u]]) for u in matched_uids if u in dH_ics[w]]
    cs.strip(a_, i, ds, C[w], 'o', rng, s=6, half=0.25)
ticks(a_, ['old', 'corr.'], [C['old'], C['w3']]); a_.set_xlim(-0.6, 1.6); a_.set_title('vs replay (m)')
a_ = ax[1]
for i, w in enumerate(('old', 'w3')):
    ds = [dist(real_by[u], sim_by[dDP_ics[w][u]]) for u in matched_uids if u in dDP_ics[w]]
    cs.strip(a_, i, ds, C[w], '^', rng, s=6, half=0.25)
ticks(a_, ['old', 'corr.'], [C['old'], C['w3']]); a_.set_xlim(-0.6, 1.6); a_.set_title('vs machine (m)')
a_ = ax[2]
sets = [('real', C['real'], [real_by[u] for u in matched_uids]),
        ('old', C['old'], [sim_by[dH_ics['old'][u]] for u in matched_uids if u in dH_ics['old']]),
        ('corr.', C['w3'], [sim_by[dH_ics['w3'][u]] for u in matched_uids if u in dH_ics['w3']])]
for i, (lab, colr, ps) in enumerate(sets):
    pts = np.concatenate([p - p[0] for p in ps])
    vox = len(set(map(tuple, np.floor(pts / 0.02).astype(int))))
    a_.bar(i, vox, width=0.7, color=colr, alpha=0.7); a_.text(i, vox + 8, str(vox), ha='center', fontsize=7)
ticks(a_, GL, GC); a_.set_title('coverage (vox.)'); a_.set_ylim(0, 860); a_.set_yticks([]); a_.spines['left'].set_visible(False); a_.tick_params(axis='x', labelsize=7)
f.suptitle('Same-trial path distance; pooled coverage', y=1.04, fontsize=7.5)
cs.save(f, 'fig7b_real_vs_sim_sets')
print('matched trials:', len(matched_uids))
for lab, colr, rows_ in groups:
    print(lab, {k: round(np.mean([float(r[k]) for r in rows_]), 4) for k in ('wander', 'path_len', 'idle_frac', 'tortuosity')})
