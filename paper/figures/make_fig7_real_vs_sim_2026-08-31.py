#!/usr/bin/env python3
"""fig7: REAL in-the-wild command tapes vs their sim re-executions (old + corrected world).
Real side = FK of the recorded 60 Hz joint-position commands through the same URDF the sim
uses (commanded/kinematic path — the executed real path lives only in the bags). Sim side =
the matched-set tapes of fig6. Same-trial pairing via ic_uid == real trial uid.
Run:  python3 paper/figures/make_fig7_real_vs_sim_2026-08-31.py <dir-with-inputs>
"""
import csv, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SRC = sys.argv[1] if len(sys.argv) > 1 else 'paper/figures'
OUT = 'paper/figures'
C = dict(real='#d62728', old='#7f7f7f', w3='#2ca02c')
rng = np.random.default_rng(0)

sim = list(csv.DictReader(open(f'{SRC}/tape_stats.csv')))
real = [r for r in csv.DictReader(open(f'{SRC}/real_tape_stats.csv')) if r.get('stub') == '0']
ZS = np.load(f'{SRC}/tape_paths.npz', allow_pickle=True)
ZR = np.load(f'{SRC}/real_tape_paths.npz', allow_pickle=True)
sim_by = {k: p for k, p in zip([str(x) for x in ZS['keys']], ZS['paths'])}
real_by = {int(str(k).split('|')[3]): p for k, p in zip(ZR['keys'], ZR['paths'])}

dH_ics = {w: {int(r['ic_uid']): f"{w}|dH|{r['ic_uid']}|{r['uid']}" for r in sim
              if r['world'] == w and r['arm'] == 'dH'} for w in ('old', 'w3')}
dDP_ics = {w: {int(r['ic_uid']): f"{w}|dDP|{r['ic_uid']}|{r['uid']}" for r in sim
               if r['world'] == w and r['arm'] == 'dDP'} for w in ('old', 'w3')}
real_succ = {int(r['uid']): r for r in real if r['label'] == 'success'}
matched_uids = sorted(set(dH_ics['old']) & set(real_succ))          # the 56-IC slice

plt.rcParams.update({'font.size': 9, 'axes.spines.top': False, 'axes.spines.right': False,
                     'figure.dpi': 150, 'savefig.bbox': 'tight'})
fig, ax = plt.subplots(2, 3, figsize=(11, 6.4))


def dist(u, a, b):
    return float(np.linalg.norm((a - a[0]) - (b - b[0]), axis=1).mean())   # start-aligned


def strip(a_, x, v, colour, marker='o'):
    v = np.asarray(v, float)
    a_.scatter(x + rng.uniform(-0.08, 0.08, len(v)), v, marker=marker, s=14,
               facecolor='white', edgecolor=colour, linewidth=0.8)
    a_.hlines(np.mean(v), x - 0.2, x + 0.2, color=colour, lw=2)


# (a-c) per-tape metrics: real (matched 56) vs sim dH old vs sim dH w3
groups = [('real cmd', C['real'], [real_succ[u] for u in matched_uids]),
          ('old dH', C['old'], [r for r in sim if r['world'] == 'old' and r['arm'] == 'dH']),
          ('corrected dH', C['w3'], [r for r in sim if r['world'] == 'w3' and r['arm'] == 'dH'])]
for j, (metric, name) in enumerate([('wander', 'wander (m)'), ('path_len', 'EEF path length (m)'),
                                    ('idle_frac', 'idle fraction of frames')]):
    a_ = ax[0][j]
    for i, (lab, col, rows_) in enumerate(groups):
        strip(a_, i, [float(r[metric]) for r in rows_], col)
    a_.set_xticks(range(3)); a_.set_xticklabels([g[0] for g in groups], fontsize=8)
    a_.set_title(name, fontsize=9)

# (d) same-trial distance: real vs its re-execution, per world (dH = replayed human commands)
a_ = ax[1][0]
for i, w in enumerate(('old', 'w3')):
    ds = [dist(u, real_by[u], sim_by[dH_ics[w][u]]) for u in matched_uids if u in dH_ics[w]]
    strip(a_, i, ds, C[w])
a_.set_xticks([0, 1]); a_.set_xticklabels(['old world', 'corrected world'], fontsize=8)
a_.set_title('same-trial path distance (m)\nREAL commanded vs REPLAYED (dH)', fontsize=9)

# (e) same-trial distance: real human demo vs the DP teacher's rollout on that IC
a_ = ax[1][1]
for i, w in enumerate(('old', 'w3')):
    ds = [dist(u, real_by[u], sim_by[dDP_ics[w][u]]) for u in matched_uids if u in dDP_ics[w]]
    strip(a_, i, ds, C[w], marker='^')
a_.set_xticks([0, 1]); a_.set_xticklabels(['old world', 'corrected world'], fontsize=8)
a_.set_title('same-trial path distance (m)\nREAL human vs DP-TEACHER rollout', fontsize=9)

# (f) coverage: pooled 2 cm voxels, matched-56 slice, start-aligned frames
a_ = ax[1][2]
sets = [('real cmd', C['real'], [real_by[u] for u in matched_uids]),
        ('old dH', C['old'], [sim_by[dH_ics['old'][u]] for u in matched_uids if u in dH_ics['old']]),
        ('corrected dH', C['w3'], [sim_by[dH_ics['w3'][u]] for u in matched_uids if u in dH_ics['w3']])]
for i, (lab, col, ps) in enumerate(sets):
    pts = np.concatenate([p - p[0] for p in ps])
    vox = len(set(map(tuple, np.floor(pts / 0.02).astype(int))))
    a_.bar(i, vox, color=col, alpha=0.7); a_.text(i, vox + 3, str(vox), ha='center', fontsize=8)
a_.set_xticks(range(3)); a_.set_xticklabels([s[0] for s in sets], fontsize=8)
a_.set_title('pooled EEF coverage, matched 56 trials\n(2 cm voxels, start-aligned)', fontsize=9)

fig.suptitle('Real in-the-wild command tapes vs sim re-executions (n=%d matched trials)\n'
             'red = real commanded FK path · grey = old world · green = corrected world; paths start-aligned'
             % len(matched_uids), fontsize=10)
fig.tight_layout(rect=[0, 0, 1, 0.92])
fig.savefig(f'{OUT}/fig7_real_vs_sim.png'); fig.savefig(f'{OUT}/fig7_real_vs_sim.pdf')
print('wrote fig7; matched trials:', len(matched_uids))
for lab, col, rows_ in groups:
    f = lambda k: np.mean([float(r[k]) for r in rows_])
    print(lab, {k: round(f(k), 4) for k in ('wander', 'path_len', 'idle_frac', 'tortuosity')})
