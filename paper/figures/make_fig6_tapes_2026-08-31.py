#!/usr/bin/env python3
"""fig6: demo-tape numeric statistics, world x source (old/w3 x dH/dDP).
Inputs produced by extract_tape_stats.py on the cluster (228 tapes, no images):
tape_stats.csv + tape_paths.npz (20-pt arc-length-resampled EEF paths).
Run:  python3 paper/figures/make_fig6_tapes_2026-08-31.py <dir-with-inputs>
"""
import csv, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SRC = sys.argv[1] if len(sys.argv) > 1 else 'paper/figures'
OUT = 'paper/figures'
C = dict(old='#7f7f7f', w3='#2ca02c')          # colour = world here (single-figure exception)
MK = dict(dH='o', dDP='^')                      # shape = source (convention)
SETS = [('old', 'dH'), ('old', 'dDP'), ('w3', 'dH'), ('w3', 'dDP')]
LBL = ['old\nhuman', 'old\nDP', 'corrected\nhuman', 'corrected\nDP']

rows = list(csv.DictReader(open(f'{SRC}/tape_stats.csv')))
Z = np.load(f'{SRC}/tape_paths.npz', allow_pickle=True)
paths, keys = Z['paths'], [str(k) for k in Z['keys']]
by_key = {k: p for k, p in zip(keys, paths)}


def col(w, a, f):
    return np.array([float(r[f]) for r in rows if r['world'] == w and r['arm'] == a])


def ics(w, a):
    return {int(r['ic_uid']): f"{w}|{a}|{r['ic_uid']}|{r['uid']}" for r in rows
            if r['world'] == w and r['arm'] == a}


rng = np.random.default_rng(0)
plt.rcParams.update({'font.size': 9, 'axes.spines.top': False, 'axes.spines.right': False,
                     'figure.dpi': 150, 'savefig.bbox': 'tight'})
fig, ax = plt.subplots(2, 3, figsize=(11, 6.4))

for j, (metric, name) in enumerate([('wander', 'wander: mean dist from start→end chord (m)'),
                                    ('path_len', 'EEF path length (m)'),
                                    ('tortuosity', 'tortuosity (path/net)')]):
    a_ = ax[0][j]
    for i, (w, arm) in enumerate(SETS):
        v = col(w, arm, metric)
        p = a_.violinplot([v], positions=[i], widths=0.6, showextrema=False)
        for b in p['bodies']:
            b.set_facecolor(C[w]); b.set_alpha(0.35); b.set_edgecolor('none')
        a_.scatter(i + rng.uniform(-0.08, 0.08, len(v)), v, marker=MK[arm], s=14,
                   facecolor='white', edgecolor=C[w], linewidth=0.8)
        a_.hlines(v.mean(), i - 0.2, i + 0.2, color=C[w], lw=2)
    a_.set_xticks(range(4)); a_.set_xticklabels(LBL, fontsize=7.5)
    a_.set_title(name, fontsize=9)
    if metric == 'tortuosity':
        a_.set_ylim(0, 15)

# (d) pooled EEF voxel coverage at 2 cm
a_ = ax[1][0]
for i, (w, arm) in enumerate(SETS):
    pts = np.concatenate([by_key[k] for k in ics(w, arm).values()])
    vox = len(set(map(tuple, np.floor(pts / 0.02).astype(int))))
    a_.bar(i, vox, color=C[w], alpha=0.85 if arm == 'dH' else 0.45)
    a_.text(i, vox + 3, str(vox), ha='center', fontsize=8)
a_.set_xticks(range(4)); a_.set_xticklabels(LBL, fontsize=7.5)
a_.set_title('pooled EEF coverage\n(2 cm voxels occupied)', fontsize=9)

# (e) same-IC path distance ACROSS WORLDS, within arm (how much the world shift moved the tapes)
a_ = ax[1][1]
nn = []
for i, arm in enumerate(['dH', 'dDP']):
    o, wds = ics('old', arm), []
    for ic, k3 in ics('w3', arm).items():
        if ic in o:
            wds.append(np.linalg.norm(by_key[o[ic]] - by_key[k3], axis=1).mean())
    wds = np.array(wds)
    a_.scatter(i + rng.uniform(-0.08, 0.08, len(wds)), wds, marker=MK[arm], s=16,
               facecolor='white', edgecolor='#d62728', linewidth=0.9)
    a_.hlines(wds.mean(), i - 0.2, i + 0.2, color='#d62728', lw=2)
    nn.append(len(wds))
a_.set_xticks([0, 1]); a_.set_xticklabels([f'human\n(n={nn[0]} ICs)', f'DP\n(n={nn[1]} ICs)'], fontsize=8)
a_.set_title('same-IC path distance\nOLD vs CORRECTED world (m)', fontsize=9)

# (f) same-IC path distance ACROSS SOURCES, within world (teacher deviation from human)
a_ = ax[1][2]
nn = []
for i, w in enumerate(['old', 'w3']):
    h, wds = ics(w, 'dH'), []
    for ic, kd in ics(w, 'dDP').items():
        if ic in h:
            wds.append(np.linalg.norm(by_key[h[ic]] - by_key[kd], axis=1).mean())
    wds = np.array(wds)
    a_.scatter(i + rng.uniform(-0.08, 0.08, len(wds)), wds, marker='s', s=16,
               facecolor='white', edgecolor=C[w], linewidth=0.9)
    a_.hlines(wds.mean(), i - 0.2, i + 0.2, color=C[w], lw=2)
    nn.append(len(wds))
a_.set_xticks([0, 1]); a_.set_xticklabels([f'old world\n(n={nn[0]} ICs)', f'corrected world\n(n={nn[1]} ICs)'], fontsize=8)
a_.set_title('same-IC path distance\nHUMAN vs DP tapes (m)', fontsize=9)

fig.suptitle('Demo-tape numeric statistics, world × source (228 tapes; 20-pt arc-length-resampled EEF paths)\n'
             'colour = world (grey old / green corrected), marker = source (circle human / triangle DP)', fontsize=10)
fig.tight_layout(rect=[0, 0, 1, 0.92])
fig.savefig(f'{OUT}/fig6_tape_stats.png'); fig.savefig(f'{OUT}/fig6_tape_stats.pdf')
print('wrote fig6')

# print the summary numbers for the analysis note
for w, arm in SETS:
    print(w, arm, {m: round(col(w, arm, m).mean(), 4) for m in ['wander', 'path_len', 'tortuosity', 'idle_frac', 'n_rows']})
