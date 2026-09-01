#!/usr/bin/env python3
"""fig10: WHAT the world model sees differently — temporal burstiness of human vs machine demos
(corrected world, frozen matched sets). Input: speed_series_w3.npz (per-decision EEF speed per tape,
extract_speed_series.py on the cluster). Three views:
(a) same-IC exemplar speed traces — same start, same task, different driving;
(b) all-tape raster: rows = tapes, x = normalized time, colour = speed — human rows are STRIPED
    (pauses/bursts), machine rows are smooth ramps;
(c) pooled speed distribution — human bimodal (dwell near zero + fast bursts), machine unimodal.
Run: python3 paper/figures/make_fig10_burstiness_2026-09-01.py <path-to-npz>
"""
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

Z = np.load(sys.argv[1])
arm_of = {}
for k in Z.files:
    arm, ic, uid = k.split('|')
    arm_of.setdefault(arm, {})[int(ic)] = Z[k]

plt.rcParams.update({'font.size': 9, 'axes.spines.top': False, 'axes.spines.right': False,
                     'figure.dpi': 150, 'savefig.bbox': 'tight'})
fig, ax = plt.subplots(1, 3, figsize=(12.5, 3.8), gridspec_kw={'width_ratios': [1.1, 1.3, 0.9]})
CH, CM = '#d95f02', '#7570b3'

# (a) exemplar: the shared IC with the LARGEST pause-fraction gap (labeled as such — a strong case,
# the median case is much subtler; see (c) for the population view)
shared = sorted(set(arm_of['dH']) & set(arm_of['dDP']))
pfH = {ic: float((arm_of['dH'][ic] < 0.0005).mean()) for ic in shared}
pfM = {ic: float((arm_of['dDP'][ic] < 0.0005).mean()) for ic in shared}
ic0 = max(shared, key=lambda ic: pfH[ic] - pfM[ic])
a = ax[0]
for arm, c, lab, pf in (('dH', CH, 'human', pfH), ('dDP', CM, 'machine', pfM)):
    sp = arm_of[arm][ic0] * 1000
    a.plot(np.arange(len(sp)) / 7.5, sp, color=c, lw=1.1, label=f'{lab} (fully stopped {pf[ic0]:.0%})')
a.set_xlabel('time (s, decision rate 7.5 Hz)'); a.set_ylabel('EEF speed (mm/decision)')
a.legend(fontsize=8, frameon=False)
a.set_title(f'(a) strongest same-trial contrast (uid {ic0})', fontsize=9.5)

# (b) raster: normalized-time speed, tapes sorted by length within arm
b = ax[1]
rows = []
for arm in ('dH', 'dDP'):
    pf = {i: float((arm_of[arm][i] < 0.0005).mean()) for i in arm_of[arm]}
    for ic in sorted(arm_of[arm], key=lambda i: -pf[i]):
        sp = arm_of[arm][ic]
        t = np.linspace(0, 1, len(sp))
        rows.append(np.interp(np.linspace(0, 1, 120), t, sp))
M = np.asarray(rows) * 1000
nH = len(arm_of['dH'])
im = b.imshow(np.where(M < 0.5, 0.0, M), aspect='auto', cmap='inferno', vmin=0, vmax=np.percentile(M, 97))
b.axhline(nH - 0.5, color='white', lw=1.5)
b.text(2, nH * 0.5, 'HUMAN', color='white', fontsize=9, va='center', fontweight='bold')
b.text(2, nH + (len(M) - nH) * 0.5, 'MACHINE', color='white', fontsize=9, va='center', fontweight='bold')
b.set_xlabel('normalized episode time'); b.set_yticks([])
b.set_title('(b) every tape: speed over time (black = FULL STOP <0.5mm)', fontsize=9.5)
fig.colorbar(im, ax=b, fraction=0.04, pad=0.02, label='mm/decision')

# (c) per-tape strict-stop fraction — the population separation (threshold-robust, d=+1.16)
c = ax[2]
rng2 = np.random.default_rng(3)
for x, (arm, col, lab) in enumerate((('dH', CH, 'human'), ('dDP', CM, 'machine'))):
    v = np.array([float((arm_of[arm][i] < 0.0005).mean()) for i in arm_of[arm]])
    c.scatter(x + rng2.uniform(-0.1, 0.1, len(v)), v, s=22, facecolor='white', edgecolor=col, linewidth=1.1)
    c.hlines(v.mean(), x - 0.25, x + 0.25, color=col, lw=2.5)
    c.text(x, -0.05, f'{lab}\n{v.mean():.2f}', ha='center', fontsize=8.5, color=col,
           transform=c.get_xaxis_transform())
c.set_xticks([]); c.set_xlim(-0.6, 1.6)
c.set_ylabel('full-stop fraction')
c.set_title('(c) full-stop fraction per tape (d=+1.16)', fontsize=9.5)

fig.suptitle('The candidate property behind the WM preference: humans STOP, machines CREEP —\n'
             'human demos fully halt 1.7x more (0.37 vs 0.22 of decisions, d=+1.16); at a loose 2 mm threshold the arms are\n'
             'indistinguishable (50% vs 48%) — the signal is true rest states + stop-to-start transients, not slowness',
             fontsize=9.5, y=1.10)
fig.savefig('paper/figures/fig10_burstiness.png'); fig.savefig('paper/figures/fig10_burstiness.pdf')
print('wrote fig10; exemplar IC', ic0)
