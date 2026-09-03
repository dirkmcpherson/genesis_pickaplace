#!/usr/bin/env python3
"""fig10: WHAT the world model sees differently — temporal burstiness of human vs machine demos
(corrected world, frozen matched sets). Input: speed_series_w3.npz (per-decision EEF speed per tape,
extract_speed_series.py on the cluster). Single-column format since 09-02:
fig10a: (left) same-IC exemplar speed traces — the shared IC with the LARGEST full-stop gap (a strong
        case; the median case is subtler), (right) per-tape full-stop fraction (d=+1.16);
fig10b: all-tape raster: rows = tapes, x = normalized time, colour = speed — human rows are STRIPED
        (pauses/bursts), machine rows are smooth ramps.
Run: python3 paper/figures/make_fig10_burstiness_2026-09-01.py [path-to-npz]
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import colstyle as cs

Z = np.load(sys.argv[1] if len(sys.argv) > 1 else 'paper/figures/speed_series_w3.npz')
arm_of = {}
for k in Z.files:
    arm, ic, uid = k.split('|')
    arm_of.setdefault(arm, {})[int(ic)] = Z[k]
CH, CM = cs.C['human'], cs.C['machine']
rng = cs.setup()

shared = sorted(set(arm_of['dH']) & set(arm_of['dDP']))
pfH = {ic: float((arm_of['dH'][ic] < 0.0005).mean()) for ic in shared}
pfM = {ic: float((arm_of['dDP'][ic] < 0.0005).mean()) for ic in shared}
ic0 = max(shared, key=lambda ic: pfH[ic] - pfM[ic])

# ---- fig10a: exemplar traces + per-tape full-stop fraction
f, (a, c) = cs.fig(h=2.0, ncols=2, gridspec_kw={'width_ratios': [1.5, 1], 'wspace': 0.5})
for arm, colr, lab, pf in (('dH', CH, 'human', pfH), ('dDP', CM, 'machine', pfM)):
    sp = arm_of[arm][ic0] * 1000
    a.plot(np.arange(len(sp)) / 7.5, sp, color=colr, lw=0.9, label=f'{lab} (stopped {pf[ic0]:.0%})')
a.set_xlabel('time (s)'); a.set_ylabel('EEF speed (mm/dec.)')
a.set_ylim(0, 31); a.legend(fontsize=7, loc='upper right'); a.set_title('Strongest same-trial case')
rng2 = np.random.default_rng(3)
for x, (arm, colr, lab, mk) in enumerate((('dH', CH, 'H', 'o'), ('dDP', CM, 'M', '^'))):
    v = np.array([float((arm_of[arm][i] < 0.0005).mean()) for i in arm_of[arm]])
    cs.strip(c, x, v, colr, mk, rng2, s=8, half=0.25)
    c.text(x, -0.06, f'{lab}\n{v.mean():.2f}', ha='center', va='top', fontsize=7.5, color=colr, transform=c.get_xaxis_transform())
c.set_xticks([]); c.set_xlim(-0.75, 1.75); c.set_ylabel('full-stop fraction'); c.set_title('All tapes (d=+1.16)')
cs.save(f, 'fig10a_burstiness_traces')

# ---- fig10b: raster
f, b = cs.fig(h=2.3)
rows = []
for arm in ('dH', 'dDP'):
    pf = {i: float((arm_of[arm][i] < 0.0005).mean()) for i in arm_of[arm]}
    for ic in sorted(arm_of[arm], key=lambda i: -pf[i]):
        sp = arm_of[arm][ic]
        rows.append(np.interp(np.linspace(0, 1, 120), np.linspace(0, 1, len(sp)), sp))
M = np.asarray(rows) * 1000
nH = len(arm_of['dH'])
im = b.imshow(np.where(M < 0.5, 0.0, M), aspect='auto', cmap='inferno', vmin=0, vmax=np.percentile(M, 97))
b.axhline(nH - 0.5, color='white', lw=1.2)
b.text(2, nH * 0.5, 'HUMAN', color='white', fontsize=8, va='center', fontweight='bold')
b.text(2, nH + (len(M) - nH) * 0.5, 'MACHINE', color='white', fontsize=8, va='center', fontweight='bold')
b.set_xlabel('normalized episode time'); b.set_yticks([]); b.set_xticks([0, 60, 119]); b.set_xticklabels(['0', '0.5', '1'])
b.set_title('Every tape: speed over time (black = full stop)')
f.colorbar(im, ax=b, fraction=0.05, pad=0.02, label='mm/decision')
cs.save(f, 'fig10b_burstiness_raster')
print('exemplar IC', ic0)
