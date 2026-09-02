#!/usr/bin/env python3
"""fig12 inputs: how the RAW human set (dHv2raw, uncapped, unpruned) differs from the PRUNED one (dHv2,
leading-idle-pruned) as DATASETS, both worlds. Contract-v1 npz; reads states/actions_delta/eef_pos only
(the images member is never touched -- np.load is lazy). Prints a CSV (one row per tape) between
CSV_BEGIN/CSV_END and set-level JSON between JSON_BEGIN/JSON_END so it can be captured over ssh.
Cluster: /cluster/tufts/shortlab/jstale02/condaenv/genesis/bin/python paper/figures/extract_raw_vs_pruned.py
"""
import csv, io, json, sys
import numpy as np
import pathlib as pl

ROOT = pl.Path('/cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines')
SETS = [('old', 'raw', ROOT / 'matched_v2' / 'dHv2raw'), ('old', 'pruned', ROOT / 'matched_v2' / 'dHv2'),
        ('w3', 'raw', ROOT / 'matched_w3' / 'dHv2raw'), ('w3', 'pruned', ROOT / 'matched_w3' / 'dHv2')]
IDLE, STOP, VOX = 1e-3, 5e-4, 0.02   # idle = ARM columns 0-5 only (col 6 = absolute gripper target, never ~0)

rows, pts = [], {}
for world, arm, d in SETS:
    for f in sorted(d.glob('*.npz')):
        z = np.load(f, allow_pickle=True)
        eef = z['eef_pos'].astype(np.float64); act = z['actions_delta'].astype(np.float64)
        steps = np.linalg.norm(np.diff(eef, axis=0), axis=1)
        idle = np.abs(act[:, :6]).max(axis=1) < IDLE
        idle7 = np.abs(act).max(axis=1) < IDLE
        lead = int(np.argmax(~idle)) if (~idle).any() else len(idle)
        trail = int(np.argmax(~idle[::-1])) if (~idle).any() else len(idle)
        a, b = eef[0], eef[-1]; ab = b - a; L2 = (ab @ ab) or 1e-12
        t = np.clip(((eef - a) @ ab) / L2, 0, 1)[:, None]
        rows.append(dict(world=world, arm=arm, uid=int(z['uid']), ic_uid=int(z['ic_uid']), stage=str(z['stage']),
                         n_rows=int(z['n']), sim_steps=int(z['sim_states'].shape[0]),
                         path_len=round(float(steps.sum()), 4), net_disp=round(float(np.linalg.norm(b - a)), 4),
                         wander=round(float(np.linalg.norm(eef - (a + t * ab), axis=1).mean()), 4),
                         idle_frac=round(float(idle.mean()), 4), idle_frac_7col=round(float(idle7.mean()), 4), leading_idle=lead, trailing_idle=trail,
                         idle_rows=int(idle.sum()), strict_stop_frac=round(float((steps < STOP).mean()), 4),
                         mean_speed=round(float(steps.mean()), 5), reward_sum=round(float(z['rewards'].sum()), 2),
                         max_sim_steps=int(z['max_sim_steps'])))
        pts[(world, arm, int(z['ic_uid']))] = eef

def vox(arrs):
    return len(set(map(tuple, np.floor(np.concatenate(arrs) / VOX).astype(int))))

summ = {}
for world in ('old', 'w3'):
    ic = {arm: {k[2] for k in pts if k[0] == world and k[1] == arm} for arm in ('raw', 'pruned')}
    common = ic['raw'] & ic['pruned']
    summ[world] = dict(n_raw=len(ic['raw']), n_pruned=len(ic['pruned']), n_common=len(common),
                       raw_only=sorted(ic['raw'] - ic['pruned']), pruned_only=sorted(ic['pruned'] - ic['raw']),
                       cov_raw=vox([pts[(world, 'raw', i)] for i in ic['raw']]),
                       cov_pruned=vox([pts[(world, 'pruned', i)] for i in ic['pruned']]),
                       cov_raw_common=vox([pts[(world, 'raw', i)] for i in common]),
                       cov_pruned_common=vox([pts[(world, 'pruned', i)] for i in common]),
                       rows_raw=int(sum(r['n_rows'] for r in rows if r['world'] == world and r['arm'] == 'raw')),
                       rows_pruned=int(sum(r['n_rows'] for r in rows if r['world'] == world and r['arm'] == 'pruned')))
buf = io.StringIO(); w = csv.DictWriter(buf, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
print('CSV_BEGIN'); print(buf.getvalue().rstrip()); print('CSV_END')
print('JSON_BEGIN'); print(json.dumps(summ)); print('JSON_END')
