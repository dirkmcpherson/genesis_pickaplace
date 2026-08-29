#!/usr/bin/env python3
"""Per-source tape census (ADVERSARIAL_AUDIT_2026-08-29 §1 control): saturation, rows, terminal rows,
reward, tipped, verify, obs ranges, r2d n_double_grant. Usage: python tape_census.py [root] > out.txt"""
import numpy as np, glob, json, os, sys
root = sys.argv[1] if len(sys.argv) > 1 else 'baselines/matched_v2'
print(f'per-source tape census, {root}')
for arm in ['dH', 'dDP', 'dHHfails', 'dDPfails', 'dHsucc_dup', 'dDPsucc_dup']:
    fs = sorted(glob.glob(f'{root}/{arm}/*.npz'))
    if not fs: continue
    A = []; S = []; L = []; T = 0; TR = 0; R = 0.0; V = {}; TIP = 0; LAB = {}
    for f in fs:
        z = np.load(f, allow_pickle=True); a = np.asarray(z['actions_delta'], float); A.append(a); L.append(len(a))
        S.append(np.asarray(z['states'], float)); T += int(np.asarray(z['terminated']).sum()); TR += int(np.asarray(z['truncated']).sum())
        R += float(np.asarray(z['rewards']).sum()); v = str(z['verify']); V[v] = V.get(v, 0) + 1
        TIP += int(bool(np.asarray(z['tipped']).any())) if 'tipped' in z else 0
        lab = str(z['label']); LAB[lab] = LAB.get(lab, 0) + 1
    A = np.concatenate(A); S = np.concatenate(S)
    rpf = f'{root}/r2d/{arm}/repeat.json'; rp = json.load(open(rpf)) if os.path.exists(rpf) else {}
    print(f'\n[{arm}] N={len(fs)} labels={LAB} rows={sum(L)} p50={np.median(L):.0f} max={max(L)} term_rows={T} trunc_rows={TR} '
          f'reward_sum={R} tipped_tapes={TIP} verify={V} r2d_n_double_grant={rp.get("n_double_grant")}')
    print('  |a| mean', np.abs(A).mean(0).round(3), ' frac|a|>=0.9', (np.abs(A) >= 0.9).mean(0).round(3), ' max', np.abs(A).max(0).round(2))
    for nm, fn in (('mean', S.mean), ('std', S.std), ('min', S.min), ('max', S.max)):
        print(f'  state {nm:4s}', fn(0).round(2))
