#!/usr/bin/env python3
"""cluster/r2d_best5_collect.py -- collect the finding-10 repair (paper/AUDIT_approach_2026-09-02.md).

Reads, for the frozen corrected-world WM block (pick_v5d4c_delta_shaped_{dH,dDP}_s80-87):
  * <OUT>/manifests/<run>.json          the 5 fraction checkpoints + BEST_selected.pt (r2d_best5_submit.sh)
  * <OUT>/logs/*.out                    RESCORE5-RESULT lines (r2d_rescore5.sh; corrected world, fresh process)
  * <RUNS>/<run>/ckpt_scores.tsv        K and the in-job `sel` score of every snapshot (selection statistic)
  * <N12>/<run>_W3_{hold,rnd}_*.log     RESCORE-RESULT lines = the BEST-of-K hold/rnd of record (base world, see
                                        r2d_rescore5.sh header)
and prints markdown: per-seed table, arm means, ignition (BEST hold >= 8/15), Fisher exact (pure python).

Selection rule for BEST-of-5 (the DP/RLPD analogue, dp_select_confirm.sh / rlpd_select_confirm.sh): among the
5 fraction checkpoints take the max in-job `sel` score (ckpt_scores.tsv column 2), ties -> later step; report
that checkpoint's fresh-process hold/rnd. A cell is only used when present == expected and both the missing
and wrongworld lists are empty (audit finding 23); otherwise it prints INVALID and the seed is excluded.

  python3 cluster/r2d_best5_collect.py [--out DIR] [--mode sample|mode] [--n12 DIR] [--runs DIR]
"""
import argparse, glob, json, math, os, re, sys
from collections import defaultdict

LAB = '/cluster/tufts/shortlab/jstale02'
ap = argparse.ArgumentParser()
ap.add_argument('--out', default=f'{LAB}/genesis_pickaplace/baselines/outputs/best5_rescore')
ap.add_argument('--n12', default=f'{LAB}/genesis_pickaplace/baselines/outputs/n12_rescore')
ap.add_argument('--runs', default=f'{LAB}/r2dreamer/runs')
ap.add_argument('--mode', default='sample', choices=['sample', 'mode'])
ap.add_argument('--ign', type=int, default=8, help='ignition threshold on BEST hold (>= ign of 15)')
args = ap.parse_args()

RE5 = re.compile(r'RESCORE5-RESULT tag=(\S+) mode=(\S+) variant=(\S+) ck=(\S+) ck_step=(\S+) set=(hold|rnd) '
                 r'picked=(\d+)/(\d+) expected=(\d+) max_steps=(\d+) missing=\[([^\]]*)\] wrongworld=\[([^\]]*)\] node=(\S+)')
REK = re.compile(r'RESCORE-RESULT tag=(pick_v5d4c_delta_shaped_(dH|dDP)_s(\d+))_W3 set=(hold|rnd) picked=(\d+)/(\d+) expected=(\d+)')

# ---- fresh corrected-world cells: (tag, mode, set) -> dict; latest job wins (job id from filename)
cells = {}
for f in sorted(glob.glob(os.path.join(args.out, 'logs', '*.out')), key=lambda p: int(re.search(r'_(\d+)\.out$', p).group(1))):
    for line in open(f, errors='replace'):
        m = RE5.search(line)
        if not m: continue
        tag, mode, variant, ck, ck_step, st, pk, pres, exp, maxs, miss, ww, node = m.groups()
        cells[(tag, mode, st)] = dict(picked=int(pk), present=int(pres), expected=int(exp), variant=variant, ck=ck,
                                      ck_step=ck_step, node=node, valid=(int(pres) == int(exp) and not miss.strip() and not ww.strip()),
                                      job=os.path.basename(f))

def cell(tag, st, mode=None):
    return cells.get((tag, mode or args.mode, st))

# ---- BEST-of-K of record (base world; n12_rescore)
rec = defaultdict(dict)
for f in glob.glob(os.path.join(args.n12, 'pick_v5d4c_delta_shaped_d*_s8?_W3_*_*.log')):
    if '_LAST_' in f: continue
    for line in open(f, errors='replace'):
        m = REK.search(line)
        if m:
            run, arm, seed, st, pk, pres, exp = m.groups()
            rec[run][st] = (int(pk), int(pres), int(exp))

def fisher_two_sided(a, b, c, d):
    """2x2 [[a,b],[c,d]] Fisher exact, two-sided (sum of table probs <= observed), pure python."""
    n = a + b + c + d; r1 = a + b; c1 = a + c
    def p(x):
        return (math.comb(r1, x) * math.comb(n - r1, c1 - x)) / math.comb(n, c1)
    p_obs = p(a); tot = 0.0
    for x in range(max(0, c1 - (n - r1)), min(r1, c1) + 1):
        px = p(x)
        if px <= p_obs * (1 + 1e-9): tot += px
    return min(1.0, tot)

rows = []
for manf in sorted(glob.glob(os.path.join(args.out, 'manifests', '*.json'))):
    man = json.load(open(manf)); run = man['run']; arm = man['arm']; seed = man['seed']
    tsv = os.path.join(args.runs, run, 'ckpt_scores.tsv')
    K = sum(1 for _ in open(tsv)) if os.path.exists(tsv) else man.get('K')
    fr = man['fractions']
    # BEST-of-5 by in-job sel (ties -> later step)
    order = ['F20', 'F40', 'F60', 'F80', 'F100']
    best5 = max(order, key=lambda k: (fr[k]['sel_score'], fr[k]['step_name']))
    per = {}
    for k in order + ['BESTK']:
        h = cell(f'{run}_{k}', 'hold'); r = cell(f'{run}_{k}', 'rnd')
        per[k] = (h, r)
    def fmt(c, n):
        if c is None: return 'PENDING'
        s = f"{c['picked']}/{c['present']}"
        return s if c['valid'] else s + f"(exp{c['expected']})INVALID"
    hK, rK = rec.get(run, {}).get('hold'), rec.get(run, {}).get('rnd')
    h5, r5 = per[best5]
    hB, rB = per['BESTK']
    rows.append(dict(arm=arm, seed=seed, K=K, bestk_ck=os.path.basename(man['bestk'].get('path', '')),
                     recK_hold=hK, recK_rnd=rK, cw_bestk_hold=hB, cw_bestk_rnd=rB,
                     best5=best5, best5_sel=fr[best5]['sel_score'], best5_ck=os.path.basename(fr[best5]['path']),
                     b5_hold=h5, b5_rnd=r5, per=per, fr=fr, run=run))

print(f"## BEST-of-5 re-selection, mode={args.mode}, world=gc_kp4_riser3_shelf6 (fresh process), hold=15 rnd=30\n")
print("| arm | seed | K_old | BEST-of-K ckpt | rec. BEST-K hold (base world) | rec. BEST-K rnd (base) | BEST-K hold (corr. world, fresh) | BEST-K rnd (corr.) | BEST-of-5 (by sel) | sel | BEST-5 hold | BEST-5 rnd | F20 h/r | F40 h/r | F60 h/r | F80 h/r | F100 h/r |")
print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
def fr_(c):
    if c is None: return '--'
    return f"{c['picked']}/{c['present']}" + ('' if c['valid'] else '!')
for r in rows:
    fk = lambda t: (f'{t[0]}/{t[1]}' + ('' if t[1] == t[2] else f'(exp{t[2]})')) if t else 'n/a'
    print(f"| {r['arm']} | {r['seed']} | {r['K']} | {r['bestk_ck']} | {fk(r['recK_hold'])} | {fk(r['recK_rnd'])} | "
          f"{fr_(r['cw_bestk_hold'])} | {fr_(r['cw_bestk_rnd'])} | {r['best5']} ({r['best5_ck']}) | {r['best5_sel']:.2f} | "
          f"{fr_(r['b5_hold'])} | {fr_(r['b5_rnd'])} | " + ' | '.join(f"{fr_(r['per'][k][0])} / {fr_(r['per'][k][1])}" for k in ['F20','F40','F60','F80','F100']) + ' |')
print("\n`!` = present != expected or missing/wrongworld episodes (INVALID, excluded); `--` = pending.\n")

# ---- arm summaries
def summarize(label, getter, denom):
    out = {}
    for arm in ('dH', 'dDP'):
        vals = []; n_ign = 0; n_val = 0; excluded = []
        for r in rows:
            if r['arm'] != arm: continue
            v = getter(r)
            if v is None: excluded.append(r['seed']); continue
            n_val += 1; vals.append(v[0] / v[1])
        out[arm] = dict(n=n_val, mean=(sum(vals) / len(vals)) if vals else float('nan'), excluded=excluded, vals=vals)
    return out

def g_rec(st):
    return lambda r: (r[f'recK_{st}'][0], r[f'recK_{st}'][1]) if (r[f'recK_{st}'] and r[f'recK_{st}'][1] == r[f'recK_{st}'][2]) else None
def g_cell(key, st):
    def g(r):
        c = r[f'{key}_{st}'] if key in ('cw_bestk', 'b5') else None
        return (c['picked'], c['present']) if (c and c['valid']) else None
    return g

print("### Arm means (mean of per-seed rates; n = valid seeds)\n")
print("| statistic | dH mean (n) | dDP mean (n) | diff | excluded seeds |")
print("|---|---|---|---|---|")
for label, gh in [('rec. BEST-of-K hold (base world)', g_rec('hold')), ('rec. BEST-of-K rnd (base world)', g_rec('rnd')),
                  ('BEST-of-K hold (corr. world, fresh)', g_cell('cw_bestk', 'hold')), ('BEST-of-K rnd (corr. world, fresh)', g_cell('cw_bestk', 'rnd')),
                  ('BEST-of-5 hold (corr. world, fresh)', g_cell('b5', 'hold')), ('BEST-of-5 rnd (corr. world, fresh)', g_cell('b5', 'rnd'))]:
    s = summarize(label, gh, None)
    d = s['dH']['mean'] - s['dDP']['mean']
    print(f"| {label} | {s['dH']['mean']:.3f} ({s['dH']['n']}) | {s['dDP']['mean']:.3f} ({s['dDP']['n']}) | {d:+.3f} | dH {s['dH']['excluded']} dDP {s['dDP']['excluded']} |")

print(f"\n### Ignition (BEST hold >= {args.ign}/15), Fisher exact two-sided (pure python)\n")
print("| criterion | dH ignited/n | dDP ignited/n | Fisher p |")
print("|---|---|---|---|")
for label, gh in [('rec. BEST-of-K hold (base world)', g_rec('hold')), ('BEST-of-K hold (corr. world, fresh)', g_cell('cw_bestk', 'hold')),
                  ('BEST-of-5 hold (corr. world, fresh)', g_cell('b5', 'hold'))]:
    cnt = {}
    for arm in ('dH', 'dDP'):
        ig = 0; n = 0
        for r in rows:
            if r['arm'] != arm: continue
            v = gh(r)
            if v is None: continue
            n += 1; ig += int(v[0] >= args.ign)
        cnt[arm] = (ig, n)
    a, na = cnt['dH']; c, nc = cnt['dDP']
    p = fisher_two_sided(a, na - a, c, nc - c) if (na and nc) else float('nan')
    print(f"| {label} | {a}/{na} | {c}/{nc} | {p:.3f} |")

# ---- per-fraction winner census + optimistic bound (max hold over the 5, NOT a selection rule)
print("\n### Which fraction won (BEST-of-5 by in-job sel) and the max-over-5 hold bound\n")
print("| arm | seed | winner | sel of winner | max hold over 5 (fraction) | max rnd over 5 (fraction) |")
print("|---|---|---|---|---|---|")
for r in rows:
    hs = [(r['per'][k][0]['picked'], k) for k in ['F20','F40','F60','F80','F100'] if r['per'][k][0] and r['per'][k][0]['valid']]
    rs = [(r['per'][k][1]['picked'], k) for k in ['F20','F40','F60','F80','F100'] if r['per'][k][1] and r['per'][k][1]['valid']]
    mh = f"{max(hs)[0]}/15 ({max(hs)[1]})" if len(hs) == 5 else f'pending ({len(hs)}/5)'
    mr = f"{max(rs)[0]}/30 ({max(rs)[1]})" if len(rs) == 5 else f'pending ({len(rs)}/5)'
    print(f"| {r['arm']} | {r['seed']} | {r['best5']} | {r['best5_sel']:.2f} | {mh} | {mr} |")

n_cells = sum(1 for (t, m, s) in cells if m == args.mode); n_valid = sum(1 for (t, m, s), c in cells.items() if m == args.mode and c['valid'])
print(f"\ncells mode={args.mode}: {n_cells} present, {n_valid} valid, expected {len(rows) * 6 * 2}")
