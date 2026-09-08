#!/usr/bin/env python3
"""DP pick checkpoints of record on spots60 (PHASE_PLAN amendment (k)) next to their hold / rnd cells of record.
Per run: sweep/selected/sweep.json (selected checkpoint, hold + rnd of record), sweep/final/sweep.json or
final_sweep.json (LAST), sweep/selected_spots60/sweep.json and sweep/final_spots60/sweep.json (new, cluster/dp_eval_spots60.sh).
usage: dp_spots60_table.py [--root baselines/outputs] [--arms 'dH:dp_w2final/dH_DP_s{s}:20-29,dDP:dp_w2final/dDP_DP_s{s}:20-29,dHv2raw:dp_v2fullw3/dHv2raw_DP_s{s}:50-57']
Exact two-sided permutation tests on per-seed spots60 counts for every arm pair (selected checkpoints; LAST where complete).

Fixture-validated 2026-09-08 (before the real cells landed): 28 synthetic runs across the three arms render the table and
all three pairwise tests correctly (p = 0.000 on a large planted gap, p = 0.200 on a small one), and a run whose LAST
checkpoint was pruned shows LAST as missing rather than silently reusing the selected one."""
import argparse, itertools, json, os


def load(path, key):
    if not os.path.exists(path):
        return None
    j = json.load(open(path)); r = j['sets'].get(key)
    return None if r is None else (int(r['picked']), int(r['n_present']), int(r['n_expected']), j.get('ckpt_step'))


def fmt(c):
    return '—' if c is None else (f'{c[0]}/{c[1]}' + ('' if c[1] == c[2] else f' (exp {c[2]})'))


def perm(a, b):
    obs = sum(a) / len(a) - sum(b) / len(b); pool = a + b; n = len(a); c = t = 0
    for idx in itertools.combinations(range(len(pool)), n):
        s = set(idx); x = [pool[i] for i in idx]; y = [pool[i] for i in range(len(pool)) if i not in s]
        t += 1; c += abs(sum(x) / n - sum(y) / len(y)) >= abs(obs) - 1e-12
    return obs, c / t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default='baselines/outputs')
    ap.add_argument('--arms', default='dH:dp_w2final/dH_DP_s{s}:20-29,dDP:dp_w2final/dDP_DP_s{s}:20-29,dHv2raw:dp_v2fullw3/dHv2raw_DP_s{s}:50-57')
    args = ap.parse_args()
    arms = {}
    for spec in args.arms.split(','):
        name, tmpl, rng = spec.split(':'); a, b = rng.split('-'); rows = []
        for s in range(int(a), int(b) + 1):
            run = os.path.join(args.root, tmpl.format(s=s)); sw = os.path.join(run, 'sweep')
            sel_h = load(os.path.join(sw, 'selected', 'sweep.json'), 'hold'); sel_r = load(os.path.join(sw, 'selected', 'sweep.json'), 'rnd')
            fin_p = os.path.join(sw, 'final', 'sweep.json') if os.path.exists(os.path.join(sw, 'final', 'sweep.json')) else os.path.join(sw, 'final_sweep.json')
            fin_r = load(fin_p, 'rnd')
            s60 = load(os.path.join(sw, 'selected_spots60', 'sweep.json'), 'spots60'); f60 = load(os.path.join(sw, 'final_spots60', 'sweep.json'), 'spots60')
            rows.append(dict(seed=s, sel_h=sel_h, sel_r=sel_r, fin_r=fin_r, s60=s60, f60=f60, sel_step=(sel_h[3] if sel_h else None)))
        arms[name] = rows
    print('| arm | seed | selected ckpt | sel hold | sel rnd/30 | LAST rnd/30 | sel spots60 | LAST spots60 |'); print('|---|---|---|---|---|---|---|---|')
    for name, rows in arms.items():
        for r in rows:
            print(f'| {name} | s{r["seed"]} | {r["sel_step"] or "—"} | {fmt(r["sel_h"])} | {fmt(r["sel_r"])} | {fmt(r["fin_r"])} | {fmt(r["s60"])} | {fmt(r["f60"]) if r["f60"] else ("=sel" if r["sel_step"] == "100000" else "—")} |')
        tot = []
        for k in ('sel_h', 'sel_r', 'fin_r', 's60', 'f60'):
            xs = [r[k] for r in rows if r[k] is not None and r[k][1] == r[k][2]]
            tot.append(f'{sum(x[0] for x in xs)}/{sum(x[1] for x in xs)} ({sum(x[0] for x in xs) / max(1, sum(x[1] for x in xs)):.3f}, n={len(xs)})' if xs else '—')
        print(f'| **{name}** | all | | ' + ' | '.join(tot) + ' |')
    names = list(arms)
    print()
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a = [r['s60'][0] for r in arms[names[i]] if r['s60'] and r['s60'][1] == r['s60'][2]]
            b = [r['s60'][0] for r in arms[names[j]] if r['s60'] and r['s60'][1] == r['s60'][2]]
            if len(a) >= 2 and len(b) >= 2:
                o, p = perm(a, b); print(f'- spots60 (selected ckpt) {names[i]} {a} vs {names[j]} {b}: Δ per-seed {o:+.2f} (rate {o / 60:+.3f}), exact two-sided perm p = {p:.3f} (n={len(a)} v {len(b)})')
            else:
                print(f'- spots60 {names[i]} vs {names[j]}: incomplete ({len(a)} v {len(b)})')


if __name__ == '__main__':
    main()
