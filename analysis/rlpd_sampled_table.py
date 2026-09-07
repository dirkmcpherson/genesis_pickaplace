#!/usr/bin/env python3
"""RLPD pick checkpoints of record: DETERMINISTIC (existing cells) vs SAMPLED (cluster/rlpd_eval_sampled.sh)
action selection, per seed, both arms, hold + rnd; exact two-sided permutation tests human vs machine.

usage: rlpd_sampled_table.py [--root baselines/rl/checkpoints] [--wave rlpd_g99v2fullw3]
                             [--human dHv2raw:60-67] [--machine dDPv2:50-57]
Reads per run: sweep/final_sweep.json | sweep/final/sweep.json (deterministic LAST cell of record: hold/66 + rnd/30 on
eval_ics_v2_w3.json), sweep/final_det15/sweep.json (deterministic hold15 of eval_ics.json, new comparator) and
sweep/final_sampled/sweep.json (sampled hold15 + rnd30 of eval_ics.json). Missing cells print as '—'; a cell whose
episodes are incomplete prints n_present with '(exp N)' and is EXCLUDED from the tests. Numbers only from the json files."""
import argparse, itertools, json, os


def load_sets(path):
    if not os.path.exists(path):
        return None
    j = json.load(open(path))
    out = {}
    for s, r in j['sets'].items():
        out[s] = (int(r['picked']), int(r['n_present']), int(r['n_expected']))
    out['_act'] = j.get('act_selection')
    return out


def fmt(c):
    if c is None:
        return '—'
    p, n, e = c
    return f'{p}/{n}' + ('' if n == e else f' (exp {e})')


def complete(c):
    return c is not None and c[1] == c[2]


def perm(a, b):
    obs = sum(a) / len(a) - sum(b) / len(b); pool = a + b; n = len(a); c = t = 0
    for idx in itertools.combinations(range(len(pool)), n):
        s = set(idx); x = [pool[i] for i in idx]; y = [pool[i] for i in range(len(pool)) if i not in s]
        t += 1; c += abs(sum(x) / n - sum(y) / len(y)) >= abs(obs) - 1e-12
    return obs, c / t


def parse_arm(spec):
    name, rng = spec.split(':'); a, b = rng.split('-'); return name, list(range(int(a), int(b) + 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default='baselines/rl/checkpoints')
    ap.add_argument('--wave', default='rlpd_g99v2fullw3')
    ap.add_argument('--human', default='dHv2raw:60-67')
    ap.add_argument('--machine', default='dDPv2:50-57')
    args = ap.parse_args()
    arms = {}
    for label, spec in (('human', args.human), ('machine', args.machine)):
        name, seeds = parse_arm(spec); rows = []
        for s in seeds:
            run = os.path.join(args.root, f'{args.wave}_{name}_s{s}')
            det = load_sets(os.path.join(run, 'sweep', 'final_sweep.json')) or load_sets(os.path.join(run, 'sweep', 'final', 'sweep.json'))
            d15 = load_sets(os.path.join(run, 'sweep', 'final_det15', 'sweep.json'))
            smp = load_sets(os.path.join(run, 'sweep', 'final_sampled', 'sweep.json'))
            s60 = load_sets(os.path.join(run, 'sweep', 'final_sampled_spots60', 'sweep.json'))
            d60 = load_sets(os.path.join(run, 'sweep', 'final_det_spots60', 'sweep.json'))
            rows.append(dict(seed=s, run=run, det=det, d15=d15, smp=smp, s60=s60, d60=d60))
        arms[label] = (name, rows)
    cols = [('det hold/66', 'det', 'hold'), ('det rnd/30', 'det', 'rnd'), ('det hold15', 'd15', 'hold'),
            ('SMP hold15', 'smp', 'hold'), ('SMP rnd/30', 'smp', 'rnd'),
            ('det spots60', 'd60', 'spots60'), ('SMP spots60', 's60', 'spots60')]
    print(f'| arm | seed | ' + ' | '.join(c[0] for c in cols) + ' |')
    print('|---|---|' + '---|' * len(cols))
    for label, (name, rows) in arms.items():
        for r in rows:
            print(f'| {name} | s{r["seed"]} | ' + ' | '.join(fmt((r[k] or {}).get(s)) for _, k, s in cols) + ' |')
        tots = []
        for _, k, s in cols:
            cs = [r[k][s] for r in rows if complete((r[k] or {}).get(s))]
            tots.append(f'{sum(c[0] for c in cs)}/{sum(c[1] for c in cs)} ({sum(c[0] for c in cs) / max(1, sum(c[1] for c in cs)):.3f}, n={len(cs)})' if cs else '—')
        print(f'| **{name}** | all | ' + ' | '.join(tots) + ' |')
    hn, hr = arms['human']; mn, mr = arms['machine']
    print()
    for lab, k, s in (('SAMPLED rnd30', 'smp', 'rnd'), ('SAMPLED hold15', 'smp', 'hold'), ('deterministic rnd30 (existing)', 'det', 'rnd'),
                      ('deterministic hold15 (new comparator)', 'd15', 'hold'),
                      ('SAMPLED spots60 (amendment (k))', 's60', 'spots60'), ('deterministic spots60', 'd60', 'spots60')):
        a = [r[k][s][0] for r in hr if complete((r[k] or {}).get(s))]; b = [r[k][s][0] for r in mr if complete((r[k] or {}).get(s))]
        if len(a) >= 2 and len(b) >= 2:
            o, p = perm(a, b)
            denom = hr[0][k][s][2] if (hr[0][k] or {}).get(s) else 1
            print(f'- {lab}: {hn} {a} vs {mn} {b} -> Δ per-seed count {o:+.2f} (rate {o / denom:+.3f}), exact two-sided perm p = {p:.3f} (n={len(a)} vs {len(b)})')
        else:
            print(f'- {lab}: incomplete ({len(a)} vs {len(b)} complete cells)')
    for label, (name, rows) in arms.items():
        pair = [(r['det']['rnd'][0], r['smp']['rnd'][0]) for r in rows if complete((r['det'] or {}).get('rnd')) and complete((r['smp'] or {}).get('rnd'))]
        if pair:
            d = [s - t for t, s in pair]
            print(f'- within-arm {name} rnd30 sampled − deterministic per seed: {d} (mean {sum(d) / len(d):+.2f} episodes of 30)')


if __name__ == '__main__':
    main()
