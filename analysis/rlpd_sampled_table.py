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


def core_census(cell_dir):
    """Physical-core classes of the episodes in one sweep/eval dir -> Counter({cores: n_episodes}).
    Cells written after 2026-09-08 carry node.cores; older ones carry only the hostname and count as 'unstamped'."""
    import collections, glob, json, os
    c = collections.Counter()
    for f in glob.glob(os.path.join(cell_dir, '*.json')):
        if os.path.basename(f) in ('sweep.json', 'metrics.json', 'bank_used.json'):
            continue
        try:
            d = json.load(open(f))
        except Exception:
            continue
        n = (d.get('node') or {})
        if n.get('cores'):
            c[str(n['cores'])] += 1
        elif n.get('hostname'):
            c[_cores_of_host(n['hostname'])] += 1
        else:
            c['unstamped'] += 1
    return c


_HOST_CORES = {}


def _cores_of_host(host):
    """Cells written before 2026-09-08 carry only a hostname; resolve it through sinfo when we are on the cluster,
    so the core-balance check also covers the already-computed cells. Off-cluster it degrades to 'unstamped'."""
    global _HOST_CORES
    if not _HOST_CORES:
        _HOST_CORES = {'_loaded': True}
        try:
            import subprocess
            out = subprocess.run(['sinfo', '-h', '-N', '-o', '%n %c'], capture_output=True, text=True, timeout=20).stdout
            for line in out.strip().split('\n'):
                p = line.split()
                if len(p) == 2:
                    _HOST_CORES[p[0]] = p[1]
        except Exception:
            pass
    return _HOST_CORES.get(host, 'unstamped')


def print_core_report(per_arm):
    """per_arm: {arm_label: Counter}. Flags a class present on one side only -- the documented divergence pattern."""
    if not any(per_arm.values()):
        return
    print('\n**Node core-count balance** (episodes per physical-core class; a class on one side only is the pattern '
          'behind every recorded cross-node disagreement):')
    for arm, c in per_arm.items():
        if c:
            print(f'- {arm}: ' + ', '.join(f'{k}-core x{v}' for k, v in sorted(c.items())))
    classes = [set(c) - {'unstamped'} for c in per_arm.values() if c]
    if len(classes) > 1:
        one_sided = set().union(*classes) - set.intersection(*classes)
        if one_sided:
            print(f'- **CORE SKEW**: {sorted(one_sided)} present on one side only -- re-run those cells pinned to a '
                  f'single class before quoting a difference.')
        else:
            print('- all arms span the same core classes.')


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
    cores = {}
    for label, (name, rows) in arms.items():
        import collections as _c
        tot = _c.Counter()
        for r in rows:
            for cell in ('final_sampled', 'final_det15', 'final_sampled_spots60', 'final_det_spots60'):
                tot += core_census(os.path.join(r['run'], 'sweep', cell))
        cores[name] = tot
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
    # Dead seeds: this project's RLPD runs have a documented failure mode where the LAST checkpoint collapses
    # (CONFOUNDS: dHv2raw s65, dDPv2 s55 in the v2 wave). They are NOT dropped -- the registered statistic is the LAST
    # checkpoint -- but they must be visible, because an arm's mean moves ~0.07 on rnd30 depending on whether one lands.
    print_core_report(cores)
    for label, (name, rows) in arms.items():
        dead = [r['seed'] for r in rows
                if complete((r['det'] or {}).get('rnd')) and r['det']['rnd'][0] <= 0.1 * r['det']['rnd'][1]]
        if dead:
            print(f'- DEAD SEEDS in {name}: s{", s".join(str(d) for d in dead)} (LAST-checkpoint collapse; INCLUDED in '
                  f'every aggregate above, as the registered statistic requires -- per-seed lists show them as ~0)')
    for label, (name, rows) in arms.items():
        pair = [(r['det']['rnd'][0], r['smp']['rnd'][0]) for r in rows if complete((r['det'] or {}).get('rnd')) and complete((r['smp'] or {}).get('rnd'))]
        if pair:
            d = [s - t for t, s in pair]
            print(f'- within-arm {name} rnd30 sampled − deterministic per seed: {d} (mean {sum(d) / len(d):+.2f} episodes of 30)')


if __name__ == '__main__':
    main()
