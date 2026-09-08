#!/usr/bin/env python3
"""Learner x source END-TO-END (full task) table (PHASE_PLAN amendment (n)): r2dreamer (WM, amendment (d)), RLPD and
DP runs, human `dHfull_all` (74 tapes) vs machine `dDPfull` (72), per-seed success-by-stage counts on hold15 / rnd30 /
spots60 / rnd300 in sample and mode, exact two-sided permutation tests human vs machine per learner, cell and stage,
and the minimum detectable effect implied by the observed per-seed spread.

Cells are the r2dreamer evaluator's layout (`fresh_eval_<set>_<mode>/metrics.json`), which baselines/eval_e2e.py
writes too, so all three learners are read by one code path:
  * WM cells carry `stages` as RATES over `episodes` and only picked/placed/placed_v2/contact/nested, where `nested`
    is the TRAINING PROXY (REVIEW_GUIDE §8 item 3). They have no slide_success / contact_push / nested_honest until
    the queued re-score lands; those print as '—'.
  * eval_e2e.py cells carry `stage_counts` (integers) for picked / placed / placed_v2 / contact / contact_push /
    slide_success / nested_proxy / nested_honest. `nested_proxy` is the column comparable with the WM `nested`.

`rnd30` is additionally reported STRATIFIED BY TRAINING SUPPORT (DP_PRUNED_GAP_2026-09-07 §0.4/§5): a start is
OUT-of-support when its can x exceeds --support-x (default 0.513 m, the farthest can in the pruned human pick set),
which splits the 30 fixed random starts 20 in / 10 out -- exactly the split that document reports.

usage: e2e_table_all.py [--rlpd-runs baselines/rl/checkpoints/e2e] [--dp-runs baselines/outputs/dp_e2e]
                        [--wm-runs $W/runs] [--sets "hold15 rnd30 spots60"] [--stages ...] [--seeds 0-7] [--strat]
Numbers only from the json files; a missing cell prints '—' and is excluded from its test.
"""
import argparse, itertools, json, os

ALL_STAGES = ('picked', 'placed_v2', 'contact', 'contact_push', 'slide_success', 'nested_proxy', 'nested_honest')
WM_ALIAS = {'nested_proxy': 'nested'}   # the WM evaluator's `nested` IS the proxy


def cell(run_dir, iset, mode, suffix=''):
    f = os.path.join(run_dir, f'fresh_eval_{iset}_{mode}{suffix}', 'metrics.json')
    if not os.path.exists(f):
        return None
    d = json.load(open(f))
    n = int(d['episodes'])
    if n == 0:
        return None
    if 'stage_counts' in d:                      # eval_e2e.py
        counts = {k: int(v) for k, v in d['stage_counts'].items()}
    else:                                        # r2dreamer evaluator (rates)
        counts = {k: int(round(float(v) * n)) for k, v in d.get('stages', {}).items()}
        for new, old in WM_ALIAS.items():
            if old in counts:
                counts[new] = counts[old]
    return dict(n=n, counts=counts, per_episode=d.get('per_episode'), path=f,
                isolation=d.get('isolation', 'shared_process'),
                nodes=d.get('nodes') or [((d.get('node') or {}).get('hostname')) or '?'])


def strat_counts(c, stage, support_x):
    """(in-support k/n, out-of-support k/n) for a cell whose per_episode records carry the IC can_pos."""
    if not c or not c['per_episode']:
        return None
    ins = outs = kin = kout = 0
    for e in c['per_episode']:
        ic = e.get('ic') or {}
        cp = ic.get('can_pos')
        if cp is None:
            return None
        got = e.get('stages', {}).get(stage)
        if got is None:
            return None
        if float(cp[0]) > support_x:
            outs += 1; kout += int(bool(got))
        else:
            ins += 1; kin += int(bool(got))
    return (kin, ins, kout, outs)


def perm(a, b):
    """exact two-sided permutation on per-seed counts -> (delta_per_seed, p)"""
    obs = sum(a) / len(a) - sum(b) / len(b); pool = a + b; n = len(a); c = t = 0
    for idx in itertools.combinations(range(len(pool)), n):
        s = set(idx); x = [pool[i] for i in idx]; y = [pool[i] for i in range(len(pool)) if i not in s]
        t += 1; c += abs(sum(x) / n - sum(y) / len(y)) >= abs(obs) - 1e-12
    return obs, c / t


def mde(a, b, n_eps, alpha_frac=0.05):
    """Smallest |delta| (as a RATE) an 8-v-8 exact permutation test could have called at alpha, from the observed
    spread: the permutation null's alpha-quantile of |mean difference|, scaled to a rate by n_eps."""
    pool = a + b; n = len(a); diffs = []
    for idx in itertools.combinations(range(len(pool)), n):
        s = set(idx); x = [pool[i] for i in idx]; y = [pool[i] for i in range(len(pool)) if i not in s]
        diffs.append(abs(sum(x) / n - sum(y) / len(y)))
    diffs.sort()
    q = diffs[int((1 - alpha_frac) * (len(diffs) - 1))]
    return q / max(n_eps, 1)


def main():
    W = os.environ.get('W', '/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03')
    ap = argparse.ArgumentParser()
    ap.add_argument('--rlpd-runs', default='baselines/rl/checkpoints/e2e')
    ap.add_argument('--dp-runs', default='baselines/outputs/dp_e2e')
    ap.add_argument('--wm-runs', default=W + '/runs')
    ap.add_argument('--wm-human', default='full_r2d_state_dHfull_all_bnormclampS8ent5_s{s}')
    ap.add_argument('--wm-machine', default='full_r2d_state_dDPfull_bnormclampS8ent5_s{s}')
    ap.add_argument('--sets', default='hold15 rnd30 spots60')
    ap.add_argument('--stages', default=' '.join(ALL_STAGES))
    ap.add_argument('--seeds', default='0-7')
    ap.add_argument('--support-x', type=float, default=0.513,
                    help='rnd30 stratification: can x above this is OUT of the training support (DP_PRUNED_GAP §0.4)')
    ap.add_argument('--strat', action='store_true', help='also print the rnd30 in/out-of-support split')
    ap.add_argument('--cell-suffix', default='',
                    help="'' = the shared-process cells (the PHASE_RESULTS §5.1 protocol, comparable with the "
                         "published world-model row); '_iso' = the isolated cells (one fresh process per start, "
                         "coordinator 2026-09-07). The world-model runs have shared cells only, so its rows print "
                         "'—' under _iso until §5.1 is re-scored under isolation.")
    args = ap.parse_args()
    a, b = args.seeds.split('-'); seeds = list(range(int(a), int(b) + 1))
    sets = args.sets.split(); stages = args.stages.split()
    learners = [
        ('WM  ', lambda arm, s: os.path.join(args.wm_runs, (args.wm_human if arm == 'dH' else args.wm_machine).format(s=s)), ('sample', 'mode')),
        ('RLPD', lambda arm, s: os.path.join(args.rlpd_runs, f'e2e_rlpd_{arm}_s{s}'), ('sample', 'mode')),
        ('DP  ', lambda arm, s: os.path.join(args.dp_runs, f'e2e_dp_{arm}_s{s}'), ('sample',)),
    ]
    for iset in sets:
        for learner, path_of, modes in learners:
            for mode in modes:
                cells = {arm: [cell(path_of(arm, s), iset, mode, args.cell_suffix) for s in seeds] for arm in ('dH', 'dDP')}
                if not any(c for cs in cells.values() for c in cs):
                    continue
                n_eps = next(c['n'] for cs in cells.values() for c in cs if c)
                nodes = sorted({h for cs in cells.values() for c in cs if c for h in c['nodes']})
                isol = sorted({c['isolation'] for cs in cells.values() for c in cs if c})
                print(f'\n### {learner.strip()} | {iset} | {mode} | {n_eps} starts x {len(seeds)} seeds | '
                      f'protocol {"/".join(isol)} | nodes {len(nodes)}: {",".join(nodes)}')
                if len(nodes) > 1:
                    print('  NOTE: cells produced on MORE THAN ONE compute node. Long-horizon full-scope episodes are '
                          'node-sensitive (coordinator 2026-09-07): the same checkpoint/IC/seed can flip outcome '
                          'between nodes. Seeds are spread across nodes, so this is variance, not bias -- but it '
                          'inflates the MDE and must be stated wherever these numbers appear.')
                print('| stage | human per-seed | human | machine per-seed | machine | Δ | p | MDE |')
                print('|---|---|---|---|---|---|---|---|')
                for st in stages:
                    per = {}
                    for arm in ('dH', 'dDP'):
                        vals = [c['counts'].get(st) for c in cells[arm] if c]
                        per[arm] = [v for v in vals if v is not None] if all(v is not None for v in vals) else None
                    if not per['dH'] or not per['dDP'] or len(per['dH']) != len(per['dDP']):
                        print(f'| {st} | — | — | — | — | — | — | — |')
                        continue
                    h, m = per['dH'], per['dDP']
                    d, p = perm(h, m)
                    print(f'| {st} | {h} | {sum(h)}/{len(h) * n_eps} ({sum(h) / (len(h) * n_eps):.3f}) | {m} | '
                          f'{sum(m)}/{len(m) * n_eps} ({sum(m) / (len(m) * n_eps):.3f}) | {d / n_eps:+.3f} | {p:.3f} | '
                          f'{mde(h, m, n_eps):.3f} |')
                if args.strat and iset.startswith('rnd'):
                    for st in ('picked', 'slide_success'):
                        rows = []
                        for arm in ('dH', 'dDP'):
                            tot = [0, 0, 0, 0]
                            ok = True
                            for c in cells[arm]:
                                sc = strat_counts(c, st, args.support_x)
                                if sc is None:
                                    ok = False; break
                                tot = [t + v for t, v in zip(tot, sc)]
                            rows.append(tot if ok else None)
                        if all(rows):
                            hh, mm = rows
                            print(f'  support split ({st}, can_x <= {args.support_x}): human in {hh[0]}/{hh[1]} '
                                  f'({hh[0] / max(hh[1], 1):.3f}) out {hh[2]}/{hh[3]} ({hh[2] / max(hh[3], 1):.3f}) | '
                                  f'machine in {mm[0]}/{mm[1]} ({mm[0] / max(mm[1], 1):.3f}) out {mm[2]}/{mm[3]} '
                                  f'({mm[2] / max(mm[3], 1):.3f})')


if __name__ == '__main__':
    main()
