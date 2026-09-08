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


def cell(run_dir, iset, mode, suffix='', root=''):
    f = os.path.join(run_dir, root, f'fresh_eval_{iset}_{mode}{suffix}', 'metrics.json')
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
    pe = d.get('per_episode') or []
    isas = d.get('isa_classes') or sorted({str(e.get('isa', 'unknown')) for e in pe}) or ['unknown']
    return dict(n=n, counts=counts, per_episode=pe, path=f,
                isolation=d.get('isolation', 'shared_process'),
                nodes=d.get('nodes') or [((d.get('node') or {}).get('hostname')) or '?'],
                isas=isas, cpus=d.get('cpu_models') or sorted({str(e.get('cpu_model', 'unknown')) for e in pe}),
                role=d.get('role', 'legacy'),
                cores=d.get('core_counts') or sorted({e.get('cpu_cores_physical') for e in pe if e.get('cpu_cores_physical')}),
                threads=d.get('thread_counts') or sorted({e.get('torch_num_threads') for e in pe if e.get('torch_num_threads')}))


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
    ap.add_argument('--cell-root', default='',
                    help="subdirectory of each run dir holding the cells: '' = the in-job PREVIEW cells (produced "
                         "wherever the training job landed -- never a number for a table), 'rec' = the pinned "
                         "CPU-only evaluation pass (the cells of record).")
    ap.add_argument('--cores', type=int, default=None,
                    help='restrict every arm to seeds whose cell ran on a machine with exactly this many physical '
                         'cores -- the clean same-hardware comparison, and the GUARD OF RECORD (coordinator verdict '
                         '2026-09-07: `cores`). Cells of another size are dropped from the test and named.')
    ap.add_argument('--isa', default=None, choices=('avx2', 'avx512'),
                    help='(DIAGNOSTIC ONLY -- the instruction-set question is UNRESOLVED, not ruled out: the '
                         'CPU-family labels behind both the original AVX claim and its withdrawal are wrong on this '
                         'cluster) restrict every arm to seeds whose cell ran on this instruction-set class. Useful '
                         'only for a future re-check of families read from /proc/cpuinfo.')
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
                cells = {arm: [cell(path_of(arm, s), iset, mode, args.cell_suffix, args.cell_root) for s in seeds] for arm in ('dH', 'dDP')}
                if not any(c for cs in cells.values() for c in cs):
                    continue
                n_eps = next(c['n'] for cs in cells.values() for c in cs if c)
                roles = sorted({c['role'] for cs in cells.values() for c in cs if c})
                if len(roles) > 1:
                    print(f'\n### {learner.strip()} | {iset} | {mode} — REFUSED: this row mixes cell roles {roles}. '
                          f'A `preview` cell is produced wherever its training job happened to land and is not '
                          f'comparable with a pinned `record` cell (or with the `legacy` world-model cells); '
                          f're-run the missing side of the pinned pass rather than mixing. Rows skipped.')
                    continue
                for flag, key, label in ((args.cores, 'cores', 'physical cores'), (args.isa, 'isas', 'instruction set')):
                    if flag is None:
                        continue
                    for arm in ('dH', 'dDP'):
                        dropped = [s for s, c in zip(seeds, cells[arm]) if c and flag not in (c[key] or [])]
                        if dropped:
                            print(f'  --{key.rstrip("s")} {flag}: dropping {arm} seeds {dropped} (other {label})')
                        cells[arm] = [(c if (c and flag in (c[key] or [])) else None) for c in cells[arm]]
                if not any(c for cs in cells.values() for c in cs):
                    continue
                nodes = sorted({h for cs in cells.values() for c in cs if c for h in c['nodes']})
                isol = sorted({c['isolation'] for cs in cells.values() for c in cs if c})
                isa_all = sorted({i for cs in cells.values() for c in cs if c for i in c['isas']})
                core_all = sorted({n for cs in cells.values() for c in cs if c for n in (c['cores'] or [])})
                thr_all = sorted({t for cs in cells.values() for c in cs if c for t in (c['threads'] or [])})
                bal = {arm: {} for arm in ('dH', 'dDP')}
                for arm in ('dH', 'dDP'):
                    for c in cells[arm]:
                        if c:
                            for n in (c['cores'] or ['?']):
                                bal[arm][n] = bal[arm].get(n, 0) + 1
                print(f'\n### {learner.strip()} | {iset} | {mode} | {n_eps} starts x {len(seeds)} seeds | '
                      f'role {"/".join(roles)} | protocol {"/".join(isol)} | cores {core_all} threads {thr_all} | '
                      f'isa {",".join(isa_all)} | nodes {len(nodes)}: {",".join(nodes)}')
                if roles == ['preview']:
                    print('  PREVIEW ONLY -- these cells came from the in-job evaluation of training jobs that ran '
                          'wherever the scheduler put them. Descriptive only; the cells of record are the pinned '
                          'CPU-only pass (--cell-root rec).')
                print(f'  core-count balance -- human {bal["dH"]} | machine {bal["dDP"]}')
                if len(core_all) > 1 or len(thr_all) > 1:
                    print('  NOTE: this comparison spans MORE THAN ONE HARDWARE CONFIGURATION (physical cores '
                          f'{core_all}, per-task threads {thr_all}). Long-horizon full-scope outcomes track MACHINE '
                          'SIZE -- same checkpoint, IC, mode, seed and horizon give different outcomes on machines '
                          'of different core counts: 53/53 same-core-count comparisons bit-identical, all 19 '
                          'disagreements with a 36-core machine on exactly one side (coordinator verdict '
                          '2026-09-07). Node NAME is not the axis; the instruction set is unresolved and is not the '
                          'guard. If the balance line above is even across arms this is variance that inflates the '
                          'MDE; if it is skewed it is BIAS and the arms must be compared within one configuration '
                          '(--cores N).')
                elif len(nodes) > 1:
                    print(f'  Cells span {len(nodes)} nodes of ONE hardware configuration ({core_all} physical '
                          f'cores, {thr_all} threads) -- safe to combine (the divergence axis is machine size, not '
                          f'the node name).')
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
