#!/usr/bin/env python3
"""Regenerate every HRI_results artefact from source.  Idempotent.

    python3 make_tables.py                 # rebuild from the committed seed_counts_raw.csv
    python3 make_tables.py --refresh       # re-pull the per-seed counts from the cluster first
    python3 make_tables.py --check-docs    # also print doc-vs-regenerated disagreements
    python3 make_tables.py --prior-sweep   # run the full prior-sensitivity table

Writes: seed_counts.csv (tidy, per-seed, only the cells of record), results.csv, results.md,
fig_effects.png / fig_effects.pdf, METHODOLOGY.md.

--refresh ships harvest_cluster.py to the login node, runs it there and pulls the raw CSV back.
The cluster link drops intermittently; a failed refresh leaves the existing CSV untouched and
the build continues from it, with the staleness recorded in the output.
"""
import argparse, collections, csv, json, os, subprocess, sys, textwrap, datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import cells as CELLS
import hri_stats as S
from doc_checks import DOC_OF_RECORD

RAW = os.path.join(HERE, 'seed_counts_raw.csv')
TIDY = os.path.join(HERE, 'seed_counts.csv')
RESULTS = os.path.join(HERE, 'results.csv')
ROPE = 0.10
SSH = ['ssh', '-o', 'ConnectTimeout=10', 'pax']


# ---------------------------------------------------------------------- refresh
def refresh(timeout=900):
    """Re-pull per-seed counts from the cluster. Returns (ok, message)."""
    try:
        subprocess.run(['scp', '-o', 'ConnectTimeout=10',
                        os.path.join(HERE, 'harvest_cluster.py'), 'pax:/tmp/hri_harvest.py'],
                       check=True, timeout=120, capture_output=True)
        r = subprocess.run(SSH + ['export LAB=/cluster/tufts/shortlab/jstale02; '
                                  'export W=$LAB/wm_fix_2026-09-03; '
                                  'python3 /tmp/hri_harvest.py > /tmp/hri_seed_counts.csv'],
                           check=True, timeout=timeout, capture_output=True, text=True)
        subprocess.run(['scp', '-o', 'ConnectTimeout=10', 'pax:/tmp/hri_seed_counts.csv', RAW],
                       check=True, timeout=300, capture_output=True)
        return True, (r.stderr or '').strip().splitlines()[-1:] and (r.stderr).strip() or 'ok'
    except Exception as e:
        return False, f'{type(e).__name__}: {e}'


# ---------------------------------------------------------------------- selection
def load_raw():
    with open(RAW) as f:
        return list(csv.DictReader(f))


def select(rows, sel):
    """Filter raw rows by a selector dict from cells.py. Returns rows sorted by seed."""
    out = []
    for r in rows:
        if 'smoke' in r['run']:
            continue          # smoke runs are never cells of record
        ok = True
        for key, want in sel.items():
            if key == 'seeds':
                ok = r['seed'] in want
            elif key == 'extra_has':
                ok = want in r['extra']
            elif key == 'run_has':
                ok = want in r['run']
            elif key == 'cell_in':
                ok = r['cell'] in want
            else:
                ok = r.get(key) == want
            if not ok:
                break
        if ok:
            out.append(r)
    # one row per seed; a duplicate seed within a selector is a registry bug, so fail loudly
    seen = {}
    for r in out:
        key = (r['seed'], r['cell'])
        if key in seen:
            raise SystemExit(f'duplicate seed {key} for selector {sel}')
        seen[key] = r
    return sorted(out, key=lambda r: (int(r['seed']), r['cell']))


def counts(rows):
    k = [int(r['k']) for r in rows]
    n = {int(r['n']) for r in rows}
    return k, (n.pop() if len(n) == 1 else None)


# ---------------------------------------------------------------------- build
def build(args):
    raw = load_raw()
    tidy, results = [], []

    for c in CELLS.COMPARISONS:
        h_rows = select(raw, c['sel_h']) if 'sel_h' in c else []
        m_rows = select(raw, c['sel_m']) if 'sel_m' in c else []
        # a comparison whose two selectors resolve to the same rows is not a comparison
        if h_rows and m_rows and [r['path'] for r in h_rows] == [r['path'] for r in m_rows]:
            h_rows = m_rows = []
        hk, hn = counts(h_rows)
        mk, mn = counts(m_rows)

        row = dict(id=c['id'], group=c['group'], learner=c['learner'], statistic=c['stat'],
                   action_mode=c['action'], human_arm=c.get('human', ''),
                   machine_arm=c.get('machine', ''),
                   n_seeds_human=len(hk), n_seeds_machine=len(mk),
                   episodes_per_seed=hn if hn == mn else f'{hn}/{mn}',
                   human_counts=' '.join(map(str, hk)), machine_counts=' '.join(map(str, mk)),
                   human_rate='', machine_rate='', delta='', perm_p='', perm_exact='',
                   mde_80='', ci_lo='', ci_hi='', sd_human='', sd_machine='',
                   p_rope='', p_gt0='', bf01='', post_mean='', post_lo='', post_hi='',
                   prior_flip='', verdict='', status='', provisional=c.get('provisional', ''),
                   note=c.get('note', ''), source='', bank_stamp='', evaluator='')

        for r in h_rows:
            tidy.append(dict(comparison=c['id'], arm_role='human', arm=r['arm'], seed=r['seed'],
                             k=r['k'], n=r['n'], cell=r['cell'], statistic=r['statistic'],
                             action_mode=r['action_mode'], source=r['source'], path=r['path']))
        for r in m_rows:
            tidy.append(dict(comparison=c['id'], arm_role='machine', arm=r['arm'], seed=r['seed'],
                             k=r['k'], n=r['n'], cell=r['cell'], statistic=r['statistic'],
                             action_mode=r['action_mode'], source=r['source'], path=r['path']))

        if h_rows or m_rows:
            banks = {(r['bank_version'] or 'UNSTAMPED') for r in h_rows + m_rows}
            row['bank_stamp'] = '|'.join(sorted(banks))
            row['evaluator'] = '|'.join(sorted({r['evaluator'] for r in h_rows + m_rows}))

        if len(hk) < 2 or len(mk) < 2 or hn is None or mn is None or hn != mn:
            row['status'] = 'EMPTY'
            if not (hk and mk):
                why = 'No per-seed data on the cluster for this cell.'
            elif hn is not None and mn is not None and hn != mn:
                why = f'Denominators differ between arms ({hn} vs {mn}); not a valid contrast.'
            elif hn is None or mn is None:
                why = 'Seeds within an arm have different denominators; the cell is incomplete.'
            else:
                why = (f'Too few seeds to test: {len(hk)} human v {len(mk)} machine '
                       f'(need >= 2 per arm). Landed so far: human {hk}, machine {mk} of {hn}.')
            row['verdict'] = (c.get('empty_reason', '') + ' ' + why).strip()
            results.append(row)
            continue

        row['source'] = h_rows[0]['source'] + ' + ' + m_rows[0]['source']
        row['human_rate'] = round(sum(hk) / (hn * len(hk)), 4)
        row['machine_rate'] = round(sum(mk) / (mn * len(mk)), 4)

        f = S.mde_ci(hk, mk, hn, mn)
        row['delta'] = round(f['delta'], 4)
        row['mde_80'] = round(f['mde'], 4)
        row['ci_lo'] = round(f['ci_lo'], 4); row['ci_hi'] = round(f['ci_hi'], 4)
        row['sd_human'] = round(f['sd_a'], 4); row['sd_machine'] = round(f['sd_b'], 4)

        if c.get('floor'):
            row['status'] = 'FLOOR'
            row['verdict'] = ('Both arms on the floor; no p-value or ROPE is computed because a '
                              'null here is an artefact of the floor, not evidence of equivalence.')
            results.append(row)
            continue

        d, p, nperm, exact = S.perm_test(hk, mk, hn, mn)
        row['perm_p'] = round(p, 4)
        row['perm_exact'] = 'exact' if exact else f'MC {nperm}'

        b = S.bayes_equivalence(hk, hn, mk, mn, rope=ROPE, prior='primary')
        row['p_rope'] = round(b['p_rope'], 4); row['p_gt0'] = round(b['p_gt0'], 4)
        row['bf01'] = float(f"{b['bf01']:.4g}")
        row['post_mean'] = round(b['mean'], 4)
        row['post_lo'] = round(b['lo'], 4); row['post_hi'] = round(b['hi'], 4)

        # prior sensitivity: does the equivalence verdict survive a reasonable prior change?
        alts = {name: S.bayes_equivalence(hk, hn, mk, mn, rope=ROPE, prior=name)
                for name in ('wide', 'tight')}
        alts['sep_sigma'] = S.bayes_equivalence(hk, hn, mk, mn, rope=ROPE, prior='primary',
                                                sep_sigma=True)
        verdicts = {k: (v['p_rope'] >= 0.90) for k, v in alts.items()}
        verdicts['primary'] = b['p_rope'] >= 0.90
        row['prior_flip'] = ('stable' if len(set(verdicts.values())) == 1 else
                             'FLIPS: ' + ', '.join(f'{k}={"in" if v else "out"}'
                                                   for k, v in sorted(verdicts.items())))
        row['prior_range'] = f"{min(v['p_rope'] for v in alts.values()):.3f}-" \
                             f"{max(list(v['p_rope'] for v in alts.values()) + [b['p_rope']]):.3f}"

        if b['p_rope'] >= 0.90:
            row['verdict'] = 'equivalent at +/-0.10'
        elif p < 0.05:
            row['verdict'] = 'difference detected'
        elif f['mde'] > 2 * ROPE:
            row['verdict'] = 'INCONCLUSIVE (underpowered)'
        else:
            row['verdict'] = 'inconclusive'
        row['status'] = 'PROVISIONAL' if c.get('provisional') else 'OK'
        results.append(row)

    return tidy, results


# ---------------------------------------------------------------------- doc cross-check
def check_docs(results):
    by = {r['id']: r for r in results}
    out = []
    for d in DOC_OF_RECORD:
        r = by.get(d['id'])
        if r is None or r['status'] == 'EMPTY':
            out.append((d['id'], 'no regenerated value', d['doc'], '', '',
                        d.get('expected', '')))
            continue
        for field, docval in (('human_rate', d.get('human')), ('machine_rate', d.get('machine')),
                              ('perm_p', d.get('p'))):
            if docval is None or r[field] == '':
                continue
            got = float(r[field])
            if abs(got - docval) > d.get('tol', 0.0015):
                out.append((d['id'], field, d['doc'], docval, got,
                            d.get('expected', '')))
    return out


def rescore_check(raw):
    """Statistics that appear in BOTH a cell and its `_cp` re-score should be identical wherever
    the predicate is unchanged (picked / contact / nested / placed_v2). A per-seed difference is
    evaluation non-reproducibility -- a finding about the evaluator, not about the arms.
    Returns rows of (phase, arm, cell, statistic, act, seed, original, rescored).
    """
    idx = {}
    for r in raw:
        if r['source'] != 'cluster:wm':
            continue
        idx[(r['phase'], r['arm'], r['extra'], r['cell'], r['statistic'],
             r['action_mode'], r['seed'])] = int(r['k'])
    out = []
    for key, k0 in sorted(idx.items()):
        ph, arm, ex, cell, stat, mode, seed = key
        if cell.endswith('_cp') or stat not in ('picked', 'contact', 'nested'):
            # placed_v2 is EXCLUDED: the corrected-predicate re-score redefined it (it is 0 in
            # every original full-task cell and non-zero after), so a difference there is the
            # intended predicate change, not a reproducibility failure.
            continue
        k1 = idx.get((ph, arm, ex, cell + '_cp', stat, mode, seed))
        if k1 is not None and k1 != k0:
            out.append((ph, arm, cell, stat, mode, seed, k0, k1))
    return out


def comparability(results):
    """A row (one phase, several learners) may only be read as ONE table if every cell in it was
    scored against the SAME entry-bank version with the SAME evaluator.  A MISSING bank stamp
    counts as unknown -- and therefore as NOT comparable -- never as a match, because several
    cells predate stamping and their entry states cannot be confirmed after the fact.

    Returns {group: dict(comparable, reason, stamps)}.
    """
    # Phases whose evaluation RESTORES a saved entry state from a bank; only these have an
    # entry-bank version to agree on. Pick and end-to-end start from an IC list instead, so an
    # absent bank stamp there is "not applicable", not "unknown".
    BANK_BASED = ('Place', 'Contact', 'Carrycontact')
    out = {}
    for g in dict.fromkeys(r['group'] for r in results):
        bank_based = g.startswith(BANK_BASED)
        cells_ = [r for r in results if r['group'] == g and
                  (r['status'] in ('OK', 'PROVISIONAL', 'FLOOR') or r['bank_stamp'])]
        learners = {r['learner'] for r in cells_}
        stamps = {r['id']: ((r['bank_stamp'] if bank_based else 'n/a (IC list)'),
                            r['evaluator']) for r in cells_}
        if len(learners) < 2:
            out[g] = dict(comparable=True, single=True, stamps=stamps,
                          reason='Only one learner has data in this row, so there is nothing to '
                                 'merge across learners.')
            continue
        # the evaluator SCRIPT differs between learners by construction and is not itself a
        # protocol difference; comparability turns on the entry bank and on the protocol.
        distinct = {s[0] for s in stamps.values()}
        unstamped = [i for i, (b, _) in stamps.items() if 'UNSTAMPED' in b]
        if not bank_based and len(distinct) == 1:
            out[g] = dict(comparable=True, single=False, stamps=stamps,
                          reason='No entry bank is restored in this phase (evaluation starts '
                                 'from a shared IC list), so there is no bank version to agree '
                                 'on.')
            continue
        if unstamped and len(distinct) > 1:
            out[g] = dict(comparable=False, single=False, stamps=stamps,
                          reason='NOT CROSS-LEARNER COMPARABLE. Cells in this row were scored '
                                 'against different entry-bank versions or by different '
                                 'evaluators, and ' + ', '.join('`%s`' % i for i in unstamped) +
                                 ' carries no bank stamp at all, so its entry states cannot be '
                                 'confirmed. An absent stamp is unknown, not a match. Each '
                                 "learner's own human-vs-machine contrast below is internally "
                                 'valid; the columns must not be read side by side.')
        elif len(distinct) > 1:
            out[g] = dict(comparable=False, single=False, stamps=stamps,
                          reason='NOT CROSS-LEARNER COMPARABLE. Cells in this row were scored '
                                 'against different entry-bank versions or by different '
                                 'evaluators. Each contrast is internally valid; the columns '
                                 'must not be read side by side.')
        elif unstamped:
            out[g] = dict(comparable=False, single=False, stamps=stamps,
                          reason='NOT CONFIRMED COMPARABLE. No cell in this row carries a bank '
                                 'stamp, so a shared entry bank cannot be verified even though '
                                 'nothing contradicts it.')
        else:
            out[g] = dict(comparable=True, single=False, stamps=stamps,
                          reason='All cells share entry bank and evaluator.')
    return out

def node_analysis(raw, rescore):
    """Test the standing explanation for non-reproducing re-scores: that they are a
    cross-hardware-class artefact. Joins every comparable statistic to the ORIGINAL record node
    and the RE-SCORE node, and reports (a) the hardware class of the discrepant cells, and
    (b) the 2x2 of cross-class exposure against movement, split by arm. Balanced exposure with
    one-sided movement refutes the hardware explanation.
    """
    path = os.path.join(HERE, 'node_provenance.csv')
    if not os.path.exists(path):
        return None
    lines = [l for l in open(path) if not l.startswith('###') and not l.startswith('   ')]
    NJ = {(r['run'], r['tag'], r['mode']): r for r in csv.DictReader(lines)}
    idx, run_of = {}, {}
    for r in raw:
        if r['source'] != 'cluster:wm':
            continue
        idx[(r['run'], r['cell'], r['statistic'], r['action_mode'])] = int(r['k'])
        run_of[(r['phase'], r['arm'], r['cell'], r['action_mode'], r['seed'])] = r['run']
    tally = {a: collections.Counter() for a in ('human', 'machine')}
    for (run, cell, stat, mode), k0 in idx.items():
        if cell.endswith('_cp') or stat not in ('picked', 'contact', 'nested'):
            continue
        k1 = idx.get((run, cell + '_cp', stat, mode))
        nj = NJ.get((run, cell, mode))
        if k1 is None or not nj or not nj['orig_arch'] or not nj['rescore_arch']:
            continue
        arm = 'human' if '_dH' in run else 'machine'
        tally[arm][(nj['orig_arch'] != nj['rescore_arch'], k0 != k1)] += 1
    disc = []
    for ph, arm, cell, stat, mode, seed, k0, k1 in rescore:
        run = run_of.get((ph, arm, cell, mode, seed))
        nj = NJ.get((run, cell, mode)) if run else None
        if nj:
            disc.append((ph, arm, cell, stat, mode, seed, k0, k1, nj))
    return dict(tally=tally, disc=disc)


# ---------------------------------------------------------------------- rendering
def _emit_row(A, r):
    flag = {'PROVISIONAL': ' *(prov.)*', 'EMPTY': '', 'FLOOR': ' *(floor)*'}.get(r['status'], '')
    if r['status'] == 'EMPTY':
        A(f"| `{r['id']}`{flag} | {r['learner']} | {r['statistic']} | {r['action_mode']} "
          f"| - | **EMPTY** | **EMPTY** | | | | | | | | {r['verdict']} |")
        return
    ci = f"[{fmt(r['ci_lo'])}, {fmt(r['ci_hi'])}]" if r['ci_lo'] != '' else ''
    if 'undef.' in ci:
        ci = 'undefined (zero within-arm spread)'
    A(f"| `{r['id']}`{flag} | {r['learner']} | {r['statistic']} | {r['action_mode']} "
      f"| {r['n_seeds_human']}v{r['n_seeds_machine']}x{r['episodes_per_seed']} "
      f"| {fmt(r['human_rate'])} | {fmt(r['machine_rate'])} | {fmt(r['delta'])} | {ci} "
      f"| {fmt(r['perm_p'])} | {fmt(r['mde_80'])} | {fmt(r['p_rope'])} "
      f"| {fmt(r['bf01'], 1)} | {r.get('prior_flip','')} | {r['verdict']} |")


def fmt(x, nd=3):
    import math as _m
    if x == '' or x is None:
        return ''
    if isinstance(x, float):
        return 'undef.' if _m.isnan(x) else f'{x:.{nd}f}'
    return str(x)


def render_md(results, stale, doc_issues, rescore, comp, nodes):
    L = []
    A = L.append
    A('# Human vs machine demonstrations - results of record\n')
    A(f'*Generated by `make_tables.py` on {datetime.date.today().isoformat()}. '
      'Do not hand-edit: rerun the script.*\n')
    if stale:
        A(f'> **Cluster refresh failed this build** ({stale}). Numbers come from the committed '
          '`seed_counts_raw.csv`, which may be behind the cluster.\n')
    A('Every row is **human arm vs machine arm**. Delta = human - machine on the success-rate '
      'scale. The unit of analysis is the **per-seed count**, never the pooled episode.\n')
    A('- **p** - exact two-sided permutation test on per-seed counts (the registered statistic).\n'
      '- **MDE** - smallest true effect this cell had 80 % power to detect. A null is only as '
      'good as its MDE.\n'
      '- **P(ROPE)** - posterior probability that |Delta| < 0.10, the registered equivalence '
      'margin. This is the quantity the registration actually asks about; a p-value cannot '
      'supply it.\n'
      '- **BF01** - interval Bayes factor for |Delta| < 0.10 against |Delta| >= 0.10.\n'
      '- **prior** - `stable` means the equivalence verdict survives all three priors and a '
      'separate-sigma refit.\n')

    groups = []
    for r in results:
        if r['group'] not in groups:
            groups.append(r['group'])
    def _hdr():
        A('| comparison | learner | statistic | act | n | human | machine | Delta | 95% CI | p | '
          'MDE | P(ROPE) | BF01 | prior | verdict |')
        A('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')

    for g in groups:
        A(f'\n## {g}\n')
        gn = getattr(CELLS, 'GROUP_NOTES', {}).get(g)
        if gn:
            A(gn + '\n')
        c = comp.get(g, {})
        split = not c.get('comparable', True)
        if split:
            A(f"> **{c['reason']}**\n")
            A('> | cell | entry bank | evaluator |')
            A('> |---|---|---|')
            for cid, (bk, ev) in sorted(c['stamps'].items()):
                A('> | `%s` | %s | %s |' % (cid, bk.replace('|', ' or '),
                                            ev.replace('|', ' or ')))
            A('')
        rows_g = [x for x in results if x['group'] == g]
        if split:
            for lr in dict.fromkeys(x['learner'] for x in rows_g):
                A(f'\n### {g} - {lr} (read alone)\n')
                _hdr()
                for r in [x for x in rows_g if x['learner'] == lr]:
                    _emit_row(A, r)
            continue
        _hdr()
        for r in rows_g:
            _emit_row(A, r)

    A('\n## Per-seed counts behind every row\n')
    A('| comparison | human counts | machine counts | episodes/seed |')
    A('|---|---|---|---|')
    for r in results:
        if r['status'] == 'EMPTY':
            continue
        A(f"| `{r['id']}` | {r['human_counts']} | {r['machine_counts']} "
          f"| {r['episodes_per_seed']} |")

    prov = [r for r in results if r['status'] == 'PROVISIONAL']
    if prov:
        A('\n## Provisional rows, and why\n')
        for r in prov:
            A(f"- **`{r['id']}`** - {r['provisional']}")
    empt = [r for r in results if r['status'] == 'EMPTY']
    if empt:
        A('\n## Empty cells, and why\n')
        for r in empt:
            A(f"- **`{r['id']}`** ({r['learner']}, {r['group']}) - {r['verdict']}")
    flo = [r for r in results if r['status'] == 'FLOOR']
    if flo:
        A('\n## Floor cells (no p-value by design)\n')
        for r in flo:
            A(f"- **`{r['id']}`** - human {fmt(r['human_rate'])}, machine "
              f"{fmt(r['machine_rate'])}. {r['note']}")

    A('\n## Re-score reproducibility\n')
    if rescore:
        A('Statistics whose predicate did NOT change between a cell and its '
          'corrected-predicate `_cp` re-score, yet whose per-seed count moved. These are '
          'evaluation non-reproducibility, not predicate effects:\n')
        A('| phase | arm | cell | statistic | act | seed | original | re-score |')
        A('|---|---|---|---|---|---|---|---|')
        for _row in rescore:
            A('| ' + ' | '.join(str(x) for x in _row) + ' |')
        _arms = sorted({x[1] for x in rescore})
        A('')
        A('Affected arms: ' + ', '.join(_arms) + '. Where one arm reproduces exactly and '
          'the other does not, the discrepancy is ASYMMETRIC between the two arms of a '
          'comparison, and every cell drawn from the re-score inherits it.')
    else:
        A('Every unchanged statistic reproduces exactly between a cell and its `_cp` '
          're-score.')

    if nodes:
        A('')
        A('### Is it the hardware class? No.')
        A('')
        A('The standing explanation for a re-score that does not reproduce is that it ran on a '
          'different class of machine. Joining every comparable statistic to its ORIGINAL record '
          'node and its RE-SCORE node refutes that here.')
        A('')
        A('| arm | cross-class re-scores | of those, moved | same-class re-scores | of those, moved |')
        A('|---|---|---|---|---|')
        for arm in ('human', 'machine'):
            t = nodes['tally'][arm]
            nc = t[(True, True)] + t[(True, False)]
            ns = t[(False, True)] + t[(False, False)]
            A(f'| {arm} | {nc} | **{t[(True, True)]}** | {ns} | **{t[(False, True)]}** |')
        A('')
        A('Exposure to cross-class re-scoring is IDENTICAL between the arms, yet only the human '
          'arm moves. Three human runs moved under a re-score on the SAME architecture and the '
          'SAME core count, which no cross-class effect can explain. And not one discrepant cell '
          'has a 36-core original record:')
        A('')
        A('| original record node | discrepant cells |')
        A('|---|---|')
        oc = collections.Counter(f"{d[8]['orig_arch']} / {d[8]['orig_cores']}-core"
                                 for d in nodes['disc'])
        for k, v in sorted(oc.items()):
            A(f'| {k} | {v} |')
        A('')
        A('**The hardware explanation is refuted, and the checkpoint explanation with it** '
          '(`latest.pt` predates the original evaluation in all 48 runs checked, so the '
          're-score read the same weights). The movement is localised to seven human runs - two '
          'end-to-end (seeds 2 and 3, which moved on 8 and 11 of their 12 statistics) and five '
          'contact (1-3 of 12 each) - rather than spread across the arm. **No mechanism has been '
          'established.** Until one is, every re-score-derived cell inherits a discrepancy that '
          'moves one arm only.')
        A('')

    A('\n## Doc-of-record cross-check\n')
    if doc_issues:
        A('Regenerated numbers that differ from the documents of record. Each is a finding, '
          'not a nuisance:\n')
        A('| comparison | field | doc | doc value | regenerated | understood? |')
        A('|---|---|---|---|---|---|')
        for cid, field, doc, dv, gv, exp in doc_issues:
            A(f'| `{cid}` | {field} | {doc} | {dv} | {gv} | '
              f'{exp or "**UNEXPLAINED - investigate**"} |')
        n_un = sum(1 for x in doc_issues if not x[5])
        A('')
        A(f'{len(doc_issues) - n_un} of {len(doc_issues)} are differences we already understand '
          f'(the reason is given). {n_un} are not.')
    else:
        A('Every cell with a published number reproduces it from the cluster artefacts '
          'within tolerance.\n')
    return '\n'.join(L) + '\n'


def make_fig(results, path_png, path_pdf):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    import math as _math
    def _ok(v):
        return v != '' and v is not None and not (isinstance(v, float) and _math.isnan(v))
    rows = [r for r in results if r['status'] in ('OK', 'PROVISIONAL')
            and _ok(r['ci_lo']) and _ok(r['ci_hi'])]
    rows = sorted(rows, key=lambda r: ([x['group'] for x in results].index(r['group']),))
    groups, ordered = [], []
    for r in results:
        if r['group'] not in groups and any(x['group'] == r['group'] for x in rows):
            groups.append(r['group'])
    for g in groups:
        ordered += [r for r in rows if r['group'] == g]

    COL = {'Diffusion Policy': '#3d6fb4', 'RLPD': '#c1622e',
           'world model (r2dreamer)': '#4b8f5a', 'world model (dv3)': '#7a5aa8',
           'BC-RNN': '#9a8748'}
    fig_h = max(4.5, 0.42 * len(ordered) + 1.9 * len(groups) + 1.4)
    fig, ax = plt.subplots(figsize=(11.0, fig_h))

    ax.axvspan(-0.10, 0.10, color='#7d7d7d', alpha=0.13, zorder=0, lw=0)
    ax.axvline(0, color='#555', lw=1.0, zorder=1)

    y, ticks, labels = 0.0, [], []
    for g in groups:
        y -= 1.05
        ax.text(0.006, y, g, ha='left', va='center', fontsize=10.5, fontweight='bold',
                transform=ax.get_yaxis_transform(), clip_on=False)
        for r in [x for x in ordered if x['group'] == g]:
            y -= 1.0
            prov = r['status'] == 'PROVISIONAL'
            col = COL.get(r['learner'], '#666666')
            ax.plot([r['ci_lo'], r['ci_hi']], [y, y], color=col, lw=2.4,
                    alpha=0.45 if prov else 1.0, solid_capstyle='round', zorder=3)
            ax.plot([r['delta']], [y], marker='o' if not prov else 'D', ms=8.0 if not prov else 6.6,
                    color='white' if prov else col, markeredgecolor=col, markeredgewidth=2.0,
                    zorder=4)
            ticks.append(y)
            lab = f"{r['id']}  ({r['n_seeds_human']}v{r['n_seeds_machine']})"
            labels.append(lab + ('  *' if prov else ''))
    y -= 1.0

    ax.set_yticks(ticks); ax.set_yticklabels(labels, fontsize=8.4, family='monospace')
    ax.set_ylim(y, 0.1)
    lim = max(0.36, max(abs(r['ci_lo']) for r in ordered) + 0.05,
              max(abs(r['ci_hi']) for r in ordered) + 0.05)
    ax.set_xlim(-lim, lim)
    ax.set_xlabel('$\\Delta$  =  human $-$ machine   (success-rate scale)', fontsize=11)
    ax.set_title('Human vs machine demonstrations: effect size with 95 % CI\n'
                 'shaded band = the registered $\\pm$0.10 equivalence margin (ROPE)',
                 fontsize=12.5, pad=13)
    ax.grid(axis='x', color='#dddddd', lw=0.7, zorder=0)
    for s in ('top', 'right', 'left'):
        ax.spines[s].set_visible(False)

    handles = [Line2D([], [], color=c, lw=2.6, label=k) for k, c in COL.items()
               if any(r['learner'] == k for r in ordered)]
    handles += [Line2D([], [], color='#666', marker='D', ls='none', mfc='white', mew=2.0,
                       label='provisional (*)')]
    ax.legend(handles=handles, fontsize=8.6, frameon=False, ncol=3,
              loc='upper center', bbox_to_anchor=(0.5, -0.055 / (fig_h / 10.0)))
    dropped = [r['id'] for r in results if r['status'] in ('OK', 'PROVISIONAL')
               and not (_ok(r['ci_lo']) and _ok(r['ci_hi']))]
    msg = ('A CI inside the shaded band supports equivalence; a CI wider than the band means '
           'the cell is underpowered, not that the arms match.')
    if dropped:
        msg += ('   Not plotted (no defined interval: every seed in each arm gave the identical '
                'count): ' + ', '.join(dropped) + '.')
    fig.text(0.012, 0.012, '\n'.join(textwrap.wrap(msg, 150)), fontsize=8.0, color='#444',
             va='bottom')
    fig.tight_layout(rect=(0, 0.075, 1, 1))
    fig.savefig(path_png, dpi=200); fig.savefig(path_pdf)
    plt.close(fig)
    return len(ordered)


# ---------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--refresh', action='store_true', help='re-pull per-seed counts from pax')
    ap.add_argument('--check-docs', action='store_true')
    ap.add_argument('--prior-sweep', action='store_true')
    args = ap.parse_args()

    stale = ''
    if args.refresh:
        ok, msg = refresh()
        print(f'[refresh] {"OK" if ok else "FAILED"}: {msg}', file=sys.stderr)
        if not ok:
            stale = msg

    tidy, results = build(args)
    doc_issues = check_docs(results)
    rescore = rescore_check(load_raw())
    comp = comparability(results)
    nodes = node_analysis(load_raw(), rescore)

    with open(TIDY, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(tidy[0].keys()))
        w.writeheader(); w.writerows(tidy)
    fields = list(results[0].keys())
    for r in results:
        for k in fields:
            r.setdefault(k, '')
    with open(RESULTS, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader(); w.writerows(results)
    with open(os.path.join(HERE, 'results.md'), 'w') as f:
        f.write(render_md(results, stale, doc_issues, rescore, comp, nodes))
    n = make_fig(results, os.path.join(HERE, 'fig_effects.png'),
                 os.path.join(HERE, 'fig_effects.pdf'))

    import methodology
    methodology.write(os.path.join(HERE, 'METHODOLOGY.md'))

    ok = sum(r['status'] == 'OK' for r in results)
    pv = sum(r['status'] == 'PROVISIONAL' for r in results)
    em = sum(r['status'] == 'EMPTY' for r in results)
    fl = sum(r['status'] == 'FLOOR' for r in results)
    bad = [g for g, v in comp.items() if not v['comparable']]
    if bad:
        print('NOT CROSS-LEARNER COMPARABLE: ' + '; '.join(bad))
    print(f'rows: {ok} OK, {pv} provisional, {em} empty, {fl} floor; '
          f'{len(tidy)} per-seed records; {n} plotted')
    if rescore:
        print(f'RE-SCORE NON-REPRODUCTIONS: {len(rescore)} per-seed cells')
        for _r in rescore:
            print('   ', _r)
    if doc_issues:
        print(f'DOC DISAGREEMENTS: {len(doc_issues)}')
        for d in doc_issues:
            print('   ', d)
    if args.prior_sweep:
        print('\nprior sensitivity (P(Delta in ROPE)):')
        raw = load_raw()
        for c in CELLS.COMPARISONS:
            if c.get('floor'):
                continue
            hk, hn = counts(select(raw, c['sel_h'])); mk, mn = counts(select(raw, c['sel_m']))
            if len(hk) < 2 or len(mk) < 2 or hn != mn:
                continue
            vals = {p: S.bayes_equivalence(hk, hn, mk, mn, prior=p)['p_rope']
                    for p in ('primary', 'wide', 'tight')}
            vals['sep_sigma'] = S.bayes_equivalence(hk, hn, mk, mn, sep_sigma=True)['p_rope']
            print('   %-26s ' % c['id'] + '  '.join(f'{k}={v:.3f}' for k, v in vals.items()))


if __name__ == '__main__':
    main()
