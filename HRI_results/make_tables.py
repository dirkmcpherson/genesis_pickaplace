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


def dead_seeds(k, n):
    """Indices of seeds that produced essentially nothing while the arm as a whole worked.

    A dead seed is a training failure, not a sample of the arm's behaviour, and one of them moves
    an arm mean by roughly 0.07 in this project, so they are marked rather than silently averaged.
    Rule: the arm's median rate must be >= 0.20 (otherwise the arm is near a floor and "dead" is
    meaningless) and the seed's rate must be below 15 % of that median. The threshold is set to
    separate a collapsed run from a merely weak one: at 0.15 it flags 4/148 and 2/60 and 0/30,
    and does NOT flag 30/148, which is a degraded seed rather than a dead one.
    """
    if not k or not n:
        return []
    import statistics
    rates = [x / n for x in k]
    med = statistics.median(rates)
    if med < 0.20:
        return []
    return [i for i, r in enumerate(rates) if r < 0.15 * med]


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
                   note=c.get('note', ''), source='', bank_stamp='', evaluator='',
                   dead_human=0, dead_machine=0,
                   rescored_from=c.get('rescored_from', ''),
                   rescore_in_flight=c.get('rescore_in_flight', ''))

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
        dh, dm = dead_seeds(hk, hn), dead_seeds(mk, mn)
        row['dead_human'] = len(dh); row['dead_machine'] = len(dm)
        row['human_counts'] = ' '.join(f'{v}*' if i in dh else str(v) for i, v in enumerate(hk))
        row['machine_counts'] = ' '.join(f'{v}*' if i in dm else str(v) for i, v in enumerate(mk))
        row['human_rate'] = round(sum(hk) / (hn * len(hk)), 4)
        row['machine_rate'] = round(sum(mk) / (mn * len(mk)), 4)

        f = S.mde_ci(hk, mk, hn, mn)
        row['delta'] = round(f['delta'], 4)
        row['mde_80'] = round(f['mde'], 4)
        row['ci_lo'] = round(f['ci_lo'], 4); row['ci_hi'] = round(f['ci_hi'], 4)
        row['sd_human'] = round(f['sd_a'], 4); row['sd_machine'] = round(f['sd_b'], 4)

        if c.get('floor'):
            row['status'] = 'SUPERSEDED' if c.get('rescore_in_flight') else 'FLOOR'
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
        if c.get('rescored_from') and len(hk) != len(mk):
            row['provisional'] = (row['provisional'] + ' RE-SCORE INCOMPLETE: '
                                  f'{len(hk)} human v {len(mk)} machine seeds have landed.').strip()
        # SUPERSEDED is a THIRD state, distinct from both OK and PROVISIONAL. Provisional means
        # "may move when more seeds land"; superseded means "the inputs to this number are being
        # overwritten right now". A reader must be able to tell those apart.
        row['status'] = ('SUPERSEDED' if row['rescore_in_flight']
                         else 'PROVISIONAL' if row['provisional'] else 'OK')
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
    # A superseded as-recorded row is kept visible for comparison but is NOT a cell of record,
    # so its (older, often unstamped) provenance must not block the row it was superseded by.
    superseded = {r['rescored_from'] for r in results if r.get('rescored_from')}
    out = {}
    for g in dict.fromkeys(r['group'] for r in results):
        bank_based = g.startswith(BANK_BASED)
        cells_ = [r for r in results if r['group'] == g and r['id'] not in superseded and
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
                          reason='All cells of record share an entry bank '
                                 f"({sorted({s[0] for s in stamps.values()})[0]}); superseded "
                                 'as-recorded rows are excluded from this check and shown only '
                                 'for comparison.')
    return out


def rescore_check(raw):
    """Cells whose EPISODES differ between a cell and its `_cp` re-score, read from the
    per-episode outcomes in node_provenance.csv. Column-level differences with zero differing
    episodes are structural (a predicate becoming earnable) and are reported separately, never
    as a reproduction failure.
    """
    path = os.path.join(HERE, 'node_provenance.csv')
    if not os.path.exists(path):
        return []
    out = []
    for r in csv.DictReader(open(path)):
        if r['run'].startswith('#') or r['moved'] != '1':
            continue
        out.append((r['run'], r['cell'], r['arm'], r['ep_diff'], r['episodes'],
                    r['cols_changed'], r['orig_host'], r['orig_cores'], r['orig_isa']))
    return out


def node_analysis(raw, rescore):
    """Read the per-cell re-score provenance table (harvest_nodes.py) and tabulate movement
    against the ORIGINAL record's hardware class. Attribution is per directory, from each
    evaluation job's own "[eval] wrote <path>" line; see harvest_nodes.py for why a per-run
    heuristic is wrong here."""
    path = os.path.join(HERE, 'node_provenance.csv')
    if not os.path.exists(path):
        return None
    rows = [r for r in csv.DictReader(open(path)) if not r['run'].startswith('#')]
    if not rows:
        return None
    by_cores = collections.Counter((r['orig_cores'], r['moved'] == '1') for r in rows)
    by_arm_exp = collections.Counter((r['arm'], r['orig_cores'] == '36') for r in rows)
    struct = [r for r in rows if r['structural_only'] == '1']
    movers = [r for r in rows if r['moved'] == '1']
    isa_of = {r['orig_cores']: r['orig_isa'] for r in rows if r['orig_isa']}
    return dict(rows=rows, by_cores=by_cores, by_arm_exp=by_arm_exp, struct=struct,
                movers=movers, isa_of=isa_of)


# ---------------------------------------------------------------------- rendering
def _dead(r):
    dh, dm = r.get('dead_human', 0), r.get('dead_machine', 0)
    return '' if not (dh or dm) else f"**{dh}H/{dm}M**"


def _emit_row(A, r):
    flag = {'PROVISIONAL': ' *(prov.)*', 'EMPTY': '', 'FLOOR': ' *(floor)*',
            'SUPERSEDED': ' **(SUPERSEDED - re-score in flight)**'}.get(r['status'], '')
    if r['status'] == 'EMPTY':
        A(f"| `{r['id']}`{flag} | {r['learner']} | {r['statistic']} | {r['action_mode']} "
          f"| - | **EMPTY** | **EMPTY** | | | | | | | | | {r['verdict']} |")
        return
    ci = f"[{fmt(r['ci_lo'])}, {fmt(r['ci_hi'])}]" if r['ci_lo'] != '' else ''
    if 'undef.' in ci:
        ci = 'undefined (zero within-arm spread)'
    A(f"| `{r['id']}`{flag} | {r['learner']} | {r['statistic']} | {r['action_mode']} "
      f"| {r['n_seeds_human']}v{r['n_seeds_machine']}x{r['episodes_per_seed']} "
      f"| {fmt(r['human_rate'])} | {fmt(r['machine_rate'])} | {fmt(r['delta'])} | {ci} "
      f"| {fmt(r['perm_p'])} | {fmt(r['mde_80'])} | {fmt(r['p_rope'])} "
      f"| {fmt(r['bf01'], 1)} | {_dead(r)} | {r.get('prior_flip','')} | {r['verdict']} |")


def fmt(x, nd=3):
    import math as _m
    if x == '' or x is None:
        return ''
    if isinstance(x, float):
        return 'undef.' if _m.isnan(x) else f'{x:.{nd}f}'
    return str(x)


def render_md(results, stale, doc_issues, rescore, comp, nodes, tidy):
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

    sup = [r for r in results if r['status'] == 'SUPERSEDED']
    if sup:
        A('\n> ## SUPERSEDED - re-score in flight\n>')
        A('> These rows are computed on inputs that are BEING OVERWRITTEN as you read them. This '
          'is not the same as *provisional*: provisional means the number may move when more '
          'seeds land, superseded means the cells it is computed from are actively being '
          'replaced. **Do not quote these.** They are excluded from the forest plot.\n>')
        A('> | row | statistic | current value | why |')
        A('> |---|---|---|---|')
        for r in sup:
            val = (f"{fmt(r['human_rate'])} v {fmt(r['machine_rate'])}"
                   if r['human_rate'] != '' else '-')
            A(f"> | `{r['id']}` | {r['statistic']} | {val} | {r['rescore_in_flight']} |")
        A('>')

    groups = []
    for r in results:
        if r['group'] not in groups:
            groups.append(r['group'])
    def _hdr():
        A('| comparison | learner | statistic | act | n | human | machine | Delta | 95% CI | p | '
          'MDE | P(ROPE) | BF01 | dead | prior | verdict |')
        A('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')

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

    seedsets = collections.defaultdict(dict)
    for t in tidy:
        seedsets[t['comparison']].setdefault(t['arm_role'], set()).add(t['seed'])
    pairs, partial = [], []
    for r in results:
        b = next((x for x in results if x['id'] == r.get('rescored_from')), None)
        if b is None or not r.get('rescored_from'):
            continue
        if r['delta'] == '' or b['delta'] == '':
            partial.append((r, b, 'the re-score has not produced a testable cell yet'))
        elif seedsets[r['id']] != seedsets[b['id']]:
            # a partial re-score compared against a complete record cell would confound the
            # correction with which seeds happen to have landed
            have = {k: len(v) for k, v in sorted(seedsets[r['id']].items())}
            want = {k: len(v) for k, v in sorted(seedsets[b['id']].items())}
            partial.append((r, b, f're-score incomplete: {have} of {want} seeds, so its movement '
                                  f'is not yet separable from which seeds have landed'))
        else:
            pairs.append((r, b))
    if pairs or partial:
        A('\n## What the re-score moved\n')
        A('Each arm is corrected by the same construction, so per-arm movement looks small and '
          'reassuring. **It is the movement in the DIFFERENCE that bears on the comparison**, '
          'and it is not small: symmetric corrections are not symmetric in effect.\n')

    if pairs:
        A('| cell | human | machine | **Delta** | p |')
        A('|---|---|---|---|---|')
        for a, b in pairs:
            A(f"| `{b['id']}` (as recorded) | {fmt(b['human_rate'])} | {fmt(b['machine_rate'])} "
              f"| {fmt(b['delta'])} | {fmt(b['perm_p'])} |")
            A(f"| `{a['id']}` (re-scored, of record) | {fmt(a['human_rate'])} "
              f"| {fmt(a['machine_rate'])} | {fmt(a['delta'])} | {fmt(a['perm_p'])} |")
            dh = a['human_rate'] - b['human_rate']; dm = a['machine_rate'] - b['machine_rate']
            dd = a['delta'] - b['delta']
            ratio = (f", a {abs(a['delta'] / b['delta']):.1f}x change"
                     if b['delta'] not in (0, '') and abs(b['delta']) > 1e-9 else "")
            A(f"| **movement** | {dh:+.3f} | {dm:+.3f} | **{dd:+.3f}**{ratio} | |")
        A('')
        A('The arms move by comparable amounts and in opposite directions, so the gap moves by '
          'more than either arm does. A reader who checks only per-arm movement would conclude '
          'the correction was harmless.')
        A('')
    for a, b, why in partial:
        A(f"- `{a['id']}` vs `{b['id']}`: movement NOT computed - {why}.")
    if partial:
        A('')

    A('\n## Re-score reproducibility\n')
    if rescore:
        A('Cells whose EPISODES reached different terminal states between the cell of record and '
          'its corrected-predicate `_cp` re-score. Column changes with zero differing episodes '
          'are structural and are excluded here (see below).')
        A('')
        A('| run | cell | arm | episodes differing | of | columns affected | original record |')
        A('|---|---|---|---|---|---|---|')
        for run, cell, arm, ed, n, cols, oh, oc, oi in rescore:
            A(f"| `{run}` | {cell} | {arm} | **{ed}** | {n} | {cols or '-'} "
              f"| {oh} ({oc}-core {oi}) |")
        A('')
    else:
        A('Every cell reproduces episode-for-episode between the cell of record and its `_cp` '
          're-score.')
        A('')

    if nodes:
        A('')
        A('### Why some cells did not reproduce: the hardware class, after all')
        A('')
        A('**This section corrects an earlier version of itself.** It previously reported that '
          'the non-reproducing re-scores were NOT a hardware effect and were confined to the '
          'human arm. That was wrong, and the fault was in the node attribution: cells were '
          "attributed to the node in the run's `events.out.tfevents` filename, which is the "
          'TRAINING node, while the evaluations ran as separate CPU jobs. Attributing each cell '
          'to the job that logged writing THAT directory reverses the finding.')
        A('')
        A('**Movement separates perfectly on the original record\'s hardware class.** A cell '
          'counts as moved only when episodes reach different terminal states; see the '
          'structural note below.')
        A('')
        A('| original record | ISA | cells moved |')
        A('|---|---|---|')
        for k in sorted({r['orig_cores'] for r in nodes['rows']}, key=lambda x: (x == '', x)):
            mv = nodes['by_cores'][(k, True)]; tot = mv + nodes['by_cores'][(k, False)]
            A(f"| {k}-core | {nodes['isa_of'].get(k, '?')} | **{mv} / {tot}** |")
        A('')
        hosts = sorted({r['orig_host'] for r in nodes['movers']})
        A(f"Every mover traces to one of two 36-core AVX2 machines ({', '.join(hosts)}), one "
          'hosting the eight end-to-end cells and the other the eight contact cells. No other '
          'hardware class produced a single non-reproducing cell.')
        A('')
        A('**The arm asymmetry is a scheduling accident, not a bias.** No machine-arm cell was '
          'ever evaluated on the 36-core class at all, so that hardware could only ever have '
          'moved human-arm cells:')
        A('')
        A('| arm | cells on 36-core | cells on other classes |')
        A('|---|---|---|')
        for arm in ('human', 'machine'):
            A(f"| {arm} | {nodes['by_arm_exp'][(arm, True)]} | "
              f"{nodes['by_arm_exp'][(arm, False)]} |")
        A('')
        A('**Re-scoring is therefore not directionally biased.** The earlier alarming reading - '
          'an unexplained defect moving one arm only - is withdrawn. What remains is the known '
          'cross-hardware-class confound, which is why affected rows stay provisional pending '
          'their pinned re-runs.')
        A('')
        sc = collections.Counter(c for r in nodes['struct'] for c in r['structural_cols'].split('|') if c)
        sa = collections.Counter(r['arm'] for r in nodes['struct'])
        A(f"**Structural column changes are counted separately and are NOT movement.** "
          f"{len(nodes['struct'])} cells have a column that was structurally zero become "
          f"non-zero with ZERO differing episodes "
          f"({', '.join(f'`{k}` x{v}' for k, v in sc.most_common())}). That is the "
          'corrected-predicate fix making a previously unearnable outcome earnable. It is not '
          f"arm-directional (human {sa['human']}, machine {sa['machine']}), and counting it as "
          'movement is what produced the earlier one-sided picture.')
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
    # SUPERSEDED rows are deliberately NOT plotted: their inputs are being overwritten, so
    # drawing them invites exactly the misreading the status exists to prevent.
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
    sup = [r['id'] for r in results if r['status'] == 'SUPERSEDED']
    msg = ('A CI inside the shaded band supports equivalence; a CI wider than the band means '
           'the cell is underpowered, not that the arms match.')
    if dropped:
        msg += ('   Not plotted (no defined interval: every seed in each arm gave the identical '
                'count): ' + ', '.join(dropped) + '.')
    if sup:
        msg += ('   Not plotted (SUPERSEDED - a re-score is overwriting these inputs): '
                + ', '.join(sup) + '.')
    fig.text(0.012, 0.012, '\n'.join(textwrap.wrap(msg, 150)), fontsize=8.0, color='#444',
             va='bottom')
    fig.tight_layout(rect=(0, 0.075, 1, 1))
    # Deterministic output: matplotlib stamps /CreationDate into the PDF, which makes an
    # otherwise byte-identical rebuild look changed to git. Suppress it so a rebuild with
    # unchanged data really is a no-op (the README claims idempotence; this makes it true).
    fig.savefig(path_png, dpi=200, metadata={'Software': 'HRI_results/make_tables.py'})
    fig.savefig(path_pdf, metadata={'CreationDate': None,
                                    'Producer': 'HRI_results/make_tables.py'})
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
    for extra in ('dead_human', 'dead_machine', 'rescored_from', 'rescore_in_flight'):
        if extra not in fields:
            fields.append(extra)
    for r in results:
        for k in fields:
            r.setdefault(k, '')
    with open(RESULTS, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader(); w.writerows(results)
    with open(os.path.join(HERE, 'results.md'), 'w') as f:
        f.write(render_md(results, stale, doc_issues, rescore, comp, nodes, tidy))
    n = make_fig(results, os.path.join(HERE, 'fig_effects.png'),
                 os.path.join(HERE, 'fig_effects.pdf'))

    import methodology
    methodology.write(os.path.join(HERE, 'METHODOLOGY.md'))

    ok = sum(r['status'] == 'OK' for r in results)
    pv = sum(r['status'] == 'PROVISIONAL' for r in results)
    em = sum(r['status'] == 'EMPTY' for r in results)
    fl = sum(r['status'] == 'FLOOR' for r in results)
    sp = sum(r['status'] == 'SUPERSEDED' for r in results)
    bad = [g for g, v in comp.items() if not v['comparable']]
    if bad:
        print('NOT CROSS-LEARNER COMPARABLE: ' + '; '.join(bad))
    print(f'rows: {ok} OK, {pv} provisional, {sp} SUPERSEDED (re-score in flight), {em} empty, '
          f'{fl} floor; {len(tidy)} per-seed records; {n} plotted')
    if sp:
        print('SUPERSEDED - re-score overwriting these inputs, do not quote: '
              + ', '.join(r['id'] for r in results if r['status'] == 'SUPERSEDED'))
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
