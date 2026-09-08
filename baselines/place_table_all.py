#!/usr/bin/env python3
"""Learner x source PLACE table (PHASE_PLAN amendment (h)): r2dreamer (WM), RLPD and DP runs, human (39) vs machine
matched-39, per-seed counts on holdE / polE in sample and mode, exact two-sided permutation tests human vs machine per
learner and cell. Reads the r2dreamer-layout metrics.json cells (fresh_eval_<bank>_<mode>/metrics.json; `placed_v2`
rate x `episodes`, `restore_failed` counted as failures already).

usage: place_table_all.py [--wm-runs $W/runs] [--rlpd-runs baselines/rl/checkpoints/place] [--dp-runs baselines/outputs/dp_place]
                          [--wm-human 's2_r2d_place_state_dH_bnormclamp1ent5_s{s}'] [--wm-machine 's2_r2d_place_state_dDP_bnormclamp1ent5_n39_s{s}']
                          [--polE-tag polE] [--seeds 0-7]
--polE-tag selects which polE cell directory to read (e.g. `polE` = fresh_eval_polE_*, or a re-scored tag such as
`polEv2` for the rebuilt bank), so the WM cells of record and any re-score are both addressable. Numbers only from the
json files; a missing cell prints '—' and is excluded from the test; a cell with restore failures shows (rf N).

Validated 2026-09-07 against the r2dreamer place cells of record: this script reproduces PHASE_RESULTS_2026-09-05 §2.y
exactly -- human 832/1184 = 0.703 vs machine-39 766/1184 = 0.647 on polE MODE, Δ +8.25 per seed (+0.056), exact p 0.227;
polE SAMPLE Δ +2.00 (+0.014), p 0.743 -- so the learner rows added below are computed by the same code path as the WM row."""
import argparse, glob, itertools, json, os


BANKS = {}   # (bank file, sha12, version) -> {learners} : provenance for the three-learner table
REASONS = {}  # slide_fail_reason census over every contact cell read (a7de6a0 per-clause diagnostics)
NODIAG = []   # contact cells produced before the slide fix -- not reportable


def cell(run_dir, bank, mode, key='placed_v2', learner_arm=('?', '?')):
    f = os.path.join(run_dir, f'fresh_eval_{bank}_{mode}', 'metrics.json')
    if not os.path.exists(f):
        return None
    d = json.load(open(f)); n = int(d['episodes'])
    if n == 0:
        return None
    k = int(round(float(d.get(key, d.get(d.get('success_key', ''), 0.0))) * n))
    rf = int(round(float(d.get('restore_failed', 0.0)) * n))
    extra = int(round(float(d['contact']) * n)) if (key == 'slide_success' and 'contact' in d) else None
    for r_, c_ in (d.get('slide_fail_reasons') or {}).items():
        REASONS[r_] = REASONS.get(r_, 0) + int(c_)
    if 'slide_success' == key and not d.get('slide_diag_available'):
        NODIAG.append(f)
    BANKS.setdefault((os.path.basename(d.get('bank_path') or d.get('entry_bank') or '?'),
                      (d.get('bank_sha256') or 'unstamped')[:12], d.get('bank_version')), set()).add(learner_arm[0])
    return (k, n, rf, d.get('bank_version'), extra)


def fmt(c):
    if c is None:
        return '—'
    t = f'{c[0]}/{c[1]}'
    if len(c) > 4 and c[4] is not None:
        t += f' [c {c[4]}]'
    return t + (f' (rf {c[2]})' if c[2] else '')


def perm(a, b):
    obs = sum(a) / len(a) - sum(b) / len(b); pool = a + b; n = len(a); c = t = 0
    for idx in itertools.combinations(range(len(pool)), n):
        s = set(idx); x = [pool[i] for i in idx]; y = [pool[i] for i in range(len(pool)) if i not in s]
        t += 1; c += abs(sum(x) / n - sum(y) / len(y)) >= abs(obs) - 1e-12
    return obs, c / t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--wm-runs', default=os.environ.get('W', '/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03') + '/runs')
    ap.add_argument('--rlpd-runs', default='baselines/rl/checkpoints/place')
    ap.add_argument('--dp-runs', default='baselines/outputs/dp_place')
    ap.add_argument('--wm-human', default='s2_r2d_place_state_dH_bnormclamp1ent5_s{s}')
    ap.add_argument('--wm-machine', default='s2_r2d_place_state_dDP_bnormclamp1ent5_n39_s{s}')
    ap.add_argument('--phase', choices=('place', 'contact'), default='place',
                    help="place: key placed_v2, runs pl_{rlpd,dp}_*, WM s2_r2d_place_*. contact (SLIDE, amendment (m)): "
                         "key slide_success (the (l') statistic), runs sl_{rlpd,dp}_*, WM s2_r2d_contact_*; bare contact "
                         "is printed beside it.")
    ap.add_argument('--polE-tag', dest='pole_tag', default='polE')
    ap.add_argument('--seeds', default='0-7')
    args = ap.parse_args()
    a, b = args.seeds.split('-'); seeds = list(range(int(a), int(b) + 1))
    KEY = 'placed_v2' if args.phase == 'place' else 'slide_success'
    if args.phase == 'contact':
        if args.wm_human == ap.get_default('wm_human'): args.wm_human = 's2_r2d_contact_state_dH_bnormclamp1ent5_subfloor_s{s}'
        if args.wm_machine == ap.get_default('wm_machine'): args.wm_machine = 's2_r2d_contact_state_dDP_bnormclamp1ent5_n11_s{s}'
        if args.rlpd_runs == ap.get_default('rlpd_runs'): args.rlpd_runs = 'baselines/rl/checkpoints/contact'
        if args.dp_runs == ap.get_default('dp_runs'): args.dp_runs = 'baselines/outputs/dp_contact'
    learners = [('r2dreamer', {'human': os.path.join(args.wm_runs, args.wm_human), 'machine': os.path.join(args.wm_runs, args.wm_machine)}, ('sample', 'mode')),
                ('RLPD', {'human': os.path.join(args.rlpd_runs, f'{"pl" if args.phase == "place" else "sl"}_rlpd_dH_s{{s}}'),
                          'machine': os.path.join(args.rlpd_runs, f'{"pl" if args.phase == "place" else "sl"}_rlpd_dDP_s{{s}}')}, ('sample', 'mode')),
                ('DP', {'human': os.path.join(args.dp_runs, f'{"pl" if args.phase == "place" else "sl"}_dp_dH_s{{s}}'),
                        'machine': os.path.join(args.dp_runs, f'{"pl" if args.phase == "place" else "sl"}_dp_dDP_s{{s}}')}, ('sample',))]
    cells = [('holdE', 'sample'), ('holdE', 'mode'), (args.pole_tag, 'sample'), (args.pole_tag, 'mode')]
    hdr = ' | '.join(f'{b_} {"S" if m == "sample" else "M"}' for b_, m in cells)
    print(f'*statistic: {KEY}' + (' (bare contact in [c N]); PHASE_PLAN (m)/(l\')' if KEY == 'slide_success' else '; PHASE_PLAN (h)') + '*\n')
    print(f'| learner | arm | seed | {hdr} |'); print('|---|---|---|' + '---|' * len(cells))
    stats = []
    for name, tmpl, modes in learners:
        per = {}
        for arm in ('human', 'machine'):
            rows = {}
            for s in seeds:
                rd = tmpl[arm].format(s=s)
                r = {c: cell(rd, c[0], c[1], key=KEY, learner_arm=(name, arm)) for c in cells}
                if any(v is not None for v in r.values()):
                    rows[s] = r
                    print(f'| {name} | {arm} | s{s} | ' + ' | '.join(fmt(r[c]) for c in cells) + ' |')
            per[arm] = rows
            tot = []
            for c in cells:
                xs = [r[c] for r in rows.values() if r[c] is not None]
                tot.append(f'{sum(x[0] for x in xs)}/{sum(x[1] for x in xs)} ({sum(x[0] for x in xs) / max(1, sum(x[1] for x in xs)):.3f}, n={len(xs)})' if xs else '—')
            print(f'| **{name}** | **{arm}** | all | ' + ' | '.join(tot) + ' |')
        for c in cells:
            if c[1] not in modes:
                continue
            ha = [per['human'][s][c][0] for s in per['human'] if per['human'][s][c] is not None]
            ma = [per['machine'][s][c][0] for s in per['machine'] if per['machine'][s][c] is not None]
            if len(ha) >= 2 and len(ma) >= 2:
                o, p = perm(ha, ma); denom = next(per['human'][s][c][1] for s in per['human'] if per['human'][s][c] is not None)
                stats.append(f'- {name} {c[0]} {c[1].upper()}: human {ha} vs machine {ma} -> Δ per-seed {o:+.2f} (rate {o / denom:+.3f}), exact two-sided perm p = {p:.3f} (n={len(ha)} v {len(ma)})')
            else:
                stats.append(f'- {name} {c[0]} {c[1].upper()}: incomplete ({len(ha)} v {len(ma)} seeds)')
    print(); print('\n'.join(stats))
    if REASONS:
        tot = sum(REASONS.values())
        print('\n**Why slide_success failed** (per-clause diagnostic, all cells above; a sub-floor pair is read by reason, not rate):')
        for r_, c_ in sorted(REASONS.items(), key=lambda kv: -kv[1]):
            print(f'- `{r_}`: {c_} episode(s) ({c_ / max(tot, 1):.1%})')
    if NODIAG:
        print(f'\n- **NOT REPORTABLE**: {len(NODIAG)} contact cell(s) lack `slide_diag_available` (evaluator predates the '
              f'settle-gate fix a7de6a0); re-run them before quoting any rate. First: {NODIAG[0]}')
    # Bank provenance (coordinator 2026-09-07): the three-learner table must show every row used the same bank version.
    if BANKS:
        print('\n**Entry-bank provenance** (file @ sha256[:12] / bank_version -> learners):')
        for (b, sha, bv), who in sorted(BANKS.items()):
            print(f'- `{b}` @ `{sha}` / `{bv}` -> {", ".join(sorted(who))}')
        per_file = {}
        for (b, sha, bv), who in BANKS.items():
            per_file.setdefault(b, set()).add((sha, bv))
        split = {b: v for b, v in per_file.items() if len(v) > 1}
        if split:
            print(f'- **BANK MISMATCH**: {split} -- rows were scored on different versions of the same bank name; '
                  f'do NOT combine them into one table until re-scored.')
        elif any(sha == 'unstamped' for (_, sha, _) in BANKS):
            print('- NOTE: some cells predate the bank stamping (2026-09-07); their bank version is not verifiable from the json.')
        else:
            print('- all rows above share one bank version per bank file.')


if __name__ == '__main__':
    main()
