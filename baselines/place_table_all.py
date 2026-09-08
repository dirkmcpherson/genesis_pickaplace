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


def cell(run_dir, bank, mode, key='placed_v2'):
    f = os.path.join(run_dir, f'fresh_eval_{bank}_{mode}', 'metrics.json')
    if not os.path.exists(f):
        return None
    d = json.load(open(f)); n = int(d['episodes'])
    if n == 0:
        return None
    k = int(round(float(d.get(key, d.get(d.get('success_key', ''), 0.0))) * n))
    rf = int(round(float(d.get('restore_failed', 0.0)) * n))
    extra = int(round(float(d['contact']) * n)) if (key == 'slide_success' and 'contact' in d) else None
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
                r = {c: cell(rd, c[0], c[1], key=KEY) for c in cells}
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


if __name__ == '__main__':
    main()
