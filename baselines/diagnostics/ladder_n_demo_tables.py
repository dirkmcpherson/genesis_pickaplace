#!/usr/bin/env python3
"""Ladder-N demonstration-side tables (LADDER_N_DEMO_CHECK_2026-09-11, Lane 7).

Reads the manifests written by

    baselines/rl/relabel_reward.py --from-records <records> --ladder <L> [--far-release]

for both demonstration sets under every ladder, joins them to Lane 5's per-tape census (which
supplies the uid and the behaviour class), and prints the markdown the doc uses:

  * per arm and variant: sum of reward, tapes reaching each rung, the slide_gain distribution,
    how many tapes pass `far_release`;
  * THE ACCEPTANCE TEST: the human tapes that complete a slide in sim (Lane 5 class 'slide' =
    the tapes the staged ladder pays its top rung) must earn near the maximum under
    `nested_ramp`. Every one that does not is listed with the clause that failed;
  * the staged cross-check against Lane 5's census, tape by tape.

    python baselines/diagnostics/ladder_n_demo_tables.py \
        --root /home/j/data/genesis_pickaplace/relabel_dev \
        --census <lane5>/can_pos_recovery/videos_ladder_2026-09-11
"""
import argparse
import json
import os

import numpy as np

ARMS = [('dHfull_all', '{human demonstrations}', 'census_human.json'),
        ('dDPfull_first', '{machine demonstrations}', 'census_machine.json')]
VARIANTS = [('_rnr', 'nested_ramp', False), ('_rnrf', 'nested_ramp', True),
            ('_rns', 'nested_sparse', False), ('_rnsf', 'nested_sparse', True),
            ('_rz', 'staged', False), ('_rs', 'sparse', False)]
RUNGS = ('picked', 'placed_v2', 'farside', 'home', 'contact_push', 'slide_success',
         'nested_v2', 'released', 'pushed')


def load(root, arm, sfx):
    p = os.path.join(root, arm + sfx, 'manifest.json')
    return json.load(open(p)) if os.path.exists(p) else None


def rows_by_file(man):
    return {r['file']: r for r in man['per_tape']}


def pct(vals, q):
    return float(np.percentile(np.asarray(vals, float), q)) if len(vals) else float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', required=True, help='directory holding <arm><suffix>/manifest.json')
    ap.add_argument('--census', default=None, help="Lane 5's videos_ladder_2026-09-11 directory")
    ap.add_argument('--max-return', type=float, default=9.0)
    a = ap.parse_args()

    cens = {}
    for arm, _label, fn in ARMS:
        if a.census and os.path.exists(os.path.join(a.census, fn)):
            c = json.load(open(os.path.join(a.census, fn)))
            cens[arm] = {r['file']: r for r in c['rows']}

    print('## 1. Reward and rungs, per arm and variant\n')
    hdr = ('| arm | ladder | far_release | Σ reward | ' + ' | '.join(RUNGS) + ' | tapes |')
    print(hdr)
    print('|' + '---|' * (len(RUNGS) + 6))
    for arm, label, _ in ARMS:
        for sfx, lad, far in VARIANTS:
            man = load(a.root, arm, sfx)
            if man is None:
                continue
            g = man['tapes_granting']
            print(f"| {label} | `{lad}` | {'on' if far else 'off'} | {man['reward_total_new']:.1f} | "
                  + ' | '.join(str(g.get(k, 0)) for k in RUNGS) + f" | {man['n_tapes']} |")
    print()

    print('## 2. `slide_gain` and `release_far` (ladder-independent; read off the ramp variant)\n')
    print('| arm | p10 (mm) | p50 (mm) | p90 (mm) | max (mm) | ≥ 1 cm | ≥ 10 cm | release ≥ 0.10 m | nested_honest |')
    print('|' + '---|' * 9)
    for arm, label, _ in ARMS:
        man = load(a.root, arm, '_rnr')
        if man is None:
            continue
        print(f"| {label} | {man['slide_gain_p10_m'] * 1000:.1f} | {man['slide_gain_p50_m'] * 1000:.1f} | "
              f"{man['slide_gain_p90_m'] * 1000:.1f} | {man['slide_gain_max_m'] * 1000:.1f} | "
              f"{man['n_tapes_slide_gain_over_1cm']} | {man['n_tapes_slide_gain_over_10cm']} | "
              f"{man['n_tapes_release_far']} | {man['n_tapes_nested_honest']} |")
    print()

    print('## 3. ACCEPTANCE TEST -- the human sim-slides under `nested_ramp`\n')
    for arm, label, _ in ARMS:
        man = load(a.root, arm, '_rnr')
        manf = load(a.root, arm, '_rnrf')
        stg = load(a.root, arm, '_rz')
        if man is None or stg is None:
            continue
        rr, rs = rows_by_file(man), rows_by_file(stg)
        rf = rows_by_file(manf) if manf else {}
        slides = [f for f, r in rs.items() if 'slide_success' in r['grants']]
        print(f'### {label}: {len(slides)} tapes complete a slide in sim '
              f'(the staged ladder pays its top rung)\n')
        if not slides:
            print('_none_\n')
            continue
        print('| uid | tape | class | staged | nested_ramp | of max | + far_release | '
              'slide_gain (mm) | release (mm) | rungs missing |')
        print('|' + '---|' * 10)
        for f in sorted(slides, key=lambda x: rr[x]['new_reward'], reverse=True):
            r, s = rr[f], rs[f]
            c = cens.get(arm, {}).get(f, {})
            miss = [k for k in ('picked', 'placed_v2', 'farside', 'home') if k not in r['grants']]
            print(f"| {c.get('ic_uid', '?')} | `{f.split('-')[1]}` | {c.get('klass', '?')} | "
                  f"{s['new_reward']:.0f} | **{r['new_reward']:.1f}** | "
                  f"{100 * r['new_reward'] / a.max_return:.0f}% | "
                  f"{rf.get(f, {}).get('new_reward', float('nan')):.1f} | "
                  f"{r['slide_gain_m'] * 1000:.0f} | {r['release_dist_m'] * 1000:.0f} | "
                  f"{','.join(miss) or '-'} |")
        got = [rr[f]['new_reward'] for f in slides]
        print(f"\nmean {np.mean(got):.2f} of {a.max_return:g}; "
              f"{sum(1 for g in got if g >= 8.0)}/{len(got)} at >= 8, "
              f"{sum(1 for g in got if g >= a.max_return - 1e-9)}/{len(got)} at the maximum.\n")

    print('## 4. Cross-check: staged reward vs Lane 5\'s census, tape by tape\n')
    for arm, label, _ in ARMS:
        man = load(a.root, arm, '_rz')
        if man is None or arm not in cens:
            print(f'{label}: no census to compare against\n')
            continue
        rr = rows_by_file(man)
        diffs = []
        for f, r in rr.items():
            c = cens[arm].get(f)
            if c is None:
                diffs.append((f, r['new_reward'], None))
            elif abs(float(c['reward_staged']) - r['new_reward']) > 1e-6:
                diffs.append((f, r['new_reward'], float(c['reward_staged'])))
        tot = sum(r['new_reward'] for r in rr.values())
        ctot = sum(float(c['reward_staged']) for c in cens[arm].values())
        print(f'{label}: {len(rr) - len(diffs)}/{len(rr)} tapes identical; '
              f'Σ mine {tot:.1f} vs census {ctot:.1f}')
        for f, mine, theirs in diffs[:20]:
            c = cens[arm].get(f, {})
            print(f'  - uid {c.get("ic_uid", "?")} `{f}`: mine {mine} vs census {theirs}')
        print()


if __name__ == '__main__':
    main()
