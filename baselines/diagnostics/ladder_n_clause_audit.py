#!/usr/bin/env python3
"""Which clause of `home` fails, on which tape (Lane 7, 2026-09-11).

The user asked for "a version of sparse that enforces the slide" -- `nested_sparse` with
`far_release` ON, which pays 1 for

    home = nested_v2 AND farside-granted AND slide_gain_m >= 0.01,
           with the release that counts taken at >= FAR_RELEASE_DIST_M from the goal.

A count of how many tapes pay it is not enough to judge the definition: the useful number is
WHICH CLAUSE stops each tape that ought to pay. This script reports, per tape and per
candidate reach:

  * the release distance (dist_xy at the first `placed_v2` grant) and whether it clears 0.10 m;
  * the first clause of `home` that fails, in ladder order, including the case where every
    clause is satisfied at some point but never at the SAME frame (`home` needs them
    simultaneous, because `nested_v2` is a statement about the state now);
  * the release-distance distribution per arm and per Lane-5 behaviour class, so the 0.10 m
    clause is justified or adjusted from the data rather than assumed.

    python baselines/diagnostics/ladder_n_clause_audit.py \
        --records /home/j/data/genesis_pickaplace/stage_records \
        --census <lane5>/can_pos_recovery/videos_ladder_2026-09-11 --reach 0.08 0.12
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

REPO = os.environ.get('GENESIS_PICKAPLACE_ROOT',
                      os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(REPO, 'baselines'))
sys.path.insert(0, os.path.join(REPO, 'baselines', 'rl'))

from stage_predicates import FAR_RELEASE_DIST_M, SLIDE_GAIN_MIN_M, StageTracker   # noqa: E402

ARMS = [('dHfull_all', '{human demonstrations}', 'census_human.json'),
        ('dDPfull_first', '{machine demonstrations}', 'census_machine.json')]

# the order `home`'s clauses are checked in, and the label each failure gets
FAIL_ORDER = ('no_pick', 'no_release', 'release_too_close', 'no_farside',
              'gain_under_1cm', 'no_nested_v2', 'not_simultaneous')
FAIL_TEXT = {
    'no_pick': 'the tape never picks the can',
    'no_release': 'picked, but `placed_v2` is never granted (no release the env recognises)',
    'release_too_close': 'released INSIDE the far-release distance (the drop-and-nudge clause)',
    'no_farside': 'the tool is never behind the can AND within the reach cap after the release',
    'gain_under_1cm': 'the can never travels 1 cm goalward from that pose',
    'no_nested_v2': 'the can never ends up settled, upright and within the touch distance',
    'not_simultaneous': 'every clause holds at some point, but never on the same frame',
}


def run_tape(rec, reach, far_release, far_dist):
    """-> per-frame arrays + the clause diagnosis for one tape."""
    tr = StageTracker(rec['goal_pos'][0][:2], float(rec['shelf_top_z']),
                      farside_reach_m=reach, far_release=far_release,
                      far_release_dist_m=far_dist)
    granted = False
    nv2, fs, gain, picked = [], [], [], []
    for i in range(rec['dec'].shape[0]):
        pg = bool(rec['placed_v2'][i]) or granted
        granted = granted or bool(rec['placed_v2'][i])
        f = tr.update(can_pos=rec['can_pos'][i], can_quat=rec['can_quat'][i],
                      goal_pos=rec['goal_pos'][i], goal_quat=rec['goal_quat'][i],
                      tool_xy=rec['tool_xy'][i], grip_cmd=float(rec['grip_cmd'][i]),
                      picked=bool(rec['picked'][i]),
                      can_goal_contact=bool(rec['can_goal_contact'][i]),
                      gripper_goal_contact=bool(rec['gripper_goal_contact'][i]),
                      placed_v2=pg)
        nv2.append(f['nested_v2']); fs.append(f['farside'])
        gain.append(f['slide_gain_m']); picked.append(f['picked'] if 'picked' in f else None)
    ep = tr.episode()
    nv2 = np.asarray(nv2, bool); fs = np.asarray(fs, bool); gain = np.asarray(gain, float)
    ever_picked = bool(np.asarray(rec['picked'], bool).any())
    ever_pv2 = bool(np.asarray(rec['placed_v2'], bool).any())

    # `home` fires on the first frame where all three hold together
    both = nv2 & fs & (gain >= SLIDE_GAIN_MIN_M)
    fail = None
    if ep['home']:
        fail = None
    elif not ever_picked:
        fail = 'no_pick'
    elif not ep['released']:
        fail = 'no_release' if not ever_pv2 else 'no_release'
    elif far_release and not ep['release_far']:
        fail = 'release_too_close'
    elif not fs.any():
        fail = 'no_farside'
    elif gain.max(initial=0.0) < SLIDE_GAIN_MIN_M:
        fail = 'gain_under_1cm'
    elif not nv2.any():
        fail = 'no_nested_v2'
    elif not both.any():
        fail = 'not_simultaneous'
    return dict(ep=ep, fail=fail, home=bool(ep['home']),
                release_dist_m=float(ep['release_dist_m']),
                release_far=bool(ep['release_far']),
                slide_gain_m=float(ep['slide_gain_m']),
                farside=bool(ep['farside']), nested_v2=bool(ep['nested_v2']),
                ever_picked=ever_picked, ever_pv2=ever_pv2,
                n_nested_frames=int(nv2.sum()), n_farside_frames=int(fs.sum()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--records', required=True)
    ap.add_argument('--census', required=True)
    ap.add_argument('--reach', type=float, nargs='+', default=[0.08, 0.12])
    ap.add_argument('--far-dist', type=float, default=FAR_RELEASE_DIST_M)
    a = ap.parse_args()

    cens, files = {}, {}
    for arm, _label, fn in ARMS:
        p = os.path.join(a.census, fn)
        if os.path.exists(p):
            cens[arm] = {r['file']: r for r in json.load(open(p))['rows']}
        fs = sorted(glob.glob(os.path.join(a.records, arm, '*.npz')))
        files[arm] = [f for f in fs if os.path.basename(f) != 'manifest.json']

    # ---- 1. the headline: nested_sparse + far_release ON --------------------------------
    print('# `nested_sparse` with `far_release` ON -- "a version of sparse that enforces '
          'the slide"\n')
    print(f'`home` = nested_v2 AND farside-granted AND slide_gain >= {SLIDE_GAIN_MIN_M * 100:g} cm, '
          f'release at >= {a.far_dist * 100:g} cm. One point per tape, nothing else pays.\n')
    res = {}
    for reach in a.reach:
        for arm, label, _ in ARMS:
            rows = {}
            for f in files[arm]:
                rows[os.path.basename(f)] = run_tape(np.load(f, allow_pickle=True), reach,
                                                     True, a.far_dist)
            res[(reach, arm, True)] = rows

    print('| reach | arm | tapes paying `home` | of tapes | uids |')
    print('|---|---|---|---|---|')
    for reach in a.reach:
        for arm, label, _ in ARMS:
            rows = res[(reach, arm, True)]
            pay = [f for f, r in rows.items() if r['home']]
            uids = sorted(cens.get(arm, {}).get(f, {}).get('ic_uid', -1) for f in pay)
            print(f"| {reach:.2f} m | {label} | **{len(pay)}** | {len(rows)} | "
                  f"{', '.join(str(u) for u in uids) if uids else '-'} |")
    print()

    # ---- 2. per-clause failure on the sim-slide tapes ------------------------------------
    for reach in a.reach:
        print(f'## Clause that fails, reach {reach:.2f} m, far_release ON -- '
              f'the tapes that complete a slide in sim\n')
        for arm, label, _ in ARMS:
            if arm not in cens:
                continue
            rows = res[(reach, arm, True)]
            slides = [f for f in rows if cens[arm].get(f, {}).get('klass') == 'slide']
            print(f'### {label} -- {len(slides)} sim-slide tapes, '
                  f'{sum(1 for f in slides if rows[f]["home"])} pay `home`\n')
            print('| uid | pays | release (cm) | farside frames | slide_gain (cm) | '
                  'nested_v2 frames | failing clause |')
            print('|---|---|---|---|---|---|---|')
            for f in sorted(slides, key=lambda x: cens[arm][x]['ic_uid']):
                r = rows[f]
                print(f"| {cens[arm][f]['ic_uid']} | {'**yes**' if r['home'] else 'no'} | "
                      f"{r['release_dist_m'] * 100:.1f} | {r['n_farside_frames']} | "
                      f"{r['slide_gain_m'] * 100:.2f} | {r['n_nested_frames']} | "
                      f"{'-' if r['home'] else r['fail']} |")
            tally = {}
            for f in slides:
                if not rows[f]['home']:
                    tally[rows[f]['fail']] = tally.get(rows[f]['fail'], 0) + 1
            for k in FAIL_ORDER:
                if tally.get(k):
                    print(f'\n- **{k}** ({tally[k]}): {FAIL_TEXT[k]}')
            print()

    # ---- 3. release-distance distribution ------------------------------------------------
    print('## Release distance -- dist_xy(can, goal) at the first `placed_v2` grant\n')
    print('| arm | tapes that release | p10 | p25 | p50 | p75 | p90 | '
          f'>= {a.far_dist * 100:g} cm |')
    print('|---|---|---|---|---|---|---|---|')
    dists = {}
    for arm, label, _ in ARMS:
        rows = res[(a.reach[0], arm, True)]
        d = np.array([r['release_dist_m'] for r in rows.values()
                      if np.isfinite(r['release_dist_m'])])
        dists[arm] = {f: r['release_dist_m'] for f, r in rows.items()}
        print(f'| {label} | {len(d)}/{len(rows)} | '
              + ' | '.join(f'{np.percentile(d, q) * 100:.1f}' for q in (10, 25, 50, 75, 90))
              + f' | {int((d >= a.far_dist).sum())} ({100 * (d >= a.far_dist).mean():.0f}%) |')
    print()

    print('### ...by Lane-5 behaviour class (the clause exists to separate a slide from a drop)\n')
    print('| arm | class | tapes | release p50 (cm) | '
          f'>= {a.far_dist * 100:g} cm | kept by the clause |')
    print('|---|---|---|---|---|---|')
    for arm, label, _ in ARMS:
        if arm not in cens:
            continue
        byk = {}
        for f, d in dists[arm].items():
            k = cens[arm].get(f, {}).get('klass', '?')
            byk.setdefault(k, []).append(d)
        for k in sorted(byk, key=lambda x: -len(byk[x])):
            v = np.array([x for x in byk[k] if np.isfinite(x)])
            if not len(v):
                print(f'| {label} | {k} | {len(byk[k])} | (never releases) | - | 0 |')
                continue
            n_ok = int((v >= a.far_dist).sum())
            print(f'| {label} | {k} | {len(byk[k])} | {np.median(v) * 100:.1f} | '
                  f'{n_ok}/{len(v)} | {100 * n_ok / len(byk[k]):.0f}% |')
    print()

    # ---- 4. what the clause costs and buys -----------------------------------------------
    print('## What `far_release` costs and buys (reach %.2f m)\n' % a.reach[-1])
    print('| arm | `home` far_release OFF | ON | tapes lost | of those, class |')
    print('|---|---|---|---|---|')
    for arm, label, _ in ARMS:
        rows_on = res[(a.reach[-1], arm, True)]
        rows_off = {os.path.basename(f): run_tape(np.load(f, allow_pickle=True), a.reach[-1],
                                                  False, a.far_dist) for f in files[arm]}
        on = {f for f, r in rows_on.items() if r['home']}
        off = {f for f, r in rows_off.items() if r['home']}
        lost = off - on
        kl = {}
        for f in lost:
            k = cens.get(arm, {}).get(f, {}).get('klass', '?')
            kl[k] = kl.get(k, 0) + 1
        print(f'| {label} | {len(off)} | {len(on)} | {len(lost)} | '
              f"{', '.join(f'{k} x{v}' for k, v in sorted(kl.items())) or '-'} |")
    print()


if __name__ == '__main__':
    main()
