#!/usr/bin/env python3
"""Calibrate Ladder N's `FARSIDE_REACH_M` against the demonstrations (Lane 7, 2026-09-11).

The brief fixes `farside` at |tool_xy - can_xy| <= 0.08 m ("close enough to push"). On the
human tapes that number is not a description of a human push: it sits at the MEDIAN of the
lever the tool actually holds while the can is moving goalward, so it discards about half of
the demonstrated slide and, in the tapes measured first, all of it.

This script measures rather than argues. For each demonstration set it reports

  (a) the distribution of the tool-to-can lever over the frames where the can actually moves
      goalward AFTER the release -- the empirical "pushing lever";
  (b) the Ladder-N rung counts and rewards as a function of the reach cap;
  (c) the acceptance test (how much of the maximum the sim-slide tapes earn) at each cap.

    python baselines/diagnostics/ladder_n_farside_sweep.py \
        --records /home/j/data/genesis_pickaplace/stage_records \
        --reach 0.08 0.10 0.12 0.15 0.20
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

from relabel_reward import offline_episode              # noqa: E402
from stage_predicates import StageTracker               # noqa: E402

ARMS = [('dHfull_all', '{human demonstrations}'), ('dDPfull_first', '{machine demonstrations}')]
MOVE_EPS = 2e-4      # m per env frame; below this the can is noise, not travelling


def lever_census(rec):
    """Attribute every goalward millimetre after the release to the tool pose that made it."""
    tr = StageTracker(rec['goal_pos'][0][:2], float(rec['shelf_top_z']))
    granted, per = False, []
    for i in range(rec['dec'].shape[0]):
        pg = bool(rec['placed_v2'][i]) or granted
        granted = granted or bool(rec['placed_v2'][i])
        per.append(tr.update(
            can_pos=rec['can_pos'][i], can_quat=rec['can_quat'][i], goal_pos=rec['goal_pos'][i],
            goal_quat=rec['goal_quat'][i], tool_xy=rec['tool_xy'][i],
            grip_cmd=float(rec['grip_cmd'][i]), picked=bool(rec['picked'][i]),
            can_goal_contact=bool(rec['can_goal_contact'][i]),
            gripper_goal_contact=bool(rec['gripper_goal_contact'][i]), placed_v2=pg))
    rel = next((i for i, f in enumerate(per) if f['released']), None)
    if rel is None:
        return None
    d = np.array([f['dist_xy_m'] for f in per])
    lev = np.array([f['lever_m'] for f in per])
    dot = np.array([f['dot_tool_goal'] for f in per])
    step = -np.diff(d)
    idx = np.where(step > MOVE_EPS)[0]
    idx = idx[idx >= rel]
    if not len(idx):
        return dict(total_mm=0.0, levers=np.zeros(0), mm=np.zeros(0), far=np.zeros(0, bool))
    return dict(total_mm=float(step[idx].sum() * 1000), levers=lev[idx], mm=step[idx] * 1000,
                far=(dot[idx] < 0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--records', required=True)
    ap.add_argument('--reach', type=float, nargs='+', default=[0.08, 0.10, 0.12, 0.15, 0.20])
    ap.add_argument('--census', default=None, help="Lane 5's videos_ladder_2026-09-11 dir")
    ap.add_argument('--max-return', type=float, default=9.0)
    a = ap.parse_args()

    cens = {}
    for arm, _ in ARMS:
        fn = os.path.join(a.census or '', 'census_%s.json' % ('human' if 'dH' in arm else 'machine'))
        if a.census and os.path.exists(fn):
            cens[arm] = {r['file']: r for r in json.load(open(fn))['rows']}

    print('## A. Where the goalward motion happens: the lever while the can travels\n')
    print('| arm | tapes that move after release | mm moved | lever p10 | p50 | p90 | max | '
          'mm at lever <= 0.08 | <= 0.12 | far side |')
    print('|' + '---|' * 10)
    recs = {}
    for arm, label in ARMS:
        files = sorted(glob.glob(os.path.join(a.records, arm, '*.npz')))
        files = [f for f in files if os.path.basename(f) != 'manifest.json']
        if not files:
            continue
        recs[arm] = files
        L, M, F, n_move, tot = [], [], [], 0, 0.0
        for f in files:
            c = lever_census(np.load(f, allow_pickle=True))
            if c is None or not len(c['levers']):
                continue
            n_move += 1
            tot += c['total_mm']
            L.append(c['levers']); M.append(c['mm']); F.append(c['far'])
        L = np.concatenate(L); M = np.concatenate(M); F = np.concatenate(F)
        print(f'| {label} | {n_move}/{len(files)} | {tot:.0f} | '
              + ' | '.join(f'{np.percentile(L, q) * 100:.1f} cm' for q in (10, 50, 90))
              + f' | {L.max() * 100:.1f} cm | {M[L <= 0.08].sum():.0f} mm '
                f"({100 * M[L <= 0.08].sum() / M.sum():.0f}%) | "
                f'{M[L <= 0.12].sum():.0f} mm ({100 * M[L <= 0.12].sum() / M.sum():.0f}%) | '
                f'{M[F].sum():.0f} mm ({100 * M[F].sum() / M.sum():.0f}%) |')
    print()

    print('## B. Rungs and reward as a function of the reach cap (`nested_ramp`, far_release off)\n')
    print('| arm | reach | farside | slide_gain>=1cm | **home** | Σ reward | slide_gain p50 (mm) '
          '| p90 | sim-slide tapes at home | mean of max |')
    print('|' + '---|' * 10)
    per_arm = {}
    for arm, label in ARMS:
        if arm not in recs:
            continue
        per_arm[arm] = {}
        slides = set()
        if arm in cens:
            slides = {f for f, r in cens[arm].items() if r.get('klass') == 'slide'}
        for reach in a.reach:
            rows = []
            for f in recs[arm]:
                rec = np.load(f, allow_pickle=True)
                res = offline_episode(rec, 'nested_ramp', tracker_kw=dict(farside_reach_m=reach))
                rows.append((os.path.basename(f), float(np.asarray(res['rewards']).sum()),
                             res['episode'], res['grants']))
            per_arm[arm][reach] = rows
            g = [e for _, _, e, _ in rows]
            home = sum(1 for e in g if e['home'])
            far = sum(1 for e in g if e['farside'])
            ge1 = sum(1 for e in g if e['slide_gain_m'] >= 0.01)
            gains = np.array([e['slide_gain_m'] for e in g])
            sl = [r for r in rows if r[0] in slides] if slides else []
            sl_home = sum(1 for r in sl if r[2]['home'])
            sl_mean = np.mean([r[1] for r in sl]) if sl else float('nan')
            print(f'| {label} | {reach:.2f} m | {far} | {ge1} | **{home}** | '
                  f'{sum(r[1] for r in rows):.1f} | {np.percentile(gains, 50) * 1000:.1f} | '
                  f'{np.percentile(gains, 90) * 1000:.1f} | '
                  f'{sl_home}/{len(sl) if sl else 0} | '
                  f'{sl_mean:.2f}/{a.max_return:g} ({100 * sl_mean / a.max_return:.0f}%) |')
    print()

    print('## C. The sim-slide tapes, tape by tape (`nested_ramp`, far_release off)\n')
    for arm, label in ARMS:
        if arm not in per_arm or arm not in cens:
            continue
        slides = [f for f, r in cens[arm].items() if r.get('klass') == 'slide']
        if not slides:
            print(f'{label}: no tape completes a slide in sim\n')
            continue
        print(f'### {label} — {len(slides)} sim-slide tapes\n')
        print('| uid | ' + ' | '.join(f'reward @ {r:.2f} m' for r in a.reach)
              + ' | slide_gain @ widest (mm) | home @ widest |')
        print('|' + '---|' * (len(a.reach) + 3))
        for f in sorted(slides, key=lambda x: cens[arm][x]['ic_uid']):
            cells = []
            for reach in a.reach:
                row = next(r for r in per_arm[arm][reach] if r[0] == f)
                cells.append(f'{row[1]:.1f}')
            wide = next(r for r in per_arm[arm][a.reach[-1]] if r[0] == f)
            print(f"| {cens[arm][f]['ic_uid']} | " + ' | '.join(cells)
                  + f" | {wide[2]['slide_gain_m'] * 1000:.0f} | "
                    f"{'yes' if wide[2]['home'] else '**no**'} |")
        print()


if __name__ == '__main__':
    main()
