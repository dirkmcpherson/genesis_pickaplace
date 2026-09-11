#!/usr/bin/env python
"""PHASE_PLAN amendment (aa): reproduce Lane 6's tip-guard counts from STAGE RECORDS.

    python baselines/diagnostics/tip_guard_demo_check.py \
        --records /home/j/data/genesis_pickaplace/stage_records/dHfull_all  --set-name human \
        --records /home/j/data/genesis_pickaplace/stage_records/dDPfull_first --set-name machine

Lane 6 measured its guard table (paper/TIP_RULE_2026-09-11.md section 5) from its OWN probe,
`baselines/tip_rule_probe.py`, which re-executes each tape with the rule disabled on the env
INSTANCE (`env.TIP_DEG = 1e9`). Lane 7's stage records are a DIFFERENT re-execution of the
same tapes -- termination suppressed with `env.never_terminate` -- and hold every input the
guards consume (`tool_xy`, `can_pos`, `can_quat`, `grip_cmd`) per ENV FRAME.

Both run the WHOLE action stream, so the two should agree frame for frame, and this script's
`grip` rows are the cross-check that they do: if the `grip` row here does not reproduce Lane
6's, the two re-executions are not the same trajectories and NOTHING below is comparable.

The arithmetic is the arithmetic the env runs, read from the modules that define it:
`stage_predicates.is_in_hand` for the guard, `stage_predicates.tilt_deg` for the tilt,
`full_env.FullTaskEnv.TIP_DEG/GRIP_OPEN` for the constants and `full_env.TIP_GUARD_SUSTAIN`
for the sustain. Nothing here is a second implementation.

Definitions, verbatim from Lane 6 so the numbers are comparable:
  fires            first frame at which (tilt > TIP_DEG AND guard) has held `sustain` frames
  in hand at fire  the tracker's in_hand on that frame
  recovers later   tilt < 20 deg on some later frame while NOT in hand
  free flat MISSED tape has a frame with tilt >= 80 deg and not in hand, and never fires
"""
import argparse
import glob
import json
import os
import pathlib as pl
import sys

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[2]))
for _p in ('baselines', 'baselines/rl', 'can_pos_recovery'):
    sys.path.insert(0, str(REPO / _p))

import stage_predicates as SP          # noqa: E402
import full_env as FE                  # noqa: E402

TIP_DEG = float(FE.FullTaskEnv.TIP_DEG)
GRIP_OPEN = float(FE.FullTaskEnv.GRIP_OPEN)
FLAT_DEG = 80.0                        # Lane 6's "horizontal"
RECOVER_DEG = 20.0                     # Lane 6's "recovers"


def per_frame(rec):
    """(tilt deg, in_hand, grip cmd, decision index) per ENV FRAME of one stage record."""
    can = np.asarray(rec['can_pos'], np.float64)
    tool = np.asarray(rec['tool_xy'], np.float64)
    quat = np.asarray(rec['can_quat'], np.float64)
    tilt = np.asarray([SP.tilt_deg(q) for q in quat], np.float64)
    in_hand = np.asarray([SP.is_in_hand(tool[i], can[i, :2]) for i in range(len(can))], bool)
    return tilt, in_hand, np.asarray(rec['grip_cmd'], np.float64), np.asarray(rec['dec'], int)


def first_sustained(mask, k):
    """First index at which `mask` has been true for k consecutive frames (Lane 6's
    tip_rule_probe._first_sustained, same semantics)."""
    c = 0
    for i, v in enumerate(mask):
        c = c + 1 if v else 0
        if c >= k:
            return i
    return None


def free_mask(tilt, in_hand, grip, guard):
    return (grip < GRIP_OPEN) if guard.startswith('grip') else (~in_hand)


def fire_index(tilt, in_hand, grip, guard, sustain):
    """First frame the rule fires.

    Default (what the env implements, and what Lane 6 measured): the sustain counts frames of
    the CONJUNCTION (tilt > TIP_DEG AND guard).

    A guard name ending in `_gonly` instead sustains the GUARD ALONE and reads the tilt
    instantaneously -- the other reading of amendment (aa)'s sentence "the tracker's not
    in_hand sustained 4 env frames". Kept so the two readings can be COMPARED rather than
    argued about; it is not what the env runs.
    """
    free = free_mask(tilt, in_hand, grip, guard)
    if not guard.endswith('_gonly'):
        return first_sustained((tilt > TIP_DEG) & free, sustain)
    run = 0
    for i in range(len(tilt)):
        run = run + 1 if free[i] else 0
        if run >= sustain and tilt[i] > TIP_DEG:
            return i
    return None


def score_tape(path, guard, sustain):
    rec = np.load(path, allow_pickle=True)
    tilt, in_hand, grip, dec = per_frame(rec)
    f = fire_index(tilt, in_hand, grip, guard, sustain)
    free_flat = bool(((tilt >= FLAT_DEG) & (~in_hand)).any())
    out = dict(file=os.path.basename(path), frames=int(len(tilt)),
               decisions=int(dec[-1]) + 1 if len(dec) else 0,
               free_flat=free_flat, fires=f is not None)
    if f is not None:
        post = slice(f + 1, None)
        out.update(fire_frame=int(f), fire_decision=int(dec[f]),
                   tilt_at_fire=round(float(tilt[f]), 2),
                   grip_at_fire=round(float(grip[f]), 3),
                   in_hand_at_fire=bool(in_hand[f]),
                   lever_at_fire_m=round(float(SP.lever_m(
                       np.asarray(rec['tool_xy'], np.float64)[f],
                       np.asarray(rec['can_pos'], np.float64)[f, :2])), 4),
                   recovers=bool(((tilt[post] < RECOVER_DEG) & (~in_hand[post])).any()),
                   decisions_left=int(dec[-1] - dec[f]))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--records', action='append', required=True,
                    help='stage-record directory (repeat for several sets)')
    ap.add_argument('--set-name', action='append', required=True,
                    help='name for each --records, in the same order')
    ap.add_argument('--guards', nargs='*', default=['grip:1', 'grip:4', 'not_in_hand:1',
                                                    'not_in_hand:4', 'not_in_hand:12'],
                    help='guard:sustain pairs to table (default = Lane 6 section 5 rows)')
    ap.add_argument('--census', action='append', default=None,
                    help='one annotate_demos census JSON per --records, in the same order; '
                         'supplies the file -> ic_uid map so tapes can be named the way '
                         'paper/TIP_RULE_2026-09-11.md names them ("human 274")')
    ap.add_argument('--out', default=None, help='write the per-tape rows here as JSON')
    a = ap.parse_args()
    assert len(a.records) == len(a.set_name), 'one --set-name per --records'

    # file -> "set ic" label, from the annotate_demos censuses (the segment layout carries no
    # ic_uid; the census is where the mapping lives). Absent -> the filename is the label.
    label = {}
    if a.census:
        assert len(a.census) == len(a.records), 'one --census per --records'
        for c, name in zip(a.census, a.set_name):
            cj = json.load(open(c))
            rowsj = cj['rows'] if isinstance(cj, dict) else cj
            for r in rowsj:
                label[r['file']] = f'{name} {r["ic_uid"]}'

    def lbl(f):
        return label.get(f, f)

    sets = {}
    for d, name in zip(a.records, a.set_name):
        fs = sorted(glob.glob(os.path.join(d, '*.npz')))
        assert fs, f'no records in {d}'
        sets[name] = fs
        print(f'[{name}] {len(fs)} stage records in {d}')
    man = {}
    for name, d in zip(a.set_name, a.records):
        mp = os.path.join(d, 'manifest.json')
        if os.path.exists(mp):
            m = json.load(open(mp))
            man[name] = m.get('node')
            print(f'[{name}] records made on {m.get("node")}')

    print(f'\nconstants: TIP_DEG {TIP_DEG:g}  GRIP_OPEN {GRIP_OPEN:g}  '
          f'HELD_LEVER_M {SP.HELD_LEVER_M:g}  flat {FLAT_DEG:g}  recover {RECOVER_DEG:g}\n')
    print('| guard | sustain | set | tapes | terminates | fires while in hand | recovers later '
          '| free-flat MISSED | median decisions left |')
    print('|---|---|---|---|---|---|---|---|---|')
    rows = {}
    for spec in a.guards:
        guard, k = spec.split(':')
        k = int(k)
        for name, fs in sets.items():
            rs = [score_tape(f, guard, k) for f in fs]
            rows[(guard, k, name)] = rs
            fired = [r for r in rs if r['fires']]
            missed = [r for r in rs if r['free_flat'] and not r['fires']]
            left = sorted(r['decisions_left'] for r in fired)
            print('| %s | %d | %s | %d | %d | %d | %d | %d | %s |' % (
                guard, k, name, len(rs), len(fired),
                sum(1 for r in fired if r['in_hand_at_fire']),
                sum(1 for r in fired if r['recovers']), len(missed),
                (int(np.median(left)) if left else '-')))
    # combined rows, the denominator Lane 6 quotes (146 tapes)
    print()
    print('| guard | sustain | ALL tapes | terminates | in hand | recovers | free-flat MISSED |')
    print('|---|---|---|---|---|---|---|')
    for spec in a.guards:
        guard, k = spec.split(':')
        k = int(k)
        rs = [r for name in sets for r in rows[(guard, k, name)]]
        fired = [r for r in rs if r['fires']]
        print('| %s | %d | %d | %d | %d | %d | %d |' % (
            guard, k, len(rs), len(fired),
            sum(1 for r in fired if r['in_hand_at_fire']),
            sum(1 for r in fired if r['recovers']),
            sum(1 for r in rs if r['free_flat'] and not r['fires'])))

    # which tapes change class between the rule of record and the amendment's guard
    base = {r['file']: r for name in sets for r in rows[('grip', 1, name)]}
    new = {r['file']: r for name in sets for r in rows[('not_in_hand', 4, name)]}
    gained = sorted(f for f in base if new[f]['fires'] and not base[f]['fires'])
    lost = sorted(f for f in base if base[f]['fires'] and not new[f]['fires'])
    inhand = sorted(f for f in base if base[f]['fires'] and base[f]['in_hand_at_fire'])
    print(f'\ngrip@1 -> not_in_hand@4: {len(gained)} tapes GAIN a tip termination, '
          f'{len(lost)} LOSE one')
    print(f'the rule of record\'s IN-HAND firings ({len(inhand)}): '
          + ', '.join(f'{lbl(f)} lever {base[f]["lever_at_fire_m"]:.3f} m' for f in inhand))
    for f in lost:
        print(f'  LOST  {lbl(f)}: grip fired @d{base[f]["fire_decision"]} '
              f'(tilt {base[f]["tilt_at_fire"]}, grip {base[f]["grip_at_fire"]}, '
              f'in_hand {int(base[f]["in_hand_at_fire"])})')
    miss = sorted(f for f in new if new[f]['free_flat'] and not new[f]['fires'])
    print(f'free-flat cans the AMENDMENT still misses ({len(miss)}): '
          + (', '.join(lbl(f) for f in miss) or 'none'))
    gained_lbl = ', '.join(lbl(f) for f in gained)
    print(f'tapes that GAIN a tip termination: {gained_lbl}')
    if a.out:
        json.dump({f'{g}:{k}:{n}': v for (g, k, n), v in rows.items()},
                  open(a.out, 'w'), indent=1)
        print(f'\nwrote {a.out}')


if __name__ == '__main__':
    main()
