#!/usr/bin/env python
"""PHASE_PLAN amendment (aa): what the tip guard costs a LADDER, per demonstration tape.

    python baselines/diagnostics/tip_guard_ladder_diff.py \
        --records /home/j/data/genesis_pickaplace/stage_records/dHfull_all --set-name human \
        --census can_pos_recovery/videos_ladder_2026-09-11/census_human.json \
        --records /home/j/data/genesis_pickaplace/stage_records/dDPfull_first --set-name machine \
        --census can_pos_recovery/videos_ladder_2026-09-11/census_machine.json \
        --ladder staged --ladder nested_ramp:far

Scores every stage record TWICE under one ladder -- once with the tip guard of record
('grip') and once with amendment (aa)'s ('not_in_hand') -- through
`relabel_reward.offline_episode`, which is the same function `--from-records` calls and which
shares `full_env.LadderAccountant` with the live env. Nothing here re-implements a reward.

It is the in-memory form of

    relabel_reward.py --from-records ... --tip-guard grip        --out <set>_rz
    relabel_reward.py --from-records ... --tip-guard not_in_hand --out <set>_rzh

and exists because writing the two sets takes ~8 minutes each while the diff takes seconds.
The two agree by construction (same call); the written sets are the artefact a launcher eats.

`<ladder>:far` appends `--far-release`.
"""
import argparse
import glob
import json
import os
import pathlib as pl
import statistics
import sys

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[2]))
for _p in ('baselines', 'baselines/rl', 'can_pos_recovery'):
    sys.path.insert(0, str(REPO / _p))

import relabel_reward as RR            # noqa: E402
import full_env as FE                  # noqa: E402

GUARDS = ('grip', 'not_in_hand')


def score(rec, ladder, far, guard):
    r = RR.offline_episode(rec, ladder, far_release=far, tip_guard=guard)
    return dict(reward=float(np.asarray(r['rewards']).sum()),
                end_reason=r['end_reason'], end_decision=int(r['end_decision']),
                grants=dict(r['grants']), paid=set(r['paid']), n_dec=int(r['n_dec']))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--records', action='append', required=True)
    ap.add_argument('--set-name', action='append', required=True)
    ap.add_argument('--census', action='append', default=None)
    ap.add_argument('--ladder', action='append', required=True,
                    help='ladder name, optionally "<name>:far" for --far-release')
    a = ap.parse_args()
    assert len(a.records) == len(a.set_name)

    label, sets = {}, {}
    for i, (d, name) in enumerate(zip(a.records, a.set_name)):
        sets[name] = sorted(glob.glob(os.path.join(d, '*.npz')))
        assert sets[name], d
        if a.census:
            cj = json.load(open(a.census[i]))
            for r in (cj['rows'] if isinstance(cj, dict) else cj):
                label[r['file']] = f'{name} {r["ic_uid"]}'

    def L(f):
        return label.get(os.path.basename(f), os.path.basename(f))

    for spec in a.ladder:
        ladder, far = (spec.split(':')[0], spec.endswith(':far'))
        print('\n' + '=' * 78)
        print(f'LADDER {ladder}  far_release={far}   '
              f'stamp(grip)      : {FE.ladder_stamp(ladder, None, far, "grip")}')
        print(f'{" " * 40}stamp(not_in_hand): '
              f'{FE.ladder_stamp(ladder, None, far, "not_in_hand")}')
        for name, fs in sets.items():
            res = {g: {} for g in GUARDS}
            for f in fs:
                # MATERIALISE the record. `offline_episode` indexes `rec[key][i]` inside its
                # per-frame loop, and an NpzFile decompresses the whole member on EVERY key
                # access -- O(frames x keys x array) instead of O(frames). A dict of the same
                # arrays is the identical input with the decompression done once.
                with np.load(f, allow_pickle=True) as z:
                    rec = {k: z[k] for k in z.files}
                for g in GUARDS:
                    res[g][f] = score(rec, ladder, far, g)
            tot = {g: sum(res[g][f]['reward'] for f in fs) for g in GUARDS}
            ends = {g: {} for g in GUARDS}
            for g in GUARDS:
                for f in fs:
                    k = res[g][f]['end_reason']
                    ends[g][k] = ends[g].get(k, 0) + 1
            print(f'\n-- {{{name} tapes}} n={len(fs)}')
            print(f'   reward total : grip {tot["grip"]:.1f} -> not_in_hand '
                  f'{tot["not_in_hand"]:.1f}  (delta {tot["not_in_hand"] - tot["grip"]:+.1f})')
            print(f'   end reasons  : grip        {dict(sorted(ends["grip"].items()))}')
            print(f'                  not_in_hand {dict(sorted(ends["not_in_hand"].items()))}')
            rungs = sorted(FE.ladder_spec(ladder)[0]) + ['slide_event', 'home', 'nested_v2',
                                                         'contact_push', 'slide_success']
            seen = []
            for rung in dict.fromkeys(rungs):
                ga = [f for f in fs if rung in res['grip'][f]['grants']]
                gb = [f for f in fs if rung in res['not_in_hand'][f]['grants']]
                lost = [L(f) for f in ga if f not in gb]
                gained = [L(f) for f in gb if f not in ga]
                seen.append((rung, len(ga), len(gb), lost, gained))
            for rung, na, nb, lost, gained in seen:
                mark = '' if (not lost and not gained) else '   <-- MOVES'
                print(f'   {rung:16s} grip {na:3d}  not_in_hand {nb:3d}'
                      + (f'   lost {lost}  gained {gained}' if mark else '') + mark)
            moved = [f for f in fs
                     if abs(res['grip'][f]['reward'] - res['not_in_hand'][f]['reward']) > 1e-9]
            print(f'   tapes whose REWARD moves: {len(moved)}')
            for f in moved:
                x, y = res['grip'][f], res['not_in_hand'][f]
                print(f'     {L(f)}: {x["reward"]:.1f} -> {y["reward"]:.1f}  '
                      f'lost {sorted(x["paid"] - y["paid"]) or "-"}  '
                      f'gained {sorted(y["paid"] - x["paid"]) or "-"}  '
                      f'end {x["end_reason"]}@{x["end_decision"]} -> '
                      f'{y["end_reason"]}@{y["end_decision"]}')
            cut = [res['grip'][f]['end_decision'] - res['not_in_hand'][f]['end_decision']
                   for f in fs
                   if res['grip'][f]['end_reason'] != res['not_in_hand'][f]['end_reason']]
            if cut:
                print(f'   tapes whose END REASON moves: {len(cut)}; decisions saved '
                      f'median {statistics.median(cut):.0f} min {min(cut)} max {max(cut)}')


if __name__ == '__main__':
    main()
