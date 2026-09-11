#!/usr/bin/env python3
"""Does the local re-execution reproduce what the tape recorded?

`paper/SLIDE_CLAUSE5_LINEAGE_2026-09-07.md` §7 reports that the `dHfull_w3` lineage does NOT
re-execute bit-exactly (2/74 there, losing the pick on 5 uids), while the census lineage does.
The re-executed frame logs are only usable as a calibration substrate to the extent they
reproduce the recording, so this measures it rather than assuming it.

Compares, per tape: decision count, summed reward, and the picked / contact / nested / tipped
verdicts the RECORDER wrote against the ones the replay produced.

usage: python3 baselines/diagnostics/replay_fidelity.py --frames <dir-with-frames/>
"""
import argparse
import glob
import os

import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--frames', required=True, help='dir containing frames/ep*.npz')
    ap.add_argument('--verbose', action='store_true')
    ap.add_argument('--per-uid', default=None,
                    help='paper/slide_per_uid_2026-09-07.txt -- the in-repo per-uid honest '
                         'reference (columns: uid outcome entry push_cm sim15 push>=3 dist_cm)')
    a = ap.parse_args()
    fs = sorted(glob.glob(os.path.join(a.frames, 'frames', 'ep*.npz'))
                or glob.glob(os.path.join(a.frames, 'ep*.npz')))
    assert fs, f'no frame logs under {a.frames}'
    rows = []
    for f in fs:
        z = np.load(f, allow_pickle=True)
        rows.append(dict(
            uid=int(z['uid']), n_rep=int(z['decisions']), n_tape=int(z['tape_n']),
            r_rep=float(z['ref_reward']), r_tape=float(z['tape_reward']),
            picked_rep=bool(z['ref_picked']), picked_tape=bool(z['tape_picked']),
            contact_rep=bool(z['ref_contact']), contact_tape=bool(z['tape_contact']),
            nested_rep=bool(z['ref_nested_proxy']), nested_tape=bool(z['tape_nested']),
            tipped_rep=bool(z['ref_tipped']), tipped_tape=bool(z['tape_tipped']),
            stage_tape=str(z['tape_stage']), honest=bool(z['ref_nested_honest'])))
    n = len(rows)
    exact = [r for r in rows if r['n_rep'] == r['n_tape'] and abs(r['r_rep'] - r['r_tape']) < 1e-6]
    print(f'{n} tapes re-executed')
    print(f'  decisions AND summed reward identical to the recording: {len(exact)}/{n}')
    for k in ('picked', 'contact', 'nested', 'tipped'):
        agree = sum(1 for r in rows if r[f'{k}_rep'] == r[f'{k}_tape'])
        lost = [r['uid'] for r in rows if r[f'{k}_tape'] and not r[f'{k}_rep']]
        gained = [r['uid'] for r in rows if r[f'{k}_rep'] and not r[f'{k}_tape']]
        print(f'  {k:8s} agree {agree}/{n}  (tape had it, replay lost it: {lost or "-"}; '
              f'replay gained it: {gained or "-"})')
    print(f'  replay reward total {sum(r["r_rep"] for r in rows):.0f} '
          f'vs tape total {sum(r["r_tape"] for r in rows):.0f}')
    print(f'  honest nested from the replay settle: {sum(r["honest"] for r in rows)}/{n}')

    if a.per_uid:
        ref = {}
        for line in open(a.per_uid):
            p = line.split()
            if len(p) >= 2 and p[0].isdigit():
                ref[int(p[0])] = p[1]
        have = [r for r in rows if r['uid'] in ref]
        print(f'\n  vs {os.path.basename(a.per_uid)} ({len(ref)} uids; {len(have)} matched):')
        for lab in sorted({ref[r["uid"]] for r in have}):
            sub = [r for r in have if ref[r['uid']] == lab]
            print(f'    {lab:9s} n {len(sub):2d}  replay honest nested {sum(r["honest"] for r in sub):2d}'
                  f'  replay proxy {sum(r["nested_rep"] for r in sub):2d}'
                  f'  replay contact {sum(r["contact_rep"] for r in sub):2d}')
        miss = sorted(set(ref) - {r['uid'] for r in rows})
        if miss:
            print(f'    uids in the reference with no replay: {miss}')

    if a.verbose:
        print('\n  uid  n_rep/n_tape   r_rep/r_tape   picked  contact  nested  tipped  stage')
        for r in sorted(rows, key=lambda x: x['uid']):
            flag = '' if (r['n_rep'] == r['n_tape'] and abs(r['r_rep'] - r['r_tape']) < 1e-6) else '  <-- DIFFERS'
            print(f'  {r["uid"]:4d}  {r["n_rep"]:4d}/{r["n_tape"]:4d}    {r["r_rep"]:5.1f}/{r["r_tape"]:5.1f}'
                  f'      {int(r["picked_rep"])}/{int(r["picked_tape"])}      {int(r["contact_rep"])}/{int(r["contact_tape"])}'
                  f'       {int(r["nested_rep"])}/{int(r["nested_tape"])}      {int(r["tipped_rep"])}/{int(r["tipped_tape"])}'
                  f'   {r["stage_tape"]}{flag}')


if __name__ == '__main__':
    main()
