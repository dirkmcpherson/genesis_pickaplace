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
