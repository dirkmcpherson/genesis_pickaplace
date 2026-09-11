#!/usr/bin/env python3
"""What is actually true at the frame the `nested` PROXY fires.

`full_env.py:699-704` grants the proxy when `info.get('contact')` is true, the grip command is
below 0.3 and both cans are upright. `info['contact']` is `GenesisCanEnv._contact`, which is
STICKY for the whole episode. So the proxy says "the can touched the goal at SOME EARLIER moment,
and right now the grip is commanded open and both cans are upright" -- the can's current position
is not in the predicate. Project documents (E2E_AUDIT_BRIEF §4, E2E_TRAINING_PROBLEMS §0) describe
it as instantaneous. This script prints, for the exact firing frame of every proxy episode,
whether there is can<->goal solver contact and how far the can is from the goal.

Consumes the per-env-frame logs from baselines/eval_e2e_stagerec.py.

usage: python3 baselines/diagnostics/proxy_firing_frame.py --roll <dir-of-arms>
"""
import argparse
import glob
import os
import pathlib as pl

import numpy as np

NESTED_TOUCH_DIST = 0.081


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--roll', required=True)
    a = ap.parse_args()
    for arm in sorted(d for d in os.listdir(a.roll) if os.path.isdir(os.path.join(a.roll, d))):
        rows = []
        for f in sorted(glob.glob(os.path.join(a.roll, arm, 'ep*', 'frames', 'ep*.npz')),
                        key=lambda p: int(pl.Path(p).stem[2:])):
            z = np.load(f, allow_pickle=True)
            if not bool(z['ref_nested_proxy']):
                continue
            pr = z['nested_proxy'].astype(bool)
            i = int(np.argmax(pr))
            can, goal, tool = z['can_pos'], z['goal_pos'], z['tool']
            d = float(np.hypot(can[i, 0] - goal[i, 0], can[i, 1] - goal[i, 1]))
            dmin = float(np.min(np.hypot(can[:i + 1, 0] - goal[:i + 1, 0],
                                         can[:i + 1, 1] - goal[:i + 1, 1])))
            rows.append(dict(ep=int(z['ep']), frame=i, n=int(z['n_frames']), dist=d, dmin=dmin,
                             contact=bool(z['can_goal_contact'][i]),
                             grip=float(z['grip_cmd'][i]), can_z=float(can[i, 2]),
                             lever=float(np.hypot(tool[i, 0] - can[i, 0],
                                                  tool[i, 1] - can[i, 1])),
                             honest=bool(z['ref_nested_honest'])))
        if not rows:
            print(f'\n{{RLPD}} {arm}: the proxy fired in NO episode (nothing to report)')
            continue
        print(f'\n{{RLPD}} {arm}: the proxy fired in {len(rows)} episodes')
        print('   ep  fire_frame/n   dist_at_fire  min_dist_before  contact_now  grip   '
              'lever_mm  can_z_cm  honest')
        for r in rows:
            print(f'  {r["ep"]:3d}  {r["frame"]:6d}/{r["n"]:5d}   {r["dist"] * 1000:8.1f} mm '
                  f'{r["dmin"] * 1000:12.1f} mm       {int(r["contact"])}       '
                  f'{r["grip"]:.3f}  {r["lever"] * 1000:7.1f}   {r["can_z"] * 100:6.2f}     '
                  f'{int(r["honest"])}')
        D = np.array([r['dist'] for r in rows])
        print(f'  can<->goal solver contact AT THE FIRING FRAME: '
              f'{sum(r["contact"] for r in rows)}/{len(rows)}')
        print(f'  within the nesting distance ({NESTED_TOUCH_DIST * 1000:.0f} mm) at that frame: '
              f'{int((D <= NESTED_TOUCH_DIST).sum())}/{len(rows)}')
        print(f'  distance at the firing frame: median {np.median(D) * 1000:.1f} mm, '
              f'range {D.min() * 1000:.1f}-{D.max() * 1000:.1f} mm')


if __name__ == '__main__':
    main()
