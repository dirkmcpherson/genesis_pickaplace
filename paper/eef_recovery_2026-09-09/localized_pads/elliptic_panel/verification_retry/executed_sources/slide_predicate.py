#!/usr/bin/env python3
"""slide_success, settled definition (2026-09-09).

Three STATE conditions, no free threshold to calibrate. The two earlier
definitions failed because both inferred "the can was released" from the
GRIPPER: (l) required grip_cmd < 0.3, which passes 2 of 74 human tapes because
people release fully, re-close to ~0.4 and push the can home with the fingers
partly shut; (p) replaced it with a displacement clause that was never
calibrated. Release is a fact about where the can's weight is, not about the hand.

  1. RELEASED  - the can stays put while the tool moves away from it. A held can
                 tracks the tool; a released one does not. No gripper term, and
                 no dependence on the tool point (the wrist-vs-tool distinction
                 that made the old `contact` clause vacuous).
  2. PUSHED    - after release, the can gets closer to the goal. Carrying is
                 already excluded by requiring release FIRST, so no geometric
                 far-side test is needed to rule it out.
  3. ARRIVED   - at the end, the can is within the nested proximity of the goal
                 and is not tipped.

usage: slide_predicate.py <tape_dir> [more_dirs...] [--out results.json]
"""
import sys, json, glob, os
import numpy as np

NESTED_M   = 0.081   # can diameter + 15 mm noise floor (metric of record)
STILL_M    = 0.002   # can counts as stationary under 2 mm of motion
TOOL_AWAY_M= 0.010   # tool must move 10 mm while the can holds still
HOLD_N     = 10      # frames the release must be sustained
GAIN_M     = 0.010   # the push must close 10 mm toward the goal

def classify(f):
    z = np.load(f, allow_pickle=True)
    s = z['states'].astype(np.float64)          # [q(6), grip, effort, can xyz, can quat, goal xy]
    can, goal = s[:, 8:11], s[:, 15:17]
    ee = z['eef_pos'].astype(np.float64)[:len(s)]
    tipped = z['tipped'].astype(bool) if 'tipped' in z.files else np.zeros(len(s), bool)
    uid = str(z['uid']) if 'uid' in z.files else os.path.basename(f)[:-4]
    T = len(s)
    out = dict(uid=uid, n=T, released=False, release_frame=None,
               pushed=False, arrived=False, slide_success=False,
               final_dist=None, gain_m=0.0)
    if T < HOLD_N + 2:
        return out
    dist = np.linalg.norm(can[:, :2] - goal, axis=1)
    out['final_dist'] = float(dist[-1])
    # (1) released: can still for HOLD_N frames while the tool moves away
    for t in range(T - HOLD_N):
        w = slice(t, t + HOLD_N)
        if np.max(np.linalg.norm(can[w] - can[t], axis=1)) > STILL_M:
            continue
        if np.max(np.linalg.norm(ee[w] - ee[t], axis=1)) < TOOL_AWAY_M:
            continue                      # tool stationary too: still holding, or idle
        if tipped[t]:
            continue
        out['released'] = True; out['release_frame'] = int(t); break
    if not out['released']:
        return out
    # (2) pushed: after release the can closes on the goal
    r = out['release_frame']
    gain = float(dist[r] - np.min(dist[r:]))
    out['gain_m'] = gain
    out['pushed'] = gain >= GAIN_M
    # (3) arrived: settled within the nested proximity, upright
    out['arrived'] = bool(dist[-1] <= NESTED_M and not tipped[-1])
    out['slide_success'] = bool(out['released'] and out['pushed'] and out['arrived'])
    return out

if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    outp = 'slide_predicate_results.json'
    if '--out' in sys.argv:
        outp = sys.argv[sys.argv.index('--out') + 1]
    rows = []
    for d in args:
        for f in sorted(glob.glob(os.path.join(d, '*.npz'))):
            try:
                rows.append(classify(f))
            except Exception as e:
                print(f'  SKIP {os.path.basename(f)}: {type(e).__name__} {e}')
    ok = [r for r in rows if r['slide_success']]
    print(f'\n{len(rows)} tapes | released {sum(r["released"] for r in rows)} '
          f'| pushed {sum(r["pushed"] for r in rows)} | arrived {sum(r["arrived"] for r in rows)} '
          f'| SLIDE_SUCCESS {len(ok)}')
    print('successes:', ' '.join(sorted(r['uid'] for r in ok)) or '(none)')
    json.dump(rows, open(outp, 'w'), indent=1)
    print('wrote', outp)
