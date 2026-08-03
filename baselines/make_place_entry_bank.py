"""DENSE place-scope entry bank (2026-08-03).

pick_entry_states.json holds ONE entry per demo, at the pick grant -- so every
scope='place' episode starts at the pick moment and release-over-shelf is many
hundreds of steps away (successes stay rare). This script banks entries at EVERY
25th frame ALONG each demo's carry segment, so place training can start anywhere
between "just picked" and "almost at the shelf".

Per success demo in --src (baselines/episodes_all):
  - entry frames = every STRIDE-th frame from the pick grant k (reused from
    pick_entry_states.json, same rule as make_pick_phase_datasets) to seg_end;
  - seg_end = placed_v2_frame - 15 when the recorded frames reach PLACED_V2
    (identical offline predicate to to_dreamer_demos_place.py / FullTaskEnv
    scope='place': grip cmd < 0.45 + shelf footprint/z-band + tilt < 20 deg,
    sustained 10 consecutive frames);
  - demos that placed under the OLD mid-lift proxy (npz stage placed/contact/
    nested) but never satisfy placed_v2 still carry valid carry segments:
    seg_end = LAST frame with can_z > CARRY_Z (0.13, FullTaskEnv.PLACE_HELD_Z)
    and grip commanded closed (> GRIP_CLOSED_FRAC);
  - stage='picked' demos (picked, never placed) are excluded -- no carry-to-
    shelf segment to sample.

Output baselines/place_entry_states_dense.json = a LIST of entries (vs the
legacy uid-keyed dict), each with the same restore fields as the legacy bank
(qpos, grip_cmd, grip_obs, can_pos, can_quat, goal_xy) plus:
  uid    demo uid
  frame  frame index within the demo
  frac   position within the carry segment, 0 (pick grant) .. 1 (seg_end)

FullTaskEnv(scope='place', entry_bank=...) accepts either format and samples
list banks uniformly over ENTRIES.

Usage: make_place_entry_bank.py [--src baselines/episodes_all] [--stride 25]
"""
import os
import argparse
import json
import pathlib as pl
import sys

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))
sys.path.insert(0, str(REPO / 'can_pos_recovery'))

import pick_env  # noqa: E402
from replay_harness import tilt_deg, in_shelf_footprint, BOX_TOP_Z  # noqa: E402

RELEASE = 0.45   # FullTaskEnv.PLACE_RELEASE
TILT = 20.0      # FullTaskEnv.PLACE_TILT_DEG
SUSTAIN = 10     # FullTaskEnv.PLACE_SUSTAIN
CARRY_Z = 0.13   # FullTaskEnv.PLACE_HELD_Z: can center above this = still held
V2_MARGIN = 15   # stop banking this many frames BEFORE placed_v2 (entry must
                 # still need a real release, not restore into the grant itself)

ap = argparse.ArgumentParser()
ap.add_argument('--src', default='baselines/episodes_all')
ap.add_argument('--bank', default='baselines/pick_entry_states.json')
ap.add_argument('--out', default='baselines/place_entry_states_dense.json')
ap.add_argument('--stride', type=int, default=25)
args = ap.parse_args()

pick_bank = {int(u): e for u, e in
             json.loads((REPO / args.bank).read_text()).items()}


def placed_v2_frame(s, a, start):
    """First frame at which the predicate has held SUSTAIN consecutive frames
    (verbatim from to_dreamer_demos_place.py)."""
    run = 0
    for j in range(start, len(s)):
        ok = (float(a[j, 6]) < RELEASE
              and in_shelf_footprint(s[j, 8:10])
              and BOX_TOP_Z + 0.01 < float(s[j, 10]) < BOX_TOP_Z + 0.07
              and tilt_deg(s[j, 11:15]) < TILT)
        run = run + 1 if ok else 0
        if run >= SUSTAIN:
            return j
    return None


entries = []
n_v2 = n_carry = n_skip = 0
for uid in sorted(pick_bank):
    p = REPO / args.src / f'{uid}.npz'
    if not p.exists():
        print(f'{uid}: MISSING in {args.src}, skip')
        n_skip += 1
        continue
    d = np.load(p, allow_pickle=True)
    s = d['states'].astype(np.float32)
    a = d['actions'].astype(np.float32)
    stage = str(d['stage'])
    k = int(pick_bank[uid]['frame'])
    v2 = placed_v2_frame(s, a, k + 1)
    if v2 is not None:
        seg_end, rule = v2 - V2_MARGIN, 'placed_v2'
        n_v2 += 1
    elif stage in ('placed', 'contact', 'nested'):
        # old-proxy placements: carry segment ends at the last held frame
        held = np.flatnonzero((s[:, 10] > CARRY_Z)
                              & (a[:, 6] > pick_env.GRIP_CLOSED_FRAC))
        held = held[held >= k]
        if not len(held):
            print(f'{uid}: stage {stage} but no held frames past grant, skip')
            n_skip += 1
            continue
        seg_end, rule = int(held[-1]), 'carry_gate'
        n_carry += 1
    else:
        print(f'{uid}: stage {stage} -- no carry-to-shelf segment, skip')
        n_skip += 1
        continue
    seg_end = max(seg_end, k)          # degenerate segment -> single entry at k
    span = max(seg_end - k, 1)
    frames = list(range(k, seg_end + 1, args.stride))
    for f in frames:
        entries.append(dict(
            uid=int(uid),
            frame=int(f),
            frac=round((f - k) / span, 4),
            qpos=[float(x) for x in s[f, :6]],
            grip_cmd=float(a[f, 6]),
            grip_obs=float(s[f, 6]),
            can_pos=[float(x) for x in s[f, 8:11]],
            can_quat=[float(x) for x in s[f, 11:15]],
            goal_xy=[float(x) for x in s[f, 15:17]]))
    print(f'{uid}: grant {k} seg_end {seg_end} ({rule}, stage {stage}) '
          f'-> {len(frames)} entries')

(REPO / args.out).write_text(json.dumps(entries, indent=1))
fr = np.array([e['frac'] for e in entries])
print(f'\n{len(entries)} entries -> {args.out}  '
      f'({n_v2} placed_v2 demos + {n_carry} carry-gated, {n_skip} skipped); '
      f'frac quartiles {np.percentile(fr, [0, 25, 50, 75, 100]).round(2)}')
