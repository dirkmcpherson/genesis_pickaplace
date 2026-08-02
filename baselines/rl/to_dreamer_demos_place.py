"""episodes_all_images -> PLACE-scope dreamer demo episodes (2026-08-02).

Slices each success demo from its PICK-ENTRY frame (baselines/pick_entry_states.json,
the same bank FullTaskEnv scope='place' resets into) to its PLACED_V2 frame + 1.
PLACED_V2 on recorded frames = grip COMMAND < 0.45 (released) + can inside the
shelf footprint/z-band + tilt < 20 deg, sustained 10 consecutive frames -- the
identical predicate FullTaskEnv scope='place' terminates on, so demo slices and
online episodes agree about where the task ends.

Output format = demonstrations/genesis_pick contract (dreamer npz, backward-looking
action/reward convention: index 0 is the entry obs with a zero action; reward 1.0
only on the final frame, is_terminal True there). Actions are normalized to the
env's [-1,1]^7 Box via pick_env.normalize_action (raw demo actions fall outside
[-1,1] -- the v6-v13 scale bug).

Source must carry an 'images' key: baselines/episodes_all_images does (state-only
episodes_all works for the --no-images state slices, but r2dreamer prefill needs
images). Demos whose recorded frames never satisfy placed_v2 are skipped and
reported.

Usage: to_dreamer_demos_place.py [--src baselines/episodes_all_images]
                                 [--dst baselines/demonstrations/genesis_place]
"""
import os
import argparse
import json
import pathlib as pl
import sys

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))
sys.path.insert(0, str(REPO / 'can_pos_recovery'))

import pick_env  # noqa: E402
from replay_harness import tilt_deg, in_shelf_footprint, BOX_TOP_Z  # noqa: E402

RELEASE = 0.45   # FullTaskEnv.PLACE_RELEASE
TILT = 20.0      # FullTaskEnv.PLACE_TILT_DEG
SUSTAIN = 10     # FullTaskEnv.PLACE_SUSTAIN

ap = argparse.ArgumentParser()
ap.add_argument('--src', default='baselines/episodes_all_images')
ap.add_argument('--dst', default='baselines/demonstrations/genesis_place')
ap.add_argument('--bank', default='baselines/pick_entry_states.json')
ap.add_argument('--scan-only', action='store_true',
                help='report per-uid placed_v2 frames without writing npz')
args = ap.parse_args()

bank = {int(u): e for u, e in json.loads((REPO / args.bank).read_text()).items()}
DST = REPO / args.dst
if not args.scan_only:
    DST.mkdir(parents=True, exist_ok=True)
    for _stale in DST.glob('genesis-*.npz'):   # length-encoding filenames: purge old gen
        _stale.unlink()


def placed_v2_frame(s, a, start):
    """First frame at which the predicate has held SUSTAIN consecutive frames."""
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


n_out = n_skip = 0
for uid in sorted(bank):
    p = REPO / args.src / f'{uid}.npz'
    if not p.exists():
        print(f'{uid}: MISSING in {args.src}, skip')
        n_skip += 1
        continue
    d = np.load(p, allow_pickle=True)
    s = d['states'].astype(np.float32)
    a_raw = d['actions'].astype(np.float32)
    k = int(bank[uid]['frame'])
    v2 = placed_v2_frame(s, a_raw, k + 1)
    if v2 is None:
        print(f'{uid}: no placed_v2 in recorded frames (stage {d["stage"]}), skip')
        n_skip += 1
        continue
    end = min(v2 + 2, len(s))              # placed_v2 frame + 1, exclusive slice
    T = end - k
    if args.scan_only:
        print(f'{uid}: entry {k} placed_v2 {v2} T={T} (stage {d["stage"]})')
        continue
    if 'images' not in d.files:
        raise SystemExit(f'{uid}: source {args.src} has no images key -- use '
                         f'episodes_all_images (or re-collect with images)')
    img = d['images'][k:end].astype(np.uint8)
    # backward-looking actions: act[i] led INTO obs[i]; index 0 = entry obs, zero action
    act = pick_env.normalize_action(a_raw[k:end - 1]).astype(np.float32)
    act = np.concatenate([np.zeros_like(act[:1]), act])
    rew = np.zeros(T, dtype=np.float32)
    rew[-1] = 1.0
    is_first = np.zeros(T, dtype=bool); is_first[0] = True
    is_last = np.zeros(T, dtype=bool); is_last[-1] = True
    is_terminal = is_last.copy()           # placed_v2 is a true termination
    np.savez_compressed(
        DST / f'genesis-{uid:04d}-{T}.npz',
        image=img, action=act, reward=rew,
        discount=(1.0 - is_terminal.astype(np.float32)),
        is_first=is_first, is_last=is_last, is_terminal=is_terminal,
        logprob=np.zeros(T, dtype=np.float32))
    n_out += 1
    print(f'{uid}: entry {k} placed_v2 {v2} T={T} (stage {d["stage"]})')

if not args.scan_only:
    print(f'\n{n_out} place demos -> {DST} ({n_skip} skipped)')
