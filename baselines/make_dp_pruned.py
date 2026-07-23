"""DP-only dataset variant: collapse idle dithering in the PRE-PICK segment.

Measured on episodes_pick (66 demos): pre-pick = 27.5% of 132k frames, and 58% of
pre-pick frames are idle (consecutive action delta < 1e-3 rad -- teleop pauses).
DP trains frame-uniform, so a quarter of its capacity models standing still.

Pruning rule (conservative, per user: never touch anything from can-interaction on):
  - j_pick = first frame with can_z > 0.09 AND grip closed (same predicate family as
    relabel_full / env picked).
  - within [0, j_pick - MARGIN): collapse each RUN of consecutive idle frames
    (max |action[t+1]-action[t]| < IDLE_EPS) down to its first frame.
  - [j_pick - MARGIN, end] is copied untouched (grasp runup + all dynamics).
Cadence: moving segments keep their 30Hz spacing; only static pauses shorten. With
absolute joint targets a collapsed pause remains a valid (obs, hold) pair.

Usage: make_dp_pruned.py [--src baselines/episodes_pick] [--dst baselines/episodes_pick_pruned]
"""
import argparse, glob, pathlib as pl
import numpy as np
import sys
sys.path.insert(0, str(pl.Path(__file__).parent)); sys.path.insert(0, str(pl.Path(__file__).parent / 'rl'))
import pick_env

ap = argparse.ArgumentParser()
ap.add_argument('--src', default='baselines/episodes_pick')
ap.add_argument('--dst', default='baselines/episodes_pick_pruned')
ap.add_argument('--margin', type=int, default=150, help='untouched frames before j_pick (5s)')
ap.add_argument('--idle-eps', type=float, default=1e-3)
args = ap.parse_args()

REPO = pl.Path('/home/j/workspace/genesis_pickaplace')
SRC, DST = REPO / args.src, REPO / args.dst
DST.mkdir(exist_ok=True)

tot_in = tot_out = 0
for p in sorted(glob.glob(str(SRC / '*.npz'))):
    d = np.load(p, allow_pickle=True)
    s, a = d['states'], d['actions']
    n = len(s)
    can_z = s[:, 10]; grip = a[:, 6]
    picked_f = (can_z > 0.09) & (grip > pick_env.GRIP_CLOSED_FRAC)
    j = int(np.argmax(picked_f)) if picked_f.any() else n
    cut = max(0, j - args.margin)
    keep = np.ones(n, dtype=bool)
    if cut > 1:
        da = np.abs(np.diff(a[:cut], axis=0)).max(axis=1)   # da[t] = delta t -> t+1
        idle = np.concatenate([[False], da < args.idle_eps])  # frame t repeats t-1
        keep[:cut] = ~idle   # drop repeats; the frame that STARTED the pause survives
    ss, aa = s[keep], a[keep]
    np.savez_compressed(DST / pl.Path(p).name, states=ss, actions=aa,
                        uid=d['uid'], n=len(ss), label=d['label'], stage=d['stage'])
    tot_in += n; tot_out += len(ss)
    print(f"{d['uid']}: {n} -> {len(ss)} (j_pick={j}, cut={cut})", flush=True)
print(f'\nTOTAL {tot_in} -> {tot_out} frames ({1 - tot_out/tot_in:.1%} pruned)')
