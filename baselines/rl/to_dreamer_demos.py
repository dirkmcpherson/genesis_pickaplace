"""Convert episodes_all_images/*.npz -> DreamerV3 demo episodes.

Dreamer's tools.load_episodes expects per-episode npz with aligned arrays:
    image (T,H,W,C) uint8, action (T,A) float32, reward (T,), is_first/is_last/
    is_terminal (T,) bool, discount (T,), logprob (T,)

Rewards use the SAME staged scheme as SACfD (full_env.STAGE_REWARD) via
train_sacfd_full.relabel_full, so a Dreamer-vs-SACfD comparison is reward-identical.
is_terminal is True only on the nested frame (true task termination); episodes that
never nest end with is_last=True / is_terminal=False so the value head bootstraps.

Usage: to_dreamer_demos.py [--src baselines/episodes_all_images]
                           [--dst ~/workspace/dreamerv3-torch/demonstrations/genesis]
"""
import argparse, glob, pathlib as pl, sys
import numpy as np

REPO = pl.Path('/home/j/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO / 'baselines')); sys.path.insert(0, str(REPO / 'baselines' / 'rl'))

ap = argparse.ArgumentParser()
ap.add_argument('--src', default='baselines/episodes_all_images')
ap.add_argument('--dst', default='/home/j/workspace/dreamerv3-torch/demonstrations/genesis')
ap.add_argument('--pick-only', action='store_true',
                help='keep only demos that reached >= picked (drop no-pick negatives)')
args = ap.parse_args()

import pick_env  # noqa: E402
from train_sacfd_full import relabel_full  # noqa: E402
from full_env import FullTaskEnv  # noqa: E402

PICK_Z = 0.1505   # FullTaskEnv.pick_z under the current world (avoids building a world)

DST = pl.Path(args.dst); DST.mkdir(parents=True, exist_ok=True)
# Filenames encode episode length (genesis-<uid>-<T>.npz), so any change to T orphans
# the previous generation instead of overwriting it -- the directory then holds BOTH
# and dreamer loads both (once silently inflated total demo reward 272 -> 476).
for _stale in DST.glob('genesis-*.npz'):
    _stale.unlink()
paths = sorted(glob.glob(str(REPO / args.src / '*.npz')))
assert paths, f'no npz in {args.src}'

n_out = n_rew = 0
for p in paths:
    d = np.load(p, allow_pickle=True)
    stage = str(d['stage'])
    if args.pick_only and stage == 'no-pick':
        continue
    # per-frame staged rewards, gated by the episode's env-measured stage
    trans, _ = relabel_full([p], PICK_Z)
    if not trans:
        continue
    T = len(trans)                       # transitions = frames - 1 (or up to nested)
    img = d['images'][:T].astype(np.uint8)
    # NORMALIZE to the env's Box(-1,1) action convention. The stored npz actions are
    # RAW (joint radians + grip 0..1); dreamer's actor emits [-1,1]. Feeding raw actions
    # made the world model learn dynamics conditioned on a scale the policy can never
    # produce (62.6% of demo action values fell outside [-1,1]) -- present in every run
    # v6-v13. SACfD never hit this because demo_buffer applies normalize_action.
    act = pick_env.normalize_action(
        np.stack([t[1] for t in trans]).astype(np.float32)).astype(np.float32)
    rew = np.array([t[2] for t in trans], dtype=np.float32)
    done = np.array([t[4] for t in trans], dtype=bool)
    is_first = np.zeros(T, dtype=bool); is_first[0] = True
    is_last = np.zeros(T, dtype=bool); is_last[-1] = True
    is_terminal = done.copy()            # True only at a nested frame
    discount = (1.0 - is_terminal.astype(np.float32))
    # --- SHIFT into dreamer's convention -------------------------------------
    # tools.simulate stores: index 0 = (reset obs, action=ZEROS, reward=0), and for
    # t>=1, action[t]/reward[t] are what led INTO obs[t] (backward-looking).
    # relabel_full gives forward-looking tuples (s_t, a_t, r_t) with a_t taken FROM
    # s_t. Feeding those unshifted put demo transitions one step out of phase with
    # collected ones -- the world model saw two contradictory dynamics.
    # EXTEND by one frame rather than truncate: the episode becomes length T+1 with
    # obs s_0..s_T, so the FINAL reward (the nested +4 -- our scarcest signal) is kept.
    n_avail = len(d['images'])
    img = d['images'][:min(T + 1, n_avail)].astype(np.uint8)
    m = len(img)                                    # T+1 normally
    act = np.concatenate([np.zeros_like(act[:1]), act])[:m]
    rew = np.concatenate([np.zeros_like(rew[:1]), rew])[:m]
    is_terminal = np.concatenate([[False], is_terminal])[:m]
    is_first = np.zeros(m, dtype=bool); is_first[0] = True
    is_last = np.zeros(m, dtype=bool); is_last[-1] = True
    discount = (1.0 - is_terminal.astype(np.float32))
    T = m
    uid = int(d['uid'])
    np.savez_compressed(
        DST / f'genesis-{uid:04d}-{T}.npz',
        image=img, action=act, reward=rew, discount=discount,
        is_first=is_first, is_last=is_last, is_terminal=is_terminal,
        logprob=np.zeros(T, dtype=np.float32))
    n_out += 1; n_rew += int((rew > 0).sum())
    print(f'{uid}: {stage:<8} T={T:<5} reward_frames={int((rew>0).sum())}', flush=True)
print(f'\n{n_out} demo episodes -> {DST}  ({n_rew} rewarded frames total)')
