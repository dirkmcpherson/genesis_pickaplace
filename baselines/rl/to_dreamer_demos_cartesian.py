"""Convert episodes_cartesian(+images) -> DreamerV3 demo episodes (CARTESIAN actions).

Same Dreamer conventions as to_dreamer_demos.py (shift: action[t]/reward[t] lead INTO
obs[t]; index 0 is the reset frame with zero action; extend-by-one keeps the final
reward), but for the 4-DOF cartesian modality:

  actions : the HUMAN's commanded ee velocity [vx,vy,vz,wy,grip01] from the bags,
            normalized to [-1,1]^5 by CartesianCanEnv (VCAP 0.11, PITCH_CAP 1.0,
            grip 0..1 -> [-1,1]). These are the demonstrator's true actions, not
            derived joint targets.
  states  : 18-dim ee-centric (unused by pixel dreamer, kept for provenance)
  rewards : staged sparse, SAME grants as the joint pipeline: per-frame proxies
            (can_z/grip thresholds on the CARTESIAN state layout) gated by the
            episode's env-measured stage from demo_manifest_auth.json.
            can_z = state[11]; grip cmd = action[4] (0..1).

Usage: to_dreamer_demos_cartesian.py [--src baselines/episodes_cartesian]
         [--dst ~/workspace/dreamerv3-torch/demonstrations/genesis_cartesian]
"""
import os
import argparse, glob, json, pathlib as pl, sys
import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', '/home/j/workspace/genesis_pickaplace'))
sys.path.insert(0, str(REPO / 'baselines'))

from cartesian_env import CartesianCanEnv  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--src', default='baselines/episodes_cartesian')
ap.add_argument('--dst', default=os.path.expanduser(
    '~/workspace/dreamerv3-torch/demonstrations/genesis_cartesian'))
ap.add_argument('--pick-only', action='store_true')
ap.add_argument('--control', choices=['vel', 'delta'], default='delta',
                help='action normalization: delta (DCAP per-step deltas) | vel (VCAP)')
args = ap.parse_args()

PICK_Z = 0.1505
SHELF_LO, SHELF_HI = 0.16, 0.30       # same shelf band as train_sacfd_full
TOUCH_XY = 0.081                       # proximity contact (goal-final metric)
GRIP_CLOSED = 0.5
STAGE_RANK = {'no-pick': 0, 'picked': 1, 'placed': 2, 'contact': 3, 'nested': 4}
STAGE_REWARD = {'picked': 1.0, 'placed': 1.0, 'contact': 2.0, 'nested': 4.0}

man = json.loads((REPO / 'baselines/demo_manifest_auth.json').read_text())
DST = pl.Path(args.dst); DST.mkdir(parents=True, exist_ok=True)
for _stale in DST.glob('genesis-*.npz'):
    _stale.unlink()

paths = sorted(glob.glob(str(REPO / args.src / '*.npz')))
assert paths, f'no npz in {args.src}'

n_out = n_rew = 0
grants = {k: 0 for k in STAGE_REWARD}
for p in paths:
    d = np.load(p)
    uid = int(d['uid'])
    stage = man.get(str(uid), {}).get('stage', 'no-pick')
    rank = STAGE_RANK.get(stage, 0)
    if args.pick_only and rank < 1:
        continue
    if 'images' not in d.files:
        print(f'{uid}: SKIP (no images -- recollect with --images)', flush=True)
        continue
    s = d['states'].astype(np.float32)
    a_raw = d['actions'].astype(np.float32)          # [vx,vy,vz,wy,grip01] physical
    if len(s) == len(a_raw) + 1:                     # transition-complete npz
        s = s[:-1]
    # TIP TERMINATION (mirrors CartesianFullTaskEnv): truncate at the first frame the
    # can lies tipped FREE (tilt>60 AND grip open) with -0.5 and is_terminal. The
    # grip-open guard preserves demos that carry the can pitched in-hand (4/5
    # contact demos reach goal contact that way); census: orientation never returns
    # below 30 once past 60, so tipped-free is a true dead end. Bogus lying spawns
    # (234/318) truncate to nothing and are skipped.
    q = s[:, 12:16]
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))
    tipped_free = (tilt > 60.0) & (a_raw[:, 4] < 0.3)
    j_tip = int(np.argmax(tipped_free)) if tipped_free.any() else -1
    if j_tip >= 0:
        s = s[:j_tip + 1]
        a_raw = a_raw[:j_tip + 1]
    n = len(s) - 1
    if n < 2:
        print(f'{uid}: SKIP (tipped at frame {j_tip} -- nothing usable)', flush=True)
        continue
    can_z = s[:, 11]
    grip = a_raw[:, 4]
    rew = np.zeros(n, dtype=np.float32)
    done = np.zeros(n, dtype=bool)
    picked_f = (can_z[:-1] > PICK_Z) & (grip[:-1] > GRIP_CLOSED)
    j_pick = int(np.argmax(picked_f)) if picked_f.any() and rank >= 1 else -1
    if j_pick == 0:
        continue
    if j_pick > 0:
        rew[j_pick] += STAGE_REWARD['picked']; grants['picked'] += 1
        if rank >= 2:
            pl_f = (np.arange(n) > j_pick) & (can_z[:-1] > SHELF_LO) & (can_z[:-1] < SHELF_HI)
            j_pl = int(np.argmax(pl_f)) if pl_f.any() else -1
            if j_pl > 0:
                rew[j_pl] += STAGE_REWARD['placed']; grants['placed'] += 1
                if rank >= 3:
                    dxy = np.hypot(s[:-1, 9] - s[:-1, 16], s[:-1, 10] - s[:-1, 17])
                    c_f = (np.arange(n) > j_pl) & (dxy < TOUCH_XY)
                    j_c = int(np.argmax(c_f)) if c_f.any() else -1
                    if j_c > 0:
                        rew[j_c] += STAGE_REWARD['contact']; grants['contact'] += 1
                        if rank >= 4:
                            rew[n - 1] += STAGE_REWARD['nested']; grants['nested'] += 1
                            done[n - 1] = True
    if j_tip >= 0 and not done[n - 1]:
        rew[n - 1] += -0.5                     # tip penalty at the truncated frame
        done[n - 1] = True                     # is_terminal: env terminates here too
        grants.setdefault('tipped', 0); grants['tipped'] += 1
    _norm = (CartesianCanEnv.normalize_delta if args.control == 'delta'
             else CartesianCanEnv.normalize_action)
    act = _norm(a_raw[:n]).astype(np.float32)
    # --- dreamer shift + extend-by-one (see to_dreamer_demos.py rationale) ---------
    m = min(n + 1, len(d['images']))
    img = d['images'][:m].astype(np.uint8)
    act = np.concatenate([np.zeros_like(act[:1]), act])[:m]
    rew = np.concatenate([np.zeros_like(rew[:1]), rew])[:m]
    is_terminal = np.concatenate([[False], done])[:m]
    is_first = np.zeros(m, dtype=bool); is_first[0] = True
    is_last = np.zeros(m, dtype=bool); is_last[-1] = True
    np.savez_compressed(
        DST / f'genesis-{uid:04d}-{m}.npz',
        image=img, action=act, reward=rew,
        discount=(1.0 - is_terminal.astype(np.float32)),
        is_first=is_first, is_last=is_last, is_terminal=is_terminal,
        logprob=np.zeros(m, dtype=np.float32))
    n_out += 1; n_rew += int((rew > 0).sum())
    print(f'{uid}: {stage:<8} T={m:<5} reward_frames={int((rew > 0).sum())}', flush=True)
print(f'\n{n_out} episodes -> {DST}  ({n_rew} rewarded frames; grants {grants})')
