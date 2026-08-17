"""PLACE-PHASE REPLAY GATE + inspection videos.

Replays each carved place segment (baselines/episodes_place_phase) from its
BANKED POST-PICK entry state inside FullTaskEnv(scope='place'), scores the
env's own placed_v2 terminal, and writes one mp4 per demo for manual review.

WHY A CLOSED-LOOP FOLLOWER AND NOT TAPE REPLAY (P1, paper/p1_delta_divergence):
a tape in a self-referenced (measured-ref) action space reproduces DRIVE, not
POSITION -- once the arm is a few mm off the demonstrated path nothing in the
tape pulls it back. So we follow the recorded ABSOLUTE joint commands
closed-loop, exactly as rerecord_delta_demos.py does:

    a_arm = clip((cmd_j - q_measured) / leash, -1, 1)
    advance j when |q_meas - ref_j|_inf < TOL, or after MAX_DWELL steps

On-track this is bit-faithful to the demo's own command. Off-track it corrects.

The gate is scored with the ENV's placed_v2 (info['placed_v2'] / the terminal),
never a re-implemented predicate -- same import-not-reimplement rule that closed
the grip-column bug family. Entry states come from the env's own scope='place'
reset (entry_bank=place_entry_states.json), so the gate measures the exact IC
distribution training will see.

P2 CAVEAT: episodes replayed sequentially in one process share solver residue.
Read the AGGREGATE, not single marginal demos. Shard with --shard-idx/--shard-n
(one env per process) and the manifest records the split.

Usage:
  gate_place_demos.py --uids 242 247 248 251 252     # smoke
  gate_place_demos.py --videos                       # all 37 + mp4s
  gate_place_demos.py --shard-idx 0 --shard-n 6 --videos
"""
import os
import argparse
import glob
import json
import pathlib as pl
import sys
import time

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))
sys.path.insert(0, str(REPO / 'can_pos_recovery'))

TOL = 0.025            # one delta cap: within a demonstrated frame of motion
MAX_DWELL = 8
DILATION_CAP = 3.0

ap = argparse.ArgumentParser()
ap.add_argument('--demo-dir', default='baselines/episodes_place_phase')
ap.add_argument('--bank', default='baselines/place_entry_states.json')
ap.add_argument('--uids', type=int, nargs='*', default=None)
ap.add_argument('--shard-idx', type=int, default=0)
ap.add_argument('--shard-n', type=int, default=1)
ap.add_argument('--videos', action='store_true')
ap.add_argument('--video-dir', default=None)
ap.add_argument('--render', type=int, nargs=2, default=(480, 640))
args = ap.parse_args()

VID = pl.Path(args.video_dir) if args.video_dir else (
    pl.Path('/tmp/claude-1000/-home-j-workspace-genesis-pickaplace/'
            '85fd3758-abf4-4299-a3e2-48a960c5be6c/scratchpad/place_videos'))

from full_env import FullTaskEnv  # noqa: E402

paths = sorted(glob.glob(str(REPO / args.demo_dir / '*.npz')),
               key=lambda p: int(pl.Path(p).stem))
if args.uids:
    want = set(args.uids)
    paths = [p for p in paths if int(pl.Path(p).stem) in want]
if args.shard_n > 1:
    paths = paths[args.shard_idx::args.shard_n]
assert paths, 'no place demos selected'

env = FullTaskEnv(backend='cpu', scope='place', action_mode='delta_joint',
                  delta_ref='measured', entry_bank=str(REPO / args.bank),
                  render_size=tuple(args.render) if args.videos else None,
                  max_steps=10 ** 6)
assert env.scope == 'place', env.scope
leash = float(env.delta_leash)
print(f'[gate] scope={env.scope} delta_ref=measured leash={leash} '
      f'bank={args.bank} n={len(paths)} shard={args.shard_idx}/{args.shard_n} '
      f'videos={args.videos}', flush=True)

if args.videos:
    VID.mkdir(parents=True, exist_ok=True)
    import cv2

recs = []
for p in paths:
    d = np.load(p, allow_pickle=True)
    uid = int(d['uid'])
    S = d['states'].astype(np.float64)
    A = d['actions'].astype(np.float64)          # ABSOLUTE joint targets + grip
    cmd, grip = A[:, :6], np.clip(A[:, 6], 0.0, 1.0)
    ref = np.concatenate([S[1:, :6], S[-1:, :6]], axis=0)
    n_src = len(S)
    t0 = time.time()
    try:
        obs, _ = env.reset(options={'uid': uid})
    except Exception as e:
        recs.append(dict(uid=uid, status=f'RESET_FAIL:{type(e).__name__}'))
        print(f'PLACE uid={uid} RESET_FAIL {e}', flush=True)
        continue

    frames = []
    step_cap = int(np.ceil(DILATION_CAP * n_src))
    j = dwell = stalls = steps = 0
    granted = tipped = False
    while j < n_src and steps < step_cap:
        q = np.asarray(obs[:6], dtype=np.float64)
        a = np.concatenate([np.clip((cmd[j] - q) / leash, -1.0, 1.0),
                            [grip[j] * 2.0 - 1.0]]).astype(np.float32)
        obs, r, term, trunc, info = env.step(a)
        steps += 1
        if args.videos and env.genv.render_size is not None:
            im = env.genv._obs().get('image')
            if im is not None:
                frames.append(np.asarray(im)[:, :, ::-1])   # RGB -> BGR for cv2
        if info.get('placed_v2'):
            granted = True
        if term:
            tipped = bool(info.get('tipped'))
            break
        qn = np.asarray(obs[:6], dtype=np.float64)
        if float(np.max(np.abs(qn - ref[j]))) < TOL:
            j += 1
            dwell = 0
        else:
            dwell += 1
            if dwell >= MAX_DWELL:
                j += 1
                dwell = 0
                stalls += 1

    rec = dict(uid=uid, status='OK', placed_v2=bool(granted),
               src_stage=str(d['stage']) if 'stage' in d.files else '?',
               n_src=n_src, steps=steps, dilation=round(steps / n_src, 2),
               stalls=stalls, tipped=bool(tipped),
               truncated=bool(j < n_src), wall_s=round(time.time() - t0, 1))
    recs.append(rec)
    print('PLACE ' + ' '.join(f'{k}={v}' for k, v in rec.items()), flush=True)

    if args.videos and frames:
        h, w = frames[0].shape[:2]
        tag = 'PLACED' if granted else ('TIP' if tipped else 'fail')
        out = VID / f'place_{uid}_{tag}.mp4'
        vw = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*'mp4v'), 30, (w, h))
        for f_ in frames:
            vw.write(f_)
        vw.release()

ok = [r for r in recs if r.get('status') == 'OK']
n_pv2 = sum(1 for r in ok if r['placed_v2'])
man = VID / f'_gate_manifest_shard{args.shard_idx}of{args.shard_n}.json'
VID.mkdir(parents=True, exist_ok=True)
man.write_text(json.dumps(dict(
    config=dict(demo_dir=args.demo_dir, bank=args.bank, tol=TOL,
                max_dwell=MAX_DWELL, dilation_cap=DILATION_CAP,
                leash=leash, shard_idx=args.shard_idx, shard_n=args.shard_n),
    records=recs), indent=1))
print(f'\n[gate] shard {args.shard_idx}/{args.shard_n}: placed_v2 {n_pv2}/{len(ok)}'
      f'  tipped {sum(1 for r in ok if r["tipped"])}'
      f'  truncated {sum(1 for r in ok if r["truncated"])} -> {man}', flush=True)
