"""Render the re-collected demo set with COLLECTOR-FAITHFUL execution: raw tape
(env.step(grip_motor=, arm_cmd=)), same ICs as collect_all_classified, camera on.

render_collected.py replays stored float32 actions through the clipped policy path --
post horizon-fix we know that does NOT reproduce the collected trajectory for
borderline demos. Here the video shows the same dynamical system the dataset was
recorded from; each video's outcome tag is re-measured, so if camera-render load ever
flips a borderline demo the filename exposes it instead of hiding it.

Usage: render_demos_raw.py <video_dir> [--uids U...]
"""
import os
import argparse, json, sys, pathlib as pl
import numpy as np, cv2, torch

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', '/home/j/workspace/genesis_pickaplace'))
sys.path.insert(0, str(REPO / 'baselines')); sys.path.insert(0, str(REPO / 'can_pos_recovery'))
from genesis_can_env import GenesisCanEnv
from replay_harness import (load_episode, tilt_deg, NESTED_TOUCH_DIST,
                            STATIC_BOTTLE_POSITION)

def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)

ap = argparse.ArgumentParser()
ap.add_argument('video_dir')
ap.add_argument('--uids', type=int, nargs='*', default=None)
args = ap.parse_args()
VID = REPO / args.video_dir; VID.mkdir(parents=True, exist_ok=True)

BUCKET = {0: (0.4381, 0.1), 1: (0.4381, -0.05), 2: (0.4381, -0.2), None: (0.4381, -0.05)}
STUB = {'290', '322'}
tbl = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())['trials']
fk = {int(k): v for k, v in
      json.loads((REPO / 'can_pos_recovery/fk_recovered.json').read_text()).items()}

env = GenesisCanEnv(backend='cpu', render_size=(480, 640))
env.max_steps = 10 ** 9        # NEVER fire the eval-horizon _nested mid-tape
CANZ = env.w['can_start_z']
GOAL = (STATIC_BOTTLE_POSITION[0], STATIC_BOTTLE_POSITION[1], env.w['goal_start_z'])

uids = args.uids or sorted(int(u) for u in tbl if u not in STUB)
for uid in uids:
    if list(VID.glob(f'{uid}_*.mp4')):
        print(f'{uid}: already rendered, skip', flush=True); continue
    try:
        r = tbl[str(uid)]
        label = r.get('label', '?')
        if r.get('status') in ('ok', 'ok_batch'):
            env.reset(uid=uid)
        else:
            f = fk.get(uid, {})
            seed = f.get('close_xy') or (f.get('can_xy') if f.get('conf') in ('HIGH', 'MED')
                                         else BUCKET[f.get('pos')])
            env.reset(can_pos=(seed[0], seed[1], CANZ), can_quat=[1, 0, 0, 0], goal_pos=GOAL)
        vel, gp = load_episode(uid)
        frames, picked, placed, contact = [], False, False, False
        for i in range(len(vel) - 1):
            a = np.concatenate([vel[i], [np.clip(gp[i] / 100.0, 0, 1)]]).astype(np.float32)
            obs, done, info = env.step(a, grip_motor=gp[i], arm_cmd=vel[i])
            picked |= bool(info['picked']); placed |= bool(info['placed'])
            contact |= bool(info['contact'])
            if i % 3 == 0:                      # 30Hz cmds -> 10fps capture, 20fps write = 2x
                frames.append(np.asarray(env.w['cam'].render()[0])[:, :, ::-1])
        hold = np.concatenate([vel[-1], [np.clip(gp[-1] / 100.0, 0, 1)]]).astype(np.float32)
        for _ in range(100):
            obs, done, info = env.step(hold, grip_motor=gp[-1], arm_cmd=vel[-1])
        frames.append(np.asarray(env.w['cam'].render()[0])[:, :, ::-1])
        w = env.w
        bp_, gp_ = np_(w['bottle'].get_pos()), np_(w['goal'].get_pos())
        touch = float(np.hypot(bp_[0] - gp_[0], bp_[1] - gp_[1])) <= NESTED_TOUCH_DIST
        nested = bool(picked and touch and tilt_deg(np_(w['bottle'].get_quat())) < 20
                      and tilt_deg(np_(w['goal'].get_quat())) < 20)
        stage = ('nested' if nested else 'contact' if contact else 'placed' if placed
                 else 'picked' if picked else 'no-pick')
        out = VID / f'{uid}_{label[:4]}_{stage}.mp4'
        vw = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*'mp4v'), 20,
                             (frames[0].shape[1], frames[0].shape[0]))
        for fr in frames:
            vw.write(fr.astype(np.uint8))
        vw.release()
        print(f'{uid}: {label}/{stage} ({len(frames)}f) -> {out.name}', flush=True)
    except Exception as e:
        print(f'{uid}: RENDER FAILED {type(e).__name__}: {e}', flush=True)
