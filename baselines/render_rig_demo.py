"""Render a demo episode as a side-by-side rig video: top cam | wrist cam.

Replays the recorded tape through the collector-faithful path with the dv3 camera
rig live, writing an mp4 at display resolution (the DATASET stays 64x64x6 -- this
is for human verification of framing only).

Usage: render_rig_demo.py <uid> [--out x.mp4] [--res 256] [--stride 3]
"""
import argparse, pathlib as pl, sys
import numpy as np, cv2

REPO = pl.Path('/home/j/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO / 'baselines')); sys.path.insert(0, str(REPO / 'can_pos_recovery'))

ap = argparse.ArgumentParser()
ap.add_argument('uid', type=int)
ap.add_argument('--out', default=None)
ap.add_argument('--res', type=int, default=256)
ap.add_argument('--stride', type=int, default=3, help='render every Nth command')
args = ap.parse_args()

from genesis_can_env import GenesisCanEnv  # noqa: E402
from replay_harness import load_episode  # noqa: E402

env = GenesisCanEnv(backend='cpu', camera_rig=True)
env.max_steps = 10 ** 9
env.w['cam_top'].set_params(fov=68)
vel, gp = load_episode(args.uid)
env.reset(uid=args.uid)

R = args.res
out = args.out or str(REPO / f'baselines/camera_rig_samples/rig_demo_{args.uid}.mp4')
vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), 20, (2 * R, R + 24))
stage = 'no-pick'
for i in range(len(vel) - 1):
    _, _, info = env.step(np.concatenate([vel[i], [np.clip(gp[i] / 100.0, 0, 1)]]).astype(np.float32),
                          grip_motor=gp[i], arm_cmd=vel[i])
    if info.get('contact'): stage = 'contact'
    elif info.get('placed'): stage = 'placed'
    elif info.get('picked'): stage = 'picked'
    if i % args.stride:
        continue
    w = env.w
    w['cam_wrist'].move_to_attach()
    top = cv2.resize(np.asarray(w['cam_top'].render()[0], dtype=np.uint8), (R, R))
    wrist = cv2.resize(np.asarray(w['cam_wrist'].render()[0], dtype=np.uint8), (R, R))
    frame = np.zeros((R + 24, 2 * R, 3), dtype=np.uint8)
    frame[24:, :R] = top[:, :, ::-1]
    frame[24:, R:] = wrist[:, :, ::-1]
    for x, lab in ((8, f'TOP overhead'), (R + 8, 'WRIST (above, -90+180)')):
        cv2.putText(frame, lab, (x, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(frame, f'uid {args.uid}  cmd {i}  {stage}', (2 * R - 210, 17),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    vw.write(frame)
vw.release()
print(f'{out}  (final stage {stage})')
