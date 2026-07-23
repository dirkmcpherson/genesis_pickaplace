"""Human-in-the-loop goal tuning: render ONE demo replay with the goal can at a given
position, so a human can compare the teal goal can vs the real footage and say which way
to nudge it. The placed (orange) can is the shared landmark (known world position).

Usage: render_goal_tune.py --uid 232 --goal 0.704 -0.113 [--out <mp4>]
"""
import argparse, json, sys, pathlib as pl
import numpy as np, cv2, torch
from replay_harness import (build_world, load_episode, gripper_targets, tilt_deg,
                            HARDCODED_START, REPO)

def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)

ap = argparse.ArgumentParser()
ap.add_argument('--uid', type=int, required=True)
ap.add_argument('--goal', type=float, nargs=2, required=True)
ap.add_argument('--out', default=None)
args = ap.parse_args()
GX, GY = args.goal
BUCKET = {0: (0.4381, 0.1), 1: (0.4381, -0.05), 2: (0.4381, -0.2), None: (0.4381, -0.05)}

tbl = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())
wcfg = tbl['world']; trials = tbl['trials']
fk = {int(k): v for k, v in json.loads((REPO / 'can_pos_recovery/fk_recovered.json').read_text()).items()}
w = build_world(backend='cpu', finger_force=wcfg['finger_force'], finger_kp=wcfg['finger_kp'],
                can_height=wcfg['can_height'], can_rho=wcfg['can_rho'],
                substeps=wcfg.get('substeps', 1), table=True, can_radius=wcfg.get('can_radius', 0.035),
                camera=True)
scene, kin, bottle, goal, kdofs, eef, cam = (w['scene'], w['kinova'], w['bottle'], w['goal'],
                                             w['kdofs'], w['eef'], w['cam'])
uid = args.uid; r = trials[str(uid)]
solved = r['status'] in ('ok', 'ok_batch')
can_quat = list(r.get('can_quat') or [1, 0, 0, 0]) if solved else [1, 0, 0, 0]
if solved:
    can_pos = tuple(r['can_pos'])
else:
    f = fk.get(uid, {}); seed = f.get('close_xy') or (f.get('can_xy') if f.get('conf') in ('HIGH', 'MED') else BUCKET[f.get('pos')])
    can_pos = (seed[0], seed[1], w['can_start_z'])
vel, gp = load_episode(uid)
kin.set_dofs_position(np.array(HARDCODED_START), kdofs); kin.zero_all_dofs_velocity()
bottle.set_pos(can_pos); bottle.set_quat(can_quat)
goal.set_pos((GX, GY, w['goal_start_z'])); goal.set_quat([1, 0, 0, 0])
for e in (bottle, goal):
    try: e.zero_all_dofs_velocity()
    except Exception: pass
scene.step()
frames = []
for i in range(len(vel)):
    kin.control_dofs_position(vel[i], dofs_idx_local=kdofs[:6])
    kin.control_dofs_position(np.array(gripper_targets(gp[i])), dofs_idx_local=np.array(kdofs[-4:]))
    for _ in range(3): scene.step()
    if i % 2 == 0: frames.append(np.asarray(cam.render()[0])[:, :, ::-1])
for _ in range(60): scene.step()
frames.append(np.asarray(cam.render()[0])[:, :, ::-1])
out = args.out or f'/home/j/workspace/genesis_pickaplace/can_pos_recovery/_scratch/goaltune_{uid}_g{GX:.3f}_{GY:.3f}.mp4'
vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), 15, (frames[0].shape[1], frames[0].shape[0]))
for fr in frames: vw.write(fr.astype(np.uint8))
vw.release()
print(f"{uid} goal=({GX},{GY}): {len(frames)} frames -> {out}", flush=True)
