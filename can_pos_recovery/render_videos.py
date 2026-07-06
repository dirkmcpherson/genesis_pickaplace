"""Render each trial in trial_placements.json to an mp4 (offscreen EGL camera).

Solved trials replay their winning placement; unsolved ones replay the best seed so the
failure mode is visible. Filenames encode the outcome, e.g. 243_S-nested.mp4,
232_placed-only.mp4. Resumable: skips files that already exist.

Usage: render_videos.py [uid ...]        (default: everything in the table)
Writes: can_pos_recovery/videos/<uid>_<outcome>.mp4
"""
import sys, json, pathlib as pl
import numpy as np
import cv2
from replay_harness import (build_world, load_episode, gripper_targets, tilt_deg,
                            HARDCODED_START, STATIC_BOTTLE_POSITION, REPO)
import torch

def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)

OUTDIR = REPO / 'can_pos_recovery/videos'
OUTDIR.mkdir(exist_ok=True)
FRAME_EVERY = 2          # one frame per 2 commands ~= 15 fps of demo time
BUCKET = {0: (0.4381, 0.1), 1: (0.4381, -0.05), 2: (0.4381, -0.2), None: (0.4381, -0.05)}

table = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())
wcfg = table['world']
fk_all = {int(k): v for k, v in
          json.loads((REPO / 'can_pos_recovery/fk_recovered.json').read_text()).items()}

w = build_world(backend='cpu', finger_force=wcfg['finger_force'], finger_kp=wcfg['finger_kp'],
                can_height=wcfg['can_height'], can_rho=wcfg['can_rho'],
                substeps=wcfg.get('substeps', 1), table=wcfg.get('table', False),
                can_radius=wcfg.get('can_radius', 0.035), camera=True)
scene, kinova, bottle, goal, kdofs, eef, cam = (w['scene'], w['kinova'], w['bottle'],
                                                w['goal'], w['kdofs'], w['eef'], w['cam'])

uids = [int(u) for u in sys.argv[1:]] or sorted(int(u) for u in table['trials'])
for uid in uids:
    r = table['trials'][str(uid)]
    fk = fk_all[uid]
    solved = r['status'] in ('ok', 'ok_batch')
    can_quat = tuple(r.get('can_quat') or (1, 0, 0, 0)) if solved else (1, 0, 0, 0)
    if solved:
        can_pos, goal_pos = tuple(r['can_pos']), tuple(r['goal_pos'])
    else:
        seed = fk.get('close_xy') or (fk['can_xy'] if fk['conf'] in ('HIGH', 'MED')
                                      else BUCKET[fk['pos']])
        can_pos = (seed[0], seed[1], w['can_start_z'])
        goal_pos = (STATIC_BOTTLE_POSITION[0], STATIC_BOTTLE_POSITION[1], w['goal_start_z'])
    existing = list(OUTDIR.glob(f'{uid}_*.mp4'))
    if existing:
        print(f'{uid}: exists ({existing[0].name}), skip'); continue

    vel, gp = load_episode(uid)
    kinova.set_dofs_position(np.array(HARDCODED_START), kdofs)
    kinova.zero_all_dofs_velocity()
    bottle.set_pos(can_pos); bottle.set_quat(list(can_quat))
    goal.set_pos(goal_pos); goal.set_quat([1, 0, 0, 0])
    for ent in (bottle, goal):
        try: ent.zero_all_dofs_velocity()
        except Exception: pass
    scene.step()

    frames = []
    picked = success = False
    max_z = 0.0
    for i in range(len(vel)):
        kinova.control_dofs_position(vel[i], dofs_idx_local=kdofs[:6])
        kinova.control_dofs_position(np.array(gripper_targets(gp[i])),
                                     dofs_idx_local=np.array(kdofs[-4:]))
        for _ in range(3):
            scene.step()
        bp = np_(bottle.get_pos())
        max_z = max(max_z, float(bp[2]))
        if bp[2] > w['pick_z'] and gp[i] > 30: picked = True
        if i % 30 == 0 and not success:
            c = np_(bottle.get_contacts(goal)['position'])
            if (c.size and c.shape[0]) and float(np_(eef.get_pos())[0]) < float(bp[0]):
                success = True
        if i % FRAME_EVERY == 0:
            rgb = cam.render()[0]
            frames.append(np.asarray(rgb)[:, :, ::-1])   # RGB -> BGR for cv2
    for _ in range(100):
        scene.step()
    rgb = cam.render()[0]; frames.append(np.asarray(rgb)[:, :, ::-1])
    # nested at end
    c = np_(bottle.get_contacts(goal)['position'])
    ncon = 0 if c.size == 0 else c.shape[0]
    nested = bool(ncon and tilt_deg(np_(bottle.get_quat())) < 20
                  and tilt_deg(np_(goal.get_quat())) < 20)
    outcome = ('S-nested' if success and nested else 'S' if success else
               'placed-only' if max_z > w['pick_z'] and np_(bottle.get_pos())[2] > 0.11 else
               'picked-drop' if picked else 'no-pick')
    out = OUTDIR / f'{uid}_{outcome}.mp4'
    vw = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*'mp4v'), 15,
                         (frames[0].shape[1], frames[0].shape[0]))
    for f in frames:
        vw.write(f.astype(np.uint8))
    vw.release()
    print(f'{uid}: {outcome} ({len(frames)} frames) -> {out.name}', flush=True)
print('done')
