"""OVERNIGHT: render the BEST-EFFORT replay of EVERY episode + log closeness diagnostics.

Mirrors render_videos.py's validated replay EXACTLY (position-target arm cmds, gripper via
gripper_targets(gp[i]) with invert=True default, 3 physics steps/cmd, static goal unless a
trustworthy per-trial goal was recovered) but:
  * renders ALL trials fresh (no skip) into can_pos_recovery/videos_gallery/,
  * logs a rich per-trial metrics row to videos_gallery/metrics.json so we can hunt for a
    SYSTEMATIC failure pattern -- the point of eyeballing all of them at once. Key columns:
    can_goal_dist_xy (how far the placed can ended from the goal it was aimed at),
    final_can_z / max_z (did it lift / stay up), final_tilt (fell over?), pick_step,
    success_step.

Honesty: goal is relocated ONLY for solved, success-labeled, NOT-goal_moved trials (the
same quarantine example.py applies); everything else uses the static goal, so a can that
lands far from a static goal is shown honestly rather than having the goal moved onto it.

Usage: render_all_episodes.py [uid ...]        (default: all non-stub trials in the table)
"""
import sys, json, pathlib as pl
import numpy as np
import cv2
from replay_harness import (build_world, load_episode, gripper_targets, tilt_deg,
                            HARDCODED_START, STATIC_BOTTLE_POSITION, REPO, NESTED_TOUCH_DIST)
import torch

def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)

OUTDIR = REPO / 'can_pos_recovery/videos_gallery'
OUTDIR.mkdir(exist_ok=True)
FRAME_EVERY = 2
STUBS = {'290', '322'}   # gripper never closes -- not real demos
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

uids = [int(u) for u in sys.argv[1:]] or sorted(
    int(u) for u in table['trials'] if u not in STUBS)
metrics = []
for uid in uids:
    r = table['trials'][str(uid)]
    fk = fk_all.get(uid, {})
    label = r.get('label', '?')
    solved = r['status'] in ('ok', 'ok_batch')
    can_quat = tuple(r.get('can_quat') or (1, 0, 0, 0)) if solved else (1, 0, 0, 0)
    # #24: use the ONE corrected static goal (0.656,-0.103, from real tool_pose place-cluster)
    # for all demos -- honest single goal, matches the remeasure. (Old per-trial recovered-goal
    # relocation is disabled.)
    use_recovered_goal = False
    if solved:
        can_pos = tuple(r['can_pos'])
        goal_pos = tuple(r['goal_pos']) if use_recovered_goal else \
            (STATIC_BOTTLE_POSITION[0], STATIC_BOTTLE_POSITION[1], w['goal_start_z'])
    else:
        seed = fk.get('close_xy') or (fk.get('can_xy') if fk.get('conf') in ('HIGH', 'MED')
                                      else BUCKET[fk.get('pos')])
        can_pos = (seed[0], seed[1], w['can_start_z'])
        goal_pos = (STATIC_BOTTLE_POSITION[0], STATIC_BOTTLE_POSITION[1], w['goal_start_z'])

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
    max_z = 0.0; pick_step = success_step = -1
    for i in range(len(vel)):
        kinova.control_dofs_position(vel[i], dofs_idx_local=kdofs[:6])
        kinova.control_dofs_position(np.array(gripper_targets(gp[i])),
                                     dofs_idx_local=np.array(kdofs[-4:]))
        for _ in range(3):
            scene.step()
        bp = np_(bottle.get_pos())
        max_z = max(max_z, float(bp[2]))
        if not picked and bp[2] > w['pick_z'] and gp[i] > 30:
            picked = True; pick_step = i
        if i % 30 == 0 and not success:
            c = np_(bottle.get_contacts(goal)['position'])
            if (c.size and c.shape[0]) and float(np_(eef.get_pos())[0]) < float(bp[0]):
                success = True; success_step = i
        if i % FRAME_EVERY == 0:
            frames.append(np.asarray(cam.render()[0])[:, :, ::-1])
    for _ in range(100):
        scene.step()
    frames.append(np.asarray(cam.render()[0])[:, :, ::-1])

    bp = np_(bottle.get_pos()); bq = np_(bottle.get_quat()); gpos = np_(goal.get_pos())
    touch = float(np.hypot(bp[0] - gpos[0], bp[1] - gpos[1])) <= NESTED_TOUCH_DIST
    tilt = float(tilt_deg(bq))
    # picked precondition (panel fix) + proximity touch (see NESTED_TOUCH_DIST)
    nested = bool(picked and touch and tilt < 20 and tilt_deg(np_(goal.get_quat())) < 20)
    placed = max_z > w['pick_z'] and float(bp[2]) > 0.11
    outcome = ('S-nested' if success and nested else 'S' if success else
               'placed-only' if placed else 'picked-drop' if picked else 'no-pick')
    can_goal_dist = float(np.hypot(bp[0] - gpos[0], bp[1] - gpos[1]))
    row = dict(uid=uid, label=label, solved=solved, outcome=outcome,
               picked=picked, success=success, placed=bool(placed), nested=nested,
               pick_step=pick_step, success_step=success_step, n_cmds=len(vel),
               max_z=round(max_z, 4), final_can_z=round(float(bp[2]), 4),
               final_tilt_deg=round(tilt, 1),
               can_final_xy=[round(float(bp[0]), 4), round(float(bp[1]), 4)],
               goal_xy=[round(float(gpos[0]), 4), round(float(gpos[1]), 4)],
               can_goal_dist_xy=round(can_goal_dist, 4),
               used_recovered_goal=use_recovered_goal)
    metrics.append(row)
    out = OUTDIR / f'{uid}_{label[:4]}_{outcome}.mp4'
    vw = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*'mp4v'), 15,
                         (frames[0].shape[1], frames[0].shape[0]))
    for f in frames:
        vw.write(f.astype(np.uint8))
    vw.release()
    (OUTDIR / 'metrics.json').write_text(json.dumps(metrics, indent=2))  # checkpoint each trial
    print(f'{uid}({label[:4]}): {outcome} pick@{pick_step} maxz={max_z:.3f} '
          f'tilt={tilt:.0f} can-goal={can_goal_dist:.3f} -> {out.name}', flush=True)

# summary over success-labeled trials (the ones a human completed -> we should too)
succ = [m for m in metrics if m['label'] == 'success']
from collections import Counter
oc = Counter(m['outcome'] for m in succ)
print(f'\n=== SUCCESS-LABELED ({len(succ)}) outcomes: {dict(oc)} ===')
print('done ->', OUTDIR)
