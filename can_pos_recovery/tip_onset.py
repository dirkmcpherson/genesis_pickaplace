"""WHEN does the can start tipping -- at/just after the fingers take the can (a swing from
the grip contact) or much later during the carry (dynamics)? Distinguishes the two tip-over
hypotheses now that table height is ruled out.

Onset is measured relative to the PICK frame (can first lifts above pick_z while gripper
closed = fingers have the can), not the gripper COMMAND, which closes long before contact.
For each demo: pick_frame, tilt20_frame, Delta = tilt20 - pick (small => tips right as it's
grasped = swing; large => carry). Also peak angular speed in the 30 frames after pick, and
tilt at pick / pick+20 / pick+40.
"""
import json, pathlib as pl
import numpy as np, torch
from replay_harness import (build_world, load_episode, gripper_targets, tilt_deg,
                            HARDCODED_START, STATIC_BOTTLE_POSITION, REPO)

def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)

DEMOS = [232, 233, 235, 257, 261, 267]   # 232 stable; rest tip
BUCKET = {0: (0.4381, 0.1), 1: (0.4381, -0.05), 2: (0.4381, -0.2), None: (0.4381, -0.05)}
table = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())
wcfg = table['world']
fk_all = {int(k): v for k, v in
          json.loads((REPO / 'can_pos_recovery/fk_recovered.json').read_text()).items()}
w = build_world(backend='cpu', finger_force=wcfg['finger_force'], finger_kp=wcfg['finger_kp'],
                can_height=wcfg['can_height'], can_rho=wcfg['can_rho'],
                substeps=wcfg.get('substeps', 1), table=True,
                can_radius=wcfg.get('can_radius', 0.035))
scene, kinova, bottle, goal, kdofs, eef = (w['scene'], w['kinova'], w['bottle'],
                                           w['goal'], w['kdofs'], w['eef'])
CANZ, PICKZ = w['can_start_z'], w['pick_z']


def place(uid):
    r = table['trials'][str(uid)]
    solved = r['status'] in ('ok', 'ok_batch')
    cq = list(r.get('can_quat') or [1, 0, 0, 0]) if solved else [1, 0, 0, 0]
    if solved:
        cx, cy = r['can_pos'][0], r['can_pos'][1]
        gx, gy = (r['goal_pos'][0], r['goal_pos'][1]) if (r.get('label') == 'success'
                  and not r.get('goal_moved')) else STATIC_BOTTLE_POSITION[:2]
    else:
        fk = fk_all.get(uid, {})
        seed = fk.get('close_xy') or (fk.get('can_xy') if fk.get('conf') in ('HIGH', 'MED')
                                      else BUCKET[fk.get('pos')])
        cx, cy = seed[0], seed[1]
        gx, gy = STATIC_BOTTLE_POSITION[:2]
    return (cx, cy, CANZ), cq, (gx, gy, w['goal_start_z'])


def ang_speed():
    try: return float(np.linalg.norm(np_(bottle.get_ang())))
    except Exception: return float('nan')


for uid in DEMOS:
    can_pos, can_quat, goal_pos = place(uid)
    vel, gp = load_episode(uid)
    kinova.set_dofs_position(np.array(HARDCODED_START), kdofs); kinova.zero_all_dofs_velocity()
    bottle.set_pos(can_pos); bottle.set_quat(can_quat)
    goal.set_pos(goal_pos); goal.set_quat([1, 0, 0, 0])
    for e in (bottle, goal):
        try: e.zero_all_dofs_velocity()
        except Exception: pass
    scene.step()

    pick = tilt20 = -1
    peak_ang_after_pick = 0.0
    tilt_series = []
    for i in range(len(vel)):
        kinova.control_dofs_position(vel[i], dofs_idx_local=kdofs[:6])
        kinova.control_dofs_position(np.array(gripper_targets(gp[i])),
                                     dofs_idx_local=np.array(kdofs[-4:]))
        for _ in range(3):
            scene.step()
        bp = np_(bottle.get_pos()); tl = float(tilt_deg(np_(bottle.get_quat())))
        tilt_series.append(tl)
        if pick < 0 and bp[2] > PICKZ and gp[i] > 30:
            pick = i
        if pick >= 0 and 0 <= i - pick <= 30:
            peak_ang_after_pick = max(peak_ang_after_pick, ang_speed())
        if tilt20 < 0 and tl > 20:
            tilt20 = i
    if pick < 0:
        print(f"{uid}: NO-PICK (tilt20@{tilt20})", flush=True); continue
    delta = (tilt20 - pick) if tilt20 >= 0 else None
    tp = lambda k: tilt_series[min(pick + k, len(tilt_series) - 1)]
    print(f"{uid}: pick@{pick} tilt20@{tilt20} Δ(onset-pick)={delta} "
          f"| tilt[pick]={tp(0):.1f} [+20]={tp(20):.1f} [+40]={tp(40):.1f} "
          f"peak_ang_after_pick={peak_ang_after_pick:.2f} ncmds={len(vel)}", flush=True)
print('done')
