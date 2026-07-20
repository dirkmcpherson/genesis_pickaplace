"""#24: re-measure contact with the CORRECTED static goal.

The goal can was static during data collection, but our sim had it at (0.6,-0.2) -- ~11cm
off the real place-cluster derived from tool_pose (median (0.656,-0.103)). This re-runs the
joint replays of all success demos with the goal at the corrected location and reports the
honest contact/nested counts vs the old static-goal numbers. No video (fast).

The goal is data-derived (real human placements), NOT fit to the sim -> honest, not manufacturing.

Usage: python can_pos_recovery/remeasure_contact.py [--goal X Y]
"""
import argparse, json, sys, pathlib as pl
import numpy as np, torch
from replay_harness import (build_world, load_episode, gripper_targets, tilt_deg,
                            HARDCODED_START, in_shelf_footprint, BOX_TOP_Z, REPO,
                            NESTED_TOUCH_DIST)

def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)

ap = argparse.ArgumentParser()
ap.add_argument('--goal', type=float, nargs=2, default=[0.672, -0.221])  # human-validated SOUTH
ap.add_argument('--label', default='success', help="'success' (default) or 'fail' (negative control)")
ap.add_argument('--finger-kp', type=float, default=None, help='override world finger_kp (compliance)')
ap.add_argument('--every', type=int, default=1, help='stride over sorted demos (fast subset, e.g. 3)')
args = ap.parse_args()
GX, GY = args.goal
BUCKET = {0: (0.4381, 0.1), 1: (0.4381, -0.05), 2: (0.4381, -0.2), None: (0.4381, -0.05)}

tbl = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())
wcfg = tbl['world']; trials = tbl['trials']
fk = {int(k): v for k, v in json.loads((REPO / 'can_pos_recovery/fk_recovered.json').read_text()).items()}
_fkp = args.finger_kp if args.finger_kp is not None else wcfg['finger_kp']
w = build_world(backend='cpu', finger_force=wcfg['finger_force'], finger_kp=_fkp,
                can_height=wcfg['can_height'], can_rho=wcfg['can_rho'],
                substeps=wcfg.get('substeps', 1), table=True, can_radius=wcfg.get('can_radius', 0.035))
scene, kin, bottle, goal, kdofs, eef = (w['scene'], w['kinova'], w['bottle'], w['goal'], w['kdofs'], w['eef'])
CANZ, GOALZ, PICKZ = w['can_start_z'], w['goal_start_z'], w['pick_z']

uids = sorted(int(u) for u, r in trials.items() if r.get('label') == args.label and u not in ('290', '322'))
uids = uids[::args.every]
print(f"re-measuring {len(uids)} success demos | corrected goal ({GX},{GY}) vs old static (0.6,-0.2)", flush=True)

agg = dict(picked=0, placed=0, contact=0, nested=0, n=0)
for uid in uids:
    r = trials[str(uid)]
    solved = r['status'] in ('ok', 'ok_batch')
    can_quat = list(r.get('can_quat') or [1, 0, 0, 0]) if solved else [1, 0, 0, 0]
    if solved:
        can_pos = tuple(r['can_pos'])
    else:
        f = fk.get(uid, {}); seed = f.get('close_xy') or (f.get('can_xy') if f.get('conf') in ('HIGH', 'MED') else BUCKET[f.get('pos')])
        can_pos = (seed[0], seed[1], CANZ)
    vel, gp = load_episode(uid)
    kin.set_dofs_position(np.array(HARDCODED_START), kdofs); kin.zero_all_dofs_velocity()
    bottle.set_pos(can_pos); bottle.set_quat(can_quat)
    goal.set_pos((GX, GY, GOALZ)); goal.set_quat([1, 0, 0, 0])
    for e in (bottle, goal):
        try: e.zero_all_dofs_velocity()
        except Exception: pass
    scene.step()
    picked = contact = False; maxz = 0.0
    for i in range(len(vel)):
        kin.control_dofs_position(vel[i], dofs_idx_local=kdofs[:6])
        kin.control_dofs_position(np.array(gripper_targets(gp[i])), dofs_idx_local=np.array(kdofs[-4:]))
        for _ in range(3): scene.step()
        bp = np_(bottle.get_pos()); maxz = max(maxz, float(bp[2]))
        if not picked and bp[2] > PICKZ and gp[i] > 30: picked = True
        if not contact and i % 30 == 0:
            c = np_(bottle.get_contacts(goal)['position'])
            if picked and (c.size and c.shape[0]) and float(np_(eef.get_pos())[0]) < float(bp[0]): contact = True
    for _ in range(100): scene.step()
    bp = np_(bottle.get_pos()); gpos = np_(goal.get_pos())
    touch = float(np.hypot(bp[0] - gpos[0], bp[1] - gpos[1])) <= NESTED_TOUCH_DIST
    nested = bool(picked and touch and tilt_deg(np_(bottle.get_quat())) < 20 and tilt_deg(np_(goal.get_quat())) < 20)
    placed = maxz > PICKZ and float(bp[2]) > 0.11
    for k, v in (('picked', picked), ('placed', placed), ('contact', contact), ('nested', nested)):
        agg[k] += bool(v)
    agg['n'] += 1
    print(f"{uid}: picked={picked} placed={placed} contact={contact} nested={nested}", flush=True)

n = agg['n']
print(f"\n=== CORRECTED-GOAL contact re-measure (goal {GX},{GY}) over {n} success demos ===")
print(f"picked {agg['picked']}/{n}  placed {agg['placed']}/{n}  contact {agg['contact']}/{n}  nested {agg['nested']}/{n}")
print(f"(old static-goal gallery kp40: contact 21/75)")
