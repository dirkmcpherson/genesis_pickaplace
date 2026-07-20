"""Human-in-the-loop goal fix (user: 233's pick/place/slide are PERFECT, so the goal must
sit where its final state counts as NESTED).

Step 1: replay --uid with the goal PARKED FAR (no interference), record the sim can
        trajectory; slide detector -> final rest pos + slide direction.
Step 2: implied goal = rest + DIAM * dir (cans touch, approach side).
Step 3: re-run the replay with the goal AT the implied spot; settle 100; report the honest
        nested predicate (picked precondition + both upright + contact).

Usage: goal_nested_fit.py --uid 233 [--goal X Y   # skip step 2, verify this goal instead]
"""
import argparse, json, pathlib as pl
import numpy as np, torch
from replay_harness import (build_world, load_episode, gripper_targets, tilt_deg,
                            HARDCODED_START, REPO, NESTED_TOUCH_DIST)
from goal_from_slides import slide_contact, DIAM

def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)

ap = argparse.ArgumentParser()
ap.add_argument('--uid', type=int, required=True)
ap.add_argument('--goal', type=float, nargs=2, default=None)
args = ap.parse_args()

BUCKET = {0: (0.4381, 0.1), 1: (0.4381, -0.05), 2: (0.4381, -0.2), None: (0.4381, -0.05)}
tbl = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())
wcfg = tbl['world']; trials = tbl['trials']
fk = {int(k): v for k, v in json.loads((REPO / 'can_pos_recovery/fk_recovered.json').read_text()).items()}
w = build_world(backend='cpu', finger_force=wcfg['finger_force'], finger_kp=wcfg['finger_kp'],
                can_height=wcfg['can_height'], can_rho=wcfg['can_rho'],
                substeps=wcfg.get('substeps', 1), table=True, can_radius=wcfg.get('can_radius', 0.035))
scene, kin, bottle, goal, kdofs, eef = (w['scene'], w['kinova'], w['bottle'], w['goal'], w['kdofs'], w['eef'])

r = trials[str(args.uid)]
solved = r['status'] in ('ok', 'ok_batch')
can_quat = list(r.get('can_quat') or [1, 0, 0, 0]) if solved else [1, 0, 0, 0]
if solved:
    can_pos = tuple(r['can_pos'])
else:
    f = fk.get(args.uid, {})
    seed = f.get('close_xy') or (f.get('can_xy') if f.get('conf') in ('HIGH', 'MED') else BUCKET[f.get('pos')])
    can_pos = (seed[0], seed[1], w['can_start_z'])
vel, gp = load_episode(args.uid)

def replay(goal_pos):
    kin.set_dofs_position(np.array(HARDCODED_START), kdofs); kin.zero_all_dofs_velocity()
    bottle.set_pos(can_pos); bottle.set_quat(can_quat)
    goal.set_pos(goal_pos); goal.set_quat([1, 0, 0, 0])
    for e in (bottle, goal):
        try: e.zero_all_dofs_velocity()
        except Exception: pass
    scene.step()
    picked = False
    traj = np.zeros((len(vel), 2))
    for i in range(len(vel)):
        kin.control_dofs_position(vel[i], dofs_idx_local=kdofs[:6])
        kin.control_dofs_position(np.array(gripper_targets(gp[i])), dofs_idx_local=np.array(kdofs[-4:]))
        for _ in range(3): scene.step()
        bp = np_(bottle.get_pos()); traj[i] = bp[:2]
        if not picked and bp[2] > w['pick_z'] and gp[i] > 30: picked = True
    for _ in range(100): scene.step()
    bp = np_(bottle.get_pos()); bq = np_(bottle.get_quat()); gq = np_(goal.get_pos())
    dist = float(np.hypot(bp[0] - gq[0], bp[1] - gq[1]))
    touch = dist <= NESTED_TOUCH_DIST
    nested = bool(picked and touch and tilt_deg(bq) < 20 and tilt_deg(np_(goal.get_quat())) < 20)
    return traj, bp, float(tilt_deg(bq)), picked, dist, nested

if args.goal is None:
    # Step 1: goal parked far -> natural rest
    traj, bp, tilt, picked, ncon, _ = replay((3.0, 3.0, 0.2))
    res = slide_contact(traj, np.asarray(gp).ravel())
    if res is None:
        print(f"{args.uid}: no clean slide detected; rest=({bp[0]:.3f},{bp[1]:.3f}) tilt={tilt:.0f}")
        raise SystemExit(1)
    contact, u, q = res
    rest = bp[:2]
    implied = rest + DIAM * u
    print(f"{args.uid}: rest=({rest[0]:.4f},{rest[1]:.4f}) tilt={tilt:.0f} picked={picked} "
          f"slide_dir=({u[0]:+.2f},{u[1]:+.2f}) straight={q['straight']}")
    print(f"IMPLIED GOAL = ({implied[0]:.4f},{implied[1]:.4f})   (rest + {DIAM}*dir)")
    gx, gy = float(implied[0]), float(implied[1])
else:
    gx, gy = args.goal

# Step 3: verify nested at (gx,gy)
traj, bp, tilt, picked, dist, nested = replay((gx, gy, w['goal_start_z']))
print(f"\nVERIFY goal=({gx:.4f},{gy:.4f}): final can=({bp[0]:.3f},{bp[1]:.3f}) tilt={tilt:.0f} "
      f"picked={picked} center-dist={dist:.4f} (touch<={NESTED_TOUCH_DIST}) -> NESTED={nested}")
