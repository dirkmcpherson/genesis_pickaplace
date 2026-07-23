"""Pick+place isolation from the CURRENT-recovered rest position (branch experiment).

Spawns the can at the gripper-current first-contact rest_xy (grasp_current.json), goal
parked far (slide irrelevant), and measures pick+place. Compares against the FK-based
number to see if the better spawn point (esp. for drag demos) raises pick+place.

Usage: pickplace_from_current.py [reps]
"""
import os
import sys, json, pathlib as pl
import numpy as np
REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', '/home/j/workspace/genesis_pickaplace'))
sys.path.insert(0, str(REPO / 'can_pos_recovery'))
from replay_harness import build_world, load_episode, rollout

REPS = int(sys.argv[1]) if len(sys.argv) > 1 else 3
tbl = json.loads((REPO/'can_pos_recovery/trial_placements.json').read_text()); wc = tbl['world']
fk = {int(k): v for k, v in json.loads((REPO/'can_pos_recovery/fk_recovered.json').read_text()).items()}
gc = {int(k): v for k, v in json.loads((REPO/'can_pos_recovery/grasp_current.json').read_text()).items()}

w = build_world(backend='cpu', finger_force=wc['finger_force'], finger_kp=wc['finger_kp'],
                can_height=wc['can_height'], can_rho=wc['can_rho'], substeps=wc['substeps'],
                table=wc['table'], can_radius=wc['can_radius'])
CAN_Z = w['can_start_z']
uids = sorted(u for u in gc if fk.get(u, {}).get('label') == 'success'
              and fk[u]['conf'] in ('HIGH', 'MED') and str(u) not in ('290', '322'))

agg = {'picked': 0, 'placed': 0, 'n': 0}; pick_cov = place_cov = 0
print(f"PICK+PLACE from CURRENT-recovered rest_xy, goal parked far, {REPS} reps")
print(f"{'uid':>4} {'rest_xy':>15} {'drag':>5} {'FKΔ':>5} marks  pick place")
for uid in uids:
    vel, gp = load_episode(uid)
    cx, cy = gc[uid]['rest_xy']
    fkd = np.hypot(cx - fk[uid]['can_xy'][0], cy - fk[uid]['can_xy'][1]) * 100
    pk = pl_ = 0; marks = []
    for _ in range(REPS):
        r = rollout(w, vel, gp, (cx, cy, CAN_Z), goal_pos=None)
        agg['picked'] += r['picked']; agg['placed'] += r['placed']; agg['n'] += 1
        pk += r['picked']; pl_ += r['placed']
        marks.append('P' if r['placed'] else 'p' if r['picked'] else '.')
    pick_cov += (pk > 0); place_cov += (pl_ > 0)
    print(f"{uid:>4} ({cx:+.3f},{cy:+.3f}) {gc[uid]['drag_cm']:>5.1f} {fkd:>4.1f} "
          f"{' '.join(marks)}  {pk/REPS:.2f} {pl_/REPS:.2f}", flush=True)
n = agg['n']
print(f"\n=== PICK+PLACE from current-recovered rest ({len(uids)} trials) ===")
print(f"per-run: picked {agg['picked']/n:.2f} placed {agg['placed']/n:.2f}")
print(f"coverage (>=1/{REPS}): picked {pick_cov}/{len(uids)} placed {place_cov}/{len(uids)}")
print("compare FK-based pick+place (frozen_fk / pickplace_eval) once both are in")
