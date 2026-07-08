"""Isolate PICK + PLACE from the slide (panel/user question): is the pick-and-place
recoverable at high rate from the FK can position alone, independent of the goal?

Goal can is parked FAR AWAY, so slide/contact is irrelevant. For each success-labeled
trial, replay at the frozen FK position and measure: picked (lifted >5cm, gripper closed)
and placed (came to rest on the shelf footprint). No search, no goal relocation.

Usage: pickplace_eval.py [reps]
"""
import sys, json, pathlib as pl
import numpy as np
REPO = pl.Path('/home/james/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO / 'can_pos_recovery'))
from replay_harness import build_world, load_episode, rollout

REPS = int(sys.argv[1]) if len(sys.argv) > 1 else 3
tbl = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text()); wc = tbl['world']
fk = {int(k): v for k, v in
      json.loads((REPO / 'can_pos_recovery/fk_recovered.json').read_text()).items()}
w = build_world(backend='cpu', finger_force=wc['finger_force'], finger_kp=wc['finger_kp'],
                can_height=wc['can_height'], can_rho=wc['can_rho'], substeps=wc['substeps'],
                table=wc['table'], can_radius=wc['can_radius'])
CAN_Z = w['can_start_z']
uids = sorted(u for u, v in fk.items()
              if v['label'] == 'success' and v['conf'] in ('HIGH', 'MED')
              and str(u) not in ('290', '322'))

agg = {'picked': 0, 'placed': 0, 'n': 0}
pick_cov = place_cov = 0
print(f"PICK+PLACE isolation: frozen FK, goal parked FAR, {REPS} reps. slide/goal irrelevant.")
print(f"{'uid':>4} {'conf':>4} {'grasp_z':>7} marks(p=pick P=place)  pick-rate place-rate")
for uid in uids:
    vel, gp = load_episode(uid)
    cx, cy = fk[uid]['can_xy']
    pk = pl_ = 0; marks = []
    for _ in range(REPS):
        r = rollout(w, vel, gp, (cx, cy, CAN_Z), goal_pos=None)  # None -> goal parked far
        agg['picked'] += r['picked']; agg['placed'] += r['placed']; agg['n'] += 1
        pk += r['picked']; pl_ += r['placed']
        marks.append(('P' if r['placed'] else 'p' if r['picked'] else '.'))
    pick_cov += (pk > 0); place_cov += (pl_ > 0)
    print(f"{uid:>4} {fk[uid]['conf']:>4} {fk[uid]['grasp_mid_z']:>7.3f} {' '.join(marks)}"
          f"   {pk/REPS:.2f}  {pl_/REPS:.2f}", flush=True)
n = agg['n']
print(f"\n=== PICK+PLACE (frozen FK, goal irrelevant) ===")
print(f"trials {len(uids)} | per-run: picked {agg['picked']/n:.2f} placed {agg['placed']/n:.2f}")
print(f"coverage (>=1/{REPS}): picked {pick_cov}/{len(uids)} placed {place_cov}/{len(uids)}")
