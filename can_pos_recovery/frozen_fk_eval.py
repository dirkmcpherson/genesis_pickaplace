"""Recovery-honest evaluation (panel remediation #2): decouple RECOVERY from FIT.

For every success-labeled trial, place the can at its FK-RECOVERED position (no per-trial
search, no spiral) and the goal can at the TRUE STATIC location (no relocation), then
replay under the corrected metric (contact requires picked; nested requires picked).

This answers the question the searched "50/75 coverage" cannot: how many demos reproduce
success from the RECOVERED positions alone, with the real fixed goal? The gap between this
and the searched coverage is the portion that was fit rather than recovered.

Usage: frozen_fk_eval.py [reps]     (world v2, CPU)
"""
import sys, json, pathlib as pl
import numpy as np
REPO = pl.Path('/home/james/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO / 'can_pos_recovery'))
from replay_harness import build_world, load_episode, rollout, STATIC_BOTTLE_POSITION

REPS = int(sys.argv[1]) if len(sys.argv) > 1 else 3
tbl = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())
wc = tbl['world']
fk = {int(k): v for k, v in
      json.loads((REPO / 'can_pos_recovery/fk_recovered.json').read_text()).items()}

w = build_world(backend='cpu', finger_force=wc['finger_force'], finger_kp=wc['finger_kp'],
                can_height=wc['can_height'], can_rho=wc['can_rho'], substeps=wc['substeps'],
                table=wc['table'], can_radius=wc['can_radius'])
CAN_Z = w['can_start_z']
GOAL = (STATIC_BOTTLE_POSITION[0], STATIC_BOTTLE_POSITION[1], w['goal_start_z'])

# success-labeled, non-stub, with a usable FK position
uids = sorted(u for u, v in fk.items()
              if v['label'] == 'success' and v['conf'] in ('HIGH', 'MED')
              and str(u) not in ('290', '322'))

agg = {'picked': 0, 'contact': 0, 'nested': 0, 'n': 0}
solved_contact = solved_nested = 0
searched = tbl['trials']
print(f"FROZEN-FK eval: can@FK position, goal@static {GOAL[:2]}, {REPS} reps, corrected metric")
print(f"{'uid':>4} {'conf':>4} {'FK can_xy':>16} {'searched?':>9} marks  c-rate n-rate")
for uid in uids:
    vel, gp = load_episode(uid)
    cx, cy = fk[uid]['can_xy']
    marks = []
    cc = nn = 0
    for _ in range(REPS):
        r = rollout(w, vel, gp, (cx, cy, CAN_Z), GOAL)
        agg['picked'] += r['picked']; agg['contact'] += r['success']; agg['nested'] += bool(r['nested'])
        agg['n'] += 1
        cc += r['success']; nn += bool(r['nested'])
        marks.append(('S' if r['success'] else '.') + ('N' if r['nested'] else '.'))
    solved_contact += (cc > 0); solved_nested += (nn > 0)
    st = searched.get(str(uid), {}).get('status', '-')
    gm = searched.get(str(uid), {}).get('goal_moved')
    tag = ('ok/moved' if gm else st) if st in ('ok', 'ok_batch') else st
    print(f"{uid:>4} {fk[uid]['conf']:>4} ({cx:+.3f},{cy:+.3f}) {tag:>9} {' '.join(marks)}"
          f"  {cc/REPS:.2f}  {nn/REPS:.2f}", flush=True)

n = agg['n']
print(f"\n=== RECOVERY-HONEST (frozen FK, static goal, corrected metric) ===")
print(f"trials: {len(uids)} | per-run: picked {agg['picked']/n:.2f} contact {agg['contact']/n:.2f} nested {agg['nested']/n:.2f}")
print(f"coverage (>=1 of {REPS} reps): contact {solved_contact}/{len(uids)} nested {solved_nested}/{len(uids)}")
print(f"\ncompare: searched coverage claimed 50/75; searched replay contact 0.57 nested 0.35")
