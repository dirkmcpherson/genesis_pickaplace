"""Negative control: replay the FAIL-labeled demos at their FK-recovered can position
with the goal can at its standard spot (no per-trial search, no goal relocation) and
check they fail, as they did in the real world.

A fail-labeled demo that replays to success is a red flag: sim-physics divergence,
label noise (cf. 322/331 double-listings), or a demo that failed for reasons the sim
doesn't model. Report per-checkpoint outcomes so failures can be compared to the real
failure modes (videos in inthewild_trials/raw/ where available).

Usage: negative_control.py [reps]   (world v2, CPU backend)
"""
import sys, json
from replay_harness import build_world, load_episode, rollout, STATIC_BOTTLE_POSITION, REPO

REPS = int(sys.argv[1]) if len(sys.argv) > 1 else 2
BUCKET = {0: (0.4381, 0.1), 1: (0.4381, -0.05), 2: (0.4381, -0.2), None: (0.4381, -0.05)}

fk_all = {int(k): v for k, v in
          json.loads((REPO / 'can_pos_recovery/fk_recovered.json').read_text()).items()}
fails = {u: v for u, v in sorted(fk_all.items()) if v['label'] == 'fail'}

# config parity with the positive eval (audit fix): world from the placements table
_wc = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())['world']
w = build_world(backend='cpu', finger_force=_wc['finger_force'], finger_kp=_wc['finger_kp'],
                can_height=_wc['can_height'], can_rho=_wc['can_rho'],
                substeps=_wc['substeps'], table=_wc['table'], can_radius=_wc['can_radius'])
goal = (STATIC_BOTTLE_POSITION[0], STATIC_BOTTLE_POSITION[1], w['goal_start_z'])

print(f"NEGATIVE CONTROL - {len(fails)} fail-labeled demos, world v2, {REPS} reps")
print(f"{'uid':>4} {'conf':>4} {'seed':>16}  outcomes (S=success P=placed p=picked .=nothing)")
n_false_pos = 0
for uid, fk in fails.items():
    vel, gp = load_episode(uid)
    seed = tuple(fk['can_xy']) if fk['conf'] in ('HIGH', 'MED') else BUCKET[fk['pos']]
    marks = []
    nested_fp = False
    for _ in range(REPS):
        # stop_on_success=True: SAME scoring path as the positive eval (audit fix --
        # fail demos were previously scored full-episode, stricter than success demos)
        r = rollout(w, vel, gp, (seed[0], seed[1], w['can_start_z']), goal)
        m = 'S' if r['success'] else 'P' if r['placed'] else 'p' if r['picked'] else '.'
        if r['nested']:
            m += 'N'; nested_fp = True
        marks.append(m)
    if 'S' in ''.join(marks):
        n_false_pos += 1
    print(f"{uid:>4} {fk['conf']:>4} ({seed[0]:+.3f},{seed[1]:+.3f})  {' '.join(marks)}",
          flush=True)
print(f"\nfail-labeled demos that replay to contact-SUCCESS: {n_false_pos}/{len(fails)}"
      f"  (N marks = would also pass the stricter nested metric)")
