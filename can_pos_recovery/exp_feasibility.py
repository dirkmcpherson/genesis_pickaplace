"""Feasibility probe: under example.py's control, can trial 232 pick up the can at all?
Tests finger-force variants at the FK-recovered position, plus a small position sweep.

Usage: python can_pos_recovery/exp_feasibility.py <finger_force|none> [backend]
"""
import sys, time, json
import numpy as np
from replay_harness import build_world, load_episode, rollout, STATIC_BOTTLE_POSITION

ff_arg = sys.argv[1] if len(sys.argv) > 1 else 'none'
finger_force = None if ff_arg == 'none' else float(ff_arg)
backend = sys.argv[2] if len(sys.argv) > 2 else 'gpu'

w = build_world(backend=backend, finger_force=finger_force)
vel, gp = load_episode(232)

FK_XY = (0.488, 0.086)
candidates = [FK_XY] + [(FK_XY[0] + dx, FK_XY[1] + dy)
                        for dx, dy in [(-0.02, 0), (0.02, 0), (0, -0.02), (0, 0.02),
                                       (-0.04, 0), (-0.02, -0.02), (-0.02, 0.02)]]
print(f"config: finger_force={ff_arg} backend={backend}")
for xy in candidates:
    t0 = time.time()
    # pick gate only: grasp for 232 happens ~ step 400 per recovery; run 550 cmds
    r = rollout(w, vel, gp, (xy[0], xy[1], 0.05), STATIC_BOTTLE_POSITION, max_cmds=550)
    print(f"can=({xy[0]:+.3f},{xy[1]:+.3f}) picked={r['picked']} max_z={r['max_z']} "
          f"final={r['final_can']} [{time.time()-t0:.0f}s]")
    if r['picked']:
        print("-> PICKED. running full episode at this position...")
        r = rollout(w, vel, gp, (xy[0], xy[1], 0.05), STATIC_BOTTLE_POSITION)
        print(json.dumps(r))
        break
