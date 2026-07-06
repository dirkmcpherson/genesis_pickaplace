"""Grip-config sweep: which finger (kp, force-range) lets the open-loop replay actually
hold the can? Runs pick-gate rollouts for a few trials at their FK seeds + neighbors.

Usage: python can_pos_recovery/exp_grip.py <finger_kp|none> <finger_force|none> [uid...]
"""
import sys, time
import numpy as np
from replay_harness import build_world, load_episode, rollout, STATIC_BOTTLE_POSITION

fkp = None if sys.argv[1] == 'none' else float(sys.argv[1])
ff = None if sys.argv[2] == 'none' else float(sys.argv[2])
uids = [int(u) for u in sys.argv[3:]] or [232, 235, 236, 278]

# FK seeds from the handoff doc sample (fk_recovered.json may still be generating)
SEED = {232: (0.488, 0.086), 235: (0.478, 0.088), 236: (0.469, -0.055), 278: (0.445, -0.174),
        242: (0.493, 0.114), 243: (0.452, -0.037), 237: (0.453, -0.182), 234: (0.424, -0.137)}
GATE = {232: 550, 235: 550, 236: 550, 278: 550, 242: 550, 243: 550, 237: 550, 234: 550}

w = build_world(backend='cpu', finger_force=ff, finger_kp=fkp)
print(f"config: finger_kp={sys.argv[1]} finger_force={sys.argv[2]}", flush=True)
for uid in uids:
    vel, gp = load_episode(uid)
    sx, sy = SEED[uid]
    best = None
    for dx, dy in [(0, 0), (-0.02, 0), (-0.01, 0), (0.01, 0), (0, -0.01), (0, 0.01)]:
        xy = (sx + dx, sy + dy)
        t0 = time.time()
        r = rollout(w, vel, gp, (xy[0], xy[1], 0.05), STATIC_BOTTLE_POSITION,
                    max_cmds=GATE.get(uid, 550))
        print(f"  {uid} can=({xy[0]:+.3f},{xy[1]:+.3f}) picked={r['picked']} "
              f"max_z={r['max_z']} [{time.time()-t0:.0f}s]", flush=True)
        if r['picked']:
            best = xy
            break
    if best:
        r = rollout(w, vel, gp, (best[0], best[1], 0.05), STATIC_BOTTLE_POSITION)
        print(f"  {uid} FULL at ({best[0]:+.3f},{best[1]:+.3f}): picked={r['picked']} "
              f"placed={r['placed']} success={r['success']} final={r['final_can']}", flush=True)
