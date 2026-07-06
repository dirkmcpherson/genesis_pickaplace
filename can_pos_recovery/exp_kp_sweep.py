"""Finger-kp sweep on trials whose FK grasp height is comfortably on the can:
does raising finger kp stop the mid-carry slip and get us to placed/success?

Usage: python can_pos_recovery/exp_kp_sweep.py <kp> [uid...]
"""
import sys, json, time
from replay_harness import build_world, load_episode, rollout, STATIC_BOTTLE_POSITION, REPO

kp = float(sys.argv[1])
uids = [int(u) for u in sys.argv[2:]] or [235, 278, 234, 243]
fk = {int(k): v for k, v in json.loads((REPO / 'can_pos_recovery/fk_recovered.json').read_text()).items()}

w = build_world(backend='cpu', finger_force=50.0, finger_kp=kp)
print(f'finger_kp={kp} finger_force=50', flush=True)
for uid in uids:
    vel, gp = load_episode(uid)
    sx, sy = fk[uid]['can_xy']
    for dx, dy in [(0, 0), (-0.01, 0), (0.01, 0), (0, -0.01), (0, 0.01), (-0.02, 0)]:
        xy = (sx + dx, sy + dy)
        t0 = time.time()
        r = rollout(w, vel, gp, (xy[0], xy[1], 0.05), STATIC_BOTTLE_POSITION)
        print(f'  {uid} z={fk[uid]["grasp_mid_z"]:.3f} can=({xy[0]:+.3f},{xy[1]:+.3f}) '
              f'picked={r["picked"]} placed={r["placed"]} success={r["success"]} '
              f'max_z={r["max_z"]} final={r["final_can"]} [{time.time()-t0:.0f}s]', flush=True)
        if r['placed'] or r['success']:
            break
