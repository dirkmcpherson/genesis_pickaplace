"""Tall-can hypothesis test: same trials, can height 0.10 (soup can) instead of 0.075.
FK grasp heights cluster at/above the 0.075 can's top; a 0.10 can puts them 2-4cm below
the rim where a pinch grasp is stable.

Usage: python can_pos_recovery/exp_tallcan.py <finger_kp> <can_height> [uid...]
"""
import sys, json, time
from replay_harness import build_world, load_episode, rollout, REPO, STATIC_BOTTLE_POSITION

kp = float(sys.argv[1]); ch = float(sys.argv[2]); rho = float(sys.argv[3]); ss = int(sys.argv[4])
uids = [int(u) for u in sys.argv[5:]] or [235, 278, 234, 243, 232, 236]
fk = {int(k): v for k, v in json.loads((REPO / 'can_pos_recovery/fk_recovered.json').read_text()).items()}

w = build_world(backend='cpu', finger_force=50.0, finger_kp=kp, can_height=ch, can_rho=rho,
                substeps=ss)
gz = w['goal_start_z']
print(f'finger_kp={kp} can_height={ch} can_z={w["can_start_z"]:.4f} goal_z={gz:.4f}', flush=True)
n_ok = 0
for uid in uids:
    vel, gp = load_episode(uid)
    sx, sy = fk[uid]['can_xy']
    got = False
    for dx, dy in [(0, 0), (-0.01, 0), (0.01, 0), (0, -0.01), (0, 0.01), (-0.02, 0), (0.02, 0)]:
        xy = (sx + dx, sy + dy)
        t0 = time.time()
        r = rollout(w, vel, gp, (xy[0], xy[1], w['can_start_z']),
                    (STATIC_BOTTLE_POSITION[0], STATIC_BOTTLE_POSITION[1], gz))
        print(f'  {uid} gz={fk[uid]["grasp_mid_z"]:.3f} can=({xy[0]:+.3f},{xy[1]:+.3f}) '
              f'picked={r["picked"]} placed={r["placed"]} success={r["success"]} '
              f'max_z={r["max_z"]} final={r["final_can"]} [{time.time()-t0:.0f}s]', flush=True)
        if r['success']:
            got = True; break
        if r['placed'] and not got:
            got = True; break
    n_ok += int(got)
print(f'--- {n_ok}/{len(uids)} trials reached placed-or-better', flush=True)
