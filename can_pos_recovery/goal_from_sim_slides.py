"""Sim-consistent goal estimate (chosen finalize path).

Same ring/circle method as goal_from_slides.py, but on the EXACT sim can-center trajectory
instead of the real tool_pose (which carries an unknown tool->can offset). We replay each
success demo's joints with the GOAL MOVED FAR AWAY, so the picked can rests naturally where
the human stopped pushing (the joint trajectory already encodes the stop -- the human halted
because the real can hit the real goal). The sim can-rest pos + slide direction feed the
identical detector. The resulting goal is in the sim frame and is therefore self-consistent
with the contact metric and with training (no offset caveat).

Usage: python can_pos_recovery/goal_from_sim_slides.py
"""
import os
import json, pathlib as pl
import numpy as np, torch
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from replay_harness import build_world, load_episode, gripper_targets, HARDCODED_START, REPO
from goal_from_slides import slide_contact, DIAM

def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)

BUCKET = {0: (0.4381, 0.1), 1: (0.4381, -0.05), 2: (0.4381, -0.2), None: (0.4381, -0.05)}
tbl = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())
wcfg = tbl['world']; trials = tbl['trials']
fk = {int(k): v for k, v in json.loads((REPO / 'can_pos_recovery/fk_recovered.json').read_text()).items()}

w = build_world(backend='cpu', finger_force=wcfg['finger_force'], finger_kp=wcfg['finger_kp'],
                can_height=wcfg['can_height'], can_rho=wcfg['can_rho'],
                substeps=wcfg.get('substeps', 1), table=True, can_radius=wcfg.get('can_radius', 0.035))
scene, kin, bottle, goal, kdofs, eef = (w['scene'], w['kinova'], w['bottle'], w['goal'], w['kdofs'], w['eef'])
CANZ = w['can_start_z']

uids = sorted(int(u) for u, r in trials.items() if r.get('label') == 'success' and u not in ('290', '322'))
print(f"replaying {len(uids)} success demos, goal moved far away, recording sim can-center", flush=True)

rows = []
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
    goal.set_pos((3.0, 3.0, 0.2)); goal.set_quat([1, 0, 0, 0])   # far away -> no interference
    for e in (bottle, goal):
        try: e.zero_all_dofs_velocity()
        except Exception: pass
    scene.step()
    can_xy = np.zeros((len(vel), 2))
    for i in range(len(vel)):
        kin.control_dofs_position(vel[i], dofs_idx_local=kdofs[:6])
        kin.control_dofs_position(np.array(gripper_targets(gp[i])), dofs_idx_local=np.array(kdofs[-4:]))
        for _ in range(3): scene.step()
        can_xy[i] = np_(bottle.get_pos())[:2]
    res = slide_contact(can_xy, np.asarray(gp).ravel())
    if res is None:
        print(f"{uid}: SKIP (no clean sim slide)", flush=True); continue
    contact, u, q = res
    goal_p = contact + DIAM * u
    rows.append(dict(uid=uid, cx=float(contact[0]), cy=float(contact[1]),
                     ux=float(u[0]), uy=float(u[1]), gx=float(goal_p[0]), gy=float(goal_p[1]), **q))
    print(f"{uid}: contact=({contact[0]:.3f},{contact[1]:.3f}) dir=({u[0]:+.2f},{u[1]:+.2f}) "
          f"goal=({goal_p[0]:.3f},{goal_p[1]:.3f}) alen={q['approach_len']:.3f}", flush=True)

C = np.array([[r['cx'], r['cy']] for r in rows])
G = np.array([[r['gx'], r['gy']] for r in rows])

def circfit(C, d=DIAM):
    c = C.mean(0).copy()
    for _ in range(60):
        dd = C - c; rn = np.hypot(dd[:, 0], dd[:, 1]) + 1e-9
        c = c - np.linalg.lstsq(-dd / rn[:, None], rn - d, rcond=None)[0]
    rms = float(np.sqrt(np.mean((np.hypot(*(C - c).T) - d) ** 2)) * 1000)
    return c, rms

cfit, rms = circfit(C)
rng = np.random.default_rng(0)
bs = np.array([circfit(C[rng.integers(0, len(C), len(C))])[0] for _ in range(400)])
print(f"\n=== SIM-consistent goal over {len(rows)} demos ===")
print(f"circle-fit   ({cfit[0]:.4f},{cfit[1]:.4f})  rms={rms:.1f}mm")
print(f"contact cent ({C.mean(0)[0]:.4f},{C.mean(0)[1]:.4f})  median ({np.median(C,0)[0]:.4f},{np.median(C,0)[1]:.4f})")
print(f"proj median  ({np.median(G,0)[0]:.4f},{np.median(G,0)[1]:.4f})")
print(f"bootstrap 95% CI  x[{np.percentile(bs[:,0],2.5):.3f},{np.percentile(bs[:,0],97.5):.3f}] "
      f"y[{np.percentile(bs[:,1],2.5):.3f},{np.percentile(bs[:,1],97.5):.3f}]")

fig, ax = plt.subplots(figsize=(11, 10))
ax.scatter(C[:, 0], C[:, 1], c='steelblue', s=45, label='sim can-rest (exact center)', zorder=3)
for r in rows:
    ax.annotate('', xy=(r['gx'], r['gy']), xytext=(r['cx'], r['cy']),
                arrowprops=dict(arrowstyle='->', color='orange', alpha=0.45, lw=1))
    ax.text(r['cx'], r['cy'], str(r['uid']), fontsize=6, color='navy')
th = np.linspace(0, 2 * np.pi, 100)
ax.plot(cfit[0] + DIAM * np.cos(th), cfit[1] + DIAM * np.sin(th), 'g--', alpha=0.7, label=f'fit r={DIAM}')
ax.scatter([cfit[0]], [cfit[1]], c='green', marker='*', s=420, zorder=6,
           label=f'SIM circle-fit ({cfit[0]:.3f},{cfit[1]:.3f})')
ax.scatter([np.median(C, 0)[0]], [np.median(C, 0)[1]], c='black', marker='*', s=250, zorder=6,
           label=f'contact median ({np.median(C,0)[0]:.3f},{np.median(C,0)[1]:.3f})')
ax.scatter([0.656], [-0.103], c='cyan', marker='P', s=150, label='old A (0.656,-0.103)', zorder=5)
ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_aspect('equal'); ax.grid(alpha=0.3)
ax.legend(fontsize=8, loc='upper left'); ax.set_title(f'SIM-consistent goal from {len(rows)} slides')
S = str(REPO / 'can_pos_recovery/_scratch')
plt.tight_layout(); plt.savefig(f'{S}/goal_ring_sim.png', dpi=90); print(f"plot -> {S}/goal_ring_sim.png")

(REPO / 'can_pos_recovery/goal_from_sim_slides.json').write_text(json.dumps(dict(
    n=len(rows), circle_fit=[float(cfit[0]), float(cfit[1])], circle_rms_mm=rms,
    contact_median=[float(np.median(C, 0)[0]), float(np.median(C, 0)[1])],
    proj_median=[float(np.median(G, 0)[0]), float(np.median(G, 0)[1])],
    ci_x=[float(np.percentile(bs[:, 0], 2.5)), float(np.percentile(bs[:, 0], 97.5))],
    ci_y=[float(np.percentile(bs[:, 1], 2.5)), float(np.percentile(bs[:, 1], 97.5))],
    trials={r['uid']: dict(contact=[r['cx'], r['cy']], goal_est=[r['gx'], r['gy']],
                           straight=r['straight'], approach_len=r['approach_len']) for r in rows},
), indent=2))
print("saved -> can_pos_recovery/goal_from_sim_slides.json")
