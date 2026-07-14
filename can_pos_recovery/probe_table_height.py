"""Probe: does raising the table stop the can tipping over during carry?

Hypothesis (user): the table is too low, so the fixed arm trajectory grips the can HIGH on
its body (top-heavy) -> it tips. Raising the table lifts the can into the grasp so the
fingers close lower (nearer the base) -> stable.

One table height per process (gs.init is once-per-process). For a fixed set of tip-prone
success demos it replays each at this height and records: pick, grasp height (eef_z minus
can-center at the pick frame -- decreases as the grasp moves down the can), whether the can
stays UPRIGHT to the end (tilt<20), contact. Writes scratchpad/table_probe_<T>.json.

Usage: probe_table_height.py --table-top 0.05 [--n-tip 12 --n-stable 4]
"""
import argparse, json, pathlib as pl
import numpy as np
import torch
from replay_harness import (build_world, load_episode, gripper_targets, tilt_deg,
                            HARDCODED_START, STATIC_BOTTLE_POSITION, REPO)

def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)

SCRATCH = pl.Path('/tmp/claude-1000/-home-james-workspace-genesis-pickaplace/'
                  '5d60af7b-ae54-45fa-bc0c-e90077b3afaf/scratchpad')
BUCKET = {0: (0.4381, 0.1), 1: (0.4381, -0.05), 2: (0.4381, -0.2), None: (0.4381, -0.05)}

ap = argparse.ArgumentParser()
ap.add_argument('--table-top', type=float, default=0.05)
ap.add_argument('--can-friction', type=float, default=None, help='can material friction (build_world default 0.2)')
ap.add_argument('--table-friction', type=float, default=None, help='table+shelf surface friction (default 0.5) -- the effective set-down friction under the max-combine rule')
ap.add_argument('--finger-kp', type=float, default=None, help='finger stiffness; lower = softer/compliant (rubber-band proxy)')
ap.add_argument('--finger-force', type=float, default=None, help='override world_cfg finger_force')
ap.add_argument('--n-tip', type=int, default=12)
ap.add_argument('--n-stable', type=int, default=4)
args = ap.parse_args()

table = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())
wcfg = table['world']
fk_all = {int(k): v for k, v in
          json.loads((REPO / 'can_pos_recovery/fk_recovered.json').read_text()).items()}
# pick the demo set from the overnight gallery metrics: tip-prone (picked & tilt>45) + stable controls
gm = json.loads((REPO / 'can_pos_recovery/videos_gallery/metrics.json').read_text())
S = [r for r in gm if r['label'] == 'success']
tip = [r['uid'] for r in S if r['picked'] and r['final_tilt_deg'] > 45][:args.n_tip]
stable = [r['uid'] for r in S if r['outcome'] in ('S', 'S-nested')
          and r['final_tilt_deg'] < 20][:args.n_stable]
demoset = [(u, 'tip') for u in tip] + [(u, 'stable') for u in stable]

_ff = args.finger_force if args.finger_force is not None else wcfg['finger_force']
_cf = args.can_friction if args.can_friction is not None else 0.2
_tf = args.table_friction if args.table_friction is not None else 0.5
_fkp = args.finger_kp if args.finger_kp is not None else wcfg['finger_kp']
w = build_world(backend='cpu', finger_force=_ff, finger_kp=_fkp,
                can_height=wcfg['can_height'], can_rho=wcfg['can_rho'],
                substeps=wcfg.get('substeps', 1), table=True,
                can_radius=wcfg.get('can_radius', 0.035), table_top=args.table_top,
                can_friction=_cf, table_friction=_tf, goal_friction=max(2.0, _tf))
scene, kinova, bottle, goal, kdofs, eef = (w['scene'], w['kinova'], w['bottle'],
                                           w['goal'], w['kdofs'], w['eef'])
CANZ = w['can_start_z']; GOALZ = w['goal_start_z']; PICKZ = w['pick_z']
print(f"[probe] table_top={args.table_top:.3f} can_start_z={CANZ:.3f} pick_z={PICKZ:.3f} "
      f"| {len(tip)} tip + {len(stable)} stable demos", flush=True)


def place(uid):
    r = table['trials'][str(uid)]
    solved = r['status'] in ('ok', 'ok_batch')
    can_quat = list(r.get('can_quat') or [1, 0, 0, 0]) if solved else [1, 0, 0, 0]
    if solved:
        cx, cy = r['can_pos'][0], r['can_pos'][1]
        if r.get('label') == 'success' and not r.get('goal_moved'):
            gx, gy = r['goal_pos'][0], r['goal_pos'][1]
        else:
            gx, gy = STATIC_BOTTLE_POSITION[0], STATIC_BOTTLE_POSITION[1]
    else:
        fk = fk_all.get(uid, {})
        seed = fk.get('close_xy') or (fk.get('can_xy') if fk.get('conf') in ('HIGH', 'MED')
                                      else BUCKET[fk.get('pos')])
        cx, cy = seed[0], seed[1]
        gx, gy = STATIC_BOTTLE_POSITION[0], STATIC_BOTTLE_POSITION[1]
    return (cx, cy, CANZ), can_quat, (gx, gy, GOALZ)


rows = []
for uid, cls in demoset:
    can_pos, can_quat, goal_pos = place(uid)
    vel, gp = load_episode(uid)
    kinova.set_dofs_position(np.array(HARDCODED_START), kdofs); kinova.zero_all_dofs_velocity()
    bottle.set_pos(can_pos); bottle.set_quat(can_quat)
    goal.set_pos(goal_pos); goal.set_quat([1, 0, 0, 0])
    for ent in (bottle, goal):
        try: ent.zero_all_dofs_velocity()
        except Exception: pass
    scene.step()

    picked = success = False
    max_z = 0.0; grasp_eef_minus_can = None
    for i in range(len(vel)):
        kinova.control_dofs_position(vel[i], dofs_idx_local=kdofs[:6])
        kinova.control_dofs_position(np.array(gripper_targets(gp[i])),
                                     dofs_idx_local=np.array(kdofs[-4:]))
        for _ in range(3):
            scene.step()
        bp = np_(bottle.get_pos()); max_z = max(max_z, float(bp[2]))
        if not picked and bp[2] > PICKZ and gp[i] > 30:
            picked = True
            grasp_eef_minus_can = float(np_(eef.get_pos())[2] - bp[2])  # grasp height above can center
        if i % 30 == 0 and not success:
            c = np_(bottle.get_contacts(goal)['position'])
            if (c.size and c.shape[0]) and float(np_(eef.get_pos())[0]) < float(bp[0]):
                success = True
    for _ in range(100):
        scene.step()
    bq = np_(bottle.get_quat()); final_tilt = float(tilt_deg(bq))
    upright = final_tilt < 20
    rows.append(dict(uid=uid, cls=cls, picked=picked, upright=upright, contact=success,
                     final_tilt=round(final_tilt, 1), max_z=round(max_z, 4),
                     grasp_eef_minus_can=(round(grasp_eef_minus_can, 4)
                                          if grasp_eef_minus_can is not None else None)))
    print(f"  {uid}({cls}): pick={picked} upright={upright} contact={success} "
          f"tilt={final_tilt:.0f} grasp_dz={grasp_eef_minus_can}", flush=True)

n = len(rows)
npick = sum(r['picked'] for r in rows)
nupright = sum(r['upright'] for r in rows)
ncontact = sum(r['contact'] for r in rows)
# among demos that picked, what fraction stayed upright?
pk = [r for r in rows if r['picked']]
up_given_pick = sum(r['upright'] for r in pk) / max(len(pk), 1)
gdz = [r['grasp_eef_minus_can'] for r in pk if r['grasp_eef_minus_can'] is not None]
summ = dict(table_top=args.table_top, can_friction=_cf, table_friction=_tf,
            finger_kp=_fkp, finger_force=_ff,
            can_start_z=round(CANZ, 4), pick_z=round(PICKZ, 4),
            n=n, pick=npick, upright=nupright, contact=ncontact,
            upright_given_pick=round(up_given_pick, 3),
            mean_grasp_dz=round(float(np.mean(gdz)), 4) if gdz else None, rows=rows)
out = SCRATCH / f'probe_tf{_tf:.1f}_cf{_cf:.2f}_fkp{_fkp:.0f}.json'
out.write_text(json.dumps(summ, indent=2))
print(f"\n[probe] tf={_tf:.1f} cf={_cf:.2f} fkp={_fkp:.0f} tt={args.table_top:.3f}: "
      f"pick {npick}/{n}, upright {nupright}/{n}, contact {ncontact}/{n}, "
      f"upright|pick {up_given_pick:.2f}, grasp_dz {summ['mean_grasp_dz']} -> {out.name}", flush=True)
