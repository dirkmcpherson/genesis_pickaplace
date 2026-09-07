#!/usr/bin/env python3
"""Unit check for the `contact_push` predicate (genesis_can_env.step, 2026-09-07, PHASE_PLAN amendment (g)).

Scripted states, CPU, one world: a HELD can (a place/pick-grant bank entry restored as full_env does), the pick-can
then shifted `gap` metres forward out of the fingers (0 = still held), and the GOAL can teleported to touch the
pick-can at angle theta from the eef->can direction (table plane, same z). Flags are read on the next env.step.
  (a) theta = 0, gap > 0: goal straight ahead, gripper behind the can, not touching the goal -> contact T, contact_push T
  (b) theta > 90 (dot > 0), no gripper-goal contact: goal on the gripper's side -> contact may be T, contact_push F (wrong side)
  (c) theta = 0, gap = 0: held can pushed straight into the goal -- the finger tips protrude past the can and touch the
      goal too -> contact T, contact_push F (gripper-goal contact)
  control: goal 0.5 m away -> contact F, contact_push F
Exits non-zero if the canonical cases do not come out as expected.
usage: contact_push_check.py [--variant gc_kp4_riser3_shelf6] [--bank <holdE_place.json>] [--uid 252]"""
import argparse, json, os, pathlib as pl, sys
import numpy as np

HERE = pl.Path(__file__).resolve().parent
REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', HERE.parents[1]))
for p in (REPO / 'baselines', REPO / 'baselines' / 'rl', REPO / 'can_pos_recovery'):
    sys.path.insert(0, str(p))

ap = argparse.ArgumentParser()
ap.add_argument('--variant', default='gc_kp4_riser3_shelf6')
ap.add_argument('--bank', default=None, help='entry bank JSON (dict uid->entry or list); default: a held-can state from pick_entry_states.json')
ap.add_argument('--uid', type=int, default=252)
ap.add_argument('--thetas', type=float, nargs='*', default=[0.0, 30.0, 60.0, 90.0, 100.0, 110.0, 135.0, 180.0])
ap.add_argument('--gaps', type=float, nargs='*', default=[0.0, 0.03, 0.06, 0.09], help='forward shift of the pick-can out of the fingers (m)')
args = ap.parse_args()

import sim_variant_hook as svh
svh.apply_pre(args.variant)
from genesis_can_env import GenesisCanEnv, np_, GP_CLOSE
from replay_harness import HARDCODED_START, gripper_targets

env = GenesisCanEnv(backend='cpu', max_steps=10**9)
svh.apply_post(env, args.variant)
w = env.w

bank_path = pl.Path(args.bank) if args.bank else REPO / 'baselines' / 'pick_entry_states.json'
raw = json.loads(bank_path.read_text())
entries = [dict(e, uid=int(u)) for u, e in raw.items()] if isinstance(raw, dict) else [dict(e, uid=int(e['uid'])) for e in raw]
e = next(x for x in sorted(entries, key=lambda x: (x['uid'], x['frame'])) if x['uid'] == args.uid)
print(f'[check] bank {bank_path.name} uid {e["uid"]} frame {e["frame"]} grip_cmd {e["grip_cmd"]:.2f} grip_obs {e["grip_obs"]:.2f}')

CAN_R = float(env.world_cfg.get('can_radius', 0.035))
TOUCH = 2.0 * CAN_R - 0.002   # 2 mm overlap so the solver registers the pair


def restore_held():
    """Mirror full_env._restore_place_entry: can + goal + arm from the entry, hold the commands, settle."""
    goal_xy = e.get('goal_xy') or [0.672, -0.221]
    goal_pos = (float(goal_xy[0]), float(goal_xy[1]), w['goal_start_z'])
    env.reset(can_pos=e['can_pos'], can_quat=e['can_quat'], goal_pos=goal_pos)
    kin = w['kinova']
    q = np.array(HARDCODED_START, dtype=np.float64); q[:6] = e['qpos']
    q[6:] = gripper_targets(float(e['grip_obs']) * 100.0)
    kin.set_dofs_position(q, w['kdofs']); kin.zero_all_dofs_velocity()
    w['bottle'].set_pos(e['can_pos']); w['bottle'].set_quat(list(e['can_quat']))
    try: w['bottle'].zero_all_dofs_velocity()
    except Exception: pass
    kin.control_dofs_position(np.asarray(e['qpos'], np.float64), dofs_idx_local=w['kdofs'][:6])
    kin.control_dofs_position(np.array(gripper_targets(float(e['grip_cmd']) * 100.0)), dofs_idx_local=np.array(w['kdofs'][-4:]))
    for _ in range(20): w['scene'].step()
    env._picked = True   # the pick already happened in the tape this state came from (full_env pre-grants it)


def place_goal_and_step(theta_deg=None, far=False, gap=0.0):
    restore_held()
    bp = np_(w['bottle'].get_pos()); ee = np_(w['eef'].get_pos())
    u = bp[:2] - ee[:2]; u = u / np.linalg.norm(u)             # eef -> can (forward)
    n = np.array([-u[1], u[0]])                                 # left-hand normal
    if gap > 0:   # shift the pick-can forward out of the fingers (a can that has just left the grasp)
        bp = np.array([bp[0] + gap * u[0], bp[1] + gap * u[1], bp[2]])
        w['bottle'].set_pos([float(x) for x in bp])
        try: w['bottle'].zero_all_dofs_velocity()
        except Exception: pass
    th = np.deg2rad(theta_deg or 0.0)
    d = np.cos(th) * u + np.sin(th) * n
    gxy = bp[:2] + (0.5 if far else TOUCH) * d
    w['goal'].set_pos([float(gxy[0]), float(gxy[1]), float(bp[2])]); w['goal'].set_quat([1, 0, 0, 0])
    try: w['goal'].zero_all_dofs_velocity()
    except Exception: pass
    # one env step holding the entry commands (grip closed = held can); the predicates read this step's solver contacts
    a = np.concatenate([np.asarray(e['qpos'], np.float64), [float(e['grip_cmd'])]])
    _obs, _done, info = env.step(a)
    bp2 = np_(w['bottle'].get_pos()); ee2 = np_(w['eef'].get_pos()); gp = np_(w['goal'].get_pos())
    dot = float((ee2[0]-bp2[0])*(gp[0]-bp2[0]) + (ee2[1]-bp2[1])*(gp[1]-bp2[1]))
    bg = np_(w['bottle'].get_contacts(w['goal'])['position']); gg = np_(w['goal'].get_contacts(w['kinova'])['position'])
    gg_rel = [[round(float(np.dot(p[:2] - bp2[:2], u)), 3), round(float(np.dot(p[:2] - bp2[:2], n)), 3)] for p in (gg.reshape(-1, 3) if gg.size else [])]
    return dict(theta=('far' if far else theta_deg), gap=gap, dot=round(dot, 5), ee_x_lt_can_x=bool(ee2[0] < bp2[0]), gg_xy_rel_can=gg_rel,
                bottle_goal_contacts=int(bg.shape[0] if bg.size else 0), gripper_goal_contacts=int(gg.shape[0] if gg.size else 0),
                picked=bool(info['picked']), contact=bool(info['contact']), contact_push=bool(info['contact_push']),
                contact_gripper_goal=bool(info['contact_gripper_goal']), contact_farside=bool(info['contact_farside']),
                contact_frame=info['contact_frame'], contact_push_frame=info['contact_push_frame'])


rows = [place_goal_and_step(far=True)] + [place_goal_and_step(t, gap=g) for g in args.gaps for t in args.thetas]
print(f'\n{"gap":>5} {"theta":>6} {"dot":>9} {"ee_x<can_x":>10} {"bg":>3} {"gg":>3} {"contact":>8} {"push":>6} {"gg_any":>7} {"farside":>8}  gg contact xy rel. can (fwd, left)')
for r in rows:
    print(f'{r["gap"]:>5.2f} {str(r["theta"]):>6} {r["dot"]:>9.5f} {str(r["ee_x_lt_can_x"]):>10} {r["bottle_goal_contacts"]:>3} {r["gripper_goal_contacts"]:>3} '
          f'{str(r["contact"]):>8} {str(r["contact_push"]):>6} {str(r["contact_gripper_goal"]):>7} {str(r["contact_farside"]):>8}  {r["gg_xy_rel_can"]}')
print('\nrows_json=' + json.dumps(rows))

# canonical cases: (a) theta 0, smallest gap with bg>0 and gg==0; (b) the first theta>90 row with bg>0, gg==0, dot>0;
# (c) theta 0 at gap 0 (held can pushed straight in) with bg>0 and gg>0
sc = [r for r in rows if r['theta'] != 'far']
a = next((r for r in sc if r['theta'] == 0.0 and r['bottle_goal_contacts'] > 0 and r['gripper_goal_contacts'] == 0), None)
b = next((r for r in sc if r['theta'] > 90 and r['bottle_goal_contacts'] > 0 and r['gripper_goal_contacts'] == 0 and r['dot'] > 0), None)
c = next((r for r in sc if r['theta'] == 0.0 and r['gap'] == 0.0 and r['bottle_goal_contacts'] > 0 and r['gripper_goal_contacts'] > 0), None)
ok = True
ok &= bool(a and a['contact'] and a['contact_push']); print(f'(a) ahead, gripper behind the can, no gripper-goal contact: {"gap %.2f" % a["gap"] if a else "NO SUCH ROW"} contact={a and a["contact"]} push={a and a["contact_push"]}')
ok &= bool(b and not b['contact_push']); print(f'(b) goal on the gripper side (dot>0), no gripper-goal contact: {"gap %.2f theta %s" % (b["gap"], b["theta"]) if b else "NO SUCH ROW"} contact={b and b["contact"]} push={b and b["contact_push"]}')
ok &= bool(c and c['contact'] and not c['contact_push'] and c['contact_gripper_goal']); print(f'(c) held can pushed straight in, finger tips touch the goal: {"gap 0" if c else "NO SUCH ROW"} contact={c and c["contact"]} push={c and c["contact_push"]} gripper_goal={c and c["contact_gripper_goal"]}')
print('CONTROL far goal:', 'contact', rows[0]['contact'], 'push', rows[0]['contact_push'])
print('UNIT CHECK', 'PASS' if ok and not rows[0]['contact'] else 'FAIL')
sys.exit(0 if ok and not rows[0]['contact'] else 1)
