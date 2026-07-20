import sys, json, pathlib as pl, numpy as np, torch
sys.path.insert(0, 'can_pos_recovery')
from replay_harness import build_world, load_episode, gripper_targets, HARDCODED_START, REPO
def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)
UID = int(sys.argv[1])
tbl = json.loads((REPO/'can_pos_recovery/trial_placements.json').read_text())
wcfg = tbl['world']; r = tbl['trials'][str(UID)]
w = build_world(backend='cpu', finger_force=wcfg['finger_force'], finger_kp=wcfg['finger_kp'],
                can_height=wcfg['can_height'], can_rho=wcfg['can_rho'], substeps=wcfg['substeps'],
                table=True, can_radius=wcfg['can_radius'])
scene,kin,bottle,goal,kdofs,eef = w['scene'],w['kinova'],w['bottle'],w['goal'],w['kdofs'],w['eef']
vel, gp = load_episode(UID)
kin.set_dofs_position(np.array(HARDCODED_START), kdofs); kin.zero_all_dofs_velocity()
bottle.set_pos(tuple(r['can_pos'])); bottle.set_quat(list(r.get('can_quat') or [1,0,0,0]))
goal.set_pos((0.672,-0.221,w['goal_start_z'])); goal.set_quat([1,0,0,0])
for e in (bottle,goal):
    try: e.zero_all_dofs_velocity()
    except Exception: pass
scene.step()
rows=[]
for i in range(len(vel)):
    kin.control_dofs_position(vel[i], dofs_idx_local=kdofs[:6])
    kin.control_dofs_position(np.array(gripper_targets(gp[i])), dofs_idx_local=np.array(kdofs[-4:]))
    for _ in range(3): scene.step()
    bp=np_(bottle.get_pos()); ep=np_(eef.get_pos())
    rows.append([bp[0],bp[1],bp[2],ep[0],ep[1],ep[2]])
np.save(sys.argv[2], np.array(rows))
print("replay-path done", len(rows))
