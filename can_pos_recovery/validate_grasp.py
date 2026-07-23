"""Validate force-controlled grasp across several episodes/positions vs baseline.
Builds the scene ONCE and loops episodes (like example.py). Uses example.py's exact
success test: manipuland contacts the goal can AND eef is behind the can.
Does NOT touch example.py.

Usage: validate_grasp.py <force|baseline>
"""
import os
os.environ.setdefault('PYOPENGL_PLATFORM', 'egl')
import sys, pathlib as pl
import numpy as np

MODE = sys.argv[1] if len(sys.argv) > 1 else 'force'
REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', '/home/j/workspace/genesis_pickaplace'))
sys.path.insert(0, str(REPO))
import genesis as gs
import torch
from kinova import (JOINT_NAMES, EEF_NAME,
                    TRIALS_POSITION_0 as P0, TRIALS_POSITION_1 as P1, TRIALS_POSITION_2 as P2)

def np_(x):
    return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)

UIDS = [235, 232, 257, 242,   322, 243, 236, 280,   290, 278, 237, 234]
POSITION_0 = (0.4381, 0.1, 0.05); POSITION_1 = (0.4381, -0.05, 0.05); POSITION_2 = (0.4381, -0.2, 0.05)
POS = {0: POSITION_0, 1: POSITION_1, 2: POSITION_2}
STATIC = (0.6, -0.2, 0.19)
def posof(u): return 0 if u in P0 else 1 if u in P1 else 2 if u in P2 else None

gs.init(backend=gs.gpu, seed=0, precision="32", logging_level="warning")
if MODE == 'force':
    sim = gs.options.SimOptions(dt=0.01, substeps=4)
    rigid = gs.options.RigidOptions(constraint_timeconst=0.02, iterations=100)
else:
    sim = gs.options.SimOptions(dt=0.01, substeps=1)
    rigid = gs.options.RigidOptions()
scene = gs.Scene(show_viewer=False, sim_options=sim, rigid_options=rigid)
scene.add_entity(gs.morphs.Plane())
BR, BH, BW = 0.035, 0.075, 0.75
scene.add_entity(material=gs.materials.Rigid(rho=1000, friction=0.5),
                 morph=gs.morphs.Box(size=(0.4, BW, 0.12), pos=(0.75, -BW/4, 0.05)))
kinova = scene.add_entity(gs.morphs.URDF(file=str(REPO / 'gen3_lite_2f_robotiq_85.urdf'),
                                         fixed=True, pos=(0.0, 0.0, 0.05)))
bottle = scene.add_entity(material=gs.materials.Rigid(rho=2000, friction=0.2),
                          morph=gs.morphs.Cylinder(pos=POSITION_0, radius=BR, height=BH))
goal = scene.add_entity(material=gs.materials.Rigid(rho=1000, friction=2.0),
                        morph=gs.morphs.Cylinder(pos=STATIC, radius=BR, height=BH))
kdofs = [kinova.get_joint(n).dof_idx_local for n in JOINT_NAMES]
eef = kinova.get_link(EEF_NAME)
L_BOT, R_BOT, L_TIP, R_TIP = kdofs[-4], kdofs[-3], kdofs[-2], kdofs[-1]
scene.build()

if MODE == 'force':
    kinova.set_dofs_force_range(lower=np.array([-50.]*4), upper=np.array([50.]*4), dofs_idx_local=kdofs[-4:])

start = [0.3268500269015339, -1.4471734542578538, 2.3453266624159497, -1.3502152158191212,
         2.209384006676201, -1.5125125137062945, -1, 1, -0.5, 0.5]

def tilt_deg():
    w, x, y, z = np_(bottle.get_quat())
    zz = 1 - 2*(x*x + y*y); zx = 2*(x*z + w*y); zy = 2*(y*z - w*x)
    return float(np.degrees(np.arccos(max(-1, min(1, zz/(np.sqrt(zx*zx+zy*zy+zz*zz)+1e-9))))))

print(f"MODE={MODE}")
print(f"{'uid':>4} {'pos':>3} {'success':>7} {'@step':>6} {'maxspin':>7} {'lifted':>6} {'tilt':>5} {'finz':>5} {'nstep':>6}")
succ_by_pos = {0: [0, 0], 1: [0, 0], 2: [0, 0]}
for uid in UIDS:
    p = posof(uid)
    f = REPO / f'inthewild_trials/{uid}_episodes.npy'
    if p is None or not f.exists():
        print(f"{uid:>4}   skip (pos={p}, exists={f.exists()})"); continue
    d = np.load(f, allow_pickle=True).item()
    vel = np.asarray(d['vel_cmd']); gpos = np.asarray(d['gripper_pos'])
    # setup (mirror example.py)
    kinova.set_dofs_position(np.array(start), kdofs)
    bottle.set_pos(POS[p]); bottle.set_quat([1, 0, 0, 0])
    goal.set_pos(STATIC); goal.set_quat([1, 0, 0, 0])
    scene.step()
    success = False; succ_step = -1; lifted = 0; max_spin = 0.0
    for i in range(len(vel)):
        kinova.control_dofs_position(vel[i], dofs_idx_local=kdofs[:6])
        motor = (100 - gpos[i][0]) / 100
        if MODE == 'force':
            close = float(np.clip(1.0 - motor, 0.0, 1.0)); tau = 10.0 * close
            kinova.control_dofs_force(np.array([+tau, -tau]), dofs_idx_local=[L_BOT, R_BOT])
            kinova.control_dofs_position(np.array([-0.5, -0.5]), dofs_idx_local=[L_TIP, R_TIP])
        else:
            kinova.control_dofs_position(np.array([-motor, motor, -0.5, -0.5]), dofs_idx_local=kdofs[-4:])
        scene.step()
        max_spin = max(max_spin, float(np.linalg.norm(np_(bottle.get_ang()))))
        if float(np_(bottle.get_pos())[2]) > 0.09: lifted = 1
        if i % 30 == 0 and not success:
            ncon = np_(bottle.get_contacts(with_entity=goal)['position'])
            ncon = 0 if ncon.size == 0 else ncon.shape[0]
            if ncon and float(np_(eef.get_pos())[0]) < float(np_(bottle.get_pos())[0]):
                success = True; succ_step = i; break
    finz = float(np_(bottle.get_pos())[2])
    succ_by_pos[p][1] += 1; succ_by_pos[p][0] += int(success)
    print(f"{uid:>4} {p:>3} {('YES' if success else 'no'):>7} {succ_step:>6} "
          f"{max_spin:>7.1f} {lifted:>6} {tilt_deg():>5.0f} {finz:>5.3f} {len(vel):>6}")

tot_s = sum(v[0] for v in succ_by_pos.values()); tot_n = sum(v[1] for v in succ_by_pos.values())
print(f"\n[{MODE}] success by position: " +
      "  ".join(f"P{p}={succ_by_pos[p][0]}/{succ_by_pos[p][1]}" for p in (0, 1, 2)) +
      f"  | TOTAL={tot_s}/{tot_n}")
