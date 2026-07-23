"""Close the loop: does placing the can at its FK-RECOVERED position (instead of the
hand-coded bucket) raise success? Same baseline control (what example.py uses) in both
runs; only the can's start (x,y) changes. Does NOT touch example.py.

Usage: close_loop.py <bucket|recovered>
"""
import os
os.environ.setdefault('PYOPENGL_PLATFORM', 'egl')
import sys, pathlib as pl
import numpy as np
MODE = sys.argv[1] if len(sys.argv) > 1 else 'bucket'
REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', '/home/j/workspace/genesis_pickaplace'))
sys.path.insert(0, str(REPO))
import genesis as gs
import torch
from kinova import (JOINT_NAMES, EEF_NAME,
                    TRIALS_POSITION_0 as P0, TRIALS_POSITION_1 as P1, TRIALS_POSITION_2 as P2)

def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)

UIDS = [235, 232, 257, 242,  322, 243, 236, 280,  290, 278, 237, 234]
BUCKET = {0: (0.4381, 0.1, 0.05), 1: (0.4381, -0.05, 0.05), 2: (0.4381, -0.2, 0.05)}
# FK-recovered (x,y) from recover_v2 (HIGH/MED conf); LOW-conf (322,290) fall back to bucket
RECOVERED = {
    235: (0.478, 0.088), 232: (0.488, 0.086), 257: (0.442, 0.098), 242: (0.493, 0.114),
    243: (0.452, -0.037), 236: (0.469, -0.055), 280: (0.447, -0.004),
    278: (0.445, -0.174), 237: (0.453, -0.182), 234: (0.424, -0.137),
    # 322, 290 -> LOW confidence: keep bucket
}
STATIC = (0.6, -0.2, 0.19)
def posof(u): return 0 if u in P0 else 1 if u in P1 else 2 if u in P2 else None

def start_xy(uid):
    p = posof(uid); bz = 0.05
    if MODE == 'recovered' and uid in RECOVERED:
        rx, ry = RECOVERED[uid]; return (rx, ry, bz), 'rec'
    return BUCKET[p], 'buk'

gs.init(backend=gs.gpu, seed=0, precision="32", logging_level="warning")
# baseline control config = what example.py originally had (the bug config)
scene = gs.Scene(show_viewer=False, sim_options=gs.options.SimOptions(dt=0.01, substeps=1))
scene.add_entity(gs.morphs.Plane())
BR, BH, BW = 0.035, 0.075, 0.75
scene.add_entity(material=gs.materials.Rigid(rho=1000, friction=0.5),
                 morph=gs.morphs.Box(size=(0.4, BW, 0.12), pos=(0.75, -BW/4, 0.05)))
kinova = scene.add_entity(gs.morphs.URDF(file=str(REPO / 'gen3_lite_2f_robotiq_85.urdf'),
                                         fixed=True, pos=(0.0, 0.0, 0.05)))
bottle = scene.add_entity(material=gs.materials.Rigid(rho=2000, friction=0.2),
                          morph=gs.morphs.Cylinder(pos=BUCKET[0], radius=BR, height=BH))
goal = scene.add_entity(material=gs.materials.Rigid(rho=1000, friction=2.0),
                        morph=gs.morphs.Cylinder(pos=STATIC, radius=BR, height=BH))
kdofs = [kinova.get_joint(n).dof_idx_local for n in JOINT_NAMES]
eef = kinova.get_link(EEF_NAME)
scene.build()

start = [0.3268500269015339, -1.4471734542578538, 2.3453266624159497, -1.3502152158191212,
         2.209384006676201, -1.5125125137062945, -1, 1, -0.5, 0.5]
print(f"MODE={MODE}")
print(f"{'uid':>4} {'pos':>3} {'src':>4} {'start(x,y)':>16} {'success':>7} {'@step':>6}")
sbp = {0: [0, 0], 1: [0, 0], 2: [0, 0]}
for uid in UIDS:
    p = posof(uid)
    d = np.load(REPO / f'inthewild_trials/{uid}_episodes.npy', allow_pickle=True).item()
    vel = np.asarray(d['vel_cmd']); gpos = np.asarray(d['gripper_pos'])
    spos, src = start_xy(uid)
    kinova.set_dofs_position(np.array(start), kdofs)
    bottle.set_pos(spos); bottle.set_quat([1, 0, 0, 0])
    goal.set_pos(STATIC); goal.set_quat([1, 0, 0, 0])
    scene.step()
    success = False; succ_step = -1
    for i in range(len(vel)):
        kinova.control_dofs_position(vel[i], dofs_idx_local=kdofs[:6])
        motor = (100 - gpos[i][0]) / 100
        kinova.control_dofs_position(np.array([-motor, motor, -0.5, -0.5]), dofs_idx_local=kdofs[-4:])
        scene.step()
        if i % 30 == 0:
            nc = np_(bottle.get_contacts(with_entity=goal)['position'])
            nc = 0 if nc.size == 0 else nc.shape[0]
            if nc and float(np_(eef.get_pos())[0]) < float(np_(bottle.get_pos())[0]):
                success = True; succ_step = i; break
    sbp[p][1] += 1; sbp[p][0] += int(success)
    print(f"{uid:>4} {p:>3} {src:>4} ({spos[0]:+.3f},{spos[1]:+.3f}) {('YES' if success else 'no'):>7} {succ_step:>6}")
ts = sum(v[0] for v in sbp.values()); tn = sum(v[1] for v in sbp.values())
print(f"\n[{MODE}] " + "  ".join(f"P{p}={sbp[p][0]}/{sbp[p][1]}" for p in (0,1,2)) + f"  | TOTAL={ts}/{tn}")
