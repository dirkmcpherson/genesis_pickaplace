"""Add 'close_xy' to fk_recovered.json: fingertip-midpoint at the START of the carry
closure. Bags show humans often close on the can and then DRAG it several cm along the
table before lifting; FK-at-min-z lands mid-drag, but the can must START where the
fingers first close for open-loop replay to reproduce the drag.
"""
import os
os.environ.setdefault('PYOPENGL_PLATFORM', 'egl')
import sys, json, pathlib as pl
import numpy as np
REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', '/home/j/workspace/genesis_pickaplace'))
sys.path.insert(0, str(REPO))
import genesis as gs
import torch
from kinova import JOINT_NAMES

def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)

fkj = REPO / 'can_pos_recovery/fk_recovered.json'
data = {int(k): v for k, v in json.loads(fkj.read_text()).items()}

gs.init(backend=gs.cpu, seed=0, precision="32", logging_level="warning")
scene = gs.Scene(show_viewer=False)
kinova = scene.add_entity(gs.morphs.URDF(file=str(REPO / 'gen3_lite_2f_robotiq_85.urdf'),
                                         fixed=True, pos=(0.0, 0.0, 0.05)))
kdofs = [kinova.get_joint(n).dof_idx_local for n in JOINT_NAMES]
lfd = kinova.get_link('left_finger_dist_link'); rfd = kinova.get_link('right_finger_dist_link')
scene.build()

for uid, v in sorted(data.items()):
    if not v.get('carry'):
        v['close_xy'] = None
        continue
    a = v['carry'][0]
    d = np.load(REPO / f'inthewild_trials/{uid}_episodes.npy', allow_pickle=True).item()
    vel = np.asarray(d['vel_cmd']); gp = np.asarray(d['gripper_pos'])[:, 0]
    motor = (100 - gp[a]) / 100
    kinova.set_dofs_position(np.concatenate([vel[a], [-motor, motor, -0.5, -0.5]]), kdofs)
    mid = (np_(lfd.get_pos()) + np_(rfd.get_pos())) / 2
    v['close_xy'] = [round(float(mid[0]), 4), round(float(mid[1]), 4)]
    drag = float(np.hypot(mid[0] - v['can_xy'][0], mid[1] - v['can_xy'][1]))
    if drag > 0.02:
        print(f"{uid}: close_xy={v['close_xy']} vs grasp_xy={v['can_xy']}  drag={drag*100:.1f}cm")

fkj.write_text(json.dumps({str(k): v for k, v in data.items()}, indent=1))
print("updated", fkj)
