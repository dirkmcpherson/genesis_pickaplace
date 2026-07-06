"""Diagnose the mid-carry drop: replay one trial, log bottle z / finger driver angle vs
target / bottle<->finger contact count every 10 cmds around the carry.

Usage: python can_pos_recovery/diag_carry.py <uid> <canx> <cany> [finger_kp] [finger_force]
"""
import sys
import numpy as np
import torch
from replay_harness import (build_world, load_episode, gripper_targets,
                            STATIC_BOTTLE_POSITION, HARDCODED_START)

def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)

uid = int(sys.argv[1]); canx = float(sys.argv[2]); cany = float(sys.argv[3])
fkp = float(sys.argv[4]) if len(sys.argv) > 4 else 100.0
ff = float(sys.argv[5]) if len(sys.argv) > 5 else 50.0

w = build_world(backend='cpu', finger_force=ff, finger_kp=fkp)
scene, kinova, bottle, goal, kdofs, eef = (w['scene'], w['kinova'], w['bottle'],
                                           w['goal'], w['kdofs'], w['eef'])
vel, gp = load_episode(uid)

kinova.set_dofs_position(np.array(HARDCODED_START), kdofs)
kinova.zero_all_dofs_velocity()
bottle.set_pos((canx, cany, 0.05)); bottle.set_quat([1, 0, 0, 0])
goal.set_pos(STATIC_BOTTLE_POSITION); goal.set_quat([1, 0, 0, 0])
scene.step()

print(f"{'i':>4} {'gp':>5} {'drv_tgt':>7} {'drv_act':>7} {'tip_act':>7} {'ncon':>4} "
      f"{'can_z':>6} {'can_xy':>15} {'eef_z':>6}")
was_up = False
for i in range(len(vel)):
    kinova.control_dofs_position(vel[i], dofs_idx_local=kdofs[:6])
    tgt = gripper_targets(gp[i])
    kinova.control_dofs_position(np.array(tgt), dofs_idx_local=np.array(kdofs[-4:]))
    for _ in range(3):
        scene.step()
    bp = np_(bottle.get_pos())
    if i % 10 == 0 or (was_up and bp[2] < 0.08):
        q = np_(kinova.get_dofs_position(dofs_idx_local=kdofs))
        c = bottle.get_contacts(kinova)['position']
        nc = 0 if np_(c).size == 0 else np_(c).shape[0]
        print(f"{i:>4} {gp[i]:>5.0f} {tgt[1]:>7.3f} {q[7]:>7.3f} {q[9]:>7.3f} {nc:>4} "
              f"{bp[2]:>6.3f} ({bp[0]:+.3f},{bp[1]:+.3f}) {np_(eef.get_pos())[2]:>6.3f}")
    if bp[2] > 0.10: was_up = True
    if was_up and bp[2] < 0.06:
        print(f"DROPPED at cmd {i}: can=({bp[0]:+.3f},{bp[1]:+.3f},{bp[2]:.3f})")
        break
