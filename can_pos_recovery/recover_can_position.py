"""v2: back out the can position robustly against human-demo noise (gripper test-pulses,
missed grabs). The real pickup is the closure that CARRIES the can: reach down -> close ->
lift -> transport to the goal -> release. We pick the closed interval with the largest
horizontal EEF transport, then take the reach-bottom (min EEF z) inside it as the grasp
instant. Falls back + flags low-confidence when the gripper never really closes.
"""
import os
os.environ.setdefault('PYOPENGL_PLATFORM', 'egl')
import sys, pathlib as pl
import numpy as np
REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', '/home/j/workspace/genesis_pickaplace'))
sys.path.insert(0, str(REPO))
import genesis as gs
import torch
from kinova import (JOINT_NAMES, EEF_NAME,
                    TRIALS_POSITION_0 as P0, TRIALS_POSITION_1 as P1, TRIALS_POSITION_2 as P2)

def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)

UIDS = [235, 232, 257, 242,  322, 243, 236, 280,  290, 278, 237, 234]
BUCKET = {0: (0.4381, 0.1, 0.05), 1: (0.4381, -0.05, 0.05), 2: (0.4381, -0.2, 0.05)}
GOAL = (0.6, -0.2)
def posof(u): return 0 if u in P0 else 1 if u in P1 else 2 if u in P2 else None
GP_CLOSE = 30.0   # absolute gripper-closed threshold (motor 0..100); rejects "never closed"

gs.init(backend=gs.gpu, seed=0, precision="32", logging_level="warning")
scene = gs.Scene(show_viewer=False)
kinova = scene.add_entity(gs.morphs.URDF(file=str(REPO / 'gen3_lite_2f_robotiq_85.urdf'),
                                         fixed=True, pos=(0.0, 0.0, 0.05)))
kdofs = [kinova.get_joint(n).dof_idx_local for n in JOINT_NAMES]
lfd = kinova.get_link('left_finger_dist_link'); rfd = kinova.get_link('right_finger_dist_link')
eef = kinova.get_link(EEF_NAME)
scene.build()

def fk_all(vel, gp):
    """forward-kinematics over the whole trajectory: returns eef xyz + fingertip-midpoint xyz per step."""
    n = len(vel); ez = np.zeros((n, 3)); mid = np.zeros((n, 3))
    for i in range(n):
        motor = (100 - gp[i]) / 100
        kinova.set_dofs_position(np.concatenate([vel[i], [-motor, motor, -0.5, -0.5]]), kdofs)
        ez[i] = np_(eef.get_pos()); mid[i] = (np_(lfd.get_pos()) + np_(rfd.get_pos())) / 2
    return ez, mid

def intervals(mask, minlen=15):
    out = []; i = 0; n = len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]: j += 1
            if j - i >= minlen: out.append((i, j-1))
            i = j
        else: i += 1
    return out

print(f"{'uid':>4} {'pos':>3} | {'recovered (x, y)':>17} {'z':>6} | {'bucket xy':>14} {'err':>6} | "
      f"{'conf':>5} {'grasp@':>6} | {'release xy (->goal?)':>20}")
rec = {0: [], 1: [], 2: []}
for uid in UIDS:
    p = posof(uid)
    d = np.load(REPO / f'inthewild_trials/{uid}_episodes.npy', allow_pickle=True).item()
    vel = np.asarray(d['vel_cmd']); gp = np.asarray(d['gripper_pos'])[:, 0]
    ez, mid = fk_all(vel, gp)
    closed = gp > GP_CLOSE
    ivs = intervals(closed)
    conf = 'LOW'; grasp = None; rel = None
    if ivs:
        def lift(iv): a, b = iv; return float(ez[a:b+1, 2].max() - ez[a:b+1, 2].min())
        def rel_goal(iv): a, b = iv; return float(np.hypot(ez[b,0]-GOAL[0], ez[b,1]-GOAL[1]))
        # the carry is the closure that LIFTS and RELEASES near the known goal (anchor on the end state)
        cands = [iv for iv in ivs if lift(iv) > 0.05 and rel_goal(iv) < 0.25]
        if cands:
            carry = max(cands, key=lift); conf = 'HIGH'
        else:
            carry = max(ivs, key=lift); conf = 'MED' if lift(carry) > 0.05 else 'LOW'
        a, b = carry
        seg = slice(a, a + max(1, int(0.6*(b-a)) + 1))   # reach-bottom in first 60% of the carry
        grasp = a + int(np.argmin(ez[seg, 2]))
        rel = ez[b, :2]
    if grasp is None:
        grasp = int(np.argmin(ez[:, 2]))   # fallback: global reach-bottom
    cx, cy, cz = mid[grasp]
    bx, by, _ = BUCKET[p]; err = float(np.hypot(cx-bx, cy-by))
    if conf in ('HIGH', 'MED'): rec[p].append((cx, cy))
    relstr = f"({rel[0]:+.2f},{rel[1]:+.2f})" if rel is not None else "  --  "
    print(f"{uid:>4} {p:>3} | ({cx:+.3f}, {cy:+.3f}) {cz:+.3f} | ({bx:+.3f},{by:+.3f}) {err:>6.3f} | "
          f"{conf:>5} {grasp:>6} | {relstr:>20}")

print("\n--- per-bucket spread of HIGH/MED-confidence recoveries ---")
for p in (0, 1, 2):
    arr = np.array(rec[p])
    if len(arr):
        print(f"P{p}: bucket=({BUCKET[p][0]:+.3f},{BUCKET[p][1]:+.3f})  "
              f"recovered mean=({arr[:,0].mean():+.3f},{arr[:,1].mean():+.3f})  n={len(arr)}  "
              f"x[{arr[:,0].min():+.3f},{arr[:,0].max():+.3f}] y[{arr[:,1].min():+.3f},{arr[:,1].max():+.3f}]")
