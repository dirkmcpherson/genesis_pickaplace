"""FK-at-grasp recovery over ALL episodes on disk (successful + failed labels).

Extends recover_can_position.py's v2 logic to every trial and saves a JSON with, per uid:
  - label (pos bucket + success/fail from kinova.py lists, dispatch order like example.py)
  - confidence (HIGH/MED/LOW), carry interval, grasp instant
  - recovered can start (fingertip-midpoint xy at grasp)
  - recovered release point (fingertip-midpoint xy/z at end of carry) -> seed for goal can

Usage: python can_pos_recovery/fk_all_trials.py
Writes: can_pos_recovery/fk_recovered.json
"""
import os
os.environ.setdefault('PYOPENGL_PLATFORM', 'egl')
import sys, json, pathlib as pl
import numpy as np
REPO = pl.Path('/home/j/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO))
import genesis as gs
import torch
from kinova import JOINT_NAMES, EEF_NAME
from kinova import (trials_position_0_successful as p0s, trials_position_0_failed as p0f,
                    trials_position_1_successful as p1s, trials_position_1_failed as p1f,
                    trials_position_2_successful as p2s, trials_position_2_failed as p2f)

def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)

BUCKET = {0: (0.4381, 0.1, 0.05), 1: (0.4381, -0.05, 0.05), 2: (0.4381, -0.2, 0.05)}
GOAL = (0.6, -0.2)
GP_CLOSE = 30.0

def label_of(uid):
    # position by example.py's dispatch order over the successful lists, else failed lists
    for p, lst in ((0, p0s), (1, p1s), (2, p2s)):
        if uid in lst: return p, 'success'
    for p, lst in ((0, p0f), (1, p1f), (2, p2f)):
        if uid in lst: return p, 'fail'
    return None, 'unlisted'

gs.init(backend=gs.cpu, seed=0, precision="32", logging_level="warning")
scene = gs.Scene(show_viewer=False)
kinova = scene.add_entity(gs.morphs.URDF(file=str(REPO / 'gen3_lite_2f_robotiq_85.urdf'),
                                         fixed=True, pos=(0.0, 0.0, 0.05)))
kdofs = [kinova.get_joint(n).dof_idx_local for n in JOINT_NAMES]
lfd = kinova.get_link('left_finger_dist_link'); rfd = kinova.get_link('right_finger_dist_link')
eef = kinova.get_link(EEF_NAME)
scene.build()

def fk_all(vel, gp):
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

uids = sorted(int(p.name.split('_')[0]) for p in (REPO / 'inthewild_trials').glob('*_episodes.npy'))
results = {}
for uid in uids:
    p, lab = label_of(uid)
    d = np.load(REPO / f'inthewild_trials/{uid}_episodes.npy', allow_pickle=True).item()
    vel = np.asarray(d['vel_cmd']); gp = np.asarray(d['gripper_pos'])[:, 0]
    ez, mid = fk_all(vel, gp)
    closed = gp > GP_CLOSE
    ivs = intervals(closed)
    conf = 'LOW'; grasp = None; carry = None
    if ivs:
        def lift(iv): a, b = iv; return float(ez[a:b+1, 2].max() - ez[a:b+1, 2].min())
        def rel_goal(iv): a, b = iv; return float(np.hypot(ez[b,0]-GOAL[0], ez[b,1]-GOAL[1]))
        cands = [iv for iv in ivs if lift(iv) > 0.05 and rel_goal(iv) < 0.25]
        if cands:
            carry = max(cands, key=lift); conf = 'HIGH'
        else:
            carry = max(ivs, key=lift); conf = 'MED' if lift(carry) > 0.05 else 'LOW'
        a, b = carry
        seg = slice(a, a + max(1, int(0.6*(b-a)) + 1))
        grasp = a + int(np.argmin(ez[seg, 2]))
    if grasp is None:
        grasp = int(np.argmin(ez[:, 2]))
    entry = {
        'pos': p, 'label': lab, 'conf': conf, 'n_steps': int(len(vel)),
        'grasp_idx': int(grasp),
        'can_xy': [round(float(mid[grasp, 0]), 4), round(float(mid[grasp, 1]), 4)],
        'grasp_mid_z': round(float(mid[grasp, 2]), 4),
        'carry': [int(carry[0]), int(carry[1])] if carry else None,
        'release_xy': [round(float(mid[carry[1], 0]), 4), round(float(mid[carry[1], 1]), 4)] if carry else None,
        'release_z': round(float(mid[carry[1], 2]), 4) if carry else None,
        'bucket': list(BUCKET[p]) if p is not None else None,
    }
    results[uid] = entry
    print(f"{uid} pos={p} {lab:8s} conf={conf:4s} grasp@{grasp:4d} "
          f"can=({entry['can_xy'][0]:+.3f},{entry['can_xy'][1]:+.3f}) "
          f"rel={entry['release_xy']} carry={entry['carry']}")

out = REPO / 'can_pos_recovery/fk_recovered.json'
out.write_text(json.dumps(results, indent=1))
print(f"\nwrote {out} ({len(results)} trials)")
