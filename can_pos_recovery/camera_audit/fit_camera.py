import glob, json, os
import numpy as np
import cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

D = os.path.dirname(os.path.abspath(__file__))
CONV = 'zyx'
PLC = json.load(open('/home/james/workspace/genesis_pickaplace/can_pos_recovery/trial_placements.json'))['trials']
ANCH = json.load(open(f'{D}/anchors.json'))
Z_CAN_TOP_BASE = 0.113 + 0.0505 - 0.05
GOAL_XY = (0.672, -0.221)

def load_tape():
    X, Th, B = [], [], []
    for f in sorted(glob.glob(f'{D}/pairs3_*.npz')):
        a = np.load(f)['pairs']
        X.append(a[:,1:4]); Th.append(a[:,4:7]); B.append(a[:,7:16])
    return np.concatenate(X), np.concatenate(Th), np.concatenate(B).reshape(-1,3,3)

X, Th, B = load_tape()
Rt = Rotation.from_euler(CONV, Th, degrees=True)
can_anch = [(np.array([PLC[u]['can_pos'][0], PLC[u]['can_pos'][1], Z_CAN_TOP_BASE]),
             np.array(ANCH[u]['can_px']), u)
            for u in ANCH if 'can_px' in ANCH[u]]
goal_px = [np.array(ANCH[u]['goal_px']) for u in ANCH if 'goal_px' in ANCH[u]]
print('tape frames', len(X), 'can anchors', len(can_anch), 'goal rays', len(goal_px))

def cam_project(camp, Xw):
    fx, fy, cx, cy, k1 = camp[:5]
    Xc = Rotation.from_rotvec(camp[5:8]).apply(np.atleast_2d(Xw)) + camp[8:11]
    x, y = Xc[:,0]/Xc[:,2], Xc[:,1]/Xc[:,2]
    r2 = x*x+y*y
    return np.stack([fx*x*(1+k1*r2)+cx, fy*y*(1+k1*r2)+cy], 1)

M = 2
def unpack(p):
    return p[:11], p[11:11+3*M].reshape(M,3), p[11+3*M]

def resid(p, w_anchor=8.0):
    camp, O, zg = unpack(p)
    # tape: best (offset, blob) per frame, capped contribution
    out = []
    Pm = [cam_project(camp, X + Rt.apply(O[m])) for m in range(M)]
    best = np.full(len(X), 60.0)   # cap: frames with no candidate under 60px saturate
    for m in range(M):
        for j in range(3):
            uj = B[:,j,:2]
            valid = ~np.isnan(uj[:,0])
            d = np.where(valid, np.linalg.norm(Pm[m]-uj, axis=1), 1e9)
            best = np.minimum(best, d)
    out.append(best)
    for Xa, ua, _ in can_anch:
        out.append(w_anchor*(cam_project(camp, Xa[None])[0] - ua))
    Xg = np.array([GOAL_XY[0], GOAL_XY[1], zg])
    for ug in goal_px:
        out.append(w_anchor*(cam_project(camp, Xg[None])[0] - ug))
    return np.concatenate([np.atleast_1d(o).ravel() for o in out])

p0 = np.zeros(11+3*M+1)
p0[:5] = [900, 900, 640, 360, 0]
m_old = np.load(f'{D}/cam4_model_em.npz')['camp']
p0[5:8], p0[8:11] = m_old[6:9], m_old[9:12]
p0[11:11+3*M] = [0,0,-0.04, 0,0,0.04]
p0[-1] = 0.20
lo = [500,500,500,260,-0.25] + [-10]*3 + [-3]*3 + [-0.08]*(3*M) + [0.13]
hi = [1600,1600,780,460,0.25] + [10]*3 + [3]*3 + [0.08]*(3*M) + [0.28]
sol = least_squares(resid, p0, loss='soft_l1', f_scale=5.0, bounds=(lo,hi), max_nfev=20000)
camp, O, zg = unpack(sol.x)
print(f'cam fx {camp[0]:.0f} fy {camp[1]:.0f} cx {camp[2]:.0f} cy {camp[3]:.0f} '
      f'k1 {camp[4]:.3f} | z_goal(base) {zg:.3f} |o| {[round(float(np.linalg.norm(o))*100,1) for o in O]}cm')
# anchor residuals
print('\ncan-top anchors:')
for Xa, ua, u in can_anch:
    d = cam_project(camp, Xa[None])[0] - ua
    print(f'  {u}: dpx ({d[0]:+.0f},{d[1]:+.0f}) |{np.linalg.norm(d):.0f}px|')
Xg = np.array([GOAL_XY[0], GOAL_XY[1], zg])
dg = [np.linalg.norm(cam_project(camp, Xg[None])[0]-ug) for ug in goal_px]
print(f'goal: med {np.median(dg):.0f}px max {np.max(dg):.0f}px')
# tape residual health
Pm = [cam_project(camp, X + Rt.apply(O[m])) for m in range(M)]
best = np.full(len(X), 1e9)
for m in range(M):
    for j in range(3):
        uj = B[:,j,:2]; valid = ~np.isnan(uj[:,0])
        d = np.where(valid, np.linalg.norm(Pm[m]-uj, axis=1), 1e9)
        best = np.minimum(best, d)
print(f'tape: med {np.median(best):.0f}px p80 {np.percentile(best,80):.0f}px '
      f'frac<20px {np.mean(best<20):.2f}')
np.savez(f'{D}/cam4_model_final.npz', camp=camp, O=O, zg=zg, conv=CONV)
