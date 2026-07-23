"""Find the through-the-gripper wrist view: sweep camera orientations along all six
eef-frame axes (camera slightly retracted the opposite way), rendered at a GRASP
state where the can sits between the fingers. The right axis shows can + fingertips.
"""
import os
import pathlib as pl, sys
import numpy as np, cv2

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', '/home/j/workspace/genesis_pickaplace'))
sys.path.insert(0, str(REPO / 'can_pos_recovery')); sys.path.insert(0, str(REPO / 'baselines'))
from replay_harness import build_world, gripper_targets

OUT = REPO / 'baselines/camera_rig_samples'; OUT.mkdir(exist_ok=True)
w = build_world(backend='cpu', finger_force=50.0, finger_kp=40.0, can_height=0.101,
                can_rho=1000, substeps=8, table=True, can_radius=0.033, camera=True)
scene, kin, bottle, goal, kdofs, eef = (w['scene'], w['kinova'], w['bottle'],
                                        w['goal'], w['kdofs'], w['eef'])
cam = w['cam']; cam.set_params(fov=80)   # wide: fingertips + target

d = np.load(REPO / 'baselines/episodes_all/300.npz', allow_pickle=True)
s = d['states']
GRASP = s[int(0.28 * (len(s) - 1))]     # can between fingers
CARRY = s[int(0.6 * (len(s) - 1))]

def set_state(sv):
    kin.set_dofs_position(np.array(sv[:6]), kdofs[:6])
    kin.set_dofs_position(np.array(gripper_targets(float(np.clip(sv[6], 0, 1)) * 100.0)),
                          np.array(kdofs[-4:]))
    kin.zero_all_dofs_velocity()
    bottle.set_pos(sv[8:11]); bottle.set_quat(list(sv[11:15]))
    goal.set_pos((sv[15], sv[16], w['goal_start_z'])); goal.set_quat([1, 0, 0, 0])
    scene.step()

def look_along(axis):
    """offset_T whose camera -z (view dir) is eef-frame `axis`, retracted 6cm behind."""
    z_cam = -np.asarray(axis, dtype=float)          # cam -z = view dir = axis
    up_guess = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(up_guess, z_cam)) > 0.9:
        up_guess = np.array([0.0, 1.0, 0.0])
    x_cam = np.cross(up_guess, z_cam); x_cam /= np.linalg.norm(x_cam)
    y_cam = np.cross(z_cam, x_cam)
    R = np.stack([x_cam, y_cam, z_cam], axis=1)
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = -np.asarray(axis, float) * 0.06
    return T

AXES = {'top_p20': (20, 0.10), 'top_p30': (30, 0.10), 'top_p40': (40, 0.10), 'top_p30_o13': (30, 0.13)}
R_ = 256
rows = []
for st_name, sv in (('grasp', GRASP), ('carry', CARRY)):
    tiles = []
    for name, (pitch, off) in AXES.items():
        T = look_along((0, 0, 1))
        a = np.deg2rad(pitch); c, si = np.cos(a), np.sin(a)
        pitch_R = np.array([[c, 0, -si], [0, 1, 0], [si, 0, c]])   # tilt DOWN from above
        r = np.deg2rad(270); rc, rs = np.cos(r), np.sin(r)
        roll_R = np.array([[rc, -rs, 0], [rs, rc, 0], [0, 0, 1]])  # -90 for human viewing
        T[:3, :3] = T[:3, :3] @ pitch_R @ roll_R
        T[:3, 3] = np.array([+off, 0.0, -0.03])                    # +x = world-UP (verified)
        cam.attach(eef, T)
        set_state(sv)
        cam.move_to_attach()
        fr = cv2.resize(np.asarray(cam.render()[0])[:, :, ::-1].astype(np.uint8), (R_, R_))
        cam.detach()
        lab = f'{name}_{st_name}'
        cv2.putText(fr, lab, (5, R_ - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(fr, lab, (5, R_ - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        tiles.append(fr)
    rows.append(np.hstack(tiles))
cv2.imwrite(str(OUT / 'WRIST_AXIS_SWEEP.png'), np.vstack(rows))
print('sweep sheet written')
