"""Prototype the DreamerV3 two-camera rig: TOP-DOWN candidates + eef-attached WRIST cam.

Renders each candidate camera across representative scene states (start / grasp /
carry / nest, reconstructed from recorded episodes_all states) and tiles contact
sheets for the user to pick from. No dataset regen here -- view selection only.

Usage: camera_rig_proto.py --out <dir>
"""
import argparse, glob, pathlib as pl, sys
import numpy as np, cv2

REPO = pl.Path('/home/j/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO / 'can_pos_recovery')); sys.path.insert(0, str(REPO / 'baselines'))
import torch

def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)

ap = argparse.ArgumentParser()
ap.add_argument('--out', default='baselines/camera_rig_samples')
ap.add_argument('--res', type=int, default=256)
args = ap.parse_args()
OUT = REPO / args.out; OUT.mkdir(parents=True, exist_ok=True)

from replay_harness import build_world, gripper_targets  # noqa: E402

# ---- candidate TOP cameras (full arm base + table + shelf + goal in frame) ----
TOPS = {
    'topA_overhead_hi':  dict(pos=(0.38, -0.08, 1.55), lookat=(0.38, -0.08, 0.10), fov=50, up=(1, 0, 0)),
    'topB_overhead_lo':  dict(pos=(0.40, -0.08, 1.15), lookat=(0.40, -0.08, 0.10), fov=68, up=(1, 0, 0)),
    'topC_behind_tilt':  dict(pos=(-0.42, 0.0, 1.25), lookat=(0.52, -0.10, 0.08), fov=52, up=(0, 0, 1)),
    'topD_side_tilt':    dict(pos=(0.40, 0.55, 1.15), lookat=(0.48, -0.12, 0.08), fov=55, up=(0, 0, 1)),
}
# ---- candidate WRIST offsets (camera frame: looks along -z, y ~ up) ----
def rotx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def offset_T(pos, rot):
    T = np.eye(4); T[:3, :3] = rot; T[:3, 3] = pos
    return T

WRISTS = {
    # v2: much more standoff so fingers + can + surface are in frame, gripper at edge
    'wrist4_high_back':   offset_T((0.0, -0.16, -0.10), rotx(np.deg2rad(150))),
    'wrist5_higher_back': offset_T((0.0, -0.22, -0.16), rotx(np.deg2rad(145))),
    'wrist6_side_high':   offset_T((0.14, -0.10, -0.10),
                                   rotx(np.deg2rad(150)) @
                                   np.array([[np.cos(0.6), -np.sin(0.6), 0],
                                             [np.sin(0.6), np.cos(0.6), 0], [0, 0, 1]])),
    'wrist7_fwd_tilt':    offset_T((0.0, -0.12, -0.20), rotx(np.deg2rad(130))),
}

w = build_world(backend='cpu', finger_force=50.0, finger_kp=40.0, can_height=0.101,
                can_rho=1000, substeps=8, table=True, can_radius=0.033, camera=True)
scene, kin, bottle, goal, kdofs, eef = (w['scene'], w['kinova'], w['bottle'],
                                        w['goal'], w['kdofs'], w['eef'])
cam = w['cam']
cam.set_params(fov=50)

# ---- representative scene states from the recollected dataset ----
def episode_states(uid):
    d = np.load(REPO / f'baselines/episodes_all/{uid}.npz', allow_pickle=True)
    return d['states']

STATES = []
for uid, tag in ((300, 'nested-demo'), (232, 'placed-demo')):
    s = episode_states(uid)
    n = len(s)
    for frac, name in ((0.0, 'start'), (0.28, 'grasp'), (0.6, 'carry'), (0.97, 'end')):
        STATES.append((f'{uid}_{name}', s[int(frac * (n - 1))]))

def set_state(sv):
    kin.set_dofs_position(np.array(sv[:6]), kdofs[:6])
    motor = float(np.clip(sv[6], 0, 1)) * 100.0
    kin.set_dofs_position(np.array(gripper_targets(motor)), np.array(kdofs[-4:]))
    kin.zero_all_dofs_velocity()
    bottle.set_pos(sv[8:11]); bottle.set_quat(list(sv[11:15]))
    goal.set_pos((sv[15], sv[16], w['goal_start_z'])); goal.set_quat([1, 0, 0, 0])
    scene.step()

def snap():
    return np.asarray(cam.render()[0])[:, :, ::-1].astype(np.uint8)

R = args.res
sheets = {}
for cam_name, pose in TOPS.items():
    tiles = []
    cam.set_pose(pos=pose['pos'], lookat=pose['lookat'], up=pose.get('up'))
    cam.set_params(fov=pose['fov'])
    for st_name, sv in STATES:
        set_state(sv)
        fr = cv2.resize(snap(), (R, R))
        cv2.putText(fr, st_name, (5, R - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
        cv2.putText(fr, st_name, (5, R - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        tiles.append(fr)
    sheet = np.hstack(tiles)
    cv2.putText(sheet, cam_name, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
    cv2.putText(sheet, cam_name, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    sheets[cam_name] = sheet
    cv2.imwrite(str(OUT / f'{cam_name}.png'), sheet)
cv2.imwrite(str(OUT / 'TOP_CANDIDATES.png'), np.vstack(list(sheets.values())))

wr_sheets = []
for wr_name, T in WRISTS.items():
    cam.attach(eef, T)
    tiles = []
    for st_name, sv in STATES:
        set_state(sv)
        cam.move_to_attach()
        fr = cv2.resize(snap(), (R, R))
        cv2.putText(fr, st_name, (5, R - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
        cv2.putText(fr, st_name, (5, R - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        tiles.append(fr)
    cam.detach()
    sheet = np.hstack(tiles)
    cv2.putText(sheet, wr_name, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
    cv2.putText(sheet, wr_name, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    wr_sheets.append(sheet)
    cv2.imwrite(str(OUT / f'{wr_name}.png'), sheet)
cv2.imwrite(str(OUT / 'WRIST_CANDIDATES.png'), np.vstack(wr_sheets))
print(f'sheets -> {OUT}')
